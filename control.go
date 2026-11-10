//go:build linux

package main

// ---------- control loop: sense -> classify -> pick -> control -> act ----------
//
// Errors are data, not branches. Every input is a value plus when it was last valid: a failed or
// implausible read changes nothing, the input just gets older.
//
//	sense     stores each reading and when it last read valid
//	classify  each input -> ok / unknown / danger (unknown: never seen, or unknownAfter past due)
//	pick      (mode, worst class) -> one controller: shutdown, idrac, safe, fixed or model
//	control   the picked controller turns the readings into what the hardware should do
//	act       writes what differs from what each actuator last confirmed; an actuator that won't take
//	          its value is an input like any other and goes unknown the same way
//
// A panic or an impossible value in classify, pick or a controller -> the safe controller.

import (
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"runtime/debug"
	"slices"
	"sort"
	"strconv"
	"strings"
	"time"
)

const modelCeiling = 50 // the model never sends more (fixed sends whatever it is given)

// journald priority prefixes (SyslogLevelPrefix): what a log watcher filters on
const (
	prErr  = "<3>"
	prWarn = "<4>"
)

// bmcAPI: what the control loop needs from the BMC (a fake one drives the tests).
type bmcAPI interface {
	Sensors() ([]Sensor, error)
	Reading(Sensor) (float64, bool, string, error)
	FanDuty(pct int) error
	FanAuto() error
	SetPowerCap(watts int) error
	GetPowerLimit() (PowerLimit, error)
}

// input: a reading and when it was last valid (zero = never: counts from the start of this run).
type input struct {
	V    float64
	Seen time.Time
}

// inputs: everything decide looks at. The key sets are the expected sensors, fixed at the first good
// read (or carried over from the last run), so a sensor that stops reporting can't drop out.
type inputs struct {
	Temps  map[string]input // BMC temperatures
	Fans   map[string]input // BMC fan RPM
	HDD    map[string]input // drive temperatures, by serial
	Drives time.Time        // last drive scan that found >= inputs.hdd_min drives
	Power  input            // wall watts; optional (the model falls back to its curves without it)
	Cap    input            // power cap read back from the iDRAC, W (an inactive cap reads as power.cap_max)
	Clamp  input            // CPU clamp read back from intel_pstate: 1 = clamped
}

func (in *inputs) init() {
	if in.Temps == nil {
		in.Temps = map[string]input{}
	}
	if in.Fans == nil {
		in.Fans = map[string]input{}
	}
	if in.HDD == nil {
		in.HDD = map[string]input{}
	}
}

// actuator: what was last written successfully. Whether it took is checked by reading it back (the
// inputs Fans, Cap, Clamp), never assumed from a write's return code.
type actuator struct {
	have string    // last value written without an error; "" = none (never, or the last write failed)
	at   time.Time // when
}

func (a *actuator) result(want string, err error, now time.Time) {
	if err == nil {
		a.have, a.at = want, now
		return
	}
	a.have = ""
}

// desired: what one cycle wants the hardware to do.
type desired struct {
	fan   string  // "auto" (the iDRAC's control) or the duty percent
	duty  float64 // the duty before rounding and the ceiling, for logs and the down-ramp
	cap   int     // iDRAC power cap, W
	clamp bool    // CPUs at their lowest frequency, no turbo
	why   string
	state string // the controller that produced it
	off   bool   // shut the server down
}

// dangerRun: consecutive new readings at/above the shutdown line (danger is confirmed at dangerReads).
type dangerRun struct {
	n    int
	seen time.Time // the last reading counted
}

const (
	dangerReads = 3                // consecutive readings at/above a shutdown line before the poweroff
	driveSlack  = time.Minute      // a drive pass may run this much longer than usual
	safeRPMTol  = 0.25             // safe: fan RPM within this fraction of what safe.duty should give
	kickEvery   = 30 * time.Second // missing drive temperatures: re-read at most this often
)

// poweroff: the last resort. A clean OS shutdown, so it works without the BMC (a var: tests replace it).
var poweroff = func() error { return exec.Command("systemctl", "poweroff").Run() }

type controller struct {
	bmc     bmcAPI
	now     func() time.Time
	start   time.Time
	sensors []Sensor
	kick    chan struct{} // ask the drive goroutine for an early re-read
	reload  chan func()   // validated settings to apply before the next cycle

	in   inputs
	held []string // inputs bridged with their last value this cycle

	fan, pcap, clamp actuator
	duty             float64              // last duty written (0 = none, or the iDRAC's): the model's down-ramp starts here
	last             desired              // last cycle's decision
	safeOK           time.Time            // safe: last cycle its readbacks matched (or it was entered)
	danger           map[string]dangerRun // readings at/above their shutdown line, confirming
	lastKick         time.Time            // last early drive re-read asked for
	shutdownTried    bool                 // the shutdown's best-effort writes were tried once
	off              *desired             // the shutdown decided: it stays (latched)
	bmcDown          bool                 // the BMC didn't answer this cycle

	// normal control
	modelI, capCorr float64
	modelCap        float64   // the model's cap request after hysteresis; 0 = none yet
	hotHist         []float64 // hottest CPU, last 60 s
	raplOff         float64   // learned wall - RAPL offset (W); valid when raplLearned
	raplLearned     bool
	hddSnap         time.Time // drive snapshot last looked at (missing drives are logged once per snapshot)
	saveErr         string
	poweredOff      bool // poweroff accepted: nothing more to do
}

func newController(bmc bmcAPI) *controller {
	c := &controller{bmc: bmc, now: time.Now, kick: make(chan struct{}, 1), reload: make(chan func(), 1), danger: map[string]dangerRun{}}
	c.start = c.now()
	c.in.init()
	return c
}

// overdue: how far past its normal refresh an input is.
func (c *controller) overdue(seen time.Time, period time.Duration, now time.Time) time.Duration {
	if seen.IsZero() {
		seen = c.start
	}
	return now.Sub(seen) - period
}

// ---------- sense ----------

func sensorKey(s Sensor) string {
	switch s.Type {
	case sensorTemp:
		if s.Entity == 3 { // entity 3.x = processor x
			return fmt.Sprintf("CPU%d", s.Inst)
		}
		return strings.ReplaceAll(strings.ReplaceAll(s.Name, " Temp", ""), " ", "_")
	case sensorFan:
		return strings.ReplaceAll(s.Name, " RPM", "")
	}
	return "power"
}

func (c *controller) sense(now time.Time) {
	temps, fans, why := c.senseBMC(now)
	cpus, cwhy := senseCoretemp()
	for k, v := range cpus {
		temps[k] = v
	}
	for k, w := range cwhy {
		why[k] = w
	}
	if len(c.in.Temps) == 0 { // the first good read decides which sensors are expected from now on
		if hottestCPU(temps) == 0 || len(fans) == 0 {
			c.logInputs(now, why)
			return
		}
		for k := range temps {
			c.in.Temps[k] = input{}
		}
		for _, k := range requiredTemps { // never seen: unknown
			c.in.Temps[k] = input{}
		}
		for k := range fans {
			c.in.Fans[k] = input{}
		}
		log.Printf("temperature sensors: %s; fans: %d", fmtTemps(temps), len(fans))
	}
	// required_temps follow the config (also after a reload, or state carried over from a run with
	// other settings): a required one is expected even unseen. Only a key never seen can go, so a
	// sensor fanctl discovered stays expected whatever required_temps says.
	for _, k := range requiredTemps {
		if _, ok := c.in.Temps[k]; !ok {
			c.in.Temps[k] = input{}
		}
	}
	for k := range c.in.Temps {
		if v, ok := temps[k]; ok {
			c.in.Temps[k] = input{v, now}
		}
	}
	for k, in := range c.in.Temps {
		if in.Seen.IsZero() && !slices.Contains(requiredTemps, k) {
			delete(c.in.Temps, k)
		}
	}
	for k := range c.in.Fans {
		if v, ok := fans[k]; ok {
			c.in.Fans[k] = input{v, now}
		}
	}
	// the cap is read back every inputs.reassert (a write verifies it too, see act); a hung BMC
	// costs one timeout per cycle, not two
	if !c.bmcDown && now.Sub(c.in.Cap.Seen) >= reassert {
		if pl, err := c.bmc.GetPowerLimit(); err == nil {
			w := float64(pl.Watts)
			if !pl.Active {
				w = pcapMax
			}
			c.in.Cap = input{w, now}
		} else {
			why["power cap"] = err.Error()
		}
	}
	if clamped, err := clampState(); err == nil { // an input only when safe uses the clamp (readings)
		c.in.Clamp = input{map[bool]float64{false: 0, true: 1}[clamped], now}
	} else {
		why["CPU clamp"] = err.Error()
	}
	c.senseDrives(now)
	c.logInputs(now, why)
}

// senseBMC reads the BMC's temperatures (all but the CPUs: those come from coretemp), fans and wall
// power; why says what the BMC answered for each reading it didn't give.
func (c *controller) senseBMC(now time.Time) (temps, fans map[string]float64, why map[string]string) {
	temps, fans, why = map[string]float64{}, map[string]float64{}, map[string]string{}
	c.bmcDown = false
	if c.sensors == nil {
		s, err := c.bmc.Sensors()
		if err != nil {
			why["*"] = "sensor repository: " + err.Error()
			c.bmcDown = true
			return
		}
		c.sensors = s
		log.Printf("sensor repository: %d temperature/fan/power sensors", len(s))
	}
	var fail error
	for _, s := range c.sensors {
		if s.Type == sensorTemp && s.Entity == 3 { // a CPU: read from coretemp instead
			continue
		}
		k := sensorKey(s)
		if fail != nil { // the BMC stopped answering: don't wait out a timeout per remaining sensor
			why[k] = "not read: " + fail.Error()
			continue
		}
		v, ok, w, err := c.bmc.Reading(s)
		switch {
		case err != nil:
			fail, why[k], c.bmcDown = err, err.Error(), true
		case !ok:
			why[k] = w
		case s.Type == sensorTemp && !(v > 0 && v < 127):
			why[k] = fmt.Sprintf("implausible %g C", v)
		case s.Type == sensorTemp:
			temps[k] = v
		case s.Type == sensorFan:
			fans[k] = v
		case s.Type == sensorCurrent:
			c.in.Power = input{v, now}
		}
	}
	if fail == nil && len(fans) == 0 {
		c.sensors = nil // walk the repository again next time
		why["*"] = fmt.Sprintf("empty sensor read (%d temperatures, %d fans)", len(temps), len(fans))
	}
	return
}

// hwmonDir: where the kernel lists its hardware sensors.
var hwmonDir = "/sys/class/hwmon"

// senseCoretemp reads each CPU package's own temperature sensor through the kernel (coretemp): the
// sensors the iDRAC reads over PECI, without the iDRAC in the path (it reads ~1 C lower than the
// iDRAC, 60 h of data). "Package id N" is CPU N+1. Found by name and label every time: the hwmonN
// numbering changes across boots.
func senseCoretemp() (cpus map[string]float64, why map[string]string) {
	cpus, why = map[string]float64{}, map[string]string{}
	dirs, _ := filepath.Glob(hwmonDir + "/hwmon*")
	for _, d := range dirs {
		if n, _ := readTrim(d + "/name"); n != "coretemp" {
			continue
		}
		labels, _ := filepath.Glob(d + "/temp*_label")
		for _, l := range labels {
			var pkg int
			if lab, _ := readTrim(l); !strings.HasPrefix(lab, "Package id ") {
				continue
			} else if _, err := fmt.Sscanf(lab, "Package id %d", &pkg); err != nil {
				continue
			}
			k := fmt.Sprintf("CPU%d", pkg+1)
			raw, err := readTrim(strings.TrimSuffix(l, "_label") + "_input")
			mc, perr := strconv.Atoi(raw)
			switch {
			case err != nil:
				why[k] = "coretemp: " + err.Error()
			case perr != nil:
				why[k] = fmt.Sprintf("coretemp: unparsable %q", raw)
			case !(mc > 0 && mc < 127000):
				why[k] = fmt.Sprintf("coretemp: implausible %g C", float64(mc)/1000)
			default:
				cpus[k] = float64(mc) / 1000
			}
		}
	}
	return
}

// senseDrives takes the drive goroutine's latest snapshot. The scan decides which drives are
// expected (a removal is only accepted once a second scan agrees, see hddLoop).
func (c *controller) senseDrives(now time.Time) {
	hdds.Lock()
	ids, raw, at, scanned := hdds.ids, hdds.temps, hdds.at, hdds.scanned
	hdds.Unlock()
	temps := map[Drive]float64{} // a copy: the snapshot is shared with hddLoop, and /metrics reads st.hdd
	for d, t := range raw {
		if t > 0 && t < 100 { // a garbled SMART/LOG SENSE byte is no reading
			temps[d] = t
		} else if at != c.hddSnap {
			log.Printf("%sdrive %s: implausible temperature %g C", prWarn, d.Serial, t)
		}
	}
	st.Lock()
	st.hdd = temps
	st.Unlock()
	if !scanned { // nothing known yet this run: the drive inputs keep their carried-over ages
		return
	}
	if len(ids) >= hddMin {
		c.in.Drives = at
	}
	keep, missing := map[string]bool{}, []string{}
	for _, d := range ids {
		keep[d.Serial] = true
		if t, ok := temps[d]; ok {
			if at.After(c.in.HDD[d.Serial].Seen) {
				c.in.HDD[d.Serial] = input{t, at}
			}
		} else {
			missing = append(missing, d.Serial)
			if _, known := c.in.HDD[d.Serial]; !known {
				c.in.HDD[d.Serial] = input{}
			}
		}
	}
	for s := range c.in.HDD {
		if !keep[s] {
			delete(c.in.HDD, s)
		}
	}
	if len(missing) > 0 {
		if now.Sub(c.lastKick) >= kickEvery {
			c.kickDrives(now) // re-read (and, for a pulled drive, rescan) now instead of in inputs.hdd_interval
		}
		if at != c.hddSnap {
			log.Printf("%sdrive temperature missing: %v", prWarn, missing)
		}
	}
	if len(ids) < hddMin && at != c.hddSnap {
		log.Printf("%sonly %d drives found (inputs.hdd_min %d)", prWarn, len(ids), hddMin)
	}
	c.hddSnap = at
}

func (c *controller) kickDrives(now time.Time) {
	select {
	case c.kick <- struct{}{}:
	default:
	}
	c.lastKick = now
}

// logInputs logs every expected temperature and fan that didn't read valid this cycle, with the
// source's answer: a warning while its last value still stands in, an error once it is unknown.
func (c *controller) logInputs(now time.Time, why map[string]string) {
	c.held = c.held[:0]
	if w := why["*"]; w != "" {
		log.Printf("%sBMC: %s", prWarn, w)
	}
	one := func(kind, k, unit string, in input) {
		if in.Seen.Equal(now) {
			return
		}
		w := why[k]
		if w == "" {
			w = why["*"]
		}
		if w == "" && isCPU(k) {
			w = "no coretemp package sensor"
		}
		if w == "" {
			w = "not in reply"
		}
		od := c.overdue(in.Seen, interval, now)
		switch {
		case in.Seen.IsZero():
			log.Printf("%s%s %s: no reading yet (%s): unknown", prErr, kind, k, w)
		case od < unknownAfter:
			c.held = append(c.held, k)
			log.Printf("%s%s %s: no reading (%s) for %s: holding %g%s", prWarn, kind, k, w,
				now.Sub(in.Seen).Round(time.Second), in.V, unit)
		default:
			log.Printf("%s%s %s: no reading (%s), %s past due: unknown", prErr, kind, k, w, od.Round(time.Second))
		}
	}
	for _, k := range sortedKeys(c.in.Temps) {
		one("temperature", k, "C", c.in.Temps[k])
	}
	for _, k := range sortedKeys(c.in.Fans) {
		one("fan", k, " rpm", c.in.Fans[k])
	}
}

func isCPU(k string) bool { return strings.HasPrefix(k, "CPU") }

func sortedKeys(m map[string]input) []string {
	k := make([]string, 0, len(m))
	for x := range m {
		k = append(k, x)
	}
	sort.Strings(k)
	return k
}

// ---------- classify ----------

// Reading: everything classify knows about one input this cycle.
type Reading interface{ reading() }

// Fresh: read valid this cycle.
type Fresh struct{ V float64 }

// Stale: the last valid value, and how far past its due time it is (<= 0: not due yet).
type Stale struct {
	V       float64
	Overdue time.Duration
}

// NeverSeen: no valid reading this run (and none carried over).
type NeverSeen struct{}

func (Fresh) reading()     {}
func (Stale) reading()     {}
func (NeverSeen) reading() {}

// Class is ordered: the worst input picks the controller.
type Class int

const (
	classOK Class = iota
	classUnknown
	classDanger
)

func (k Class) String() string { return [...]string{"ok", "unknown", "danger"}[k] }

// kind: what an input is, which decides its limits.
type kind int

const (
	kindCPU kind = iota
	kindDrive
	kindExhaust
	kindOtherTemp // inlet: no limit of its own
	kindFan
	kindOther // drive scan, actuators: only "known or not"
)

type named struct {
	key  string
	kind kind
	r    Reading
}

func readingOf(in input, period time.Duration, now time.Time) Reading {
	switch {
	case in.Seen.IsZero():
		return NeverSeen{}
	case in.Seen.Equal(now):
		return Fresh{in.V}
	}
	return Stale{in.V, now.Sub(in.Seen) - period}
}

// readings: every input this cycle, in a fixed order.
func (c *controller) readings(now time.Time) []named {
	var out []named
	for _, k := range sortedKeys(c.in.Temps) {
		kd := kindOtherTemp
		switch {
		case isCPU(k):
			kd = kindCPU
		case k == "Exhaust":
			kd = kindExhaust
		}
		out = append(out, named{k, kd, readingOf(c.in.Temps[k], interval, now)})
	}
	for _, k := range sortedKeys(c.in.Fans) {
		out = append(out, named{k, kindFan, readingOf(c.in.Fans[k], interval, now)})
	}
	for _, k := range sortedKeys(c.in.HDD) {
		out = append(out, named{"drive " + k, kindDrive, readingOf(c.in.HDD[k], hddInterval+driveSlack, now)})
	}
	out = append(out, named{"drive scan", kindOther, readingOf(input{1, c.in.Drives}, hddInterval+driveSlack, now)})
	out = append(out, named{"power cap", kindOther, readingOf(c.in.Cap, reassert, now)})
	if safeClamp {
		out = append(out, named{"CPU clamp", kindOther, readingOf(c.in.Clamp, interval, now)})
	}
	return out
}

// seenOf: when the reading behind n was taken (for counting new readings).
func (c *controller) seenOf(n named) time.Time {
	switch n.kind {
	case kindDrive:
		return c.in.HDD[strings.TrimPrefix(n.key, "drive ")].Seen
	case kindFan:
		return c.in.Fans[n.key].Seen
	}
	return c.in.Temps[n.key].Seen
}

// confirm: a reading at/above its shutdown line is danger only after dangerReads consecutive new
// readings; until then it is unknown (the safe controller acts at once) and a drive is re-read now.
func (c *controller) confirm(n named, k Class, now time.Time) Class {
	if k != classDanger {
		delete(c.danger, n.key)
		return k
	}
	r, seen := c.danger[n.key], c.seenOf(n)
	if !seen.Equal(r.seen) {
		r.n, r.seen = r.n+1, seen
		c.danger[n.key] = r
		log.Printf("%s%s %g at/above its shutdown line (%d/%d)", prErr, n.key, value(n.r), r.n, dangerReads)
		if n.kind == kindDrive && r.n < dangerReads {
			c.kickDrives(now)
		}
	}
	if r.n >= dangerReads {
		return classDanger
	}
	return classUnknown
}

// classify: never seen or unknownAfter past due -> unknown; otherwise the value against its limits.
func classify(n named) Class {
	var v float64
	switch r := n.r.(type) {
	case NeverSeen:
		return classUnknown
	case Stale:
		if r.Overdue >= unknownAfter {
			return classUnknown
		}
		v = r.V
	case Fresh:
		v = r.V
	}
	switch n.kind {
	case kindCPU:
		if v >= cpuAuto {
			return classDanger
		}
	case kindDrive:
		if v >= hddAuto {
			return classDanger
		}
	case kindExhaust:
		if v >= exhaustAuto {
			return classDanger
		}
	case kindFan:
		if v < fanMinRPM { // commanded but not turning: the fan actuator isn't doing what it's told
			return classUnknown
		}
	}
	return classOK
}

func value(r Reading) float64 {
	switch r := r.(type) {
	case Fresh:
		return r.V
	case Stale:
		return r.V
	}
	return 0
}

// ---------- pick ----------

// pick: danger -> shutdown (whatever the mode); idrac mode -> idrac (the iDRAC has its own sensors);
// unknown -> safe; otherwise the mode's controller (model or fixed).
func pick(mode string, worst Class) string {
	switch {
	case worst == classDanger:
		return "shutdown"
	case mode == "idrac":
		return "idrac"
	case worst == classUnknown:
		return "safe"
	}
	return mode
}

// overFixed: a reading at its fixed.*_max: fixed hands over to model (and stays there).
func overFixed(ns []named) (string, bool) {
	for _, n := range ns {
		if classify(n) != classOK {
			continue
		}
		limit := map[kind]float64{kindCPU: fixedCPUMax, kindDrive: fixedHDDMax, kindExhaust: fixedExhaustMax}[n.kind]
		if limit > 0 && value(n.r) >= limit {
			return fmt.Sprintf("%s %g >= %g", n.key, value(n.r), limit), true
		}
	}
	return "", false
}

// ---------- control ----------

// ctlSafe: [safe] from the config (tested 2026-11-10: 20 % with the CPUs at minimum power holds a full load).
func ctlSafe(why string) desired {
	return desired{fan: strconv.Itoa(int(safeDuty)), duty: safeDuty, cap: int(safeCap), clamp: safeClamp, why: why, state: "safe"}
}

// decide: classify, pick, and run the picked controller.
func (c *controller) decide(now time.Time) (desired, error) {
	st.Lock()
	mode, fixedPct := st.mode, st.fixed
	st.Unlock()
	if c.off != nil { // a shutdown is final: the poweroff is under way
		return *c.off, nil
	}
	ns := c.readings(now)
	worst, worstKey := classOK, ""
	present := map[string]bool{}
	for _, n := range ns {
		present[n.key] = true
		if k := c.confirm(n, classify(n), now); k > worst {
			worst, worstKey = k, n.key
		}
	}
	for k := range c.danger { // a key gone (pulled drive, required_temps change) starts over if it returns
		if !present[k] {
			delete(c.danger, k)
		}
	}
	if len(c.in.Temps) == 0 || len(c.in.Fans) == 0 { // no sensor set yet: everything unknown
		worst, worstKey = max(worst, classUnknown), "no sensor data yet"
	}
	if mode == "fixed" {
		if why, over := overFixed(ns); over {
			st.Lock()
			st.mode = "model"
			st.Unlock()
			log.Printf("%sfixed -> model: %s", prWarn, why)
			mode = "model"
		}
	}
	why := fmt.Sprintf("%s %s", worstKey, worst)
	switch pick(mode, worst) {
	case "shutdown":
		return c.shutdown(why), nil
	case "idrac":
		return desired{fan: "auto", cap: int(pcapMax), why: "mode=idrac", state: "idrac"}, nil
	case "safe":
		return c.safe(why, now), nil
	case "fixed":
		return desired{fan: strconv.Itoa(fixedPct), duty: float64(fixedPct), cap: int(pcapMax), why: "fixed", state: "fixed"}, nil
	}
	return c.ctlModel(ns, now)
}

// safeWindow: how long the safe controller may take to be in effect by its read-backs (the fans
// spinning down from a high duty take a while).
const safeWindow = 15 * time.Second

// safe: the safe controller, checked by its read-backs; not in effect within safeWindow -> shutdown.
// Every way into safe (unknown input, decide failing) goes through here.
func (c *controller) safe(why string, now time.Time) desired {
	if c.last.state != "safe" {
		c.safeOK = now // just entered: the fans need a moment to reach safe.duty
	} else if bad := c.safeMismatch(now); bad == "" {
		c.safeOK = now
	} else if now.Sub(c.safeOK) >= safeWindow { // safe can't be established: last resort
		return c.shutdown("safe not established: " + bad)
	}
	return ctlSafe(why)
}

// shutdown: orderly poweroff; meanwhile the iDRAC cools, the CPUs at minimum power. Latched.
func (c *controller) shutdown(why string) desired {
	d := desired{fan: "auto", cap: int(pcapMin), clamp: true, why: why, state: "shutdown", off: true}
	c.off = &d
	return d
}

// safeMismatch: what the readbacks say differs from the safe controller's settings ("" = established).
func (c *controller) safeMismatch(now time.Time) string {
	usable := func(in input) bool { return !in.Seen.IsZero() && c.overdue(in.Seen, interval, now) < unknownAfter }
	var bad []string
	if c.in.Cap.Seen.IsZero() || c.overdue(c.in.Cap.Seen, reassert, now) >= unknownAfter || c.in.Cap.V != safeCap {
		bad = append(bad, fmt.Sprintf("power cap reads %g W, want %g", c.in.Cap.V, safeCap))
	}
	if safeClamp && (!usable(c.in.Clamp) || c.in.Clamp.V != 1) {
		bad = append(bad, "CPU clamp not in place")
	}
	var rpm []float64
	for _, in := range c.in.Fans {
		if usable(in) {
			rpm = append(rpm, in.V)
		}
	}
	if len(rpm) == 0 {
		bad = append(bad, "no fan RPM")
	} else {
		sort.Float64s(rpm)
		med, want := rpm[len(rpm)/2], pctToRPM(safeDuty)
		if math.Abs(med-want) > safeRPMTol*want {
			bad = append(bad, fmt.Sprintf("fans at %.0f rpm, want ~%.0f", med, want))
		}
	}
	return strings.Join(bad, "; ")
}

// ctlModel: power first (the cap holds the CPUs at model.cpu_target with the fans at model.fan_max), then the
// fans (drive and exhaust demand, or more power than the cap allows), never above modelCeiling.
// Only runs when every input is usable, so every value below is real or a moment old.
func (c *controller) ctlModel(ns []named, now time.Time) (desired, error) {
	temps, hdd := map[string]float64{}, map[string]float64{}
	for _, n := range ns {
		switch n.kind {
		case kindCPU, kindExhaust, kindOtherTemp:
			temps[n.key] = value(n.r)
		case kindDrive:
			hdd[strings.TrimPrefix(n.key, "drive ")] = value(n.r)
		}
	}
	watts := 0.0
	if c.overdue(c.in.Power.Seen, interval, now) < unknownAfter {
		watts = c.in.Power.V
	}
	r, capW, err := c.normal(temps, hdd, watts)
	if err != nil {
		return desired{}, err
	}
	st.Lock()
	st.demand, st.hasDemand, st.exhExpected = r.d, true, r.exhExpected
	st.Unlock()
	if math.IsNaN(r.target) || math.IsInf(r.target, 0) { // int(NaN) is undefined: could mean 0 %
		return desired{}, fmt.Errorf("duty computed as %v", r.target)
	}
	pct := max(0, min(modelCeiling, int(math.Round(r.target))))
	return desired{fan: strconv.Itoa(pct), duty: r.target, cap: capW, why: r.why, state: "model"}, nil
}

// normal: the model's control law (unchanged from before the redesign; without a power reading it
// falls back to the temperature curves).
func (c *controller) normal(temps, hdd map[string]float64, watts float64) (decision, int, error) {
	var r decision
	capW := int(pcapMax)
	ivl := interval.Seconds()
	ramping := c.duty > 0
	switch {
	default:
		// feedforward power: RAPL reacts within a cycle; the iDRAC wall reading lags ~30 s.
		// wall ~= RAPL + offset (fans, drives, RAM, PSU loss), learned slowly (~5 min)
		pr, hasPr := raplWatts()
		if hasPr && watts > 0 {
			if !c.raplLearned {
				c.raplOff, c.raplLearned = watts-pr, true
			} else {
				c.raplOff += ivl / 300 * (watts - pr - c.raplOff)
			}
		}
		ff := watts
		if hasPr && c.raplLearned {
			ff = math.Max(watts, pr+c.raplOff)
		}
		r = decide(temps, hdd, ff, true, c.modelI)
		st.Lock()
		st.ffWatts = ff
		st.Unlock()
		d, hot := r.d, hottestCPU(temps)
		over := hot - cpuTarget
		pinned := fanMax > 0 && d.cpu >= fanMax
		cpuEff := d.cpu
		if fanMax > 0 {
			cpuEff = math.Min(d.cpu, fanMax)
			if pinned { // CPU wants more than model.fan_max: hold the fans there, power does the rest
				r.target = math.Max(fanMax, math.Max(d.hdd, d.exhaust))
				r.why = fmt.Sprintf("model (fan ceiling %.0f%%, power capped)", fanMax)
			}
			// cap: most power that settles at model.cpu_target with fans at model.fan_max, plus a correction
			// that tightens while the CPU is still above target at the ceiling
			h := c.hotHist
			if pinned && over > 0 && (over >= 2 || (len(h) > 1 && h[len(h)-1] >= h[0])) {
				c.capCorr -= kw * over * ivl / 60
			} else if over < -2 {
				c.capCorr = math.Min(0, c.capCorr+kw*ivl/60)
			}
			c.capCorr = math.Max(-150, math.Min(0, c.capCorr))
			want := modelMaxWatts(inletOf(temps), fanMax) + c.capCorr
			if want >= pcapMax {
				want = pcapMax
			} else {
				want = math.Max(pcapMin, want)
			}
			if math.IsNaN(want) || math.IsInf(want, 0) {
				return r, 0, fmt.Errorf("power cap computed as %v", want)
			}
			if c.modelCap == 0 || math.Abs(want-c.modelCap) >= pcapStep || (want == pcapMax) != (c.modelCap == pcapMax) {
				c.modelCap = math.Round(want)
			}
			capW = int(c.modelCap)
		}
		emergency := hot >= cpuLimit
		if emergency { // past the hard line: no waiting
			r.target, r.why = math.Max(r.target, emergencyPct), fmt.Sprintf("cpu %.0fC >= limit %.0f", hot, cpuLimit)
		} else if ramping && r.target < c.duty { // step up at once, down <= 2%/cycle
			r.target = math.Max(r.target, c.duty-2)
		}
		// correction (anti-windup): only while the fans really run at the CPU's own demand, the
		// temperature is near the target, and the duty can still move that way
		c.hotHist = append(c.hotHist, hot)
		if n := max(2, int(60/ivl)+1); len(c.hotHist) > n {
			c.hotHist = c.hotHist[len(c.hotHist)-n:]
		}
		lo, hi := c.hotHist[0], c.hotHist[0]
		for _, v := range c.hotHist {
			lo, hi = math.Min(lo, v), math.Max(hi, v)
		}
		settled := float64(len(c.hotHist)) > 60/ivl && hi-lo <= 1
		// ... and only while the model's own request (not the warm floor) sets the CPU demand, from
		// real readings (a bridged value would train it on stale data)
		onCPU := !emergency && r.modelRan && r.modelPart >= r.warmPart && math.Abs(r.target-cpuEff) < 0.5 &&
			cpuEff >= math.Max(d.hdd, d.exhaust)
		if len(c.held) == 0 && settled && onCPU && math.Abs(over) < 5 && !(d.cpu <= minPct && over < 0) && !(pinned && over > 0) {
			c.modelI = math.Max(-iMax, math.Min(iMax, c.modelI+ki*over*ivl/60))
		}
		st.Lock()
		st.modelI, st.capCorr, st.modelOn = c.modelI, c.capCorr, true
		st.Unlock()
	}
	if fanMax <= 0 {
		c.modelCap = 0
	}
	return r, capW, nil
}

// ---------- act ----------

// act makes each actuator match d, writing only what differs (the duty also every inputs.reassert, which
// retakes the fans after an iDRAC reset). The clamp goes first: it is local, so it works even when
// the BMC doesn't.
func (c *controller) act(d desired, now time.Time) {
	if d.off {
		c.actShutdown(d)
		return
	}
	want := map[bool]string{false: "off", true: "on"}[d.clamp]
	var err error
	if d.clamp {
		var newly bool
		if newly, err = clampOn(); newly {
			log.Printf("%sCPU clamp on (%s)", prWarn, d.why)
		}
	} else {
		var restored bool
		if restored, err = clampOff(); restored {
			log.Printf("%sCPU clamp off", prWarn)
		}
	}
	if err != nil {
		log.Printf("%sCPU clamp %s failed: %v", prErr, want, err)
	}
	c.clamp.result(want, err, now)

	if w := strconv.Itoa(d.cap); c.pcap.have != w || (!c.in.Cap.Seen.IsZero() && c.in.Cap.V != float64(d.cap)) {
		err := c.bmc.SetPowerCap(d.cap) // reads the limit back and fails unless it took
		if err != nil {
			log.Printf("%spower cap %d W failed: %v", prErr, d.cap, err)
		} else {
			c.in.Cap = input{float64(d.cap), now} // that read-back
			log.Printf("power cap %d W (%s)", d.cap, d.why)
		}
		c.pcap.result(w, err, now)
	}

	if d.fan != c.fan.have || now.Sub(c.fan.at) >= reassert {
		var err error
		if d.fan == "auto" {
			err = c.bmc.FanAuto()
		} else {
			pct, _ := strconv.Atoi(d.fan)
			err = c.bmc.FanDuty(pct)
		}
		changed := d.fan != c.fan.have
		c.fan.result(d.fan, err, now)
		switch {
		case err != nil:
			log.Printf("%sfans -> %s failed: %v", prErr, d.fan, err)
		case changed && d.fan == "auto":
			log.Printf("fans -> iDRAC auto: %s", d.why)
		case changed:
			log.Printf("duty %s%% (%s) %s", d.fan, d.why, fmtTemps(c.heldValues()))
		}
		if err == nil {
			c.duty, _ = strconv.ParseFloat(d.fan, 64) // the down-ramp starts from what was really sent ("auto": 0)
		}
	}
	if d.state != c.last.state || (d.state == "safe" && d.why != c.last.why) {
		pr := prWarn
		if d.state == "safe" || d.state == "shutdown" {
			pr = prErr
		}
		log.Printf("%scontroller %s -> %s (%s)", pr, orNone(c.last.state), d.state, d.why)
	}
	c.last = d

	st.Lock()
	st.auto, st.duty = 0, -1
	if c.fan.have == "auto" {
		st.auto = 1
	} else if p, err := strconv.Atoi(c.fan.have); err == nil {
		st.duty = p
	}
	if p, err := strconv.Atoi(c.pcap.have); err == nil {
		st.powerCap = p
	}
	st.reason, st.state, st.clamped = d.why, d.state, c.clamp.have == "on"
	st.Unlock()
}

// actShutdown: the poweroff first, so nothing after it can delay or stop it; then, once, the fans to
// the iDRAC and the CPUs to minimum power, failures ignored (the machine is going down).
func (c *controller) actShutdown(d desired) {
	if !c.poweredOff {
		log.Printf("%sSHUTDOWN: %s", prErr, d.why)
		if err := poweroff(); err != nil {
			log.Printf("%spoweroff failed: %v (retrying next cycle)", prErr, err)
		} else {
			c.poweredOff = true
		}
	}
	if !c.shutdownTried {
		c.shutdownTried = true
		try("shutdown: fans to the iDRAC", func() { _ = c.bmc.FanAuto() })
		try("shutdown: power cap", func() { _ = c.bmc.SetPowerCap(d.cap) })
		try("shutdown: CPU clamp", func() { _, _ = clampOn() })
	}
	if d.state != c.last.state {
		log.Printf("%scontroller %s -> shutdown (%s)", prErr, orNone(c.last.state), d.why)
	}
	c.last = d
	st.Lock()
	st.auto, st.duty, st.reason, st.state = 1, -1, d.why, d.state
	st.Unlock()
}

func orNone(s string) string {
	if s == "" {
		return "none"
	}
	return s
}

func (c *controller) heldValues() map[string]float64 {
	v := map[string]float64{}
	for k, in := range c.in.Temps {
		if !in.Seen.IsZero() {
			v[k] = in.V
		}
	}
	return v
}

// ---------- the loop ----------

// try runs one step; a panic (a bug, a reply the parsers didn't expect) is logged and that step
// simply produced nothing this cycle.
func try(step string, f func()) (ok bool) {
	defer func() {
		if p := recover(); p != nil {
			log.Printf("%s%s: panic: %v\n%s", prErr, step, p, debug.Stack())
			st.Lock()
			st.errors++
			st.Unlock()
			ok = false
		}
	}()
	f()
	return true
}

func (c *controller) cycle(now time.Time) {
	try("sense", func() { c.sense(now) })
	var d desired
	var err error
	if !try("decide", func() { d, err = c.decide(now) }) {
		err = errors.New("panic")
	}
	if err != nil { // a bug or an impossible value: nothing to trust but the safe controller
		log.Printf("%sdecide: %v", prErr, err)
		why := "decide failed: " + err.Error()
		if !try("safe", func() { d = c.safe(why, now) }) {
			d = ctlSafe(why) // even the read-back check failed: plain safe
		}
	}
	try("act", func() { c.act(d, now) })
	try("save", func() { c.saveState(now) })
	st.Lock()
	st.temps, st.fans, st.watts = c.heldValues(), map[string]float64{}, c.in.Power.V
	for k, in := range c.in.Fans {
		st.fans[k] = in.V
	}
	st.ages = map[string]float64{}
	for k, in := range c.in.Temps {
		st.ages[k] = now.Sub(orStart(in.Seen, c.start)).Seconds()
	}
	for k, in := range c.in.Fans {
		st.ages[k] = now.Sub(orStart(in.Seen, c.start)).Seconds()
	}
	for k, in := range c.in.HDD {
		st.ages["drive "+k] = now.Sub(orStart(in.Seen, c.start)).Seconds()
	}
	st.lastOK = float64(now.UnixMilli()) / 1000
	st.Unlock()
}

func orStart(t, start time.Time) time.Time {
	if t.IsZero() {
		return start
	}
	return t
}

func (c *controller) run() {
	sdNotify("READY=1")
	for {
		select {
		case apply := <-c.reload:
			apply()
			c.fan.have = "" // re-send the duty under the new settings
		default:
		}
		c.cycle(c.now())
		sdNotify("WATCHDOG=1")
		time.Sleep(interval)
	}
}

func fmtTemps(m map[string]float64) string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	parts := make([]string, len(keys))
	for i, k := range keys {
		parts[i] = fmt.Sprintf("%s=%g", k, m[k])
	}
	return strings.Join(parts, " ")
}

// ---------- state across restarts ----------

// stateFile: the inputs with their ages, saved every cycle, so a restart or re-exec carries a running
// gap (and the sensors it must still expect) over: a sensor gone during the restart stays gone.
var stateFile = "/run/fanctl/inputs.json"

func (c *controller) saveState(now time.Time) {
	err := func() error {
		b, err := json.Marshal(struct {
			Saved time.Time
			In    inputs
		}{now, c.in})
		if err != nil {
			return err
		}
		tmp := stateFile + ".tmp"
		if err := os.WriteFile(tmp, b, 0o644); err != nil {
			return err
		}
		return os.Rename(tmp, stateFile) // atomic: a crash mid-write never leaves half a file
	}()
	msg := ""
	if err != nil {
		msg = err.Error()
	}
	if msg != c.saveErr && msg != "" {
		log.Printf("%ssave %s: %v", prErr, stateFile, err)
	}
	c.saveErr = msg
}

// loadState takes over the previous run's inputs; no file = a fresh start.
func (c *controller) loadState() {
	b, err := os.ReadFile(stateFile)
	if errors.Is(err, os.ErrNotExist) {
		return
	}
	var s struct {
		Saved time.Time
		In    inputs
	}
	if err == nil {
		err = json.Unmarshal(b, &s)
	}
	if err != nil {
		log.Printf("%signoring %s: %v", prWarn, stateFile, err)
		return
	}
	c.in = s.In
	c.in.init()
	log.Printf("inputs from %s (saved %s ago): %d temperatures, %d fans, %d drives", stateFile,
		c.now().Sub(s.Saved).Round(time.Second), len(c.in.Temps), len(c.in.Fans), len(c.in.HDD))
}

// ---------- CPU clamp (RAPL is locked by the BIOS: the frequency limit through intel_pstate) ----------

var (
	pstateDir     = "/sys/devices/system/cpu/intel_pstate"
	throttleState = "/run/fanctl/throttle" // intel_pstate values the clamp replaced; present while clamped
)

func readTrim(p string) (string, error) {
	b, err := os.ReadFile(p)
	return strings.TrimSpace(string(b)), err
}

func exists(p string) bool { _, err := os.Stat(p); return err == nil }

func writePstate(name, v string) error {
	if err := os.WriteFile(pstateDir+"/"+name, []byte(v), 0); err != nil {
		return fmt.Errorf("intel_pstate %s=%s: %w", name, v, err)
	}
	return nil
}

// clampOn turns turbo off and caps the CPUs at their lowest frequency, saving the old settings first
// (and refusing if it can't: the clamp could never be undone). Writes only what differs. newly = the
// clamp was not in place before this call.
func clampOn() (newly bool, err error) {
	if !exists(throttleState) {
		nt, err1 := readTrim(pstateDir + "/no_turbo")
		mx, err2 := readTrim(pstateDir + "/max_perf_pct")
		if err := errors.Join(err1, err2); err != nil {
			return false, fmt.Errorf("read intel_pstate: %w", err)
		}
		if err := os.WriteFile(throttleState, []byte(nt+" "+mx+"\n"), 0o644); err != nil {
			return false, fmt.Errorf("save CPU settings before the clamp: %w", err)
		}
		newly = true
	}
	mn, err1 := readTrim(pstateDir + "/min_perf_pct")
	nt, err2 := readTrim(pstateDir + "/no_turbo")
	mx, err3 := readTrim(pstateDir + "/max_perf_pct")
	if err := errors.Join(err1, err2, err3); err != nil {
		return newly, fmt.Errorf("read intel_pstate: %w", err)
	}
	if nt != "1" {
		err = errors.Join(err, writePstate("no_turbo", "1"))
	}
	if mx != mn {
		err = errors.Join(err, writePstate("max_perf_pct", mn))
	}
	return newly, err
}

// clampState: are the CPUs clamped right now (turbo off and max_perf_pct at its minimum)?
func clampState() (bool, error) {
	nt, err1 := readTrim(pstateDir + "/no_turbo")
	mx, err2 := readTrim(pstateDir + "/max_perf_pct")
	mn, err3 := readTrim(pstateDir + "/min_perf_pct")
	if err := errors.Join(err1, err2, err3); err != nil {
		return false, err
	}
	return nt == "1" && mx == mn, nil
}

// clampOff restores what clampOn saved (restored=false: nothing was clamped).
func clampOff() (restored bool, err error) {
	b, err := os.ReadFile(throttleState)
	if errors.Is(err, os.ErrNotExist) {
		return false, nil
	} else if err != nil {
		return false, err
	}
	f := strings.Fields(string(b))
	if len(f) != 2 {
		return false, fmt.Errorf("%s: unexpected content %q", throttleState, b)
	}
	if err := errors.Join(writePstate("max_perf_pct", f[1]), writePstate("no_turbo", f[0])); err != nil {
		return false, err
	}
	return true, os.Remove(throttleState)
}
