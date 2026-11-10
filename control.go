//go:build linux

package main

// ---------- control loop: sense -> choose (classify, pick) -> the picked controller ----------
//
// Errors are data, not branches. Every input is a value plus when it was last valid: a failed or
// implausible read changes nothing, the input just gets older.
//
//	sense     stores each reading and when it last read valid
//	classify  each input -> ok / unknown / danger (unknown: never seen, or unknownAfter past due)
//	pick      (mode, worst class) -> one controller: shutdown, idrac, safe, fixed or model
//	control   the picked controller writes what it wants (setFans, setCap, setClamp write only what
//	          differs); whether it took is read back as inputs (fan RPM, power cap, clamp)
//
// A panic or an impossible value in classify, pick or a controller -> the safe controller.

import (
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"maps"
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

// inputs: every reading that has read valid, kept with when it last did (carried over across a
// restart). A key appears with its first valid reading and stays, so a sensor that stops reporting
// ages into unknown instead of dropping out. Which inputs count is choose's business (readings).
type inputs struct {
	Temps  map[string]input // BMC temperatures
	Fans   map[string]input // BMC fan RPM
	HDD    map[string]input // drive temperatures, by serial
	Scan   input            // last drive scan: number of drives found
	Drives []string         // the serials that scan found (the drives that count)
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

// dangerRun: consecutive new readings at/above the shutdown line (danger is confirmed at dangerReads).
type dangerRun struct {
	n    int
	seen time.Time // the last reading counted
}

const (
	dangerReads = 3                // consecutive readings at/above a shutdown line before the poweroff
	driveSlack  = time.Minute      // a drive pass may run this much longer than usual
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
	state, why       string               // the controller that ran last cycle, and why it was picked
	safeOK           time.Time            // safe: last cycle its readbacks matched (or it was entered)
	danger           map[string]dangerRun // readings at/above their shutdown line, confirming
	lastKick         time.Time            // last early drive re-read asked for
	shutdownTried    bool                 // the shutdown's best-effort writes were tried once
	off              string               // why the shutdown was decided ("" = none): it stays (latched)
	bmcDown          bool                 // the BMC didn't answer this cycle

	// normal control
	modelI, capCorr float64
	modelCap        float64   // the model's cap request after hysteresis; 0 = none yet
	hotHist         []float64 // hottest CPU, last 60 s
	raplOff         float64   // learned wall - RAPL offset (W); valid when raplLearned
	raplLearned     bool
	hddSnap         time.Time         // drive snapshot last looked at (implausible values are logged once per snapshot)
	readWhy         map[string]string // what each source answered for a reading it didn't give, this cycle
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

// sense stores every reading that read valid as (value, now), and what each source answered for
// the ones that didn't. Nothing else: which inputs count and what to do about missing ones is choose's.
func (c *controller) sense(now time.Time) {
	temps, fans, why := c.senseBMC(now)
	c.readWhy = why // filled in as sense goes: this cycle's even if a later step fails
	cpus, cwhy := senseCoretemp()
	maps.Copy(temps, cpus)
	maps.Copy(why, cwhy)
	for k, v := range temps {
		c.in.Temps[k] = input{v, now}
	}
	for k, v := range fans {
		c.in.Fans[k] = input{v, now}
	}
	// the cap is read back every inputs.reassert (setCap's own read-back covers writes); a hung BMC
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
	if clamped, err := clampState(); err == nil {
		c.in.Clamp = input{map[bool]float64{false: 0, true: 1}[clamped], now}
	} else {
		why["CPU clamp"] = err.Error()
	}
	c.senseDrives()
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

// senseDrives takes the drive goroutine's latest snapshot: the scan (count and serials) and each
// plausible drive temperature, stamped with the snapshot's time.
func (c *controller) senseDrives() {
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
	c.hddSnap = at
	st.Lock()
	st.hdd = temps
	st.Unlock()
	if !scanned { // nothing known yet this run: the drive inputs keep their carried-over ages
		return
	}
	c.in.Scan, c.in.Drives = input{float64(len(ids)), at}, nil
	stored := c.in.HDD
	c.in.HDD = map[string]input{} // the drives the scan found (a pulled or replaced one leaves)
	for _, d := range ids {
		c.in.Drives = append(c.in.Drives, d.Serial)
		c.in.HDD[d.Serial] = stored[d.Serial]
		if t, ok := temps[d]; ok && at.After(stored[d.Serial].Seen) {
			c.in.HDD[d.Serial] = input{t, at}
		}
	}
}

func (c *controller) kickDrives(now time.Time) {
	select {
	case c.kick <- struct{}{}:
	default:
	}
	c.lastKick = now
}

// logInputs logs every input that counts and isn't fine, with the source's answer or the limit it
// missed: a warning while its last value still stands in, an error once it is unknown.
func (c *controller) logInputs(ns []named) {
	c.held = c.held[:0]
	if w := c.readWhy["*"]; w != "" {
		log.Printf("%sBMC: %s", prWarn, w)
	}
	for _, n := range ns {
		w := c.readWhy[n.key]
		if w == "" {
			w = c.readWhy["*"]
		}
		if w == "" {
			w = "no reading"
		}
		switch v := value(n.r); {
		case n.kind == kindDriveCount && v < float64(hddMin) && !isNeverSeen(n.r):
			log.Printf("%sdrive scan found %g drives (inputs.hdd_min %d): unknown", prErr, v, hddMin)
		case n.kind == kindFan && v < fanMinRPM && !isNeverSeen(n.r):
			log.Printf("%sfan %s at %g rpm (inputs.fan_min_rpm %g): unknown", prErr, n.key, v, fanMinRPM)
		case n.kind == kindDrive && c.in.HDD[strings.TrimPrefix(n.key, "drive ")].Seen.Before(c.in.Scan.Seen):
			log.Printf("%s%s: no temperature in the last scan, holding %g", prWarn, n.key, v)
		}
		switch r := n.r.(type) {
		case NeverSeen:
			log.Printf("%s%s: never read (%s): unknown", prErr, n.key, w)
		case Stale:
			switch {
			case r.Overdue < 0: // not due yet
			case r.Overdue < unknownAfter:
				c.held = append(c.held, n.key)
				log.Printf("%s%s: %s, %s past due: holding %g", prWarn, n.key, w, r.Overdue.Round(time.Second), r.V)
			default:
				log.Printf("%s%s: %s, %s past due: unknown", prErr, n.key, w, r.Overdue.Round(time.Second))
			}
		}
	}
}

func isCPU(k string) bool { return strings.HasPrefix(k, "CPU") }

func isNeverSeen(r Reading) bool { _, ok := r.(NeverSeen); return ok }

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
	kindDriveCount // drives found by the scan: fewer than inputs.hdd_min -> unknown
	kindOther      // read-backs: only "known or not"
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
	temps := maps.Clone(c.in.Temps)
	for _, k := range requiredTemps { // required: counted even if never read
		if _, ok := temps[k]; !ok {
			temps[k] = input{}
		}
	}
	for _, k := range sortedKeys(temps) {
		kd := kindOtherTemp
		switch {
		case isCPU(k):
			kd = kindCPU
		case k == "Exhaust":
			kd = kindExhaust
		}
		out = append(out, named{k, kd, readingOf(temps[k], interval, now)})
	}
	for _, k := range sortedKeys(c.in.Fans) {
		out = append(out, named{k, kindFan, readingOf(c.in.Fans[k], interval, now)})
	}
	drives := slices.Clone(c.in.Drives)
	sort.Strings(drives)
	for _, k := range drives { // the drives of the last scan
		out = append(out, named{"drive " + k, kindDrive, readingOf(c.in.HDD[k], hddInterval+driveSlack, now)})
	}
	out = append(out, named{"drive scan", kindDriveCount, readingOf(c.in.Scan, hddInterval+driveSlack, now)})
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
	case kindDriveCount:
		if v < float64(hddMin) { // drives missing, or the PERC lost them: not "fewer drives to watch"
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

// choose: classify every input, confirm danger, and pick the controller. The only place that looks at
// ages and classes; the controllers below only control.
func (c *controller) choose(now time.Time) (name, why string, ns []named) {
	if c.off != "" { // a shutdown is final: the poweroff is under way
		return "shutdown", c.off, nil
	}
	st.Lock()
	mode := st.mode
	st.Unlock()
	ns = c.readings(now)
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
	c.logInputs(ns)
	for _, k := range c.in.Drives { // a drive without a temperature in the last scan: re-read it now
		if c.in.HDD[k].Seen.Before(c.in.Scan.Seen) && now.Sub(c.lastKick) >= kickEvery {
			c.kickDrives(now)
		}
	}
	if len(c.in.Temps) == 0 || len(c.in.Fans) == 0 { // no sensor set yet: everything unknown
		worst, worstKey = max(worst, classUnknown), "no sensor data yet"
	}
	if mode == "fixed" {
		if over, hit := overFixed(ns); hit {
			st.Lock()
			st.mode = "model"
			st.Unlock()
			log.Printf("%sfixed -> model: %s", prWarn, over)
			mode = "model"
		}
	}
	return pick(mode, worst), fmt.Sprintf("%s %s", worstKey, worst), ns
}

// ---------- controllers: each one writes what it wants through setFans, setCap, setClamp ----------

// ctlModel: power first (the cap holds the CPUs at model.cpu_target with the fans at model.fan_max), then
// the fans (drive and exhaust demand, or more power than the cap allows), never above modelCeiling.
// Only picked when every input is usable, so every value below is real or a moment old.
func (c *controller) ctlModel(ns []named, now time.Time) error {
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
		return err
	}
	if math.IsNaN(r.target) || math.IsInf(r.target, 0) { // int(NaN) is undefined: could mean 0 %
		return fmt.Errorf("duty computed as %v", r.target)
	}
	st.Lock()
	st.demand, st.hasDemand, st.exhExpected = r.d, true, r.exhExpected
	st.Unlock()
	c.apply(strconv.Itoa(max(0, min(modelCeiling, int(math.Round(r.target))))), capW, false, r.why, now)
	return nil
}

// ctlFixed: fixed.duty (or a /control choice), as it is.
func (c *controller) ctlFixed(now time.Time) {
	st.Lock()
	pct := st.fixed
	st.Unlock()
	c.apply(strconv.Itoa(pct), int(pcapMax), false, "fixed", now)
}

// ctlIdrac: the iDRAC runs the fans.
func (c *controller) ctlIdrac(now time.Time) {
	c.apply("auto", int(pcapMax), false, "mode=idrac", now)
}

// safeWindow: how long the safe controller may take to be in effect by its read-backs (the fans
// spinning down from a high duty take a while).
const safeWindow = 15 * time.Second

// ctlSafe: [safe] from the config (tested 2026-11-10: 20 % with the CPUs at minimum power holds a full
// load). It must be in effect by its read-backs within safeWindow; if not: shutdown, the last resort.
func (c *controller) ctlSafe(why string, now time.Time) {
	if c.state != "safe" {
		c.safeOK = now // just entered: the fans need a moment to reach safe.duty
	} else if bad := c.safeMismatch(now); bad == "" {
		c.safeOK = now
	} else if now.Sub(c.safeOK) >= safeWindow {
		c.ctlShutdown("safe not established: " + bad)
		return
	} else {
		c.fan.have = "" // not in effect yet: send the duty again rather than wait for reassert
	}
	c.apply(strconv.Itoa(int(safeDuty)), int(safeCap), safeClamp, why, now)
}

// ctlShutdown: the poweroff first, so nothing after it can delay or stop it; then, once, the fans to
// the iDRAC and the CPUs to minimum power, failures ignored (the machine is going down). Latched.
func (c *controller) ctlShutdown(why string) {
	if c.off == "" {
		c.off = why
	}
	if !c.poweredOff {
		log.Printf("%sSHUTDOWN: %s", prErr, why)
		if err := poweroff(); err != nil {
			log.Printf("%spoweroff failed: %v (retrying next cycle)", prErr, err)
		} else {
			c.poweredOff = true
		}
	}
	if !c.shutdownTried { // once, each on its own (one failing can't skip the others); failures don't matter
		c.shutdownTried = true
		now := time.Now()
		try("shutdown: fans to the iDRAC", func() { c.setFans("auto", why, now) })
		try("shutdown: CPU clamp", func() { c.setClamp(true, why) })
		try("shutdown: power cap", func() { c.setCap(int(pcapMin), why, now) })
	}
}

// safeMismatch: why the safe controller isn't in effect by its read-backs ("" = it is): the fans at
// least safe.check_duty's RPM, the power cap at most safe.cap, the clamp on if [safe] uses it.
func (c *controller) safeMismatch(now time.Time) string {
	usable := func(in input) bool { return !in.Seen.IsZero() && c.overdue(in.Seen, interval, now) < unknownAfter }
	var bad []string
	if c.in.Cap.Seen.IsZero() || c.overdue(c.in.Cap.Seen, reassert, now) >= unknownAfter || c.in.Cap.V > safeCap {
		bad = append(bad, fmt.Sprintf("power cap reads %g W, want <= %g", c.in.Cap.V, safeCap))
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
		return strings.Join(append(bad, "no fan RPM"), "; ")
	}
	sort.Float64s(rpm)
	if med, floor := rpm[len(rpm)/2], pctToRPM(safeCheckDuty); med < floor { // faster is fine: more cooling
		bad = append(bad, fmt.Sprintf("fans at %.0f rpm, want >= %.0f", med, floor))
	}
	return strings.Join(bad, "; ")
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

// ---------- writing: what the controllers call ----------

// apply: the three actuators, always in this order: the clamp first (local, works without the BMC),
// then the power cap, then the fans. Each writes only what differs.
func (c *controller) apply(fans string, capW int, clamp bool, why string, now time.Time) {
	c.setClamp(clamp, why)
	c.setCap(capW, why, now)
	c.setFans(fans, why, now)
}

// setFans: the duty ("auto": the iDRAC's control) when it isn't what was last written, and again every
// inputs.reassert (retakes the fans after an iDRAC reset).
func (c *controller) setFans(want, why string, now time.Time) {
	if want == c.fan.have && now.Sub(c.fan.at) < reassert {
		return
	}
	var err error
	if want == "auto" {
		err = c.bmc.FanAuto()
	} else {
		pct, _ := strconv.Atoi(want)
		err = c.bmc.FanDuty(pct)
	}
	changed := want != c.fan.have
	c.fan.result(want, err, now)
	if err != nil {
		log.Printf("%sfans %s failed: %v", prErr, want, err)
		return
	}
	c.duty, _ = strconv.ParseFloat(want, 64) // the model's down-ramp starts here ("auto": 0)
	if changed {
		log.Printf("fans %s (%s) %s", want, why, fmtTemps(c.heldValues()))
	}
}

// setCap: the iDRAC power cap when it isn't what was last written or reads back different.
// SetPowerCap reads the limit back and fails unless it took, so a success is the read-back.
func (c *controller) setCap(w int, why string, now time.Time) {
	if strconv.Itoa(w) == c.pcap.have && (c.in.Cap.Seen.IsZero() || c.in.Cap.V == float64(w)) {
		return
	}
	err := c.bmc.SetPowerCap(w)
	c.pcap.result(strconv.Itoa(w), err, now)
	if err != nil {
		log.Printf("%spower cap %d W failed: %v", prErr, w, err)
		return
	}
	c.in.Cap = input{float64(w), now}
	log.Printf("power cap %d W (%s)", w, why)
}

// setClamp: the CPU clamp on or off (both write only what differs).
func (c *controller) setClamp(on bool, why string) {
	var changed bool
	var err error
	if on {
		changed, err = clampOn()
	} else {
		changed, err = clampOff()
	}
	c.clamp.result(strconv.FormatBool(on), err, time.Time{})
	switch {
	case err != nil:
		log.Printf("%sCPU clamp on=%v failed: %v", prErr, on, err)
	case changed:
		log.Printf("%sCPU clamp on=%v (%s)", prWarn, on, why)
	}
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

// runController: the picked controller. An error means it couldn't produce anything sane (NaN).
func (c *controller) runController(name, why string, ns []named, now time.Time) error {
	switch name {
	case "shutdown":
		c.ctlShutdown(why)
	case "idrac":
		c.ctlIdrac(now)
	case "safe":
		c.ctlSafe(why, now)
	case "fixed":
		c.ctlFixed(now)
	default:
		return c.ctlModel(ns, now)
	}
	return nil
}

func (c *controller) cycle(now time.Time) {
	try("sense", func() { c.sense(now) })
	var name, why string
	var ns []named
	if !try("choose", func() { name, why, ns = c.choose(now) }) {
		name, why = "safe", "choosing the controller failed"
	}
	var err error
	if !try(name, func() { err = c.runController(name, why, ns, now) }) {
		err = errors.New("panic")
	}
	if err != nil && name != "safe" && name != "shutdown" { // a bug or an impossible value: the safe one
		log.Printf("%s%s controller: %v", prErr, name, err)
		name, why = "safe", name+" controller failed: "+err.Error()
		try("safe", func() { c.ctlSafe(why, now) })
	}
	if c.off != "" {
		name, why = "shutdown", c.off
	}
	if name != c.state || (name == "safe" && why != c.why) {
		pr := prWarn
		if name == "safe" || name == "shutdown" {
			pr = prErr
		}
		log.Printf("%scontroller %s -> %s (%s)", pr, orNone(c.state), name, why)
	}
	c.state, c.why = name, why
	try("save", func() { c.saveState(now) })
	st.Lock()
	st.state, st.reason, st.clamped = name, why, c.clamp.have == "true"
	st.auto, st.duty = 0, -1
	if c.fan.have == "auto" {
		st.auto = 1
	} else if p, err := strconv.Atoi(c.fan.have); err == nil {
		st.duty = p
	}
	if p, err := strconv.Atoi(c.pcap.have); err == nil {
		st.powerCap = p
	}
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
	for k, in := range c.in.Temps { // placeholders from older builds: never-read inputs aren't stored
		if in.Seen.IsZero() {
			delete(c.in.Temps, k)
		}
	}
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
