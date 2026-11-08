//go:build linux

// fanctl: Dell R730xd fan controller + Prometheus exporter.
//
// Reads CPU/inlet/exhaust temps, fan RPM and wall power from the local BMC (/dev/ipmi0) and drive
// temps through the PERC (megaraid passthrough), picks a fan duty, and sets it with Dell's OEM fan
// commands. In model mode the duty comes from the fitted thermal model, and when the fans reach
// FAN_MAX the iDRAC power cap holds the CPU at target instead.
//
// Fail-safe: a failed sensor read, an over-limit temperature, missing drive temps, or this process
// exiting hands control back to the iDRAC (automatic mode). Under systemd (Type=notify,
// WatchdogSec) a hung control loop is killed, and ExecStopPost runs `fanctl auto`.
//
//	fanctl [run]   control loop (reads its settings from the environment)
//	fanctl auto    hand the fans back to the iDRAC and exit
//	fanctl probe   print every reading fanctl uses; changes nothing
//	fanctl prepare create the PERC ioctl node if missing (run as root before the service)
package main

import (
	"crypto/subtle"
	"fmt"
	"log"
	"math"
	"net"
	"net/http"
	"os"
	"os/exec"
	"os/signal"
	"runtime/debug"
	"sort"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"
)

// ---------- settings (environment, same names as the old Python fanctl) ----------

type point struct{ x, y float64 }
type curve []point

type cpuModel struct{ pb, r0, k, n, scale float64 }

var (
	mode0, controlToken, listen                            string
	requiredTemps                                          []string
	hddMin                                                 int
	fanMinRPM                                              float64
	fixedPct0                                              int
	minPct, cpuTarget, modelMargin, cpuLimit, emergencyPct float64
	fanMax, pcapMin, pcapMax, pcapStep, kw, ki, iMax       float64
	cpuAuto, hddAuto, exhaustAuto                          float64
	interval, hddInterval, hddRescan, reassert             time.Duration
	cpuCurve, hddCurve, exhaustCurve, warmCurve, fanRPMTab curve
	cpuModels                                              []cpuModel
	exhaustModel                                           []float64 // a, b (EXHAUST_MODEL)
	exhaustExcessCurve                                     curve
)

func env(k, def string) string {
	if v, ok := os.LookupEnv(k); ok {
		return v
	}
	return def
}

func envFloat(k, def string) float64 {
	v, err := strconv.ParseFloat(strings.TrimSpace(env(k, def)), 64)
	if err != nil {
		log.Fatalf("%s: %v", k, err)
	}
	return v
}

func envSeconds(k, def string) time.Duration {
	return time.Duration(envFloat(k, def) * float64(time.Second))
}

// envCurve parses "x:y,x:y,..." into points sorted by x.
func envCurve(k, def string, minPoints int) curve {
	var c curve
	for _, f := range strings.Split(env(k, def), ",") {
		xs, ys, ok := strings.Cut(strings.TrimSpace(f), ":")
		x, err1 := strconv.ParseFloat(xs, 64)
		y, err2 := strconv.ParseFloat(ys, 64)
		if !ok || err1 != nil || err2 != nil {
			log.Fatalf("%s: bad point %q", k, f)
		}
		c = append(c, point{x, y})
	}
	if len(c) < minPoints {
		log.Fatalf("%s: needs at least %d points", k, minPoints)
	}
	sort.Slice(c, func(i, j int) bool { return c[i].x < c[j].x })
	return c
}

func envModel(k, def string) cpuModel {
	f := strings.Split(env(k, def), ",")
	if len(f) != 5 {
		log.Fatalf("%s: want pb,r0,k,n,scale", k)
	}
	v := make([]float64, 5)
	for i, s := range f {
		var err error
		if v[i], err = strconv.ParseFloat(strings.TrimSpace(s), 64); err != nil {
			log.Fatalf("%s: %v", k, err)
		}
	}
	return cpuModel{v[0], v[1], v[2], v[3], v[4]}
}

func loadConfig() {
	mode0 = env("MODE", "curve") // curve | model | fixed | auto
	fixedPct0 = int(envFloat("FIXED_PCT", "20"))
	minPct = envFloat("MIN_PCT", "10")
	// duty = max of the three curves; below a curve's first point: MIN_PCT
	cpuCurve = envCurve("CPU_CURVE", "55:10,62:15,68:25,74:40,80:60", 1)
	hddCurve = envCurve("HDD_CURVE", "38:10,41:20,44:35,47:50", 1)
	exhaustCurve = envCurve("EXHAUST_CURVE", "38:10,42:25,46:45", 1)
	// The exhaust demand reacts only to heat the sensors don't explain (PERC, RAM, PCIe, a blocked vent).
	// Expected exhaust, fitted on steady history (fit 2026-11-08 over 2026-10-31..11-08, rmse 1.2 C; airflow
	// cancels out: faster fans take more heat off a part but spread it through proportionally more air):
	//   T_exh = T_inlet + a * (T_cpu - T_inlet) + b * (T_hdd - T_inlet)    (hottest CPU, hottest drive)
	// EXHAUST_EXCESS_CURVE maps the excess (measured - expected, C) to a duty. EXHAUST_AUTO still applies to
	// the absolute exhaust, and a cycle without a drive reading falls back to the absolute EXHAUST_CURVE.
	for _, f := range strings.Split(env("EXHAUST_MODEL", "0.3716,0.1911"), ",") {
		x, err := strconv.ParseFloat(strings.TrimSpace(f), 64)
		if err != nil {
			log.Fatalf("EXHAUST_MODEL: %v", err)
		}
		exhaustModel = append(exhaustModel, x)
	}
	exhaustExcessCurve = envCurve("EXHAUST_EXCESS_CURVE", "3:10,5:20,7:40", 1)
	// MODE=model: lowest duty that keeps the hottest CPU at CPU_TARGET, from the fitted steady-state model
	//   T_cpu = T_inlet + (P - pb) * (r0 + k * (rpm/1000)^-n) * scale      (P = iDRAC wall power)
	// plus a slow integral correction on the measured temperature (model error, dust, layout, room).
	cpuTarget = envFloat("CPU_TARGET", "75")
	modelMargin = envFloat("MODEL_MARGIN", "3") // model aims this far below CPU_TARGET
	cpuLimit = envFloat("CPU_LIMIT", "78")      // at/above: jump to EMERGENCY_PCT at once
	emergencyPct = envFloat("EMERGENCY_PCT", "50")
	cpuModels = []cpuModel{ // pb,r0,k,n,scale (fit 2026-10-31, current drive layout)
		envModel("CPU1_MODEL", "58.6228,0.1107,0.6588,1.4572,0.9565"),
		envModel("CPU2_MODEL", "89.5481,0.1445,1.1485,1.8038,1.0562")}
	fanRPMTab = envCurve("FAN_RPM_TABLE", "10:3270,15:4060,20:4790,25:5560,30:6430", 2)
	// fan ceiling: in model mode the CPU part of the demand never exceeds FAN_MAX; instead the iDRAC
	// power cap is set to the most power that settles at CPU_TARGET with the fans at FAN_MAX
	fanMax = envFloat("FAN_MAX", "0")     // 0 = no ceiling, no cap management
	pcapMin = envFloat("PCAP_MIN", "230") // never cap below this (W)
	pcapMax = envFloat("PCAP_MAX", "586") // "no cap" (the iDRAC's highest accepted limit)
	pcapStep = envFloat("PCAP_STEP", "6") // only re-send when the cap moves this much (W)
	kw = envFloat("PCAP_KW", "8")         // cap correction, W per C per minute
	// model mode: fan floor that rises as the hottest CPU warms, so a warm CPU keeps headroom for a load step
	warmCurve = envCurve("WARM_CURVE", "68:10,72:15,76:20", 1)
	ki = envFloat("MODEL_KI", "1.0")     // % per C per minute
	iMax = envFloat("MODEL_I_MAX", "15") // correction limit, +/- %
	cpuAuto = envFloat("CPU_AUTO", "85")
	hddAuto = envFloat("HDD_AUTO", "50")
	exhaustAuto = envFloat("EXHAUST_AUTO", "50")
	interval = envSeconds("INTERVAL", "10")
	hddInterval = envSeconds("HDD_INTERVAL", "300")
	hddRescan = envSeconds("HDD_RESCAN", "600") // look for inserted/removed drives
	reassert = envSeconds("REASSERT", "60")     // re-send the duty this often
	listen = env("LISTEN", "127.0.0.1:9101")    // comma-separated
	controlToken = env("CONTROL_TOKEN", "")     // empty = /control disabled
	// temperatures that must read every cycle (a missing one never counts as "cool")
	for _, t := range strings.Split(env("REQUIRED_TEMPS", "CPU1,Inlet,Exhaust"), ",") {
		if t = strings.TrimSpace(t); t != "" {
			requiredTemps = append(requiredTemps, t)
		}
	}
	hddMin = int(envFloat("HDD_MIN", "0"))      // fewer drives seen -> iDRAC
	fanMinRPM = envFloat("FAN_MIN_RPM", "1000") // a fan below this (dead/stalled) -> iDRAC
	switch mode0 {
	case "curve", "model", "fixed", "auto":
	default:
		log.Fatalf("MODE must be curve, model, fixed or auto")
	}
	validateConfig()
}

// ---------- reload (systemctl reload fanctl = SIGHUP) ----------
//
// systemd hands the settings over as environment variables (EnvironmentFile), read once at start. On a
// reload fanctl reads the env file itself, has a "fanctl check" subprocess validate it (a bad value there
// exits that process, not this one), and only then swaps the settings in between two control cycles, so
// the fans never pass through the iDRAC. MODE/FIXED_PCT from the file replace a runtime /control choice.
// LISTEN and CONTROL_TOKEN changes need a restart.

func envFilePath() string { return env("FANCTL_ENV_FILE", "/etc/fanctl.env") }

// readEnvFile parses KEY=VALUE lines like systemd's EnvironmentFile: blank lines and # or ; comments are
// skipped, and one pair of surrounding quotes is removed.
func readEnvFile(path string) (map[string]string, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	m := map[string]string{}
	for _, l := range strings.Split(string(b), "\n") {
		l = strings.TrimSpace(l)
		if l == "" || l[0] == '#' || l[0] == ';' {
			continue
		}
		k, v, ok := strings.Cut(l, "=")
		if !ok {
			continue
		}
		k, v = strings.TrimSpace(k), strings.TrimSpace(v)
		if len(v) >= 2 && (v[0] == '"' || v[0] == '\'') && v[len(v)-1] == v[0] {
			v = v[1 : len(v)-1]
		}
		m[k] = v
	}
	return m, nil
}

var fileKeys map[string]string // what the env file held at the last (re)load

// prepareReload reads and validates the env file; the returned func applies it (call between cycles).
func prepareReload() (func(), error) {
	m, err := readEnvFile(envFilePath())
	if err != nil {
		return nil, err
	}
	self, err := os.Executable()
	if err != nil {
		return nil, err
	}
	cmd := exec.Command(self, "check")
	cmd.Env = []string{"PATH=/usr/sbin:/usr/bin:/sbin:/bin"}
	for k, v := range m {
		cmd.Env = append(cmd.Env, k+"="+v)
	}
	if out, err := cmd.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("%s refused: %s", envFilePath(), strings.TrimSpace(string(out)))
	}
	return func() {
		oldListen, oldToken := listen, controlToken
		for k := range fileKeys { // a line removed from the file falls back to its default
			if _, still := m[k]; !still {
				os.Unsetenv(k)
			}
		}
		for k, v := range m {
			os.Setenv(k, v)
		}
		fileKeys = m
		st.Lock() // /metrics reads the settings under this lock
		loadConfig()
		listen, controlToken = oldListen, oldToken
		st.mode, st.fixed = mode0, fixedPct0
		st.Unlock()
		log.Printf("reloaded %s: mode=%s", envFilePath(), mode0)
	}, nil
}

// validateConfig refuses settings that would make the controller unsafe or stall the loop past the
// systemd watchdog. A refused config exits, and systemd leaves the fans with the iDRAC.
func validateConfig() {
	check := func(ok bool, f string, a ...any) {
		if !ok {
			log.Fatalf("config: "+f, a...)
		}
	}
	fin := func(v ...float64) bool {
		for _, x := range v {
			if math.IsNaN(x) || math.IsInf(x, 0) {
				return false
			}
		}
		return true
	}
	pct := func(v float64) bool { return v >= 0 && v <= 100 }
	check(fin(minPct, cpuTarget, modelMargin, cpuLimit, emergencyPct, fanMax, pcapMin, pcapMax, pcapStep, kw, ki, iMax,
		cpuAuto, hddAuto, exhaustAuto, fanMinRPM), "settings must be finite numbers")
	check(interval >= time.Second && interval <= 10*time.Second, "INTERVAL must be 1-10 s (the systemd watchdog is 60 s)")
	check(hddInterval >= 10*time.Second && hddRescan >= hddInterval, "need HDD_INTERVAL >= 10 s and HDD_RESCAN >= HDD_INTERVAL")
	check(reassert >= interval, "REASSERT must be >= INTERVAL")
	check(minPct >= 1 && minPct <= 100, "MIN_PCT must be 1-100")
	check(fixedPct0 >= 5 && fixedPct0 <= 100, "FIXED_PCT must be 5-100")
	check(fanMax == 0 || (fanMax >= minPct && fanMax <= 100), "FAN_MAX must be 0 or MIN_PCT-100")
	check(emergencyPct >= minPct && emergencyPct <= 100, "EMERGENCY_PCT must be MIN_PCT-100")
	check(cpuTarget-modelMargin > 20 && cpuTarget < cpuLimit && cpuLimit < cpuAuto && cpuAuto <= 100,
		"need 20 < CPU_TARGET-MODEL_MARGIN, CPU_TARGET < CPU_LIMIT < CPU_AUTO <= 100")
	check(hddAuto > 0 && hddAuto <= 70 && exhaustAuto > 0 && exhaustAuto <= 80, "HDD_AUTO must be <= 70, EXHAUST_AUTO <= 80")
	check(pcapMin >= 100 && pcapMin < pcapMax && pcapMax <= 2000 && pcapMax == math.Trunc(pcapMax) && pcapStep > 0,
		"need 100 <= PCAP_MIN < PCAP_MAX <= 2000, PCAP_MAX a whole number, PCAP_STEP > 0")
	check(hddMin >= 0 && fanMinRPM >= 0, "HDD_MIN and FAN_MIN_RPM must be >= 0")
	check(ki >= 0 && iMax >= 0 && kw >= 0, "MODEL_KI, MODEL_I_MAX, PCAP_KW must be >= 0")
	check(len(exhaustModel) == 2 && fin(exhaustModel...) && exhaustModel[0] >= 0 && exhaustModel[1] >= 0 && exhaustModel[0]+exhaustModel[1] < 1,
		"EXHAUST_MODEL: need a,b >= 0 with a+b < 1 (the exhaust is a mix of inlet air and air warmed by the parts)")
	for name, c := range map[string]curve{"CPU_CURVE": cpuCurve, "HDD_CURVE": hddCurve, "EXHAUST_CURVE": exhaustCurve, "WARM_CURVE": warmCurve, "EXHAUST_EXCESS_CURVE": exhaustExcessCurve} {
		for _, p := range c {
			check(fin(p.x) && pct(p.y), "%s: bad point %g:%g (duty must be 0-100)", name, p.x, p.y)
		}
	}
	for i, p := range fanRPMTab {
		check(pct(p.x) && fin(p.y) && p.y > 0 && (i == 0 || (p.x > fanRPMTab[i-1].x && p.y > fanRPMTab[i-1].y)),
			"FAN_RPM_TABLE must rise in both duty and rpm")
	}
	for i, m := range cpuModels {
		check(fin(m.pb, m.r0, m.k, m.n, m.scale) && m.pb >= 0 && m.pb < 1000 && m.r0 >= 0 && m.k > 0 && m.n > 0 && m.n < 10 && m.scale > 0,
			"CPU%d_MODEL: need finite pb (0-1000), r0 >= 0, k > 0, n (0-10), scale > 0", i+1)
	}
}

// ---------- shared state (control loop -> /metrics, /control -> control loop) ----------

type demand struct{ cpu, hdd, exhaust float64 }

var st = struct {
	sync.Mutex
	temps, fans        map[string]float64
	hdd                map[Drive]float64
	watts              float64 // 0 = no reading
	duty, auto, errors int
	reason, mode       string
	fixed              int
	lastOK             float64
	powerCap           int // 0 = never set
	capCorr, ffWatts   float64
	modelI             float64
	modelOn, hasDemand bool
	demand             demand
	exhExpected        float64 // 0 = not computed (no model or no drive temperature)
}{duty: -1, auto: 1, reason: "start"}

// drive temperatures, refreshed by their own goroutine so a slow or hung drive never stalls the loop
var hdds = struct {
	sync.Mutex
	ids     []Drive
	temps   map[Drive]float64
	at      time.Time
	scanned bool
}{}

// ---------- model ----------

func interp(c curve, v float64) float64 { // MIN_PCT below the first point, last y above the last
	if v <= c[0].x {
		return math.Max(minPct, c[0].y)
	}
	for i := 1; i < len(c); i++ {
		if a, b := c[i-1], c[i]; v <= b.x {
			return a.y + (b.y-a.y)*(v-a.x)/(b.x-a.x)
		}
	}
	return c[len(c)-1].y
}

// rpmToPct inverts FAN_RPM_TABLE (duty:rpm), linear between points, extrapolated past the ends.
func rpmToPct(rpm float64) float64 {
	t := fanRPMTab
	for i := 1; ; i++ {
		if a, b := t[i-1], t[i]; rpm <= b.y || i == len(t)-1 {
			return a.x + (b.x-a.x)*(rpm-a.y)/(b.y-a.y)
		}
	}
}

func pctToRPM(pct float64) float64 {
	t := fanRPMTab
	for i := 1; ; i++ {
		if a, b := t[i-1], t[i]; pct <= b.x || i == len(t)-1 {
			return a.y + (b.y-a.y)*(pct-a.x)/(b.x-a.x)
		}
	}
}

// modelPct: lowest duty at which every CPU settles at CPU_TARGET for this power and inlet, or 100.
func modelPct(watts, inlet float64) float64 {
	need := minPct
	for _, m := range cpuModels {
		if watts <= m.pb {
			continue
		}
		allowed := (cpuTarget-modelMargin-inlet)/((watts-m.pb)*m.scale) - m.r0 // airflow budget left
		if allowed <= 0 {
			return 100 // unreachable by airflow alone
		}
		need = math.Max(need, rpmToPct(1000*math.Pow(m.k/allowed, 1/m.n)))
	}
	return math.Min(100, need)
}

// modelMaxWatts: most wall power at which every CPU settles at CPU_TARGET with the fans at pct.
func modelMaxWatts(inlet, pct float64) float64 {
	x, w := pctToRPM(pct)/1000, math.Inf(1)
	for _, m := range cpuModels {
		w = math.Min(w, m.pb+(cpuTarget-modelMargin-inlet)/((m.r0+m.k*math.Pow(x, -m.n))*m.scale))
	}
	return w
}

func hottestCPU(temps map[string]float64) float64 {
	hot := 0.0
	for k, v := range temps {
		if strings.HasPrefix(k, "CPU") {
			hot = math.Max(hot, v)
		}
	}
	return hot
}

func inletOf(temps map[string]float64) float64 {
	if v, ok := temps["Inlet"]; ok {
		return v
	}
	return 25
}

type decision struct {
	target              float64
	ok                  bool // false = hand the fans to the iDRAC
	why                 string
	d                   demand
	modelPart, warmPart float64
	modelRan            bool
	exhExpected         float64 // EXHAUST_MODEL's expected exhaust, 0 = not used this cycle
}

func decide(temps map[string]float64, hdd map[Drive]float64, watts float64, model bool, modelI float64) decision {
	var r decision
	cpu, exhaust, disk := hottestCPU(temps), temps["Exhaust"], 0.0
	for _, v := range hdd {
		disk = math.Max(disk, v)
	}
	cpuDemand := interp(cpuCurve, cpu)
	if model && watts > 0 { // feedforward from power + integral, never below the warm floor
		r.modelPart, r.warmPart, r.modelRan = modelPct(watts, inletOf(temps))+modelI, interp(warmCurve, cpu), true
		cpuDemand = math.Max(r.modelPart, r.warmPart)
	}
	r.d = demand{cpu: math.Max(minPct, cpuDemand), hdd: interp(hddCurve, disk), exhaust: interp(exhaustCurve, exhaust)}
	if disk > 0 { // without a drive reading: the absolute curve
		in := inletOf(temps)
		r.exhExpected = in + exhaustModel[0]*(cpu-in) + exhaustModel[1]*(disk-in)
		r.d.exhaust = interp(exhaustExcessCurve, exhaust-r.exhExpected)
	}
	switch {
	case cpu >= cpuAuto:
		r.why = fmt.Sprintf("cpu %.0fC >= %.0f", cpu, cpuAuto)
	case disk >= hddAuto:
		r.why = fmt.Sprintf("hdd %.0fC >= %.0f", disk, hddAuto)
	case exhaust >= exhaustAuto:
		r.why = fmt.Sprintf("exhaust %.0fC >= %.0f", exhaust, exhaustAuto)
	default:
		src, v := "cpu", r.d.cpu
		if r.d.hdd > v {
			src, v = "hdd", r.d.hdd
		}
		if r.d.exhaust > v {
			src, v = "exhaust", r.d.exhaust
		}
		kind := "curve"
		if model {
			kind = "model"
		}
		r.target, r.ok, r.why = v, true, fmt.Sprintf("%s (%s)", kind, src)
	}
	return r
}

// ---------- RAPL ----------

type raplSample struct {
	t time.Time
	e int64
}

var raplPrev = map[string]raplSample{}

// raplWatts: CPU package power from the RAPL energy counters (reacts within a cycle, unlike the
// iDRAC's lagging wall-power reading), summed over sockets.
func raplWatts() (float64, bool) {
	now, total, ok := time.Now(), 0.0, false
	for _, dom := range []string{"intel-rapl:0", "intel-rapl:1"} {
		base := "/sys/class/powercap/" + dom
		e, err1 := readInt(base + "/energy_uj")
		rng, err2 := readInt(base + "/max_energy_range_uj")
		if err1 != nil || err2 != nil {
			continue
		}
		prev, had := raplPrev[dom]
		raplPrev[dom] = raplSample{now, e}
		if had && now.After(prev.t) {
			de := e - prev.e
			if de < 0 {
				de += rng // counter wrap
			}
			total += float64(de) / 1e6 / now.Sub(prev.t).Seconds()
			ok = true
		}
	}
	return total, ok
}

func readInt(p string) (int64, error) {
	b, err := os.ReadFile(p)
	if err != nil {
		return 0, err
	}
	return strconv.ParseInt(strings.TrimSpace(string(b)), 10, 64)
}

// ---------- control loop ----------

type controller struct {
	bmc      *BMC
	sensors  []Sensor
	expect   map[string]bool // temperature sensors seen on the first good read
	fansSeen map[string]bool // fans seen on the first good read
	kick     chan struct{}   // ask the drive goroutine for an early re-read
	reload   chan func()     // validated settings to apply before the next cycle

	current         float64 // last duty sent; valid when manual
	manual          bool
	sentAt          time.Time
	modelI, capCorr float64
	lastCap         float64 // valid when capSet
	capSet          bool
	hotHist         []float64 // hottest CPU, last 60 s
	raplOff         float64   // learned wall - RAPL offset (W); valid when raplLearned
	raplLearned     bool
}

func (c *controller) readBMC() (temps, fans map[string]float64, watts float64, err error) {
	if c.sensors == nil {
		if c.sensors, err = c.bmc.Sensors(); err != nil {
			c.sensors = nil
			return nil, nil, 0, fmt.Errorf("sensor repository: %w", err)
		}
		log.Printf("sensor repository: %d temperature/fan/power sensors", len(c.sensors))
	}
	temps, fans = map[string]float64{}, map[string]float64{}
	for _, s := range c.sensors {
		v, ok, err := c.bmc.Reading(s)
		if err != nil && s.Type != sensorCurrent { // a failed power read is tolerated
			return nil, nil, 0, err
		}
		if !ok {
			continue
		}
		switch s.Type {
		case sensorTemp:
			if s.Entity == 3 { // entity 3.x = processor x
				temps[fmt.Sprintf("CPU%d", s.Inst)] = v
			} else {
				temps[strings.ReplaceAll(strings.ReplaceAll(s.Name, " Temp", ""), " ", "_")] = v
			}
		case sensorFan:
			fans[strings.ReplaceAll(s.Name, " RPM", "")] = v
		case sensorCurrent:
			watts = v
		}
	}
	if len(temps) == 0 || len(fans) == 0 {
		c.sensors = nil // walk the repository again next time
		return nil, nil, 0, fmt.Errorf("empty sensor read")
	}
	// a temperature that drops out must not look like a cool part, and a garbage value must not
	// either: both fail the cycle (-> iDRAC)
	if c.expect == nil {
		if hottestCPU(temps) == 0 {
			return nil, nil, 0, fmt.Errorf("no CPU temperature")
		}
		c.expect = map[string]bool{}
		for k := range temps {
			c.expect[k] = true
		}
		log.Printf("temperature sensors: %s", fmtTemps(temps))
	}
	for k := range c.expect {
		if _, ok := temps[k]; !ok {
			return nil, nil, 0, fmt.Errorf("temperature %s stopped reporting", k)
		}
	}
	for _, k := range requiredTemps {
		if _, ok := temps[k]; !ok {
			return nil, nil, 0, fmt.Errorf("required temperature %s has no reading", k)
		}
	}
	// a dead, stalled or vanished fan: the iDRAC compensates with the others, we can't
	if c.fansSeen == nil {
		c.fansSeen = map[string]bool{}
		for k := range fans {
			c.fansSeen[k] = true
		}
	}
	for k := range c.fansSeen {
		if v, ok := fans[k]; !ok || v < fanMinRPM {
			return nil, nil, 0, fmt.Errorf("fan %s at %g rpm (missing or below FAN_MIN_RPM)", k, v)
		}
	}
	for k, v := range temps {
		if !(v > 0 && v < 127) {
			return nil, nil, 0, fmt.Errorf("implausible %s reading %g C", k, v)
		}
	}
	return temps, fans, watts, nil
}

func (c *controller) setAuto(why string) error {
	err := c.bmc.FanAuto()
	st.Lock()
	if err == nil {
		st.auto, st.duty = 1, -1
	}
	st.reason = why
	st.Unlock()
	return err
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

func (c *controller) step() error {
	temps, fans, watts, err := c.readBMC()
	if err != nil {
		return err
	}
	hdds.Lock()
	ids, hdd, hddAt, scanned := hdds.ids, hdds.temps, hdds.at, hdds.scanned
	hdds.Unlock()
	st.Lock()
	st.temps, st.fans, st.hdd, st.watts, st.lastOK = temps, fans, hdd, watts, float64(time.Now().UnixMilli())/1000
	mode, fixedPct, reason, capNow := st.mode, st.fixed, st.reason, st.powerCap
	st.Unlock()

	if mode != "model" {
		c.capSet = false
		if capNow != 0 && capNow != int(pcapMax) { // left model mode: don't leave our cap behind
			if err := c.bmc.SetPowerCap(int(pcapMax)); err != nil {
				return err
			}
			st.Lock()
			st.powerCap = int(pcapMax)
			st.Unlock()
			log.Printf("power cap lifted (%d W, mode=%s)", int(pcapMax), mode)
		}
	}
	var r decision
	ivl := interval.Seconds()
	switch {
	case mode == "auto":
		r.why = "mode=auto"
	case !scanned:
		r.why = "scanning drives"
	case len(ids) < hddMin:
		r.why = fmt.Sprintf("only %d drives found (HDD_MIN %d)", len(ids), hddMin)
	case time.Since(hddAt) > hddInterval+2*time.Minute:
		r.why = "hdd temps stale"
	case len(hdd) < len(ids): // can't see every drive -> don't guess
		r.why = fmt.Sprintf("hdd temps missing (%d/%d)", len(hdd), len(ids))
		select {
		case c.kick <- struct{}{}: // retry the drive read now
		default:
		}
	case mode == "fixed":
		r = decide(temps, hdd, 0, false, 0) // still honor the safety limits
		if r.ok {
			r.target, r.why = float64(fixedPct), "fixed"
		}
	case mode == "model":
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
		if r.ok {
			d, hot := r.d, hottestCPU(temps)
			over := hot - cpuTarget
			pinned := fanMax > 0 && d.cpu >= fanMax
			cpuEff := d.cpu
			if fanMax > 0 {
				cpuEff = math.Min(d.cpu, fanMax)
				if pinned { // CPU wants more than FAN_MAX: hold the fans there, power does the rest
					r.target = math.Max(fanMax, math.Max(d.hdd, d.exhaust))
					r.why = fmt.Sprintf("model (fan ceiling %.0f%%, power capped)", fanMax)
				}
				// cap: most power that settles at CPU_TARGET with fans at FAN_MAX, plus a correction
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
				if !c.capSet || math.Abs(want-c.lastCap) >= pcapStep || (want == pcapMax) != (c.lastCap == pcapMax) {
					if math.IsNaN(want) || math.IsInf(want, 0) {
						return fmt.Errorf("power cap computed as %v", want)
					}
					w := int(math.Round(want))
					if err := c.bmc.SetPowerCap(w); err != nil {
						return err
					}
					c.lastCap, c.capSet = float64(w), true
					st.Lock()
					st.powerCap = w
					st.Unlock()
					log.Printf("power cap %d W (inlet %gC, correction %.0f W)", w, inletOf(temps), c.capCorr)
				}
			}
			emergency := hot >= cpuLimit
			if emergency { // past the hard line: no waiting
				r.target, r.why = math.Max(r.target, emergencyPct), fmt.Sprintf("cpu %.0fC >= limit %.0f", hot, cpuLimit)
			} else if c.manual && r.target < c.current { // step up at once, down <= 2%/cycle
				r.target = math.Max(r.target, c.current-2)
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
			// ... and only while the model's own request (not the warm floor) sets the CPU demand
			onCPU := !emergency && r.modelRan && r.modelPart >= r.warmPart && math.Abs(r.target-cpuEff) < 0.5 &&
				cpuEff >= math.Max(d.hdd, d.exhaust)
			if settled && onCPU && math.Abs(over) < 5 && !(d.cpu <= minPct && over < 0) && !(pinned && over > 0) {
				c.modelI = math.Max(-iMax, math.Min(iMax, c.modelI+ki*over*ivl/60))
			}
			st.Lock()
			st.modelI, st.capCorr, st.modelOn = c.modelI, c.capCorr, true
			st.Unlock()
		}
	default:
		r = decide(temps, hdd, 0, false, 0)
		if r.ok && c.manual && r.target < c.current { // step up immediately, down at most 2%/cycle
			r.target = math.Max(r.target, c.current-2)
		}
	}
	if hot := hottestCPU(temps); r.ok && hot >= cpuLimit && r.target < emergencyPct { // every mode, incl. fixed
		r.target, r.why = emergencyPct, fmt.Sprintf("cpu %.0fC >= limit %.0f", hot, cpuLimit)
	}
	if r.ok || r.d != (demand{}) {
		st.Lock()
		st.demand, st.hasDemand, st.exhExpected = r.d, true, r.exhExpected
		st.Unlock()
	}

	if !r.ok {
		// sent every cycle, not only on the transition: an earlier hand-back may have failed and left
		// the BMC in manual at a low duty
		if err := c.setAuto(r.why); err != nil {
			return err
		}
		if c.manual || reason != r.why {
			log.Printf("auto: %s", r.why)
		}
		c.manual, c.hotHist = false, nil
	} else if !c.manual || math.Round(r.target) != math.Round(c.current) || time.Since(c.sentAt) >= reassert {
		// re-sent periodically: retakes the fans after an iDRAC reset
		if math.IsNaN(r.target) || math.IsInf(r.target, 0) { // int(NaN) is undefined: could mean 0 %
			return fmt.Errorf("duty computed as %v", r.target)
		}
		pct := max(0, min(100, int(math.Round(r.target))))
		if err := c.bmc.FanDuty(pct); err != nil {
			return err
		}
		if !c.manual || math.Round(r.target) != math.Round(c.current) {
			vals := make([]string, 0, len(hdd))
			for _, v := range hdd {
				vals = append(vals, fmt.Sprintf("%g", v))
			}
			sort.Strings(vals)
			log.Printf("duty %d%% (%s) %s hdd=[%s]", int(math.Round(r.target)), r.why, fmtTemps(temps), strings.Join(vals, " "))
		}
		c.current, c.manual, c.sentAt = r.target, true, time.Now()
		st.Lock()
		st.auto, st.duty, st.reason = 0, pct, "manual"
		st.Unlock()
	}
	return nil
}

// safeStep runs one cycle; a panic (a bug, a reply the parsers didn't expect) counts as a failed
// cycle like any other error instead of killing the process.
func (c *controller) safeStep() (err error) {
	defer func() {
		if p := recover(); p != nil {
			err = fmt.Errorf("panic: %v\n%s", p, debug.Stack())
		}
	}()
	return c.step()
}

func (c *controller) run() {
	sdNotify("READY=1")
	for {
		select {
		case apply := <-c.reload:
			apply()
			c.manual = false // re-send the duty under the new settings
		default:
		}
		if err := c.safeStep(); err != nil { // any failure -> safe mode
			st.Lock()
			st.errors++
			st.Unlock()
			log.Printf("error: %v -> auto", err)
			_ = c.setAuto("error")
			c.manual = false
		}
		sdNotify("WATCHDOG=1")
		time.Sleep(interval)
	}
}

// hddWorker runs hddLoop and restarts it after a panic; meanwhile the drive data goes stale and
// the control loop hands the fans to the iDRAC.
func hddWorker(kick <-chan struct{}) {
	for {
		p, err := OpenPerc()
		if err != nil { // no controller (yet): publish "no drives" (HDD_MIN decides) and retry
			log.Printf("drives: %v (retrying in 60 s)", err)
			hdds.Lock()
			hdds.ids, hdds.temps, hdds.at, hdds.scanned = nil, map[Drive]float64{}, time.Now(), true
			hdds.Unlock()
			time.Sleep(time.Minute)
			continue
		}
		func() {
			defer func() {
				if r := recover(); r != nil {
					log.Printf("drive reader panic: %v\n%s", r, debug.Stack())
				}
			}()
			hddLoop(p, kick)
		}()
		time.Sleep(interval)
	}
}

// hddLoop keeps hdds fresh: every HDD_INTERVAL (or when kicked), rescanning every HDD_RESCAN.
func hddLoop(p *Perc, kick <-chan struct{}) {
	var ids []Drive
	var scanAt time.Time
	shrink := false
	for {
		if time.Since(scanAt) >= hddRescan {
			found := p.Scan()
			switch {
			case scanAt.IsZero():
				log.Printf("drives: %v", found)
				ids = found
			case fmt.Sprint(found) == fmt.Sprint(ids):
				shrink = false
			case len(found) < len(ids) && !shrink:
				// a drive missing from one scan may be a controller hiccup: keep expecting it (its
				// missing temperature keeps the iDRAC in charge) until the next scan agrees
				log.Printf("drives missing from scan: %v -> %v (confirming on the next scan)", ids, found)
				shrink = true
			default:
				log.Printf("drives changed: %v -> %v", ids, found)
				ids, shrink = found, false
			}
			scanAt = time.Now()
		}
		temps := map[Drive]float64{}
		for _, d := range ids {
			if t, ok := p.Temp(d); ok {
				temps[d] = t
			}
		}
		hdds.Lock()
		hdds.ids, hdds.temps, hdds.at, hdds.scanned = ids, temps, time.Now(), true
		hdds.Unlock()
		select {
		case <-time.After(hddInterval):
		case <-kick:
		}
	}
}

// ---------- systemd ----------

func sdNotify(msg string) {
	sock := os.Getenv("NOTIFY_SOCKET")
	if sock == "" {
		return
	}
	conn, err := net.DialUnix("unixgram", nil, &net.UnixAddr{Name: sock, Net: "unixgram"})
	if err != nil {
		return
	}
	defer conn.Close()
	_, _ = conn.Write([]byte(msg))
}

// ---------- HTTP: /metrics, /control ----------

func num(v float64) string { return strconv.FormatFloat(v, 'f', -1, 64) }

var labelEscaper = strings.NewReplacer(`\`, `\\`, `"`, `\"`, "\n", `\n`)

func lbl(s string) string { return labelEscaper.Replace(s) } // Prometheus label value escaping

func metrics(w http.ResponseWriter, _ *http.Request) {
	st.Lock()
	defer st.Unlock()
	var b strings.Builder
	line := func(f string, a ...any) { fmt.Fprintf(&b, f+"\n", a...) }
	sorted := func(m map[string]float64) []string {
		k := make([]string, 0, len(m))
		for x := range m {
			k = append(k, x)
		}
		sort.Strings(k)
		return k
	}
	line("# TYPE fanctl_temperature_celsius gauge")
	for _, k := range sorted(st.temps) {
		line(`fanctl_temperature_celsius{sensor="%s"} %s`, lbl(k), num(st.temps[k]))
	}
	line("# TYPE fanctl_fan_rpm gauge")
	for _, k := range sorted(st.fans) {
		line(`fanctl_fan_rpm{fan="%s"} %s`, lbl(k), num(st.fans[k]))
	}
	line("# TYPE fanctl_hdd_temperature_celsius gauge")
	for d, v := range st.hdd {
		line(`fanctl_hdd_temperature_celsius{id="%d",serial="%s"} %s`, d.ID, lbl(d.Serial), num(v))
	}
	if st.powerCap > 0 {
		line("# TYPE fanctl_power_cap_watts gauge\nfanctl_power_cap_watts %d", st.powerCap)
		line("# TYPE fanctl_power_cap_correction_watts gauge\nfanctl_power_cap_correction_watts %.1f", st.capCorr)
	}
	if st.ffWatts > 0 {
		line("# TYPE fanctl_feedforward_watts gauge\nfanctl_feedforward_watts %.0f", st.ffWatts)
	}
	if st.modelOn {
		line("# TYPE fanctl_model_correction_percent gauge\nfanctl_model_correction_percent %.2f", st.modelI)
		line("# TYPE fanctl_cpu_target_celsius gauge\nfanctl_cpu_target_celsius %s", num(cpuTarget))
	}
	line("# TYPE fanctl_demand_percent gauge")
	if st.hasDemand {
		line(`fanctl_demand_percent{source="cpu"} %.1f`, st.demand.cpu)
		line(`fanctl_demand_percent{source="hdd"} %.1f`, st.demand.hdd)
		line(`fanctl_demand_percent{source="exhaust"} %.1f`, st.demand.exhaust)
	}
	if st.exhExpected > 0 {
		line("# TYPE fanctl_exhaust_expected_celsius gauge\nfanctl_exhaust_expected_celsius %.2f", st.exhExpected)
		if v, ok := st.temps["Exhaust"]; ok {
			line("# TYPE fanctl_exhaust_excess_celsius gauge\nfanctl_exhaust_excess_celsius %.2f", v-st.exhExpected)
		}
	}
	if st.watts > 0 {
		line("# TYPE fanctl_power_watts gauge\nfanctl_power_watts %s", num(st.watts))
	}
	line("# TYPE fanctl_duty_percent gauge\nfanctl_duty_percent %d", st.duty)
	line("# TYPE fanctl_auto_mode gauge\nfanctl_auto_mode %d", st.auto)
	line("# TYPE fanctl_errors_total counter\nfanctl_errors_total %d", st.errors)
	line("# TYPE fanctl_last_read_timestamp_seconds gauge\nfanctl_last_read_timestamp_seconds %.3f", st.lastOK)
	line(`fanctl_info{mode="%s",reason="%s"} 1`, lbl(st.mode), lbl(st.reason))
	line("# TYPE fanctl_fixed_percent gauge\nfanctl_fixed_percent %d", st.fixed)
	w.Header().Set("Content-Type", "text/plain; version=0.0.4")
	_, _ = w.Write([]byte(b.String()))
}

// control: /control?token=..&mode=curve|auto|fixed|model[&pct=N] - switch mode at runtime.
func control(w http.ResponseWriter, r *http.Request) {
	q := r.URL.Query()
	if controlToken == "" || subtle.ConstantTimeCompare([]byte(q.Get("token")), []byte(controlToken)) != 1 {
		http.Error(w, "forbidden", http.StatusForbidden)
		return
	}
	mode := q.Get("mode")
	switch mode {
	case "curve", "auto", "fixed", "model":
	default:
		http.Error(w, "mode must be curve, auto, fixed or model", http.StatusBadRequest)
		return
	}
	st.Lock()
	defer st.Unlock()
	if mode == "fixed" {
		pct, err := strconv.Atoi(q.Get("pct"))
		if err != nil || pct < 5 || pct > 100 {
			http.Error(w, "fixed needs pct=5..100", http.StatusBadRequest)
			return
		}
		st.fixed = pct
	}
	st.mode = mode
	log.Printf("control: mode=%s fixed=%d", mode, st.fixed)
	fmt.Fprintf(w, "ok mode=%s fixed=%d\n", mode, st.fixed)
}

// serve listens on addr, retrying until the address exists (the LAN address can come up after us).
func serve(addr string, h http.Handler) {
	for {
		srv := &http.Server{Addr: addr, Handler: h, ReadHeaderTimeout: 5 * time.Second,
			ReadTimeout: 10 * time.Second, WriteTimeout: 10 * time.Second, MaxHeaderBytes: 8 << 10}
		err := srv.ListenAndServe()
		log.Printf("http %s: %v (retrying)", addr, err)
		time.Sleep(10 * time.Second)
	}
}

// ---------- subcommands ----------

// fansToIDRAC hands the fans back, retrying through a busy or briefly unresponsive BMC. Bounded so
// it finishes within systemd's TimeoutStopSec (20 s) even when every attempt times out (5 s).
func fansToIDRAC(bmc *BMC) error {
	deadline := time.Now().Add(12 * time.Second)
	for {
		err := bmc.FanAuto()
		if err == nil || time.Now().After(deadline) {
			return err
		}
		log.Printf("fans -> iDRAC auto: %v (retrying)", err)
		time.Sleep(time.Second)
	}
}

func cmdAuto() {
	bmc, err := OpenBMC()
	for deadline := time.Now().Add(30 * time.Second); err != nil && time.Now().Before(deadline); { // early boot
		time.Sleep(time.Second)
		bmc, err = OpenBMC()
	}
	if err == nil {
		err = fansToIDRAC(bmc)
	}
	if err != nil {
		log.Fatalf("fans -> iDRAC auto: %v", err)
	}
	log.Print("fans -> iDRAC automatic control")
}

// cmdPrepare creates /dev/megaraid_sas_ioctl_node: the megaraid_sas driver registers the char
// device but never creates the node (smartctl and storcli mknod it themselves). Runs as root
// before the sandboxed service (ExecStartPre=+).
func cmdPrepare() {
	const node = "/dev/megaraid_sas_ioctl_node"
	if _, err := os.Stat(node); err == nil {
		return
	}
	b, err := os.ReadFile("/proc/devices")
	if err != nil {
		log.Fatal(err)
	}
	for _, line := range strings.Split(string(b), "\n") {
		if f := strings.Fields(line); len(f) == 2 && f[1] == "megaraid_sas_ioctl" {
			major, err := strconv.Atoi(f[0])
			if err == nil {
				err = syscall.Mknod(node, syscall.S_IFCHR|0o600, major<<8)
			}
			if err != nil {
				log.Fatalf("create %s: %v", node, err)
			}
			log.Printf("created %s (major %d)", node, major)
			return
		}
	}
	log.Print("no megaraid_sas controller: no drive temperatures")
}

func cmdProbe() {
	bmc, err := OpenBMC()
	if err != nil {
		log.Fatal(err)
	}
	sensors, err := bmc.Sensors()
	if err != nil {
		log.Fatalf("sensor repository: %v", err)
	}
	fmt.Println("== BMC sensors")
	for _, s := range sensors {
		v, ok, err := bmc.Reading(s)
		val := "no reading"
		if err != nil {
			val = "error: " + err.Error()
		} else if ok {
			val = num(math.Round(v*100) / 100)
		}
		fmt.Printf("  %-18s num=0x%02x type=0x%02x entity=%d.%d  %s\n", s.Name, s.Num, s.Type, s.Entity, s.Inst, val)
	}
	if pl, err := bmc.GetPowerLimit(); err != nil {
		fmt.Println("== DCMI power limit: error:", err)
	} else {
		fmt.Printf("== DCMI power limit: %d W, active=%v, raw % x\n", pl.Watts, pl.Active, pl.raw)
	}
	raplWatts()
	time.Sleep(time.Second)
	if w, ok := raplWatts(); ok {
		fmt.Printf("== RAPL package power: %.1f W\n", w)
	} else {
		fmt.Println("== RAPL: unavailable")
	}
	p, err := OpenPerc()
	if err != nil {
		fmt.Println("== drives: error:", err)
		return
	}
	fmt.Printf("== drives (megaraid host %d)\n", p.host)
	for _, d := range p.Scan() {
		t, _ := p.Temp(d)
		fmt.Printf("  megaraid,%-3d %-22s ata=%-5v %g C\n", d.ID, d.Serial, d.ata, t)
	}
}

func main() {
	log.SetFlags(0) // journald adds timestamps
	cmd := "run"
	if len(os.Args) > 1 {
		cmd = os.Args[1]
	}
	switch cmd {
	case "auto":
		cmdAuto()
		return
	case "probe":
		cmdProbe()
		return
	case "prepare":
		cmdPrepare()
		return
	case "check": // validate the settings in the environment (used by reload); exits 1 if refused
		loadConfig()
		fmt.Println("config ok")
		return
	case "run":
	default:
		fmt.Fprintln(os.Stderr, "usage: fanctl [run|auto|probe|prepare|check]")
		os.Exit(2)
	}

	loadConfig()
	st.mode, st.fixed = mode0, fixedPct0
	fileKeys, _ = readEnvFile(envFilePath()) // systemd already loaded it; remember which keys it set
	bmc, err := OpenBMC()
	if err != nil {
		log.Fatal(err)
	}
	c := &controller{bmc: bmc, kick: make(chan struct{}, 1), reload: make(chan func(), 1)}

	hup := make(chan os.Signal, 1)
	signal.Notify(hup, syscall.SIGHUP)
	go func() {
		for range hup {
			apply, err := prepareReload()
			if err != nil {
				log.Printf("reload: %v (keeping the current settings)", err)
				continue
			}
			select {
			case c.reload <- apply:
			default:
				log.Print("reload: one already pending")
			}
		}
	}()

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGTERM, syscall.SIGINT)
	go func() {
		<-sig
		err := fansToIDRAC(bmc)
		bmc.mu.Lock() // held until exit: the control loop can't send another duty after the hand-back
		if err != nil {
			log.Printf("shutdown: fans -> iDRAC auto failed: %v", err)
			os.Exit(1)
		}
		log.Print("restored automatic fan control")
		os.Exit(0)
	}()

	go hddWorker(c.kick)

	mux := http.NewServeMux()
	mux.HandleFunc("/metrics", metrics)
	mux.HandleFunc("/control", control)
	for _, a := range strings.Split(listen, ",") {
		if a = strings.TrimSpace(a); a != "" {
			go serve(a, mux)
		}
	}
	log.Printf("mode=%s interval=%v listen=%s", mode0, interval, listen)
	c.run()
}
