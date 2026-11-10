//go:build linux

// fanctl: Dell R730xd fan controller + Prometheus exporter.
//
// Reads CPU temperatures from the kernel (coretemp), inlet/exhaust temps, fan RPM and wall power from the
// local BMC (/dev/ipmi0), and drive temps through the PERC (megaraid passthrough). Each cycle classifies
// every input (ok / unknown / danger) and the worst one picks the controller: model (power first via the
// iDRAC cap, then fans), fixed, safe (unknown input: fixed duty, CPUs at minimum power), shutdown
// (danger: orderly poweroff) or idrac. See control.go.
//
// Settings: /etc/fanctl.toml (config.go). Under systemd (Type=notify, WatchdogSec) a hung control loop
// is killed, and ExecStopPost runs `fanctl auto`: fans to the iDRAC, CPU clamp undone.
//
//	fanctl [run]          control loop
//	fanctl auto           hand the fans back to the iDRAC and exit
//	fanctl probe          print every reading fanctl uses; changes nothing
//	fanctl prepare        create the PERC ioctl node if missing (run as root before the service)
//	fanctl check [file|-] validate a config file; exit 1 if refused
package main

import (
	"crypto/subtle"
	"fmt"
	"log"
	"math"
	"net"
	"net/http"
	"os"
	"os/signal"
	"runtime/debug"
	"sort"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"
)

// ---------- settings (loaded from /etc/fanctl.toml: config.go) ----------

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
	cpuAuto, hddAuto, exhaustAuto                          float64 // danger: at or above -> orderly shutdown
	fixedCPUMax, fixedHDDMax, fixedExhaustMax              float64 // fixed mode hands over to model at or above
	interval, hddInterval, hddRescan, reassert             time.Duration
	unknownAfter                                           time.Duration // past due -> unknown
	safeDuty, safeCap                                      float64       // the safe controller
	safeClamp                                              bool
	cpuCurve, hddCurve, exhaustCurve, warmCurve, fanRPMTab curve
	cpuModels                                              []cpuModel
	exhaustModel                                           []float64 // a, b (model.exhaust_model)
	exhaustExcessCurve                                     curve
)

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
	check(interval >= time.Second && interval <= 10*time.Second, "inputs.interval must be 1-10 s (the systemd watchdog is 60 s)")
	check(hddInterval >= 10*time.Second && hddRescan >= hddInterval, "need inputs.hdd_interval >= 10 s and inputs.hdd_rescan >= inputs.hdd_interval")
	check(reassert >= interval, "inputs.reassert must be >= inputs.interval")
	check(minPct >= 1 && minPct <= 100, "model.min_duty must be 1-100")
	check(fixedPct0 >= 0 && fixedPct0 <= 100, "fixed.duty must be 0-100")
	check(fin(fixedCPUMax, fixedHDDMax, fixedExhaustMax) && fixedCPUMax < cpuAuto && fixedHDDMax < hddAuto && fixedExhaustMax < exhaustAuto,
		"fixed.cpu_max / hdd_max / exhaust_max must be finite and below shutdown.cpu / hdd / exhaust")
	check(fanMax == 0 || (fanMax >= minPct && fanMax <= 100), "model.fan_max must be 0 or model.min_duty-100")
	check(emergencyPct >= minPct && emergencyPct <= modelCeiling, "model.emergency_duty must be model.min_duty-%d", modelCeiling)
	check(cpuTarget-modelMargin > 20 && cpuTarget < cpuLimit && cpuLimit < cpuAuto && cpuAuto <= 100,
		"need 20 < model.cpu_target - model.margin, cpu_target < model.cpu_limit < shutdown.cpu <= 100")
	check(hddAuto > 0 && hddAuto <= 70 && exhaustAuto > 0 && exhaustAuto <= 80, "shutdown.hdd must be <= 70, shutdown.exhaust <= 80")
	check(pcapMin >= 100 && pcapMin < pcapMax && pcapMax <= 2000 && pcapMax == math.Trunc(pcapMax) && pcapStep > 0,
		"need 100 <= power.cap_min < power.cap_max <= 2000, cap_max a whole number, power.cap_step > 0")
	check(fin(safeDuty, safeCap) && safeDuty >= minPct && safeDuty <= 100 && safeCap >= pcapMin && safeCap <= pcapMax,
		"safe.duty must be model.min_duty-100, safe.cap power.cap_min-power.cap_max")
	check(unknownAfter >= interval && unknownAfter <= time.Minute, "inputs.unknown_after must be inputs.interval-60 s")
	check(len(cpuModels) >= 1 && len(fanRPMTab) >= 2, "model.cpu_models needs >= 1 entry, model.fan_rpm_table >= 2 points")
	check(hddMin >= 0 && fanMinRPM >= 0, "inputs.hdd_min and inputs.fan_min_rpm must be >= 0")
	check(ki >= 0 && iMax >= 0 && kw >= 0, "model.ki, model.i_max, power.kw must be >= 0")
	check(len(exhaustModel) == 2 && fin(exhaustModel...) && exhaustModel[0] >= 0 && exhaustModel[1] >= 0 && exhaustModel[0]+exhaustModel[1] < 1,
		"model.exhaust_model: need [a, b] >= 0 with a+b < 1 (the exhaust is a mix of inlet air and air warmed by the parts)")
	for name, c := range map[string]curve{"model.cpu_curve": cpuCurve, "model.hdd_curve": hddCurve, "model.exhaust_curve": exhaustCurve, "model.warm_curve": warmCurve, "model.exhaust_excess_curve": exhaustExcessCurve} {
		for _, p := range c {
			check(fin(p.x) && pct(p.y), "%s: bad point %g:%g (duty must be 0-100)", name, p.x, p.y)
		}
	}
	for i, p := range fanRPMTab {
		check(pct(p.x) && fin(p.y) && p.y > 0 && (i == 0 || (p.x > fanRPMTab[i-1].x && p.y > fanRPMTab[i-1].y)),
			"model.fan_rpm_table must rise in both duty and rpm")
	}
	for i, m := range cpuModels {
		check(fin(m.pb, m.r0, m.k, m.n, m.scale) && m.pb >= 0 && m.pb < 1000 && m.r0 >= 0 && m.k > 0 && m.n > 0 && m.n < 10 && m.scale > 0,
			"model.cpu_models[%d]: need finite pb (0-1000), r0 >= 0, k > 0, n (0-10), scale > 0", i)
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
	exhExpected        float64            // 0 = not computed (no model or no drive temperature)
	ages               map[string]float64 // seconds since each expected input last read valid
	state              string             // the controller that ran last
	clamped            bool
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

func interp(c curve, v float64) float64 { // model.min_duty below the first point, last y above the last
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

// rpmToPct inverts model.fan_rpm_table (duty:rpm), linear between points, extrapolated past the ends.
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

// modelPct: lowest duty at which every CPU settles at model.cpu_target for this power and inlet, or 100.
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

// modelMaxWatts: most wall power at which every CPU settles at model.cpu_target with the fans at pct.
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
	exhExpected         float64 // model.exhaust_model's expected exhaust, 0 = not used this cycle
}

func decide(temps, hdd map[string]float64, watts float64, model bool, modelI float64) decision {
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

// hddWorker runs hddLoop and restarts it after a panic; meanwhile the drive data goes stale and
// the control loop hands the fans to the iDRAC.
func hddWorker(kick <-chan struct{}) {
	for {
		p, err := OpenPerc()
		if err != nil { // no controller (yet): publish "no drives" (inputs.hdd_min decides) and retry
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

// hddLoop keeps hdds fresh: every inputs.hdd_interval (or when kicked), rescanning every inputs.hdd_rescan.
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
	b01 := map[bool]int{false: 0, true: 1}
	line("# TYPE fanctl_controller gauge")
	for _, k := range []string{"model", "fixed", "safe", "shutdown", "idrac"} {
		line(`fanctl_controller{controller="%s"} %d`, k, b01[st.state == k])
	}
	line("# TYPE fanctl_cpu_clamped gauge\nfanctl_cpu_clamped %d", b01[st.clamped])
	line("# TYPE fanctl_input_age_seconds gauge")
	ak := make([]string, 0, len(st.ages))
	for k := range st.ages {
		ak = append(ak, k)
	}
	sort.Strings(ak)
	for _, k := range ak {
		line(`fanctl_input_age_seconds{input="%s"} %.0f`, lbl(k), st.ages[k])
	}
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
	if mode == "auto" {
		mode = "idrac"
	}
	switch mode {
	case "model", "fixed", "idrac":
	default:
		http.Error(w, "mode must be model, fixed or idrac", http.StatusBadRequest)
		return
	}
	st.Lock()
	defer st.Unlock()
	if mode == "fixed" {
		pct, err := strconv.Atoi(q.Get("pct"))
		if err != nil || pct < 0 || pct > 100 {
			http.Error(w, "fixed needs pct=0..100", http.StatusBadRequest)
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

// handoffFile, if fresh, tells the next "fanctl auto" (ExecStopPost) that a new fanctl is about to take over:
// leave the fans at their duty instead of handing them to the iDRAC (the fans would spin up for a few
// seconds). It's honoured once and only within handoffMaxAge, so a later crash still hands back.
const handoffFile, handoffMaxAge = "/run/fanctl-handoff", time.Minute

// handoffFresh: a new fanctl is about to take over (handoffFile younger than handoffMaxAge).
func handoffFresh() (bool, time.Duration) {
	fi, err := os.Stat(handoffFile)
	if err != nil {
		return false, 0
	}
	age := time.Since(fi.ModTime())
	return age >= 0 && age < handoffMaxAge, age
}

// cmdAuto: no controller from here on. CPU clamp undone, fans to the iDRAC (unless a restart is
// handing over), power cap lifted to power.cap_max. Reads nothing else: a bad config file must never
// block the hand-back.
func cmdAuto() {
	if restored, err := clampOff(); err != nil {
		log.Printf("%sCPU clamp restore failed: %v", prErr, err)
	} else if restored {
		log.Print("CPU clamp off")
	}
	fresh, age := handoffFresh()
	os.Remove(handoffFile) // honoured once: the last step of a stop
	if fresh {
		log.Printf("%s is %.0fs old: leaving the fans for the restarting fanctl", handoffFile, age.Seconds())
		return
	}
	bmc, err := OpenBMC()
	for deadline := time.Now().Add(30 * time.Second); err != nil && time.Now().Before(deadline); { // early boot
		time.Sleep(time.Second)
		bmc, err = OpenBMC()
	}
	if err == nil {
		err = fansToIDRAC(bmc)
		if w := capMaxForAuto(); w <= 0 {
			log.Printf("%spower cap not lifted: power.cap_max %d", prErr, w)
		} else if cerr := bmc.SetPowerCap(w); cerr != nil {
			log.Printf("%spower cap %d W failed: %v", prErr, w, cerr)
		} else {
			log.Printf("power cap %d W", w)
		}
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
		v, ok, why, err := bmc.Reading(s)
		val := "no reading (" + why + ")"
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
	case "check": // validate a config file ("-" = stdin; used by reload); exits 1 if refused
		path := configPath()
		if len(os.Args) > 2 {
			path = os.Args[2]
		}
		cmdCheck(path)
		return
	case "run":
	default:
		fmt.Fprintln(os.Stderr, "usage: fanctl [run|auto|probe|prepare|check [file|-]]")
		os.Exit(2)
	}

	loadConfig()
	controlToken = readToken()
	st.mode, st.fixed = mode0, fixedPct0
	bmc, err := OpenBMC()
	if err != nil {
		log.Fatal(err)
	}
	c := newController(bmc)
	c.loadState()

	// SIGUSR2 (systemctl kill -s USR2 fanctl): replace this process with the binary at the same path, e.g.
	// after installing a new version. Same PID, never stopped, so the fans keep their duty and stay out of
	// the iDRAC's hands; the new binary takes over on its first cycle.
	usr2 := make(chan os.Signal, 1)
	signal.Notify(usr2, syscall.SIGUSR2)
	go func() {
		for range usr2 {
			self := os.Args[0]
			if !strings.HasPrefix(self, "/") {
				if p, err := os.Executable(); err == nil {
					self = p
				}
			}
			c.reload <- func() {
				bmc.mu.Lock() // no duty is sent between here and the exec
				log.Printf("re-exec %s", self)
				err := syscall.Exec(self, os.Args, os.Environ())
				bmc.mu.Unlock()
				log.Printf("re-exec failed: %v (keeping this process)", err)
			}
		}
	}()

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
		if fresh, age := handoffFresh(); fresh { // restart: the new fanctl takes the fans over at their duty
			bmc.mu.Lock()
			log.Printf("%s is %.0fs old: leaving the fans for the restarting fanctl", handoffFile, age.Seconds())
			os.Exit(0)
		}
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
