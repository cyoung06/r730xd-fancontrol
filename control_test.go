package main

import (
	"errors"
	"os"
	"path/filepath"
	"strconv"
	"testing"
	"time"
)

// ---------- a fake BMC and a fake host, so whole cycles run in tests ----------

type fakeBMC struct {
	sensors  []Sensor
	vals     map[string]float64 // by sensorKey
	missing  map[string]bool    // answer "reading unavailable"
	readErr  error              // every Reading fails (BMC not answering)
	dutyErr  error              // FanDuty / FanAuto fail
	capErr   error              // SetPowerCap fails
	capFloor int                // SetPowerCap refuses caps below this (an iDRAC with a higher minimum)
	capPanic bool               // SetPowerCap panics (a reply the parser didn't expect)
	rpm      map[string]float64 // a fan's RPM regardless of the duty (dead fan); otherwise it follows the duty
	fan      string             // "auto" or the duty
	cap      int
	fanCalls int
}

func newFakeBMC() *fakeBMC {
	return &fakeBMC{
		sensors: []Sensor{
			{Type: sensorTemp, Entity: 3, Inst: 1, Name: "CPU1 Temp"},
			{Type: sensorTemp, Entity: 3, Inst: 2, Name: "CPU2 Temp"},
			{Type: sensorTemp, Entity: 64, Name: "Inlet Temp"},
			{Type: sensorTemp, Entity: 64, Name: "Exhaust Temp"},
			{Type: sensorFan, Name: "Fan1 RPM"},
			{Type: sensorFan, Name: "Fan2 RPM"},
			{Type: sensorCurrent, Name: "Pwr Consumption"},
		},
		vals:    map[string]float64{"CPU1": 50, "CPU2": 51, "Inlet": 25, "Exhaust": 30, "Fan1": 3000, "Fan2": 3100, "power": 200},
		missing: map[string]bool{},
		rpm:     map[string]float64{},
	}
}

func (f *fakeBMC) Sensors() ([]Sensor, error) { return f.sensors, nil }
func (f *fakeBMC) Reading(s Sensor) (float64, bool, string, error) {
	if f.readErr != nil {
		return 0, false, "", f.readErr
	}
	k := sensorKey(s)
	if f.missing[k] {
		return 0, false, "reading unavailable, status 0x60", nil
	}
	if s.Type == sensorFan {
		if v, ok := f.rpm[k]; ok {
			return v, true, "", nil
		}
		switch p, err := strconv.Atoi(f.fan); {
		case err == nil:
			return pctToRPM(float64(p)), true, "", nil
		case f.fan == "auto":
			return 5000, true, "", nil
		}
	}
	return f.vals[k], true, "", nil
}
func (f *fakeBMC) GetPowerLimit() (PowerLimit, error) {
	if f.readErr != nil {
		return PowerLimit{}, f.readErr
	}
	return PowerLimit{Active: f.cap > 0, Watts: f.cap}, nil
}
func (f *fakeBMC) FanDuty(p int) error {
	f.fanCalls++
	if f.dutyErr != nil {
		return f.dutyErr
	}
	f.fan = strconv.Itoa(p)
	return nil
}
func (f *fakeBMC) FanAuto() error {
	f.fanCalls++
	if f.dutyErr != nil {
		return f.dutyErr
	}
	f.fan = "auto"
	return nil
}
func (f *fakeBMC) SetPowerCap(w int) error {
	if f.capPanic {
		panic("unexpected DCMI reply")
	}
	if f.capErr != nil {
		return f.capErr
	}
	if w < f.capFloor {
		return errors.New("dcmi set power limit: cc 0xc9 (out of range)")
	}
	f.cap = w
	return nil
}

var t0 = time.Unix(1_800_000_000, 0)

func at(s int) time.Time { return t0.Add(time.Duration(s) * time.Second) }

// rig: settings, a fake intel_pstate and /run/fanctl in a temp dir, and a controller on a fake BMC.
func rig(t *testing.T) (*controller, *fakeBMC, func(string) string) {
	t.Helper()
	loadConfigBytes([]byte(`
[inputs]
required_temps = ["CPU1", "CPU2", "Inlet", "Exhaust"]
interval = "5s"
hdd_min = 0
`))
	st.Lock()
	st.mode, st.fixed = "model", 20
	st.Unlock()
	hdds.Lock()
	hdds.ids, hdds.temps, hdds.at, hdds.scanned = nil, nil, t0, true // a scan done at t0 (HDD_MIN 0: no drives needed)
	hdds.Unlock()

	dir := t.TempDir()
	oldP, oldT, oldS, oldH, oldM := pstateDir, throttleState, stateFile, hwmonDir, shutdownMark
	pstateDir, throttleState, stateFile, hwmonDir = filepath.Join(dir, "intel_pstate"), filepath.Join(dir, "throttle"), filepath.Join(dir, "inputs.json"), filepath.Join(dir, "hwmon")
	shutdownMark = filepath.Join(dir, "shutdown")
	t.Cleanup(func() { pstateDir, throttleState, stateFile, hwmonDir, shutdownMark = oldP, oldT, oldS, oldH, oldM })
	for i, name := range []string{"power_meter", "coretemp", "coretemp"} { // like the server: hwmon0 power_meter, 1-2 coretemp
		h := filepath.Join(hwmonDir, "hwmon"+strconv.Itoa(i))
		os.MkdirAll(h, 0o755)
		os.WriteFile(filepath.Join(h, "name"), []byte(name+"\n"), 0o644)
		if name == "coretemp" {
			os.WriteFile(filepath.Join(h, "temp1_label"), []byte("Package id "+strconv.Itoa(i-1)+"\n"), 0o644)
			os.WriteFile(filepath.Join(h, "temp2_label"), []byte("Core 0\n"), 0o644)
			os.WriteFile(filepath.Join(h, "temp2_input"), []byte("99000\n"), 0o644) // a core: must be ignored
		}
	}
	os.Mkdir(pstateDir, 0o755)
	for k, v := range map[string]string{"no_turbo": "0", "max_perf_pct": "100", "min_perf_pct": "40"} {
		os.WriteFile(filepath.Join(pstateDir, k), []byte(v+"\n"), 0o644)
	}
	get := func(k string) string { v, _ := readTrim(filepath.Join(pstateDir, k)); return v }

	f := newFakeBMC()
	c := newController(f)
	c.start = t0
	return c, f, get
}

// run: one cycle, with the fake's CPU1/CPU2 written to the fake coretemp (missing = no _input file).
func run(c *controller, f *fakeBMC, now time.Time) {
	for n := 1; n <= 2; n++ {
		p := filepath.Join(hwmonDir, "hwmon"+strconv.Itoa(n), "temp1_input")
		k := "CPU" + strconv.Itoa(n)
		if f.missing[k] {
			os.Remove(p)
		} else {
			os.WriteFile(p, []byte(strconv.Itoa(int(f.vals[k]*1000))+"\n"), 0o644)
		}
	}
	c.cycle(now)
}

// want checks what the hardware was told after a cycle.
func want(t *testing.T, s int, c *controller, f *fakeBMC, get func(string) string, fan string, capW int, clamped bool) {
	t.Helper()
	gotClamp := get("max_perf_pct") == "40" && get("no_turbo") == "1"
	if f.fan != fan || f.cap != capW || gotClamp != clamped {
		t.Fatalf("t=%ds: fan %q cap %d clamped %v; want fan %q cap %d clamped %v (reason %q)",
			s, f.fan, f.cap, gotClamp, fan, capW, clamped, c.why)
	}
}

func TestCycleNormal(t *testing.T) {
	c, f, get := rig(t)
	for s := 0; s <= 20; s += 5 {
		run(c, f, at(s))
	}
	want(t, 20, c, f, get, "10", int(pcapMax), false) // everything below the curves: MIN_PCT
	if c.state != "model" {
		t.Fatalf("controller %q", c.state)
	}
}

// The real 11-10 blip: a reading gone for two reads stands in with its last value; nothing changes.
func TestCycleBlip(t *testing.T) {
	c, f, get := rig(t)
	run(c, f, at(0))
	f.missing["Inlet"] = true
	for s := 5; s <= 10; s += 5 {
		run(c, f, at(s))
		want(t, s, c, f, get, "10", int(pcapMax), false)
	}
	f.missing["Inlet"] = false
	run(c, f, at(15))
	want(t, 15, c, f, get, "10", int(pcapMax), false)
}

// Unknown = 15 s past due: for a 5 s input, the 3rd missed read (20 s after the last good one) ->
// safe (20 %, PCAP_MIN, clamp); back -> model, clamp restored exactly, duty ramps down from 20 %.
func TestCycleUnknownToSafe(t *testing.T) {
	for _, k := range []string{"CPU2", "Inlet", "Exhaust", "Fan2"} {
		t.Run(k, func(t *testing.T) {
			c, f, get := rig(t)
			run(c, f, at(0)) // last good reading of k
			f.missing[k] = true
			for s := 5; s <= 15; s += 5 { // < 15 s past due: last value stands in
				run(c, f, at(s))
				want(t, s, c, f, get, "10", int(pcapMax), false)
			}
			for s := 20; s <= 60; s += 5 {
				run(c, f, at(s))
				want(t, s, c, f, get, "20", int(pcapMin), true)
			}
			f.missing[k] = false
			run(c, f, at(65))
			want(t, 65, c, f, get, "18", int(pcapMax), false)
			if get("no_turbo") != "0" || get("max_perf_pct") != "100" || exists(throttleState) {
				t.Fatalf("clamp not restored exactly")
			}
		})
	}
}

// The BMC stops answering (reads and writes): its inputs go unknown -> safe. Fan and cap writes fail;
// the clamp (local) still applies; the CPUs stay visible through coretemp.
func TestCycleBMCDown(t *testing.T) {
	c, f, get := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.readErr, f.dutyErr, f.capErr = errors.New("ipmi: no response in 5s"), errors.New("ipmi: no response"), errors.New("ipmi: no response")
	for s := 5; s <= 30; s += 5 {
		run(c, f, at(s))
	}
	// BMC inputs unknown since t=20 -> safe; the CPUs stay visible (coretemp), the clamp (local) applies
	if c.state != "safe" || get("max_perf_pct") != "40" || !c.in.Temps["CPU1"].Seen.Equal(at(30)) {
		t.Fatalf("BMC down: %q clamp %s CPU1 seen %v", c.state, get("max_perf_pct"), c.in.Temps["CPU1"].Seen)
	}
	// safe can't be confirmed (no fan RPM, no cap readback) for 15 s -> shutdown
	run(c, f, at(35))
	if c.state != "shutdown" || calls != 1 {
		t.Fatalf("safe unconfirmable 15 s: %q, poweroff calls %d", c.state, calls)
	}
}

// A write error is logged and retried, never an input: the readbacks say what took.
func TestCycleWriteErrors(t *testing.T) {
	c, f, get := rig(t)
	run(c, f, at(0))
	st.Lock()
	st.mode, st.fixed = "fixed", 30
	st.Unlock()
	f.capErr, f.dutyErr = errors.New("dcmi: cc 0xc1"), errors.New("fan duty: cc 0xc3")
	for s := 5; s <= 40; s += 5 {
		run(c, f, at(s))
		if c.state != "fixed" {
			t.Fatalf("t=%d: %q", s, c.state)
		}
	}
	f.capErr, f.dutyErr = nil, nil
	run(c, f, at(45))
	want(t, 45, c, f, get, "30", int(pcapMax), false)
}

// The safe controller has to be in effect by its readbacks: an iDRAC that refuses safe.cap -> 15 s
// later, the last resort.
func TestCycleSafeNotEstablished(t *testing.T) {
	c, f, get := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	f.capFloor = int(safeCap) + 10 // refuses 230 W; the cap reads back as the last one taken
	run(c, f, at(0))
	f.missing["Inlet"] = true
	for s := 5; s <= 30; s += 5 { // safe from t=20; cap still reads 586
		run(c, f, at(s))
	}
	if c.state != "safe" || calls != 0 {
		t.Fatalf("t=30: %q calls %d", c.state, calls)
	}
	run(c, f, at(35)) // 15 s in safe without its cap
	if c.state != "shutdown" || calls != 1 {
		t.Fatalf("safe not established 15 s: %q, poweroff calls %d", c.state, calls)
	}
	want(t, 35, c, f, get, "auto", int(pcapMax), true) // shutdown: the iDRAC cools; the 230 W cap is still refused
}

// A dead fan (commanded, below inputs.fan_min_rpm) is unknown -> safe.
func TestCycleDeadFan(t *testing.T) {
	c, f, get := rig(t)
	run(c, f, at(0))
	f.rpm["Fan1"] = 400
	run(c, f, at(5))
	want(t, 5, c, f, get, "20", int(pcapMin), true)
	delete(f.rpm, "Fan1")
	run(c, f, at(10))
	want(t, 10, c, f, get, "18", int(pcapMax), false)
}

// Danger (a reading at its shutdown line) needs 3 consecutive new readings: meanwhile unknown (safe,
// or idrac in idrac mode); then orderly shutdown with the iDRAC cooling, once, in every mode.
func TestCycleDanger(t *testing.T) {
	for _, mode := range []string{"model", "fixed", "idrac"} {
		for _, k := range []string{"CPU1", "Exhaust", "drive"} {
			t.Run(mode+"/"+k, func(t *testing.T) {
				c, f, get := rig(t)
				calls := 0
				old := poweroff
				poweroff = func() error { calls++; return nil }
				defer func() { poweroff = old }()
				d := Drive{ID: 1, Serial: "Z1P34VPQ"}
				hdds.Lock()
				hdds.ids, hdds.temps, hdds.at, hdds.scanned = []Drive{d}, map[Drive]float64{d: 30}, at(0), true
				hdds.Unlock()
				st.Lock()
				st.mode = mode
				st.Unlock()
				run(c, f, at(0))
				for i, s := range []int{5, 10, 15} {
					switch k {
					case "CPU1":
						f.vals["CPU1"] = cpuAuto
					case "Exhaust":
						f.vals["Exhaust"] = exhaustAuto
					case "drive": // each confirmation is a fresh drive read (fanctl kicks one)
						hdds.Lock()
						hdds.temps, hdds.at = map[Drive]float64{d: hddAuto}, at(s)
						hdds.Unlock()
					}
					run(c, f, at(s))
					if i < 2 && (calls != 0 || c.state == "shutdown") {
						t.Fatalf("reading %d: shutdown before confirmation", i+1)
					}
				}
				run(c, f, at(20))
				if calls != 1 || c.state != "shutdown" {
					t.Fatalf("poweroff calls %d, controller %q", calls, c.state)
				}
				want(t, 20, c, f, get, "auto", int(pcapMin), true)
			})
		}
	}
}

// One reading over the line, then a normal one: no shutdown, the count starts over.
func TestCycleDangerGlitch(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	for i, v := range []float64{exhaustAuto, 30, exhaustAuto, exhaustAuto, 30} {
		f.vals["Exhaust"] = v
		run(c, f, at(5+5*i))
	}
	if calls != 0 {
		t.Fatalf("glitches powered off")
	}
}

// A garbled drive byte (SMART can return up to 254) is no reading, not a danger.
func TestCycleDriveImplausible(t *testing.T) {
	c, f, get := rig(t)
	d := Drive{ID: 1, Serial: "Z1P34VPQ"}
	hdds.Lock()
	hdds.ids, hdds.temps, hdds.at, hdds.scanned = []Drive{d}, map[Drive]float64{d: 30}, at(0), true
	hdds.Unlock()
	run(c, f, at(0))
	hdds.Lock()
	hdds.temps, hdds.at = map[Drive]float64{d: 254}, at(5)
	hdds.Unlock()
	run(c, f, at(5))
	want(t, 5, c, f, get, "10", int(pcapMax), false)
	if len(c.danger) != 0 || c.in.HDD["Z1P34VPQ"].V != 30 {
		t.Fatalf("implausible drive value used: danger %v value %g", c.danger, c.in.HDD["Z1P34VPQ"].V)
	}
}

// A failed fan write doesn't skip the down-ramp: it continues from the last duty written.
func TestCycleRampAfterWriteError(t *testing.T) {
	c, f, get := rig(t)
	run(c, f, at(0))
	f.missing["Inlet"] = true
	for s := 5; s <= 20; s += 5 {
		run(c, f, at(s)) // safe from 20: 20 %
	}
	f.missing["Inlet"] = false
	run(c, f, at(25))
	want(t, 25, c, f, get, "18", int(pcapMax), false)
	f.dutyErr = errors.New("fan duty: timeout")
	run(c, f, at(30)) // 16 % not sent
	f.dutyErr = nil
	run(c, f, at(35))
	want(t, 35, c, f, get, "16", int(pcapMax), false)
}

// required_temps follow the config, also on a reload: a sensor added is expected, one removed (and
// never seen) is dropped.
func TestCycleRequiredTempsFollowConfig(t *testing.T) {
	c, f, get := rig(t)
	run(c, f, at(0))
	loadConfigBytes([]byte("[inputs]\nrequired_temps = [\"CPU1\", \"Exhuast\"]\ninterval = \"5s\"\n"))
	run(c, f, at(5))
	want(t, 5, c, f, get, "20", int(pcapMin), true) // the typo is never seen: unknown
	loadConfigBytes([]byte("[inputs]\nrequired_temps = [\"CPU1\", \"Exhaust\"]\ninterval = \"5s\"\n"))
	run(c, f, at(10))
	if _, still := c.in.Temps["Exhuast"]; still || c.state != "model" {
		t.Fatalf("typo still expected after the fix: %v, %q", c.in.Temps, c.state)
	}
}

// Fixed sends whatever it is given (even 100 %) until a reading reaches its FIXED_*_MAX: then model,
// and it stays model.
func TestCycleFixed(t *testing.T) {
	c, f, get := rig(t)
	st.Lock()
	st.mode, st.fixed = "fixed", 100
	st.Unlock()
	run(c, f, at(0))
	want(t, 0, c, f, get, "100", int(pcapMax), false)
	f.vals["CPU1"] = fixedCPUMax
	run(c, f, at(5))
	if c.state != "model" || st.mode != "model" {
		t.Fatalf("at FIXED_CPU_MAX: controller %q mode %q", c.state, st.mode)
	}
	f.vals["CPU1"] = 50
	run(c, f, at(10))
	if c.state != "model" {
		t.Fatalf("cooled down: back to %q, want to stay model", c.state)
	}
}

// The model never sends more than modelCeiling, whatever it computes.
func TestCycleModelCeiling(t *testing.T) {
	c, f, _ := rig(t)
	f.vals["CPU1"] = cpuAuto - 0.5 // past CPU_LIMIT: EMERGENCY_PCT, still under danger
	run(c, f, at(0))
	if p, _ := strconv.Atoi(f.fan); p > modelCeiling {
		t.Fatalf("model sent %d%%", p)
	}
}

// idrac mode: the iDRAC runs the fans; unknown inputs don't override it (it has its own sensors).
func TestCycleIdrac(t *testing.T) {
	c, f, get := rig(t)
	st.Lock()
	st.mode = "idrac"
	st.Unlock()
	run(c, f, at(0))
	f.missing["Inlet"] = true
	for s := 5; s <= 30; s += 5 {
		run(c, f, at(s))
	}
	want(t, 30, c, f, get, "auto", int(pcapMax), false)
}

// A restart in the middle of a gap: the new run expects the same sensors and counts from the last
// good reading, also for a sensor that vanished during the restart.
func TestCycleRestart(t *testing.T) {
	c, f, get := rig(t)
	run(c, f, at(0))
	f.missing["Inlet"] = true
	run(c, f, at(5))
	run(c, f, at(10)) // saved: Inlet last seen at 0

	c2 := newController(f) // the next run
	c2.start = at(14)
	c2.loadState()
	f.missing["Fan1"] = true // gone across the restart too: last seen at 10
	run(c2, f, at(15))
	want(t, 15, c2, f, get, "10", int(pcapMax), false)
	run(c2, f, at(20)) // Inlet 15 s past due
	want(t, 20, c2, f, get, "20", int(pcapMin), true)
	if !c2.in.Temps["Inlet"].Seen.Equal(at(0)) || !c2.in.Fans["Fan1"].Seen.Equal(at(10)) {
		t.Fatalf("ages not carried over: Inlet %v Fan1 %v", c2.in.Temps["Inlet"].Seen, c2.in.Fans["Fan1"].Seen)
	}
}

// Never seen is unknown at once: a required sensor missing at the start -> safe, never model without it.
func TestCycleNeverSeen(t *testing.T) {
	c, f, get := rig(t)
	f.missing["Exhaust"] = true
	run(c, f, at(0))
	want(t, 0, c, f, get, "20", int(pcapMin), true)
	f.missing["Exhaust"] = false
	run(c, f, at(5))
	want(t, 5, c, f, get, "18", int(pcapMax), false)
}

// BMC down from the very start, nothing carried over: everything unknown -> safe.
func TestCycleBlindStart(t *testing.T) {
	c, f, get := rig(t)
	f.readErr = errors.New("ipmi: no response in 5s")
	run(c, f, at(0))
	want(t, 0, c, f, get, "20", int(pcapMin), true)
	f.readErr = nil
	run(c, f, at(5))
	want(t, 5, c, f, get, "18", int(pcapMax), false)
}

// Drive temperatures age on their own period: unknown 15 s after the next read was due (plus driveSlack).
func TestCycleDriveGap(t *testing.T) {
	c, f, get := rig(t)
	d := Drive{ID: 1, Serial: "Z1P34VPQ"}
	hdds.Lock()
	hdds.ids, hdds.temps, hdds.at, hdds.scanned = []Drive{d}, map[Drive]float64{d: 30}, at(0), true
	hdds.Unlock()
	run(c, f, at(0))
	hdds.Lock()
	hdds.temps, hdds.at = map[Drive]float64{}, at(300) // next read: no temperature
	hdds.Unlock()
	run(c, f, at(370)) // due at 300, plus driveSlack (60 s) for a slow pass, plus unknown_after (15 s)
	want(t, 370, c, f, get, "10", int(pcapMax), false)
	run(c, f, at(375))
	want(t, 375, c, f, get, "20", int(pcapMin), true)
}

// CPU temperatures come from coretemp, not the BMC: per-core sensors are ignored; a coretemp
// package that disappears goes unknown like any other input.
func TestCycleCoretemp(t *testing.T) {
	c, f, get := rig(t)
	run(c, f, at(0))
	if v := c.in.Temps["CPU1"].V; v != 50 {
		t.Fatalf("CPU1 = %g, want 50 (coretemp package, not core 99)", v)
	}
	f.missing["CPU2"] = true
	for s := 5; s <= 15; s += 5 {
		run(c, f, at(s))
	}
	want(t, 15, c, f, get, "10", int(pcapMax), false)
	run(c, f, at(20))
	want(t, 20, c, f, get, "20", int(pcapMin), true)
}

// pick is total: every (mode, worst class) maps to exactly one controller, as the design table says.
func TestPickTable(t *testing.T) {
	wantTab := map[string][3]string{ // ok, unknown, danger
		"model": {"model", "safe", "shutdown"},
		"fixed": {"fixed", "safe", "shutdown"},
		"idrac": {"idrac", "idrac", "shutdown"},
	}
	for mode, w := range wantTab {
		for k := classOK; k <= classDanger; k++ {
			if got := pick(mode, k); got != w[k] {
				t.Errorf("pick(%s, %s) = %s, want %s", mode, k, got, w[k])
			}
		}
	}
}

// classify over every Reading shape and limit edge.
func TestClassify(t *testing.T) {
	loadConfigBytes(nil)
	cases := []struct {
		kind kind
		r    Reading
		want Class
	}{
		{kindCPU, NeverSeen{}, classUnknown},
		{kindCPU, Fresh{50}, classOK},
		{kindCPU, Fresh{cpuAuto}, classDanger},
		{kindCPU, Stale{50, unknownAfter - time.Second}, classOK},
		{kindCPU, Stale{50, unknownAfter}, classUnknown},
		{kindCPU, Stale{cpuAuto, unknownAfter}, classUnknown}, // a stale value is never trusted, not even a hot one
		{kindDrive, Fresh{hddAuto}, classDanger},
		{kindExhaust, Fresh{exhaustAuto}, classDanger},
		{kindOtherTemp, Fresh{99}, classOK}, // inlet: no limit of its own
		{kindFan, Fresh{fanMinRPM - 1}, classUnknown},
		{kindFan, Fresh{fanMinRPM}, classOK},
		{kindOther, Stale{1, -time.Minute}, classOK}, // not due yet
	}
	for _, tc := range cases {
		if got := classify(named{"x", tc.kind, tc.r}); got != tc.want {
			t.Errorf("classify(%v, %#v) = %s, want %s", tc.kind, tc.r, got, tc.want)
		}
	}
}

// The clamp saves the old settings once, writes only what differs, reports whether it was new, and is
// undone exactly (also after a crash: the saved file alone is enough).
func TestClamp(t *testing.T) {
	dir := t.TempDir()
	oldDir, oldState := pstateDir, throttleState
	pstateDir, throttleState = filepath.Join(dir, "intel_pstate"), filepath.Join(dir, "run-throttle")
	defer func() { pstateDir, throttleState = oldDir, oldState }()
	os.Mkdir(pstateDir, 0o755)
	set := func(k, v string) { os.WriteFile(filepath.Join(pstateDir, k), []byte(v+"\n"), 0o644) }
	get := func(k string) string { v, _ := readTrim(filepath.Join(pstateDir, k)); return v }
	set("no_turbo", "0")
	set("max_perf_pct", "100")
	set("min_perf_pct", "40")

	if ok, err := clampOff(); ok || err != nil {
		t.Fatalf("nothing clamped: restored=%v err=%v", ok, err)
	}
	for i := 0; i < 3; i++ { // every throttled cycle calls it again
		newly, err := clampOn()
		if err != nil || newly != (i == 0) {
			t.Fatalf("call %d: newly=%v err=%v", i, newly, err)
		}
	}
	if get("no_turbo") != "1" || get("max_perf_pct") != "40" {
		t.Fatalf("clamped: no_turbo=%s max_perf_pct=%s", get("no_turbo"), get("max_perf_pct"))
	}
	set("max_perf_pct", "70") // someone loosened it while clamped: the next cycle puts it back
	if _, err := clampOn(); err != nil || get("max_perf_pct") != "40" {
		t.Fatalf("re-clamp: err %v max_perf_pct %s", err, get("max_perf_pct"))
	}
	if ok, err := clampOff(); !ok || err != nil {
		t.Fatalf("restore: restored=%v err=%v", ok, err)
	}
	if get("no_turbo") != "0" || get("max_perf_pct") != "100" || exists(throttleState) {
		t.Fatalf("restored: no_turbo=%s max_perf_pct=%s state left %v", get("no_turbo"), get("max_perf_pct"), exists(throttleState))
	}
	// a half-applied clamp (one write failed) is still undone by the next clampOff: the state file decides
	if os.Geteuid() != 0 { // root ignores the read-only mode
		os.Chmod(filepath.Join(pstateDir, "max_perf_pct"), 0o444) // readable, writes fail
		if _, err := clampOn(); err == nil || get("no_turbo") != "1" || get("max_perf_pct") != "100" || !exists(throttleState) {
			t.Fatalf("half clamp: err %v no_turbo %s max_perf_pct %s state %v", err, get("no_turbo"), get("max_perf_pct"), exists(throttleState))
		}
		os.Chmod(filepath.Join(pstateDir, "max_perf_pct"), 0o644)
		if ok, err := clampOff(); !ok || err != nil || get("no_turbo") != "0" || get("max_perf_pct") != "100" {
			t.Fatalf("undo half clamp: restored=%v err=%v no_turbo=%s", ok, err, get("no_turbo"))
		}
	}
	// can't save the old settings -> refuse to clamp (it could never be undone after a crash)
	throttleState = filepath.Join(dir, "missing-dir", "throttle")
	if _, err := clampOn(); err == nil || get("max_perf_pct") != "100" {
		t.Fatalf("unsaveable state: err %v max_perf_pct %s", err, get("max_perf_pct"))
	}
}

// On a host with coretemp (the server), the real sysfs tree yields one plausible value per package.
// Read-only.
func TestRealCoretemp(t *testing.T) {
	cpus, why := senseCoretemp()
	if len(cpus) == 0 && len(why) == 0 {
		t.Skip("no coretemp on this host")
	}
	t.Logf("coretemp: %v (failures: %v)", cpus, why)
	if len(why) != 0 || cpus["CPU1"] <= 0 {
		t.Fatalf("coretemp read: %v %v", cpus, why)
	}
}

// Sensors fanctl discovers stay expected whatever required_temps lists: CPU2 and Exhaust are watched
// with required_temps = [CPU1, Inlet] (the example config's shape).
func TestCycleDiscoveredSensorsKept(t *testing.T) {
	c, f, get := rig(t)
	loadConfigBytes([]byte("[inputs]\nrequired_temps = [\"CPU1\", \"Inlet\"]\ninterval = \"5s\"\n"))
	run(c, f, at(0))
	for _, k := range []string{"CPU1", "CPU2", "Inlet", "Exhaust"} {
		if c.in.Temps[k].Seen.IsZero() {
			t.Fatalf("%s not expected/filled: %v", k, c.in.Temps)
		}
	}
	f.missing["CPU2"] = true
	for s := 5; s <= 20; s += 5 {
		run(c, f, at(s))
	}
	want(t, 20, c, f, get, "20", int(pcapMin), true) // CPU2 is watched: unknown -> safe
}

// A cap the iDRAC reverted (its own reset) is written again, not left to fail safe into a shutdown.
func TestCycleCapRewrittenOnReadback(t *testing.T) {
	c, f, get := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.missing["Inlet"] = true
	for s := 5; s <= 25; s += 5 {
		run(c, f, at(s)) // safe from 20, cap 230
	}
	f.cap = int(pcapMax)        // the iDRAC put its own limit back
	c.in.Cap.Seen = time.Time{} // force the next read-back
	for s := 30; s <= 60; s += 5 {
		run(c, f, at(s))
	}
	if calls != 0 || f.cap != int(safeCap) || c.state != "safe" {
		t.Fatalf("reverted cap: poweroff %d, cap %d, %q", calls, f.cap, c.state)
	}
	_ = get
}

// A shutdown is final: the fans stay with the iDRAC while the OS powers off.
func TestCycleShutdownLatched(t *testing.T) {
	c, f, get := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.vals["Exhaust"] = exhaustAuto
	for s := 5; s <= 15; s += 5 {
		run(c, f, at(s))
	}
	f.vals["Exhaust"] = 30 // cooled, or the reading went away during the OS shutdown
	f.missing["Inlet"] = true
	for s := 20; s <= 80; s += 5 {
		run(c, f, at(s))
		want(t, s, c, f, get, "auto", int(pcapMin), true)
	}
	if calls != 1 {
		t.Fatalf("poweroff calls %d", calls)
	}
}

// decide failing goes through the safe path: the safe window starts, nothing powers off at once.
func TestCycleDecideFailSafeWindow(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	st.Lock()
	st.mode, st.fixed = "fixed", 100 // fans fast: they need time to come down to safe.duty
	st.Unlock()
	run(c, f, at(0))
	f.rpm["Fan1"], f.rpm["Fan2"] = 4790, 4790 // the fake's fans don't go through the table broken below
	fanRPMTab = curve{{10, 3270}}             // one point (validation would refuse it): the model panics
	defer loadConfigBytes(nil)
	st.Lock()
	st.mode = "model"
	st.Unlock()
	for s := 5; s <= 40; s += 5 { // the safe read-back check panics too: plain safe, no crash, no poweroff
		run(c, f, at(s))
		if c.state != "safe" || calls != 0 {
			t.Fatalf("t=%d decide failing: %q calls %d", s, c.state, calls)
		}
	}
}

// Shutdown: the poweroff goes first, so a write that panics (or a hung BMC) can't stop or delay it.
func TestCycleShutdownPoweroffFirst(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.vals["Exhaust"] = exhaustAuto
	f.capPanic = true
	for s := 5; s <= 25; s += 5 {
		run(c, f, at(s))
	}
	if calls != 1 || c.state != "shutdown" || f.fan != "auto" {
		t.Fatalf("poweroff calls %d, controller %q, fans %q (want the iDRAC's)", calls, c.state, f.fan)
	}
}

// A sensor newly required on a reload that already reads fine is used at once: no spurious safe cycle.
func TestCycleNewlyRequiredReading(t *testing.T) {
	c, f, _ := rig(t)
	run(c, f, at(0))
	delete(c.in.Temps, "CPU2") // not expected so far (e.g. missing at the first good read)
	loadConfigBytes([]byte("[inputs]\nrequired_temps = [\"CPU1\", \"CPU2\", \"Inlet\", \"Exhaust\"]\ninterval = \"5s\"\n"))
	run(c, f, at(5))
	if c.state != "model" || c.in.Temps["CPU2"].Seen.IsZero() {
		t.Fatalf("newly required, reading: %q, CPU2 %v", c.state, c.in.Temps["CPU2"])
	}
}

// Without the clamp in [safe], intel_pstate isn't needed at all: a host without it runs normally.
func TestCycleNoClampNoPstate(t *testing.T) {
	c, f, _ := rig(t)
	loadConfigBytes([]byte("[safe]\nclamp = false\n[inputs]\nrequired_temps = [\"CPU1\", \"CPU2\", \"Inlet\", \"Exhaust\"]\ninterval = \"5s\"\n"))
	os.RemoveAll(pstateDir)
	for s := 0; s <= 30; s += 5 {
		run(c, f, at(s))
	}
	if c.state != "model" {
		t.Fatalf("no intel_pstate, clamp unused: %q (%s)", c.state, c.why)
	}
}

// A pulled drive's danger count is forgotten: reinserted, one reading at the line doesn't shut down.
func TestCycleDangerCountForgotten(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	d := Drive{ID: 1, Serial: "Z1P34VPQ"}
	set := func(ids []Drive, temps map[Drive]float64, s int) {
		hdds.Lock()
		hdds.ids, hdds.temps, hdds.at, hdds.scanned = ids, temps, at(s), true
		hdds.Unlock()
	}
	set([]Drive{d}, map[Drive]float64{d: 30}, 0)
	run(c, f, at(0))
	set([]Drive{d}, map[Drive]float64{d: hddAuto}, 5)
	run(c, f, at(5))
	set([]Drive{d}, map[Drive]float64{d: hddAuto}, 10)
	run(c, f, at(10))                 // 2/3
	set(nil, map[Drive]float64{}, 15) // pulled (the scans agree)
	run(c, f, at(15))
	set([]Drive{d}, map[Drive]float64{d: hddAuto}, 20) // back, one reading at the line
	run(c, f, at(20))
	if calls != 0 {
		t.Fatalf("one reading after reinsertion powered off")
	}
}

// Safe is in effect with the fans at least check_duty's RPM: faster (the iDRAC raising them) is fine.
func TestCycleSafeFansFaster(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.missing["Inlet"] = true
	f.rpm["Fan1"], f.rpm["Fan2"] = 9000, 9000
	for s := 5; s <= 60; s += 5 {
		run(c, f, at(s))
	}
	if calls != 0 || c.state != "safe" {
		t.Fatalf("fans faster than safe: %q, poweroff calls %d", c.state, calls)
	}
}

// Fans below check_duty's RPM: safe sends its duty again every cycle while it waits, and powers off
// only if 15 s of that doesn't help.
func TestCycleSafeResendsFans(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.missing["Inlet"] = true
	for s := 5; s <= 20; s += 5 {
		run(c, f, at(s)) // safe from 20
	}
	f.rpm["Fan1"], f.rpm["Fan2"] = 1500, 1500 // something else slowed them
	before := f.fanCalls
	for s := 25; s <= 30; s += 5 {
		run(c, f, at(s))
	}
	if f.fanCalls-before < 2 {
		t.Fatalf("duty not re-sent while safe isn't in effect: %d writes", f.fanCalls-before)
	}
	delete(f.rpm, "Fan1")
	delete(f.rpm, "Fan2") // the re-send worked
	for s := 35; s <= 60; s += 5 {
		run(c, f, at(s))
	}
	if calls != 0 {
		t.Fatalf("powered off although the re-send fixed it")
	}
}

// The stored drives mirror the scan: a pulled drive leaves (metrics, state file).
func TestCycleDriveLeavesWithScan(t *testing.T) {
	c, f, _ := rig(t)
	d1, d2 := Drive{ID: 1, Serial: "A"}, Drive{ID: 2, Serial: "B"}
	hdds.Lock()
	hdds.ids, hdds.temps, hdds.at, hdds.scanned = []Drive{d1, d2}, map[Drive]float64{d1: 30, d2: 31}, at(0), true
	hdds.Unlock()
	run(c, f, at(0))
	hdds.Lock()
	hdds.ids, hdds.temps, hdds.at = []Drive{d1}, map[Drive]float64{d1: 30}, at(5)
	hdds.Unlock()
	run(c, f, at(5))
	if _, still := c.in.HDD["B"]; still || len(c.in.HDD) != 1 {
		t.Fatalf("pulled drive kept: %v", c.in.HDD)
	}
}

// A shutdown leaves its mark (so the stop hook keeps minimum power); a failing poweroff keeps retrying
// the hand-over of the fans to the iDRAC too.
func TestCycleShutdownMarkAndRetry(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return errors.New("logind not answering") }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.vals["Exhaust"] = exhaustAuto
	f.dutyErr = errors.New("ipmi: timeout") // the first hand-over fails too
	for s := 5; s <= 15; s += 5 {
		run(c, f, at(s))
	}
	if !exists(shutdownMark) || c.state != "shutdown" {
		t.Fatalf("shutdown mark %v, controller %q", exists(shutdownMark), c.state)
	}
	f.dutyErr = nil
	run(c, f, at(20))
	if f.fan != "auto" || calls < 2 {
		t.Fatalf("poweroff failing: fans %q (want the iDRAC's), poweroff tries %d", f.fan, calls)
	}
}

// Most fans must reach check_duty's RPM: one fast fan doesn't hide a slow one (lower median).
func TestCycleSafeMostFans(t *testing.T) {
	c, f, _ := rig(t)
	calls := 0
	old := poweroff
	poweroff = func() error { calls++; return nil }
	defer func() { poweroff = old }()
	run(c, f, at(0))
	f.missing["Inlet"] = true
	f.rpm["Fan1"], f.rpm["Fan2"] = 2000, 9000 // one above fan_min_rpm but slow, one fast
	for s := 5; s <= 35; s += 5 {
		run(c, f, at(s))
	}
	if calls != 1 {
		t.Fatalf("half the fans slow in safe: poweroff calls %d", calls)
	}
}
