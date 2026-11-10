package main

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

var gapT0 = time.Unix(1_800_000_000, 0)

func gapAt(s int) time.Time { return gapT0.Add(time.Duration(s) * time.Second) }

func gapController() *controller {
	return &controller{
		expect:   map[string]bool{"CPU1": true, "CPU2": true, "Exhaust": true, "Inlet": true},
		fansSeen: map[string]bool{"Fan1": true, "Fan2": true},
	}
}

// gapRead returns one cycle's readings without the named sensors, plus the BMC's reason for each.
func gapRead(missing ...string) (temps, fans map[string]float64, why map[string]string) {
	temps = map[string]float64{"CPU1": 77, "CPU2": 78, "Exhaust": 54, "Inlet": 32}
	fans = map[string]float64{"Fan1": 3000, "Fan2": 3100}
	why = map[string]string{}
	for _, k := range missing {
		delete(temps, k)
		delete(fans, k)
		why[k] = "reading unavailable, status 0x60"
	}
	return
}

// Every expected temperature and fan: hold the last value for 20 s, then level 2 (floor + throttle)
// for as long as it stays gone; back to 0 when it returns.
func TestFillGaps(t *testing.T) {
	for _, k := range []string{"CPU2", "Exhaust", "Inlet", "Fan2"} {
		t.Run(k, func(t *testing.T) {
			c := gapController()
			want := map[string]float64{"CPU2": 78, "Exhaust": 54, "Inlet": 32, "Fan2": 3100}[k]
			step := func(s int, missing bool) (float64, error) {
				var temps, fans map[string]float64
				var why map[string]string
				if missing {
					temps, fans, why = gapRead(k)
				} else {
					temps, fans, why = gapRead()
				}
				err := c.fillGaps(temps, fans, why, gapAt(s))
				if v, ok := temps[k]; ok {
					return v, err
				}
				return fans[k], err
			}
			check := func(s int, missing bool, level int) {
				t.Helper()
				v, err := step(s, missing)
				if err != nil || c.gapLevel != level || st.gap != level || v != want {
					t.Fatalf("t=%ds missing=%v: err %v level %d metric %d value %v (want level %d value %v)",
						s, missing, err, c.gapLevel, st.gap, v, level, want)
				}
			}
			check(0, false, 0)
			for s := 5; s < 25; s += 5 { // first miss at 5: held until 20 s later
				check(s, true, 1)
			}
			for s := 25; s <= 600; s += 5 { // 20 s+: floor + throttle, never gives up
				check(s, true, 2)
			}
			check(605, false, 0)
			// the real 11-10 blip: 2 missed reads -> held only, no floor
			check(610, true, 1)
			check(615, true, 1)
			check(620, false, 0)
		})
	}
}

// Two sensors at once: level is the worst; each has its own 20 s.
func TestFillGapsTwoSensors(t *testing.T) {
	c := gapController()
	run := func(s int, missing ...string) {
		temps, fans, why := gapRead(missing...)
		if err := c.fillGaps(temps, fans, why, gapAt(s)); err != nil {
			t.Fatal(err)
		}
	}
	run(0)
	run(5, "CPU1")
	run(15, "CPU1", "Fan1")
	if c.gapLevel != 1 {
		t.Fatalf("t=15: level %d", c.gapLevel)
	}
	run(25, "CPU1", "Fan1") // CPU1 gone 20 s, Fan1 10 s
	if c.gapLevel != 2 {
		t.Fatalf("t=25: level %d", c.gapLevel)
	}
	run(30, "Fan1") // CPU1 back, Fan1 15 s
	if c.gapLevel != 1 {
		t.Fatalf("t=30: level %d", c.gapLevel)
	}
}

// A sensor never seen with a value has nothing to hold: it fails at once, with the BMC's reason.
func TestFillGapsNoPrevious(t *testing.T) {
	for _, k := range []string{"Inlet", "Fan1"} {
		c := gapController()
		temps, fans, why := gapRead(k)
		err := c.fillGaps(temps, fans, why, gapAt(0))
		if err == nil || !strings.Contains(err.Error(), k) || !strings.Contains(err.Error(), "status 0x60") {
			t.Fatalf("%s: want error with reason, got %v", k, err)
		}
	}
}

// The clamp saves the old settings once, is idempotent, and is undone exactly (also after a crash:
// the saved file alone is enough).
func TestThrottleClamp(t *testing.T) {
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

	if ok, err := throttleOff(); ok || err != nil {
		t.Fatalf("nothing clamped: restored=%v err=%v", ok, err)
	}
	for i := 0; i < 3; i++ { // every throttled cycle calls it again
		if err := throttleOn(); err != nil {
			t.Fatal(err)
		}
	}
	if get("no_turbo") != "1" || get("max_perf_pct") != "40" {
		t.Fatalf("clamped: no_turbo=%s max_perf_pct=%s", get("no_turbo"), get("max_perf_pct"))
	}
	if ok, err := throttleOff(); !ok || err != nil {
		t.Fatalf("restore: restored=%v err=%v", ok, err)
	}
	if get("no_turbo") != "0" || get("max_perf_pct") != "100" {
		t.Fatalf("restored: no_turbo=%s max_perf_pct=%s", get("no_turbo"), get("max_perf_pct"))
	}
	if _, err := os.Stat(throttleState); !os.IsNotExist(err) {
		t.Fatalf("state file left: %v", err)
	}
	// can't save the old settings -> refuse to clamp (it could never be undone after a crash)
	throttleState = filepath.Join(dir, "missing-dir", "throttle")
	if err := throttleOn(); err == nil || get("max_perf_pct") != "100" {
		t.Fatalf("unsaveable state: err %v max_perf_pct %s", err, get("max_perf_pct"))
	}
}
