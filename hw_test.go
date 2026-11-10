package main

import (
	"os"
	"path/filepath"
	"strconv"
	"testing"
)

// Hardware tests: root on the server only, FANCTL_HW_TEST=1. Each changes a live setting for a moment
// and puts it back. The service also manages both settings: run these only while it has no gap
// (fanctl_cpu_clamped 0, fanctl_safe_point 0), so neither side saves the other's values as the originals.

func hwOnly(t *testing.T) {
	if os.Getenv("FANCTL_HW_TEST") != "1" {
		t.Skip("set FANCTL_HW_TEST=1 (root, on the server)")
	}
}

func cpu0MaxFreq() (int, error) {
	v, err := readTrim("/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq")
	if err != nil {
		return 0, err
	}
	return strconv.Atoi(v)
}

// The real intel_pstate clamp: on, the CPUs' frequency limit drops; off, everything is as before.
func TestHWClamp(t *testing.T) {
	hwOnly(t)
	oldState := throttleState
	throttleState = filepath.Join(t.TempDir(), "throttle")
	defer func() { throttleState = oldState }()
	get := func(k string) string { v, _ := readTrim(pstateDir + "/" + k); return v }

	nt0, mx0 := get("no_turbo"), get("max_perf_pct")
	f0, err := cpu0MaxFreq()
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("before: no_turbo=%s max_perf_pct=%s min_perf_pct=%s cpu0 scaling_max_freq=%d", nt0, mx0, get("min_perf_pct"), f0)
	restored := false
	defer func() { // whatever happens below, never leave the host clamped
		if !restored {
			if _, err := clampOff(); err != nil {
				t.Errorf("RESTORE FAILED: %v (no_turbo was %s, max_perf_pct %s)", err, nt0, mx0)
			}
		}
	}()
	if _, err := clampOn(); err != nil {
		t.Fatal(err)
	}
	nt1, mx1 := get("no_turbo"), get("max_perf_pct")
	f1, err1 := cpu0MaxFreq()
	t.Logf("clamped: no_turbo=%s max_perf_pct=%s cpu0 scaling_max_freq=%d (%v)", nt1, mx1, f1, err1)
	ok, err := clampOff()
	restored = ok && err == nil
	nt2, mx2 := get("no_turbo"), get("max_perf_pct")
	f2, err2 := cpu0MaxFreq()
	t.Logf("restored (%v, %v): no_turbo=%s max_perf_pct=%s cpu0 scaling_max_freq=%d (%v)", ok, err, nt2, mx2, f2, err2)
	if nt1 != "1" || mx1 != get("min_perf_pct") || err1 != nil || f1 >= f0 {
		t.Errorf("clamp had no effect")
	}
	if !restored || nt2 != nt0 || mx2 != mx0 || err2 != nil || f2 != f0 {
		t.Errorf("not restored exactly")
	}
}

// The real iDRAC cap at PCAP_MIN: set, read back, then the previous cap again (an inactive cap is put
// back as FANCTL_HW_CAP_MAX: on iDRAC8 "no cap" is a high limit, never a deactivate).
func TestHWCap(t *testing.T) {
	hwOnly(t)
	min, err1 := strconv.Atoi(os.Getenv("FANCTL_HW_CAP_MIN"))
	max, err2 := strconv.Atoi(os.Getenv("FANCTL_HW_CAP_MAX"))
	if err1 != nil || err2 != nil {
		t.Fatal("set FANCTL_HW_CAP_MIN / FANCTL_HW_CAP_MAX (watts, = PCAP_MIN / PCAP_MAX)")
	}
	bmc, err := OpenBMC()
	if err != nil {
		t.Fatal(err)
	}
	prev, err := bmc.GetPowerLimit()
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("before: %d W active=%v", prev.Watts, prev.Active)
	back := prev.Watts
	if !prev.Active {
		back = max
	}
	restored := false
	defer func() {
		if !restored {
			if err := bmc.SetPowerCap(back); err != nil {
				t.Errorf("RESTORE FAILED: cap should be %d W: %v", back, err)
			}
		}
	}()
	setErr := bmc.SetPowerCap(min)
	got, gerr := bmc.GetPowerLimit()
	t.Logf("set %d W: err=%v, iDRAC reports %d W active=%v (%v)", min, setErr, got.Watts, got.Active, gerr)
	rerr := bmc.SetPowerCap(back)
	now, nerr := bmc.GetPowerLimit()
	restored = rerr == nil && nerr == nil && now.Watts == back && now.Active
	t.Logf("restored: err=%v, iDRAC reports %d W active=%v (%v)", rerr, now.Watts, now.Active, nerr)
	if setErr != nil || gerr != nil || got.Watts != min || !got.Active {
		t.Errorf("cap %d W not applied", min)
	}
	if !restored {
		t.Errorf("not restored to %d W", back)
	}
}
