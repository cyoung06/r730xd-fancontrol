package main

import (
	"strings"
	"testing"
)

// apply runs again on every reload; it must give the same result as the first load, never accumulate
// (a doubled list once failed validation and exited the running controller).
func TestLoadConfigTwice(t *testing.T) {
	b := []byte(`mode = "model"` + "\n")
	loadConfigBytes(b)
	m, r := len(exhaustModel), len(requiredTemps)
	loadConfigBytes(b)
	loadConfigBytes(b)
	if len(exhaustModel) != m || len(requiredTemps) != r || m != 2 {
		t.Fatalf("after reloads: exhaustModel %d (want %d, first 2), requiredTemps %d (want %d)", len(exhaustModel), m, len(requiredTemps), r)
	}
}

// A typo is refused, never silently replaced by a default.
func TestConfigUnknownKey(t *testing.T) {
	if _, err := parseConfig([]byte("[model]\nfan_maxx = 20\n")); err == nil || !strings.Contains(err.Error(), "fan_maxx") {
		t.Fatalf("unknown key accepted: %v", err)
	}
}

// Sections land where they belong; fixed.*_max default to 3 C below the shutdown lines.
func TestConfigValues(t *testing.T) {
	loadConfigBytes([]byte(`
mode = "fixed"
[shutdown]
exhaust = 65
[safe]
duty = 22
cap = 240
clamp = false
[model]
fan_max = 20
warm_curve = [[80, 15], [77, 10]]
[fixed]
duty = 100
[inputs]
interval = "5s"
unknown_after = "15s"
`))
	if mode0 != "fixed" || fixedPct0 != 100 || exhaustAuto != 65 || fixedExhaustMax != 62 || fixedCPUMax != cpuAuto-3 {
		t.Fatalf("mode %s fixed %d exhaust %g/%g cpu max %g", mode0, fixedPct0, exhaustAuto, fixedExhaustMax, fixedCPUMax)
	}
	if safeDuty != 22 || safeCap != 240 || safeClamp || fanMax != 20 || warmCurve[0].x != 77 {
		t.Fatalf("safe %g %g %v fan_max %g warm %v", safeDuty, safeCap, safeClamp, fanMax, warmCurve)
	}
}
