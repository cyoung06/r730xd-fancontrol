package main

import (
	"os"
	"path/filepath"
	"testing"
)

// The reload validates the bytes by running "<self> check -"; in the test binary "self" is the test
// binary, so answer that call the way the real binary does.
func TestMain(m *testing.M) {
	if len(os.Args) > 2 && os.Args[1] == "check" {
		cmdCheck(os.Args[2])
		os.Exit(0)
	}
	os.Exit(m.Run())
}

// TestReloadFullPath runs the whole reload: read the file, validate it in a subprocess, apply it here.
// Base file: FANCTL_TEST_CONFIG (the real fanctl.toml), else built-in defaults.
func TestReloadFullPath(t *testing.T) {
	base := ""
	if p := os.Getenv("FANCTL_TEST_CONFIG"); p != "" {
		b, err := os.ReadFile(p)
		if err != nil {
			t.Fatal(err)
		}
		base = string(b)
	}
	file := filepath.Join(t.TempDir(), "fanctl.toml")
	os.Setenv("FANCTL_CONFIG", file)
	defer os.Unsetenv("FANCTL_CONFIG")
	write := func(s string) {
		if err := os.WriteFile(file, []byte(s), 0o600); err != nil {
			t.Fatal(err)
		}
	}
	// the overrides go first: TOML keys before any [section] header are top-level, so prepend a
	// [model] table of our own only when the base doesn't have one
	withWarm := func(warm string) string {
		if base == "" {
			return "[model]\nwarm_curve = " + warm + "\n"
		}
		return base + "\n# test\n" // the real file: only check it loads
	}

	write(withWarm("[[74, 10], [78, 15], [82, 20]]"))
	loadConfig()
	if base == "" && warmCurve[0].x != 74 {
		t.Fatalf("start: warm %v", warmCurve)
	}

	// 1. good change
	write(withWarm("[[77, 10], [80, 15], [83, 20]]"))
	apply, err := prepareReload()
	if err != nil {
		t.Fatalf("good file refused: %v", err)
	}
	apply()
	if base == "" && (warmCurve[0].x != 77 || len(warmCurve) != 3) {
		t.Fatalf("warm curve not applied: %v", warmCurve)
	}
	t.Logf("applied: warm_curve=%v mode=%s", warmCurve, mode0)
	good := warmCurve

	// 2. bad value: refused by the check subprocess, running settings untouched
	write("[model]\ncpu_limit = 99\n")
	if _, err := prepareReload(); err == nil {
		t.Fatalf("bad file accepted")
	} else {
		t.Logf("bad file refused: %v", err)
	}
	// 3. a typo: refused before the subprocess
	write("[modle]\nfan_max = 20\n")
	if _, err := prepareReload(); err == nil {
		t.Fatalf("typo accepted")
	}
	if warmCurve[0] != good[0] || cpuLimit == 99 {
		t.Fatalf("refused reload changed settings: warm %v limit %v", warmCurve, cpuLimit)
	}

	// 4. the same good file again, three times: identical (no accumulation)
	write(withWarm("[[77, 10], [80, 15], [83, 20]]"))
	for i := 0; i < 3; i++ {
		apply, err = prepareReload()
		if err != nil {
			t.Fatal(err)
		}
		apply()
	}
	if len(exhaustModel) != 2 || len(warmCurve) != len(good) {
		t.Fatalf("after repeated reloads: warm %v exhaust %v", warmCurve, exhaustModel)
	}
}

// A config the validator would accept must be one the controller can run: empty curves are refused.
func TestReloadRefusesEmptyCurve(t *testing.T) {
	file := filepath.Join(t.TempDir(), "fanctl.toml")
	os.Setenv("FANCTL_CONFIG", file)
	defer os.Unsetenv("FANCTL_CONFIG")
	os.WriteFile(file, []byte("[model]\nhdd_curve = []\n"), 0o600)
	if _, err := prepareReload(); err == nil {
		t.Fatal("empty hdd_curve accepted")
	}
}

// A reload re-reading the same mode doesn't undo a fixed -> model hand-over; a changed mode applies.
func TestReloadKeepsFallback(t *testing.T) {
	file := filepath.Join(t.TempDir(), "fanctl.toml")
	os.Setenv("FANCTL_CONFIG", file)
	defer os.Unsetenv("FANCTL_CONFIG")
	os.WriteFile(file, []byte("mode = \"fixed\"\n[fixed]\nduty = 40\n"), 0o600)
	loadConfig()
	st.mode, st.fixed = "model", 40 // fixed handed over to model
	apply, err := prepareReload()
	if err != nil {
		t.Fatal(err)
	}
	apply()
	if st.mode != "model" {
		t.Fatalf("same file reloaded: mode %q, want model kept", st.mode)
	}
	os.WriteFile(file, []byte("mode = \"fixed\"\n[fixed]\nduty = 50\n"), 0o600)
	if apply, err = prepareReload(); err != nil {
		t.Fatal(err)
	}
	apply()
	if st.mode != "fixed" || st.fixed != 50 {
		t.Fatalf("changed file: mode %q fixed %d", st.mode, st.fixed)
	}
}
