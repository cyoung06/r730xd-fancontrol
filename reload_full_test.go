package main

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// The reload validates the file by running "<self> check"; in the test binary "self" is the test binary,
// so answer that call the way the real binary does.
func TestMain(m *testing.M) {
	if len(os.Args) > 1 && os.Args[1] == "check" {
		loadConfig()
		fmt.Println("config ok")
		os.Exit(0)
	}
	os.Exit(m.Run())
}

// TestReloadFullPath runs the whole reload: read the env file, validate it in a subprocess, apply it in
// this process. Base file: FANCTL_TEST_ENV (the real fanctl.env), else built-in defaults.
func TestReloadFullPath(t *testing.T) {
	base := ""
	if p := os.Getenv("FANCTL_TEST_ENV"); p != "" {
		b, err := os.ReadFile(p)
		if err != nil {
			t.Fatal(err)
		}
		base = string(b)
	}
	set := func(text, k, v string) string { // replace or add k=v
		var out []string
		found := false
		for _, l := range strings.Split(text, "\n") {
			if strings.HasPrefix(l, k+"=") {
				if v != "" {
					out = append(out, k+"="+v)
				}
				found = true
				continue
			}
			out = append(out, l)
		}
		if !found && v != "" {
			out = append(out, k+"="+v)
		}
		return strings.Join(out, "\n")
	}
	dir := t.TempDir()
	file := filepath.Join(dir, "fanctl.env")
	os.Setenv("FANCTL_ENV_FILE", file)
	write := func(s string) {
		if err := os.WriteFile(file, []byte(s), 0o600); err != nil {
			t.Fatal(err)
		}
	}

	// start: as systemd would, the file's values are in the environment
	start := set(set(base, "WARM_CURVE", "74:10,78:15,82:20"), "HDD_MIN", "3")
	write(start)
	m, err := readEnvFile(file)
	if err != nil {
		t.Fatal(err)
	}
	for k, v := range m {
		os.Setenv(k, v)
	}
	loadConfig()
	fileKeys = m
	if warmCurve[0].x != 74 || hddMin != 3 {
		t.Fatalf("start: warm %v hddMin %d", warmCurve, hddMin)
	}

	// 1. good change: new warm curve, HDD_MIN line removed (back to its default)
	write(set(set(start, "WARM_CURVE", "77:10,80:15,83:20"), "HDD_MIN", ""))
	apply, err := prepareReload()
	if err != nil {
		t.Fatalf("good file refused: %v", err)
	}
	apply()
	if warmCurve[0].x != 77 || warmCurve[2].y != 20 || len(warmCurve) != 3 {
		t.Fatalf("warm curve not applied: %v", warmCurve)
	}
	if _, still := os.LookupEnv("HDD_MIN"); still {
		t.Fatalf("removed HDD_MIN still set")
	}
	if len(exhaustModel) != 2 {
		t.Fatalf("exhaustModel %v", exhaustModel)
	}
	t.Logf("applied: WARM_CURVE=%v HDD_MIN=%d (default) mode=%s", warmCurve, hddMin, mode0)

	// 2. bad change: refused, running settings untouched
	write(set(start, "CPU_LIMIT", "99"))
	if _, err := prepareReload(); err == nil {
		t.Fatalf("bad file accepted")
	} else {
		t.Logf("bad file refused: %v", err)
	}
	if warmCurve[0].x != 77 || cpuLimit == 99 {
		t.Fatalf("refused reload changed settings: warm %v limit %v", warmCurve, cpuLimit)
	}

	// 3. reload again with the same good file: identical result (no accumulation)
	write(set(set(start, "WARM_CURVE", "77:10,80:15,83:20"), "HDD_MIN", ""))
	for i := 0; i < 3; i++ {
		apply, err = prepareReload()
		if err != nil {
			t.Fatal(err)
		}
		apply()
	}
	if warmCurve[0].x != 77 || len(exhaustModel) != 2 {
		t.Fatalf("after repeated reloads: warm %v exhaust %v", warmCurve, exhaustModel)
	}
}
