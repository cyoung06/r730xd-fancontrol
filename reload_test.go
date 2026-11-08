package main

import (
	"os"
	"testing"
)

// loadConfig runs again on every reload; it must give the same result as the first load, never
// accumulate (a doubled list once failed validation and exited the running controller).
func TestLoadConfigTwice(t *testing.T) {
	os.Setenv("MODE", "model")
	loadConfig()
	m, r := len(exhaustModel), len(requiredTemps)
	loadConfig()
	loadConfig()
	if len(exhaustModel) != m || len(requiredTemps) != r || m != 2 {
		t.Fatalf("after reloads: exhaustModel %d (want %d, first 2), requiredTemps %d (want %d)", len(exhaustModel), m, len(requiredTemps), r)
	}
}
