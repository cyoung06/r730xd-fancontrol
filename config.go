//go:build linux

package main

// ---------- settings: /etc/fanctl.toml ----------
//
// One TOML file, sections by controller. Unknown keys are refused (a typo must not silently fall back
// to a default). The control token lives in its own file (http.token_file), so this one holds no
// secret. A refused file exits at start (systemd leaves the fans with the iDRAC); on a reload the
// running settings stay.

import (
	"bytes"
	"fmt"
	"io"
	"log"
	"os"
	"os/exec"
	"sort"
	"strings"
	"time"

	"github.com/BurntSushi/toml"
)

func configPath() string {
	if p := os.Getenv("FANCTL_CONFIG"); p != "" {
		return p
	}
	return "/etc/fanctl.toml"
}

// duration: "5s", "300s", "10m" in the file.
type duration time.Duration

func (d *duration) UnmarshalText(b []byte) error {
	v, err := time.ParseDuration(string(b))
	*d = duration(v)
	return err
}

// points: [[x, y], ...] in the file.
type points [][2]float64

func (p points) curve() curve {
	c := make(curve, len(p))
	for i, xy := range p {
		c[i] = point{xy[0], xy[1]}
	}
	sort.Slice(c, func(i, j int) bool { return c[i].x < c[j].x })
	return c
}

type config struct {
	Mode string `toml:"mode"` // model | fixed | idrac

	Shutdown struct { // danger: at/above -> orderly shutdown
		CPU     float64 `toml:"cpu"`
		HDD     float64 `toml:"hdd"`
		Exhaust float64 `toml:"exhaust"`
	} `toml:"shutdown"`

	Safe struct { // any input unknown
		Duty  float64 `toml:"duty"`
		Cap   float64 `toml:"cap"`   // iDRAC power cap, W
		Clamp bool    `toml:"clamp"` // CPUs at their lowest frequency, no turbo
		// in effect when the fans read at least this duty's RPM and the power is at its minimum
		CheckDuty float64 `toml:"check_duty"`
	} `toml:"safe"`

	Model struct {
		MinDuty            float64     `toml:"min_duty"`
		FanMax             float64     `toml:"fan_max"` // the goal: power holds the CPUs once the fans are here (0 = no cap management)
		CPUTarget          float64     `toml:"cpu_target"`
		Margin             float64     `toml:"margin"`         // the model aims this far below cpu_target
		CPULimit           float64     `toml:"cpu_limit"`      // at/above: emergency_duty at once
		EmergencyDuty      float64     `toml:"emergency_duty"` //
		KI                 float64     `toml:"ki"`             // correction, % per C per minute
		IMax               float64     `toml:"i_max"`          // correction limit, +/- %
		WarmCurve          points      `toml:"warm_curve"`     // fan floor as the hottest CPU warms
		HDDCurve           points      `toml:"hdd_curve"`
		ExhaustModel       []float64   `toml:"exhaust_model"`        // a, b: expected exhaust from inlet, CPU and drives
		ExhaustExcessCurve points      `toml:"exhaust_excess_curve"` // excess over expected (C) -> duty
		CPUCurve           points      `toml:"cpu_curve"`            // without a power reading
		ExhaustCurve       points      `toml:"exhaust_curve"`        // without a drive reading
		CPUModels          [][]float64 `toml:"cpu_models"`           // per CPU: pb, r0, k, n, scale
		FanRPMTable        points      `toml:"fan_rpm_table"`        // duty -> rpm at fixed duty
	} `toml:"model"`

	Fixed struct {
		Duty       float64 `toml:"duty"`        // 0-100
		CPUMax     float64 `toml:"cpu_max"`     // at/above -> model, for good (0 = shutdown line - 3)
		HDDMax     float64 `toml:"hdd_max"`     //
		ExhaustMax float64 `toml:"exhaust_max"` //
	} `toml:"fixed"`

	Power struct {
		CapMin  float64 `toml:"cap_min"`  // never cap below (W)
		CapMax  float64 `toml:"cap_max"`  // "no cap": the iDRAC's highest accepted limit
		CapStep float64 `toml:"cap_step"` // only re-send when the cap moves this much (W)
		KW      float64 `toml:"kw"`       // cap correction, W per C per minute
	} `toml:"power"`

	Inputs struct {
		RequiredTemps []string `toml:"required_temps"`
		HDDMin        int      `toml:"hdd_min"`
		FanMinRPM     float64  `toml:"fan_min_rpm"`
		Interval      duration `toml:"interval"`
		HDDInterval   duration `toml:"hdd_interval"`
		HDDRescan     duration `toml:"hdd_rescan"`
		Reassert      duration `toml:"reassert"`
		UnknownAfter  duration `toml:"unknown_after"` // past due -> unknown
	} `toml:"inputs"`

	HTTP struct {
		Listen    []string `toml:"listen"`
		TokenFile string   `toml:"token_file"` // empty or missing file = /control disabled
	} `toml:"http"`
}

func defaults() config {
	var c config
	c.Mode = "model"
	c.Shutdown.CPU, c.Shutdown.HDD, c.Shutdown.Exhaust = 85, 50, 65
	c.Safe.Duty, c.Safe.Cap, c.Safe.Clamp = 20, 230, true // check_duty: see parseConfig
	m := &c.Model
	m.MinDuty, m.FanMax, m.CPUTarget, m.Margin, m.CPULimit, m.EmergencyDuty = 10, 0, 75, 3, 78, 50
	m.KI, m.IMax = 1.0, 15
	m.WarmCurve = points{{68, 10}, {72, 15}, {76, 20}}
	m.HDDCurve = points{{38, 10}, {41, 20}, {44, 35}, {47, 50}}
	// expected exhaust, fitted on steady history (2026-11-08, rmse 1.2 C):
	//   T_exh = T_inlet + a*(T_cpu - T_inlet) + b*(T_hdd - T_inlet)   (hottest CPU, hottest drive)
	m.ExhaustModel = []float64{0.3716, 0.1911}
	m.ExhaustExcessCurve = points{{3, 10}, {5, 20}, {7, 40}}
	m.CPUCurve = points{{55, 10}, {62, 15}, {68, 25}, {74, 40}, {80, 60}}
	m.ExhaustCurve = points{{38, 10}, {42, 25}, {46, 45}}
	// T_cpu = T_inlet + (P - pb) * (r0 + k*(rpm/1000)^-n) * scale   (P = wall power; fit 2026-10-31)
	m.CPUModels = [][]float64{{58.6228, 0.1107, 0.6588, 1.4572, 0.9565}, {89.5481, 0.1445, 1.1485, 1.8038, 1.0562}}
	m.FanRPMTable = points{{10, 3270}, {15, 4060}, {20, 4790}, {25, 5560}, {30, 6430}}
	c.Fixed.Duty = 20
	c.Power.CapMin, c.Power.CapMax, c.Power.CapStep, c.Power.KW = 230, 586, 6, 8
	in := &c.Inputs
	in.RequiredTemps = []string{"CPU1", "Inlet", "Exhaust"}
	in.FanMinRPM = 1000
	in.Interval, in.HDDInterval, in.HDDRescan, in.Reassert = duration(5*time.Second), duration(300*time.Second), duration(600*time.Second), duration(60*time.Second)
	in.UnknownAfter = duration(15 * time.Second)
	c.HTTP.Listen = []string{"127.0.0.1:9101"}
	return c
}

// parseConfig decodes b over the defaults; unknown keys are an error.
func parseConfig(b []byte) (config, error) {
	c := defaults()
	md, err := toml.Decode(string(b), &c)
	if err != nil {
		return c, err
	}
	if u := md.Undecoded(); len(u) > 0 {
		return c, fmt.Errorf("unknown keys: %v", u)
	}
	if c.Mode == "auto" {
		c.Mode = "idrac"
	}
	if c.Safe.CheckDuty == 0 { // default: 15 %, never above safe.duty
		c.Safe.CheckDuty = min(15, c.Safe.Duty)
	}
	if c.Fixed.CPUMax == 0 {
		c.Fixed.CPUMax = c.Shutdown.CPU - 3
	}
	if c.Fixed.HDDMax == 0 {
		c.Fixed.HDDMax = c.Shutdown.HDD - 3
	}
	if c.Fixed.ExhaustMax == 0 {
		c.Fixed.ExhaustMax = c.Shutdown.Exhaust - 3
	}
	return c, nil
}

// apply sets the running settings from c and validates them (exits if refused). Called at start and,
// after the check subprocess accepted the same bytes, on a reload.
func apply(c config) {
	switch c.Mode {
	case "model", "fixed", "idrac":
	default:
		log.Fatalf("config: mode must be model, fixed or idrac, not %q", c.Mode)
	}
	mode0, fixedPct0 = c.Mode, int(c.Fixed.Duty)
	cpuAuto, hddAuto, exhaustAuto = c.Shutdown.CPU, c.Shutdown.HDD, c.Shutdown.Exhaust
	safeDuty, safeCap, safeClamp, safeCheckDuty = c.Safe.Duty, c.Safe.Cap, c.Safe.Clamp, c.Safe.CheckDuty
	m := c.Model
	minPct, fanMax, cpuTarget, modelMargin, cpuLimit, emergencyPct = m.MinDuty, m.FanMax, m.CPUTarget, m.Margin, m.CPULimit, m.EmergencyDuty
	ki, iMax = m.KI, m.IMax
	warmCurve, hddCurve, cpuCurve, exhaustCurve = m.WarmCurve.curve(), m.HDDCurve.curve(), m.CPUCurve.curve(), m.ExhaustCurve.curve()
	exhaustModel = append([]float64(nil), m.ExhaustModel...)
	exhaustExcessCurve, fanRPMTab = m.ExhaustExcessCurve.curve(), m.FanRPMTable.curve()
	cpuModels = nil
	for i, v := range m.CPUModels {
		if len(v) != 5 {
			log.Fatalf("config: model.cpu_models[%d]: want [pb, r0, k, n, scale]", i)
		}
		cpuModels = append(cpuModels, cpuModel{v[0], v[1], v[2], v[3], v[4]})
	}
	fixedCPUMax, fixedHDDMax, fixedExhaustMax = c.Fixed.CPUMax, c.Fixed.HDDMax, c.Fixed.ExhaustMax
	pcapMin, pcapMax, pcapStep, kw = c.Power.CapMin, c.Power.CapMax, c.Power.CapStep, c.Power.KW
	in := c.Inputs
	requiredTemps = append([]string(nil), in.RequiredTemps...)
	hddMin, fanMinRPM = in.HDDMin, in.FanMinRPM
	interval, hddInterval, hddRescan, reassert = time.Duration(in.Interval), time.Duration(in.HDDInterval), time.Duration(in.HDDRescan), time.Duration(in.Reassert)
	unknownAfter = time.Duration(in.UnknownAfter)
	listen = strings.Join(c.HTTP.Listen, ",")
	tokenFile = c.HTTP.TokenFile
	validateConfig()
}

// loadConfigBytes parses and applies b (exits if refused).
func loadConfigBytes(b []byte) {
	c, err := parseConfig(b)
	if err != nil {
		log.Fatalf("config: %v", err)
	}
	apply(c)
}

// loadConfig reads and applies the config file (exits if refused).
func loadConfig() {
	b, err := os.ReadFile(configPath())
	if err != nil {
		log.Fatalf("config: %v", err)
	}
	loadConfigBytes(b)
}

var tokenFile string

// readToken: the /control token from http.token_file ("" = /control disabled). Read at start only.
func readToken() string {
	if tokenFile == "" {
		return ""
	}
	b, err := os.ReadFile(tokenFile)
	if err != nil {
		log.Printf("%s/control disabled: %v", prWarn, err)
		return ""
	}
	return strings.TrimSpace(string(b))
}

// cmdCheck validates a config file (or "-" = stdin): exit 1 if refused. Used by reload.
func cmdCheck(path string) {
	var b []byte
	var err error
	if path == "-" {
		b, err = io.ReadAll(os.Stdin)
	} else {
		b, err = os.ReadFile(path)
	}
	if err != nil {
		log.Fatalf("config: %v", err)
	}
	loadConfigBytes(b)
	fmt.Println("config ok")
}

// ---------- reload (systemctl reload fanctl = SIGHUP) ----------
//
// The file is read once; a "fanctl check -" subprocess validates exactly those bytes (a bad value exits
// that process, not this one), and only then are they applied in between two control cycles, so the
// fans never pass through the iDRAC. mode / fixed.duty from the file replace a runtime /control choice.
// http.listen and http.token_file changes need a restart.

// prepareReload reads and validates the config file; the returned func applies it (call between cycles).
func prepareReload() (func(), error) {
	b, err := os.ReadFile(configPath())
	if err != nil {
		return nil, err
	}
	c, err := parseConfig(b)
	if err != nil {
		return nil, fmt.Errorf("%s refused: %v", configPath(), err)
	}
	self, err := os.Executable()
	if err != nil {
		return nil, err
	}
	cmd := exec.Command(self, "check", "-")
	cmd.Env = []string{"PATH=/usr/sbin:/usr/bin:/sbin:/bin"}
	cmd.Stdin = bytes.NewReader(b)
	if out, err := cmd.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("%s refused: %s", configPath(), strings.TrimSpace(string(out)))
	}
	return func() {
		oldListen, oldToken, oldMode, oldFixed := listen, tokenFile, mode0, fixedPct0
		st.Lock() // /metrics reads the settings under this lock
		apply(c)
		listen, tokenFile = oldListen, oldToken
		// the file's mode / fixed.duty replace a runtime choice only when they changed in the file: a
		// reload must not undo a fixed -> model hand-over (or a /control choice) by re-reading the same
		if mode0 != oldMode || fixedPct0 != oldFixed {
			st.mode, st.fixed = mode0, fixedPct0
		}
		st.Unlock()
		log.Printf("reloaded %s: mode=%s", configPath(), mode0)
	}, nil
}

// capMaxForAuto: power.cap_max for "fanctl auto", read leniently: it must never block the hand-back.
func capMaxForAuto() int {
	c := defaults()
	if b, err := os.ReadFile(configPath()); err == nil {
		if _, err := toml.Decode(string(b), &c); err != nil {
			log.Printf("%s%s: %v (using cap_max %g)", prErr, configPath(), err, c.Power.CapMax)
		}
	}
	return int(c.Power.CapMax)
}
