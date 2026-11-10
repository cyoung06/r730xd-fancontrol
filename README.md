# r730xd-fancontrol

A quiet fan controller for the Dell PowerEdge R730xd (iDRAC8). It keeps the fans as low as the
hardware allows, typically 10–20 %, and holds the CPUs at a target temperature. When low fan
speed isn't enough, it caps power through the iDRAC instead of spinning the fans up.

`fanctl` is a single static Go binary. It talks to the BMC through the kernel IPMI driver
(`/dev/ipmi0`) and to the drives through the PERC driver (`/dev/megaraid_sas_ioctl_node`), and
needs no `ipmitool`, `smartctl`, Python or containers. It also exports Prometheus metrics.

## How it works

Every `inputs.interval` seconds fanctl reads the inlet, exhaust and CPU temperatures, fan RPM and wall
power from the BMC. Drive temperatures come through the PERC every `inputs.hdd_interval` seconds, on their
own thread. It then picks a duty and sets it with Dell's OEM fan command (`0x30 0x30`).

Modes (`mode`):

| mode    | duty |
|---------|------|
| `model` | power first (the cap holds the CPUs at `model.cpu_target` with the fans at `model.fan_max`), then the fans: drive and exhaust demand, never above 50 %. Without a power reading it uses the temperature curves |
| `fixed` | `fixed.duty`, any value 0-100 %. A reading at its `fixed.*_max` hands over to `model` for good (until fixed is chosen again) |
| `idrac` | iDRAC automatic control (`auto` is accepted as the same) |

Whatever the mode, an unknown input selects the safe controller and a reading in danger shuts the
server down (see Fail-safe).

### Changing settings without a restart

Edit `/etc/fanctl.toml`, then `sudo systemctl reload fanctl`. fanctl reads the file once, has a
`fanctl check -` subprocess validate exactly those bytes, and swaps the settings in between two control
cycles, so the fans never pass through the iDRAC. A refused file (a bad value, or an unknown key) is
logged and the running settings stay. `mode` / `fixed.duty` from the file replace a choice made through
`/control`; `http.listen` and `http.token_file` still need a restart. `fanctl check [file]` validates a
file by hand.

New binary: install it over `/usr/local/bin/fanctl`, then `sudo systemctl kill -s USR2 fanctl`. fanctl
re-execs itself in place (same PID), so the fans keep their duty instead of passing through the iDRAC.

To restart anyway without the hand-back (e.g. a unit-file change, which USR2 doesn't apply):
`sudo touch /run/fanctl-handoff` within a minute before the stop makes both the stopping fanctl and the
following `fanctl auto` leave the fans alone, once.

A running sensor gap survives a restart or re-exec (see Fail-safe: `/run/fanctl/inputs.json`).

### The exhaust model (all modes with drive readings)

The exhaust stands in for parts without a temperature sensor (PERC, RAM, PCIe cards). Most of its heat
comes from the CPUs and drives, which have their own control, so fanctl compares the exhaust against what
those explain:

```
T_exh_expected = T_inlet + a * (T_cpu - T_inlet) + b * (T_hdd - T_inlet)      (hottest CPU, hottest drive)
```

The exhaust is a mix of inlet air and air warmed by each part, so `a, b >= 0` and `a + b < 1`, with no
constant term. Fan speed drops out: faster fans take more heat off a part but spread it through
proportionally more air. `model.exhaust_excess_curve` turns the excess (measured minus expected) into a duty,
so the exhaust only raises the fans for heat the sensors don't account for. `shutdown.exhaust` still applies
to the absolute exhaust, and a cycle without a drive reading uses the absolute `model.exhaust_curve`. The
default `a, b` (0.3716, 0.1911) were fitted on 8 days of steady data from my R730xd (rmse 1.2 C); refit
for another machine.

### The thermal model (`mode = "model"`)

The steady-state CPU temperature is modelled per socket as

```
T_cpu = T_inlet + (P - pb) * (r0 + k * (rpm/1000)^-n) * scale
```

- `P` is the iDRAC wall power.
- `pb` is the share of that power that doesn't heat the CPU.
- `r0 + k·rpm^-n` is the CPU-to-air thermal resistance, which falls as airflow rises.

fanctl inverts this model to get the fan RPM, and from that the duty, that holds `model.cpu_target - model.margin`.

- **Feedforward from RAPL:** the iDRAC power reading lags by about 30 s. RAPL package power reacts within one cycle, so fanctl uses `max(wall, RAPL + learned offset)`.
- **Integral correction:** a slow correction trims model error (dust, room, drive layout). It only integrates when the temperature has settled and the model is in control, so it can't wind up.
- **Warm floor:** `model.warm_curve` sets a minimum duty that rises as the CPU warms, which keeps headroom for a sudden load step.
- **Fan ceiling and power cap:**
  - The CPU never pushes the fans past `model.fan_max`.
  - Instead, the DCMI power limit is set to the most power that still holds `model.cpu_target` at `model.fan_max`, plus a correction that tightens while the CPU stays above target.
  - The cap is verified by reading it back. It is never deactivated, because on iDRAC8 a later activate silently fails, so "no cap" is `power.cap_max` (586 W).
  - The exception action is forced to "no action" (never a hard power-off), and leaving model mode lifts the cap back to `power.cap_max`.
- **Emergency:** at `model.cpu_limit` the fans jump to `model.emergency_duty` at once. At the `[shutdown]` lines the server shuts down (Fail-safe).

> **The default model parameters were fitted on one specific machine.** That's my R730xd, with
> its CPUs, heatsinks, drive layout and blanks, from a 12-hour fan% × power-cap benchmark. Your box
> will differ. Keep `model.margin` generous (without `model.fan_max` and a power reading the model runs on its curves) and watch the
> `fanctl_model_correction_percent` metric. A correction pinned at ±`model.i_max` means the model
> doesn't fit your machine. `model.fan_rpm_table` (duty:RPM at fixed duty) is also machine-specific.

## Fail-safe

Errors are data, not branches. Each cycle: **sense → classify → pick → control → act**.

- **Sense.** CPU1/CPU2 come from the CPUs' own package sensors through the kernel (coretemp,
  `Package id 0/1`), not the iDRAC: the same sensors without the iDRAC's dropouts, ~1 °C lower.
  Inlet, exhaust, fans and wall power come from the BMC, drives from the PERC. Every input is a value
  plus when it last read valid; a failed, timed-out, implausible or "unavailable" read changes
  nothing, the input just gets older. Each one is logged with the source's answer.
- **Classify.** Each input (each temperature, fan and drive, the drive count against
  `inputs.hdd_min`, and the read-backs of the power cap and the CPU clamp) is **ok**, **unknown** or
  **danger**:
  - unknown: never seen this run, or `inputs.unknown_after` (15 s) past its due time (5 s inputs: the
    3rd missed read, 20 s after the last good one; drives: 15 s after a missed `inputs.hdd_interval`
    read, plus a minute for a slow pass); a fan below `inputs.fan_min_rpm`. Until then the last value
    stands in. Writes are never inputs: an error is logged and retried, and the read-backs say what took.
  - danger: CPU ≥ `shutdown.cpu`, drive ≥ `shutdown.hdd`, exhaust ≥ `shutdown.exhaust` (inlet has
    no line), for **3 consecutive new readings**. Until confirmed it counts as unknown, and a drive is
    re-read at once. Drive values outside 1-99 °C are no reading.
- **Pick.** The worst class: danger → **shutdown** (any mode); mode `idrac` → **idrac**; unknown →
  **safe**; otherwise the mode's controller (**model** or **fixed**).
- **Control.**
  - safe: `[safe]` (20 % fans, CPUs at minimum power: turbo off and `intel_pstate` `max_perf_pct` =
    min since RAPL is locked by the BIOS, iDRAC cap at `safe.cap`); held a full CPU load with falling
    temperatures. It must be **verifiably** in effect: fans reading at least `safe.check_duty`'s RPM
    (faster is fine), the cap at most `safe.cap`, the clamp on. Until then it re-sends its settings;
    not in effect after 15 s → shutdown.
  - shutdown: fans to the iDRAC, CPUs at minimum power, and a clean OS poweroff
    (`systemctl poweroff`), once.
- **Act.** Each actuator is written only when it differs from what was last written.

A panic or an impossible value (NaN) in classify, pick or a controller → the safe controller. Inputs
and ages are saved every cycle in `/run/fanctl/inputs.json`, so a restart continues a running gap.

| situation | what happens |
|---|---|
| a reading missing, timed out, implausible, "unavailable"; the BMC not answering | last value for 15 s past due, then safe (`fanctl_input_age_seconds`, `fanctl_controller`, `fanctl_cpu_clamped`) |
| drive temperature missing, fewer than `inputs.hdd_min` drives, the PERC unreachable | same, on the drives' own period; a drive missing from one rescan stays expected until the next scan agrees |
| a required sensor never seen this run | unknown at once → safe |
| a fan below `inputs.fan_min_rpm` | unknown → safe |
| duty, cap or clamp writes failing | logged and retried; in safe, read-backs not matching for 15 s → shutdown |
| the BMC not answering at all | its inputs unknown → safe; safe can't be confirmed (no fan RPM, no cap read-back) → shutdown 15 s later |
| a reading at its `[shutdown]` line for 3 consecutive readings | danger → orderly shutdown, the iDRAC cooling meanwhile |
| CPU ≥ `model.cpu_limit` in `model` | duty ≥ `model.emergency_duty` |
| invalid settings (e.g. `inputs.interval` > 10 s, limits out of order, non-finite model values) | refuses to start; fans stay with the iDRAC |
| fanctl stops, crashes or is killed | systemd `ExecStopPost=fanctl auto`: CPU clamp undone, fans → iDRAC auto (retries for up to 12 s), power cap → `power.cap_max`; restart after 5 s |
| control loop hangs | systemd `WatchdogSec=60` kills it → `ExecStopPost` → restart |
| whole OS hangs | optional BMC hardware watchdog resets the server |
| boot | `fanctl-failsafe.service` sets iDRAC auto before fanctl starts |
| iDRAC reset | fanctl re-sends its duty every `inputs.reassert` s and takes the fans back |

## Requirements

- Dell R730xd, or likely other 13th-gen PowerEdge, with iDRAC8. Dell removed the raw fan command
  in iDRAC9 firmware 3.34 and later.
- Linux on amd64, with `ipmi_si` and `ipmi_devintf` loaded (Ubuntu loads them by default).
- Drive temperatures need a PERC on the `megaraid_sas` driver (HBA mode works). SAS drives use
  LOG SENSE page 0Dh; SATA drives use SMART attribute 194. Without a PERC, fanctl runs without
  drive temperatures.

## Install

```sh
go build -trimpath -ldflags="-s -w" -o fanctl .          # or cross-compile: GOOS=linux GOARCH=amd64 CGO_ENABLED=0
sudo ./fanctl probe                                       # read-only: every reading fanctl will use
```

`probe` lists the BMC sensors with their values, the DCMI power limit, RAPL power and the drives
it finds. Compare them with `ipmitool sdr` before going further.

```sh
sudo install -m 755 fanctl /usr/local/bin/fanctl
sudo install -m 644 fanctl.toml.example /etc/fanctl.toml  # then edit it; fanctl check validates it
(umask 077; openssl rand -hex 16 | sudo tee /etc/fanctl.token >/dev/null)   # optional: enables /control
sudo install -m 644 systemd/fanctl.service systemd/fanctl-failsafe.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable fanctl-failsafe.service
sudo systemctl enable --now fanctl.service
journalctl -u fanctl -f
```

Optional BMC hardware watchdog. systemd pets the iDRAC watchdog, and if the OS hangs for 60 s the
iDRAC resets the server:

```sh
sudo install -m 644 systemd/ipmi_watchdog.conf /etc/modules-load.d/
sudo modprobe ipmi_watchdog
sudo mkdir -p /etc/systemd/system.conf.d
sudo install -m 644 systemd/watchdog.conf /etc/systemd/system.conf.d/
sudo systemctl daemon-reexec
```

## Usage

```
fanctl [run]   control loop (settings: /etc/fanctl.toml, or $FANCTL_CONFIG)
fanctl check [file|-]  validate a config file; exit 1 if refused
fanctl auto    hand the fans back to the iDRAC and exit
fanctl probe   print every reading fanctl uses; changes nothing
fanctl prepare create /dev/megaraid_sas_ioctl_node if missing (the driver doesn't; the unit runs this)
```

Runtime mode switch, when `http.token_file` holds a token:

```
curl 'http://127.0.0.1:9101/control?token=...&mode=fixed&pct=30'
curl 'http://127.0.0.1:9101/control?token=...&mode=model'
```

## Settings

All settings go in `/etc/fanctl.toml`, one section per controller; see
[`fanctl.toml.example`](fanctl.toml.example) for every key with its default. Unknown keys are refused.

| section | keys |
|---|---|
| top level | `mode`: `model`, `fixed`, `idrac` (`auto` = `idrac`) |
| `[shutdown]` | `cpu`, `hdd`, `exhaust`: danger lines, orderly poweroff at/above |
| `[safe]` | `duty`, `cap` (W), `clamp`: what an unknown input gets |
| `[model]` | `min_duty`, `fan_max`, `cpu_target`, `margin`, `cpu_limit`, `emergency_duty`, `ki`, `i_max`, the curves (`[[temp, duty], ...]`), `exhaust_model`, `cpu_models`, `fan_rpm_table` |
| `[fixed]` | `duty` (0-100), `cpu_max` / `hdd_max` / `exhaust_max`: hand over to model at/above (0 = shutdown line − 3) |
| `[power]` | `cap_min`, `cap_max` ("no cap"), `cap_step`, `kw` |
| `[inputs]` | `required_temps`, `hdd_min`, `fan_min_rpm`, `interval` (≤ 10 s), `hdd_interval`, `hdd_rescan`, `reassert`, `unknown_after` (durations: `"5s"`) |
| `[http]` | `listen` (list), `token_file`: the `/control` token, kept out of this file |

## Metrics

`GET /metrics` serves the following:

| metric | |
|---|---|
| `fanctl_temperature_celsius{sensor}` | temperatures (CPU1/CPU2 from coretemp, the rest from the BMC) |
| `fanctl_fan_rpm{fan}` | fan speeds |
| `fanctl_hdd_temperature_celsius{id,serial}` | drive temperatures |
| `fanctl_power_watts` | wall power |
| `fanctl_feedforward_watts` | power used by the model (RAPL feedforward) |
| `fanctl_duty_percent` | duty sent (`-1` in auto) |
| `fanctl_auto_mode` | 1 = the iDRAC is in control |
| `fanctl_controller{controller}` | 1 for the controller that ran last: model, fixed, safe, shutdown, idrac |
| `fanctl_cpu_clamped` | 1 = CPUs at their lowest frequency |
| `fanctl_input_age_seconds{input}` | seconds since each input last read valid |
| `fanctl_demand_percent{source}` | what each of cpu/hdd/exhaust asks for |
| `fanctl_model_correction_percent` | integral correction |
| `fanctl_power_cap_watts` | iDRAC power cap |
| `fanctl_power_cap_correction_watts` | cap correction |
| `fanctl_errors_total` | failed cycles |
| `fanctl_last_read_timestamp_seconds` | last good sensor read |
| `fanctl_info{mode,reason}` | why the current duty |

## License

MIT
