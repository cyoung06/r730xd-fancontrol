# r730xd-fancontrol

A quiet fan controller for the Dell PowerEdge R730xd (iDRAC8). It keeps the fans as low as the
hardware allows, typically 10–20 %, and holds the CPUs at a target temperature. When low fan
speed isn't enough, it caps power through the iDRAC instead of spinning the fans up.

`fanctl` is a single static Go binary. It talks to the BMC through the kernel IPMI driver
(`/dev/ipmi0`) and to the drives through the PERC driver (`/dev/megaraid_sas_ioctl_node`), and
needs no `ipmitool`, `smartctl`, Python or containers. It also exports Prometheus metrics.

## How it works

Every `INTERVAL` seconds fanctl reads the inlet, exhaust and CPU temperatures, fan RPM and wall
power from the BMC. Drive temperatures come through the PERC every `HDD_INTERVAL` seconds, on their
own thread. It then picks a duty and sets it with Dell's OEM fan command (`0x30 0x30`).

Modes (`MODE`):

| mode    | duty |
|---------|------|
| `model` | lowest duty that settles the hottest CPU at `CPU_TARGET`, from a fitted thermal model (below) |
| `curve` | highest of three temperature curves: CPU, hottest drive, exhaust |
| `fixed` | `FIXED_PCT` (the safety limits still apply) |
| `auto`  | iDRAC automatic control |

### The thermal model (`MODE=model`)

The steady-state CPU temperature is modelled per socket as

```
T_cpu = T_inlet + (P - pb) * (r0 + k * (rpm/1000)^-n) * scale
```

- `P` is the iDRAC wall power.
- `pb` is the share of that power that doesn't heat the CPU.
- `r0 + k·rpm^-n` is the CPU-to-air thermal resistance, which falls as airflow rises.

fanctl inverts this model to get the fan RPM, and from that the duty, that holds `CPU_TARGET - MODEL_MARGIN`.

- **Feedforward from RAPL:** the iDRAC power reading lags by about 30 s. RAPL package power reacts within one cycle, so fanctl uses `max(wall, RAPL + learned offset)`.
- **Integral correction:** a slow correction trims model error (dust, room, drive layout). It only integrates when the temperature has settled and the model is in control, so it can't wind up.
- **Warm floor:** `WARM_CURVE` sets a minimum duty that rises as the CPU warms, which keeps headroom for a sudden load step.
- **Fan ceiling and power cap:**
  - The CPU never pushes the fans past `FAN_MAX`.
  - Instead, the DCMI power limit is set to the most power that still holds `CPU_TARGET` at `FAN_MAX`, plus a correction that tightens while the CPU stays above target.
  - The cap is verified by reading it back. It is never deactivated, because on iDRAC8 a later activate silently fails, so "no cap" is `PCAP_MAX` (586 W).
- **Emergency:** at `CPU_LIMIT` the fans jump to `EMERGENCY_PCT` at once. Above that, the `*_AUTO` limits hand control to the iDRAC.

> **The default model parameters were fitted on one specific machine.** That's my R730xd, with
> its CPUs, heatsinks, drive layout and blanks, from a 12-hour fan% × power-cap benchmark. Your box
> will differ. Start with `MODE=curve`, or keep `MODEL_MARGIN` generous and watch the
> `fanctl_model_correction_percent` metric. A correction pinned at ±`MODEL_I_MAX` means the model
> doesn't fit your machine. `FAN_RPM_TABLE` (duty:RPM at fixed duty) is also machine-specific.

## Fail-safe

The fans must never be left low while nothing is watching. Every failure path ends with the iDRAC
in control:

| failure | what happens |
|---|---|
| sensor read fails, BMC times out | fans → iDRAC auto, retried next cycle |
| a known drive stops reporting, drive data stale | fans → iDRAC auto |
| panic in a control cycle (bug, unexpected BMC reply) | recovered: fans → iDRAC auto, next cycle retries |
| panic in the drive reader | recovered and restarted; stale drive data → iDRAC auto meanwhile |
| CPU ≥ `CPU_AUTO`, drive ≥ `HDD_AUTO`, exhaust ≥ `EXHAUST_AUTO` | fans → iDRAC auto |
| fanctl stops, crashes or is killed | systemd `ExecStopPost=fanctl auto` (retries for up to 12 s), restart after 5 s |
| control loop hangs | systemd `WatchdogSec=30` kills it → `ExecStopPost` → restart |
| whole OS hangs | optional BMC hardware watchdog resets the server |
| boot | `fanctl-failsafe.service` sets iDRAC auto before fanctl starts |
| iDRAC reset | fanctl re-sends its duty every `REASSERT` s and takes the fans back |

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
sudo install -m 600 fanctl.env.example /etc/fanctl.env    # then edit it
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
fanctl [run]   control loop (settings from the environment / /etc/fanctl.env)
fanctl auto    hand the fans back to the iDRAC and exit
fanctl probe   print every reading fanctl uses; changes nothing
```

Runtime mode switch, when `CONTROL_TOKEN` is set:

```
curl 'http://127.0.0.1:9101/control?token=...&mode=fixed&pct=30'
curl 'http://127.0.0.1:9101/control?token=...&mode=model'
```

## Settings

All settings go in `/etc/fanctl.env`; see [`fanctl.env.example`](fanctl.env.example) for the
annotated defaults. The main ones:

| variable | default | |
|---|---|---|
| `MODE` | `curve` | `model`, `curve`, `fixed`, `auto` |
| `MIN_PCT` | `10` | duty floor |
| `CPU_TARGET` / `MODEL_MARGIN` | `75` / `3` | model mode: target; the model aims this far below |
| `CPU_LIMIT` / `EMERGENCY_PCT` | `78` / `50` | jump straight to this duty at the limit |
| `FAN_MAX` | `0` | fan ceiling for the CPU part; caps power instead (0 = off) |
| `PCAP_MIN` / `PCAP_MAX` | `230` / `586` | power cap range (W); `PCAP_MAX` = "no cap" |
| `WARM_CURVE` | `68:10,72:15,76:20` | model mode: duty floor vs hottest CPU |
| `CPU_CURVE`, `HDD_CURVE`, `EXHAUST_CURVE` | | `temp:duty,...` |
| `CPU_AUTO`, `HDD_AUTO`, `EXHAUST_AUTO` | `85`, `50`, `50` | hand over to the iDRAC at/above |
| `CPU1_MODEL`, `CPU2_MODEL` | fitted | `pb,r0,k,n,scale` |
| `FAN_RPM_TABLE` | `10:3270,...,30:6430` | measured RPM at fixed duty |
| `INTERVAL` / `HDD_INTERVAL` / `HDD_RESCAN` | `10` / `300` / `600` | seconds |
| `REASSERT` | `60` | re-send the duty this often (s) |
| `LISTEN` | `127.0.0.1:9101` | comma-separated listen addresses |
| `CONTROL_TOKEN` | empty | enables `/control` |

## Metrics

`GET /metrics` serves the following:

| metric | |
|---|---|
| `fanctl_temperature_celsius{sensor}` | BMC temperatures |
| `fanctl_fan_rpm{fan}` | fan speeds |
| `fanctl_hdd_temperature_celsius{id,serial}` | drive temperatures |
| `fanctl_power_watts` | wall power |
| `fanctl_feedforward_watts` | power used by the model (RAPL feedforward) |
| `fanctl_duty_percent` | duty sent (`-1` in auto) |
| `fanctl_auto_mode` | 1 = the iDRAC is in control |
| `fanctl_demand_percent{source}` | what each of cpu/hdd/exhaust asks for |
| `fanctl_model_correction_percent` | integral correction |
| `fanctl_power_cap_watts` | iDRAC power cap |
| `fanctl_power_cap_correction_watts` | cap correction |
| `fanctl_errors_total` | failed cycles |
| `fanctl_last_read_timestamp_seconds` | last good sensor read |
| `fanctl_info{mode,reason}` | why the current duty |

## License

MIT
