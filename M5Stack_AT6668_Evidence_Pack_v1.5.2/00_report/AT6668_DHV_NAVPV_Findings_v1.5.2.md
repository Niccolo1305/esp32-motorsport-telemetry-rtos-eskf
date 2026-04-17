# AT6668 GPS Velocity Investigation on M5Stack GPS Unit

## 1. Executive Summary

This document summarizes a focused investigation into the GPS velocity outputs exposed by the tested M5Stack GPS unit on an AT6668-based receiver path. The purpose was to determine whether the module could provide a higher-quality or more explicit velocity channel than the existing NMEA Speed Over Ground (`SOG`) already used by the firmware.

The final production decision is narrow and specific to the tested module, tested firmware, and tested command path. `DHV` is not usable as the primary velocity source because it updates at approximately `1 Hz`, while the main GPS fix path updates at approximately `10 Hz`. Periodic `NAV-PV` is not available through the tested `CFG-PRT -> CFG-MSG` path, because the module accepts `CFG-PRT` but NACKs `CFG-MSG` for periodic `NAV-PV` enable. As a result, `gps_sog_kmh` remains the only viable production velocity source at `10 Hz`.

This report is intended both as an internal engineering record and as a vendor-facing escalation package for M5Stack. The key value of the work is not a new filter behavior, but a reproducible evidence chain: `.bin` logs, MQTT traces, and offline replay all converge on the same conclusion, and the retained `202 B` SD record format preserves that evidence for future module comparisons.

## 2. System Under Test

### Module context

- Vendor product: M5Stack GPS unit using an AT6668-based receiver path
- Host MCU: M5Stack Atom S3 / ESP32-S3
- GPS interface: UART1 at `115200` baud

### Firmware context

- `v1.4.2`: baseline production path using NMEA GPS speed
- `v1.5.0`: explicit `NAV-PV` attempt and diagnostic expansion
- `v1.5.1`: `DHV` primary-path attempt
- `v1.5.2`: production path simplified back to `gps_sog_kmh`, while diagnostic fields were retained

### Test environment

- Indoor static tests
- Outdoor static tests
- Standalone power-bank operation, without serial tether
- MQTT-based remote observation
- SITL / offline replay from SD logs

## 3. Objective

The investigation had three goals:

1. Determine whether `DHV` could replace NMEA `SOG` as a higher-quality or more explicit production velocity source.
2. Determine whether periodic binary `NAV-PV` could be enabled and used as a `10 Hz` production velocity channel.
3. Decide whether the existing `SOG` path should remain the production baseline.

For this project, a velocity source is considered **usable as a primary source** only if it:

- supports the effective `10 Hz` GPS update path
- is compatible with stationary detection
- is compatible with straight-line ZARU speed gates
- is compatible with ESKF speed correction
- is compatible with COG baseline logic

## 4. Instrumentation Added

### SD / binary logging

The SD record was extended from `164 B` (`v1.4.2`) to `202 B` (`v1.5.x`) to preserve GPS source and timing diagnostics.

Additional fields retained in the `202 B` record:

| Field | Size |
|---|---:|
| `nav_speed2d` | 4 B |
| `nav_s_acc` | 4 B |
| `nav_vel_n` | 4 B |
| `nav_vel_e` | 4 B |
| `nav_vel_valid` | 1 B |
| `gps_speed_source` | 1 B |
| `nav_fix_us` | 8 B |
| `dhv_gdspd` | 4 B |
| `dhv_fix_us` | 8 B |
| **Total** | **38 B** |

### What the extra 38 B buy

The extra `38 B` do not add new filter dynamics. They buy:

- deterministic offline proof of source availability
- deterministic offline proof of source selection
- repeatable SITL replay of GPS timing and source behavior
- vendor-facing evidence without requiring serial capture

At `50 Hz`, the extra `38 B` add:

- `1900 B/s`
- `6.84 MB/hour` (decimal MB)

This storage cost was judged small compared with the diagnostic value already extracted from the field data.

### MQTT diagnostics

The following MQTT fields were added or used during the investigation:

- GPS timing: `gps_fix_us`, `gps_epoch`, `dhv_fix_us`, `nav_fix_us`
- Source visibility: `spd_src`, `nav_spd2d`, `dhv_spd2d`
- CASIC parser diagnostics: `dbg_sync`, `dbg_frames`, `dbg_ck_fail`, `dbg_other`, `dbg_navpv`, `dbg_navpv_vel`, `dbg_ovf`
- Last-frame diagnostics: `dbg_cls`, `dbg_id`, `dbg_len`, `dbg_ack_fid`, `dbg_ack_cls`, `dbg_ack_id`
- Configuration outcome diagnostics: `dbg_prt_ack`, `dbg_prt_nack`, `dbg_cfg_try`, `dbg_cfg_ack`, `dbg_cfg_nack`

`CFG-PRT` outcome is already visible in MQTT in the current firmware and therefore no longer requires a serial cable for outdoor verification.

### SITL / offline tooling

The following tools were used or updated during the investigation:

- `Tool/bin_to_csv.py`
- `Tool/motec_exporter.py`
- `Tool/sitl_hal/sitl_replay.py`
- `Tool/sitl_hal/static_gate_debug.py`
- `Tool/sitl_hal/gps_speed_compare.py`
- `Tool/sitl_hal/static_bench_validator.py`
- `Tool/sitl_hal/static_bench_compare.py`

These tools allow offline reconstruction of:

- GPS timing and staleness behavior
- source selection behavior
- static-gate outcomes
- speed-channel comparisons

## 5. Methodology

### `.bin` log analysis

Static and quasi-static `.bin` logs were replayed offline to evaluate:

- whether GPS speed alone could trigger stationary-gate failures
- whether outdoor RF quality changed the failure mode
- whether DHV or NAV-PV became active in the logs

### MQTT trace analysis

MQTT traces were used because the outdoor workflow ran from a power bank, without a serial tether. They were used to measure:

- effective update rate of `gps_fix_us`
- effective update rate of `dhv_fix_us`
- whether `CFG-PRT` was ACKed
- whether `CFG-MSG` was ACKed or NACKed
- whether any periodic `NAV-PV` frames appeared after configuration

### CASIC ACK/NACK interpretation

The investigation used the observed CASIC ACK/NACK fields:

- `dbg_cls`
- `dbg_id`
- `dbg_ack_fid`
- `dbg_ack_cls`
- `dbg_ack_id`

to determine whether the module:

- accepted `CFG-PRT`
- rejected periodic `NAV-PV` enable via `CFG-MSG`

### Freshness / source validation

The work explicitly separated:

- parser correctness
- source frequency
- source selection policy

This prevented a false conclusion that `DHV` was unusable because of parser bugs or freshness policy. The evidence showed instead that `DHV` itself updated at about `1 Hz`.

### Indoor vs outdoor separation

Indoor and outdoor logs were both used so that:

- RF degradation and multipath could be identified when present
- protocol limitations could be distinguished from bad reception

Outdoor static logs were especially important because they removed RF quality as the primary blocker while `DHV` and `NAV-PV` still failed to become viable production sources.

## 6. Evidence

### 6.1 Artifact inventory

Primary artifacts used in this report:

| ID | Artifact | Role |
|---|---|---|
| `A1` | `tel_11_speed_compare.txt` | Speed-channel comparison on static bench data |
| `A2` | `tel_20_static.txt` | Outdoor static validation |
| `A3` | `tel_21_static.txt` | Outdoor static validation |
| `A4` | `tel_20_gate.txt` | Outdoor static gate-failure attribution |
| `A5` | `tel_21_gate.txt` | Outdoor static gate-failure attribution |
| `M0` | `Telemetria.json` | Initial MQTT symptom capture |
| `M1` | `Telemetria1.json` | MQTT timing capture after `gps_fix_us` instrumentation |
| `M2` | `Telemetria2.json` | MQTT timing capture after explicit `PCAS03` test |
| `M3` | `Telemetria3.json` | MQTT protocol diagnostics after `CFG-PRT -> CFG-MSG` test |

### 6.2 `tel_11` speed-channel comparison

Source artifact: `A1`

Clean-dt (`90–110 ms`) comparison:

| Metric | Module speed | Lat/lon/dt speed | `kf_vel` |
|---|---:|---:|---:|
| Mean | `1.861 km/h` | `3.434 km/h` | `1.647 km/h` |
| Correlation with `kf_vel` | `0.741` | `0.271` | — |
| Ratio `>= 2.0 km/h` | `35.8%` | `63.4%` | `27.3%` |

Observed conclusion:

- module-reported speed is substantially closer to `kf_vel` than position-derived speed
- lat/lon/dt-derived speed is much noisier than the module-reported speed
- therefore “compute speed from position” is not a valid improvement path on this hardware/log set

Classification:

- **Proven observation** based on log analysis

### 6.3 Outdoor static gating (`tel_20` / `tel_21`)

Source artifacts: `A2`, `A3`, `A4`, `A5`

Key results:

| Log | Static ratio | `due_to_gps` | Result |
|---|---:|---:|---|
| `tel_20` | `100.0%` | `0` | PASS |
| `tel_21` | `97.5%` | `0` | PASS |

Observed conclusion:

- outdoor RF quality is not the blocker
- GPS-induced stationary-gate failures disappear outdoors
- the remaining blocker is protocol/channel behavior, not outdoor signal quality

Classification:

- **Proven observation** based on outdoor logs

### 6.4 MQTT timing evidence

Source artifacts: `M1`, `M2`

Extracted timing:

| File | Mean `gps_fix_us` delta | Mean `dhv_fix_us` delta | Interpretation |
|---|---:|---:|---|
| `Telemetria1.json` | `~107339 us` | `~1000764 us` | GPS fix path ~10 Hz, DHV path ~1 Hz |
| `Telemetria2.json` | `~107588 us` | `~1000099 us` | GPS fix path ~10 Hz, DHV path ~1 Hz |

Observed conclusion:

- `PCAS02` successfully drives the main GPS fix path at about `10 Hz`
- `DHV` does not follow that rate and remains at about `1 Hz`

Classification:

- **Proven observation** based on MQTT timing deltas

### 6.5 MQTT protocol diagnostics

Source artifact: `M3`

Relevant fields:

| Field | Value |
|---|---:|
| `dbg_prt_ack` | `1` |
| `dbg_prt_nack` | `0` |
| `dbg_cfg_try` | `1` |
| `dbg_cfg_ack` | `0` |
| `dbg_cfg_nack` | `1` |
| `dbg_navpv` | `0` |
| `dbg_navpv_vel` | `0` |
| `dbg_cls` | `5` |
| `dbg_id` | `0` |
| `dbg_ack_fid` | `0` |
| `dbg_ack_cls` | `6` |
| `dbg_ack_id` | `1` |

Observed conclusion:

- `CFG-PRT` was accepted
- `CFG-MSG` for periodic `NAV-PV` was NACKed
- no periodic `NAV-PV` stream was observed afterward

Classification:

- **Proven observation** based on MQTT diagnostics

### 6.6 Initial live symptom (`Telemetria.json`)

Source artifact: `M0`

Observed behavior:

- `spd_src` oscillated between `0` and `2`
- `dhv_fix_us` changed at approximately `1 Hz`
- this was the first live symptom that `DHV` could not remain the active production source under the existing freshness policy

This trace predates the later `gps_fix_us` instrumentation and therefore is useful mainly as an early symptom record, not as the strongest timing proof.

Classification:

- **Proven observation**, but weaker than `Telemetria1/2/3`

## 7. DHV Findings

The following statements are supported by the evidence above:

- `DHV` is parsed correctly.
- `DHV` updates regularly.
- `DHV` updates at approximately `1 Hz`, not `10 Hz`.
- Explicit `PCAS03` formatting did not change the observed `DHV` rate.

This is sufficient to reject `DHV` as the primary `10 Hz` control source on the tested module.

Classification:

- **Proven observations**: parser works, `DHV` is present, `DHV` is ~`1 Hz`
- **Engineering conclusion**: `DHV` is not fit for use as the production velocity source in this firmware pipeline

## 8. NAV-PV Findings

The following statements are supported by the evidence above:

- passive CASIC parsing and ACK/NACK interpretation work
- binary output can be enabled on the port via `CFG-PRT`
- periodic `NAV-PV` enable via `CFG-MSG (0x06, 0x01)` is NACKed
- no periodic `NAV-PV` frames were observed afterward

This is sufficient to reject periodic `NAV-PV` as an available production channel on this module through the tested command path.

Classification:

- **Proven observations**: `CFG-PRT` ACK, `CFG-MSG` NACK, no `NAV-PV`
- **Engineering conclusion**: periodic `NAV-PV` is not available through the tested configuration path on the tested module

## 9. Public Ecosystem Scan

A small public-source scan was performed to check whether the observed behavior could be explained by a documented M5Stack-specific initialization sequence rather than by the tested module firmware or product configuration.

### 9.1 Official M5Stack product documentation

Source artifact: `P1`

The current public M5Stack documentation for the Atomic GPS Base v2.0 states:

- module model / receiver path based on `AT6668`
- default UART interface at `115200 bps @ 8N1`
- `NMEA0183` output
- output frequency `1 Hz (max 10 Hz)`

However, that documentation does **not** publish any M5Stack-specific initialization sequence for:

- unlocking higher-rate `DHV`
- enabling periodic binary `NAV-PV`
- switching from a text-only mode into a vendor-specific mixed binary/text mode beyond the standard CASIC command path

### 9.2 M5Stack community evidence

Source artifact: `P2`

A public M5Stack community thread reports low-speed position-jump behavior on an `ATGM336H-6N (AT6668)` module running at `10 Hz`. This does not prove anything about `DHV` directly, but it is relevant because it shows that other users have observed behavior quirks on the same receiver family in practical logging use.

No public workaround in that thread documents a special M5Stack-only initialization sequence for:

- increasing `DHV` output rate
- enabling periodic `NAV-PV`

### 9.3 Public AT6668 / M5Stack user writeups

Source artifacts: `P3`, `P4`

Public user writeups around the M5Stack AT6668 GPS unit confirm that:

- `PCAS03` is the expected public mechanism for enabling NMEA sentences
- `DHV` is a known exposed sentence on this receiver family

Those sources are useful because they confirm that the investigation followed the same public command family seen by other users. They do **not** provide a documented hidden sequence for unlocking `DHV` at `10 Hz`, nor do they document a M5Stack-specific binary enable path for periodic `NAV-PV`.

One of the referenced public writeups explicitly notes uncertainty around the PCAS command set and the lack of a complete protocol specification in that context. That is consistent with the outcome of this investigation: the public command path is sufficient to observe the behavior, but not sufficient to unlock a working high-rate alternative velocity channel on the tested module.

### 9.4 Public-scan conclusion

Observed conclusion:

- no documented public M5Stack-specific initialization sequence was found that unlocks higher-rate `DHV`
- no documented public M5Stack-specific initialization sequence was found that enables periodic `NAV-PV` beyond the tested CASIC path

Classification:

- **Proven observation**: no such public sequence was found in the scanned sources
- **Engineering inference**: the measured behavior is more consistent with a module firmware or product-configuration limitation on the tested unit than with a missing publicly documented setup step

This wording is intentionally narrow. It does **not** claim a universal silicon limitation for all AT6668-based products.

## 10. Production Decision

The production decision is:

- keep `gps_sog_kmh` as the sole production velocity source
- keep `DHV` and `NAV-PV` fields only for diagnostics
- keep the `202 B` record because the additional diagnostics have already proven their value

This decision is pragmatic:

- it preserves the working `10 Hz` production path
- it avoids further development time on channels that did not become viable
- it preserves the evidence needed for vendor escalation and future module comparisons

## 11. Vendor Questions / Requested Clarifications

The following clarifications are requested from M5Stack:

1. Is `DHV` intentionally rate-limited to approximately `1 Hz` on this module firmware, even when the main positioning path is configured for approximately `10 Hz`?
2. Is periodic `NAV-PV` unsupported on this product, disabled in this firmware build, or dependent on a different command sequence than the tested `CFG-PRT -> CFG-MSG` path?
3. Does this module expose any other supported high-rate velocity channel suitable for production use at approximately `10 Hz`?
4. If periodic `NAV-PV` is supported, what is the correct enable sequence and expected ACK behavior for this module specifically?

## Appendix A — Proven observations vs engineering inference

### Proven observations

- Module-reported speed tracks `kf_vel` better than position-derived speed in `tel_11`
- Outdoor static logs do not show GPS-driven stationary-gate failures
- Main GPS fix path updates at approximately `10 Hz`
- `DHV` updates at approximately `1 Hz`
- `CFG-PRT` is ACKed
- `CFG-MSG` for periodic `NAV-PV` is NACKed
- No periodic `NAV-PV` stream appears after the tested enable attempt

### Engineering inference

- The tested AT6668 module/firmware likely imposes a practical `DHV` rate limitation on this path
- The tested AT6668 module/firmware likely does not support periodic `NAV-PV` through the tested command sequence

## Appendix B — Record-size rationale

`v1.4.2` record size: `164 B`  
`v1.5.2` record size: `202 B`  
Increase: `38 B`

Those `38 B` are retained because they provide:

- source visibility
- source-timing visibility
- deterministic replay support
- vendor-facing, portable evidence

Without them, the firmware would still run, but this investigation would have been materially weaker and much harder to defend to a third party.

## Appendix C — Reference artifacts

| ID | Filename | Description |
|---|---|---|
| `A1` | `tel_11_speed_compare.txt` | Static speed-channel comparison report |
| `A2` | `tel_20_static.txt` | Outdoor static validation report |
| `A3` | `tel_21_static.txt` | Outdoor static validation report |
| `A4` | `tel_20_gate.txt` | Outdoor static gate debug report |
| `A5` | `tel_21_gate.txt` | Outdoor static gate debug report |
| `M0` | `Telemetria.json` | Initial MQTT capture showing DHV-selected symptom bursts |
| `M1` | `Telemetria1.json` | MQTT capture used to estimate GPS fix vs DHV update rate |
| `M2` | `Telemetria2.json` | MQTT capture after explicit `PCAS03` test |
| `M3` | `Telemetria3.json` | MQTT capture proving `CFG-PRT` ACK and `CFG-MSG` NACK |
| `R1` | `Multimode_satellite_navigation_receiver_cn.pdf` | Receiver protocol reference consulted during the investigation |
| `R2` | `AT6668.pdf` | Chip/module reference consulted during the investigation |
| `R3` | `MAX2659.pdf` | Front-end reference consulted during the investigation |
| `P1` | `Atomic GPS Base v2.0 product PDF` | Public M5Stack product documentation for the AT6668-based Atomic GPS Base v2.0 |
| `P2` | `M5Stack Community thread: ATGM336H-6N (AT6668) position jumps below 30 km/h` | Public community evidence of practical AT6668 behavior quirks |
| `P3` | `Qiita: M5Stack GPS Unit v1.1 (AT6668)` | Public AT6668 / PCAS03 / DHV user writeup |
| `P4` | `Qiita: pygps2 NMEA parser article with AT6668 DHV coverage` | Public confirmation that `DHV` is a known exposed sentence on this receiver family |

Raw artifacts are retained by the author and can be shared with M5Stack on request.
