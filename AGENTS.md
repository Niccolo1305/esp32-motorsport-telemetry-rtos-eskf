# Repository Guidelines

This file is the repository-local operating guide for Codex and other coding
agents. It replaces the old ignored `CLAUDE.md`; keep this file current when
build commands, data formats, hardware targets, or validation expectations
change.

## Project Overview

This repository contains an ESP32-S3 motorsport telemetry system for M5Stack
AtomS3 and AtomS3R devices. The firmware samples IMU data at 50 Hz, ingests GPS
position and speed at about 10 Hz, runs Madgwick plus ESKF sensor fusion, writes
binary logs to SD, publishes MQTT live telemetry, and supports Python
post-processing for CSV, dashboards, MoTeC export, and offline SITL replay.

Current production targets:

- `m5stack-atoms3`: AtomS3 with MPU-6886 IMU, current firmware baseline
  `v1.5.2-clean`, 202-byte telemetry records.
- `m5stack-atoms3r`: AtomS3R with BMI270 IMU plus BMM150 magnetometer, current
  firmware baseline `v1.8.7-atoms3r`, 256-byte v6 telemetry records.

License: GPL-3.0.

## Read First

For non-trivial work, gather context in this order:

1. `README.md` for product behavior, hardware targets, binary formats, and
   current status.
2. `docs/PROJECT_STRUCTURE.md` for the intended folder layout.
3. `platformio.ini` for build environments and active compile flags.
4. `src/Telemetria.ino` for the authoritative firmware changelog and pipeline
   overview.
5. `src/types.h` before changing binary log structs or file headers.
6. The relevant Python tool under `tools/` before changing telemetry schemas,
   parsers, or offline analysis.

Do not treat chat history as durable project truth. If a decision matters after
the current turn, put it in a tracked document or code comment where future
contributors will find it.

## Working Tree Rules

The repository often contains local captures, generated reports, evidence packs,
and partially staged cleanup. Before editing, inspect `git status --short`.
Do not delete, move, regenerate, or normalize telemetry data, PDFs, screenshots,
or evidence folders unless the user explicitly asks for that cleanup.

Prefer small, reviewable edits. Keep unrelated refactors out of feature fixes.
When touching files with user changes, read the nearby code carefully and work
with the existing changes instead of reverting them.

## Build And Flash

PlatformIO is the canonical firmware build system. Run commands from the
repository root.

```bash
# Production firmware
pio run -e m5stack-atoms3
pio run -e m5stack-atoms3r

# Flash production firmware
pio run -e m5stack-atoms3 -t upload
pio run -e m5stack-atoms3r -t upload

# Serial monitor
pio device monitor
```

Sensor bench firmware environments:

```bash
pio run -e atoms3_bench
pio run -e atoms3r_bench
pio run -e atoms3_bench -t upload
pio run -e atoms3r_bench -t upload
```

CAN helper firmware environments:

```bash
pio run -e can_sniffer
pio run -e can_obd2_validator
```

## Python Tools

Install Python dependencies from the repository root:

```bash
pip install -r tools/requirements.txt
```

Common tool commands:

```bash
# Convert binary log to CSV
python tools/bin_to_csv.py <input.bin> <output.csv>

# Run offline SITL replay from BIN or CSV
python tools/sitl_hal/sitl_replay.py <input.bin> --output <output_sitl.csv>
python tools/sitl_hal/sitl_replay.py <input.csv> --output <output_sitl.csv>

# Export to MoTeC i2 Pro
python tools/motec_exporter.py <input.csv> --output <output.ld>

# Start the local telemetry browser/API, when needed
python tools/telemetry_server.py
```

`tools/script/schema_guard.py` is a compatibility guard used by legacy one-off
analysis scripts to reject newer v5/v6 CSV layouts. It is not a complete
firmware/Python struct alignment validator.

## Validation Entrypoint

Use `tools/validate.ps1` as the first-choice local validation entrypoint. It
does not replace deeper hardware or drive-log validation; it makes the common
checks repeatable from the repository root.

```powershell
.\tools\validate.ps1 -Target docs      # git diff whitespace checks
.\tools\validate.ps1 -Target python    # Python syntax check for tracked/unignored tools
.\tools\validate.ps1 -Target atoms3    # PlatformIO build for AtomS3
.\tools\validate.ps1 -Target atoms3r   # PlatformIO build for AtomS3R
.\tools\validate.ps1 -Target firmware  # both production firmware builds
.\tools\validate.ps1 -Target bench     # both IMU bench builds
.\tools\validate.ps1 -Target can       # both CAN helper builds
.\tools\validate.ps1 -Target all       # docs + python + production firmware
```

## Architecture

Main firmware modules:

- `src/Telemetria.ino` wires hardware initialization, display, SD logging,
  MQTT, button handling, and file header creation.
- `src/types.h` owns `TelemetryRecord`, `FileHeader`, `FileHeaderV6`, `GpsData`,
  and `ImuRawData`.
- `src/filter_task.cpp` owns the 50 Hz signal-processing and navigation
  pipeline.
- `src/imu_task.cpp` samples the selected `IImuProvider`.
- `src/gps_task.cpp` ingests NMEA GPS plus DHV/NAV-PV diagnostics.
- `src/sd_writer.cpp` and `src/storage_manager.*` own async SD writes and
  backend-specific integrity behavior.
- `tools/bin_to_csv.py` is the shared binary-log parser for downstream Python
  tools.
- `tools/sitl_hal/sitl_replay.py` replays the firmware pipeline offline from
  logged sensor-frame data.

FreeRTOS task layout:

- `Task_I2C`, core 0, 50 Hz: IMU acquisition through `IImuProvider`.
- `Task_GPS`, core 0: GPS and diagnostic serial ingest.
- `Task_Filter`, core 1, 50 Hz: calibration, Madgwick, ESKF, ZARU, NHC, EMA,
  and SD record packing.
- `Task_SD_Writer`, core 1: async binary SD sink.
- `loop()`, core 1: display, MQTT, and button interactions.

Shared IPC objects include `imuQueue`, `sd_queue`, `telemetry_mutex`, and
`gps_mutex`. Keep mutex-protected shared state updates short; the filter path is
timing-sensitive.

## Firmware Invariants

Preserve these rules unless the user explicitly asks for a design change and
the matching parser/tooling updates are included in the same work:

- AtomS3R production runtime must not use `M5.Imu.getAccelData`,
  `M5.Imu.getGyroData`, `M5.Imu.getTemp`, or `M5.Imu.getMag`.
- On AtomS3R, `M5Unified` is board/display/power infrastructure only.
  `cfg.internal_imu = false` must be set before `M5.begin(cfg)`.
- `Bmi270Provider` owns Bosch BMI270/BMM150 bring-up on `M5.In_I2C` and must
  output sensor-faithful native data. Do not hide axis remaps inside the
  provider.
- `imu_axis_remap.h` and the filter/calibration pipeline own chip-frame to
  pipeline-frame conversion.
- Vehicle-frame signals after `rotate_3d()` use `X=forward`, `Y=left`, `Z=up`.
- ESKF position is local ENU from the first GPS fix.
- ZARU is the primary source of gyro-bias correction for the main navigation
  path.
- EMA is display/log smoothing only. Do not feed EMA output back into ZARU,
  variance estimation, or GPS correction.
- GPS control uses NMEA SOG as the production speed path. DHV and passive
  NAV-PV channels are diagnostics unless a deliberate design change says
  otherwise.
- AtomS3R magnetometer data is currently logging-only. No 9-DoF heading fusion
  is active.

## Binary Format Invariants

Binary compatibility is part of the product. Any change to C++ telemetry structs
must be reflected in `tools/bin_to_csv.py`, SITL, MoTeC export, dashboard/server
readers, README format tables, and any affected evidence tooling.

Current production layouts:

- AtomS3: base `FileHeader` is 80 bytes; `TelemetryRecord` is 202 bytes.
- AtomS3R v6: `FileHeaderV6` is 256 bytes; `TelemetryRecord` is 256 bytes and
  sector-aligned.
- Legacy records supported by `tools/bin_to_csv.py`: 122, 127, 155, 164, 190,
  202, 215, 224, 242, and 256 bytes.

AtomS3R v6 record data must preserve acquisition truth:

- `bmi_raw_*` are native BMI270 register values.
- `bmi_acc_*_g` and `bmi_gyr_*_dps` are provider-scale physical values before
  pipeline remap.
- BMM150 physical channels are Bosch-compensated microtesla values.
- `record_magic`, `record_seq`, and `crc16` are integrity fields. Keep parser
  resync and preallocated-log EOF behavior in sync with firmware writes.

## Operational Checklists

Use these checklists when a change touches a high-risk surface. They are not
generic process; they describe the minimum files and checks that keep this
firmware, its binary logs, and the Python tools aligned.

### Binary Format Change

Use this when changing `TelemetryRecord`, `FileHeader`, `FileHeaderV6`, record
sizes, field order, field units, CRC/magic/sequence behavior, calibration
records, or CSV column names.

- Update `src/types.h` first, including `static_assert` sizes and comments.
- Update record packing in `src/filter_task.cpp`, calibration-record injection
  in `src/calibration.cpp` when applicable, and header creation in
  `src/Telemetria.ino`.
- Update SD integrity assumptions in `src/sd_writer.cpp` and
  `src/storage_manager.*` when record size, CRC, magic, sequence, or
  preallocation behavior changes.
- Update `tools/bin_to_csv.py` format registry, headers, preamble output,
  plausibility checks, resync behavior, and legacy fallback rules.
- Update downstream readers that consume parsed columns: at minimum
  `tools/sitl_hal/sitl_replay.py`, `tools/motec_exporter.py`, and
  `tools/telemetry_server.py` when their input columns or metadata change.
- Update `README.md` binary-format tables, current-version notes, and any
  evidence/report text that claims exact sizes or field semantics.
- Validate with both production builds: `tools/validate.ps1 -Target firmware`.
- Run parser smoke checks on representative logs when available. If no
  representative `.bin` exists for the new layout, state that gap explicitly in
  the final response and describe the substitute validation.

### Pipeline, Filter, Or ESKF Change

Use this when changing `src/filter_task.cpp`, Madgwick/VGPL behavior, ESKF
prediction or correction, ZARU, NHC, gravity removal, frame semantics, GPS
freshness, or smoothing.

- Read the pipeline comment block in `src/Telemetria.ino` and update it when the
  processing order, source of truth, or logged diagnostics change.
- Preserve frame semantics: vehicle frame is `X=forward`, `Y=left`, `Z=up`, and
  ESKF position is local ENU from the first GPS fix.
- Keep EMA out of variance, ZARU, and correction inputs unless the work is an
  explicit design change.
- Keep the production GPS speed control path on NMEA SOG unless the work
  explicitly changes that policy and updates SITL and diagnostics with it.
- For AtomS3R, keep provider-native BMI/BMM acquisition truth separate from
  pipeline-remapped values.
- Update logged diagnostics when they are needed to explain or replay the new
  behavior offline.
- Mirror deterministic behavior in `tools/sitl_hal/sitl_replay.py` when the
  changed logic affects replayable navigation, GPS gating, ZARU flags, or error
  metrics.
- Validate with the affected production build. Prefer both production builds
  when shared pipeline code changes.
- If hardware or real-drive validation is needed, say what remains unvalidated
  and which log or bench run should close the gap.

### Python Parser Or Tool Change

Use this when changing `tools/bin_to_csv.py`, SITL replay, MoTeC export,
dashboard/server readers, static validators, analysis scripts, or generated
report scripts.

- Identify whether the tool is authoritative infrastructure or a one-off
  analysis helper. Keep one-off scripts from silently becoming format contracts.
- For parser changes, update `tools/bin_to_csv.py` first and keep legacy record
  support intact unless removal is the explicit goal.
- Reuse the shared parser registry instead of duplicating struct formats in
  downstream tools.
- Keep unit names and column aliases consistent with `README.md` and SITL
  compatibility helpers.
- Run the changed tool against a representative input when available.
- When no representative input exists, at least run
  `tools/validate.ps1 -Target python` and state the missing fixture or log
  clearly.
- Do not commit generated CSV, dashboard images, or report outputs unless they
  are intentionally part of an evidence pack or test fixture.

## Validation Expectations

Run the narrowest validation that matches the changed surface:

- Firmware-only changes: `tools/validate.ps1 -Target atoms3` and/or
  `tools/validate.ps1 -Target atoms3r`.
- Shared production firmware changes: `tools/validate.ps1 -Target firmware`.
- Bench firmware changes: `tools/validate.ps1 -Target bench` or the matching
  individual PlatformIO environment when a narrower build is enough.
- CAN helper changes: `tools/validate.ps1 -Target can` or the matching
  individual PlatformIO environment when a narrower build is enough.
- Binary layout changes: both production builds plus parser smoke checks on a
  representative `.bin` or a clearly documented reason no sample was available.
- Python parser/tool changes: run the changed script against representative
  input when available; otherwise run `tools/validate.ps1 -Target python` and
  state the test gap.
- Documentation-only changes: `tools/validate.ps1 -Target docs`.

If hardware flashing or real telemetry capture is required but not possible in
the current environment, say that directly and describe what was validated
instead.

## Planning Guidance

Use lightweight inline plans for ordinary fixes. For large refactors, binary
format changes, sensor-fusion changes, or migrations touching many files, create
or update a plan under `plans/NN-short-name.md`. Plans should state the target
behavior, files touched, validation commands, binary compatibility impact, and
rollback or recovery path.

Do not copy generic template process into this repository unless it helps this
firmware project. Prefer project-specific commands, invariants, and data-format
checks over broad workflow ceremony.
