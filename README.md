# esp32-motorsport-telemetry-rtos-eskf
Dual-core FreeRTOS telemetry firmware for ESP32-S3 with 50Hz ESKF sensor fusion and MoTeC i2 Pro integration.

# ESP32 Telemetria

![Firmware](https://img.shields.io/badge/firmware-v1.0.0-blue)
![Platform](https://img.shields.io/badge/platform-ESP32--S3-informational)
![License](https://img.shields.io/badge/license-GPL--3.0-green)

> Motorsport telemetry system for the **M5Stack Atom S3** (ESP32-S3).
> 50 Hz IMU · GPS fusion · Error-State Kalman Filter · SD logging · MoTeC i2 Pro export

![Dashboard – full lap raw vs ESKF](img/Giro_completo_raw.jpeg)

---

## Overview

ESP32 Telemetria is a self-contained data-acquisition unit that fits in your hand and logs professional-grade telemetry at 50 Hz. It fuses a 6-DOF IMU (MPU-6886) with GPS to estimate position, velocity, and heading through an Error-State Kalman Filter running in real time on the Atom S3's dual-core ESP32-S3.

Recorded sessions are stored in a compact binary format on a micro-SD card and can be post-processed through a Python toolchain that outputs interactive dashboards or native **MoTeC i2 Pro** log files — the same format used by professional motorsport engineers.

**Current firmware:** v1.0.0

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| Main board | [M5Stack Atom S3](https://shop.m5stack.com/products/atoms3-dev-kit) | — |
| IMU | MPU-6886 (built-in) | I2C |
| GPS | ATGM336H-6N | UART1 (pins 1/2), 115200 baud |
| Storage | Micro-SD | SPI (pins 5–8) |
| Display | Built-in LCD | M5Unified |

---

## Features

- **50 Hz IMU pipeline** — Madgwick AHRS → gravity removal → ENU linear acceleration
- **Ellipsoid hard/soft-iron calibration** for the accelerometer (derived from tumble-test dataset)
- **Error-State Kalman Filter (ESKF2D)** — 5-state `[px, py, vx, vy, θ]` fused with GPS at ~10 Hz
- **Shadow ESKF_6D** — 6-state filter with online gyro-bias estimation, running in parallel for validation
- **GPS outlier rejection** via Mahalanobis innovation gate (χ² threshold 11.83, 3-DOF)
- **ZARU stationarity detection** — variance of ωz over a 50-sample window triggers zero-angular-rate updates for thermal bias correction
- **Async SD logging** via FreeRTOS queue — 122 bytes/record at 50 Hz, auto lap splitting
- **MQTT publishing** at 10 Hz over Wi-Fi for live telemetry monitoring
- **MoTeC i2 Pro export** — native `.ld` format, 12 channels (8 @ 50 Hz + 4 @ 10 Hz)
- **Interactive dashboard** built with Plotly/Dash for post-session analysis

---

## Screenshots

| Raw vs ESKF – roundabout | Tight corner (filtered) |
|--------------------------|------------------------|
| ![Rotonda raw vs ESKF](img/Rotonda_raw_eskf.jpeg) | ![Curva stretta filtered](img/Curva_stretta_filtered.jpeg) |

| Fast corner (raw) | Yaw · Roll · Grip |
|-------------------|-------------------|
| ![Curva veloce raw](img/Curva_veloce_raw.jpeg) | ![Yaw roll grip](img/Yaw_roll_grip_in_curva.jpeg) |

| MoTeC i2 Pro |
|--------------|
| ![MoTeC](img/Motec.png) |

---

## Quick Start

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- Python ≥ 3.9

```bash
pip install -r Tool/requirements.txt
```

### Build & Flash

```bash
# Build only
pio run

# Build and upload
pio run --target upload

# Open serial monitor
pio device monitor

# Upload and monitor in one shot
pio run --target upload && pio device monitor
```

### Flash pre-built binary

If you just want to flash the firmware without setting up PlatformIO, use `esptool`:

```bash
pip install esptool

esptool.py --chip esp32s3 --port COM3 --baud 921600 write_flash 0x0 firmware_merged.bin
```

Replace `COM3` with your port (`/dev/ttyUSB0` on Linux/macOS). The merged binary is provided as a release asset on the [Releases](../../releases) page.

### Wi-Fi & MQTT Configuration

All runtime configuration is read from `wifi_config.txt` placed at the **root of the SD card** (not tracked in git — see `wifi_config.example.txt` for the template):

```ini
SSID=MyNetwork
PASSWORD=mypassword
SSID2=FallbackNetwork      # optional second network
PASSWORD2=fallback_pass

MQTT_BROKER=192.168.1.10   # optional — MQTT disabled if absent
MQTT_PORT=1883             # optional, default 1883
MQTT_TOPIC=vehicle/data    # optional, default: telemetry/<unit_id>/data
```

The device works fully offline without Wi-Fi — SD logging is independent of network availability.

---

## On-Device Controls (BtnA / Screen Tap)

The Atom S3 has a single capacitive button (BtnA) that covers the entire LCD surface. All user interaction happens through tap patterns and a long press, detected via a 400 ms debounce window.

### During Boot

| Gesture | When | Effect |
|---------|------|--------|
| **Single tap** | Wi-Fi connection screen | Skips Wi-Fi, boots in SD-only offline mode |
| **Double tap** | Red "INSERT SD!" screen | Bypasses SD requirement, boots with MQTT only |

### During Runtime

| Gesture | Effect |
|---------|--------|
| **Single tap** | Display sleep toggle (see below) |
| **Double tap** | Toggles lap recording. Idle → 5 s countdown → recording (blue). Double tap again → back to idle |
| **Triple tap** | Toggles Wi-Fi on/off. A green/red circle in the bottom-right corner shows current state |
| **Long press (≥ 1.5 s)** | Manual IMU recalibration — 2 s static window, yellow screen with countdown |

### Display Sleep

A single tap shows **"DISPLAY OFF?"** for 5 seconds. A second tap within the window confirms and puts the LCD to sleep (backlight off + controller sleep). Any tap while sleeping wakes the display immediately. Double and triple taps during the confirmation window cancel it and execute normally.

Turning the display off during a session reduces internal temperature and consequently the thermal drift of the MPU-6886, improving gyroscope bias stability over long runs. Critical alarms (GPS lost, SD write error) automatically wake the display regardless of sleep state.

### Display States

| Screen Color | Meaning |
|--------------|---------|
| **Black** | Idle — showing live IMU + GPS data, not recording |
| **Red** ("DISPLAY OFF?") | Sleep confirmation pending — waiting for second tap (5 s) |
| **Red** (static) | 5 s countdown to lap start |
| **Blue** | Lap recording active |
| **Red** (flashing ~2.5 Hz) | Alarm: GPS lost or SD write error |
| **Yellow** (with countdown) | Recalibration in progress |
| **Screen off** | Display sleeping — all acquisition and logging continue |

### Wi-Fi Indicator

A small circle in the bottom-right corner of the LCD:
- **Green** — Wi-Fi connected
- **Red** — Wi-Fi disabled

---

## Typical Session Workflow

### 1 — Hardware setup

- Insert a FAT32 micro-SD card (≤ 16 GB, Class 10 or faster).
- Mount the device with the GPS antenna facing upward and unobstructed. The ATGM336H-6N tracks up to ~30 satellites across multiple constellations with a clear sky view — avoid metal enclosures.
- Place `wifi_config.txt` at the SD card root for MQTT live streaming (optional).

### 2 — Boot and GPS fix

- Power on. The device runs a **2-second static calibration** — keep it stationary during this window.
- Wait for GPS fix: **~10 s** warm start, **2–5 min** cold start. Satellite count and HDOP are shown on the idle screen.
- If fix quality is poor, trigger a **manual recalibration** with a long press while stationary.

### 3 — Recording

- **Double tap** to start a lap (5 s countdown, then blue screen).
- **Double tap** again to stop and return to idle.
- Multiple laps can be recorded in the same session without rebooting — each double-tap pair is tracked by the `lap` counter in the binary file.

Each session is saved as `tel_XX.bin` on the SD card, where `XX` increments at every boot.

### 4 — Extract and convert

```bash
python Tool/bin_to_csv.py tel_XX.bin
```

The converter detects the firmware version from the file header and launches an interactive split menu for large multi-lap files. Output: one or more `tel_XX_partN.csv` files, one row per 20 ms sample.

### 5 — Analyse

```bash
python Tool/dashboard.py
# or: python Tool/dashboard.py tel_XX_part1.csv
```

---

## Post-Processing Tools

All tools are in `Tool/` and accept `.bin` files directly or `.csv` files from `bin_to_csv.py`.

### 1 — Binary to CSV

```bash
python Tool/bin_to_csv.py <file.bin>
```

### 2 — Interactive Dashboard

```bash
python Tool/dashboard.py
# Opens at http://127.0.0.1:8050
```

A WebGL-accelerated viewer for post-session analysis. All charts are linked: hovering or clicking on any graph moves a cursor across all four simultaneously.

#### Charts

| Chart | Description |
|-------|-------------|
| **GPS Map** | Full track on a satellite map, colour-coded by speed. Toggle **ESKF overlay** to compare raw GPS scatter against the fused Kalman trajectory |
| **G-G Diagram** | Lateral vs longitudinal acceleration (friction circle). Toggle **3D** to add the vertical axis (Az) |
| **Yaw Rate** | Gyroscope Z over time or distance. Toggle **Roll Rate** (Gx) and **Grip** (2-D acceleration magnitude) as overlays |
| **Velocity Timeline** | Speed split into brake (red) / accel (green) / coast (grey) traces. Used for sector selection |

#### Controls

- **Raw Data** — switch from EMA-filtered (α = 0.06, τ ≈ 313 ms) to zero-latency post-Madgwick data
- **X-Axis: Distance** — switch Yaw and Timeline X-axis from time (s) to distance (m)
- **3D G-G** — toggle G-G between 2-D and 3-D
- **G-G Fixed Scale** — lock axes to the full-session envelope for cross-sector comparison
- **Show Roll Rate / Show Grip** — toggle Yaw chart overlays

#### Sector Selection

1. **Click on the Timeline** to set Start (cyan line), click again for End (yellow line).
2. **Apply Sector** — all charts rebuild on the selected range, KPIs update.
3. Alternatively, type timestamps or distances into the **Start / End** fields and press Apply.
4. **Reset** to return to the full session.

### 3 — MoTeC i2 Pro Export

```bash
python Tool/motec_exporter.py tel_XX.bin
python Tool/motec_exporter.py tel_XX.bin --venue "Mugello"
python Tool/motec_exporter.py tel_XX.csv -o session.ld
```

Produces a native `.ld` file openable in **i2 Pro** or **i2 Standard**:

| Channel | Rate | Unit |
|---------|------|------|
| Ground Speed (ESKF) | 50 Hz | km/h |
| G Force Long / Lat / Vert | 50 Hz | G |
| Yaw / Roll / Pitch Rate | 50 Hz | deg/s |
| Sensor Temperature | 50 Hz | °C |
| GPS Latitude / Longitude | 10 Hz | deg |
| GPS Altitude | 10 Hz | m |
| GPS Speed | 10 Hz | km/h |

---

## Architecture

### FreeRTOS Tasks

| Task | Core | Priority | Responsibility |
|------|------|----------|----------------|
| `Task_I2C` | 0 | 3 | Reads MPU-6886 every 20 ms, pushes to `imuQueue` (overwrite) |
| `Task_GPS` | 0 | 2 | Polls UART1, parses NMEA, writes to `gpsDatiCondivisi` (mutex) |
| `Task_Filter` | 1 | 2 | Consumes IMU queue, runs full signal chain, drives ESKF |
| `Task_SD_Writer` | 1 | 1 | Async SD logging via queue, flushes every 50 samples |
| `loop()` | 1 | — | Display refresh (5 Hz) · MQTT publish (10 Hz) |

### Signal Processing Pipeline

Each 20 ms cycle inside `Task_Filter`:

```
IMU raw
  └─ 1. Ellipsoid calibration (hard/soft-iron)
  └─ 2. Geometric alignment (frame rotation → vehicle axes)
  └─ 3. Madgwick AHRS (quaternion, adaptive β)
  └─ 4. Gravity removal → linear acceleration (ENU)
  └─ 5. Bivio (data split)
        ├─ Raw path → ESKF predict/correct
        └─ EMA path (α=0.06, τ≈313ms) → display · MQTT · SD
  └─ 6. Stationarity detection (var(ωz), 50-sample window) → ZARU
  └─ 7. ESKF predict @ 50 Hz
  └─ 8. ESKF correct on GPS fix @ ~10 Hz
        (sequential: position → speed → course-over-ground)
  └─ 9. Shadow ESKF_6D predict/correct (parallel validation)
```

### Kalman Filter (`src/eskf.h`)

Two header-only implementations using `BasicLinearAlgebra`:

| Filter | States | Role |
|--------|--------|------|
| **ESKF2D** | `[px, py, vx, vy, θ]` | Primary — logged as `kf_*` |
| **ESKF_6D** | `[px, py, vx, vy, θ, b_gz]` | Shadow + gyro-bias estimation — logged as `kf6_*` |

Key design choices:
- Dynamic GPS noise: `R = 0.05 × HDOP²`
- Mahalanobis innovation gate — rejects outliers before each sequential update
- Joseph-form covariance update for numerical stability
- Velocity/course updates gated at > 5 km/h

### Why two filters?

The firmware runs **ESKF2D** and **ESKF_6D** in parallel on the same IMU+GPS data. This is intentional A/B testing, not redundancy.

The **5D filter** relies on an external ZARU to subtract gyro thermal drift at standstill. Simple, well-tuned, primary filter since v0.9.0.

The **6D filter** adds `b_gz` as a sixth state, estimated online via the cross-covariance between heading and bias — theoretically superior as it learns drift even in motion without heuristic conditions. Not yet validated against the 5D on real track data.

Both log to SD at 50 Hz. Only the 5D drives display and MQTT. A future version will promote the 6D or remove it based on offline comparison.

---

## Calibration

Boot calibration (2 s static window, 100 samples) handles gyro bias automatically. Accelerometer calibration is hardcoded — derived from a tumble test and validated against a pendulum fixture.

To recalibrate:

1. Collect a new tumble-test recording (rotate through all orientations)
2. Convert with `bin_to_csv.py`
3. Refit the ellipsoid offline (see `Dati telemetria/Tumble Test/`)
4. Update `CALIB_B[3]` and `CALIB_W[3][3]` in `src/Telemetria.ino` around line 735

| Report | Description |
|--------|-------------|
| `Reports/IMU_Calibration_and_Validation_Report (Final).pdf` | Full calibration methodology and pendulum validation |
| `Reports/report_ellipsoid_fitting_en.pdf` | Ellipsoid fitting algorithm and tumble-test results |
| `Reports/MPU6886_ThermalDrift_Report_v3.1.pdf` | Thermal drift characterisation of the MPU-6886 |

---

## Binary Log Format

`RecordTelemetria` — 122 bytes per record at 50 Hz (`__attribute__((packed))`):

| Field | Type | Description |
|-------|------|-------------|
| `t_ms` | uint32 | IMU hardware timestamp (ms) |
| `ax..temp_c` | 7 × float | EMA-filtered accel (G) + gyro (deg/s) + temperature (°C) |
| `lap` | uint8 | Lap counter |
| `gps_lat`, `gps_lon` | 2 × double | GPS coordinates (WGS84) |
| `gps_speed_kmh`, `gps_alt_m` | 2 × float | GPS ground speed and altitude |
| `gps_sats`, `gps_hdop` | uint8 + float | Satellite count and HDOP |
| `kf_x..kf_heading` | 4 × float | ESKF2D output: position (m), speed (m/s), heading (rad) |
| `raw_ax..raw_gz` | 6 × float | Post-Madgwick IMU (zero latency, bypasses EMA) |
| `kf6_x..kf6_bgz` | 5 × float | ESKF_6D shadow output + estimated gyro bias |

Legacy 78-byte format (firmware < v0.9.8) is supported by all post-processing tools.

---

## MQTT Live Telemetry

When Wi-Fi and an MQTT broker are configured, the device publishes JSON at **10 Hz**.

### Topic

Default: `telemetry/<unit_id>/data` where `<unit_id>` is derived from the last 3 bytes of the chip MAC (e.g. `TelUnit_A3F2C1`). Override with `MQTT_TOPIC=` in the config file.

### Telemetry payload (10 Hz)

```json
{
  "ax": -0.12, "ay": 0.34, "az": 1.00,
  "gx":  0.21, "gy": -0.10, "gz": 1.85,
  "lap": 1,
  "lat": 45.123456, "lon": 9.123456,
  "spd": 87.3, "alt": 142.0, "sats": 14, "hdop": 0.9,
  "kfx": 12.34, "kfy": -5.67, "kfv": 24.23, "kfh": 1.57
}
```

| Field | Unit | Description |
|-------|------|-------------|
| `ax` `ay` `az` | G | EMA-filtered linear acceleration (gravity removed) |
| `gx` `gy` `gz` | °/s | EMA-filtered gyroscope rates |
| `lap` | — | `1` while recording, `0` otherwise |
| `lat` `lon` | ° | GPS coordinates (WGS84) |
| `spd` | km/h | GPS ground speed |
| `alt` | m | GPS altitude |
| `sats` / `hdop` | — | Satellites tracked / horizontal dilution of precision |
| `kfx` `kfy` | m | ESKF2D position in ENU frame |
| `kfv` | m/s | ESKF2D fused speed |
| `kfh` | rad | ESKF2D estimated heading |

### Heartbeat (every 60 s)

```json
{
  "heartbeat": true, "records": 18000, "uptime_s": 360,
  "gps_stale": false, "sd_err": false,
  "stk_filter": 1248, "stk_i2c": 876, "stk_sd": 1104
}
```

`records` = samples written to SD · `gps_stale` = no NMEA in last 2 s · `sd_err` = write error occurred · `stk_*` = FreeRTOS stack high-water marks (bytes free).

### SD low-space warning (one-shot at boot)

```json
{ "warning": "SD_LOW_SPACE", "free_mb": 42 }
```

---

## Repository Structure

```
ESP32-Telemetria/
│
├── src/
│   ├── Telemetria.ino          # Main firmware (~2 200 lines)
│   └── eskf.h                  # Header-only ESKF library (ESKF2D + ESKF_6D)
│
├── Tool/
│   ├── bin_to_csv.py           # Binary → CSV converter with interactive lap split
│   ├── dashboard.py            # Plotly/Dash interactive telemetry viewer
│   ├── motec_exporter.py       # MoTeC i2 Pro .ld exporter
│   └── requirements.txt        # Python dependencies
│
├── CSV/                        # Sample recorded sessions (ready to use with the tools)
│   ├── tel_47.bin / .csv / .ld
│   └── tel_48.bin / .csv / .ld
│
├── Reports/                    # Technical documentation (PDF)
├── img/                        # Screenshots used in this README
├── Datasheet Sensori/          # MPU-6886, ATGM336H-6N, MAX2659 datasheets
│
├── platformio.ini
├── wifi_config.example.txt
└── README.md
```

> `.pio/` (build artefacts) and `wifi_config.txt` (credentials) are excluded via `.gitignore`.

---

## Dependencies

Managed by PlatformIO (`platformio.ini`):

| Library | Version | Use |
|---------|---------|-----|
| M5Unified | ^0.2.2 | Hardware abstraction (display, IMU, power) |
| PubSubClient | ^2.8 | MQTT client |
| TinyGPSPlus | ^1.0.3 | NMEA sentence parser |
| BasicLinearAlgebra | ^3.6 | Matrix operations for ESKF |

---

## Troubleshooting

### SD card not detected at boot

Red **"INSERT SD!"** screen, retries every 2 seconds.

- **Dirty contacts** — remove and reinsert, wait for automatic retry.
- **Wrong format** — must be **FAT32**. exFAT is not supported.
- **Card too large** — use ≤ 16 GB; larger cards are often exFAT by default.

Double-tap BtnA to bypass and boot in MQTT-only mode.

### Poor GPS fix or no fix

- Move to an open area with clear sky view.
- Cold start: 2–5 min. Warm start: ~10 s.
- Check HDOP on the idle screen — values above 2.0 indicate poor geometry.

### MQTT not connecting

SD logging is unaffected by network state. If Wi-Fi is unavailable at boot the device starts offline; if the connection drops mid-session it reconnects automatically in the background. Verify SSID, password, and broker IP in `wifi_config.txt`.

### Dashboard slow on large files

Designed for single-lap analysis (~150 000 rows max before sluggishness). For full-session or multi-lap work use MoTeC i2 Pro or MATLAB.

### Calibration drift

If the device was moved during the 2-second boot tare, gyro bias will be off — symptoms are non-zero yaw rate at standstill or slow heading rotation while parked. Fix: long press (≥ 1.5 s) while stationary to re-run calibration.

---

## Known Limitations

### Kalman filter

- **ESKF_6D unvalidated** — runs as shadow only; not yet compared against the 5D on real track data.
- **Position drift on long sessions** — heading error accumulates between GPS corrections. ZARU compensates at standstill but does not replace a full GPS-denied navigation solution.

### GPS and map projection

- **AEQD origin from first GPS fix** — if the first fix has poor quality (cold start, high HDOP), the ESKF overlay on the map will be offset. A future binary header will store the origin fix with quality metadata.
- **GPS multipath is silent** — reflections appear as position spikes or high HDOP in the data. The innovation gate rejects the worst outliers but moderate multipath degrades trajectory quality without warning.

### Hardware

- **`SD.begin()` blocks button polling** — presses during the SD retry may be missed. The bypass timeout has been extended to mitigate this; a hardware interrupt would fully solve it.
- **No calibration motion detection** — the firmware does not detect if the device is moved during the 2-second tare.

### Post-processing

- **Dashboard single-lap only** — sluggish above ~150 000 rows. Use MoTeC or MATLAB for full sessions.
- **No automatic lap splitting** — `bin_to_csv.py` splits interactively, not automatically by lap counter.

---

## Roadmap

| Item | Status | Notes |
|------|--------|-------|
| ESKF_6D validation | In progress | A/B comparison on recorded track sessions; 6D becomes primary if it outperforms |
| Binary session header | Planned | Store GPS origin fix + HDOP + firmware version at file start; fixes AEQD offset |
| Hardware interrupt for BtnA | Planned | `attachInterrupt()` on GPIO 41 — captures clicks during blocking SD calls |
| Shake detection at boot | Planned | Detect motion during tare window and warn or auto-repeat |
| Dashboard multi-lap support | Under evaluation | Requires downsampling or virtualised rendering above 150k rows |

---

## Contributing

This project is primarily a portfolio and learning exercise, but feedback is welcome.

If you spot a bug, have a suggestion, or want to discuss the signal processing or filter design, open a [GitHub Issue](../../issues). Constructive criticism on the math or engineering choices is especially appreciated.

Pull requests are welcome for bug fixes or documentation improvements. For changes to `eskf.h` or the core signal pipeline, include a brief description of the validation performed (ideally with recorded data).

---

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).
