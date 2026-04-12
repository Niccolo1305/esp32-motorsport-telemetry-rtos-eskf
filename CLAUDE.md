# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 motorsport telemetry system (M5Stack Atom S3) with 50 Hz IMU + 10 Hz GPS, real-time ESKF sensor fusion, binary SD logging, MQTT live telemetry, and Python post-processing tools (CSV export, Plotly dashboard, MoTeC i2 Pro exporter).

**Firmware version:** v1.3.2 | **License:** GPL-3.0

## Build & Flash

PlatformIO project targeting `m5stack-atoms3` (ESP32-S3, Arduino framework).

```bash
# Build firmware
pio run

# Build and upload
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor
```

No test framework is configured. Validation is done via logged data analysis with the Python tools.

## Python Tools

```bash
cd Tool
pip install -r requirements.txt

# Convert binary .tel/.bin to CSV (interactive lap splitting)
python bin_to_csv.py

# Launch interactive Plotly/Dash dashboard
python dashboard.py

# Export to MoTeC i2 Pro .ld format
python motec_exporter.py <input.csv> [--output out.ld]
```

## Architecture

### FreeRTOS Task Layout

| Task | Core | Priority | Rate | Role |
|------|------|----------|------|------|
| `Task_I2C` | 0 | 3 | 50 Hz | IMU acquisition (MPU-6886 via I2C) |
| `Task_GPS` | 0 | 2 | polling | NMEA parsing (ATGM336H via UART1) |
| `Task_Filter` | 1 | 2 | 50 Hz | 12-step signal processing + ESKF predict/correct |
| `Task_SD_Writer` | 1 | 1 | async | Binary SD logging via FreeRTOS queue |
| `loop()` | 1 | — | 100 Hz | Display (5 Hz), MQTT (10 Hz), button handling |

**IPC:** `imuQueue` (overwrite, I2C→Filter), `sd_queue` (FIFO, Filter→SD), `telemetry_mutex` (Filter→loop), `gps_mutex` (GPS→Filter/loop).

### Data Processing Pipeline (filter_task.cpp)

Full visual diagram in `IMU and GPS Data Processing-2026-04-12.pdf`.

**Acquisition:**
- MPU-6886 (I2C, 50 Hz exact via vTaskDelayUntil) → Task_I2C → xQueueOverwrite → Task_Filter
- AT6668 GNSS (UART1, 10 Hz) → Task_GPS → gps_mutex

**Calibration chain (every 20 ms cycle):**
1. Dynamic dt (Δtimestamp/1e6, guarded 0..1s)
2. Ellipsoidal cal: a_cal = W(a_raw - B) — hard/soft-iron
3. Geometric align: rotate_3D(φ,θ) chip→vehicle frame
4. Mounting bias removal (zeroed at rest)
5. Gyro bias + frame rotation → g_r [°/s], g_rad [rad/s]
6. Madgwick AHRS (dual-gate adaptive β) → quaternion + gravity vector
7. Gravity removal: lin_a = a_r - grav(q)

**DATA FORK — after step 7, two parallel paths:**

EMA path (smoothed, for telemetry output):
- 8: EMA filter α=0.06 (τ≈313ms) on accel+gyro
- 9: Circular variance buffer (50 samples, 1s) → var_gz, mean_g*
- 10: Thermal bias snapshot (frozen before output)
- → shared_telemetry (mutex) → MQTT 10Hz + LCD 5Hz + SD record

RAW path (zero-latency, for navigation):
- 11: GPS snapshot (static cache, epoch freshness check, anti-Null-Island)
- 12: Stationary detection (var_gz<0.05 AND GPS<2km/h AND |mean_gz|<2.5°/s)
- 13: ZARU 3-axis (static: instant mean, straight: +=0.01×Δ with triple gate lat+gz+COG)
- 14a: ESKF 5D predict (gz - thermal_bias, body→ENU, pos+vel integration, ZUPT if stationary)
- 14b: ESKF 6D predict (shadow, internal bias X₅, SD-only logging)
- 15: NHC v_lateral=0 (active >5km/h, gated |lin_ay|<0.5g, 50Hz)
- 16: ESKF correct (if GPS fresh <5s): WGS84→ENU, 3-stage sequential (①pos R=0.05×HDOP² ②speed >5km/h ③COG dist>1m), Mahalanobis innovation gate, Joseph form
- If GPS stale >5s → predict-only mode + alarm
- COG variation (>15m baseline) feeds back into straight-ZARU gate

**Outputs:**
- 17a: ESKF state → telemetry_mutex → MQTT + LCD
- 17b: SD record 127B packed (EMA + raw + GPS + ESKF5D + ESKF6D shadow + ZARU flags + tbias_gz) → xQueueSend(depth=200) → Task_SD_Writer

### Key Modules

- **`eskf.h`** — Header-only ESKF2D (5-state) and ESKF_6D (6-state with gyro bias estimation). Uses BasicLinearAlgebra for matrix ops.
- **`madgwick.h`** — Header-only Madgwick AHRS with adaptive β (accel deviation + yaw rate gates).
- **`config.h`** — All tuning constants, pin definitions, calibration matrices. Primary knob-turning file.
- **`types.h`** — `TelemetryRecord` (127-byte binary format), `GpsData`, `ImuRawData` structs.
- **`globals.h/.cpp`** — All cross-module shared state (queues, mutexes, flags) via `extern`.
- **`wifi_manager.cpp`** — Reads `/wifi_config.txt` from SD for WiFi/MQTT credentials at boot.

### Binary Record Format (v1.3.2): 127 bytes

26-byte file header (magic "TEL", format version, firmware string, record size as uint16, start time), then packed structs: timestamp(4B) + EMA IMU(28B) + lap(1B) + GPS(29B) + ESKF2D(16B) + raw IMU(24B) + ESKF_6D(20B) + ZARU diagnostics(5B).

`bin_to_csv.py` handles format version detection and record parsing.

## Important Conventions

- **Coordinate frame:** All output is vehicle-frame (X=forward, Y=left, Z=up) after geometric alignment. ESKF uses local ENU via AEQD projection from first GPS fix.
- **Units:** Acceleration in G, gyro in °/s, GPS speed in km/h, temperature in °C. MoTeC export converts to m/s and rad/s as needed.
- **Calibration constants in `config.h`** are specific to the author's MPU-6886 unit. New hardware requires recalibration via tumble-test recordings.
- **ESKF_6D is shadow/experimental** — logged as `kf6_*` fields but not used for primary output.
- **GPS corrections gated at >5 km/h** to prevent standstill position jumps.
- Record size changed from uint8 to uint16 in header (BUG-8 fix) — tools must handle both.
