// SPDX-License-Identifier: GPL-3.0-or-later
// types.h — Shared struct definitions for telemetry, GPS, IMU, and SD logging
#pragma once

#include <stdint.h>

// ── WiFi Credentials ───────────────────────────────────────────────────────

struct WifiCredential {
  char ssid[33];     // max 32 chars + null terminator
  char password[65]; // max 64 chars + null terminator
};

// ── Binary SD Record ───────────────────────────────────────────────────────
// Compact binary struct (155 bytes/sample, zero padding).
// __attribute__((packed)): no alignment padding bytes.
//
// Memory layout:
//   uint64_t timestamp_us  8 bytes  — IMU hardware clock [µs] (v1.4.0, was uint32 ms)
//   float ax,ay,az        12 bytes  — EMA-filtered
//   float gx,gy,gz        12 bytes  — EMA-filtered
//   float temp_c           4 bytes
//   uint8_t lap            1 byte   → IMU EMA subtotal: 37 bytes
//   double gps_lat         8 bytes
//   double gps_lon         8 bytes
//   float gps_speed_kmh    4 bytes
//   float gps_alt_m        4 bytes
//   uint8_t gps_sats       1 byte
//   float gps_hdop         4 bytes  → GPS subtotal: 29 bytes (v0.9.5)
//   float kf_x             4 bytes
//   float kf_y             4 bytes
//   float kf_vel           4 bytes
//   float kf_heading       4 bytes  → ESKF 5D subtotal: 16 bytes
//   float raw_ax/ay/az    12 bytes  — post-Madgwick without EMA [g] (v0.9.8)
//   float raw_gx/gy/gz    12 bytes  — vehicle-frame without EMA [deg/s] (v0.9.8)
//   float kf6_x            4 bytes  — 6D shadow position [m] (v0.9.8)
//   float kf6_y            4 bytes
//   float kf6_vel          4 bytes
//   float kf6_heading      4 bytes
//   float kf6_bgz          4 bytes  — estimated gz bias [rad/s]
//   uint8_t zaru_flags     1 byte   — ZARU/NHC activation flags (v1.3.1)
//   float tbias_gz         4 bytes  — learned thermal bias gz [deg/s] (v1.3.1)
//   float sensor_ax/ay/az 12 bytes  — sensor-frame raw accel [G] (SITL, v1.4.0)
//   float sensor_gx/gy/gz 12 bytes  — sensor-frame raw gyro [deg/s] (SITL, v1.4.0)
//                         ────────
//   TOTAL:                155 bytes/sample
//
// Python read: struct.unpack('<Q7fBddffBf4f6f5fBf6f', chunk_155_byte)
//
// 16 GB SD at 50 Hz: ~109 million samples ≈ 24 h of continuous logging

struct __attribute__((packed)) TelemetryRecord {
  uint64_t timestamp_us; // 8  — IMU hardware clock [µs] (v1.4.0)
  float ax, ay, az;      // 12 — EMA linear acceleration [g]
  float gx, gy, gz;      // 12 — EMA angular rate [deg/s]
  float temp_c;          // 4  — MPU-6886 temperature [°C]
  uint8_t lap;           // 1  — session flag (0=pit, 1=on track)
  double gps_lat;        // 8  — WGS84 latitude [deg]
  double gps_lon;        // 8  — WGS84 longitude [deg]
  float gps_speed_kmh;   // 4  — speed [km/h]
  float gps_alt_m;       // 4  — altitude [m]
  uint8_t gps_sats;      // 1  — satellites locked
  float gps_hdop;        // 4  — GPS geometric precision (HDOP)
  float kf_x;            // 4  — 5D ENU East position [m]
  float kf_y;            // 4  — 5D ENU North position [m]
  float kf_vel;          // 4  — 5D speed [m/s]
  float kf_heading;      // 4  — 5D heading [rad]
  // ── Raw IMU post-Madgwick (v0.9.8) ──
  float raw_ax, raw_ay, raw_az; // 12 — linear accel without EMA [g]
  float raw_gx, raw_gy, raw_gz; // 12 — vehicle-frame gyro without EMA [deg/s]
  // ── ESKF 6D Shadow (v0.9.8) ──
  float kf6_x;       // 4  — 6D ENU East position [m]
  float kf6_y;       // 4  — 6D ENU North position [m]
  float kf6_vel;     // 4  — 6D speed [m/s]
  float kf6_heading; // 4  — 6D heading [rad]
  float kf6_bgz;     // 4  — estimated gz bias [rad/s]
  // ── ZARU/NHC Diagnostics (v1.3.1) ──
  //
  // zaru_flags — bitmask, each bit = 1 when the system is active this sample:
  //   bit 0 (0x01): Static ZARU      — vehicle stationary (< 2 km/h, low variance)
  //                                     → thermal_bias_gz = mean_gz (instant)
  //   bit 1 (0x02): Straight ZARU    — fast straight (> 40 km/h, triple gate)
  //                                     → thermal_bias_gz updated via slow EMA
  //   bit 2 (0x04): NHC              — Non-Holonomic Constraint (v_lat = 0)
  //                                     → ESKF heading correction, always in motion
  //   bit 3 (0x08): Recalibration   — first sample after mid-session recalibration
  //                                     → calibration params changed, see CalibrationRecord
  //
  // Typical combinations in the CSV:
  //   0 = nessun sistema attivo (curva lenta, < 5 km/h senza stazionarietà)
  //   1 = ai box, fermo — ZARU statico
  //   4 = in curva o rettilineo lento — solo NHC
  //   6 = rettilineo veloce — NHC + Straight ZARU insieme
  //   5 = impossibile (stationary + NHC: velocità < 2 E > 5 km/h)
  //
  uint8_t zaru_flags;
  float tbias_gz;     // 4  — thermal_bias_gz [deg/s] (learned bias value)
  // ── SITL Raw Sensor-Frame (v1.4.0) ──
  float sensor_ax, sensor_ay, sensor_az; // 12 — sensor-frame raw accel [G]
  float sensor_gx, sensor_gy, sensor_gz; // 12 — sensor-frame raw gyro [deg/s]
};
static_assert(sizeof(TelemetryRecord) == 155, "TelemetryRecord must be 155 bytes");

// ── Binary File Header ─────────────────────────────────────────────────────
// Written at the start of every .bin file on the SD (v0.9.7).
// Allows the Python converter to identify the firmware version and record_size
// without hardcoded assumptions. Magic "TEL" for detection vs legacy files.

struct __attribute__((packed)) FileHeader {
  uint8_t magic[3];          // "TEL" = {0x54, 0x45, 0x4C}
  uint8_t header_version;    // 3 (v1→v2: record_size uint16; v2→v3: calib params)
  char firmware_version[16]; // e.g. "v1.4.0\0" + padding
  uint16_t record_size;      // BUG-8 fix: was uint8_t, overflows if record > 255 B
  uint32_t start_time_ms;    // millis() at file open
  // ── v3: boot calibration parameters (SITL pipeline reconstruction) ──
  // Session-specific outputs of calibrate_alignment(), stored so SITL can
  // reconstruct sensor→vehicle frame offline. B/W matrices are in config.h.
  float cal_sin_phi, cal_cos_phi;              // 8  — mounting rotation
  float cal_sin_theta, cal_cos_theta;          // 8  — mounting rotation
  float cal_bias_ax, cal_bias_ay, cal_bias_az; // 12 — accel residual bias [G]
  float cal_bias_gx, cal_bias_gy, cal_bias_gz; // 12 — gyro boot bias [deg/s]
};
static_assert(sizeof(FileHeader) == 66, "FileHeader must be 66 bytes");

// ── IMU Raw Data (from Task_I2C queue) ─────────────────────────────────────
// ImuRawData includes temp_c: temperature is read in Task_I2C where the I2C
// bus is in exclusive use, eliminating the race condition with
// getAccelData/getGyroData that existed when it was read in Task_Filter.

struct ImuRawData {
  float ax, ay, az;
  float gx, gy, gz;
  float temp_c; // temperature read in the exclusive I2C context
  uint64_t timestamp_us;
};

// ── Filtered Telemetry (shared between Task_Filter and loop()) ─────────────

struct FilteredTelemetry {
  float ema_ax, ema_ay, ema_az;
  float ema_gx, ema_gy, ema_gz;
  float kf_x, kf_y, kf_vel, kf_heading; // ESKF output (v0.9.2)
};

// ── GPS Data (shared between Task_GPS and Task_Filter/loop()) ──────────────
// Protected by gps_mutex, separate from telemetry_mutex: lock lasts only long
// enough to copy ~30 bytes, without interfering with the 50 Hz IMU cycle.

struct GpsData {
  double lat = 0.0;
  double lon = 0.0;
  float speed_kmh = 0.0f;
  float alt_m = 0.0f;  // altitude [m] (v0.7.3)
  float hdop = 99.9f;  // geometric precision HDOP (v0.9.5)
  uint8_t sats = 0;
  bool valid = false;   // true only after first valid fix
  uint32_t epoch = 0;   // monotonic fix counter (v0.8.0): Task_Filter
                        // compares with last_epoch to detect a fresh fix
  uint32_t fix_ms = 0;  // millis() of last valid fix (v0.9.11: staleness)
};
