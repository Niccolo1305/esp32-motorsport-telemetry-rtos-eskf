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
// Compact binary struct (122 bytes/sample, zero padding).
// __attribute__((packed)): no alignment padding bytes.
//
// Memory layout:
//   uint32_t timestamp_ms  4 bytes
//   float ax,ay,az        12 bytes  — EMA-filtered
//   float gx,gy,gz        12 bytes  — EMA-filtered
//   float temp_c           4 bytes
//   uint8_t lap            1 byte   → IMU EMA subtotal: 33 bytes
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
//                         ────────
//   TOTAL:                122 bytes/sample
//
// Python read: struct.unpack('<I7fBddffBf4f6f5f', chunk_122_byte)
//
// 16 GB SD at 50 Hz: ~138 million samples ≈ 31 h of continuous logging

struct __attribute__((packed)) TelemetryRecord {
  uint32_t timestamp_ms; // 4  — IMU hardware clock (us/1000)
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
};
static_assert(sizeof(TelemetryRecord) == 122, "TelemetryRecord must be 122 bytes");

// ── Binary File Header ─────────────────────────────────────────────────────
// Written at the start of every .bin file on the SD (v0.9.7).
// Allows the Python converter to identify the firmware version and record_size
// without hardcoded assumptions. Magic "TEL" for detection vs legacy files.

struct __attribute__((packed)) FileHeader {
  uint8_t magic[3];          // "TEL" = {0x54, 0x45, 0x4C}
  uint8_t header_version;    // 2 (v1→v2: record_size widened to uint16_t)
  char firmware_version[16]; // e.g. "v1.0.0\0" + padding
  uint16_t record_size;      // BUG-8 fix: was uint8_t, overflows if record > 255 B
  uint32_t start_time_ms;    // millis() at file open
};
static_assert(sizeof(FileHeader) == 26, "FileHeader must be 26 bytes");

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
