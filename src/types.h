// SPDX-License-Identifier: GPL-3.0-or-later
// types.h - Shared struct definitions for telemetry, GPS, IMU, and SD logging
#pragma once

#include <stdint.h>

// Wifi credentials loaded from /wifi_config.txt on SD.
struct WifiCredential {
  char ssid[33];     // max 32 chars + null terminator
  char password[65]; // max 64 chars + null terminator
};

// GPS speed source actually used by Task_Filter for this sample.
// 0 = NMEA RMC/VTG SOG fallback, 1 = passive NAV-PV listener, 2 = NMEA DHV.
enum GpsSpeedSource : uint8_t {
  GPS_SPD_NMEA_SOG = 0,
  GPS_SPD_NAV_PV   = 1,
  GPS_SPD_NMEA_DHV = 2,
};

// Binary SD record written by Task_Filter to sd_queue.
// Compact packed struct: 202 bytes/sample, no padding.
//
// Memory layout:
//   uint64_t timestamp_us  8 bytes  - IMU hardware clock [us] (v1.4.0)
//   float ax,ay,az        12 bytes  - EMA linear acceleration [g]
//   float gx,gy,gz        12 bytes  - EMA angular rate [deg/s]
//   float temp_c           4 bytes  - MPU-6886 temperature [C]
//   uint8_t lap            1 byte   - session flag
//   double gps_lat         8 bytes  - WGS84 latitude [deg]
//   double gps_lon         8 bytes  - WGS84 longitude [deg]
//   float gps_sog_kmh      4 bytes  - NMEA SOG [km/h]
//   float gps_alt_m        4 bytes  - altitude [m]
//   uint8_t gps_sats       1 byte   - satellites used
//   float gps_hdop         4 bytes  - HDOP
//   float kf_x             4 bytes  - 5D ENU East [m]
//   float kf_y             4 bytes  - 5D ENU North [m]
//   float kf_vel           4 bytes  - 5D speed [m/s]
//   float kf_heading       4 bytes  - 5D heading [rad]
//   float raw_ax/ay/az    12 bytes  - post-Madgwick linear accel without EMA [g]
//   float raw_gx/gy/gz    12 bytes  - vehicle-frame gyro without EMA [deg/s]
//   float kf6_x/y/vel/h   16 bytes  - 6D shadow state
//   float kf6_bgz          4 bytes  - estimated gz bias [rad/s]
//   uint8_t zaru_flags     1 byte   - ZARU/NHC activation bitmask
//   float tbias_gz         4 bytes  - learned thermal bias gz [deg/s]
//   float sensor_ax/ay/az 12 bytes  - sensor-frame raw accel [G]
//   float sensor_gx/gy/gz 12 bytes  - sensor-frame raw gyro [deg/s]
//   uint64_t gps_fix_us    8 bytes  - esp_timer timestamp of last NMEA fix [us]
//   uint8_t gps_valid      1 byte   - mirror of GpsData.valid for this sample
//   float nav_speed2d      4 bytes  - passive NAV-PV speed2D [m/s]
//   float nav_s_acc        4 bytes  - passive NAV-PV speed accuracy estimate [m/s]
//   float nav_vel_n        4 bytes  - passive NAV-PV North velocity [m/s]
//   float nav_vel_e        4 bytes  - passive NAV-PV East velocity [m/s]
//   uint8_t nav_vel_valid  1 byte   - 0=invalid, 6=2D, 7=3D
//   uint8_t gps_speed_src  1 byte   - GpsSpeedSource enum value
//   uint64_t nav_fix_us    8 bytes  - esp_timer when passive NAV-PV was parsed [us]
//   float dhv_gdspd        4 bytes  - NMEA DHV receiver-reported ground speed [m/s]
//   uint64_t dhv_fix_us    8 bytes  - esp_timer when DHV was parsed [us]
//   --- v1.6.0 AtomS3R (USE_BMI270) additions: 13 bytes → 215 total ---
//   float mag_mx/my/mz    12 bytes  - magnetometer [M5Unified raw, arbitrary units]
//   uint8_t mag_valid       1 byte  - getMag() return value (read success, not freshness)
//
// Python read (202B): struct.unpack('<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ', chunk)
// Python read (215B): struct.unpack('<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ3fB', chunk)
struct __attribute__((packed)) TelemetryRecord {
  uint64_t timestamp_us; // 8
  float ax, ay, az;      // 12
  float gx, gy, gz;      // 12
  float temp_c;          // 4
  uint8_t lap;           // 1

  double gps_lat;        // 8
  double gps_lon;        // 8
  float gps_sog_kmh;     // 4
  float gps_alt_m;       // 4
  uint8_t gps_sats;      // 1
  float gps_hdop;        // 4

  float kf_x;            // 4
  float kf_y;            // 4
  float kf_vel;          // 4
  float kf_heading;      // 4

  float raw_ax, raw_ay, raw_az; // 12
  float raw_gx, raw_gy, raw_gz; // 12

  float kf6_x;           // 4
  float kf6_y;           // 4
  float kf6_vel;         // 4
  float kf6_heading;     // 4
  float kf6_bgz;         // 4

  // zaru_flags bitmask:
  //   bit 0 (0x01): Static ZARU
  //   bit 1 (0x02): Straight ZARU
  //   bit 2 (0x04): NHC
  //   bit 3 (0x08): First sample after recalibration
  //
  // Typical combinations:
  //   0 = no system active
  //   1 = stationary
  //   4 = NHC only
  //   6 = NHC + Straight ZARU
  uint8_t zaru_flags;    // 1
  float tbias_gz;        // 4

  float sensor_ax, sensor_ay, sensor_az; // 12
  float sensor_gx, sensor_gy, sensor_gz; // 12

  // SITL GPS timing metadata:
  //   gps_fix_us uses the same timebase as timestamp_us, enabling offline
  //   replay of the firmware stale-GPS gate:
  //   (timestamp_us - gps_fix_us) > 5_000_000 us.
  //   gps_valid reproduces the firmware gps_slow decision path.
  uint64_t gps_fix_us;   // 8
  uint8_t gps_valid;     // 1

  float nav_speed2d;     // 4
  float nav_s_acc;       // 4
  float nav_vel_n;       // 4
  float nav_vel_e;       // 4
  uint8_t nav_vel_valid; // 1
  uint8_t gps_speed_source; // 1
  uint64_t nav_fix_us;   // 8

  float dhv_gdspd;       // 4
  uint64_t dhv_fix_us;   // 8

#ifdef USE_BMI270
  // Magnetometer data — appended to preserve backward compatibility with 202B records.
  // Units: M5Unified raw output (arbitrary, AK8963-scaled, uncompensated).
  // MAG_W/MAG_B calibration (config.h) does NOT replace BMM150 Bosch trim compensation
  // and does NOT enable magnetic heading or 9-DOF fusion in this release.
  // Python read (215B): struct.unpack('<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ3fB', chunk)
  float mag_mx, mag_my, mag_mz;  // 12 — calibrated via MAG_W/MAG_B (identity placeholder)
  uint8_t mag_valid;              //  1 — M5Unified getMag() return value (read success, NOT freshness)
#endif
};
#ifdef USE_BMI270
static_assert(sizeof(TelemetryRecord) == 215, "TelemetryRecord must be 215 bytes (BMI270+BMM150)");
#else
static_assert(sizeof(TelemetryRecord) == 202, "TelemetryRecord must be 202 bytes");
#endif

// File header written once at the start of every .bin file.
// Magic "TEL" lets Python tools distinguish modern files from legacy raw logs.
struct __attribute__((packed)) FileHeader {
  uint8_t magic[3];
  uint8_t header_version;
  char firmware_version[16];
  uint16_t record_size; // BUG-8 fix: was uint8_t, overflows if record > 255 B
  uint32_t start_time_ms;
  float cal_sin_phi, cal_cos_phi;
  float cal_sin_theta, cal_cos_theta;
  float cal_bias_ax, cal_bias_ay, cal_bias_az;
  float cal_bias_gx, cal_bias_gy, cal_bias_gz;
};
static_assert(sizeof(FileHeader) == 66, "FileHeader must be 66 bytes");

// IMU sample forwarded from Task_I2C to Task_Filter.
// temp_c is captured in the exclusive I2C context to avoid cross-task bus races.
// NOTE: not packed — actual sizeof depends on compiler alignment.
// Both builds see the same struct (no #ifdef) to avoid FreeRTOS queue size mismatch.
struct ImuRawData {
  float ax, ay, az;
  float gx, gy, gz;
  float temp_c;
  uint64_t timestamp_us;
  // Magnetometer — populated by Bmi270Provider, zero/false for Mpu6886Provider.
  // Units: M5Unified raw output (arbitrary, AK8963-scaled, NOT compensated µT).
  // See Docs/AtomS3R_Migration_Guide.md §3 and §6 for the M5Unified BMM150
  // conversion bug (uses AK8963 factor, no trim register compensation).
  float mx, my, mz;
  bool mag_valid;  // M5Unified getMag() return value (read success, NOT freshness)
};

struct FilteredTelemetry {
  float ema_ax, ema_ay, ema_az;
  float ema_gx, ema_gy, ema_gz;
  float kf_x, kf_y, kf_vel, kf_heading;
};

// GPS snapshot protected by gps_mutex.
// The lock is held only long enough to copy this struct, avoiding interference
// with the 50 Hz IMU path.
struct GpsData {
  double lat = 0.0;
  double lon = 0.0;
  float sog_kmh = 0.0f; // NMEA RMC/VTG SOG [km/h]
  float alt_m = 0.0f;
  float hdop = 99.9f;
  uint8_t sats = 0;
  bool valid = false;
  uint32_t epoch = 0;   // monotonic NMEA fix counter
  uint64_t fix_us = 0;  // esp_timer timestamp of last valid NMEA fix [us]

  // Passive CASIC NAV-PV listener kept for diagnostics and future variants.
  // In v1.5.2-clean the firmware no longer probes binary enable at boot, so
  // nav_* is typically expected to stay zero on the tested AT6668 module.
  float nav_speed2d = 0.0f;
  float nav_s_acc = 0.0f;
  float nav_vel_n = 0.0f;
  float nav_vel_e = 0.0f;
  uint8_t nav_vel_valid = 0;
  uint64_t nav_fix_us = 0;

  // NMEA DHV receiver-reported horizontal ground speed [m/s].
  // Logged for diagnostics only in v1.5.2-clean; production speed source is NMEA SOG.
  float dhv_gdspd = 0.0f;
  uint64_t dhv_fix_us = 0;
};
