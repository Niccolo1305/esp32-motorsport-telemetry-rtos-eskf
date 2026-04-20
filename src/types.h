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
// Compact packed struct:
//   - AtomS3   : 202 bytes/sample (legacy stable layout)
//   - AtomS3R  : 242 bytes/sample (v5 raw/FIFO layout)
//
// AtomS3R v5 replaces the old ambiguous provider-frame block with acquisition
// truth: native BMI270 raw, converted BMI270 physical channels, native BMM150
// raw + RHALL, Bosch-compensated BMM150 uT, and explicit freshness/overflow
// diagnostics.
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

  // Zero-latency pipeline channels kept for diagnostics and offline replay.
  float pipe_lin_ax, pipe_lin_ay, pipe_lin_az; // 12
  float pipe_body_gx, pipe_body_gy, pipe_body_gz; // 12

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
  uint8_t zaru_flags;    // 1
  float tbias_gz;        // 4

#ifdef USE_BMI270
  int16_t bmi_raw_ax, bmi_raw_ay, bmi_raw_az; // 6
  int16_t bmi_raw_gx, bmi_raw_gy, bmi_raw_gz; // 6
  float bmi_acc_x_g, bmi_acc_y_g, bmi_acc_z_g; // 12
  float bmi_gyr_x_dps, bmi_gyr_y_dps, bmi_gyr_z_dps; // 12
  int16_t bmm_raw_x, bmm_raw_y, bmm_raw_z; // 6
  uint16_t bmm_rhall; // 2
  float bmm_ut_x, bmm_ut_y, bmm_ut_z; // 12
  uint8_t mag_valid; // 1
  uint8_t mag_sample_fresh; // 1
  uint8_t mag_overflow; // 1
  uint8_t imu_sample_fresh; // 1
  uint8_t fifo_frames_drained; // 1
  uint8_t fifo_backlog; // 1
  uint8_t fifo_overrun; // 1
  uint8_t reserved0; // 1
#else
  // Legacy AtomS3 provider-frame channels retained to keep the 202-byte format
  // stable and backwards-compatible.
  float sensor_ax, sensor_ay, sensor_az; // 12
  float sensor_gx, sensor_gy, sensor_gz; // 12
#endif

  // GPS timing metadata: same timebase as timestamp_us.
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
};
#ifdef USE_BMI270
static_assert(sizeof(TelemetryRecord) == 242, "TelemetryRecord must be 242 bytes (AtomS3R v5 raw/FIFO)");
#else
static_assert(sizeof(TelemetryRecord) == 202, "TelemetryRecord must be 202 bytes");
#endif

// File header written once at the start of every .bin file.
// Magic "TEL" lets Python tools distinguish modern files from legacy raw logs.
enum FileHeaderFlags : uint16_t {
  FILE_HEADER_FLAG_MAG_REF_VALID = 0x0001,
};

struct __attribute__((packed)) FileHeader {
  uint8_t magic[3];
  uint8_t header_version;
  char firmware_version[16];
  uint16_t record_size; // was uint8_t before header v2
  uint32_t start_time_ms;
  float cal_sin_phi, cal_cos_phi;
  float cal_sin_theta, cal_cos_theta;
  float cal_bias_ax, cal_bias_ay, cal_bias_az;
  float cal_bias_gx, cal_bias_gy, cal_bias_gz;
  uint16_t header_flags;
  float mag_ref_ut_x, mag_ref_ut_y, mag_ref_ut_z;
};
static_assert(sizeof(FileHeader) == 80, "FileHeader must be 80 bytes");

// IMU sample forwarded from Task_I2C to Task_Filter.
// temp_c is captured in the exclusive I2C context to avoid cross-task bus races.
// NOTE: not packed; sizeof depends on compiler alignment.
struct ImuRawData {
  float bmi_acc_x_g, bmi_acc_y_g, bmi_acc_z_g;
  float bmi_gyr_x_dps, bmi_gyr_y_dps, bmi_gyr_z_dps;
  float temp_c;
  uint64_t timestamp_us;
  int16_t bmi_raw_ax, bmi_raw_ay, bmi_raw_az;
  int16_t bmi_raw_gx, bmi_raw_gy, bmi_raw_gz;
  int16_t bmm_raw_x, bmm_raw_y, bmm_raw_z;
  uint16_t bmm_rhall;
  float bmm_ut_x, bmm_ut_y, bmm_ut_z;
  bool imu_sample_fresh;
  bool mag_sample_fresh;
  bool mag_valid;
  bool mag_overflow;
  uint8_t fifo_frames_drained;
  uint8_t fifo_backlog;
  bool fifo_overrun;
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
  float nav_speed2d = 0.0f;
  float nav_s_acc = 0.0f;
  float nav_vel_n = 0.0f;
  float nav_vel_e = 0.0f;
  uint8_t nav_vel_valid = 0;
  uint64_t nav_fix_us = 0;

  // NMEA DHV receiver-reported horizontal ground speed [m/s].
  float dhv_gdspd = 0.0f;
  uint64_t dhv_fix_us = 0;
};
