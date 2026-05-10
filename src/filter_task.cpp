// SPDX-License-Identifier: GPL-3.0-or-later
// filter_task.cpp — FreeRTOS Task_Filter (Core 1, priority 2)
//
// Math pipeline: calibration → alignment → ZARU→gyro rad (corrected) →
// VGPL centripetal compensation → Madgwick (corrected gyro) → gravity removal →
// Statistical Engine (raw variance) → ZARU update → ESKF 5D (corrected gz) →
// ESKF 6D shadow (raw gz) → NHC → GPS correct → EMA (end-of-pipe) → SD.
//
// Architecture: "End-of-Pipe EMA" + "ZARU-to-Madgwick" (v1.3.5)
//
// Three structural fixes over v1.3.4:
//
// 1. End-of-Pipe EMA: the EMA filter is an aesthetic layer for display/MQTT/SD
//    and must NOT contaminate the ZARU detection path. Variance buffers feed on
//    raw gyro so the EMA's ~313 ms decay tail cannot trigger false stationary
//    detection after an abrupt stop.
//
// 2. ZARU-to-Madgwick ("Bias Starvation" fix): thermal_bias_gx/gy/gz from the
//    previous cycle's ZARU is subtracted from gx_r/gy_r/gz_r BEFORE computing
//    gx_rad/gy_rad/gz_rad. Madgwick and VGPL receive corrected gyro, preventing
//    the mathematical horizon from tilting under MPU-6886 electronic drift
//    (-4.7 °/s/hr on gx). Without this, the tilted gravity vector projects
//    ~0.08g per degree into lin_ax/lin_ay, poisoning ESKF velocity and
//    permanently blocking the Straight-Line ZARU lateral gate (|lin_ax|<0.05g).
//    Raw values are preserved as gz_rad_raw for ESKF 6D shadow comparison.
//
// 3. Straight-Line ZARU zero-latency gate: gz_gate uses raw corrected
//    (gz_r - thermal_bias_gz) instead of EMA, closing instantly on corner entry.
#include "filter_task.h"

#include "globals.h"
#include "gps_nav2.h"
#include "imu_axis_remap.h"
#include "math_utils.h"

static constexpr uint64_t MOUNT_YAW_OBS_SENTINEL_TS = UINT64_MAX - 1ULL;
static constexpr uint64_t MOUNT_YAW_BOOT_SENTINEL_TS = UINT64_MAX - 2ULL;
static constexpr uint8_t ESKF_GPS_MIN_SATS = 8;
static constexpr float ESKF_GPS_MAX_HDOP = 2.0f;
static constexpr uint64_t GPS_FRESH_MAX_US = 1500000ULL;
static constexpr uint64_t GPS_STALE_US = 5000000ULL;
static constexpr float ESKF_GPS_POS_GATE_MIN_M = 8.0f;
static constexpr float ESKF_GPS_POS_GATE_MARGIN_M = 3.0f;
static constexpr float ESKF_GPS_POS_GATE_SPEED_FACTOR = 1.35f;
static constexpr float ESKF_GPS_POS_GATE_MAX_ACCEL_MS2 = 6.0f;
static constexpr float ESKF_GPS_POS_GATE_HDOP_FACTOR_M = 4.0f;
static constexpr bool MOUNT_YAW_APPLY_LIVE = false;
static constexpr float MOUNT_YAW_MIN_SPEED_KMH = 18.0f;
static constexpr float MOUNT_YAW_MAX_HDOP = 1.5f;
static constexpr uint8_t MOUNT_YAW_MIN_SATS = 10;
static constexpr float MOUNT_YAW_MAX_ABS_GZ_DPS = 2.0f;
static constexpr float MOUNT_YAW_MAX_ABS_COG_RATE_DPS = 8.0f;
static constexpr float MOUNT_YAW_MAX_LAT_EXPECTED_G = 0.05f;
static constexpr float MOUNT_YAW_MIN_ABS_GPS_ACCEL_G = 0.025f;
static constexpr float MOUNT_YAW_MIN_LONG_CORR = 0.20f;
static constexpr float MOUNT_YAW_MIN_QUALITY = 0.55f;
static constexpr float MOUNT_YAW_MIN_DURATION_S = 2.5f;
static constexpr float MOUNT_YAW_MAX_DURATION_S = 6.5f;
static constexpr float MOUNT_YAW_COOLDOWN_S = 1.0f;
static constexpr uint8_t MOUNT_YAW_SPEED_HIST_N = 64;
static constexpr uint16_t MOUNT_YAW_IMU_HIST_N = 256;
static constexpr float MOUNT_YAW_GPS_ACCEL_MIN_BASELINE_S = 2.0f;
static constexpr float MOUNT_YAW_GPS_ACCEL_MAX_BASELINE_S = 4.0f;
static constexpr uint8_t MOUNT_YAW_BOOT_MIN_OBS = 5;
static constexpr float MOUNT_YAW_BOOT_MIN_QUALITY = 0.65f;
static constexpr float MOUNT_YAW_BOOT_MIN_LONG_CORR = 0.45f;
static constexpr float MOUNT_YAW_BOOT_MAX_OBS_STABILITY_STD_DEG = 15.0f;
static constexpr float MOUNT_YAW_BOOT_MAX_CONSENSUS_STD_DEG = 8.0f;

struct MountYawWindow {
  bool active = false;
  uint64_t start_us = 0;
  uint64_t end_us = 0;
  uint16_t count = 0;
  float sum_ax = 0.0f;
  float sum_ay = 0.0f;
  float sum_ax2 = 0.0f;
  float sum_ay2 = 0.0f;
  float sum_axay = 0.0f;
  float sum_acc = 0.0f;
  float sum_acc2 = 0.0f;
  float sum_axacc = 0.0f;
  float sum_ayacc = 0.0f;
  float sum_abs_acc = 0.0f;
  float sum_speed = 0.0f;
  float min_speed = 9999.0f;
  float sum_gz2 = 0.0f;
  float sum_cog2 = 0.0f;
  float sum_lat_expected2 = 0.0f;
  float sum_az2 = 0.0f;
  uint16_t stab_count = 0;
  float stab_sum_ax = 0.0f;
  float stab_sum_ay = 0.0f;
  float stab_sum_acc = 0.0f;
  float stab_sum_axacc = 0.0f;
  float stab_sum_ayacc = 0.0f;
  float stab_sum_abs_acc = 0.0f;
  uint8_t stability_n = 0;
  float stability_sin_sum = 0.0f;
  float stability_cos_sum = 0.0f;
};

struct MountYawBootState {
  bool active = false;
  bool emitted = false;
  bool just_activated = false;
  float yaw_deg = 0.0f;
  float yaw_rad = 0.0f;
  float sin_yaw = 0.0f;
  float cos_yaw = 1.0f;
  uint8_t obs_count = 0;
  float sin_sum = 0.0f;
  float cos_sum = 0.0f;
  float quality_sum = 0.0f;
  float corr_sum = 0.0f;
  float stability_sum = 0.0f;
};

struct MountYawState {
  MountYawWindow win;
  MountYawBootState boot;
  uint8_t obs_id = 0;
  uint64_t cooldown_until_us = 0;
  uint32_t last_speed_epoch = 0;
  uint64_t last_speed_fix_us = 0;
  float last_speed_ms = 0.0f;
  float gps_acc_ema_g = 0.0f;
  bool has_speed = false;
  uint8_t speed_hist_idx = 0;
  uint8_t speed_hist_count = 0;
  uint64_t speed_hist_fix_us[MOUNT_YAW_SPEED_HIST_N] = {};
  float speed_hist_ms[MOUNT_YAW_SPEED_HIST_N] = {};
  uint32_t gps_acc_lag_us = 0;
  uint16_t imu_hist_idx = 0;
  uint16_t imu_hist_count = 0;
  uint64_t imu_hist_us[MOUNT_YAW_IMU_HIST_N] = {};
  float imu_hist_ax[MOUNT_YAW_IMU_HIST_N] = {};
  float imu_hist_ay[MOUNT_YAW_IMU_HIST_N] = {};
  float imu_hist_az[MOUNT_YAW_IMU_HIST_N] = {};
  uint32_t last_cog_epoch = 0;
  uint64_t last_cog_fix_us = 0;
  float last_east_m = 0.0f;
  float last_north_m = 0.0f;
  float last_cog_rad = 0.0f;
  float cog_rate_ema_dps = 0.0f;
  bool has_cog = false;
};

struct GpsCorrectionGateState {
  bool has_ref = false;
  float east_m = 0.0f;
  float north_m = 0.0f;
  float speed_kmh = 0.0f;
  uint64_t fix_us = 0;
  uint8_t reject_count = 0;
};

static float clamp01f(float value) {
  if (!isfinite(value)) return 0.0f;
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

static float normalize_mount_yaw_deg(float angle) {
  while (angle > 90.0f) angle -= 180.0f;
  while (angle < -90.0f) angle += 180.0f;
  return angle;
}

static float wrap_pi(float value) {
  while (value > (float)M_PI) value -= 2.0f * (float)M_PI;
  while (value < -(float)M_PI) value += 2.0f * (float)M_PI;
  return value;
}

static float circular_std_deg(float sin_sum, float cos_sum, float n) {
  if (n <= 0.0f) return 180.0f;
  float r = sqrtf(sin_sum * sin_sum + cos_sum * cos_sum) / n;
  r = fminf(1.0f, fmaxf(1.0e-6f, r));
  return sqrtf(fmaxf(0.0f, -2.0f * logf(r))) / DEG2RAD;
}

static void rotate_mount_yaw_xy(float yaw_rad, float& x, float& y) {
  const float c = cosf(yaw_rad);
  const float s = sinf(yaw_rad);
  const float x_corr = c * x - s * y;
  const float y_corr = s * x + c * y;
  x = x_corr;
  y = y_corr;
}

static bool gps_fix_quality_ok(const GpsData& gps) {
  return gps.valid &&
         gps.sats >= ESKF_GPS_MIN_SATS &&
         gps.hdop <= ESKF_GPS_MAX_HDOP &&
         isfinite(gps.lat) && isfinite(gps.lon) &&
         fabs(gps.lat) > 1.0e-9 && fabs(gps.lon) > 1.0e-9;
}

static bool gps_correction_gate_allows(const GpsCorrectionGateState& gate,
                                       float east_m,
                                       float north_m,
                                       float speed_kmh,
                                       float hdop,
                                       uint64_t fix_us) {
  if (!gate.has_ref || gate.fix_us == 0 || fix_us <= gate.fix_us) {
    return true;
  }
  const float dt_s = (float)(fix_us - gate.fix_us) / 1000000.0f;
  if (dt_s <= 0.0f) return false;

  const float de = east_m - gate.east_m;
  const float dn = north_m - gate.north_m;
  const float dist_m = sqrtf(de * de + dn * dn);
  const float ref_speed_ms = fmaxf(fmaxf(speed_kmh, gate.speed_kmh) / 3.6f, 3.0f);
  const float hdop_m = fminf(fmaxf(isfinite(hdop) ? hdop : 2.0f, 0.5f), 10.0f)
                     * ESKF_GPS_POS_GATE_HDOP_FACTOR_M;
  const float allowed_m = fmaxf(ESKF_GPS_POS_GATE_MIN_M,
                                ref_speed_ms * dt_s * ESKF_GPS_POS_GATE_SPEED_FACTOR
                                + 0.5f * ESKF_GPS_POS_GATE_MAX_ACCEL_MS2 * dt_s * dt_s
                                + hdop_m
                                + ESKF_GPS_POS_GATE_MARGIN_M);
  return dist_m <= allowed_m;
}

static void gps_correction_gate_accept(GpsCorrectionGateState& gate,
                                       float east_m,
                                       float north_m,
                                       float speed_kmh,
                                       uint64_t fix_us) {
  gate.has_ref = true;
  gate.east_m = east_m;
  gate.north_m = north_m;
  gate.speed_kmh = speed_kmh;
  gate.fix_us = fix_us;
  gate.reject_count = 0;
}

static void update_mount_yaw_speed_history(MountYawState& state,
                                           uint64_t fix_us,
                                           float speed_ms) {
  state.speed_hist_fix_us[state.speed_hist_idx] = fix_us;
  state.speed_hist_ms[state.speed_hist_idx] = speed_ms;
  state.speed_hist_idx = (uint8_t)((state.speed_hist_idx + 1) % MOUNT_YAW_SPEED_HIST_N);
  if (state.speed_hist_count < MOUNT_YAW_SPEED_HIST_N) state.speed_hist_count++;
}

static bool mount_yaw_speed_baseline_accel(const MountYawState& state,
                                           uint64_t fix_us,
                                           float speed_ms,
                                           float& acc_g,
                                           uint32_t& lag_us) {
  bool found = false;
  float best_dt = 0.0f;
  float best_speed = 0.0f;
  uint64_t best_hist_us = 0;
  for (uint8_t i = 0; i < state.speed_hist_count; i++) {
    const uint64_t hist_us = state.speed_hist_fix_us[i];
    if (hist_us == 0 || hist_us >= fix_us) continue;
    const float dt_s = (float)(fix_us - hist_us) / 1000000.0f;
    if (dt_s >= MOUNT_YAW_GPS_ACCEL_MIN_BASELINE_S &&
        dt_s <= MOUNT_YAW_GPS_ACCEL_MAX_BASELINE_S &&
        dt_s > best_dt) {
      best_dt = dt_s;
      best_speed = state.speed_hist_ms[i];
      best_hist_us = hist_us;
      found = true;
    }
  }
  if (!found) return false;
  acc_g = (speed_ms - best_speed) / best_dt / G_ACCEL;
  lag_us = (uint32_t)((fix_us - best_hist_us) / 2ULL);
  return true;
}

static void update_mount_yaw_imu_history(MountYawState& state,
                                         uint64_t timestamp_us,
                                         float ax,
                                         float ay,
                                         float az) {
  state.imu_hist_us[state.imu_hist_idx] = timestamp_us;
  state.imu_hist_ax[state.imu_hist_idx] = ax;
  state.imu_hist_ay[state.imu_hist_idx] = ay;
  state.imu_hist_az[state.imu_hist_idx] = az;
  state.imu_hist_idx = (uint16_t)((state.imu_hist_idx + 1) % MOUNT_YAW_IMU_HIST_N);
  if (state.imu_hist_count < MOUNT_YAW_IMU_HIST_N) state.imu_hist_count++;
}

static bool mount_yaw_delayed_imu(const MountYawState& state,
                                  uint64_t target_us,
                                  float& ax,
                                  float& ay,
                                  float& az) {
  if (state.imu_hist_count == 0) return false;
  bool found = false;
  uint64_t best_err = UINT64_MAX;
  uint16_t best_i = 0;
  for (uint16_t i = 0; i < state.imu_hist_count; i++) {
    const uint64_t sample_us = state.imu_hist_us[i];
    if (sample_us == 0) continue;
    const uint64_t err = (sample_us > target_us) ? (sample_us - target_us)
                                                : (target_us - sample_us);
    if (err < best_err) {
      best_err = err;
      best_i = i;
      found = true;
    }
  }
  if (!found || best_err > 50000ULL) return false;
  ax = state.imu_hist_ax[best_i];
  ay = state.imu_hist_ay[best_i];
  az = state.imu_hist_az[best_i];
  return true;
}

static void reset_mount_yaw_window(MountYawWindow& w) {
  w = MountYawWindow{};
}

static void start_mount_yaw_window(MountYawWindow& w, uint64_t timestamp_us) {
  reset_mount_yaw_window(w);
  w.active = true;
  w.start_us = timestamp_us;
  w.end_us = timestamp_us;
}

static void flush_mount_yaw_stability_segment(MountYawWindow& w) {
  if (w.stab_count < (uint16_t)FREQ_HZ / 2 || w.stab_sum_abs_acc < 0.01f) {
    w.stab_count = 0;
    w.stab_sum_ax = 0.0f;
    w.stab_sum_ay = 0.0f;
    w.stab_sum_acc = 0.0f;
    w.stab_sum_axacc = 0.0f;
    w.stab_sum_ayacc = 0.0f;
    w.stab_sum_abs_acc = 0.0f;
    return;
  }

  const float n = (float)w.stab_count;
  const float mean_ax = w.stab_sum_ax / n;
  const float mean_ay = w.stab_sum_ay / n;
  const float mean_acc = w.stab_sum_acc / n;
  const float cov_ax_acc = w.stab_sum_axacc / n - mean_ax * mean_acc;
  const float cov_ay_acc = w.stab_sum_ayacc / n - mean_ay * mean_acc;
  const float yaw_deg = normalize_mount_yaw_deg(atan2f(cov_ax_acc, cov_ay_acc) / DEG2RAD);
  const float yaw_rad = yaw_deg * DEG2RAD;
  w.stability_sin_sum += sinf(yaw_rad);
  w.stability_cos_sum += cosf(yaw_rad);
  if (w.stability_n < 255) w.stability_n++;

  w.stab_count = 0;
  w.stab_sum_ax = 0.0f;
  w.stab_sum_ay = 0.0f;
  w.stab_sum_acc = 0.0f;
  w.stab_sum_axacc = 0.0f;
  w.stab_sum_ayacc = 0.0f;
  w.stab_sum_abs_acc = 0.0f;
}

static void accumulate_mount_yaw_window(MountYawWindow& w,
                                        uint64_t timestamp_us,
                                        float ax, float ay, float az,
                                        float gps_acc_g,
                                        float speed_kmh,
                                        float gz_corr_dps,
                                        float cog_rate_dps,
                                        float lat_expected_g) {
  w.end_us = timestamp_us;
  w.count++;
  w.sum_ax += ax;
  w.sum_ay += ay;
  w.sum_ax2 += ax * ax;
  w.sum_ay2 += ay * ay;
  w.sum_axay += ax * ay;
  w.sum_acc += gps_acc_g;
  w.sum_acc2 += gps_acc_g * gps_acc_g;
  w.sum_axacc += ax * gps_acc_g;
  w.sum_ayacc += ay * gps_acc_g;
  w.sum_abs_acc += fabsf(gps_acc_g);
  w.sum_speed += speed_kmh;
  if (speed_kmh < w.min_speed) w.min_speed = speed_kmh;
  w.sum_gz2 += gz_corr_dps * gz_corr_dps;
  w.sum_cog2 += cog_rate_dps * cog_rate_dps;
  w.sum_lat_expected2 += lat_expected_g * lat_expected_g;
  w.sum_az2 += az * az;
  w.stab_count++;
  w.stab_sum_ax += ax;
  w.stab_sum_ay += ay;
  w.stab_sum_acc += gps_acc_g;
  w.stab_sum_axacc += ax * gps_acc_g;
  w.stab_sum_ayacc += ay * gps_acc_g;
  w.stab_sum_abs_acc += fabsf(gps_acc_g);
  if (w.stab_count >= (uint16_t)FREQ_HZ) {
    flush_mount_yaw_stability_segment(w);
  }
}

static bool compute_mount_yaw_observation(const MountYawWindow& w,
                                          float& yaw_deg,
                                          float& quality,
                                          float& long_corr,
                                          float& lat_rms_before,
                                          float& lat_rms_after,
                                          float& duration_s,
                                          float& mean_speed_kmh,
                                          float& mean_abs_acc_g,
                                          float& rms_gz_dps,
                                          float& rms_cog_rate_dps,
                                          float& rms_lat_expected_g,
                                          float& rms_az_g,
                                          float& stability_std_deg,
                                          uint8_t& stability_n) {
  if (!w.active || w.count < (uint16_t)(MOUNT_YAW_MIN_DURATION_S * FREQ_HZ)) {
    return false;
  }
  const float n = (float)w.count;
  duration_s = (float)(w.end_us - w.start_us) / 1000000.0f;
  if (duration_s < MOUNT_YAW_MIN_DURATION_S) return false;

  const float mean_ax = w.sum_ax / n;
  const float mean_ay = w.sum_ay / n;
  const float var_ax = fmaxf(0.0f, w.sum_ax2 / n - mean_ax * mean_ax);
  const float var_ay = fmaxf(0.0f, w.sum_ay2 / n - mean_ay * mean_ay);
  const float cov_xy = w.sum_axay / n - mean_ax * mean_ay;

  const float trace = var_ax + var_ay;
  const float disc = sqrtf(fmaxf(0.0f, (var_ax - var_ay) * (var_ax - var_ay)
                                      + 4.0f * cov_xy * cov_xy));
  const float lambda_min = 0.5f * (trace - disc);
  float vx = 1.0f;
  float vy = 0.0f;
  if (fabsf(cov_xy) > 1.0e-9f) {
    vx = cov_xy;
    vy = lambda_min - var_ax;
  } else if (var_ay < var_ax) {
    vx = 0.0f;
    vy = 1.0f;
  }
  const float norm_v = sqrtf(vx * vx + vy * vy);
  if (norm_v < 1.0e-9f) return false;
  vx /= norm_v;
  vy /= norm_v;

  yaw_deg = normalize_mount_yaw_deg(atan2f(-vy, vx) / DEG2RAD);
  const float yaw_rad = yaw_deg * DEG2RAD;
  const float c = cosf(yaw_rad);
  const float s = sinf(yaw_rad);

  const float mean_acc = w.sum_acc / n;
  const float var_acc = fmaxf(0.0f, w.sum_acc2 / n - mean_acc * mean_acc);
  const float cov_ax_acc = w.sum_axacc / n - mean_ax * mean_acc;
  const float cov_ay_acc = w.sum_ayacc / n - mean_ay * mean_acc;
  const float var_lat = fmaxf(0.0f, c * c * var_ax + s * s * var_ay - 2.0f * c * s * cov_xy);
  const float var_long = fmaxf(0.0f, s * s * var_ax + c * c * var_ay + 2.0f * s * c * cov_xy);
  const float cov_long_acc = s * cov_ax_acc + c * cov_ay_acc;

  long_corr = 0.0f;
  if (var_long > 1.0e-10f && var_acc > 1.0e-10f) {
    long_corr = cov_long_acc / sqrtf(var_long * var_acc);
  }
  lat_rms_before = sqrtf(var_ax);
  lat_rms_after = sqrtf(var_lat);
  mean_speed_kmh = w.sum_speed / n;
  mean_abs_acc_g = w.sum_abs_acc / n;
  rms_gz_dps = sqrtf(w.sum_gz2 / n);
  rms_cog_rate_dps = sqrtf(w.sum_cog2 / n);
  rms_lat_expected_g = sqrtf(w.sum_lat_expected2 / n);
  rms_az_g = sqrtf(w.sum_az2 / n);
  stability_n = w.stability_n;
  stability_std_deg = (stability_n >= 2)
      ? circular_std_deg(w.stability_sin_sum, w.stability_cos_sum, (float)stability_n)
      : 180.0f;

  float q_duration = clamp01f((duration_s - MOUNT_YAW_MIN_DURATION_S) / 3.5f);
  float q_speed = clamp01f((mean_speed_kmh - MOUNT_YAW_MIN_SPEED_KMH) / 27.0f);
  float q_gz = clamp01f(1.0f - rms_gz_dps / 1.5f);
  float q_cog = clamp01f(1.0f - rms_cog_rate_dps / MOUNT_YAW_MAX_ABS_COG_RATE_DPS);
  float q_lat_expected = clamp01f(1.0f - rms_lat_expected_g / MOUNT_YAW_MAX_LAT_EXPECTED_G);
  float q_acc = clamp01f((mean_abs_acc_g - MOUNT_YAW_MIN_ABS_GPS_ACCEL_G) / 0.075f);
  float q_corr = clamp01f((long_corr - MOUNT_YAW_MIN_LONG_CORR) / 0.6f);
  quality = 0.14f * q_duration + 0.10f * q_speed + 0.12f * q_gz
          + 0.10f * q_cog + 0.14f * q_lat_expected + 0.18f * q_acc
          + 0.22f * q_corr;

  return mean_abs_acc_g >= MOUNT_YAW_MIN_ABS_GPS_ACCEL_G
      && long_corr >= MOUNT_YAW_MIN_LONG_CORR
      && quality >= MOUNT_YAW_MIN_QUALITY;
}

static bool enqueue_mount_yaw_observation(const MountYawWindow& w,
                                          uint8_t obs_id,
                                          float yaw_deg,
                                          float quality,
                                          float long_corr,
                                          float lat_rms_before,
                                          float lat_rms_after,
                                          float duration_s,
                                          float mean_speed_kmh,
                                          float mean_abs_acc_g,
                                          float rms_gz_dps,
                                          float rms_cog_rate_dps,
                                          float rms_lat_expected_g,
                                          float rms_az_g,
                                          float stability_std_deg,
                                          uint8_t stability_n
#ifdef USE_BMI270
                                          , uint32_t& record_seq
#endif
) {
  if (sd_queue == NULL) return false;
  TelemetryRecord rec = {};
  rec.timestamp_us = MOUNT_YAW_OBS_SENTINEL_TS;
  rec.ax = yaw_deg;
  rec.ay = quality;
  rec.az = duration_s;
  rec.gx = mean_speed_kmh;
  rec.gy = mean_abs_acc_g;
  rec.gz = rms_gz_dps;
  rec.temp_c = rms_cog_rate_dps;
  rec.lap = obs_id;
  rec.gps_sog_kmh = long_corr;
  rec.gps_alt_m = lat_rms_before;
  rec.gps_hdop = lat_rms_after;
  rec.kf_x = (lat_rms_before > 1.0e-6f)
      ? 100.0f * (lat_rms_before - lat_rms_after) / lat_rms_before
      : 0.0f;
  rec.kf_y = rms_lat_expected_g;
  rec.kf_vel = rms_az_g;
  rec.kf_heading = stability_std_deg;
  rec.kf6_bgz = (float)stability_n;
  rec.gps_fix_us = w.start_us;
  rec.nav_fix_us = w.end_us;
#ifdef USE_BMI270
  rec.seq = record_seq++;
#endif
  if (xQueueSend(sd_queue, &rec, 0) != pdTRUE) {
    sd_enqueue_fail_count++;
    sd_records_dropped++;
    return false;
  }
  Serial.printf("[MOUNT_YAW] OBS id=%u yaw=%+.2f q=%.2f dur=%.1fs corr=%.2f stab=%.1f n=%u\n",
                (unsigned)obs_id, yaw_deg, quality, duration_s, long_corr,
                stability_std_deg, (unsigned)stability_n);
  return true;
}

static bool enqueue_mount_yaw_boot(const MountYawBootState& boot,
                                   uint64_t now_us
#ifdef USE_BMI270
                                   , uint32_t& record_seq
#endif
) {
  if (sd_queue == NULL) return false;
  TelemetryRecord rec = {};
  rec.timestamp_us = MOUNT_YAW_BOOT_SENTINEL_TS;
  rec.ax = boot.yaw_deg;
  rec.ay = circular_std_deg(boot.sin_sum, boot.cos_sum, (float)boot.obs_count);
  rec.az = (float)boot.obs_count;
  rec.gx = boot.quality_sum / fmaxf(1.0f, (float)boot.obs_count);
  rec.gy = boot.corr_sum / fmaxf(1.0f, (float)boot.obs_count);
  rec.gz = boot.stability_sum / fmaxf(1.0f, (float)boot.obs_count);
  rec.gps_sog_kmh = boot.sin_yaw;
  rec.gps_alt_m = boot.cos_yaw;
  rec.gps_fix_us = now_us;
  rec.nav_fix_us = now_us;
#ifdef USE_BMI270
  rec.seq = record_seq++;
#endif
  if (xQueueSend(sd_queue, &rec, 0) != pdTRUE) {
    sd_enqueue_fail_count++;
    sd_records_dropped++;
    return false;
  }
  Serial.printf("[MOUNT_YAW] BOOT yaw=%+.2f std=%.1f obs=%u q=%.2f corr=%.2f\n",
                boot.yaw_deg,
                rec.ay,
                (unsigned)boot.obs_count,
                rec.gx,
                rec.gy);
  return true;
}

static bool consider_mount_yaw_boot(MountYawState& state,
                                    float yaw_deg,
                                    float quality,
                                    float long_corr,
                                    float stability_std_deg,
                                    uint64_t now_us
#ifdef USE_BMI270
                                    , uint32_t& record_seq
#endif
) {
  if (state.boot.active || state.boot.emitted) return false;
  if (quality < MOUNT_YAW_BOOT_MIN_QUALITY ||
      long_corr < MOUNT_YAW_BOOT_MIN_LONG_CORR ||
      stability_std_deg > MOUNT_YAW_BOOT_MAX_OBS_STABILITY_STD_DEG) {
    return false;
  }

  const float yaw_rad = normalize_mount_yaw_deg(yaw_deg) * DEG2RAD;
  state.boot.sin_sum += sinf(yaw_rad) * quality;
  state.boot.cos_sum += cosf(yaw_rad) * quality;
  state.boot.quality_sum += quality;
  state.boot.corr_sum += long_corr;
  state.boot.stability_sum += stability_std_deg;
  if (state.boot.obs_count < 255) state.boot.obs_count++;

  if (state.boot.obs_count < MOUNT_YAW_BOOT_MIN_OBS) {
    return false;
  }

  const float consensus_std_deg =
      circular_std_deg(state.boot.sin_sum, state.boot.cos_sum, state.boot.quality_sum);
  if (consensus_std_deg > MOUNT_YAW_BOOT_MAX_CONSENSUS_STD_DEG) {
    return false;
  }

  state.boot.yaw_rad = atan2f(state.boot.sin_sum, state.boot.cos_sum);
  state.boot.yaw_deg = normalize_mount_yaw_deg(state.boot.yaw_rad / DEG2RAD);
  state.boot.yaw_rad = state.boot.yaw_deg * DEG2RAD;
  state.boot.sin_yaw = sinf(state.boot.yaw_rad);
  state.boot.cos_yaw = cosf(state.boot.yaw_rad);

  if (!state.boot.emitted) {
    state.boot.emitted = enqueue_mount_yaw_boot(state.boot,
                                                now_us
#ifdef USE_BMI270
                                                , record_seq
#endif
    );
    if (!state.boot.emitted) {
      Serial.println("[MOUNT_YAW] BOOT recommendation dropped: SD queue unavailable/full.");
      return false;
    }
  }
  if (MOUNT_YAW_APPLY_LIVE) {
    state.boot.active = true;
    state.boot.just_activated = true;
  } else {
    state.boot.active = false;
    state.boot.just_activated = false;
    Serial.println("[MOUNT_YAW] BOOT recommendation logged only; live frame unchanged.");
  }
  return true;
}

static void finalize_mount_yaw_window(MountYawState& state,
                                      uint64_t now_us
#ifdef USE_BMI270
                                      , uint32_t& record_seq
#endif
) {
  if (!state.win.active) return;
  flush_mount_yaw_stability_segment(state.win);
  float yaw_deg = 0.0f;
  float quality = 0.0f;
  float long_corr = 0.0f;
  float lat_rms_before = 0.0f;
  float lat_rms_after = 0.0f;
  float duration_s = 0.0f;
  float mean_speed_kmh = 0.0f;
  float mean_abs_acc_g = 0.0f;
  float rms_gz_dps = 0.0f;
  float rms_cog_rate_dps = 0.0f;
  float rms_lat_expected_g = 0.0f;
  float rms_az_g = 0.0f;
  float stability_std_deg = 180.0f;
  uint8_t stability_n = 0;

  if (compute_mount_yaw_observation(state.win,
                                    yaw_deg,
                                    quality,
                                    long_corr,
                                    lat_rms_before,
                                    lat_rms_after,
                                    duration_s,
                                    mean_speed_kmh,
                                    mean_abs_acc_g,
                                    rms_gz_dps,
                                    rms_cog_rate_dps,
                                    rms_lat_expected_g,
                                    rms_az_g,
                                    stability_std_deg,
                                    stability_n)) {
    state.obs_id++;
    enqueue_mount_yaw_observation(state.win,
                                  state.obs_id,
                                  yaw_deg,
                                  quality,
                                  long_corr,
                                  lat_rms_before,
                                  lat_rms_after,
                                  duration_s,
                                  mean_speed_kmh,
                                  mean_abs_acc_g,
                                  rms_gz_dps,
                                  rms_cog_rate_dps,
                                  rms_lat_expected_g,
                                  rms_az_g,
                                  stability_std_deg,
                                  stability_n
#ifdef USE_BMI270
                                  , record_seq
#endif
    );
    consider_mount_yaw_boot(state,
                            yaw_deg,
                            quality,
                            long_corr,
                            stability_std_deg,
                            now_us
#ifdef USE_BMI270
                            , record_seq
#endif
    );
    state.cooldown_until_us = now_us + (uint64_t)(MOUNT_YAW_COOLDOWN_S * 1000000.0f);
  }
  reset_mount_yaw_window(state.win);
}

void Task_Filter(void *pvParameters) {
  ImuRawData data;
  uint32_t last_eskf_epoch = 0; // last GPS epoch processed by the ESKF
  uint64_t last_timestamp_us = 0;
  MountYawState mount_yaw;
  GpsCorrectionGateState gps_gate;
#ifdef USE_BMI270
  uint32_t record_seq = 0;
#endif

  // ── v1.3.2: function-scope state (was static inside loop) ─────────────────
  // Moved here so calibrate_alignment() can trigger a full reset via
  // recalibration_pending without needing access to Task_Filter internals.
  float gz_var_buf[VAR_BUF_SIZE] = {};
  float gx_var_buf[VAR_BUF_SIZE] = {};  // v1.2.1: ZARU extended to gx
  float gy_var_buf[VAR_BUF_SIZE] = {};  // v1.2.1: ZARU extended to gy
  int gz_var_idx = 0;
  int gz_var_count = 0;
  float thermal_bias_gz = 0.0f;  // [°/s] learned dynamic thermal offset
  float thermal_bias_gx = 0.0f;  // [°/s] v1.2.1
  float thermal_bias_gy = 0.0f;  // [°/s] v1.2.1
  GpsData last_gps = {};         // GPS snapshot cache (anti-Null-Island)
  bool gps_ever_had_fix = false; // cold start "no fix yet" is not GPS lost
  float cog_ref_east = 0.0f;
  float cog_ref_north = 0.0f;
  float prev_cog_rad = 0.0f;  // AUDIT-NOTE (PIPELINE-MINOR-4, not applied):
                               // first COG variation is computed against 0.0f
                               // (due-North) instead of actual initial heading.
                               // If heading is near 0 at acquisition, the gate
                               // may open one baseline early. Self-corrects
                               // after the second baseline. Straight-Line ZARU
                               // requires speed >40 km/h — cannot trigger in
                               // the first seconds, so impact is negligible.
  bool has_cog_ref = false;
  float cog_variation = 99.0f;   // default: no straight detected
  bool first_sample_after_recalib = false;
  float prev_cent_x = 0.0f;      // v1.4.1: VGPL lateral rate limiter state
  float prev_long_y = 0.0f;      // v1.4.1: VGPL longitudinal rate limiter state
  float prev_v_eskf = 0.0f;      // v1.4.1: previous velocity for dv/dt
#ifdef USE_BMI270
  uint32_t imu_stale_consecutive = 0;
#endif

  for (;;) {
    if (xQueueReceive(imuQueue, &data, portMAX_DELAY) == pdTRUE) {
#ifdef USE_BMI270
      if (!data.imu_sample_fresh) {
        if (imu_stale_consecutive < UINT32_MAX) {
          imu_stale_consecutive++;
        }
        if (imu_stale_consecutive == 1U ||
            imu_stale_consecutive == 5U ||
            (imu_stale_consecutive % 50U) == 0U) {
          Serial.printf("[FILTER] IMU stale sample skipped: count=%lu backlog=%u overrun=%u\n",
                        (unsigned long)imu_stale_consecutive,
                        (unsigned)data.fifo_backlog,
                        data.fifo_overrun ? 1U : 0U);
        }
        continue;
      }
      if (imu_stale_consecutive > 0U) {
        Serial.printf("[FILTER] IMU fresh again after %lu stale sample(s).\n",
                      (unsigned long)imu_stale_consecutive);
        imu_stale_consecutive = 0;
      }
#endif

      // Dynamic DT from hardware timestamp: compensates scheduler jitter.
      float dt_real_sec = DT;
      if (last_timestamp_us > 0) {
        uint64_t dt_us = data.timestamp_us - last_timestamp_us;
        dt_real_sec = (float)dt_us / 1000000.0f;
        if (dt_real_sec <= 0.0f || dt_real_sec > 1.0f)
          dt_real_sec = DT;
      }
      last_timestamp_us = data.timestamp_us;

      // ── PHASE 1: First telemetry_mutex — Steps 1-6 only ─────────────────
      // Scope: calibration → gravity removal.
      // EMA is no longer computed here (moved to end-of-pipe after ZARU).
      // prev_* globals are read into locals so the EMA can be computed later
      // without holding the mutex. Writing back to prev_* happens in Phase 4.
      //
      // AUDIT-NOTE (CROSS-MINOR-3, updated v1.3.5): mutex now covers Steps 1-6
      // only (~300 µs, ~1.5% of 20 ms period). Previously held through Steps 7-8
      // (EMA + variance, ~500 µs). The remaining concern is calibrate_alignment(),
      // which zeroes prev_ax..prev_gz under telemetry_mutex. If it runs between
      // Phase 1 (read) and Phase 4 (write-back), Phase 4 would overwrite those
      // zeros with EMA values from the stale locals. This is safe: calibrate_
      // alignment() also sets recalibration_pending=true, which is checked at
      // the top of the next iteration and triggers a full reset (including
      // prev_* → 0). One-cycle inconsistency at 50 Hz — not observable.
      float local_prev_ax, local_prev_ay, local_prev_az;
      float local_prev_gx, local_prev_gy, local_prev_gz;
      uint8_t lap_snap;

      // Intermediate pipeline results declared here; used across phases.
      float ax_r, ay_r, az_r;
      float gx_r, gy_r, gz_r;
      float gx_rad, gy_rad, gz_rad;          // ZARU-corrected [rad/s] → Madgwick, VGPL, ESKF 5D
      float gx_rad_raw, gy_rad_raw, gz_rad_raw;  // uncorrected [rad/s] → ESKF 6D shadow, variance buf
      float lin_ax, lin_ay, lin_az;

      if (xSemaphoreTake(telemetry_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {

        // ── RECALIBRATION RESET (v1.3.2) ────────────────────────────────────
        // calibrate_alignment() sets this flag under telemetry_mutex after
        // writing new biases and resetting the quaternion. Task_Filter clears
        // it here, discards the stale IMU sample that was queued during the
        // 2 s calibration window, and resets all navigation/filter state.
        if (recalibration_pending.load()) {
          recalibration_pending = false;
          eskf.reset();
          eskf6.reset();
          gps_origin_set = false;
          memset(gz_var_buf, 0, sizeof(gz_var_buf));
          memset(gx_var_buf, 0, sizeof(gx_var_buf));
          memset(gy_var_buf, 0, sizeof(gy_var_buf));
          gz_var_idx = 0;
          gz_var_count = 0;
          thermal_bias_gz = 0.0f;
          thermal_bias_gx = 0.0f;
          thermal_bias_gy = 0.0f;
          cog_ref_east = 0.0f;
          cog_ref_north = 0.0f;
          prev_cog_rad = 0.0f;
          has_cog_ref = false;
          cog_variation = 99.0f;
          last_gps = GpsData{};
          last_eskf_epoch = 0;
          last_timestamp_us = 0;
          first_sample_after_recalib = true;
          prev_cent_x = 0.0f;  // VGPL: reset stale centripetal compensation
          prev_long_y = 0.0f;  // VGPL: reset stale longitudinal compensation
          prev_v_eskf = 0.0f;
          mount_yaw = MountYawState{};
          gps_gate = GpsCorrectionGateState{};
          Serial.println("[FILTER] Recalibration reset complete.");
          xSemaphoreGive(telemetry_mutex);
          continue; // discard stale IMU sample, start fresh
        }

        // STEP 0.5: explicit chip -> pipeline convention remap.
        float acc_chip_x = data.bmi_acc_x_g;
        float acc_chip_y = data.bmi_acc_y_g;
        float acc_chip_z = data.bmi_acc_z_g;
        float gyr_chip_x = data.bmi_gyr_x_dps;
        float gyr_chip_y = data.bmi_gyr_y_dps;
        float gyr_chip_z = data.bmi_gyr_z_dps;
        remap_chip_axes_to_pipeline(acc_chip_x, acc_chip_y, acc_chip_z);
        remap_chip_axes_to_pipeline(gyr_chip_x, gyr_chip_y, gyr_chip_z);

        // STEP 1: Ellipsoidal calibration (pipeline convention input frame)
        float ax_cal = acc_chip_x, ay_cal = acc_chip_y, az_cal = acc_chip_z;
        apply_ellipsoidal_calibration(ax_cal, ay_cal, az_cal);

        // STEP 2: Geometric alignment (vehicle frame)
        float ax_r_raw, ay_r_raw, az_r_raw;
        rotate_3d(ax_cal, ay_cal, az_cal, ax_r_raw, ay_r_raw, az_r_raw, cos_phi,
                 sin_phi, cos_theta, sin_theta);

        // STEP 3: Mounting bias (post-ellipsoid residuals)
        ax_r = ax_r_raw - bias_ax;
        ay_r = ay_r_raw - bias_ay;
        az_r = az_r_raw - bias_az;
        if (MOUNT_YAW_APPLY_LIVE && mount_yaw.boot.active) {
          rotate_mount_yaw_xy(mount_yaw.boot.yaw_rad, ax_r, ay_r);
        }

        // STEP 4: Gyroscope — electronic bias + vehicle-frame rotation
        // K_gs (G-sensitivity matrix) was removed in v1.3.0: the MPU-6886 bias
        // drift (~0.08 °/s/min on gx/gy, non-thermal) dominates the G-sensitivity
        // signal (~0.5 °/s), making K_gs unreliable. ZARU 3-axis handles both
        // thermal and electronic drift in real-time. See tel_94 validation test.
        float gx_clean = gyr_chip_x - bias_gx;
        float gy_clean = gyr_chip_y - bias_gy;
        float gz_clean = gyr_chip_z - bias_gz;

        rotate_3d(gx_clean, gy_clean, gz_clean, gx_r, gy_r, gz_r, cos_phi,
                 sin_phi, cos_theta, sin_theta);
        if (MOUNT_YAW_APPLY_LIVE && mount_yaw.boot.active) {
          rotate_mount_yaw_xy(mount_yaw.boot.yaw_rad, gx_r, gy_r);
        }
        if (MOUNT_YAW_APPLY_LIVE && mount_yaw.boot.just_activated) {
          // The mount-yaw correction changes the vehicle X/Y basis. Clear
          // frame-dependent learnt state before the corrected frame feeds ZARU,
          // Madgwick and the ESKF on subsequent samples.
          thermal_bias_gx = 0.0f;
          thermal_bias_gy = 0.0f;
          thermal_bias_gz = 0.0f;
          memset(gz_var_buf, 0, sizeof(gz_var_buf));
          memset(gx_var_buf, 0, sizeof(gx_var_buf));
          memset(gy_var_buf, 0, sizeof(gy_var_buf));
          gz_var_idx = 0;
          gz_var_count = 0;
          has_cog_ref = false;
          cog_variation = 0.0f;
          prev_cent_x = 0.0f;
          prev_long_y = 0.0f;
          ahrs.q[0] = 1.0f; ahrs.q[1] = 0.0f; ahrs.q[2] = 0.0f; ahrs.q[3] = 0.0f;
          first_sample_after_recalib = true;
          mount_yaw.boot.just_activated = false;
        }

        // STEP 4b: ZARU→Madgwick bias pre-correction (v1.3.5)
        // ─────────────────────────────────────────────────────────────────
        // thermal_bias_* from the PREVIOUS cycle's ZARU update (20 ms delay,
        // negligible). Corrected values feed Madgwick and VGPL; raw values
        // preserved for ESKF 6D shadow and the variance buffer.
        //
        // Without this correction, Madgwick receives gx in electronic drift
        // (-4.7 °/s/hr on the author's MPU-6886, per BiasDrift_Report).
        // With VGPL_BETA_FLOOR=0.005, the equilibrium horizon tilt reaches
        // ω_bias/(2·β·Fs) ≈ 2.35°/s / (2×0.005×50) ≈ 4.7° after 30 min,
        // projecting sin(4.7°)×1g ≈ 0.082g phantom acceleration into
        // lin_ax/lin_ay. This poisons the ESKF velocity estimate and
        // permanently blocks the Straight-Line ZARU gate (|lin_ax|<0.05g).
        gx_rad_raw = gx_r * DEG2RAD;
        gy_rad_raw = gy_r * DEG2RAD;
        gz_rad_raw = gz_r * DEG2RAD;

        gx_rad = (gx_r - thermal_bias_gx) * DEG2RAD;  // ZARU-corrected
        gy_rad = (gy_r - thermal_bias_gy) * DEG2RAD;
        gz_rad = (gz_r - thermal_bias_gz) * DEG2RAD;

        // STEP 5: Madgwick AHRS with VGPL centripetal compensation (v1.3.4/v1.3.5)
        // ───────────────────────────────────────────────────────────────────
        // The Dual-Gate Adaptive Beta (v1.1.1) failed: forcing beta=0 during
        // cornering caused unbounded quaternion drift from MPU-6886 gx/gy bias
        // (0.076 °/s/min electronic drift), producing -0.5G false deceleration.
        //
        // VGPL fix: subtract estimated centripetal acceleration from the accel
        // vector BEFORE feeding it to Madgwick. After compensation, the accel
        // reads ~1G (pure gravity) even in cornering → beta stays > 0 always
        // → accelerometer continuously corrects gyro bias → no drift.
        //
        // Centripetal in body frame: a_cent_y = v * ω / g  [G]
        // Uses ESKF velocity from previous cycle (20 ms delay → <0.005G error).
        // Rate-limited to suppress gy vibration/spike-induced transients.
        // Confidence-gated via ESKF P matrix: degrades to uncompensated Madgwick
        // when GPS is lost and velocity estimate becomes unreliable.

        float v_eskf = eskf.speed_ms();  // from previous cycle (20 ms old)

        // Longitudinal derivative before updating prev_v_eskf
        float dv_dt = (v_eskf - prev_v_eskf) / dt_real_sec;
        prev_v_eskf = v_eskf;

        // Confidence gate: ESKF velocity variance → compensation trust
        // P(2,2)+P(3,3) = velocity uncertainty. Low = GPS-fused, high = dead-reckoning.
        float v_var = eskf.P(2, 2) + eskf.P(3, 3);
        float v_confidence = fminf(1.0f, 1.0f / (1.0f + v_var));

        // 1. Centripetal estimate: body-frame lateral (X axis) [G]
        // In a left turn (v > 0, gz > 0), acceleration is to the left (-X).
        float cent_x_raw = -(v_eskf * gz_rad) / G_ACCEL;
        float cent_x = cent_x_raw * v_confidence;

        // Rate limiter: prevents spike on gz noise or sudden ESKF correction
        if (cent_x > prev_cent_x + VGPL_RATE_LIMIT) cent_x = prev_cent_x + VGPL_RATE_LIMIT;
        if (cent_x < prev_cent_x - VGPL_RATE_LIMIT) cent_x = prev_cent_x - VGPL_RATE_LIMIT;
        prev_cent_x = cent_x;

        // 2. Longitudinal estimate: body-frame forward (Y axis) [G]
        // Forward acceleration (dv/dt > 0) -> +Y.
        float long_y_raw = dv_dt / G_ACCEL;
        float long_y = long_y_raw * v_confidence;

        if (long_y > prev_long_y + VGPL_RATE_LIMIT) long_y = prev_long_y + VGPL_RATE_LIMIT;
        if (long_y < prev_long_y - VGPL_RATE_LIMIT) long_y = prev_long_y - VGPL_RATE_LIMIT;
        prev_long_y = long_y;

        // Compensated accel for Madgwick (≈ pure gravity reference)
        // Subtract kinematic accelerations so Madgwick only sees gravity
        float ax_madg = ax_r - cent_x;   // lateral: subtract centripetal
        float ay_madg = ay_r - long_y;   // longitudinal: subtract dv/dt
        float az_madg = az_r;            // vertical: untouched

        ahrs.sampleperiod = dt_real_sec;
        // v1.3.5: gx/gy/gz_rad are ZARU-corrected → Madgwick sees true-zero-mean
        // gyro, preventing horizon tilt from electronic bias drift.
        ahrs.update_imu(gx_rad, gy_rad, gz_rad, ax_madg, ay_madg, az_madg);

        // STEP 6: Gravity removal (from RAW accel, not compensated)
        // The gravity vector estimated by Madgwick is now accurate because
        // the compensated input kept beta > 0. But lin_a must use the RAW
        // accelerometer (ax_r, ay_r) to preserve real dynamic forces.
        float grav_x, grav_y, grav_z;
        ahrs.get_gravity_vector(grav_x, grav_y, grav_z);
        lin_ax = ax_r - grav_x;
        lin_ay = ay_r - grav_y;
        lin_az = az_r - grav_z;
        update_mount_yaw_imu_history(mount_yaw, data.timestamp_us, lin_ax, lin_ay, lin_az);

        // Snapshot prev_* EMA globals for use in the end-of-pipe EMA (Phase 3).
        // calibrate_alignment() may zero these between now and Phase 4 write-back,
        // but recalibration_pending ensures the next iteration resets everything.
        local_prev_ax = prev_ax; local_prev_ay = prev_ay; local_prev_az = prev_az;
        local_prev_gx = prev_gx; local_prev_gy = prev_gy; local_prev_gz = prev_gz;

        // lap_snap captured inside the lock: consistent with the pipeline cycle.
        lap_snap = (system_state == 2) ? 1 : 0;

        xSemaphoreGive(telemetry_mutex);
      } else {
        continue; // mutex acquisition failed — skip this sample
      }

      // ── PHASE 2: Variance buffer + GPS + ZARU + ESKF (no mutex) ──────────
      // All state accessed here (variance buffers, thermal_bias_*, ESKF objects,
      // last_gps) is owned exclusively by Task_Filter (no shared global).

      // ── STEP 8: Statistical ZUPT/ZARU Engine — 3-axis (v1.2.1 / v1.3.5) ──
      // Circular buffer of 50 samples (1 s at 50 Hz) of RAW vehicle-frame gyro.
      // v1.3.5 change: buffers now store gz_r/gx_r/gy_r (post-rotation, pre-EMA)
      // instead of ema_gz/gx/gy. This eliminates the ~313 ms EMA decay lag from
      // the detection path: when the vehicle stops abruptly, raw variance spikes
      // immediately while EMA variance would have remained low for ~1.6 s
      // (causing false stationary detection and a spurious thermal_bias capture).
      // gx/gy share the same index and warm-up counter: no extra overhead.
      // NOTE: buffers are pre-ZARU-correction to avoid feedback loop — thermal_bias
      // is subtracted only at the output (EMA layer / display / log), not here.
      //
      // Warm-up: variance is not computed until gz_var_count >= VAR_BUF_SIZE
      // → prevents false positives from a partially filled buffer.
      gz_var_buf[gz_var_idx] = gz_r;
      gx_var_buf[gz_var_idx] = gx_r;
      gy_var_buf[gz_var_idx] = gy_r;
      gz_var_idx = (gz_var_idx + 1) % VAR_BUF_SIZE;
      if (gz_var_count < VAR_BUF_SIZE) gz_var_count++;

      // Variance and mean computed only after complete warm-up.
      // var_gz and mean_gz determine is_stationary.
      // var_gx/gy are the 3-axis gate; mean_gx/gy update thermal_bias_gx/gy.
      float var_gz = -1.0f;  // sentinel: <0 = warm-up incomplete
      float var_gx = -1.0f;
      float var_gy = -1.0f;
      float mean_gz = 0.0f;
      float mean_gx = 0.0f;
      float mean_gy = 0.0f;
      if (gz_var_count >= VAR_BUF_SIZE) {
        float sum    = 0.0f, sum_sq    = 0.0f;
        float sum_gx = 0.0f, sum_sq_gx = 0.0f;
        float sum_gy = 0.0f, sum_sq_gy = 0.0f;
        for (int i = 0; i < VAR_BUF_SIZE; i++) {
          sum       += gz_var_buf[i];
          sum_sq    += gz_var_buf[i] * gz_var_buf[i];
          sum_gx    += gx_var_buf[i];
          sum_sq_gx += gx_var_buf[i] * gx_var_buf[i];
          sum_gy    += gy_var_buf[i];
          sum_sq_gy += gy_var_buf[i] * gy_var_buf[i];
        }
        mean_gz = sum    / (float)VAR_BUF_SIZE;
        mean_gx = sum_gx / (float)VAR_BUF_SIZE;
        mean_gy = sum_gy / (float)VAR_BUF_SIZE;
        // fmaxf: prevents numerically negative variance from floating-point
        // cancellation when samples are nearly identical (vehicle stationary).
        var_gz = fmaxf(0.0f, (sum_sq    / (float)VAR_BUF_SIZE) - (mean_gz * mean_gz));
        var_gx = fmaxf(0.0f, (sum_sq_gx / (float)VAR_BUF_SIZE) - (mean_gx * mean_gx));
        var_gy = fmaxf(0.0f, (sum_sq_gy / (float)VAR_BUF_SIZE) - (mean_gy * mean_gy));
      }
      // ──────────────────────────────────────────────────────────────────────

      // ── DATA FORK ──────────────────────────────────────────────────────────
      // The ESKF is fed RAW post-Madgwick IMU data:
      //   - lin_ax, lin_ay: linear acceleration [g] (gravity removed via
      //     Madgwick quaternion) → zero phase latency.
      //   - gz_rad: yaw angular rate [rad/s] (raw gyro in vehicle frame)
      //     corrected by the learned thermal_bias_gz.
      //
      // EMA is computed AFTER all navigation updates (end-of-pipe). It does NOT
      // enter the ESKF and is used exclusively for display/MQTT/SD logging.
      // ──────────────────────────────────────────────────────────────────────

      // GPS snapshot for SD and ESKF
      if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        last_gps = shared_gps_data;
        xSemaphoreGive(gps_mutex);
      } else {
        gps_mutex_timeout_count++;
      }

      // ── GPS Velocity Source Selection (v1.8.10) ───────────────────────────
      // NAV2-PVH speed2D is the primary scalar speed when fresh/valid. NMEA SOG
      // remains the fallback. DHV is retained as a 1 Hz diagnostic only.
      // NAV2 velE/velN are transformed by evaluate_nav2_pvh() for diagnostics
      // and future vector correction, but this patch uses only scalar speed2D.
      const Nav2PvhSelection nav2_speed =
          evaluate_nav2_pvh(last_gps, data.timestamp_us);
      float gps_speed_kmh_used = nav2_speed.ok
                                     ? nav2_speed.speed_kmh
                                     : last_gps.sog_kmh;
      uint8_t gps_speed_source = nav2_speed.ok
                                     ? GPS_SPD_NAV2_PVH
                                     : GPS_SPD_NMEA_SOG;
      const bool gps_has_fix = last_gps.valid && last_gps.fix_us > 0ULL;
      const uint64_t gps_age_us =
          (gps_has_fix && data.timestamp_us >= last_gps.fix_us)
              ? (data.timestamp_us - last_gps.fix_us)
              : UINT64_MAX;
      const bool gps_fix_fresh = gps_has_fix && gps_age_us <= GPS_FRESH_MAX_US;
      if (gps_has_fix) {
        gps_ever_had_fix = true;
      }
      // Cold start with 0 satellites is "acquiring", not "lost". Raise the
      // red GPS LOST alarm only after the unit has seen at least one real fix.
      const bool gps_lost_after_fix =
          gps_ever_had_fix && (!gps_has_fix || gps_age_us > GPS_STALE_US);
      gps_stale = gps_lost_after_fix;

      // ── Stationary Condition (Statistical Engine + GPS) ───────────────────
      // is_stationary = true ONLY IF:
      //   1. Buffer warm-up is complete (50 samples collected)
      //   2. gz, gx AND gy raw variance are all below their respective noise floors
      //      (3-axis gate: prevents false positives from chassis vibration coupling
      //      into roll/pitch even when gz happens to be quiet)
      //   3. Fresh GPS speed is < 2 km/h (stale/no-fix GPS never asserts slow)
      //   4. |mean_gz| < 2.5°/s (Sanity Gate v0.9.9)
      bool is_stationary = false;
      if (var_gz >= 0.0f) { // warm-up complete
        bool gps_slow = gps_fix_fresh &&
                        (gps_speed_kmh_used < ZUPT_GPS_MAX_KMH);
        // Sanity Gate: mean_gz < 2.5°/s excludes slow rotational manoeuvres.
        // Real thermal bias does not exceed ~1.5°/s
        // (30°C swing × 0.032°/s/°C ≈ 1°/s for MPU-6886).
        // A 1.6 km/h manoeuvre produces 8-10°/s → blocked.
        // Threshold 2.5 leaves margin for extreme bias without deadlock.
        is_stationary = (var_gz < VAR_STILLNESS_GZ_THRESHOLD)
                     && (var_gx < VAR_STILLNESS_GXY_THRESHOLD)
                     && (var_gy < VAR_STILLNESS_GXY_THRESHOLD)
                     && gps_slow
                     && (fabsf(mean_gz) < 2.5f);
      }

      // ── Adaptive ZARU (Thermal Bias Learning) ─────────────────────────────
      // When the vehicle is stationary, the raw gz buffer mean represents
      // the residual thermal drift of gyro Z. This value is saved as
      // thermal_bias_gz and subtracted from gz_rad before feeding the ESKF
      // and from gz_r before feeding the end-of-pipe EMA.
      // In motion, the bias learned at standstill keeps compensating the
      // drift without introducing latency.
      if (is_stationary) {
        thermal_bias_gz = mean_gz; // [°/s], instant learning
        thermal_bias_gx = mean_gx; // v1.2.1
        thermal_bias_gy = mean_gy; // v1.2.1
      }

      // ── Enhanced Straight-Line ZARU (v1.3.1) ──────────────────────────────
      // Triple gate: lateral accel + yaw rate + COG variation.
      // COG computed over a long baseline (>15 m displacement) to
      // suppress GPS position noise (σ_COG ≈ σ_pos/baseline).
      // Learning uses mean from variance buffer (not ema) for
      // consistency with the static ZARU path.
      //
      // Previous version (v0.9.7): only lat + gz gates, no COG.
      // BUG-3 fixes preserved: speed 40 km/h, lin_ax (not ema_ax).
      // v1.3.2: cog_ref_east/north/prev_cog_rad/has_cog_ref/cog_variation
      // are now function-scope (moved for recalibration reset support).
      // v1.3.5: gz_gate uses RAW corrected data (gz_r - thermal_bias_gz) for
      // zero-latency response. The previous version used EMA-filtered data
      // (local_prev_gz, τ≈313 ms delay) which held the gate open for several
      // hundred ms after corner entry, allowing the ZARU to capture the
      // initial yaw rate as kinematic contamination in the thermal bias.
      // A spurious single-sample closure from MEMS noise in a straight is
      // harmless: with STRAIGHT_ALPHA=0.01, one missed sample out of 2500/s
      // has zero practical impact on bias convergence.

      bool straight_zaru_active = false;
      if (!is_stationary && gps_fix_fresh &&
          gps_speed_kmh_used > STRAIGHT_MIN_SPEED_KMH) {
        bool lat_gate = fabsf(lin_ax) < STRAIGHT_MAX_LAT_G;
        bool gz_gate  = fabsf(gz_r - thermal_bias_gz) < 2.0f;  // RAW corrected, zero latency
        bool cog_gate = fabsf(cog_variation) < STRAIGHT_COG_MAX_RAD;

        if (lat_gate && gz_gate && cog_gate) {
          thermal_bias_gz = STRAIGHT_ALPHA * mean_gz + (1.0f - STRAIGHT_ALPHA) * thermal_bias_gz;
          thermal_bias_gx = STRAIGHT_ALPHA * mean_gx + (1.0f - STRAIGHT_ALPHA) * thermal_bias_gx;
          thermal_bias_gy = STRAIGHT_ALPHA * mean_gy + (1.0f - STRAIGHT_ALPHA) * thermal_bias_gy;
          straight_zaru_active = true;
        }
      }

      // ── ESKF 5D vs 6D: different bias strategies ──────────────────────────
      // 5D: bias is removed externally by ZARU. Since v1.3.5, gz_rad is already
      //     ZARU-corrected at source (Step 4b) → fed directly, no subtraction.
      //     The 5D state has no bias term: X=[px,py,vx,vy,θ].
      // 6D: bias is estimated internally as X[5]=b_gz (random walk model).
      //     It receives gz_rad_raw (uncorrected) so it can learn the bias
      //     independently. Feeding the corrected gz_rad would cause the
      //     internal bias state to collapse to ≈0, defeating the shadow
      //     comparison purpose.
      // Having two independent bias estimates (ZARU statistical vs Kalman
      // model-based) enables offline A/B validation via kf6_bgz in the CSV.
      eskf.predict(lin_ax, lin_ay, gz_rad, dt_real_sec, is_stationary);

      eskf6.predict(lin_ax, lin_ay, gz_rad_raw, dt_real_sec, is_stationary);

      // 6D ZUPT bias: when stationary, measured gz ≈ pure bias → observation
      if (is_stationary) {
        eskf6.correct_bias(gz_rad_raw, 0.001f);
      }

      // ── NHC: Non-Holonomic Constraint (v1.3.1) ────────────────────────────
      // v_lateral = 0 pseudo-measurement: constrains heading drift
      // continuously while in motion (> 5 km/h). Disabled during
      // extreme lateral dynamics (|lin_ax| > 0.5 g).
      bool nhc_active = false;
      if (eskf.speed_ms() > NHC_MIN_SPEED_MS &&
          fabsf(lin_ax) < NHC_MAX_LAT_G) {
        eskf.correct_nhc(NHC_R);
        eskf6.correct_nhc(NHC_R);
        nhc_active = true;
      }

      // ── GPS Correction Freshness ──────────────────────────────────────────
      // gps_fix_fresh was computed before ZARU so stale/no-fix GPS cannot
      // assert vehicle stillness. Corrections below also require a fresh epoch;
      // gps_stale is only set after a previously acquired fix is lost/stale.

      // Correct: only if there is a new GPS fix AND it is not stale
      if (gps_fix_fresh && last_gps.epoch != last_eskf_epoch) {
        bool gps_fix_accepted = gps_fix_quality_ok(last_gps);
        if (!gps_fix_accepted) {
          last_eskf_epoch = last_gps.epoch;
        } else {
          // First valid coordinate → set the ENU origin
          if (!gps_origin_set) {
            gps_origin_lat = last_gps.lat;
            gps_origin_lon = last_gps.lon;
            gps_origin_set = true;
            eskf.reset();  // clean state centred on origin
            eskf6.reset();
            gps_gate = GpsCorrectionGateState{};
            Serial.printf("[ESKF] ENU origin: %.6f, %.6f\n",
                          gps_origin_lat, gps_origin_lon);
          }
          float east_m, north_m;
          wgs84_to_enu(last_gps.lat, last_gps.lon,
                       gps_origin_lat, gps_origin_lon,
                       east_m, north_m);
          gps_fix_accepted =
              gps_correction_gate_allows(gps_gate,
                                         east_m,
                                         north_m,
                                         gps_speed_kmh_used,
                                         last_gps.hdop,
                                         last_gps.fix_us);
          if (gps_fix_accepted) {
            eskf.correct(east_m, north_m, gps_speed_kmh_used, last_gps.hdop);
            eskf6.correct(east_m, north_m, gps_speed_kmh_used, last_gps.hdop);
            gps_correction_gate_accept(gps_gate,
                                       east_m,
                                       north_m,
                                       gps_speed_kmh_used,
                                       last_gps.fix_us);
          } else if (gps_gate.reject_count < 255) {
            gps_gate.reject_count++;
            if (gps_gate.reject_count == 1U ||
                (gps_gate.reject_count % 10U) == 0U) {
              Serial.printf("[ESKF] GPS correction rejected: dist gate count=%u hdop=%.2f speed=%.1f\n",
                            (unsigned)gps_gate.reject_count,
                            last_gps.hdop,
                            gps_speed_kmh_used);
            }
          }
          last_eskf_epoch = last_gps.epoch;

          // ── COG variation for Enhanced Straight-Line ZARU (v1.3.1) ──────────
          // Long-baseline approach: COG computed only after >15 m displacement
          // from the reference position. Suppresses GPS position noise
          // (σ_COG ≈ σ_pos/baseline ≈ 1.5m/15m ≈ 0.1 rad).
          if (gps_fix_accepted && gps_speed_kmh_used > 20.0f) {
            if (!has_cog_ref) {
              cog_ref_east = east_m;
              cog_ref_north = north_m;
              has_cog_ref = true;
            } else {
              float de = east_m - cog_ref_east;
              float dn = north_m - cog_ref_north;
              float dist = sqrtf(de * de + dn * dn);
              if (dist > COG_MIN_BASELINE_M) {
                float current_cog = atan2f(de, dn);
                cog_variation = current_cog - prev_cog_rad;
                // wrap to [-π, π]
                if (cog_variation >  (float)M_PI) cog_variation -= 2.0f * (float)M_PI;
                if (cog_variation < -(float)M_PI) cog_variation += 2.0f * (float)M_PI;
                prev_cog_rad = current_cog;
                cog_ref_east = east_m;
                cog_ref_north = north_m;
              }
            }
          }
        }
      }

      // ── Mount-yaw observation sentinel (v1.8.3, diagnostic only) ───────
      // Detects straight accel/brake windows and injects MOUNT_YAW_OBS records
      // into the SD stream. MOUNT_YAW_OBS stays diagnostic. MOUNT_YAW_BOOT is
      // now a diagnostic boot recommendation only. Applying it live would
      // rotate the IMU input frame while ESKF state/covariance still belong to
      // the old frame, so the actual correction must be a pre-pipeline boot or
      // offline rewrite step until a full nav reinit path exists.
      if (gps_fix_fresh && gps_origin_set) {
        if (last_gps.epoch != mount_yaw.last_speed_epoch) {
          const float speed_ms = gps_speed_kmh_used / 3.6f;
          float acc_g = 0.0f;
          uint32_t gps_acc_lag_us = 0;
          if (mount_yaw_speed_baseline_accel(mount_yaw,
                                             last_gps.fix_us,
                                             speed_ms,
                                             acc_g,
                                             gps_acc_lag_us)) {
            mount_yaw.gps_acc_ema_g = 0.25f * acc_g + 0.75f * mount_yaw.gps_acc_ema_g;
            mount_yaw.gps_acc_lag_us = gps_acc_lag_us;
          }
          update_mount_yaw_speed_history(mount_yaw, last_gps.fix_us, speed_ms);
          mount_yaw.last_speed_ms = speed_ms;
          mount_yaw.last_speed_fix_us = last_gps.fix_us;
          mount_yaw.last_speed_epoch = last_gps.epoch;
          mount_yaw.has_speed = true;

          float east_m = 0.0f, north_m = 0.0f;
          wgs84_to_enu(last_gps.lat, last_gps.lon,
                       gps_origin_lat, gps_origin_lon,
                       east_m, north_m);
          if (mount_yaw.has_cog && last_gps.fix_us > mount_yaw.last_cog_fix_us) {
            const float de = east_m - mount_yaw.last_east_m;
            const float dn = north_m - mount_yaw.last_north_m;
            const float dist = sqrtf(de * de + dn * dn);
            const float dt_cog = (float)(last_gps.fix_us - mount_yaw.last_cog_fix_us) / 1000000.0f;
            if (dist > 1.5f && dt_cog > 0.05f && dt_cog < 1.0f) {
              const float cog = atan2f(de, dn);
              const float cog_rate_dps = wrap_pi(cog - mount_yaw.last_cog_rad) / dt_cog / DEG2RAD;
              mount_yaw.cog_rate_ema_dps = 0.35f * cog_rate_dps + 0.65f * mount_yaw.cog_rate_ema_dps;
              mount_yaw.last_cog_rad = cog;
            }
          } else {
            mount_yaw.last_cog_rad = 0.0f;
            mount_yaw.has_cog = true;
          }
          mount_yaw.last_east_m = east_m;
          mount_yaw.last_north_m = north_m;
          mount_yaw.last_cog_fix_us = last_gps.fix_us;
          mount_yaw.last_cog_epoch = last_gps.epoch;
        }

        const float gz_corr_dps = gz_r - thermal_bias_gz;
        const float speed_ms = gps_speed_kmh_used / 3.6f;
        const float lat_expected_g = speed_ms * (gz_corr_dps * DEG2RAD) / G_ACCEL;
        const bool straight_candidate =
            data.timestamp_us >= mount_yaw.cooldown_until_us &&
            gps_speed_kmh_used >= MOUNT_YAW_MIN_SPEED_KMH &&
            last_gps.hdop <= MOUNT_YAW_MAX_HDOP &&
            last_gps.sats >= MOUNT_YAW_MIN_SATS &&
            fabsf(gz_corr_dps) <= MOUNT_YAW_MAX_ABS_GZ_DPS &&
            fabsf(mount_yaw.cog_rate_ema_dps) <= MOUNT_YAW_MAX_ABS_COG_RATE_DPS &&
            fabsf(lat_expected_g) <= MOUNT_YAW_MAX_LAT_EXPECTED_G &&
#ifdef USE_BMI270
            !data.fifo_overrun &&
            data.imu_sample_fresh &&
#endif
            true;

        if (straight_candidate) {
          if (!mount_yaw.win.active) {
            start_mount_yaw_window(mount_yaw.win, data.timestamp_us);
          }
          float obs_lin_ax = lin_ax;
          float obs_lin_ay = lin_ay;
          float obs_lin_az = lin_az;
          if (mount_yaw.gps_acc_lag_us > 0 &&
              data.timestamp_us > (uint64_t)mount_yaw.gps_acc_lag_us) {
            mount_yaw_delayed_imu(mount_yaw,
                                  data.timestamp_us - (uint64_t)mount_yaw.gps_acc_lag_us,
                                  obs_lin_ax,
                                  obs_lin_ay,
                                  obs_lin_az);
          }
          accumulate_mount_yaw_window(mount_yaw.win,
                                      data.timestamp_us,
                                      obs_lin_ax,
                                      obs_lin_ay,
                                      obs_lin_az,
                                      mount_yaw.gps_acc_ema_g,
                                      gps_speed_kmh_used,
                                      gz_corr_dps,
                                      mount_yaw.cog_rate_ema_dps,
                                      lat_expected_g);
          const float win_duration_s =
              (float)(data.timestamp_us - mount_yaw.win.start_us) / 1000000.0f;
          if (win_duration_s >= MOUNT_YAW_MAX_DURATION_S) {
            finalize_mount_yaw_window(mount_yaw,
                                      data.timestamp_us
#ifdef USE_BMI270
                                      , record_seq
#endif
            );
          }
        } else if (mount_yaw.win.active) {
          finalize_mount_yaw_window(mount_yaw,
                                    data.timestamp_us
#ifdef USE_BMI270
                                    , record_seq
#endif
          );
        }
      } else if (mount_yaw.win.active) {
        finalize_mount_yaw_window(mount_yaw,
                                  data.timestamp_us
#ifdef USE_BMI270
                                  , record_seq
#endif
        );
      }

      // ── PHASE 3: End-of-pipe EMA ───────────────────────────────────────────
      // Computed AFTER all ZARU updates so that:
      //   (a) thermal_bias_* reflects the bias learned in THIS cycle.
      //   (b) Gyro EMA input is ZARU-corrected (gz_r - tb_gz), centering the
      //       EMA output on zero. The EMA state (prev_gz) then tracks the
      //       corrected signal. When the bias changes (e.g., at a stop), the
      //       EMA smoothly transitions without carrying the old bias in its
      //       internal state — masking the bias step-change in logged data.
      //   (c) No separate "- tb_*" subtraction is needed at output.
      //
      // tb_* snapshot taken HERE (after ZARU) so the logged tbias_gz field
      // reflects the bias actually used to correct the EMA in this sample
      // (improvement over v1.3.4 which snapshotted before ZARU).
      const float tb_gx = thermal_bias_gx;
      const float tb_gy = thermal_bias_gy;
      const float tb_gz = thermal_bias_gz;

      float ema_ax = alpha * lin_ax          + (1.0f - alpha) * local_prev_ax;
      float ema_ay = alpha * lin_ay          + (1.0f - alpha) * local_prev_ay;
      float ema_az = alpha * lin_az          + (1.0f - alpha) * local_prev_az;
      float ema_gx = alpha * (gx_r - tb_gx) + (1.0f - alpha) * local_prev_gx;
      float ema_gy = alpha * (gy_r - tb_gy) + (1.0f - alpha) * local_prev_gy;
      float ema_gz = alpha * (gz_r - tb_gz) + (1.0f - alpha) * local_prev_gz;

      // Capture before EMA seeding clears it (Phase 5 needs it for zaru_flags bit 3)
      const bool recalib_marker = first_sample_after_recalib;

      // v1.3.2: seed EMA from first raw sample after recalibration to prevent
      // 313 ms decay transient. After recalib, thermal_bias_* = 0.0f so
      // (gx_r - tb_gx) = gx_r — same seeding value as before.
      if (first_sample_after_recalib) {
        ema_ax = lin_ax;       ema_ay = lin_ay;       ema_az = lin_az;
        ema_gx = gx_r - tb_gx; ema_gy = gy_r - tb_gy; ema_gz = gz_r - tb_gz;
        first_sample_after_recalib = false;
      }

      // ── PHASE 4: Merged telemetry_mutex — EMA state + shared_telemetry + ESKF
      // Merges the former "second mutex" (ESKF output, old lines 459-465) with
      // the EMA state write-back and shared_telemetry update.
      //
      // AUDIT-NOTE (PIPELINE-MINOR-3, updated v1.3.5): 2 ms timeout means if
      // loop() holds the mutex (e.g. during MQTT reconnect), both EMA state
      // and ESKF output are not written to shared_telemetry for that cycle.
      // Display and MQTT show stale data for one 20 ms frame. Self-heals next
      // cycle. Also: if timeout fires, prev_* globals are not updated — next
      // cycle computes EMA from prev_* that are 40 ms old instead of 20 ms.
      // Benign at 50 Hz.
      if (xSemaphoreTake(telemetry_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        // Write-back EMA state to globals (feeds next cycle and calibrate_alignment)
        prev_ax = ema_ax; prev_ay = ema_ay; prev_az = ema_az;
        prev_gx = ema_gx; prev_gy = ema_gy; prev_gz = ema_gz;

        // EMA values are already ZARU-corrected — no "- tb_*" needed at output
        shared_telemetry.ema_ax = ema_ax;
        shared_telemetry.ema_ay = ema_ay;
        shared_telemetry.ema_az = ema_az;
        shared_telemetry.ema_gx = ema_gx;
        shared_telemetry.ema_gy = ema_gy;
        shared_telemetry.ema_gz = ema_gz;

        // ESKF output (merged from former second mutex)
        shared_telemetry.kf_x = eskf.px();
        shared_telemetry.kf_y = eskf.py();
        shared_telemetry.kf_vel = eskf.speed_ms();
        shared_telemetry.kf_heading = eskf.heading();

        xSemaphoreGive(telemetry_mutex);
      }

      // ── PHASE 5: SD record ─────────────────────────────────────────────────
      // Outside the mutex to avoid extending the lock window.
      // data.temp_c already available from the IMU queue: no extra I2C call.
      // GPS: snapshot already acquired above for the ESKF.
      if (sd_queue != NULL) {
        TelemetryRecord rec = {};
        rec.timestamp_us = data.timestamp_us;
        // EMA accel/gyro: already ZARU-corrected (no "- tb_*" subtraction needed)
        rec.ax = ema_ax;
        rec.ay = ema_ay;
        rec.az = ema_az;
        rec.gx = ema_gx;
        rec.gy = ema_gy;
        rec.gz = ema_gz;
        rec.temp_c = data.temp_c;
        rec.lap = lap_snap;
        // GPS: use the static cache last_gps (already updated above)
        rec.gps_lat = last_gps.lat;
        rec.gps_lon = last_gps.lon;
        rec.gps_sog_kmh = last_gps.sog_kmh;
        rec.gps_alt_m = last_gps.alt_m;
        rec.gps_sats = last_gps.sats;
        rec.gps_hdop = last_gps.hdop;
        // ESKF 5D output
        rec.kf_x = eskf.px();
        rec.kf_y = eskf.py();
        rec.kf_vel = eskf.speed_ms();
        rec.kf_heading = eskf.heading();
        // Zero-latency pipeline channels: gravity-removed linear accel and
        // vehicle-frame gyro before EMA.
        rec.pipe_lin_ax = lin_ax; rec.pipe_lin_ay = lin_ay; rec.pipe_lin_az = lin_az;
        rec.pipe_body_gx = gx_r;  rec.pipe_body_gy = gy_r;  rec.pipe_body_gz = gz_r;
        // ESKF 6D Shadow (v0.9.8)
        rec.kf6_x = eskf6.px();
        rec.kf6_y = eskf6.py();
        rec.kf6_vel = eskf6.speed_ms();
        rec.kf6_heading = eskf6.heading();
        rec.kf6_bgz = eskf6.bias_gz();
        // ZARU/NHC diagnostic flags (v1.3.1)
        uint8_t flags = 0;
        if (is_stationary)       flags |= 0x01; // bit 0: Static ZARU
        if (straight_zaru_active) flags |= 0x02; // bit 1: Straight ZARU
        if (nhc_active)          flags |= 0x04; // bit 2: NHC
        if (recalib_marker)      flags |= 0x08; // bit 3: Recalibration
        rec.zaru_flags = flags;
        rec.tbias_gz = tb_gz; // post-ZARU snapshot (consistent with EMA correction)
#ifdef USE_BMI270
        rec.bmi_post_lpf20_prepipe_ax = data.bmi_post_lpf20_prepipe_ax;
        rec.bmi_post_lpf20_prepipe_ay = data.bmi_post_lpf20_prepipe_ay;
        rec.bmi_post_lpf20_prepipe_az = data.bmi_post_lpf20_prepipe_az;
        rec.bmi_post_lpf20_prepipe_gx = data.bmi_post_lpf20_prepipe_gx;
        rec.bmi_post_lpf20_prepipe_gy = data.bmi_post_lpf20_prepipe_gy;
        rec.bmi_post_lpf20_prepipe_gz = data.bmi_post_lpf20_prepipe_gz;
        rec.bmi_acc_x_g = data.bmi_acc_x_g;
        rec.bmi_acc_y_g = data.bmi_acc_y_g;
        rec.bmi_acc_z_g = data.bmi_acc_z_g;
        rec.bmi_gyr_x_dps = data.bmi_gyr_x_dps;
        rec.bmi_gyr_y_dps = data.bmi_gyr_y_dps;
        rec.bmi_gyr_z_dps = data.bmi_gyr_z_dps;
        rec.bmm_raw_x = data.bmm_raw_x;
        rec.bmm_raw_y = data.bmm_raw_y;
        rec.bmm_raw_z = data.bmm_raw_z;
        rec.bmm_rhall = data.bmm_rhall;
        rec.bmm_ut_x = data.bmm_ut_x;
        rec.bmm_ut_y = data.bmm_ut_y;
        rec.bmm_ut_z = data.bmm_ut_z;
        rec.mag_valid = data.mag_valid ? 1 : 0;
        rec.mag_sample_fresh = data.mag_sample_fresh ? 1 : 0;
        rec.mag_overflow = data.mag_overflow ? 1 : 0;
        rec.imu_sample_fresh = data.imu_sample_fresh ? 1 : 0;
        rec.fifo_frames_drained = data.fifo_frames_drained;
        rec.fifo_backlog = data.fifo_backlog;
        rec.fifo_overrun = data.fifo_overrun ? 1 : 0;
        rec.sd_queue_hwm = 0; // filled with a fresh snapshot in Task_SD_Writer
#else
        // Legacy AtomS3 provider-frame channels retained for 202-byte format.
        rec.sensor_ax = data.bmi_acc_x_g;
        rec.sensor_ay = data.bmi_acc_y_g;
        rec.sensor_az = data.bmi_acc_z_g;
        rec.sensor_gx = data.bmi_gyr_x_dps;
        rec.sensor_gy = data.bmi_gyr_y_dps;
        rec.sensor_gz = data.bmi_gyr_z_dps;
#endif
        // GPS timing metadata (SITL, v1.4.2): enables deterministic offline replay of
        // staleness detection and eskf.correct() gating. Since v1.8.7 gps_valid
        // means "fresh enough for fusion", not merely TinyGPSPlus ever-valid.
        // fix_us uses same timebase as timestamp_us → (timestamp_us - gps_fix_us) > 5s.
        rec.gps_fix_us = last_gps.fix_us;
        rec.gps_valid  = gps_fix_fresh ? 1 : 0;
        // Extra GPS velocity metadata for SITL/offline analysis.
        // NAV2-PVH is the primary scalar speed when fresh; DHV remains diagnostic-only.
        rec.nav_speed2d     = last_gps.nav_speed2d;
        rec.nav_s_acc       = last_gps.nav_s_acc;
        rec.nav_vel_n       = last_gps.nav_vel_n;
        rec.nav_vel_e       = last_gps.nav_vel_e;
        rec.nav_vel_valid   = last_gps.nav_vel_valid;
        rec.gps_speed_source = gps_speed_source;
        rec.nav_fix_us      = last_gps.nav_fix_us;
        rec.dhv_gdspd       = last_gps.dhv_gdspd;
        rec.dhv_fix_us      = last_gps.dhv_fix_us;
#ifdef USE_BMI270
        rec.seq = record_seq++;
#endif
        if (xQueueSend(sd_queue, &rec, 0) != pdTRUE) {
          sd_enqueue_fail_count++;
          sd_records_dropped++;
        }
      }
    }
  }
}
