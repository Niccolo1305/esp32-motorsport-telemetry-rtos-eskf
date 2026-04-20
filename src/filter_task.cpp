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
//    permanently blocking the Straight-Line ZARU lateral gate (|lin_ay|<0.05g).
//    Raw values are preserved as gz_rad_raw for ESKF 6D shadow comparison.
//
// 3. Straight-Line ZARU zero-latency gate: gz_gate uses raw corrected
//    (gz_r - thermal_bias_gz) instead of EMA, closing instantly on corner entry.
#include "filter_task.h"

#include "globals.h"
#include "imu_axis_remap.h"
#include "math_utils.h"

void Task_Filter(void *pvParameters) {
  ImuRawData data;
  uint32_t last_eskf_epoch = 0; // last GPS epoch processed by the ESKF
  uint64_t last_timestamp_us = 0;

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

  for (;;) {
    if (xQueueReceive(imuQueue, &data, portMAX_DELAY) == pdTRUE) {

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

      if (xSemaphoreTake(telemetry_mutex, portMAX_DELAY) == pdTRUE) {

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
        // permanently blocks the Straight-Line ZARU gate (|lin_ay|<0.05g).
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
      }

      // ── GPS Velocity Source Selection (v1.5.2) ────────────────────────────
      // Hardware investigation result (AT6668 module):
      //   - DHV (NMEA): capped at 1 Hz by AT6668 firmware regardless of PCAS03.
      //   - NAV-PV (binary): CFG-MSG (0x06/0x01) NACKed even after CFG-PRT
      //     enables binary output. Module does not support periodic NAV-PV.
      //   - NMEA SOG (RMC, via TinyGPSPlus): 10 Hz, co-epoch with position fix.
      //     This is the receiver-reported Speed Over Ground exposed by NMEA.
      // Primary source: sog_kmh. DHV and NAV-PV are logged for diagnostics only.
      float gps_speed_kmh_used = last_gps.sog_kmh;
      uint8_t gps_speed_source = GPS_SPD_NMEA_SOG;

      // ── Stationary Condition (Statistical Engine + GPS) ───────────────────
      // is_stationary = true ONLY IF:
      //   1. Buffer warm-up is complete (50 samples collected)
      //   2. gz, gx AND gy raw variance are all below their respective noise floors
      //      (3-axis gate: prevents false positives from chassis vibration coupling
      //      into roll/pitch even when gz happens to be quiet)
      //   3. GPS speed (if available) is < 2 km/h (uses NMEA SOG in v1.5.2)
      //   4. |mean_gz| < 2.5°/s (Sanity Gate v0.9.9)
      bool is_stationary = false;
      if (var_gz >= 0.0f) { // warm-up complete
        bool gps_slow = !last_gps.valid ||
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
      // BUG-3 fixes preserved: speed 40 km/h, lin_ay (not ema_ay).
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
      if (!is_stationary && last_gps.valid &&
          gps_speed_kmh_used > STRAIGHT_MIN_SPEED_KMH) {
        bool lat_gate = fabsf(lin_ay) < STRAIGHT_MAX_LAT_G;
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
      // extreme lateral dynamics (|lin_ay| > 0.5 g).
      bool nhc_active = false;
      if (eskf.speed_ms() > NHC_MIN_SPEED_MS &&
          fabsf(lin_ay) < NHC_MAX_LAT_G) {
        eskf.correct_nhc(NHC_R);
        eskf6.correct_nhc(NHC_R);
        nhc_active = true;
      }

      // ── GPS Staleness Detection (v0.9.11) ─────────────────────────────────
      // If the last valid GPS fix is older than 5 s, the GPS is considered
      // lost. The ESKF continues in predict-only (IMU dead-reckoning) and
      // correct is disabled to avoid anchoring position to a fossil fix.
      // The gps_stale flag triggers the visual alarm in loop() (flashing red).
      bool gps_is_stale = last_gps.valid &&
                          (data.timestamp_us - last_gps.fix_us > 5000000ULL);
      // Pre-fix: GPS is not yet "stale" — it simply has never fixed
      if (!last_gps.valid) gps_is_stale = false;
      gps_stale = gps_is_stale;

      // Correct: only if there is a new GPS fix AND it is not stale
      if (last_gps.valid && last_gps.epoch != last_eskf_epoch && !gps_is_stale) {
        // First valid coordinate → set the ENU origin
        if (!gps_origin_set) {
          gps_origin_lat = last_gps.lat;
          gps_origin_lon = last_gps.lon;
          gps_origin_set = true;
          eskf.reset();  // clean state centred on origin
          eskf6.reset();
          Serial.printf("[ESKF] ENU origin: %.6f, %.6f\n",
                        gps_origin_lat, gps_origin_lon);
        }
        float east_m, north_m;
        wgs84_to_enu(last_gps.lat, last_gps.lon,
                     gps_origin_lat, gps_origin_lon,
                     east_m, north_m);
        eskf.correct(east_m, north_m, gps_speed_kmh_used, last_gps.hdop);
        eskf6.correct(east_m, north_m, gps_speed_kmh_used, last_gps.hdop);
        last_eskf_epoch = last_gps.epoch;

        // ── COG variation for Enhanced Straight-Line ZARU (v1.3.1) ──────────
        // Long-baseline approach: COG computed only after >15 m displacement
        // from the reference position. Suppresses GPS position noise
        // (σ_COG ≈ σ_pos/baseline ≈ 1.5m/15m ≈ 0.1 rad).
        if (gps_speed_kmh_used > 20.0f) {
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
        rec.bmi_raw_ax = data.bmi_raw_ax;
        rec.bmi_raw_ay = data.bmi_raw_ay;
        rec.bmi_raw_az = data.bmi_raw_az;
        rec.bmi_raw_gx = data.bmi_raw_gx;
        rec.bmi_raw_gy = data.bmi_raw_gy;
        rec.bmi_raw_gz = data.bmi_raw_gz;
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
        rec.reserved0 = 0;
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
        // staleness detection and eskf.correct() gating.
        // fix_us uses same timebase as timestamp_us → (timestamp_us - gps_fix_us) > 5s.
        rec.gps_fix_us = last_gps.fix_us;
        rec.gps_valid  = last_gps.valid ? 1 : 0;
        // Extra GPS velocity metadata for SITL/offline analysis.
        // NAV-PV remains a passive listener in v1.5.1; DHV is the active
        // receiver-reported horizontal ground-speed path under test.
        rec.nav_speed2d     = last_gps.nav_speed2d;
        rec.nav_s_acc       = last_gps.nav_s_acc;
        rec.nav_vel_n       = last_gps.nav_vel_n;
        rec.nav_vel_e       = last_gps.nav_vel_e;
        rec.nav_vel_valid   = last_gps.nav_vel_valid;
        rec.gps_speed_source = gps_speed_source;
        rec.nav_fix_us      = last_gps.nav_fix_us;
        rec.dhv_gdspd       = last_gps.dhv_gdspd;
        rec.dhv_fix_us      = last_gps.dhv_fix_us;
        if (xQueueSend(sd_queue, &rec, 0) != pdTRUE) {
          sd_records_dropped++;
        }
      }
    }
  }
}
