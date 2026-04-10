// SPDX-License-Identifier: GPL-3.0-or-later
// filter_task.cpp — FreeRTOS Task_Filter (Core 1, priority 2)
//
// Math pipeline: calibration → alignment → Madgwick →
// gravity removal → EMA → Statistical Engine (ema_gz variance) →
// Adaptive ZARU (thermal bias) → ESKF (raw data + ZUPT) → SD.
#include "filter_task.h"

#include <M5Unified.h>

#include "globals.h"
#include "math_utils.h"

void Task_Filter(void *pvParameters) {
  ImuRawData data;
  uint32_t last_eskf_epoch = 0; // last GPS epoch processed by the ESKF
  uint64_t last_timestamp_us = 0;

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

      if (xSemaphoreTake(telemetry_mutex, portMAX_DELAY) == pdTRUE) {

        // STEP 1: Ellipsoidal calibration (chip native frame)
        float ax_cal = data.ax, ay_cal = data.ay, az_cal = data.az;
        apply_ellipsoidal_calibration(ax_cal, ay_cal, az_cal);

        // STEP 2: Geometric alignment (vehicle frame)
        float ax_r_raw, ay_r_raw, az_r_raw;
        rotate_3d(ax_cal, ay_cal, az_cal, ax_r_raw, ay_r_raw, az_r_raw, cos_phi,
                 sin_phi, cos_theta, sin_theta);

        // STEP 3: Mounting bias (post-ellipsoid residuals)
        float ax_r = ax_r_raw - bias_ax;
        float ay_r = ay_r_raw - bias_ay;
        float az_r = az_r_raw - bias_az;

        // STEP 4: Gyroscope — electronic bias + G-sensitivity correction + vehicle-frame rotation
        float gx_clean = data.gx - bias_gx;
        float gy_clean = data.gy - bias_gy;
        float gz_clean = data.gz - bias_gz;

        // K_gs correction (chip frame, before rotate_3d).
        // K_gs[(deg/s)/g] measured by 6-face static test (tel_gsen4 + tel_82).
        // bias_gz absorbed K_gz * a_boot at boot → subtract only the incremental
        // change: K_gs * (a_cal - a_boot).  Orientation-independent at boot.
        //
        //            az(j=0)    ax(j=1)    ay(j=2)
        //  gx(i=0): +0.6056    +0.7969    -0.1346
        //  gy(i=1): +0.0712    +0.3539    -0.0908
        //  gz(i=2): -0.1547    -0.0372    -0.0003  <-- ESKF heading
        {
          const float daz = az_cal - az_boot_cal;
          const float dax = ax_cal - ax_boot_cal;
          const float day = ay_cal - ay_boot_cal;
          gx_clean -= (+0.6056f) * daz + (+0.7969f) * dax + (-0.1346f) * day;
          gy_clean -= (+0.0712f) * daz + (+0.3539f) * dax + (-0.0908f) * day;
          gz_clean -= (-0.1547f) * daz + (-0.0372f) * dax + (-0.0003f) * day;
        }

        float gx_r, gy_r, gz_r;
        rotate_3d(gx_clean, gy_clean, gz_clean, gx_r, gy_r, gz_r, cos_phi,
                 sin_phi, cos_theta, sin_theta);

        float gx_rad = gx_r * DEG2RAD;
        float gy_rad = gy_r * DEG2RAD;
        float gz_rad = gz_r * DEG2RAD;

        // STEP 5: Madgwick AHRS with real DT
        ahrs.sampleperiod = dt_real_sec;
        ahrs.update_imu(gx_rad, gy_rad, gz_rad, ax_r, ay_r, az_r);

        // STEP 6: Gravity removal
        float grav_x, grav_y, grav_z;
        ahrs.get_gravity_vector(grav_x, grav_y, grav_z);
        float lin_ax = ax_r - grav_x;
        float lin_ay = ay_r - grav_y;
        float lin_az = az_r - grav_z;

        // STEP 7: EMA (α=0.06 production: τ ≈ 313 ms at 50 Hz)
        float ema_ax = alpha * lin_ax + (1.0f - alpha) * prev_ax;
        float ema_ay = alpha * lin_ay + (1.0f - alpha) * prev_ay;
        float ema_az = alpha * lin_az + (1.0f - alpha) * prev_az;
        float ema_gx = alpha * gx_r + (1.0f - alpha) * prev_gx;
        float ema_gy = alpha * gy_r + (1.0f - alpha) * prev_gy;
        float ema_gz = alpha * gz_r + (1.0f - alpha) * prev_gz;
        prev_ax = ema_ax;
        prev_ay = ema_ay;
        prev_az = ema_az;
        prev_gx = ema_gx;
        prev_gy = ema_gy;
        prev_gz = ema_gz;

        // ── STEP 8: Statistical ZUPT/ZARU Engine — 3-axis (v1.2.1) ────
        // Circular buffer of 50 samples (1 s at 50 Hz) of EMA-filtered gyro.
        // gz variance determines is_stationary (together with GPS speed).
        // gx/gy share the same index and warm-up counter: no extra overhead.
        // NOTE: buffers use pre-correction EMA values to avoid feedback loop —
        // thermal_bias is subtracted only at the output (log/display), not here.
        // Raw gz_rad goes to the ESKF via the Data Fork (unchanged).
        //
        // Warm-up: variance is not computed until gz_var_count >= VAR_BUF_SIZE
        // → prevents false positives from a partially filled buffer.
        static float gz_var_buf[VAR_BUF_SIZE];
        static float gx_var_buf[VAR_BUF_SIZE]; // v1.2.1: ZARU extended to gx
        static float gy_var_buf[VAR_BUF_SIZE]; // v1.2.1: ZARU extended to gy
        static int gz_var_idx = 0;
        static int gz_var_count = 0;
        static float thermal_bias_gz = 0.0f; // [°/s] learned dynamic thermal offset
        static float thermal_bias_gx = 0.0f; // [°/s] v1.2.1
        static float thermal_bias_gy = 0.0f; // [°/s] v1.2.1

        gz_var_buf[gz_var_idx] = ema_gz;
        gx_var_buf[gz_var_idx] = ema_gx; // same index and warm-up as gz
        gy_var_buf[gz_var_idx] = ema_gy;
        gz_var_idx = (gz_var_idx + 1) % VAR_BUF_SIZE;
        if (gz_var_count < VAR_BUF_SIZE) gz_var_count++;

        // Variance and mean computed only after complete warm-up.
        // var_gz and mean_gz used to determine is_stationary.
        // mean_gx/gy used to update thermal_bias_gx/gy when stationary.
        float var_gz = -1.0f;  // sentinel: <0 = warm-up incomplete
        float mean_gz = 0.0f;
        float mean_gx = 0.0f;
        float mean_gy = 0.0f;
        if (gz_var_count >= VAR_BUF_SIZE) {
          float sum = 0.0f, sum_sq = 0.0f;
          float sum_gx = 0.0f, sum_gy = 0.0f;
          for (int i = 0; i < VAR_BUF_SIZE; i++) {
            sum    += gz_var_buf[i];
            sum_sq += gz_var_buf[i] * gz_var_buf[i];
            sum_gx += gx_var_buf[i];
            sum_gy += gy_var_buf[i];
          }
          mean_gz = sum / (float)VAR_BUF_SIZE;
          mean_gx = sum_gx / (float)VAR_BUF_SIZE;
          mean_gy = sum_gy / (float)VAR_BUF_SIZE;
          // fmaxf: prevents numerically negative variance from floating-point
          // cancellation when samples are nearly identical (vehicle stationary).
          var_gz = fmaxf(0.0f,
                         (sum_sq / (float)VAR_BUF_SIZE) - (mean_gz * mean_gz));
        }
        // ──────────────────────────────────────────────────────────────────

        // v1.2.2: snapshot ZARU bias before output to guarantee shared_telemetry
        // and SD record use the same correction value. Without this, the ZARU
        // update (outside mutex) would make the SD record see a newer bias than
        // shared_telemetry within the same cycle (1-sample / 20 ms discrepancy).
        const float tb_gx = thermal_bias_gx;
        const float tb_gy = thermal_bias_gy;
        const float tb_gz = thermal_bias_gz;

        shared_telemetry.ema_ax = ema_ax;
        shared_telemetry.ema_ay = ema_ay;
        shared_telemetry.ema_az = ema_az;
        // v1.2.1: ZARU correction applied at output (buffer read pre-correction)
        shared_telemetry.ema_gx = ema_gx - tb_gx;
        shared_telemetry.ema_gy = ema_gy - tb_gy;
        shared_telemetry.ema_gz = ema_gz - tb_gz;

        // lap_snap captured inside the lock: consistent with the EMA data just computed.
        uint8_t lap_snap = (system_state == 2) ? 1 : 0;

        xSemaphoreGive(telemetry_mutex);

        // ── STEP 9: ESKF 2D — Predict + Correct ─────────────────────────
        // Outside telemetry_mutex: the ESKF is single-thread (Task_Filter)
        // and shares no state with loop().

        // GPS snapshot for SD and ESKF
        static GpsData last_gps;
        if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          last_gps = shared_gps_data;
          xSemaphoreGive(gps_mutex);
        }

        // ── DATA FORK ──────────────────────────────────────────────────
        // The ESKF is fed RAW post-Madgwick IMU data:
        //   - lin_ax, lin_ay: linear acceleration [g] (gravity removed via
        //     Madgwick quaternion) → zero phase latency.
        //   - gz_rad: yaw angular rate [rad/s] (raw gyro in vehicle frame)
        //     corrected by the learned thermal_bias_gz.
        //
        // EMA data (ema_ax, ema_ay, ema_gz) do NOT enter the ESKF.
        // The EMA filter is used EXCLUSIVELY for:
        //   - LCD display (5 Hz)
        //   - MQTT JSON payload (10 Hz)
        //   - Binary SD logging (50 Hz)
        //   - Statistical Engine input (ema_gz variance for ZUPT/ZARU)
        //
        // Rationale: EMA with α=0.06 introduces a phase delay
        // τ = (1-α)/(α·f) ≈ 313 ms which, if fed into the ESKF, would
        // degrade the Kalman filter's dynamic response.
        // ─────────────────────────────────────────────────────────────────

        // ── Stationary Condition (Statistical Engine + GPS) ───────────
        // is_stationary = true ONLY IF:
        //   1. Buffer warm-up is complete (50 samples collected)
        //   2. ema_gz variance is below the static noise floor
        //   3. GPS speed (if available) is < 2 km/h
        //   4. |mean_gz| < 2.5°/s (Sanity Gate v0.9.9)
        bool is_stationary = false;
        if (var_gz >= 0.0f) { // warm-up complete
          bool gps_slow = !last_gps.valid ||
                          (last_gps.speed_kmh < ZUPT_GPS_MAX_KMH);
          // Sanity Gate: mean_gz < 2.5°/s excludes slow rotational manoeuvres.
          // Real thermal bias does not exceed ~1.5°/s
          // (30°C swing × 0.032°/s/°C ≈ 1°/s for MPU-6886).
          // A 1.6 km/h manoeuvre produces 8-10°/s → blocked.
          // Threshold 2.5 leaves margin for extreme bias without deadlock.
          is_stationary = (var_gz < VAR_STILLNESS_THRESHOLD) && gps_slow
                          && (fabsf(mean_gz) < 2.5f);
        }

        // ── Adaptive ZARU (Thermal Bias Learning) ──────────────────────
        // When the vehicle is stationary, the ema_gz buffer mean represents
        // the residual thermal drift of gyro Z. This value is saved as
        // thermal_bias_gz and subtracted from gz_rad before feeding the ESKF.
        // In motion, the bias learned at standstill keeps compensating the
        // drift without introducing latency.
        if (is_stationary) {
          thermal_bias_gz = mean_gz; // [°/s], instant learning
          thermal_bias_gx = mean_gx; // v1.2.1
          thermal_bias_gy = mean_gy; // v1.2.1
        }

        // ── Straight-line ZARU (v0.9.7, BUG-3 fix v0.9.13) ─────────
        // On a fast straight the true rotation is ~0: the ema_gz residual
        // is the current thermal bias. 1 % EMA to filter micro steering
        // corrections and disturbances. Convergence ~5 s at α=0.01, 50 Hz.
        //
        // BUG-3 fixes:
        //   - Speed threshold raised 20→40 km/h: at 20 km/h (5.5 m/s),
        //     ω=2°/s corresponds to R=157 m (a real curve, not a straight).
        //     At 40 km/h (11.1 m/s), R=318 m — a genuine highway straight.
        //   - Lateral gate uses lin_ay (raw post-Madgwick) instead of ema_ay:
        //     EMA τ≈313 ms smooths a rapid lane change (100 ms) below the
        //     0.02 g threshold, making the gate too permeable. Raw lin_ay
        //     captures the instantaneous lateral load without phase delay.
        if (!is_stationary && last_gps.valid && last_gps.speed_kmh > 40.0f) {
          if (fabsf(lin_ay) < 0.02f && fabsf(ema_gz) < 2.0f) {
            thermal_bias_gz = 0.01f * ema_gz + 0.99f * thermal_bias_gz;
            thermal_bias_gx = 0.01f * ema_gx + 0.99f * thermal_bias_gx; // v1.2.1: roll≈0 on straight
            thermal_bias_gy = 0.01f * ema_gy + 0.99f * thermal_bias_gy; // v1.2.1: pitch≈0 on straight
          }
        }

        // ── ESKF 5D vs 6D: different bias strategies ────────────────
        // 5D: bias is removed externally by ZARU before entering the filter.
        //     The 5D state has no bias term: X=[px,py,vx,vy,θ].
        // 6D: bias is estimated internally as X[5]=b_gz (random walk model).
        //     It receives raw gz_rad so it can learn the bias independently.
        //     Feeding gz_eskf (already ZARU-corrected) would cause double
        //     subtraction and collapse the bias state to ≈0, defeating the
        //     purpose of the shadow comparison.
        // Having two independent bias estimates (ZARU statistical vs Kalman
        // model-based) enables offline A/B validation via kf6_bgz in the CSV.
        float gz_eskf = gz_rad - (thermal_bias_gz * DEG2RAD); // [rad/s]
        eskf.predict(lin_ax, lin_ay, gz_eskf, dt_real_sec, is_stationary);

        eskf6.predict(lin_ax, lin_ay, gz_rad, dt_real_sec, is_stationary);

        // 6D ZUPT bias: when stationary, measured gz ≈ pure bias → observation
        if (is_stationary) {
          eskf6.correct_bias(gz_rad, 0.001f);
        }

        // ── GPS Staleness Detection (v0.9.11) ────────────────────────
        // If the last valid GPS fix is older than 5 s, the GPS is considered
        // lost. The ESKF continues in predict-only (IMU dead-reckoning) and
        // correct is disabled to avoid anchoring position to a fossil fix.
        // The gps_stale flag triggers the visual alarm in loop() (flashing red).
        bool gps_is_stale = last_gps.valid &&
                            (millis() - last_gps.fix_ms > 5000);
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
          eskf.correct(east_m, north_m, last_gps.speed_kmh, last_gps.hdop);
          eskf6.correct(east_m, north_m, last_gps.speed_kmh, last_gps.hdop);
          last_eskf_epoch = last_gps.epoch;
        }

        // ── ESKF → shared_telemetry (race condition fix v0.9.2) ──────
        // loop() read eskf.px()/py()/speed_ms()/heading() directly from the
        // global object without a mutex, while Task_Filter updated it at 50 Hz
        // → potentially incoherent MQTT payload.
        // Fix: atomic copy of the 4 getters under telemetry_mutex.
        if (xSemaphoreTake(telemetry_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          shared_telemetry.kf_x = eskf.px();
          shared_telemetry.kf_y = eskf.py();
          shared_telemetry.kf_vel = eskf.speed_ms();
          shared_telemetry.kf_heading = eskf.heading();
          xSemaphoreGive(telemetry_mutex);
        }

        // SD: outside the mutex to avoid extending the lock window.
        // data.temp_c already available from the IMU queue: no extra I2C call.
        // GPS: snapshot already acquired above for the ESKF.
        if (sd_queue != NULL) {
          TelemetryRecord rec;
          rec.timestamp_ms = (uint32_t)(data.timestamp_us / 1000ULL);
          rec.ax = ema_ax;
          rec.ay = ema_ay;
          rec.az = ema_az;
          // v1.2.1: ZARU correction applied to logged values (same as shared_telemetry)
          // v1.2.2: uses tb_gx/gy/gz snapshot (same bias as shared_telemetry above)
          rec.gx = ema_gx - tb_gx;
          rec.gy = ema_gy - tb_gy;
          rec.gz = ema_gz - tb_gz;
          rec.temp_c = data.temp_c;
          rec.lap = lap_snap;
          // GPS: use the static cache last_gps (already updated above)
          rec.gps_lat = last_gps.lat;
          rec.gps_lon = last_gps.lon;
          rec.gps_speed_kmh = last_gps.speed_kmh;
          rec.gps_alt_m = last_gps.alt_m;
          rec.gps_sats = last_gps.sats;
          rec.gps_hdop = last_gps.hdop;
          // ESKF 5D output
          rec.kf_x = eskf.px();
          rec.kf_y = eskf.py();
          rec.kf_vel = eskf.speed_ms();
          rec.kf_heading = eskf.heading();
          // Raw post-Madgwick IMU: bypasses EMA, zero-latency (feeds ESKF via Data Fork)
          // raw_g* are NOT ZARU-corrected: true uncorrected signal for offline
          // diagnostics and re-processing. Use gx/gy/gz (EMA) for clean values.
          rec.raw_ax = lin_ax; rec.raw_ay = lin_ay; rec.raw_az = lin_az;
          rec.raw_gx = gx_r;   rec.raw_gy = gy_r;   rec.raw_gz = gz_r;
          // ESKF 6D Shadow (v0.9.8)
          rec.kf6_x = eskf6.px();
          rec.kf6_y = eskf6.py();
          rec.kf6_vel = eskf6.speed_ms();
          rec.kf6_heading = eskf6.heading();
          rec.kf6_bgz = eskf6.bias_gz();
          xQueueSend(sd_queue, &rec, 0); // timeout=0: discard if queue full
        }
      }
    }
  }
}
