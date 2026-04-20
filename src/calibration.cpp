// SPDX-License-Identifier: GPL-3.0-or-later
// calibration.cpp — Sensor calibration (geometric alignment at boot)
//
// calibrate_alignment() runs a 2-second static sampling window (100 samples),
// computes mounting angles (phi, theta) and biases, then injects the results
// into the global calibration state under telemetry_mutex.
#include "calibration.h"

#include "IImuProvider.h"
#include "globals.h"
#include "imu_axis_remap.h"
#include "math_utils.h"

void calibrate_alignment(IImuProvider* imu) {
  // Array size and loop tied to the same constant.
  // If FREQ_HZ changes, both resize without stack buffer overflow.
  static constexpr int CALIB_SAMPLES = (int)(50.0f * 2.0f); // 2 s at 50 Hz
  static_assert(CALIB_SAMPLES <= 200, "CALIB_SAMPLES too large for the stack");
  static constexpr int CALIB_MAG_MIN_VALID = CALIB_SAMPLES / 4;

  float samples_ax[CALIB_SAMPLES], samples_ay[CALIB_SAMPLES], samples_az[CALIB_SAMPLES];
  float samples_gx[CALIB_SAMPLES], samples_gy[CALIB_SAMPLES], samples_gz[CALIB_SAMPLES];
  float samples_mag_x[CALIB_SAMPLES], samples_mag_y[CALIB_SAMPLES], samples_mag_z[CALIB_SAMPLES];
  int mag_sample_count = 0;

  // Suspend Task_I2C (Core 0) to avoid I2C bus contention.
  // Calibration is the only other legal consumer of the internal IMU bus.
  if (TaskI2CHandle != NULL) vTaskSuspend(TaskI2CHandle);

  // FIFO/AUX warm-up discard: flush stale backlog accumulated before the static
  // calibration window. On AtomS3 this is a harmless one-sample discard.
  ImuRawData warmup_discard;
  imu->update(warmup_discard);

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    ImuRawData raw;
    imu->update(raw);
    float ax = raw.bmi_acc_x_g, ay = raw.bmi_acc_y_g, az = raw.bmi_acc_z_g;
    float gx = raw.bmi_gyr_x_dps, gy = raw.bmi_gyr_y_dps, gz = raw.bmi_gyr_z_dps;
    remap_chip_axes_to_pipeline(ax, ay, az);
    remap_chip_axes_to_pipeline(gx, gy, gz);
    // Ellipsoidal calibration applied to raw samples before computing
    // mounting angles. Without this correction the AZ = -0.065 g bias
    // would be absorbed into bias_az instead of being removed.
    apply_ellipsoidal_calibration(ax, ay, az);
    samples_ax[i] = ax;
    samples_ay[i] = ay;
    samples_az[i] = az;
    samples_gx[i] = gx;
    samples_gy[i] = gy;
    samples_gz[i] = gz;
    if (raw.mag_valid && raw.mag_sample_fresh && !raw.mag_overflow && mag_sample_count < CALIB_SAMPLES) {
      samples_mag_x[mag_sample_count] = raw.bmm_ut_x;
      samples_mag_y[mag_sample_count] = raw.bmm_ut_y;
      samples_mag_z[mag_sample_count] = raw.bmm_ut_z;
      mag_sample_count++;
    }
    vTaskDelay(pdMS_TO_TICKS(DT_MS));
  }

  // Resume Task_I2C: exclusive sampling is complete.
  if (TaskI2CHandle != NULL) vTaskResume(TaskI2CHandle);

  float ax = trimmed_mean(samples_ax, CALIB_SAMPLES);
  float ay = trimmed_mean(samples_ay, CALIB_SAMPLES);
  float az = trimmed_mean(samples_az, CALIB_SAMPLES);

  // Acquire mutex: blocks Task_Filter ONLY for the trigonometric calculations
  // and bias injection (~50 µs). The 2 s I2C sampling above happens WITHOUT
  // the mutex — Task_Filter keeps running with the old biases during collection,
  // which is correct (coherent data until the swap).
  // 2 s timeout: if Task_Filter holds the lock during a Kalman correct,
  // prevents loop() from blocking indefinitely (v0.9.11). In practice the lock
  // lasts ~10 ms max, so 2 s is more than sufficient.
  if (xSemaphoreTake(telemetry_mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    bias_gx = trimmed_mean(samples_gx, CALIB_SAMPLES);
    bias_gy = trimmed_mean(samples_gy, CALIB_SAMPLES);
    bias_gz = trimmed_mean(samples_gz, CALIB_SAMPLES);

    float phi = atan2f(ay, az);
    float norm_ay_az_sq = ay * ay + az * az;
    float norm_ay_az = (norm_ay_az_sq > 0.0f)
                           ? (norm_ay_az_sq * fast_inv_sqrt(norm_ay_az_sq))
                           : 0.0f;
    float theta = atan2f(-ax, norm_ay_az);

    sin_phi = sinf(phi);
    cos_phi = cosf(phi);
    sin_theta = sinf(theta);
    cos_theta = cosf(theta);

    float ax_r, ay_r, az_r;
    rotate_3d(ax, ay, az, ax_r, ay_r, az_r, cos_phi, sin_phi, cos_theta,
             sin_theta);
    bias_ax = ax_r - 0.0f;
    bias_ay = ay_r - 0.0f;
    bias_az = az_r - 1.0f;
    if (mag_sample_count >= CALIB_MAG_MIN_VALID) {
      mag_ref_ut_x = trimmed_mean(samples_mag_x, mag_sample_count);
      mag_ref_ut_y = trimmed_mean(samples_mag_y, mag_sample_count);
      mag_ref_ut_z = trimmed_mean(samples_mag_z, mag_sample_count);
      mag_ref_valid = true;
    } else {
      mag_ref_ut_x = 0.0f;
      mag_ref_ut_y = 0.0f;
      mag_ref_ut_z = 0.0f;
      mag_ref_valid = false;
    }

    prev_ax = 0;
    prev_ay = 0;
    prev_az = 0;
    prev_gx = 0;
    prev_gy = 0;
    prev_gz = 0;
    ahrs.q[0] = 1.0f;
    ahrs.q[1] = 0.0f;
    ahrs.q[2] = 0.0f;
    ahrs.q[3] = 0.0f;

    // v1.3.2: signal Task_Filter to reset ESKF, ZARU buffers, and GPS origin.
    // The flag is set here under telemetry_mutex so Task_Filter reads it
    // coherently on the next iteration (after this mutex is released).
    recalibration_pending = true;

    xSemaphoreGive(telemetry_mutex);

    // v1.4.0: inject CalibrationRecord into SD stream for SITL.
    // Sentinel UINT64_MAX in timestamp_us (~584,942 years) is impossible in real data.
    // Python parser detects it and updates calibration params for subsequent samples.
    // sd_queue is NULL during boot calibration (created later) — only mid-session fires.
    if (sd_queue != NULL) {
      TelemetryRecord crec = {};
      crec.timestamp_us = UINT64_MAX;   // sentinel: 0xFFFFFFFFFFFFFFFF
      crec.ax = sin_phi;                // reuse float fields with known semantics
      crec.ay = cos_phi;
      crec.az = sin_theta;
      crec.gx = cos_theta;
      crec.gy = bias_ax;
      crec.gz = bias_ay;
      crec.temp_c = bias_az;
      crec.gps_sog_kmh = bias_gx;
      crec.gps_alt_m = bias_gy;
      crec.gps_hdop = bias_gz;
      xQueueSend(sd_queue, &crec, 0);
    }
  } else {
    Serial.println("[CALIB] Mutex timeout 2s: calibration aborted!");
  }
}
