// SPDX-License-Identifier: GPL-3.0-or-later
// calibration.cpp — Sensor calibration (geometric alignment at boot)
//
// calibrate_alignment() runs a 2-second static sampling window (100 samples),
// computes mounting angles (phi, theta) and biases, then injects the results
// into the global calibration state under telemetry_mutex.
#include "calibration.h"

#include <M5Unified.h>

#include "globals.h"
#include "math_utils.h"

void calibrate_alignment() {
  // Array size and loop tied to the same constant.
  // If FREQ_HZ changes, both resize without stack buffer overflow.
  static constexpr int CALIB_SAMPLES = (int)(50.0f * 2.0f); // 2 s at 50 Hz
  static_assert(CALIB_SAMPLES <= 200, "CALIB_SAMPLES too large for the stack");

  float samples_ax[CALIB_SAMPLES], samples_ay[CALIB_SAMPLES], samples_az[CALIB_SAMPLES];
  float samples_gx[CALIB_SAMPLES], samples_gy[CALIB_SAMPLES], samples_gz[CALIB_SAMPLES];

  // Suspend Task_I2C (Core 0) to avoid I2C bus contention.
  // Without this protection, getAccelData/getGyroData here and in Task_I2C
  // run on two cores in parallel on the same physical bus, producing
  // interleaved reads and a corrupted calibration (v0.9.2).
  if (TaskI2CHandle != NULL) vTaskSuspend(TaskI2CHandle);

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    M5.Imu.getAccelData(&ax, &ay, &az);
    M5.Imu.getGyroData(&gx, &gy, &gz);
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
  } else {
    Serial.println("[CALIB] Mutex timeout 2s: calibration aborted!");
  }
}
