// SPDX-License-Identifier: GPL-3.0-or-later
// madgwick.h — Madgwick AHRS (Attitude and Heading Reference System)
//
// Header-only. Uses fast_inv_sqrt from math_utils.h.
// Adaptive beta: decays linearly from full to zero as ||a|| deviates from 1g.
#pragma once

#include <math.h>

#include "config.h"
#include "math_utils.h"

class MadgwickAHRS {
public:
  float sampleperiod;
  float beta;
  float q[4];

  MadgwickAHRS(float period = DT, float b = 0.1f)
      : sampleperiod(period), beta(b) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
  }

  void update_imu(float gx, float gy, float gz, float ax, float ay, float az) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;

    // STEP 1 — Accelerometer normalisation.
    // norm_accel_sq saved pre-normalisation for adaptive beta.
    // BUG-5 fix: threshold guard instead of exact zero comparison —
    // subnormal inputs would pass == 0.0f and overflow fast_inv_sqrt.
    float norm_accel_sq = ax * ax + ay * ay + az * az;
    if (norm_accel_sq < 1e-8f)
      return;
    float inv_norm_accel = fast_inv_sqrt(norm_accel_sq);
    float raw_norm = norm_accel_sq * inv_norm_accel;
    ax *= inv_norm_accel;
    ay *= inv_norm_accel;
    az *= inv_norm_accel;

    // STEP 2 — Gradient Descent (orientation error).
    float s0 = -_2q2 * (2.0f * q1 * q3 - _2q0 * q2 - ax) +
               _2q1 * (2.0f * q0 * q1 + _2q2 * q3 - ay);
    float s1 = _2q3 * (2.0f * q1 * q3 - _2q0 * q2 - ax) +
               _2q0 * (2.0f * q0 * q1 + _2q2 * q3 - ay) -
               4.0f * q1 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az);
    float s2 = -_2q0 * (2.0f * q1 * q3 - _2q0 * q2 - ax) +
               _2q3 * (2.0f * q0 * q1 + _2q2 * q3 - ay) -
               4.0f * q2 * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2 - az);
    float s3 = _2q1 * (2.0f * q1 * q3 - _2q0 * q2 - ax) +
               _2q2 * (2.0f * q0 * q1 + _2q2 * q3 - ay);

    float norm_s_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (norm_s_sq < 1e-8f)
      return;
    float inv_norm_s = fast_inv_sqrt(norm_s_sq);
    s0 *= inv_norm_s;
    s1 *= inv_norm_s;
    s2 *= inv_norm_s;
    s3 *= inv_norm_s;

    // LINEAR ADAPTIVE BETA
    // distance = 0.000 g → full beta | distance = 0.150 g → zero beta
    // Threshold 0.15 g ≈ 0.75 g lateral (Pythagoras): correct for car manoeuvres.
    // With ellipsoidal calibration active, ‖a‖ at rest ≈ 1.000 g exactly
    // → working point always in full-beta regime when stationary.
    float dist_from_1g = fabsf(raw_norm - 1.0f);
    float beta_eff = beta * fmaxf(0.0f, 1.0f - (dist_from_1g / 0.15f));

    // STEP 3 — Quaternion derivative: q_dot = 0.5 * q ⊗ ω
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // STEP 4 — Euler integration + linear beta correction
    q0 += (qDot0 - beta_eff * s0) * sampleperiod;
    q1 += (qDot1 - beta_eff * s1) * sampleperiod;
    q2 += (qDot2 - beta_eff * s2) * sampleperiod;
    q3 += (qDot3 - beta_eff * s3) * sampleperiod;

    // STEP 5 — Quaternion normalisation
    float norm_q_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (norm_q_sq > 0.0f) {
      float inv_norm_q = fast_inv_sqrt(norm_q_sq);
      q[0] = q0 * inv_norm_q;
      q[1] = q1 * inv_norm_q;
      q[2] = q2 * inv_norm_q;
      q[3] = q3 * inv_norm_q;
    }
  }

  // Third column of Rᵀ: projects [0,0,1g] into the sensor frame.
  void get_gravity_vector(float &gx_out, float &gy_out, float &gz_out) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    gx_out = 2.0f * (q1 * q3 - q0 * q2);
    gy_out = 2.0f * (q0 * q1 + q2 * q3);
    gz_out = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  }
};
