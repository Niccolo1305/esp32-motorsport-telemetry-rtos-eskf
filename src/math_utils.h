// SPDX-License-Identifier: GPL-3.0-or-later
// math_utils.h — Math utilities: fast_inv_sqrt, rotate_3d, ellipsoid calibration, trimmed_mean
//
// Header-only (all functions inline). No global state dependency.
#pragma once

#include <math.h>
#include <string.h>
#include <algorithm>

#include "config.h"

// Fast Inverse Square Root (Quake III, UB-safe with memcpy, 2 N-R iterations).
// Relative error < 0.0002 % with 2 iterations.
inline float fast_inv_sqrt(float x) {
  if (x <= 0.0f)
    return 0.0f;
  float xhalf = 0.5f * x;
  uint32_t i;
  memcpy(&i, &x, sizeof(i));
  i = 0x5f3759df - (i >> 1);
  memcpy(&x, &i, sizeof(x));
  x = x * (1.5f - xhalf * x * x);
  x = x * (1.5f - xhalf * x * x);
  return x;
}

// Two-step Euler rotation: Roll φ then Pitch θ.
// Parameters passed explicitly: thread-safe (no global reads).
inline void rotate_3d(float x, float y, float z, float &x2, float &y1, float &z2,
              float c_phi, float s_phi, float c_theta, float s_theta) {
  float y1_ = y * c_phi - z * s_phi;
  float z1 = y * s_phi + z * c_phi;
  x2 = x * c_theta + z1 * s_theta;
  y1 = y1_;
  z2 = -x * s_theta + z1 * c_theta;
}

// Ellipsoidal calibration: a_cal = W * (a_raw - b)
// Applied in the chip's native frame (BEFORE rotate_3d).
// Cost: 9 FMA on hardware FPU → negligible at 50 Hz.
inline void apply_ellipsoidal_calibration(float &ax, float &ay, float &az) {
  const float bx = ax - CALIB_B[0];
  const float by = ay - CALIB_B[1];
  const float bz = az - CALIB_B[2];
  ax = CALIB_W[0][0] * bx + CALIB_W[0][1] * by + CALIB_W[0][2] * bz;
  ay = CALIB_W[1][0] * bx + CALIB_W[1][1] * by + CALIB_W[1][2] * bz;
  az = CALIB_W[2][0] * bx + CALIB_W[2][1] * by + CALIB_W[2][2] * bz;
}

inline float trimmed_mean(float *arr, int n) {
  std::sort(arr, arr + n);
  int trim = (int)(n * 0.1f);
  float sum = 0.0f;
  int count = 0;
  for (int i = trim; i < n - trim; i++) {
    sum += arr[i];
    count++;
  }
  return (count > 0) ? sum / (float)count : 0.0f;
}
