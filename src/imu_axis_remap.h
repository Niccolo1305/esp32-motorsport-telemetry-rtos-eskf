// SPDX-License-Identifier: GPL-3.0-or-later
// imu_axis_remap.h - Explicit chip->pipeline axis convention mapping
//
// The acquisition layer must stay sensor-faithful. This helper documents the
// firmware convention expected by calibration.cpp and filter_task.cpp.
#pragma once

inline void remap_chip_axes_to_pipeline(float& x, float& y, float& z) {
#ifdef USE_BMI270
  const float in_x = x;
  const float in_y = y;
  x = in_y;
  y = -in_x;
  z = z;
#else
  (void)x;
  (void)y;
  (void)z;
#endif
}
