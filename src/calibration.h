// SPDX-License-Identifier: GPL-3.0-or-later
// calibration.h — Sensor calibration (geometric alignment at boot)
#pragma once

class IImuProvider;

void calibrate_alignment(IImuProvider* imu);
