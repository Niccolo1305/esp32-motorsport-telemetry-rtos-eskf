// SPDX-License-Identifier: GPL-3.0-or-later
// IImuProvider.h — Pure virtual interface for IMU data acquisition (HAL)
//
// Mirrors IGpsProvider.h: decouples Task_I2C and calibrate_alignment()
// from the physical sensor. Concrete implementations:
//   - Mpu6886Provider (M5Stack AtomS3, MPU-6886 via M5Unified)
//   - Bmi270Provider (AtomS3R, BMI270+BMM150 via Bosch Sensor APIs)
#pragma once

#include "types.h"

class IImuProvider {
public:
    virtual ~IImuProvider() = default;

    /// Initialise hardware and cache the bring-up verification result.
    /// Concrete backends may implement this as a register-level or functional
    /// verification step depending on the sensor family.
    /// Must be called once in setup() AFTER M5.begin().
    virtual void begin() = 0;

    /// Read accelerometer, gyroscope, temperature, optional magnetometer data,
    /// and capture a single timestamp for the sample.
    virtual void update(ImuRawData& outData) = 0;

    /// Returns the cached bring-up verification result from begin().
    /// No new bus traffic is expected here.
    virtual bool verifyConfig() = 0;
};
