// SPDX-License-Identifier: GPL-3.0-or-later
// IImuProvider.h — Pure virtual interface for IMU data acquisition (HAL)
//
// Mirrors IGpsProvider.h: decouples Task_I2C and calibrate_alignment()
// from the physical sensor. Concrete implementations:
//   - Mpu6886Provider (M5Stack AtomS3, MPU-6886 via M5Unified)
//   - Future: Bmi270Provider (AtomS3R, BMI270)
#pragma once

#include "types.h"

class IImuProvider {
public:
    virtual ~IImuProvider() = default;

    /// Initialise hardware: I2C bus, FSR verification, DLPF configuration.
    /// Must be called once in setup() AFTER M5.begin().
    virtual void begin() = 0;

    /// Read accelerometer, gyroscope, temperature and capture timestamp.
    /// Populates all fields of outData in a single call.
    virtual void update(ImuRawData& outData) = 0;

    /// Returns true if FSR and DLPF registers match expected values.
    /// Result is cached from begin(); no I2C traffic.
    virtual bool verifyConfig() = 0;
};
