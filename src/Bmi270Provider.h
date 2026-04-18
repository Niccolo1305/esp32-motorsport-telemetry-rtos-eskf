// SPDX-License-Identifier: GPL-3.0-or-later
// Bmi270Provider.h — Concrete IImuProvider for BMI270+BMM150 (M5Stack AtomS3R)
//
// Backend: M5Unified 0.2.13+ (handles BMI270 init, config file upload,
// AUX I2C BMM150 setup, axis corrections, and data readout internally).
//
// This provider does NOT access Wire or BMI270/BMM150 registers directly.
// M5.begin() must be called before begin().
//
// FSR/DLPF/ODR: accepts M5Unified defaults (operational assumptions, not verified):
//   Accel FSR: assumed ±8g (config blob, not register-inspectable — confirm at bring-up)
//   Gyro FSR:  assumed ±2000 dps (config blob, not register-inspectable — confirm at bring-up)
//   DLPF:      determined by config blob (bandwidth unknown — characterize from noise data)
//   BMM150 ODR: 30 Hz (register 0x4C = 0x38, verified in M5Unified source)
// If these defaults prove inadequate, a low-level backend replacing M5Unified
// entirely (not a hybrid) would be required.
//
// Magnetometer units: M5Unified raw output (arbitrary, AK8963-scaled).
//   M5Unified applies AK8963 conversion factor (10*4912/32760) instead of
//   BMM150-specific compensation. No trim register readout. The output is
//   NOT in µT or any standard physical unit. Downstream calibration matrices
//   (MAG_W/MAG_B in config.h) absorb the missing compensation once populated
//   via offline empirical ellipsoid fit.
//
// verifyConfig(): functional/operational check only — not register-level.
//   Reads one accel sample and verifies plausible magnitude (~1g at rest).
//   Does NOT verify FSR/DLPF register values.
//
// NOTE: verifyConfig() is a functional M5Unified backend check only; it does
// NOT prove BMI270/BMM150 FSR, DLPF, ODR, or magnetometer data semantics.
// Those must be confirmed during hardware bring-up and noise characterization.
//
// Lifetime: MUST be a static or global object (same pattern as Mpu6886Provider).
//
// Usage:
//   // In Telemetria.ino, file scope:
//   static Bmi270Provider imuProvider;
//
//   // In setup(), AFTER M5.begin():
//   imuProvider.begin();
//   if (!imuProvider.verifyConfig()) { /* display error */ }
//
//   // When creating Task_I2C:
//   xTaskCreatePinnedToCore(Task_I2C, "I2C", 4096, &imuProvider, 3, &TaskI2CHandle, 0);
//
//   // In calibrate_alignment():
//   calibrate_alignment(&imuProvider);
#pragma once

#include <M5Unified.h>
#include <math.h>

#include "IImuProvider.h"

class Bmi270Provider : public IImuProvider {
public:
    void begin() override {
        // M5Unified has already initialized BMI270+BMM150 during M5.begin().
        // Functional verification: attempt a sensor read and check plausibility.
        float tax, tay, taz;
        _config_ok = M5.Imu.getAccelData(&tax, &tay, &taz);
        if (_config_ok) {
            float amag = sqrtf(tax * tax + tay * tay + taz * taz);
            // Plausibility: total accel magnitude should be near 1g at rest.
            // Wide range [0.5, 1.5] to tolerate orientation and vibration.
            _config_ok = (amag > 0.5f && amag < 1.5f);
            Serial.printf("[IMU] BMI270 functional check: |a|=%.3fg -> %s\n",
                          amag, _config_ok ? "OK" : "OUT OF RANGE");
        } else {
            Serial.println("[IMU] BMI270 functional check FAILED — M5.Imu not responding");
        }

        // Check mag availability (separate from accel/gyro OK).
        float tmx, tmy, tmz;
        _mag_available = M5.Imu.getMag(&tmx, &tmy, &tmz);
        Serial.printf("[IMU] BMM150 mag: %s\n",
                      _mag_available ? "detected" : "NOT DETECTED");
    }

    void update(ImuRawData& out) override {
        M5.Imu.getAccelData(&out.ax, &out.ay, &out.az);
        M5.Imu.getGyroData(&out.gx, &out.gy, &out.gz);
        out.temp_c = 0.0f;
        M5.Imu.getTemp(&out.temp_c);
        if (_mag_available) {
            // mag_valid reflects M5Unified update cycle result, not mag-specific
            // freshness. At 50 Hz sampling with 30 Hz BMM150 ODR, some cycles
            // return cached (non-fresh) mag data with valid=true.
            out.mag_valid = M5.Imu.getMag(&out.mx, &out.my, &out.mz);
        } else {
            out.mx = 0.0f;
            out.my = 0.0f;
            out.mz = 0.0f;
            out.mag_valid = false;
        }
        out.timestamp_us = esp_timer_get_time();
    }

    bool verifyConfig() override {
        return _config_ok;
    }

private:
    bool _config_ok = false;
    bool _mag_available = false;
};
