// SPDX-License-Identifier: GPL-3.0-or-later
// Mpu6886Provider.h — Concrete IImuProvider for MPU-6886 (M5Stack AtomS3)
//
// Encapsulates all IMU hardware concerns:
//   - I2C bus initialisation (Wire.begin on AtomS3 internal pins 38/39)
//   - FSR verification (GYRO_CONFIG 0x1B, ACCEL_CONFIG 0x1C)
//   - Hardware DLPF configuration (CONFIG 0x1A, ACCEL_CONFIG2 0x1D)
//   - Accelerometer, gyroscope and temperature readout via M5Unified
//   - Hardware timestamp capture (esp_timer_get_time)
//
// Lifetime: MUST be a static or global object (same pattern as SerialGpsWrapper).
//
// Usage:
//   // In Telemetria.ino, file scope:
//   static Mpu6886Provider imuProvider;
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
#include <Wire.h>

#include "IImuProvider.h"

class Mpu6886Provider : public IImuProvider {
public:
    void begin() override {
        // M5Unified uses a protected internal I2C bus and leaves Wire uninitialised.
        // Explicit init on AtomS3 internal pins (SDA=38, SCL=39) is required
        // before any Wire.beginTransmission() — without this, the ESP32 crashes.
        Wire.begin(38, 39);

        // ── FSR Verification ────────────────────────────────────────────────
        // M5Unified sets ±8g / ±2000 dps during M5.begin(). Verify the
        // registers actually reflect this before the calibration pipeline
        // uses the scale factors.
        // GYRO_CONFIG  (0x1B): bits 4:3 = 0b11 → ±2000 dps
        // ACCEL_CONFIG (0x1C): bits 4:3 = 0b10 → ±8 g
        Wire.beginTransmission(0x68);
        Wire.write(0x1B);
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 1);
        uint8_t gyro_cfg = Wire.available() ? Wire.read() : 0xFF;

        Wire.beginTransmission(0x68);
        Wire.write(0x1C);
        Wire.endTransmission(false);
        Wire.requestFrom(0x68, 1);
        uint8_t accel_cfg = Wire.available() ? Wire.read() : 0xFF;

        bool gyro_ok  = ((gyro_cfg  & 0x18) == 0x18); // ±2000 dps
        bool accel_ok = ((accel_cfg & 0x18) == 0x10); // ±8 g

        if (gyro_ok && accel_ok) {
            Serial.printf("[IMU] FSR OK: GYRO=0x%02X (+/-2000dps) ACCEL=0x%02X (+/-8g)\n",
                          gyro_cfg, accel_cfg);
        } else {
            Serial.printf("[IMU] FSR MISMATCH! GYRO=0x%02X (want 0x18) ACCEL=0x%02X (want 0x10)\n",
                          gyro_cfg, accel_cfg);
        }

        // ── Hardware DLPF at 20 Hz ──────────────────────────────────────────
        // MPU-6886 registers:
        //   0x1A CONFIG        → DLPF_CFG = 4 → gyro BW  ~20 Hz
        //   0x1D ACCEL_CONFIG2 → A_DLPF_CFG = 4 → accel BW ~21 Hz
        Wire.beginTransmission(0x68);
        Wire.write(0x1A);
        Wire.write(0x04);
        uint8_t dlpf_gyro_err = Wire.endTransmission();

        Wire.beginTransmission(0x68);
        Wire.write(0x1D);
        Wire.write(0x04);
        uint8_t dlpf_accel_err = Wire.endTransmission();

        bool dlpf_ok = (dlpf_gyro_err == 0 && dlpf_accel_err == 0);

        if (dlpf_ok) {
            Serial.println("[IMU] Hardware DLPF forced to 20Hz (ACK OK).");
        } else {
            Serial.printf("[IMU] DLPF WARN: gyro_err=%d accel_err=%d\n",
                          dlpf_gyro_err, dlpf_accel_err);
        }

        _config_ok = gyro_ok && accel_ok && dlpf_ok;
    }

    void update(ImuRawData& out) override {
        M5.Imu.getAccelData(&out.ax, &out.ay, &out.az);
        M5.Imu.getGyroData(&out.gx, &out.gy, &out.gz);
        out.temp_c = 0.0f;
        M5.Imu.getTemp(&out.temp_c);
        out.timestamp_us = esp_timer_get_time();
    }

    bool verifyConfig() override {
        return _config_ok;
    }

private:
    bool _config_ok = false;
};
