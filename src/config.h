// SPDX-License-Identifier: GPL-3.0-or-later
// config.h — Constants, pin definitions, and calibration parameters
#pragma once

#include <math.h>

// ── Firmware ────────────────────────────────────────────────────────────────
const char *const FIRMWARE_VERSION = "v1.2.1";

// ── Timing and Sampling ────────────────────────────────────────────────────
const float FREQ_HZ = 50.0f;           // Core system sampling frequency (IMU + ESKF)
const float DT = 1.0f / FREQ_HZ;       // Delta Time in seconds
const int DT_MS = (int)(DT * 1000.0f); // Delta Time in milliseconds for FreeRTOS

// ── Main Loop ──────────────────────────────────────────────────────────────
const int LOOP_HZ = 100;
const int MQTT_10HZ_CYCLES = LOOP_HZ / 10;
const int DISPLAY_5HZ_CYCLES = LOOP_HZ / 5;

// ── Math ───────────────────────────────────────────────────────────────────
const float DEG2RAD = (float)M_PI / 180.0f;

// ── EMA Filter ─────────────────────────────────────────────────────────────
// Single EMA on all accel + gyro axes (tau ~313 ms at 50 Hz).
// Does NOT feed the ESKF (Data Fork): used only for display/MQTT/SD/ZARU.
const float alpha = 0.06f;

// ── Statistical ZUPT/ZARU Engine ───────────────────────────────────────────
static constexpr int VAR_BUF_SIZE = 50;                // 1 s of samples at 50 Hz
static constexpr float VAR_STILLNESS_THRESHOLD = 0.05f; // [(deg/s)^2] ema_gz variance threshold
static constexpr float ZUPT_GPS_MAX_KMH = 2.0f;        // [km/h] GPS speed threshold
static constexpr int SD_FLUSH_EVERY = 50;               // flush interval: 1 s at 50 Hz

// ── WiFi ───────────────────────────────────────────────────────────────────
static constexpr int MAX_WIFI_NETWORKS = 2;

// ── GPS Hardware (AtomS3 Grove) ────────────────────────────────────────────
#define GPS_RX_PIN 1      // Grove: module TX → MCU RX (confirmed by hardware test)
#define GPS_TX_PIN 2      // Grove: module RX → MCU TX (confirmed by hardware test)
#define GPS_BAUD   115200 // ATGM336H-6N ships at 115200 (non-standard, not 9600)

// ── Micro SD SPI Pins ──────────────────────────────────────────────────────
#define SCK_PIN  7
#define MISO_PIN 8
#define MOSI_PIN 6
#define CS_PIN   5 // If the card is not detected, try 38

// ── Ellipsoidal Calibration (Tumble Test) ──────────────────────────────────
// ⚠️ DEFAULT UNCALIBRATED STATE
// You MUST perform your own tumble test to populate these matrices.
// See the README for instructions. Failure to do so will cause ESKF drift.
//
// Formula: a_calibrated = W * (a_raw - b)
// Applied in Task_Filter as STEP 1 (before rotate_3d and Madgwick).

// Bias vector (Offset) - Default: 0.0f
const float CALIB_B[3] = {
    0.0f, // AX
    0.0f, // AY
    0.0f  // AZ
};

// Transformation Matrix (Scale & Orthogonality) - Default: Identity Matrix
// WARNING: Do not set the main diagonal to 0, or acceleration will be multiplied by 0!
const float CALIB_W[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

/*
// EXAMPLE OF A CALIBRATED SET (From the author's specific MPU-6886 unit)
// Result: σ(‖a‖) 0.042 g → 0.023 g (−45.6 %)
// AZ bias = −0.065 g corrects the anomaly identified in pendulum tests.
const float CALIB_B_EXAMPLE[3] = { -0.00125f, +0.00429f, -0.06491f };

const float CALIB_W_EXAMPLE[3][3] = {
    {+1.000824f, -0.000511f, -0.001575f},
    {-0.000511f, +1.000989f, -0.000132f},
    {-0.001575f, -0.000132f, +1.003466f}
};
*/