// SPDX-License-Identifier: GPL-3.0-or-later
// config.h - Constants, pin definitions, and calibration parameters
#pragma once

#include <math.h>

#ifdef USE_BMI270
const char *const FIRMWARE_VERSION = "v1.7.5-atoms3r";
#else
const char *const FIRMWARE_VERSION = "v1.5.2-clean";
#endif

// Timing and sampling
const float FREQ_HZ = 50.0f;           // Core system sampling frequency (IMU + ESKF)
const float DT = 1.0f / FREQ_HZ;       // Delta time in seconds
const int DT_MS = (int)(DT * 1000.0f); // Delta time in milliseconds for FreeRTOS

// Main loop
const int LOOP_HZ = 100;
const int MQTT_10HZ_CYCLES = LOOP_HZ / 10;
const int DISPLAY_5HZ_CYCLES = LOOP_HZ / 5;

// Math
const float DEG2RAD = (float)M_PI / 180.0f;

// Single EMA on all accel + gyro axes (tau ~313 ms at 50 Hz).
// Does NOT feed the ESKF (Data Fork): used only for display/MQTT/SD/ZARU.
const float alpha = 0.06f;

// Statistical ZUPT/ZARU engine
static constexpr int VAR_BUF_SIZE = 50; // 1 s of samples at 50 Hz
// Variance thresholds for raw gyro (post-rotation, pre-EMA).
// Raw MEMS noise is higher than EMA-smoothed: starting estimates,
// tune empirically from stationary recordings (raw_gx/gy/gz in SD log).
#ifdef USE_BMI270
// BMI270 thresholds for the Bosch-direct 50 Hz / ~20 Hz LPF configuration.
// Retune from fresh stationary recordings on the actual unit and mount.
static constexpr float VAR_STILLNESS_GZ_THRESHOLD  = 0.05f; // [(deg/s)^2]
static constexpr float VAR_STILLNESS_GXY_THRESHOLD = 0.08f; // [(deg/s)^2]
#else
// MPU-6886 ZARU thresholds (tuned empirically from stationary recordings)
static constexpr float VAR_STILLNESS_GZ_THRESHOLD  = 0.25f; // [(deg/s)^2]
static constexpr float VAR_STILLNESS_GXY_THRESHOLD = 0.35f; // [(deg/s)^2]
#endif
static constexpr float ZUPT_GPS_MAX_KMH = 2.0f; // [km/h] GPS speed threshold
static constexpr int SD_FLUSH_EVERY  = 250;     // flush interval: 5 s at 50 Hz (reduces FAT/GC spikes)
static constexpr int SD_QUEUE_DEPTH  = 200;     // record buffer depth (4 s at 50 Hz)
static constexpr int SD_TASK_STACK   = 8192;    // Task_SD_Writer stack size (bytes)
static constexpr int SD_WRITE_RETRY_DELAY_MS = 100;   // retry cadence for transient SD no-progress writes
static constexpr int SD_WRITE_STALL_TIMEOUT_MS = 10000; // max per-record no-progress window before fatal

// Non-holonomic constraint (NHC)
static constexpr float NHC_R = 0.5f;            // [(m/s)^2] lateral vel measurement noise
static constexpr float NHC_MIN_SPEED_MS = 1.4f; // ~5 km/h; below this NHC is off
static constexpr float NHC_MAX_LAT_G = 0.5f;    // [g] disable NHC above this lateral accel

// Enhanced straight-line ZARU
static constexpr float STRAIGHT_ALPHA = 0.01f;         // EMA alpha for straight-line bias learning
static constexpr float STRAIGHT_COG_MAX_RAD = 0.10f;   // [rad] ~5.7 deg max COG change over baseline
static constexpr float STRAIGHT_MIN_SPEED_KMH = 40.0f; // min speed for straight detection
static constexpr float STRAIGHT_MAX_LAT_G = 0.05f;     // [g] max lateral accel
static constexpr float COG_MIN_BASELINE_M = 15.0f;     // [m] min displacement for COG computation

// Virtual Gravity Plane Lock (VGPL)
static constexpr float VGPL_NORM_GATE = 0.15f;   // [g] residual norm deviation that fully disables beta
static constexpr float VGPL_RATE_LIMIT = 0.15f;  // [g/sample] max per-cycle change for each compensation channel
static constexpr float VGPL_BETA_FLOOR = 0.005f; // minimum beta (never fully zero)

// WiFi
static constexpr int MAX_WIFI_NETWORKS = 2;

#ifdef USE_BMI270
// AtomS3R board constants.
// M5.begin() initializes the internal board I2C bus. The AtomS3R Bosch-direct
// provider uses M5.In_I2C as the only transport for the internal sensor bus.
static constexpr int ATOMS3R_IMU_SDA_PIN = 45;
static constexpr int ATOMS3R_IMU_SCL_PIN = 0;
static constexpr uint32_t ATOMS3R_IMU_I2C_FREQ = 400000;
static constexpr uint8_t ATOMS3R_BMI270_ADDR_PRIMARY = 0x69;
static constexpr uint8_t ATOMS3R_BMI270_ADDR_FALLBACK = 0x68;
static constexpr uint8_t ATOMS3R_BMM150_ADDR = 0x10;
#define GPS_RX_PIN 1      // TODO: verify Grove connector mapping on AtomS3R
#define GPS_TX_PIN 2      // TODO: verify
#define GPS_BAUD   115200
#define SCK_PIN  7        // TODO: verify SPI connector on AtomS3R
#define MISO_PIN 8        // TODO: verify
#define MOSI_PIN 6        // TODO: verify
#define CS_PIN   5        // TODO: verify
#else
// AtomS3 GPIO (verified by hardware test)
#define GPS_RX_PIN 1      // Grove: module TX -> MCU RX
#define GPS_TX_PIN 2      // Grove: module RX -> MCU TX
#define GPS_BAUD   115200 // ATGM336H-6N ships at 115200 (non-standard, not 9600)
#define SCK_PIN  7
#define MISO_PIN 8
#define MOSI_PIN 6
#define CS_PIN   5 // If the card is not detected, try 38
#endif

// Ellipsoidal calibration (tumble test)
// Default uncalibrated state for AtomS3R: perform a dedicated tumble test
// before trusting long-run ESKF results on the new sensor.
#ifdef USE_BMI270
const float CALIB_B[3] = { 0.0f, 0.0f, 0.0f };
const float CALIB_W[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};
#else
// MPU-6886 calibrated (author's specific unit).
// Result: sigma(norm(a)) 0.042 g -> 0.023 g (-45.6%)
// AZ bias = -0.065 g corrects the anomaly identified in pendulum tests.
const float CALIB_B[3] = { -0.00125f, +0.00429f, -0.06491f };

const float CALIB_W[3][3] = {
    {+1.000824f, -0.000511f, -0.001575f},
    {-0.000511f, +1.000989f, -0.000132f},
    {-0.001575f, -0.000132f, +1.003466f}
};
#endif
