// AtomS3 MPU-6886 Sensor Bench - Bare-metal characterization firmware
// Runs standardized tests and outputs CSV via USB Serial for offline analysis.
//
// Build:  pio run -e atoms3_bench
// Flash:  pio run -e atoms3_bench -t upload
// Log:    pio device monitor | tee atoms3_results.txt

#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include "bench_common.h"

// -- MPU-6886 Constants -------------------------------------------------------
static constexpr uint8_t IMU_ADDR = 0x68;

// Register addresses
static constexpr uint8_t REG_WHO_AM_I     = 0x75;
static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_CONFIG       = 0x1A;
static constexpr uint8_t REG_ACCEL_CFG2   = 0x1D;
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_PWR_MGMT_2   = 0x6C;
static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;

// Expected values
static constexpr uint8_t EXPECTED_WHO_AM_I = 0x19;
static constexpr uint8_t EXPECTED_GYRO_FS  = 0x18; // +/-2000 dps
static constexpr uint8_t EXPECTED_ACCEL_FS = 0x10; // +/-8 g
static constexpr uint8_t EXPECTED_DLPF     = 0x04; // ~20 Hz

// -- Test Parameters ----------------------------------------------------------
static constexpr int STATIC_NOISE_SECONDS  = 300;
static constexpr int STATIC_NOISE_SAMPLES  = STATIC_NOISE_SECONDS * BENCH_FREQ_HZ;
static constexpr int ODR_TEST_SAMPLES      = 1000;
static constexpr int THERMAL_SECONDS       = 900;
static constexpr int THERMAL_SAMPLES       = THERMAL_SECONDS * BENCH_FREQ_HZ;
static constexpr int ALLAN_SECONDS         = 300;
static constexpr int ALLAN_SAMPLES         = ALLAN_SECONDS * BENCH_FREQ_HZ;
static constexpr int AXIS_CHECK_SECONDS    = 5;
static constexpr int AXIS_CHECK_SAMPLES    = AXIS_CHECK_SECONDS * BENCH_FREQ_HZ;

// -- State Machine ------------------------------------------------------------
enum TestPhase {
    PHASE_REGISTER_DUMP = 0,
    PHASE_AXIS_CHECK,
    PHASE_STATIC_NOISE,
    PHASE_ODR_JITTER,
    PHASE_THERMAL,
    PHASE_ALLAN,
    PHASE_DONE,
    PHASE_COUNT
};

static TestPhase current_phase = PHASE_REGISTER_DUMP;
static bool phase_started = false;
static uint64_t* ts_buf = nullptr;

static const char* phase_title(TestPhase phase) {
    switch (phase) {
        case PHASE_REGISTER_DUMP: return "P0: REG DUMP";
        case PHASE_AXIS_CHECK:    return "P1: AXIS CHECK";
        case PHASE_STATIC_NOISE:  return "P2: NOISE TEST";
        case PHASE_ODR_JITTER:    return "P3: ODR JITTER";
        case PHASE_THERMAL:       return "P4: THERMAL";
        case PHASE_ALLAN:         return "P5: ALLAN VAR";
        default:                  return "BENCH";
    }
}

static void update_stable_temp_window(const SensorSample* ring,
                                      int newest_index,
                                      StableTempWindowSummary& best_window) {
    const int window_samples = STABLE_TEMP_WINDOW_SAMPLES;
    const int start_index = (newest_index + 1) % window_samples;

    float temp_min = ring[start_index].temp_c;
    float temp_max = ring[start_index].temp_c;
    for (int i = 1; i < window_samples; ++i) {
        const float temp = ring[(start_index + i) % window_samples].temp_c;
        if (temp < temp_min) temp_min = temp;
        if (temp > temp_max) temp_max = temp;
    }

    float temp_p2p = temp_max - temp_min;
    if (temp_p2p > STABLE_TEMP_MAX_P2P_C) {
        return;
    }
    if (best_window.found && temp_p2p >= best_window.temp_p2p_c) {
        return;
    }

    best_window.found = true;
    best_window.temp_min_c = temp_min;
    best_window.temp_max_c = temp_max;
    best_window.temp_p2p_c = temp_p2p;
    best_window.start_ts_us = ring[start_index].ts_us;
    best_window.end_ts_us = ring[newest_index].ts_us;
    compute_window_imu_stats(ring, window_samples, start_index, window_samples, best_window.axis);
}

// -- Helper: Read one IMU sample ----------------------------------------------
static void read_imu(SensorSample& s) {
    M5.Imu.getAccelData(&s.ax, &s.ay, &s.az);
    M5.Imu.getGyroData(&s.gx, &s.gy, &s.gz);
    s.temp_c = 0.0f;
    M5.Imu.getTemp(&s.temp_c);
    s.ts_us = esp_timer_get_time();
}

// -- Phase 0: Register Dump ---------------------------------------------------
static void run_register_dump() {
    Serial.println("\n=== PHASE 0: REGISTER DUMP (MPU-6886) ===");
    display_phase("P0: REG DUMP");

    print_reg(IMU_ADDR, REG_WHO_AM_I,      "WHO_AM_I",     EXPECTED_WHO_AM_I);
    print_reg(IMU_ADDR, REG_GYRO_CONFIG,   "GYRO_CONFIG",  EXPECTED_GYRO_FS);
    print_reg(IMU_ADDR, REG_ACCEL_CONFIG,  "ACCEL_CONFIG", EXPECTED_ACCEL_FS);
    print_reg(IMU_ADDR, REG_CONFIG,        "CONFIG/DLPF",  EXPECTED_DLPF);
    print_reg(IMU_ADDR, REG_ACCEL_CFG2,    "ACCEL_CFG2",   EXPECTED_DLPF);
    print_reg_info(IMU_ADDR, REG_PWR_MGMT_1, "PWR_MGMT_1");
    print_reg_info(IMU_ADDR, REG_PWR_MGMT_2, "PWR_MGMT_2");
    print_reg_info(IMU_ADDR, REG_SMPLRT_DIV, "SMPLRT_DIV");

    Serial.println("=== END REGISTER DUMP ===\n");
}

// -- Phase 1: Axis Orientation Check ------------------------------------------
static void run_axis_check() {
    Serial.println("\n=== PHASE 1: AXIS ORIENTATION CHECK (5s) ===");
    Serial.println("Device must be flat, screen up, stationary.");
    display_phase("P1: AXIS CHECK", "Keep flat!");

    double sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    SensorSample s;

    for (int i = 0; i < AXIS_CHECK_SAMPLES; ++i) {
        read_imu(s);
        sum_ax += s.ax;
        sum_ay += s.ay;
        sum_az += s.az;
        sum_gx += s.gx;
        sum_gy += s.gy;
        sum_gz += s.gz;
        display_progress(i + 1, AXIS_CHECK_SAMPLES);
        delay(BENCH_DT_MS);
    }

    float n = (float)AXIS_CHECK_SAMPLES;
    float mean_ax = (float)(sum_ax / n);
    float mean_ay = (float)(sum_ay / n);
    float mean_az = (float)(sum_az / n);
    float mean_gx = (float)(sum_gx / n);
    float mean_gy = (float)(sum_gy / n);
    float mean_gz = (float)(sum_gz / n);

    Serial.println("--- Axis Means (device flat, screen up) ---");
    Serial.printf("  Accel: ax=%.4f g  ay=%.4f g  az=%.4f g\n", mean_ax, mean_ay, mean_az);
    Serial.printf("  Gyro:  gx=%.3f dps  gy=%.3f dps  gz=%.3f dps\n", mean_gx, mean_gy, mean_gz);
    Serial.println("  Expected: ax~0, ay~0, az~+1.0 g");

    bool az_ok = fabsf(mean_az - 1.0f) < 0.15f;
    bool ax_ok = fabsf(mean_ax) < 0.15f;
    bool ay_ok = fabsf(mean_ay) < 0.15f;
    Serial.printf("  Verdict: ax=%s  ay=%s  az=%s\n",
                  ax_ok ? "OK" : "WARN",
                  ay_ok ? "OK" : "WARN",
                  az_ok ? "OK" : "WARN");
    Serial.println("=== END AXIS CHECK ===\n");
}

// -- Phase 2: Static Noise Test -----------------------------------------------
static void run_static_noise() {
    Serial.printf("\n=== PHASE 2: STATIC NOISE TEST (%ds @ %dHz = %d samples) ===\n",
                  STATIC_NOISE_SECONDS, BENCH_FREQ_HZ, STATIC_NOISE_SAMPLES);
    Serial.println("Device must be stationary. Do not touch.");
    display_phase("P2: NOISE TEST", "Streaming 5 min...");

    Serial.println("--- CSV DATA ---");
    print_csv_header_imu();

    RunningImuStats stats;
    SensorSample s;
    for (int i = 0; i < STATIC_NOISE_SAMPLES; ++i) {
        read_imu(s);
        print_csv_row(s);
        stats.update(s);
        if ((i % 1000) == 0) display_progress(i, STATIC_NOISE_SAMPLES);
        delay(BENCH_DT_MS);
    }
    display_progress(STATIC_NOISE_SAMPLES, STATIC_NOISE_SAMPLES);

    print_running_imu_stats(stats);
    print_motion_correlation_matrix(stats);
    Serial.println("=== END STATIC NOISE TEST ===\n");
}

// -- Phase 3: ODR & Jitter Test -----------------------------------------------
static void run_odr_jitter() {
    Serial.printf("\n=== PHASE 3: ODR & JITTER TEST (%d samples) ===\n", ODR_TEST_SAMPLES);
    display_phase("P3: ODR JITTER");

    ts_buf = (uint64_t*)malloc(ODR_TEST_SAMPLES * sizeof(uint64_t));
    if (!ts_buf) {
        Serial.println("[ERROR] Failed to allocate timestamp buffer!");
        return;
    }

    for (int i = 0; i < ODR_TEST_SAMPLES; ++i) {
        float ax, ay, az, gx, gy, gz;
        M5.Imu.getAccelData(&ax, &ay, &az);
        M5.Imu.getGyroData(&gx, &gy, &gz);
        ts_buf[i] = esp_timer_get_time();
        if ((i % 100) == 0) display_progress(i, ODR_TEST_SAMPLES);
        delay(BENCH_DT_MS);
    }
    display_progress(ODR_TEST_SAMPLES, ODR_TEST_SAMPLES);

    OdrStats o = compute_odr_stats(ts_buf, ODR_TEST_SAMPLES);

    Serial.println("--- ODR RESULTS ---");
    Serial.printf("  Target:    %d Hz (dt = %d us)\n", BENCH_FREQ_HZ, BENCH_DT_MS * 1000);
    Serial.printf("  Actual:    %.2f Hz\n", o.actual_hz);
    Serial.printf("  Mean dt:   %.1f us\n", o.mean_dt_us);
    Serial.printf("  Std dt:    %.1f us\n", o.std_dt_us);
    Serial.printf("  Min dt:    %.0f us\n", o.min_dt_us);
    Serial.printf("  Max dt:    %.0f us\n", o.max_dt_us);
    Serial.printf("  Jitter:    %.2f %%\n", (o.std_dt_us / o.mean_dt_us) * 100.0f);

    Serial.println("--- DELTA TIMESTAMPS (us) ---");
    Serial.println("sample,delta_us");
    for (int i = 1; i < ODR_TEST_SAMPLES; ++i) {
        Serial.printf("%d,%llu\n", i, ts_buf[i] - ts_buf[i - 1]);
    }

    free(ts_buf);
    ts_buf = nullptr;
    Serial.println("=== END ODR JITTER TEST ===\n");
}

// -- Phase 4: Temperature Stability -------------------------------------------
static void run_thermal() {
    Serial.printf("\n=== PHASE 4: TEMPERATURE STABILITY (%ds @ %dHz) ===\n",
                  THERMAL_SECONDS, BENCH_FREQ_HZ);
    Serial.println("Streaming mode - no onboard buffering.");
    Serial.println("Device must be stationary. Monitor ambient temperature changes.");
    Serial.printf("Stable-window analysis: %ds windows with temp p2p <= %.2f C\n",
                  STABLE_TEMP_WINDOW_SECONDS, STABLE_TEMP_MAX_P2P_C);
    display_phase("P4: THERMAL", "Streaming 15 min...");

    Serial.println("--- CSV DATA ---");
    print_csv_header_imu();

    SensorSample* stable_ring = (SensorSample*)malloc(STABLE_TEMP_WINDOW_SAMPLES * sizeof(SensorSample));
    StableTempWindowSummary best_window;
    ThermalRegressionStats thermal_stats;
    SensorSample s;
    uint64_t first_ts_us = 0;

    for (int i = 0; i < THERMAL_SAMPLES; ++i) {
        read_imu(s);
        print_csv_row(s);

        if (i == 0) {
            first_ts_us = s.ts_us;
        }
        thermal_stats.update(s, (s.ts_us - first_ts_us) / 1000000.0f);

        if (stable_ring) {
            stable_ring[i % STABLE_TEMP_WINDOW_SAMPLES] = s;
            if (i + 1 >= STABLE_TEMP_WINDOW_SAMPLES) {
                update_stable_temp_window(stable_ring, i % STABLE_TEMP_WINDOW_SAMPLES, best_window);
            }
        }

        if ((i % 2500) == 0) display_progress(i, THERMAL_SAMPLES);
        delay(BENCH_DT_MS);
    }
    display_progress(THERMAL_SAMPLES, THERMAL_SAMPLES);

    print_thermal_regression_summary(thermal_stats);
    if (stable_ring) {
        print_stable_temp_window_summary(best_window);
        free(stable_ring);
    } else {
        Serial.println("[WARN] Stable-temperature window analysis skipped (malloc failed)");
    }

    Serial.println("=== END TEMPERATURE STABILITY ===\n");
}

// -- Phase 5: Allan Variance Data ---------------------------------------------
static void run_allan() {
    Serial.printf("\n=== PHASE 5: ALLAN VARIANCE DATA (%ds @ %dHz = %d samples) ===\n",
                  ALLAN_SECONDS, BENCH_FREQ_HZ, ALLAN_SAMPLES);
    Serial.println("Streaming mode - DO NOT TOUCH for 5 minutes.");
    display_phase("P5: ALLAN VAR", "DO NOT TOUCH 5min");

    Serial.println("--- CSV DATA ---");
    print_csv_header_imu();

    SensorSample s;
    for (int i = 0; i < ALLAN_SAMPLES; ++i) {
        read_imu(s);
        print_csv_row(s);
        if ((i % 2500) == 0) display_progress(i, ALLAN_SAMPLES);
        delay(BENCH_DT_MS);
    }
    display_progress(ALLAN_SAMPLES, ALLAN_SAMPLES);

    Serial.println("=== END ALLAN VARIANCE DATA ===\n");
}

// -- Setup & Loop -------------------------------------------------------------
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    // M5Unified leaves Wire uninitialised on AtomS3 - explicit init required.
    Wire.begin(38, 39);
    Wire.setTimeOut(10);

    Serial.begin(115200);
    delay(500);

    Serial.println("========================================");
    Serial.println("  AtomS3 MPU-6886 Sensor Bench v1.1");
    Serial.println("========================================");
    Serial.printf("  Chip: MPU-6886 @ I2C 0x%02X\n", IMU_ADDR);
    Serial.printf("  Target ODR: %d Hz\n", BENCH_FREQ_HZ);
    Serial.println("  FSR: +/-8g accel, +/-2000 dps gyro");
    Serial.println("  DLPF: ~20 Hz");
    Serial.println("========================================\n");

    i2c_write_reg(IMU_ADDR, REG_CONFIG, EXPECTED_DLPF);
    i2c_write_reg(IMU_ADDR, REG_ACCEL_CFG2, EXPECTED_DLPF);
    delay(10);

    display_phase("MPU-6886 Bench", "Auto-start in 1s");
    Serial.println("[INFO] Setup complete. Auto-start in 1 second.\n");
}

void loop() {
    M5.update();

    if (current_phase == PHASE_DONE) {
        delay(100);
        return;
    }

    if (!phase_started) {
        phase_started = true;
        Serial.printf("[INFO] Phase %d auto-start in 1 second.\n", current_phase);
        auto_phase_settle(phase_title(current_phase), "Settling 1s...");
    }

    switch (current_phase) {
        case PHASE_REGISTER_DUMP: run_register_dump(); break;
        case PHASE_AXIS_CHECK:    run_axis_check();    break;
        case PHASE_STATIC_NOISE:  run_static_noise();  break;
        case PHASE_ODR_JITTER:    run_odr_jitter();    break;
        case PHASE_THERMAL:       run_thermal();       break;
        case PHASE_ALLAN:         run_allan();         break;
        default: break;
    }

    current_phase = (TestPhase)(current_phase + 1);
    phase_started = false;

    if (current_phase == PHASE_DONE) {
        Serial.println("========================================");
        Serial.println("  ALL TESTS COMPLETE");
        Serial.println("========================================");
        display_phase("ALL DONE", "Tests complete!");
    } else {
        Serial.printf("[INFO] Phase %d queued. Auto-start in 1 second.\n", current_phase);
        display_phase("Next phase", "Auto-start in 1s");
    }
}
