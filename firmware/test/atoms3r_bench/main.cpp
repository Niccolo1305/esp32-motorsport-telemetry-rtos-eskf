// AtomS3R BMI270+BMM150 Sensor Bench — Bare-metal characterization firmware
// Runs standardized tests and outputs CSV via USB Serial for offline analysis.
// Includes BMM150 magnetometer characterization via BMI270 AUX I2C.
//
// Build:  pio run -e atoms3r_bench
// Flash:  pio run -e atoms3r_bench -t upload
// Log:    pio device monitor | tee atoms3r_results.txt
//
// NOTE: If M5Unified natively supports BMI270, the firmware uses M5.Imu
// for data readout (high-level path). Otherwise it falls back to direct
// register access (low-level path). Both paths are implemented.

#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include <esp32-hal-psram.h>
#include "bench_common.h"
#include "bmi270_config.h"

// ── BMI270 Constants ───────────────────────────────────────────────────────
static constexpr uint8_t BMI_ADDR_PRIMARY  = 0x69;
static constexpr uint8_t BMI_ADDR_FALLBACK = 0x68;
static uint8_t BMI_ADDR = BMI_ADDR_PRIMARY;

// Key registers
static constexpr uint8_t REG_CHIP_ID         = 0x00;
static constexpr uint8_t REG_ERR_REG         = 0x02;
static constexpr uint8_t REG_STATUS          = 0x03;
static constexpr uint8_t REG_DATA_0          = 0x04; // AUX (mag) data start
static constexpr uint8_t REG_DATA_8          = 0x0C; // Accel data start
static constexpr uint8_t REG_DATA_14         = 0x12; // Gyro data start
static constexpr uint8_t REG_INTERNAL_STATUS = 0x21;
static constexpr uint8_t REG_TEMP_0          = 0x22;
static constexpr uint8_t REG_ACC_CONF        = 0x40;
static constexpr uint8_t REG_ACC_RANGE       = 0x41;
static constexpr uint8_t REG_GYR_CONF        = 0x42;
static constexpr uint8_t REG_GYR_RANGE       = 0x43;
static constexpr uint8_t REG_AUX_CONF        = 0x44;
static constexpr uint8_t REG_AUX_DEV_ID      = 0x4B;
static constexpr uint8_t REG_AUX_IF_CONF     = 0x4C;
static constexpr uint8_t REG_AUX_RD_ADDR     = 0x4D;
static constexpr uint8_t REG_AUX_WR_ADDR     = 0x4E;
static constexpr uint8_t REG_AUX_WR_DATA     = 0x4F;
static constexpr uint8_t REG_INIT_CTRL       = 0x59;
static constexpr uint8_t REG_INIT_DATA       = 0x5E;
static constexpr uint8_t REG_PWR_CONF        = 0x7C;
static constexpr uint8_t REG_PWR_CTRL        = 0x7D;
static constexpr uint8_t REG_CMD             = 0x7E;

// Expected values
static constexpr uint8_t EXPECTED_CHIP_ID    = 0x24;
static constexpr uint8_t TARGET_ACC_CONF     = 0xA8; // ODR=100Hz, BWP=norm_avg4, perf mode
static constexpr uint8_t TARGET_ACC_RANGE    = 0x02; // ±8g
static constexpr uint8_t TARGET_GYR_CONF     = 0xA9; // ODR=100Hz, BWP=normal, perf+noise
static constexpr uint8_t TARGET_GYR_RANGE    = 0x00; // ±2000 dps

// Scaling (at ±8g and ±2000 dps — identical to MPU-6886)
static constexpr float ACCEL_SCALE = 1.0f / 4096.0f;   // LSB → g
static constexpr float GYRO_SCALE  = 1.0f / 16.384f;   // LSB → dps

// ── BMM150 Constants ───────────────────────────────────────────────────────
static constexpr uint8_t BMM150_I2C_ADDR     = 0x10;
static constexpr uint8_t BMM150_AUX_DEV_ID   = 0x20; // 0x10 << 1
static constexpr uint8_t BMM150_CHIP_ID_REG  = 0x40;
static constexpr uint8_t BMM150_EXPECTED_ID  = 0x32;
static constexpr uint8_t BMM150_PWR_CTRL     = 0x4B;
static constexpr uint8_t BMM150_OP_MODE      = 0x4C;
static constexpr uint8_t BMM150_REP_XY       = 0x51;
static constexpr uint8_t BMM150_REP_Z        = 0x52;
static constexpr uint8_t BMM150_DATA_X_LSB   = 0x42;

// ── Test Parameters ────────────────────────────────────────────────────────
static constexpr int STATIC_NOISE_SECONDS  = 300;
static constexpr int STATIC_NOISE_SAMPLES  = STATIC_NOISE_SECONDS * BENCH_FREQ_HZ;
static constexpr int ODR_TEST_SAMPLES      = 1000;
static constexpr int THERMAL_SECONDS       = 900;
static constexpr int THERMAL_SAMPLES       = THERMAL_SECONDS * BENCH_FREQ_HZ;
static constexpr int ALLAN_SECONDS         = 300;
static constexpr int ALLAN_SAMPLES         = ALLAN_SECONDS * BENCH_FREQ_HZ;
static constexpr int AXIS_CHECK_SECONDS    = 5;
static constexpr int AXIS_CHECK_SAMPLES    = AXIS_CHECK_SECONDS * BENCH_FREQ_HZ;
static constexpr int MAG_TEST_SECONDS      = 60;
static constexpr int MAG_FREQ_HZ           = 10;
static constexpr int MAG_DT_MS             = 1000 / MAG_FREQ_HZ;
static constexpr int MAG_TEST_SAMPLES      = MAG_TEST_SECONDS * MAG_FREQ_HZ;

// ── State Machine ──────────────────────────────────────────────────────────
enum TestPhase {
    PHASE_REGISTER_DUMP = 0,
    PHASE_AXIS_CHECK,
    PHASE_STATIC_NOISE,
    PHASE_ODR_JITTER,
    PHASE_THERMAL,
    PHASE_ALLAN,
    PHASE_CONFIG_TIMING,
    PHASE_MAG_TEST,
    PHASE_DONE,
    PHASE_COUNT
};

static TestPhase current_phase = PHASE_REGISTER_DUMP;
static bool phase_started = false;

// ── Runtime Flags ──────────────────────────────────────────────────────────
static bool use_m5_imu = false;   // true if M5Unified handles BMI270
static bool bmi270_ok  = false;   // true if low-level init succeeded
static bool bmm150_ok  = false;   // true if magnetometer init succeeded
static bool direct_i2c_ok = false; // true if Wire can reach BMI270 directly

// ── Buffers ────────────────────────────────────────────────────────────────
static SensorSample* sample_buf = nullptr;
static uint64_t*     ts_buf     = nullptr;
static constexpr uint32_t EXPECTED_PSRAM_BYTES = 8U * 1024U * 1024U;

static const char* phase_title(TestPhase phase) {
    switch (phase) {
        case PHASE_REGISTER_DUMP: return "P0: REG DUMP";
        case PHASE_AXIS_CHECK:    return "P1: AXIS CHECK";
        case PHASE_STATIC_NOISE:  return "P2: NOISE TEST";
        case PHASE_ODR_JITTER:    return "P3: ODR JITTER";
        case PHASE_THERMAL:       return "P4: THERMAL";
        case PHASE_ALLAN:         return "P5: ALLAN VAR";
        case PHASE_CONFIG_TIMING: return "P6: CONFIG TIME";
        case PHASE_MAG_TEST:      return "P7: MAG TEST";
        default:                  return "BENCH";
    }
}

struct I2cProbeCandidate {
    int sda;
    int scl;
    const char* label;
};

static bool begin_wire_and_find_bmi270(bool verbose = true) {
    static constexpr I2cProbeCandidate candidates[] = {
        {45, 0, "internal AtomS3R bus"},
        {2,  1, "external/Grove bus"},
    };
    static constexpr uint8_t addrs[] = { BMI_ADDR_PRIMARY, BMI_ADDR_FALLBACK };

    for (const auto& candidate : candidates) {
        Wire.end();
        Wire.begin(candidate.sda, candidate.scl);
        Wire.setTimeOut(10);
        delay(10);

        if (verbose) {
            Serial.printf("[I2C] Probing %s (SDA=%d, SCL=%d)\n",
                          candidate.label, candidate.sda, candidate.scl);
        }

        for (uint8_t addr : addrs) {
            uint8_t chip = i2c_read_reg(addr, REG_CHIP_ID);
            if (verbose) {
                Serial.printf("[BMI270] Probe addr 0x%02X -> CHIP_ID 0x%02X\n", addr, chip);
            }
            if (chip == EXPECTED_CHIP_ID) {
                BMI_ADDR = addr;
                return true;
            }
        }
    }
    return false;
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

static void report_psram_status() {
    Serial.println("[PSRAM] Runtime verification");

    if (!psramFound()) {
        Serial.println("[PSRAM] NOT DETECTED");
        Serial.println("[PSRAM] Check atoms3r_bench memory_type/psram_type and board wiring.");
        return;
    }

    uint32_t total_psram = ESP.getPsramSize();
    uint32_t free_psram  = ESP.getFreePsram();
    uint32_t min_psram   = ESP.getMinFreePsram();
    uint32_t max_alloc   = ESP.getMaxAllocPsram();

    Serial.printf("[PSRAM] Total:     %u bytes (%.2f MiB)\n",
                  total_psram, total_psram / 1048576.0f);
    Serial.printf("[PSRAM] Free:      %u bytes (%.2f MiB)\n",
                  free_psram, free_psram / 1048576.0f);
    Serial.printf("[PSRAM] Min free:  %u bytes (%.2f MiB)\n",
                  min_psram, min_psram / 1048576.0f);
    Serial.printf("[PSRAM] Max alloc: %u bytes (%.2f MiB)\n",
                  max_alloc, max_alloc / 1048576.0f);

    void* probe = ps_malloc(1024);
    if (probe != nullptr) {
        memset(probe, 0xA5, 1024);
        free(probe);
        Serial.println("[PSRAM] Probe alloc: OK (1 KiB)");
    } else {
        Serial.println("[PSRAM] Probe alloc: FAILED");
    }

    if (total_psram == EXPECTED_PSRAM_BYTES) {
        Serial.println("[PSRAM] Capacity check: OK (8 MiB expected)");
    } else {
        Serial.printf("[PSRAM] Capacity check: WARN (expected %u bytes / 8 MiB)\n",
                      EXPECTED_PSRAM_BYTES);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BMI270 Low-Level Functions
// ═══════════════════════════════════════════════════════════════════════════

static bool bmi270_upload_config() {
    // Soft reset
    i2c_write_reg(BMI_ADDR, REG_CMD, 0xB6);
    delay(2);

    // Disable advanced power save
    i2c_write_reg(BMI_ADDR, REG_PWR_CONF, 0x00);
    delayMicroseconds(500);

    // Prepare config upload
    i2c_write_reg(BMI_ADDR, REG_INIT_CTRL, 0x00);

    // Burst write config file to INIT_DATA (0x5E)
    // I2C max burst: 32 bytes per transaction on ESP32
    uint16_t offset = 0;
    while (offset < bmi270_config_actual_size) {
        uint16_t chunk = bmi270_config_actual_size - offset;
        if (chunk > 30) chunk = 30; // Wire buffer limit minus register address

        Wire.beginTransmission(BMI_ADDR);
        Wire.write(REG_INIT_DATA);
        for (uint16_t i = 0; i < chunk; i++) {
            Wire.write(pgm_read_byte(&bmi270_config_file[offset + i]));
        }
        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            Serial.printf("[ERROR] Config upload failed at offset %u, I2C err=%d\n", offset, err);
            return false;
        }
        offset += chunk;
    }

    // Complete upload
    i2c_write_reg(BMI_ADDR, REG_INIT_CTRL, 0x01);
    delay(25);

    // Verify
    uint8_t status = i2c_read_reg(BMI_ADDR, REG_INTERNAL_STATUS);
    if ((status & 0x01) != 0x01) {
        Serial.printf("[ERROR] INTERNAL_STATUS = 0x%02X (expected bit0=1)\n", status);
        return false;
    }
    Serial.printf("[OK] Config upload complete, INTERNAL_STATUS = 0x%02X\n", status);
    return true;
}

static bool bmi270_configure() {
    // FSR
    i2c_write_reg(BMI_ADDR, REG_ACC_RANGE, TARGET_ACC_RANGE);
    i2c_write_reg(BMI_ADDR, REG_GYR_RANGE, TARGET_GYR_RANGE);

    // ODR + filter bandwidth
    // Use 100 Hz ODR (0xA8 for accel, 0xA9 for gyro) to oversample at 50 Hz read rate
    // acc_conf: odr=100Hz(0x08), bwp=norm_avg4(0x02), perf_mode=1 → 0xA8
    // gyr_conf: odr=100Hz(0x09), bwp=normal(0x02), noise_perf=1, filter_perf=1 → 0xA9
    i2c_write_reg(BMI_ADDR, REG_ACC_CONF, TARGET_ACC_CONF);
    i2c_write_reg(BMI_ADDR, REG_GYR_CONF, TARGET_GYR_CONF);

    // Enable accel + gyro + temperature
    i2c_write_reg(BMI_ADDR, REG_PWR_CTRL, 0x0E);
    delay(10);

    // Verify
    uint8_t acc_conf  = i2c_read_reg(BMI_ADDR, REG_ACC_CONF);
    uint8_t acc_range = i2c_read_reg(BMI_ADDR, REG_ACC_RANGE);
    uint8_t gyr_conf  = i2c_read_reg(BMI_ADDR, REG_GYR_CONF);
    uint8_t gyr_range = i2c_read_reg(BMI_ADDR, REG_GYR_RANGE);
    uint8_t pwr_ctrl  = i2c_read_reg(BMI_ADDR, REG_PWR_CTRL);

    bool ok = (acc_range == TARGET_ACC_RANGE) &&
              (gyr_range == TARGET_GYR_RANGE) &&
              ((pwr_ctrl & 0x0E) == 0x0E);

    Serial.printf("[CONFIG] ACC_CONF=0x%02X ACC_RANGE=0x%02X GYR_CONF=0x%02X GYR_RANGE=0x%02X PWR_CTRL=0x%02X → %s\n",
                  acc_conf, acc_range, gyr_conf, gyr_range, pwr_ctrl, ok ? "OK" : "FAIL");
    return ok;
}

static void bmi270_read_raw(SensorSample& s) {
    // Burst read 12 bytes: DATA_8 (0x0C) through DATA_19 (0x17)
    // Layout: acc_x(2), acc_y(2), acc_z(2), gyr_x(2), gyr_y(2), gyr_z(2)
    uint8_t buf[12];
    Wire.beginTransmission(BMI_ADDR);
    Wire.write(REG_DATA_8);
    Wire.endTransmission(false);
    Wire.requestFrom(BMI_ADDR, (uint8_t)12);
    for (int i = 0; i < 12; i++) {
        buf[i] = Wire.available() ? Wire.read() : 0;
    }

    int16_t raw_ax = (int16_t)(buf[0]  | (buf[1]  << 8));
    int16_t raw_ay = (int16_t)(buf[2]  | (buf[3]  << 8));
    int16_t raw_az = (int16_t)(buf[4]  | (buf[5]  << 8));
    int16_t raw_gx = (int16_t)(buf[6]  | (buf[7]  << 8));
    int16_t raw_gy = (int16_t)(buf[8]  | (buf[9]  << 8));
    int16_t raw_gz = (int16_t)(buf[10] | (buf[11] << 8));

    s.ax = raw_ax * ACCEL_SCALE;
    s.ay = raw_ay * ACCEL_SCALE;
    s.az = raw_az * ACCEL_SCALE;
    s.gx = raw_gx * GYRO_SCALE;
    s.gy = raw_gy * GYRO_SCALE;
    s.gz = raw_gz * GYRO_SCALE;

    // Temperature: 2 bytes at 0x22-0x23
    uint8_t tbuf[2];
    Wire.beginTransmission(BMI_ADDR);
    Wire.write(REG_TEMP_0);
    Wire.endTransmission(false);
    Wire.requestFrom(BMI_ADDR, (uint8_t)2);
    tbuf[0] = Wire.available() ? Wire.read() : 0;
    tbuf[1] = Wire.available() ? Wire.read() : 0;
    int16_t raw_t = (int16_t)(tbuf[0] | (tbuf[1] << 8));
    s.temp_c = raw_t / 512.0f + 23.0f;

    s.ts_us = esp_timer_get_time();
}

// ── Helper: Read one IMU sample (auto-selects M5 or raw path) ─────────────
static void read_imu(SensorSample& s) {
    if (use_m5_imu) {
        M5.Imu.getAccelData(&s.ax, &s.ay, &s.az);
        M5.Imu.getGyroData(&s.gx, &s.gy, &s.gz);
        s.temp_c = 0.0f;
        M5.Imu.getTemp(&s.temp_c);
        s.ts_us = esp_timer_get_time();
    } else {
        bmi270_read_raw(s);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// BMM150 Magnetometer via AUX I2C
// ═══════════════════════════════════════════════════════════════════════════

// Write a single byte to BMM150 via BMI270 AUX manual mode
static void aux_write(uint8_t bmm_reg, uint8_t val) {
    i2c_write_reg(BMI_ADDR, REG_AUX_WR_DATA, val);
    i2c_write_reg(BMI_ADDR, REG_AUX_WR_ADDR, bmm_reg);
    delay(2); // Wait for AUX transaction to complete
}

// Read a single byte from BMM150 via BMI270 AUX manual mode
static uint8_t aux_read(uint8_t bmm_reg) {
    i2c_write_reg(BMI_ADDR, REG_AUX_RD_ADDR, bmm_reg);
    delay(2);
    return i2c_read_reg(BMI_ADDR, REG_DATA_0);
}

static bool bmm150_init() {
    if (!direct_i2c_ok) {
        Serial.println("[BMM150] Direct I2C access to BMI270 unavailable");
        return false;
    }

    Serial.println("[BMM150] Initializing via BMI270 AUX I2C...");

    // Enable AUX manual mode with burst length 8
    // AUX_IF_CONF: manual_en=1 (bit 7), burst=BL_8 (bits 1:0 = 0x03)
    i2c_write_reg(BMI_ADDR, REG_AUX_IF_CONF, 0x83);

    // Set BMM150 I2C address for AUX interface
    i2c_write_reg(BMI_ADDR, REG_AUX_DEV_ID, BMM150_AUX_DEV_ID);
    delay(1);

    // Wake BMM150 from suspend (power control bit 0 = 1)
    aux_write(BMM150_PWR_CTRL, 0x01);
    delay(5);

    // Read BMM150 chip ID
    uint8_t chip_id = aux_read(BMM150_CHIP_ID_REG);
    Serial.printf("[BMM150] CHIP_ID = 0x%02X (expected: 0x%02X) → %s\n",
                  chip_id, BMM150_EXPECTED_ID,
                  chip_id == BMM150_EXPECTED_ID ? "OK" : "FAIL");

    if (chip_id != BMM150_EXPECTED_ID) {
        Serial.println("[BMM150] Chip not detected — skipping magnetometer tests");
        return false;
    }

    // Configure Enhanced Regular preset
    aux_write(BMM150_REP_XY, 14);  // nXY = (1 + 2*14) = 29 repetitions
    aux_write(BMM150_REP_Z,  26);  // nZ  = (1 + 26)    = 27 repetitions

    // Set Normal mode (OP_MODE bits 2:1 = 0b00 = Normal, ODR bits 5:3 = 0b000 = 10Hz)
    aux_write(BMM150_OP_MODE, 0x00);
    delay(10);

    // Switch AUX to data mode (auto-read from BMM150 data registers)
    i2c_write_reg(BMI_ADDR, REG_AUX_RD_ADDR, BMM150_DATA_X_LSB);
    i2c_write_reg(BMI_ADDR, REG_AUX_IF_CONF, 0x03); // manual_en=0, burst=BL_8

    // Enable AUX in power control
    uint8_t pwr = i2c_read_reg(BMI_ADDR, REG_PWR_CTRL);
    i2c_write_reg(BMI_ADDR, REG_PWR_CTRL, pwr | 0x01); // aux_en = 1
    delay(10);

    Serial.println("[BMM150] Configured: Enhanced Regular preset, Normal mode, 10 Hz");
    return true;
}

static void bmm150_read(MagSample& ms) {
    // In auto-read mode, mag data appears at DATA_0..DATA_7
    uint8_t buf[8];
    Wire.beginTransmission(BMI_ADDR);
    Wire.write(REG_DATA_0);
    Wire.endTransmission(false);
    Wire.requestFrom(BMI_ADDR, (uint8_t)8);
    for (int i = 0; i < 8; i++) {
        buf[i] = Wire.available() ? Wire.read() : 0;
    }

    // BMM150 raw data format:
    // X: 13-bit signed (buf[0] bits 7:3 = LSB, buf[1] = MSB) → shift right 3
    // Y: 13-bit signed (buf[2] bits 7:3 = LSB, buf[3] = MSB) → shift right 3
    // Z: 15-bit signed (buf[4] bits 7:1 = LSB, buf[5] = MSB) → shift right 1
    // RHALL: 14-bit unsigned (buf[6] bits 7:2 = LSB, buf[7] = MSB) → shift right 2
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]) >> 3;
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]) >> 3;
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]) >> 1;
    uint16_t rhall = (uint16_t)((buf[7] << 8) | buf[6]) >> 2;

    // Raw LSB output (no compensation applied — needs offline trim data correction)
    // For bench purposes, raw values suffice for noise floor and axis orientation
    ms.mx = (float)raw_x;
    ms.my = (float)raw_y;
    ms.mz = (float)raw_z;
    ms.rhall = rhall;
    ms.ts_us = esp_timer_get_time();
}

static bool read_mag_sample(MagSample& ms) {
    ms.ts_us = esp_timer_get_time();
    if (use_m5_imu) {
        ms.rhall = 0;
        return M5.Imu.getMag(&ms.mx, &ms.my, &ms.mz);
    }
    if (!bmm150_ok) {
        return false;
    }
    bmm150_read(ms);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Test Phases
// ═══════════════════════════════════════════════════════════════════════════

// ── Phase 0: Register Dump ─────────────────────────────────────────────────
static void run_register_dump() {
    Serial.println("\n=== PHASE 0: REGISTER DUMP (BMI270) ===");
    display_phase("P0: REG DUMP");

    if (use_m5_imu) {
        Serial.println("  [INFO] M5Unified is handling BMI270 — limited register visibility.");
        Serial.printf("  M5.Imu type = %d\n", M5.Imu.getType());
    }

    if (!direct_i2c_ok) {
        Serial.println("  [WARN] Direct register access unavailable on Wire; skipping raw dump.");
        Serial.println("=== END REGISTER DUMP ===\n");
        return;
    }

    print_reg(BMI_ADDR, REG_CHIP_ID,         "CHIP_ID",         EXPECTED_CHIP_ID);
    print_reg_info(BMI_ADDR, REG_ERR_REG,    "ERR_REG");
    print_reg_info(BMI_ADDR, REG_STATUS,     "STATUS");
    print_reg_info(BMI_ADDR, REG_INTERNAL_STATUS, "INTERNAL_STATUS");
    print_reg(BMI_ADDR, REG_ACC_CONF,        "ACC_CONF",        TARGET_ACC_CONF);
    print_reg(BMI_ADDR, REG_ACC_RANGE,       "ACC_RANGE",       TARGET_ACC_RANGE);
    print_reg(BMI_ADDR, REG_GYR_CONF,        "GYR_CONF",        TARGET_GYR_CONF);
    print_reg(BMI_ADDR, REG_GYR_RANGE,       "GYR_RANGE",       TARGET_GYR_RANGE);
    print_reg_info(BMI_ADDR, REG_AUX_CONF,   "AUX_CONF");
    print_reg_info(BMI_ADDR, REG_PWR_CONF,   "PWR_CONF");
    print_reg_info(BMI_ADDR, REG_PWR_CTRL,   "PWR_CTRL");

    Serial.println("=== END REGISTER DUMP ===\n");
}

// ── Phase 1: Axis Orientation Check ────────────────────────────────────────
static void run_axis_check() {
    Serial.println("\n=== PHASE 1: AXIS ORIENTATION CHECK (5s) ===");
    Serial.println("Device must be flat, screen up, stationary.");
    display_phase("P1: AXIS CHECK", "Keep flat!");

    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    SensorSample s;

    for (int i = 0; i < AXIS_CHECK_SAMPLES; i++) {
        read_imu(s);
        sum_ax += s.ax; sum_ay += s.ay; sum_az += s.az;
        sum_gx += s.gx; sum_gy += s.gy; sum_gz += s.gz;
        display_progress(i + 1, AXIS_CHECK_SAMPLES);
        delay(BENCH_DT_MS);
    }

    float n = (float)AXIS_CHECK_SAMPLES;
    float mean_ax = sum_ax / n, mean_ay = sum_ay / n, mean_az = sum_az / n;
    float mean_gx = sum_gx / n, mean_gy = sum_gy / n, mean_gz = sum_gz / n;

    Serial.println("--- Axis Means (device flat, screen up) ---");
    Serial.printf("  Accel: ax=%.4f g  ay=%.4f g  az=%.4f g\n", mean_ax, mean_ay, mean_az);
    Serial.printf("  Gyro:  gx=%.3f dps  gy=%.3f dps  gz=%.3f dps\n", mean_gx, mean_gy, mean_gz);
    Serial.printf("  Expected: ax~0, ay~0, az~+1.0 g\n");

    bool az_ok = fabsf(mean_az - 1.0f) < 0.15f;
    bool ax_ok = fabsf(mean_ax) < 0.15f;
    bool ay_ok = fabsf(mean_ay) < 0.15f;
    Serial.printf("  Verdict: ax=%s  ay=%s  az=%s\n",
                  ax_ok ? "OK" : "WARN", ay_ok ? "OK" : "WARN", az_ok ? "OK" : "WARN");

    // If az is -1.0, the axis convention is inverted
    if (mean_az < -0.5f) {
        Serial.println("  *** WARNING: az is NEGATIVE (~-1g) — BMI270 Z-axis may be inverted vs MPU-6886 ***");
        Serial.println("  *** The Bmi270Provider will need sign inversion on az (and possibly gz) ***");
    }
    Serial.println("=== END AXIS CHECK ===\n");
}

// ── Phase 2: Static Noise Test ─────────────────────────────────────────────
static void run_static_noise() {
    Serial.printf("\n=== PHASE 2: STATIC NOISE TEST (%ds @ %dHz = %d samples) ===\n",
                  STATIC_NOISE_SECONDS, BENCH_FREQ_HZ, STATIC_NOISE_SAMPLES);
    Serial.println("Device must be stationary. Do not touch.");
    display_phase("P2: NOISE TEST", "Streaming 5 min...");

    Serial.println("--- CSV DATA ---");
    print_csv_header_imu();

    RunningImuStats stats;
    SensorSample s;
    for (int i = 0; i < STATIC_NOISE_SAMPLES; i++) {
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

// ── Phase 3: ODR & Jitter Test ─────────────────────────────────────────────
static void run_odr_jitter() {
    Serial.printf("\n=== PHASE 3: ODR & JITTER TEST (%d samples) ===\n", ODR_TEST_SAMPLES);
    display_phase("P3: ODR JITTER");

    ts_buf = (uint64_t*)malloc(ODR_TEST_SAMPLES * sizeof(uint64_t));
    if (!ts_buf) {
        Serial.println("[ERROR] Failed to allocate timestamp buffer!");
        return;
    }

    SensorSample s;
    for (int i = 0; i < ODR_TEST_SAMPLES; i++) {
        read_imu(s);
        ts_buf[i] = s.ts_us;
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
    for (int i = 1; i < ODR_TEST_SAMPLES; i++) {
        Serial.printf("%d,%llu\n", i, ts_buf[i] - ts_buf[i - 1]);
    }

    free(ts_buf);
    ts_buf = nullptr;
    Serial.println("=== END ODR JITTER TEST ===\n");
}

// ── Phase 4: Temperature Stability (streaming) ────────────────────────────
static void run_thermal() {
    Serial.printf("\n=== PHASE 4: TEMPERATURE STABILITY (%ds @ %dHz) ===\n",
                  THERMAL_SECONDS, BENCH_FREQ_HZ);
    Serial.println("Streaming mode. Device must be stationary.");
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
    for (int i = 0; i < THERMAL_SAMPLES; i++) {
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

// ── Phase 5: Allan Variance Data (streaming) ──────────────────────────────
static void run_allan() {
    Serial.printf("\n=== PHASE 5: ALLAN VARIANCE DATA (%ds @ %dHz = %d samples) ===\n",
                  ALLAN_SECONDS, BENCH_FREQ_HZ, ALLAN_SAMPLES);
    Serial.println("Streaming mode — DO NOT TOUCH for 5 minutes.");
    display_phase("P5: ALLAN VAR", "DO NOT TOUCH 5min");

    print_csv_header_imu();

    SensorSample s;
    for (int i = 0; i < ALLAN_SAMPLES; i++) {
        read_imu(s);
        print_csv_row(s);
        if ((i % 2500) == 0) display_progress(i, ALLAN_SAMPLES);
        delay(BENCH_DT_MS);
    }
    display_progress(ALLAN_SAMPLES, ALLAN_SAMPLES);

    Serial.println("=== END ALLAN VARIANCE DATA ===\n");
}

// ── Phase 6: Config Upload Timing ──────────────────────────────────────────
static void run_config_timing() {
    Serial.println("\n=== PHASE 6: BMI270 CONFIG UPLOAD TIMING ===");
    display_phase("P6: CONFIG TIME");

    if (!direct_i2c_ok) {
        Serial.println("  [SKIP] Direct register access unavailable on Wire.");
        Serial.println("=== END CONFIG UPLOAD TIMING (SKIPPED) ===\n");
        return;
    }

    if (use_m5_imu) {
        Serial.println("  [SKIP] M5Unified already owns the BMI270 init path.");
        Serial.println("  [SKIP] A manual soft-reset here only measures Wire timeout/recovery, not the real boot upload.");
        Serial.println("=== END CONFIG UPLOAD TIMING (SKIPPED) ===\n");
        return;
        Serial.println("  [INFO] M5Unified handles init — cannot measure upload timing.");
        Serial.println("  Performing a manual config upload for measurement only...");
    }

    // We perform a full soft-reset + config upload cycle and time it
    uint64_t t0 = esp_timer_get_time();

    // Soft reset
    i2c_write_reg(BMI_ADDR, REG_CMD, 0xB6);
    delay(2);
    i2c_write_reg(BMI_ADDR, REG_PWR_CONF, 0x00);
    delayMicroseconds(500);

    uint64_t t1 = esp_timer_get_time();

    // Upload config
    i2c_write_reg(BMI_ADDR, REG_INIT_CTRL, 0x00);

    uint16_t offset = 0;
    while (offset < bmi270_config_actual_size) {
        uint16_t chunk = bmi270_config_actual_size - offset;
        if (chunk > 30) chunk = 30;
        Wire.beginTransmission(BMI_ADDR);
        Wire.write(REG_INIT_DATA);
        for (uint16_t i = 0; i < chunk; i++) {
            Wire.write(pgm_read_byte(&bmi270_config_file[offset + i]));
        }
        Wire.endTransmission();
        offset += chunk;
    }

    i2c_write_reg(BMI_ADDR, REG_INIT_CTRL, 0x01);
    delay(25);

    uint64_t t2 = esp_timer_get_time();

    uint8_t status = i2c_read_reg(BMI_ADDR, REG_INTERNAL_STATUS);

    Serial.println("--- CONFIG UPLOAD TIMING ---");
    Serial.printf("  Config blob size:  %u bytes\n", bmi270_config_actual_size);
    Serial.printf("  Reset time:        %.1f ms\n", (t1 - t0) / 1000.0f);
    Serial.printf("  Upload + verify:   %.1f ms\n", (t2 - t1) / 1000.0f);
    Serial.printf("  Total init time:   %.1f ms\n", (t2 - t0) / 1000.0f);
    Serial.printf("  INTERNAL_STATUS:   0x%02X → %s\n", status,
                  (status & 0x01) ? "OK" : "FAIL");

    // Re-configure after reset
    if (!use_m5_imu) {
        bmi270_configure();
    }

    Serial.println("=== END CONFIG UPLOAD TIMING ===\n");
}

// ── Phase 7: BMM150 Magnetometer Test ──────────────────────────────────────
static void run_mag_test() {
    Serial.printf("\n=== PHASE 7: BMM150 MAGNETOMETER TEST (%ds @ %dHz = %d samples) ===\n",
                  MAG_TEST_SECONDS, MAG_FREQ_HZ, MAG_TEST_SAMPLES);
    display_phase("P7: MAG TEST");

    if (use_m5_imu) {
        Serial.println("[INFO] Using M5Unified magnetometer path (converted units, RHALL unavailable).");
        display_phase("P7: MAG TEST", "Collecting 60s @10Hz");
        Serial.println("--- CSV DATA ---");
        print_csv_header_mag();

        RunningAxisStats mag_stats[3];
        MagSample ms;
        for (int i = 0; i < MAG_TEST_SAMPLES; i++) {
            if (!read_mag_sample(ms)) {
                Serial.println("[SKIP] Magnetometer read failed.");
                Serial.println("=== END MAG TEST (SKIPPED) ===\n");
                return;
            }
            print_csv_row_mag(ms);
            mag_stats[0].update(ms.mx);
            mag_stats[1].update(ms.my);
            mag_stats[2].update(ms.mz);
            if ((i % 50) == 0) display_progress(i, MAG_TEST_SAMPLES);
            delay(MAG_DT_MS);
        }
        display_progress(MAG_TEST_SAMPLES, MAG_TEST_SAMPLES);

        Serial.println("=== MAGNETOMETER STATISTICS ===");
        print_axis_stats("mx", mag_stats[0].finalize());
        print_axis_stats("my", mag_stats[1].finalize());
        print_axis_stats("mz", mag_stats[2].finalize());
        Serial.println("=== END MAG TEST ===\n");
        return;
    }

    if (!direct_i2c_ok) {
        Serial.println("[SKIP] Direct register access unavailable on Wire.");
        Serial.println("=== END MAG TEST (SKIPPED) ===\n");
        return;
    }

    if (!bmm150_ok) {
        // Try to initialize now
        bmm150_ok = bmm150_init();
        if (!bmm150_ok) {
            Serial.println("[SKIP] BMM150 not available.");
            Serial.println("=== END MAG TEST (SKIPPED) ===\n");
            return;
        }
    }

    display_phase("P7: MAG TEST", "Collecting 60s @10Hz");

    // Allocate mag sample buffer (60s * 10Hz = 600 samples * ~20 bytes ≈ 12KB)
    MagSample* mag_buf = (MagSample*)malloc(MAG_TEST_SAMPLES * sizeof(MagSample));
    if (!mag_buf) {
        Serial.println("[ERROR] Failed to allocate mag buffer — streaming instead");
        // Fallback: stream to Serial
        print_csv_header_mag();
        MagSample ms;
        for (int i = 0; i < MAG_TEST_SAMPLES; i++) {
            bmm150_read(ms);
            print_csv_row_mag(ms);
            if ((i % 50) == 0) display_progress(i, MAG_TEST_SAMPLES);
            delay(MAG_DT_MS);
        }
    } else {
        for (int i = 0; i < MAG_TEST_SAMPLES; i++) {
            bmm150_read(mag_buf[i]);
            if ((i % 50) == 0) display_progress(i, MAG_TEST_SAMPLES);
            delay(MAG_DT_MS);
        }
        display_progress(MAG_TEST_SAMPLES, MAG_TEST_SAMPLES);

        // Output CSV
        Serial.println("--- CSV DATA ---");
        print_csv_header_mag();
        for (int i = 0; i < MAG_TEST_SAMPLES; i++) {
            print_csv_row_mag(mag_buf[i]);
        }

        // Compute stats per axis
        float* mbuf = (float*)malloc(MAG_TEST_SAMPLES * sizeof(float));
        if (mbuf) {
            Serial.println("=== MAGNETOMETER STATISTICS ===");
            const char* mnames[] = {"mx", "my", "mz"};
            for (int ch = 0; ch < 3; ch++) {
                for (int i = 0; i < MAG_TEST_SAMPLES; i++) {
                    switch (ch) {
                        case 0: mbuf[i] = mag_buf[i].mx; break;
                        case 1: mbuf[i] = mag_buf[i].my; break;
                        case 2: mbuf[i] = mag_buf[i].mz; break;
                    }
                }
                AxisStats st = compute_stats(mbuf, MAG_TEST_SAMPLES);
                print_axis_stats(mnames[ch], st);
            }
            free(mbuf);
        }
        free(mag_buf);
    }

    Serial.println("=== END MAG TEST ===\n");
}

// ════════════════════════════════════════════════════════════════════════════
// Setup & Loop
// ════════════════════════════════════════════════════════════════════════════

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Wire.setTimeOut(10);

    Serial.begin(115200);
    delay(500);

    Serial.println("========================================");
    Serial.println("  AtomS3R BMI270+BMM150 Sensor Bench v1.1");
    Serial.println("========================================");
    report_psram_status();
    Serial.println();

    // ── Detect IMU path ─────────────────────────────────────────────────
    // Check if M5Unified already initialized the BMI270
    uint8_t imu_type = M5.Imu.getType();
    Serial.printf("[DETECT] M5.Imu.getType() = %d\n", imu_type);

    if (imu_type != 0) {
        // M5Unified recognized an IMU — check if it's actually working
        float test_ax, test_ay, test_az;
        M5.Imu.getAccelData(&test_ax, &test_ay, &test_az);
        float norm = sqrtf(test_ax * test_ax + test_ay * test_ay + test_az * test_az);

        if (norm > 0.5f && norm < 1.5f) {
            use_m5_imu = true;
            Serial.printf("[OK] M5Unified IMU active (type=%d), accel norm=%.3f g → using M5.Imu path\n",
                          imu_type, norm);
        } else {
            Serial.printf("[WARN] M5.Imu returned suspicious norm=%.3f g — falling back to raw registers\n", norm);
        }
    }

    if (!use_m5_imu) {
        Serial.println("[INFO] M5Unified does not handle BMI270 — using low-level register path");
        direct_i2c_ok = begin_wire_and_find_bmi270();
        if (direct_i2c_ok) {
            Serial.printf("[BMI270] Found at I2C 0x%02X\n", BMI_ADDR);
            Serial.println("[BMI270] Uploading config file...");
            if (bmi270_upload_config() && bmi270_configure()) {
                bmi270_ok = true;
            }
        } else {
            Serial.println("[ERROR] BMI270 NOT DETECTED — tests will produce invalid data!");
        }
    } else {
        // Even on the M5Unified path, probe the raw register path for dump/timing phases.
        direct_i2c_ok = begin_wire_and_find_bmi270();
        if (direct_i2c_ok) {
            Serial.printf("[BMI270] Direct register path reachable at I2C 0x%02X\n", BMI_ADDR);
        } else {
            Serial.println("[WARN] M5.Imu works, but raw register access via Wire was not confirmed.");
        }
    }

    // Print summary
    Serial.printf("\n  IMU path:     %s\n", use_m5_imu ? "M5Unified" : (bmi270_ok ? "Raw registers" : "FAILED"));
    Serial.printf("  Chip:         BMI270 @ I2C 0x%02X\n", BMI_ADDR);
    Serial.printf("  Target ODR:   %d Hz\n", BENCH_FREQ_HZ);
    Serial.printf("  FSR:          +/-8g accel, +/-2000 dps gyro\n");
    Serial.println("========================================\n");

    display_phase("BMI270 Bench", "Ready — press btn");
    display_phase("BMI270 Bench", "Auto-start in 1s");
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
        case PHASE_CONFIG_TIMING: run_config_timing(); break;
        case PHASE_MAG_TEST:      run_mag_test();      break;
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
