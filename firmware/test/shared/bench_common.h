// bench_common.h - Shared utilities for AtomS3/AtomS3R sensor bench firmwares
// Statistics, CSV formatting, timing helpers.
#pragma once

#include <Arduino.h>
#include <M5Unified.h>
#include <math.h>

// -- Timing Constants ---------------------------------------------------------
static constexpr int BENCH_FREQ_HZ = 50;
static constexpr int BENCH_DT_MS = 1000 / BENCH_FREQ_HZ;  // 20 ms
static constexpr int PHASE_SETTLE_MS = 1000;
static constexpr int STABLE_TEMP_WINDOW_SECONDS = 75;
static constexpr int STABLE_TEMP_WINDOW_SAMPLES = STABLE_TEMP_WINDOW_SECONDS * BENCH_FREQ_HZ;
static constexpr float STABLE_TEMP_MAX_P2P_C = 0.10f;

// -- Data Structures ----------------------------------------------------------

struct SensorSample {
    uint64_t ts_us;
    float ax, ay, az;
    float gx, gy, gz;
    float temp_c;
};

struct MagSample {
    uint64_t ts_us;
    float mx, my, mz;
    uint16_t rhall;
};

struct AxisStats {
    float mean;
    float std_dev;
    float variance;
    float min_val;
    float max_val;
    float p2p;
};

struct RunningAxisStats {
    uint32_t count;
    double mean;
    double m2;
    float min_val;
    float max_val;

    RunningAxisStats() { reset(); }

    void reset() {
        count = 0;
        mean = 0.0;
        m2 = 0.0;
        min_val = 0.0f;
        max_val = 0.0f;
    }

    void update(float value) {
        if (count == 0) {
            min_val = value;
            max_val = value;
        } else {
            if (value < min_val) min_val = value;
            if (value > max_val) max_val = value;
        }

        ++count;
        double delta = value - mean;
        mean += delta / count;
        double delta2 = value - mean;
        m2 += delta * delta2;
    }

    AxisStats finalize() const {
        AxisStats out = {};
        if (count == 0) return out;
        out.mean = (float)mean;
        out.variance = (float)(m2 / count);
        out.std_dev = sqrtf(out.variance);
        out.min_val = min_val;
        out.max_val = max_val;
        out.p2p = max_val - min_val;
        return out;
    }
};

struct RunningImuStats {
    RunningAxisStats axis[7];
    RunningAxisStats accel_norm;
    RunningAxisStats gyro_norm;
    double motion_cross_sum[6][6];
    uint32_t motion_count;

    RunningImuStats() { reset(); }

    void reset() {
        motion_count = 0;
        accel_norm.reset();
        gyro_norm.reset();
        for (int i = 0; i < 7; ++i) {
            axis[i].reset();
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                motion_cross_sum[i][j] = 0.0;
            }
        }
    }

    void update(const SensorSample& s) {
        const float motion[6] = { s.ax, s.ay, s.az, s.gx, s.gy, s.gz };

        axis[0].update(s.ax);
        axis[1].update(s.ay);
        axis[2].update(s.az);
        axis[3].update(s.gx);
        axis[4].update(s.gy);
        axis[5].update(s.gz);
        axis[6].update(s.temp_c);

        float accel_mag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
        float gyro_mag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
        accel_norm.update(accel_mag);
        gyro_norm.update(gyro_mag);

        ++motion_count;
        for (int i = 0; i < 6; ++i) {
            for (int j = i; j < 6; ++j) {
                motion_cross_sum[i][j] += (double)motion[i] * motion[j];
            }
        }
    }
};

struct LinearRegressionAccumulator {
    uint32_t count;
    double sum_x;
    double sum_y;
    double sum_xx;
    double sum_xy;

    LinearRegressionAccumulator() { reset(); }

    void reset() {
        count = 0;
        sum_x = 0.0;
        sum_y = 0.0;
        sum_xx = 0.0;
        sum_xy = 0.0;
    }

    void update(float x, float y) {
        ++count;
        sum_x += x;
        sum_y += y;
        sum_xx += (double)x * x;
        sum_xy += (double)x * y;
    }

    bool valid() const { return count >= 2; }

    double slope() const {
        double denom = count * sum_xx - sum_x * sum_x;
        if (fabs(denom) < 1e-12) return 0.0;
        return (count * sum_xy - sum_x * sum_y) / denom;
    }

    double intercept() const {
        if (count == 0) return 0.0;
        return (sum_y - slope() * sum_x) / count;
    }
};

struct ThermalRegressionStats {
    RunningAxisStats temp_stats;
    LinearRegressionAccumulator axis_vs_temp[6];
    LinearRegressionAccumulator axis_vs_time[6];
    bool has_first;
    float first_temp_c;
    float last_temp_c;
    float first_elapsed_s;
    float last_elapsed_s;

    ThermalRegressionStats() { reset(); }

    void reset() {
        has_first = false;
        first_temp_c = 0.0f;
        last_temp_c = 0.0f;
        first_elapsed_s = 0.0f;
        last_elapsed_s = 0.0f;
        temp_stats.reset();
        for (int i = 0; i < 6; ++i) {
            axis_vs_temp[i].reset();
            axis_vs_time[i].reset();
        }
    }

    void update(const SensorSample& s, float elapsed_s) {
        const float motion[6] = { s.ax, s.ay, s.az, s.gx, s.gy, s.gz };

        if (!has_first) {
            has_first = true;
            first_temp_c = s.temp_c;
            first_elapsed_s = elapsed_s;
        }
        last_temp_c = s.temp_c;
        last_elapsed_s = elapsed_s;
        temp_stats.update(s.temp_c);

        for (int i = 0; i < 6; ++i) {
            axis_vs_temp[i].update(s.temp_c, motion[i]);
            axis_vs_time[i].update(elapsed_s, motion[i]);
        }
    }
};

struct StableTempWindowSummary {
    bool found;
    float temp_min_c;
    float temp_max_c;
    float temp_p2p_c;
    uint64_t start_ts_us;
    uint64_t end_ts_us;
    AxisStats axis[7];

    StableTempWindowSummary() { reset(); }

    void reset() {
        found = false;
        temp_min_c = 0.0f;
        temp_max_c = 0.0f;
        temp_p2p_c = 0.0f;
        start_ts_us = 0;
        end_ts_us = 0;
        for (int i = 0; i < 7; ++i) {
            axis[i] = AxisStats();
        }
    }
};

// -- Generic Helpers ----------------------------------------------------------

inline const char* imu_axis_name(int index) {
    static const char* names[] = { "ax", "ay", "az", "gx", "gy", "gz", "temp_c" };
    return names[index];
}

inline float get_sample_channel(const SensorSample& s, int index) {
    switch (index) {
        case 0: return s.ax;
        case 1: return s.ay;
        case 2: return s.az;
        case 3: return s.gx;
        case 4: return s.gy;
        case 5: return s.gz;
        case 6: return s.temp_c;
        default: return 0.0f;
    }
}

// -- Statistics ---------------------------------------------------------------

inline AxisStats compute_stats(const float* data, int n) {
    AxisStats s = {};
    if (n <= 0) return s;

    s.min_val = data[0];
    s.max_val = data[0];
    double sum = 0.0;

    for (int i = 0; i < n; i++) {
        sum += data[i];
        if (data[i] < s.min_val) s.min_val = data[i];
        if (data[i] > s.max_val) s.max_val = data[i];
    }
    s.mean = (float)(sum / n);

    double sq_sum = 0.0;
    for (int i = 0; i < n; i++) {
        double d = data[i] - s.mean;
        sq_sum += d * d;
    }
    s.variance = (float)(sq_sum / n);
    s.std_dev = sqrtf(s.variance);
    s.p2p = s.max_val - s.min_val;
    return s;
}

inline void compute_window_imu_stats(const SensorSample* samples,
                                     int capacity,
                                     int start_index,
                                     int count,
                                     AxisStats out[7]) {
    RunningAxisStats running[7];
    for (int i = 0; i < count; ++i) {
        const SensorSample& s = samples[(start_index + i) % capacity];
        for (int ch = 0; ch < 7; ++ch) {
            running[ch].update(get_sample_channel(s, ch));
        }
    }
    for (int ch = 0; ch < 7; ++ch) {
        out[ch] = running[ch].finalize();
    }
}

inline float compute_motion_correlation(const RunningImuStats& stats, int i, int j) {
    if (stats.motion_count < 2) return 0.0f;

    AxisStats si = stats.axis[i].finalize();
    AxisStats sj = stats.axis[j].finalize();
    if (si.std_dev <= 0.0f || sj.std_dev <= 0.0f) return 0.0f;

    double cross = (i <= j) ? stats.motion_cross_sum[i][j] : stats.motion_cross_sum[j][i];
    double cov = cross / stats.motion_count - (double)si.mean * sj.mean;
    return (float)(cov / ((double)si.std_dev * sj.std_dev));
}

// -- CSV Formatting -----------------------------------------------------------

inline void print_csv_header_imu() {
    Serial.println("timestamp_us,ax,ay,az,gx,gy,gz,temp_c");
}

inline void print_csv_row(const SensorSample& s) {
    Serial.printf("%llu,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.2f\n",
                  s.ts_us, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp_c);
}

inline void print_csv_header_mag() {
    Serial.println("timestamp_us,mx,my,mz,rhall");
}

inline void print_csv_row_mag(const MagSample& s) {
    Serial.printf("%llu,%.2f,%.2f,%.2f,%u\n",
                  s.ts_us, s.mx, s.my, s.mz, s.rhall);
}

// -- Statistics Reporting -----------------------------------------------------

inline void print_axis_stats(const char* name, const AxisStats& s) {
    Serial.printf("  %s: mean=%.6f  std=%.6f  var=%.6f  min=%.6f  max=%.6f  p2p=%.6f\n",
                  name, s.mean, s.std_dev, s.variance, s.min_val, s.max_val, s.p2p);
}

inline void print_full_imu_stats(const SensorSample* samples, int n) {
    float* buf = (float*)malloc(n * sizeof(float));
    if (!buf) {
        Serial.println("[ERROR] malloc failed for stats");
        return;
    }

    Serial.println("=== STATISTICS ===");
    for (int ch = 0; ch < 7; ch++) {
        for (int i = 0; i < n; i++) {
            buf[i] = get_sample_channel(samples[i], ch);
        }
        AxisStats st = compute_stats(buf, n);
        print_axis_stats(imu_axis_name(ch), st);
    }
    free(buf);
}

inline void print_running_imu_stats(const RunningImuStats& stats,
                                    const char* heading = "=== STATISTICS ===") {
    Serial.println(heading);
    for (int ch = 0; ch < 7; ++ch) {
        print_axis_stats(imu_axis_name(ch), stats.axis[ch].finalize());
    }
    Serial.println("=== DERIVED NORMS ===");
    print_axis_stats("accel_norm", stats.accel_norm.finalize());
    print_axis_stats("gyro_norm", stats.gyro_norm.finalize());
}

inline void print_motion_correlation_matrix(const RunningImuStats& stats) {
    static const char* names[] = { "ax", "ay", "az", "gx", "gy", "gz" };

    Serial.println("=== MOTION CORRELATION (Pearson r) ===");
    Serial.println("        ax      ay      az      gx      gy      gz");
    for (int row = 0; row < 6; ++row) {
        Serial.printf("  %s ", names[row]);
        for (int col = 0; col < 6; ++col) {
            Serial.printf(" %7.3f", compute_motion_correlation(stats, row, col));
        }
        Serial.println();
    }
}

inline void print_thermal_regression_summary(const ThermalRegressionStats& stats) {
    static const char* names[] = { "ax", "ay", "az", "gx", "gy", "gz" };

    AxisStats temp = stats.temp_stats.finalize();
    Serial.println("=== THERMAL REGRESSION ===");
    Serial.printf("  Duration: %.1f s\n", stats.last_elapsed_s - stats.first_elapsed_s);
    Serial.printf("  Temp: start=%.2f C  end=%.2f C  delta=%.2f C  mean=%.2f C  p2p=%.3f C\n",
                  stats.first_temp_c, stats.last_temp_c,
                  stats.last_temp_c - stats.first_temp_c, temp.mean, temp.p2p);

    Serial.println("--- Axis drift vs temperature ---");
    for (int i = 0; i < 6; ++i) {
        double slope = stats.axis_vs_temp[i].slope();
        if (i < 3) {
            Serial.printf("  %s: %.6f g/C\n", names[i], slope);
        } else {
            Serial.printf("  %s: %.6f dps/C\n", names[i], slope);
        }
    }

    Serial.println("--- Axis drift vs time ---");
    for (int i = 0; i < 6; ++i) {
        double slope_per_min = stats.axis_vs_time[i].slope() * 60.0;
        if (i < 3) {
            Serial.printf("  %s: %.3f mg/min\n", names[i], slope_per_min * 1000.0);
        } else {
            Serial.printf("  %s: %.5f dps/min\n", names[i], slope_per_min);
        }
    }
}

inline void print_stable_temp_window_summary(const StableTempWindowSummary& summary) {
    if (!summary.found) {
        Serial.printf("=== STABLE TEMP WINDOW ===\n");
        Serial.printf("  No %ds window found with temp p2p <= %.2f C\n",
                      STABLE_TEMP_WINDOW_SECONDS, STABLE_TEMP_MAX_P2P_C);
        return;
    }

    Serial.println("=== STABLE TEMP WINDOW ===");
    Serial.printf("  Duration: %d s\n", STABLE_TEMP_WINDOW_SECONDS);
    Serial.printf("  Start ts: %llu us\n", summary.start_ts_us);
    Serial.printf("  End ts:   %llu us\n", summary.end_ts_us);
    Serial.printf("  Temp: min=%.3f C  max=%.3f C  p2p=%.3f C\n",
                  summary.temp_min_c, summary.temp_max_c, summary.temp_p2p_c);
    for (int ch = 0; ch < 7; ++ch) {
        print_axis_stats(imu_axis_name(ch), summary.axis[ch]);
    }
}

// -- ODR / Jitter Measurement -------------------------------------------------

struct OdrStats {
    float mean_dt_us;
    float std_dt_us;
    float min_dt_us;
    float max_dt_us;
    float actual_hz;
};

inline OdrStats compute_odr_stats(const uint64_t* timestamps, int n) {
    OdrStats o = {};
    if (n < 2) return o;

    int nd = n - 1;
    float* deltas = (float*)malloc(nd * sizeof(float));
    if (!deltas) return o;

    for (int i = 0; i < nd; i++) {
        deltas[i] = (float)(timestamps[i + 1] - timestamps[i]);
    }
    AxisStats s = compute_stats(deltas, nd);
    o.mean_dt_us = s.mean;
    o.std_dt_us = s.std_dev;
    o.min_dt_us = s.min_val;
    o.max_dt_us = s.max_val;
    o.actual_hz = 1e6f / s.mean;
    free(deltas);
    return o;
}

// -- Display Helpers ----------------------------------------------------------

inline void display_phase(const char* title, const char* subtitle = nullptr) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(2, 4);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(title);
    if (subtitle) {
        M5.Lcd.setCursor(2, 20);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(subtitle);
    }
}

inline void display_progress(int current, int total) {
    M5.Lcd.fillRect(2, 50, 124, 16, BLACK);
    M5.Lcd.setCursor(2, 50);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.printf("%d / %d", current, total);

    int bar_w = (current * 120) / total;
    M5.Lcd.fillRect(2, 70, bar_w, 6, GREEN);
    M5.Lcd.fillRect(2 + bar_w, 70, 120 - bar_w, 6, DARKGREY);
}

inline void auto_phase_settle(const char* title, const char* subtitle = "Settling 1s...") {
    display_phase(title, subtitle);
    uint32_t start_ms = millis();
    while ((uint32_t)(millis() - start_ms) < (uint32_t)PHASE_SETTLE_MS) {
        M5.update();
        delay(20);
    }
}

// -- I2C Register Helpers -----------------------------------------------------

inline uint8_t i2c_read_reg(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

inline void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

inline void print_reg(uint8_t addr, uint8_t reg, const char* name, uint8_t expected) {
    uint8_t val = i2c_read_reg(addr, reg);
    const char* status = (val == expected) ? "OK" : "FAIL";
    Serial.printf("  REG 0x%02X (%s) = 0x%02X (expected: 0x%02X) [%s]\n",
                  reg, name, val, expected, status);
}

inline void print_reg_info(uint8_t addr, uint8_t reg, const char* name) {
    uint8_t val = i2c_read_reg(addr, reg);
    Serial.printf("  REG 0x%02X (%s) = 0x%02X\n", reg, name, val);
}
