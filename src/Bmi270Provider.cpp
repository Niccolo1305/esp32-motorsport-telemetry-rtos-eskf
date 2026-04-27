// SPDX-License-Identifier: GPL-3.0-or-later
#include "Bmi270Provider.h"

#ifdef USE_BMI270

#include <Arduino.h>
#include <esp_timer.h>
#include <string.h>

namespace {

constexpr uint8_t kBmm150Addr = BMM150_DEFAULT_I2C_ADDRESS;
constexpr uint32_t kBoschReadWriteLen = 32;
constexpr uint8_t kBmi270ChipId = BMI270_CHIP_ID;
constexpr uint8_t kBmi270AccRangeAddr = 0x41;
constexpr uint8_t kBmi270GyrRangeAddr = 0x43;

bool trim_data_present(const bmm150_trim_registers& trim) {
    return trim.dig_x1 != 0 || trim.dig_y1 != 0 || trim.dig_z4 != 0 || trim.dig_xyz1 != 0;
}

float bmi_temp_to_celsius(uint16_t raw_temp) {
    return (static_cast<float>(static_cast<int16_t>(raw_temp)) / 512.0f) + 23.0f;
}

uint8_t clamp_u8(uint16_t value) {
    return (value > 255U) ? 255U : static_cast<uint8_t>(value);
}

bool bmm150_raw_overflow(int16_t raw_x, int16_t raw_y, int16_t raw_z) {
    return (raw_x == BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) ||
           (raw_y == BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) ||
           (raw_z == BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL);
}

bool bmi2_failed(int8_t rslt) {
    return rslt < BMI2_OK;
}

} // namespace

void Bmi270Provider::begin() {
    _config_ok = false;
    _mag_ok = false;
    _have_last_sample = false;
    _last_sample = {};

    if (!M5.In_I2C.isEnabled()) {
        Serial.println("[IMU] ERROR: M5.In_I2C is not enabled after M5.begin().");
        return;
    }

    if (!probe_bmi_address()) {
        Serial.println("[IMU] ERROR: BMI270 not found on AtomS3R internal I2C bus.");
        return;
    }

    const bool bmi_ok = init_bmi270();
    const bool mag_ok = bmi_ok && init_bmm150();
    const bool fifo_ok = mag_ok && init_fifo();

    cache_register_report();
    log_bringup_report();
    _config_ok = bmi_ok && mag_ok && fifo_ok;
}

void Bmi270Provider::update(ImuRawData& out) {
    out = {};
    out.timestamp_us = esp_timer_get_time();

    if (!_config_ok) {
        return;
    }

    const bool have_realtime = read_fifo_sample(out);

    float temp_c = 0.0f;
    if (read_temperature(temp_c)) {
        out.temp_c = temp_c;
    } else if (_have_last_sample) {
        out.temp_c = _last_sample.temp_c;
    }

    out.timestamp_us = esp_timer_get_time();

    if (!have_realtime) {
        if (_have_last_sample) {
            ImuRawData stale = _last_sample;
            stale.timestamp_us = out.timestamp_us;
            stale.temp_c = out.temp_c;
            stale.imu_sample_fresh = false;
            stale.mag_sample_fresh = false;
            stale.fifo_frames_drained = out.fifo_frames_drained;
            stale.fifo_backlog = out.fifo_backlog;
            stale.fifo_overrun = out.fifo_overrun;
            out = stale;
        }

        return;
    }

    _last_sample = out;
    _have_last_sample = true;
}

bool Bmi270Provider::probe_bmi_address() {
    static constexpr uint8_t candidates[] = { kPrimaryAddr, kFallbackAddr };

    for (uint8_t addr : candidates) {
        uint8_t chip_id = 0;
        if (M5.In_I2C.readRegister(addr, BMI2_CHIP_ID_ADDR, &chip_id, 1, kI2CFreq) && chip_id == kBmi270ChipId) {
            _bmi_addr = addr;
            _chip_id = chip_id;
            return true;
        }
    }

    return false;
}

bool Bmi270Provider::init_bmi270() {
    _bmi = {};
    _bmi.intf = BMI2_I2C_INTF;
    _bmi.intf_ptr = this;
    _bmi.read = bmi_read;
    _bmi.write = bmi_write;
    _bmi.delay_us = bmi_delay_us;
    _bmi.read_write_len = kBoschReadWriteLen;
    _bmi.config_file_ptr = nullptr;

    int8_t rslt = bmi270_init(&_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi270_init failed (%d)\n", rslt);
        return false;
    }

    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_adv_power_save(DISABLE) failed (%d)\n", rslt);
        return false;
    }

    bmi2_sens_config cfg[3] = {};
    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;
    cfg[2].type = BMI2_AUX;
    rslt = bmi2_get_sensor_config(cfg, 3, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_get_sensor_config failed (%d)\n", rslt);
        return false;
    }

    // DATA-register filtered path: ODR 50 Hz + normal filter mode yields the
    // BMI270 native post-LPF signal (~22 Hz accel, ~20 Hz gyro). The FIFO raw
    // path is selected separately in init_fifo().
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_50HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_8G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    // DATA-register filtered path for gyro. For FIFO unfiltered data the
    // pre-filtered gyro scale follows ois_range, kept at +/-2000 dps.
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_50HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    cfg[2].cfg.aux.aux_en = BMI2_ENABLE;
    cfg[2].cfg.aux.manual_en = BMI2_ENABLE;
    cfg[2].cfg.aux.fcu_write_en = BMI2_ENABLE;
    cfg[2].cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    cfg[2].cfg.aux.aux_rd_burst = BMI2_AUX_READ_LEN_3;
    cfg[2].cfg.aux.odr = BMI2_AUX_ODR_25HZ;
    cfg[2].cfg.aux.offset = 0;
    cfg[2].cfg.aux.i2c_device_addr = kBmm150Addr;
    cfg[2].cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;

    rslt = bmi2_set_sensor_config(cfg, 3, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_sensor_config failed (%d)\n", rslt);
        return false;
    }

    uint8_t sensor_list[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_AUX };
    rslt = bmi2_sensor_enable(sensor_list, 3, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_sensor_enable(acc+gyr+aux) failed (%d)\n", rslt);
        return false;
    }

    return true;
}

bool Bmi270Provider::init_bmm150() {
    _bmm = {};
    _bmm.intf = BMM150_I2C_INTF;
    _bmm.intf_ptr = this;
    _bmm.read = bmm_read;
    _bmm.write = bmm_write;
    _bmm.delay_us = bmm_delay_us;

    int8_t rslt = bmm150_init(&_bmm);
    if (rslt != BMM150_OK) {
        Serial.printf("[IMU] ERROR: bmm150_init failed (%d)\n", rslt);
        return false;
    }

    bmm150_settings settings = {};
    settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    settings.pwr_mode = BMM150_POWERMODE_SLEEP;
    rslt = bmm150_set_presetmode(&settings, &_bmm);
    if (rslt != BMM150_OK) {
        Serial.printf("[IMU] ERROR: bmm150_set_presetmode failed (%d)\n", rslt);
        return false;
    }

    settings.data_rate = BMM150_DATA_RATE_25HZ;
    rslt = bmm150_set_sensor_settings(BMM150_SEL_DATA_RATE, &settings, &_bmm);
    if (rslt != BMM150_OK) {
        Serial.printf("[IMU] ERROR: bmm150_set_sensor_settings(data_rate) failed (%d)\n", rslt);
        return false;
    }

    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, &_bmm);
    if (rslt != BMM150_OK) {
        Serial.printf("[IMU] ERROR: bmm150_set_op_mode(NORMAL) failed (%d)\n", rslt);
        return false;
    }

    bmi2_sens_config aux_cfg = {};
    aux_cfg.type = BMI2_AUX;
    aux_cfg.cfg.aux.aux_en = BMI2_ENABLE;
    aux_cfg.cfg.aux.manual_en = BMI2_DISABLE;
    aux_cfg.cfg.aux.fcu_write_en = BMI2_ENABLE;
    aux_cfg.cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;
    aux_cfg.cfg.aux.aux_rd_burst = BMI2_AUX_READ_LEN_3;
    aux_cfg.cfg.aux.odr = BMI2_AUX_ODR_25HZ;
    aux_cfg.cfg.aux.offset = 0;
    aux_cfg.cfg.aux.i2c_device_addr = kBmm150Addr;
    aux_cfg.cfg.aux.read_addr = BMM150_REG_DATA_X_LSB;

    rslt = bmi2_set_sensor_config(&aux_cfg, 1, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_sensor_config(aux data mode) failed (%d)\n", rslt);
        return false;
    }

    _mag_ok = trim_data_present(_bmm.trim_data);
    if (!_mag_ok) {
        Serial.println("[IMU] ERROR: BMM150 trim data missing.");
    }

    return _mag_ok;
}

bool Bmi270Provider::init_fifo() {
    _fifo = {};
    _fifo.data = _fifo_buffer;

    int8_t rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN | BMI2_FIFO_HEADER_EN, BMI2_DISABLE, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_fifo_config(DISABLE) failed (%d)\n", rslt);
        return false;
    }

    rslt = bmi2_set_fifo_wm(0, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_fifo_wm failed (%d)\n", rslt);
        return false;
    }

    // Current schema contract:
    //   bmi_post_lpf20_prepipe_* = BMI270 FIFO filtered/post-LPF LSB at the
    //   50 Hz task rate. These channels stay pre remap/calibration/Madgwick and
    //   represent the acquisition truth used by the current pipeline.
    rslt = bmi2_set_fifo_filter_data(BMI2_ACCEL, BMI2_FIFO_FILTERED_DATA, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_fifo_filter_data(acc filtered) failed (%d)\n", rslt);
        return false;
    }

    rslt = bmi2_set_fifo_filter_data(BMI2_GYRO, BMI2_FIFO_FILTERED_DATA, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_fifo_filter_data(gyr filtered) failed (%d)\n", rslt);
        return false;
    }

    rslt = bmi2_set_fifo_down_sample(BMI2_ACCEL, 0, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_fifo_down_sample(acc=0) failed (%d)\n",
                      rslt);
        return false;
    }

    rslt = bmi2_set_fifo_down_sample(BMI2_GYRO, 0, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_fifo_down_sample(gyr=0) failed (%d)\n",
                      rslt);
        return false;
    }

    uint8_t acc_fifo_filter = 0xFF;
    uint8_t gyr_fifo_filter = 0xFF;
    uint8_t acc_fifo_downsample = 0xFF;
    uint8_t gyr_fifo_downsample = 0xFF;
    rslt = bmi2_get_fifo_filter_data(BMI2_ACCEL, &acc_fifo_filter, &_bmi);
    rslt |= bmi2_get_fifo_filter_data(BMI2_GYRO, &gyr_fifo_filter, &_bmi);
    rslt |= bmi2_get_fifo_down_sample(BMI2_ACCEL, &acc_fifo_downsample, &_bmi);
    rslt |= bmi2_get_fifo_down_sample(BMI2_GYRO, &gyr_fifo_downsample, &_bmi);
    if (rslt != BMI2_OK ||
        acc_fifo_filter != BMI2_FIFO_FILTERED_DATA ||
        gyr_fifo_filter != BMI2_FIFO_FILTERED_DATA ||
        acc_fifo_downsample != 0 ||
        gyr_fifo_downsample != 0) {
        Serial.printf("[IMU] ERROR: FIFO filtered readback mismatch acc_filt=%u gyr_filt=%u acc_down=%u gyr_down=%u rslt=%d\n",
                      acc_fifo_filter,
                      gyr_fifo_filter,
                      acc_fifo_downsample,
                      gyr_fifo_downsample,
                      rslt);
        return false;
    }

    Serial.printf("[IMU] FIFO filtered mode: acc_filt=%u gyr_filt=%u acc_down=%u gyr_down=%u\n",
                  acc_fifo_filter,
                  gyr_fifo_filter,
                  acc_fifo_downsample,
                  gyr_fifo_downsample);

    // Previous static-characterization path:
    //   true raw FIFO acquisition at 50 Hz via pre-filtered FIFO source
    //   downsampled from 1600 Hz accel / 6400 Hz gyro.
    //
    // rslt = bmi2_set_fifo_filter_data(BMI2_ACCEL, BMI2_FIFO_UNFILTERED_DATA, &_bmi);
    // rslt |= bmi2_set_fifo_filter_data(BMI2_GYRO, BMI2_FIFO_UNFILTERED_DATA, &_bmi);
    // rslt |= bmi2_set_fifo_down_sample(BMI2_ACCEL, kRawFifoAccDownsample, &_bmi);
    // rslt |= bmi2_set_fifo_down_sample(BMI2_GYRO, kRawFifoGyrDownsample, &_bmi);

    rslt = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN | BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN | BMI2_FIFO_AUX_EN,
                                BMI2_ENABLE,
                                &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] ERROR: bmi2_set_fifo_config(ENABLE) failed (%d)\n", rslt);
        return false;
    }

    return flush_fifo("post-init");
}

bool Bmi270Provider::read_fifo_sample(ImuRawData& out) {
    uint16_t fifo_length = 0;
    int8_t rslt = bmi2_get_fifo_length(&fifo_length, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] WARN: bmi2_get_fifo_length failed (%d)\n", rslt);
        out.fifo_overrun = true;
        return false;
    }

    out.fifo_backlog = clamp_u8(fifo_length);
    if (fifo_length == 0) {
        return false;
    }

    _fifo.length = (fifo_length > kFifoBufferSize) ? kFifoBufferSize : fifo_length;
    _fifo.acc_byte_start_idx = 0;
    _fifo.gyr_byte_start_idx = 0;
    _fifo.aux_byte_start_idx = 0;
    _fifo.act_recog_byte_start_idx = 0;
    _fifo.skipped_frame_count = 0;
    _fifo.sensor_time = 0;

    rslt = bmi2_read_fifo_data(&_fifo, &_bmi);
    const bool partial_read = (rslt == BMI2_W_PARTIAL_READ);
    if (bmi2_failed(rslt)) {
        Serial.printf("[IMU] WARN: bmi2_read_fifo_data failed (%d)\n", rslt);
        out.fifo_overrun = true;
        flush_fifo("read error");
        return false;
    }

    bmi2_fifo_frame acc_fifo = _fifo;
    bmi2_fifo_frame gyr_fifo = _fifo;
    bmi2_fifo_frame aux_fifo = _fifo;

    uint16_t acc_len = kMaxParsedFrames;
    uint16_t gyr_len = kMaxParsedFrames;
    uint16_t aux_len = kMaxParsedFrames;
    const int8_t acc_rslt = bmi2_extract_accel(_acc_frames, &acc_len, &acc_fifo, &_bmi);
    const int8_t gyr_rslt = bmi2_extract_gyro(_gyr_frames, &gyr_len, &gyr_fifo, &_bmi);
    const int8_t aux_rslt = bmi2_extract_aux(_aux_frames, &aux_len, &aux_fifo, &_bmi);

    if (bmi2_failed(acc_rslt) || bmi2_failed(gyr_rslt) || bmi2_failed(aux_rslt)) {
        Serial.printf("[IMU] WARN: FIFO extract failed acc=%d gyr=%d aux=%d\n", acc_rslt, gyr_rslt, aux_rslt);
        out.fifo_overrun = true;
        flush_fifo("extract error");
        return false;
    }

    out.fifo_frames_drained = clamp_u8((acc_len > gyr_len) ? acc_len : gyr_len);
    out.fifo_overrun = partial_read ||
                       (_fifo.skipped_frame_count != 0) ||
                       (acc_len == kMaxParsedFrames) ||
                       (gyr_len == kMaxParsedFrames) ||
                       (aux_len == kMaxParsedFrames);

    if ((acc_len == 0) || (gyr_len == 0)) {
        if (out.fifo_overrun) {
            flush_fifo("no imu frame after overflow");
        }

        return false;
    }

    const bmi2_sens_axes_data& acc = _acc_frames[acc_len - 1];
    const bmi2_sens_axes_data& gyr = _gyr_frames[gyr_len - 1];
    out.bmi_post_lpf20_prepipe_ax = acc.x;
    out.bmi_post_lpf20_prepipe_ay = acc.y;
    out.bmi_post_lpf20_prepipe_az = acc.z;
    out.bmi_post_lpf20_prepipe_gx = gyr.x;
    out.bmi_post_lpf20_prepipe_gy = gyr.y;
    out.bmi_post_lpf20_prepipe_gz = gyr.z;
    out.bmi_acc_x_g = acc.x * kAccelScaleG;
    out.bmi_acc_y_g = acc.y * kAccelScaleG;
    out.bmi_acc_z_g = acc.z * kAccelScaleG;
    out.bmi_gyr_x_dps = gyr.x * kGyroScaleDps;
    out.bmi_gyr_y_dps = gyr.y * kGyroScaleDps;
    out.bmi_gyr_z_dps = gyr.z * kGyroScaleDps;
    out.imu_sample_fresh = true;

    if (_mag_ok && (aux_len > 0)) {
        const uint8_t* aux_data = _aux_frames[aux_len - 1].data;
        decode_bmm150_raw(aux_data, out.bmm_raw_x, out.bmm_raw_y, out.bmm_raw_z, out.bmm_rhall);
        out.mag_sample_fresh = true;
        out.mag_overflow = bmm150_raw_overflow(out.bmm_raw_x, out.bmm_raw_y, out.bmm_raw_z);

        bmm150_mag_data mag_data = {};
        int8_t mag_rslt = bmm150_aux_mag_data(const_cast<uint8_t*>(aux_data), &mag_data, &_bmm);
        out.mag_valid = (mag_rslt == BMM150_OK) && !out.mag_overflow;
        if (out.mag_valid) {
            out.bmm_ut_x = mag_data.x;
            out.bmm_ut_y = mag_data.y;
            out.bmm_ut_z = mag_data.z;
        }
    }

    return true;
}

bool Bmi270Provider::read_temperature(float& out_temp) {
    uint16_t raw_temp = 0;
    const int8_t rslt = bmi2_get_temperature_data(&raw_temp, &_bmi);
    if (rslt != BMI2_OK) {
        return false;
    }

    out_temp = bmi_temp_to_celsius(raw_temp);
    return true;
}

bool Bmi270Provider::flush_fifo(const char* reason) {
    const int8_t rslt = bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &_bmi);
    if (rslt != BMI2_OK) {
        Serial.printf("[IMU] WARN: FIFO flush failed (%d)%s%s\n",
                      rslt,
                      reason ? " - " : "",
                      reason ? reason : "");
        return false;
    }

    delayMicroseconds(1000);
    if (reason != nullptr) {
        Serial.printf("[IMU] FIFO flush: %s\n", reason);
    }

    return true;
}

void Bmi270Provider::cache_register_report() {
    _chip_id = read_reg8(BMI2_CHIP_ID_ADDR);
    _acc_conf = read_reg8(BMI2_ACC_CONF_ADDR);
    _acc_range = read_reg8(kBmi270AccRangeAddr);
    _gyr_conf = read_reg8(BMI2_GYR_CONF_ADDR);
    _gyr_range = read_reg8(kBmi270GyrRangeAddr);
    _aux_conf = read_reg8(BMI2_AUX_CONF_ADDR);
    _aux_if_conf = read_reg8(BMI2_AUX_IF_CONF_ADDR);
    _aux_rd_addr = read_reg8(BMI2_AUX_RD_ADDR);
    _pwr_ctrl = read_reg8(BMI2_PWR_CTRL_ADDR);
    _fifo_downs = read_reg8(BMI2_FIFO_DOWNS_ADDR);
    bmi2_get_fifo_config(&_fifo_config, &_bmi);
}

void Bmi270Provider::log_bringup_report() const {
    Serial.printf("[IMU] BMI270 @ 0x%02X CHIP_ID=0x%02X\n", _bmi_addr, _chip_id);
    Serial.printf("[IMU] ACC_CONF=0x%02X ACC_RANGE=0x%02X GYR_CONF=0x%02X GYR_RANGE=0x%02X\n",
                  _acc_conf, _acc_range, _gyr_conf, _gyr_range);
    Serial.printf("[IMU] AUX_CONF=0x%02X AUX_IF_CONF=0x%02X AUX_RD_ADDR=0x%02X PWR_CTRL=0x%02X FIFO_DOWNS=0x%02X FIFO_CFG=0x%04X\n",
                  _aux_conf, _aux_if_conf, _aux_rd_addr, _pwr_ctrl, _fifo_downs, _fifo_config);
    Serial.printf("[IMU] BMM150 CHIP_ID=0x%02X trim=%s aux_mode=%s\n",
                  _bmm.chip_id,
                  trim_data_present(_bmm.trim_data) ? "OK" : "MISSING",
                  (_aux_if_conf & BMI2_AUX_MAN_MODE_EN_MASK) ? "MANUAL" : "DATA");
}

void Bmi270Provider::fillStartupDiagnostics(uint8_t* bmi_regs,
                                            size_t bmi_len,
                                            uint8_t* bmm_regs,
                                            size_t bmm_len,
                                            uint8_t& bmi_chip_id,
                                            uint8_t& bmm_chip_id) const {
    if (bmi_regs && bmi_len > 0) {
        memset(bmi_regs, 0xFF, bmi_len);
        const uint8_t values[] = {
            _chip_id,
            _acc_conf,
            _acc_range,
            _gyr_conf,
            _gyr_range,
            _aux_conf,
            _aux_if_conf,
            _aux_rd_addr,
            _pwr_ctrl,
            _fifo_downs,
            (uint8_t)(_fifo_config & 0xFF),
            (uint8_t)(_fifo_config >> 8),
        };
        const size_t n = (bmi_len < sizeof(values)) ? bmi_len : sizeof(values);
        memcpy(bmi_regs, values, n);
    }

    if (bmm_regs && bmm_len > 0) {
        memset(bmm_regs, 0xFF, bmm_len);
        bmm_regs[0] = _bmm.chip_id;
        bmm_regs[1] = trim_data_present(_bmm.trim_data) ? 1 : 0;
        bmm_regs[2] = (_aux_if_conf & BMI2_AUX_MAN_MODE_EN_MASK) ? 1 : 0;
    }

    bmi_chip_id = _chip_id;
    bmm_chip_id = _bmm.chip_id;
}

bool Bmi270Provider::read_reg(uint8_t reg, uint8_t* data, uint32_t len) const {
    return M5.In_I2C.readRegister(_bmi_addr, reg, data, len, kI2CFreq);
}

bool Bmi270Provider::write_reg(uint8_t reg, const uint8_t* data, uint32_t len) const {
    return M5.In_I2C.writeRegister(_bmi_addr, reg, data, len, kI2CFreq);
}

uint8_t Bmi270Provider::read_reg8(uint8_t reg) const {
    return M5.In_I2C.readRegister8(_bmi_addr, reg, kI2CFreq);
}

void Bmi270Provider::decode_bmm150_raw(const uint8_t aux_data[8],
                                       int16_t& raw_x,
                                       int16_t& raw_y,
                                       int16_t& raw_z,
                                       uint16_t& rhall) {
    raw_x = static_cast<int16_t>(static_cast<int16_t>((aux_data[1] << 8) | aux_data[0]) >> 3);
    raw_y = static_cast<int16_t>(static_cast<int16_t>((aux_data[3] << 8) | aux_data[2]) >> 3);
    raw_z = static_cast<int16_t>(static_cast<int16_t>((aux_data[5] << 8) | aux_data[4]) >> 1);
    rhall = static_cast<uint16_t>((static_cast<uint16_t>(aux_data[7] << 8) | aux_data[6]) >> 2);
}

BMI2_INTF_RETURN_TYPE Bmi270Provider::bmi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    auto* self = static_cast<Bmi270Provider*>(intf_ptr);
    return self->read_reg(reg_addr, reg_data, len) ? BMI2_OK : BMI2_E_COM_FAIL;
}

BMI2_INTF_RETURN_TYPE Bmi270Provider::bmi_write(uint8_t reg_addr,
                                                const uint8_t* reg_data,
                                                uint32_t len,
                                                void* intf_ptr) {
    auto* self = static_cast<Bmi270Provider*>(intf_ptr);
    return self->write_reg(reg_addr, reg_data, len) ? BMI2_OK : BMI2_E_COM_FAIL;
}

void Bmi270Provider::bmi_delay_us(uint32_t period, void* intf_ptr) {
    (void)intf_ptr;
    delayMicroseconds(period);
}

BMM150_INTF_RET_TYPE Bmi270Provider::bmm_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr) {
    auto* self = static_cast<Bmi270Provider*>(intf_ptr);
    int8_t rslt = bmi2_read_aux_man_mode(reg_addr, reg_data, static_cast<uint16_t>(len), &self->_bmi);
    return (rslt == BMI2_OK) ? BMM150_OK : BMM150_E_COM_FAIL;
}

BMM150_INTF_RET_TYPE Bmi270Provider::bmm_write(uint8_t reg_addr,
                                               const uint8_t* reg_data,
                                               uint32_t len,
                                               void* intf_ptr) {
    auto* self = static_cast<Bmi270Provider*>(intf_ptr);
    int8_t rslt = bmi2_write_aux_man_mode(reg_addr, reg_data, static_cast<uint16_t>(len), &self->_bmi);
    return (rslt == BMI2_OK) ? BMM150_OK : BMM150_E_COM_FAIL;
}

void Bmi270Provider::bmm_delay_us(uint32_t period, void* intf_ptr) {
    (void)intf_ptr;
    delayMicroseconds(period);
}

#endif
