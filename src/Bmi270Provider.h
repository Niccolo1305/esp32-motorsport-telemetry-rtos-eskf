// SPDX-License-Identifier: GPL-3.0-or-later
// Bmi270Provider.h - AtomS3R Bosch-direct IMU provider (BMI270 + BMM150)
//
// M5Unified is used only for board/display/power and as the owner-facing
// transport of the internal I2C bus (M5.In_I2C). All IMU semantics, scaling,
// FIFO handling, and magnetometer compensation are provided directly by Bosch
// Sensor APIs.
#pragma once

#include <M5Unified.h>

extern "C" {
#include "vendor/bosch/bmi270/bmi2.h"
#include "vendor/bosch/bmi270/bmi270.h"
#include "vendor/bosch/bmm150/bmm150.h"
}

#include "IImuProvider.h"

class Bmi270Provider : public IImuProvider {
public:
    void begin() override;
    void update(ImuRawData& out) override;
    bool verifyConfig() override { return _config_ok; }

private:
    static constexpr uint8_t kPrimaryAddr = 0x69;
    static constexpr uint8_t kFallbackAddr = 0x68;
    static constexpr uint32_t kI2CFreq = 400000;
    static constexpr float kAccelScaleG = 8.0f / 32768.0f;
    static constexpr float kGyroScaleDps = 2000.0f / 32768.0f;
    static constexpr uint16_t kFifoBufferSize = 2048;
    static constexpr uint16_t kMaxParsedFrames = 80;

    bool probe_bmi_address();
    bool init_bmi270();
    bool init_bmm150();
    bool init_fifo();
    bool read_fifo_sample(ImuRawData& out);
    bool read_temperature(float& out_temp);
    bool flush_fifo(const char* reason);
    void cache_register_report();
    void log_bringup_report() const;
    bool read_reg(uint8_t reg, uint8_t* data, uint32_t len) const;
    bool write_reg(uint8_t reg, const uint8_t* data, uint32_t len) const;
    uint8_t read_reg8(uint8_t reg) const;
    static void decode_bmm150_raw(const uint8_t aux_data[8],
                                  int16_t& raw_x,
                                  int16_t& raw_y,
                                  int16_t& raw_z,
                                  uint16_t& rhall);

    static BMI2_INTF_RETURN_TYPE bmi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
    static BMI2_INTF_RETURN_TYPE bmi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);
    static void bmi_delay_us(uint32_t period, void* intf_ptr);

    static BMM150_INTF_RET_TYPE bmm_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
    static BMM150_INTF_RET_TYPE bmm_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);
    static void bmm_delay_us(uint32_t period, void* intf_ptr);

    bmi2_dev _bmi = {};
    bmm150_dev _bmm = {};
    bmi2_fifo_frame _fifo = {};
    uint8_t _bmi_addr = kPrimaryAddr;
    bool _config_ok = false;
    bool _mag_ok = false;
    bool _have_last_sample = false;
    ImuRawData _last_sample = {};

    uint8_t _fifo_buffer[kFifoBufferSize] = {};
    bmi2_sens_axes_data _acc_frames[kMaxParsedFrames] = {};
    bmi2_sens_axes_data _gyr_frames[kMaxParsedFrames] = {};
    bmi2_aux_fifo_data _aux_frames[kMaxParsedFrames] = {};

    uint8_t _chip_id = 0;
    uint8_t _acc_conf = 0;
    uint8_t _acc_range = 0;
    uint8_t _gyr_conf = 0;
    uint8_t _gyr_range = 0;
    uint8_t _aux_conf = 0;
    uint8_t _aux_if_conf = 0;
    uint8_t _aux_rd_addr = 0;
    uint8_t _pwr_ctrl = 0;
    uint16_t _fifo_config = 0;
};
