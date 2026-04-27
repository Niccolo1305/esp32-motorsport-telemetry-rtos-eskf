// SPDX-License-Identifier: GPL-3.0-or-later
// crc16.h - Shared CRC16-CCITT helper for binary telemetry headers/records.
#pragma once

#include <stddef.h>
#include <stdint.h>

static inline uint16_t telemetry_crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}
