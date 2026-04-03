// SPDX-License-Identifier: GPL-3.0-or-later
// gps_task.cpp — FreeRTOS Task_GPS (Core 0, priority 2)
//
// Reads NMEA characters from the hardware serial (Grove RX=1) and feeds
// them to TinyGPSPlus.encode(). When TinyGPSPlus has decoded a complete
// valid sentence, updates shared_gps_data under gps_mutex.
//
// GPS update rate: 10 Hz (CASIC $PCAS02,100 command sent at boot).
// Task runs every 20 ms: multiple NMEA sentences are consumed quickly
// without accumulation in the UART buffer.
#include "gps_task.h"

#include "globals.h"

void Task_GPS(void *pvParameters) {
  uint32_t last_overflow_log = 0; // last logged value (avoids serial spam)

  for (;;) {
    // Drain UART buffer: encode() updates gps internally on every character
    // that completes a valid NMEA sentence.
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    // Log UART overflow (incremented by the ISR callback registered in setup())
    uint32_t ovf = gps_uart_overflow_count;
    if (ovf > last_overflow_log) {
      Serial.printf("[GPS] UART overflow detected: %u total\n", ovf);
      last_overflow_log = ovf;
    }

    // Update shared data only if TinyGPSPlus received updated data since
    // the last cycle (isUpdated() flag).
    // Note: sats is updated even without a position fix.
    if (gps.satellites.isUpdated() || gps.location.isUpdated() ||
        gps.speed.isUpdated() || gps.altitude.isUpdated()) {
      if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        shared_gps_data.sats =
            gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
        shared_gps_data.valid = gps.location.isValid();
        if (shared_gps_data.valid) {
          shared_gps_data.lat = gps.location.lat();
          shared_gps_data.lon = gps.location.lng();
          shared_gps_data.speed_kmh =
              gps.speed.isValid() ? (float)gps.speed.kmph() : 0.0f;
          shared_gps_data.alt_m =
              gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
          shared_gps_data.hdop =
              gps.hdop.isValid() ? (float)gps.hdop.hdop() : 99.9f;
          shared_gps_data.fix_ms = millis(); // v0.9.11: timestamp for staleness detection
          shared_gps_data.epoch++;  // v0.8.0: signals a fresh fix to the filter
        }
        xSemaphoreGive(gps_mutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz polling — well above the NMEA update rate
  }
}
