// SPDX-License-Identifier: GPL-3.0-or-later
// gps_task.cpp — FreeRTOS Task_GPS (Core 0, priority 2)
//
// Receives an IGpsProvider* via pvParameters and polls it at 50 Hz.
// When the provider decodes a complete NMEA fix, updates shared_gps_data
// under gps_mutex for consumption by Task_Filter and loop().
//
// GPS update rate: 10 Hz (CASIC $PCAS02,100 command sent by provider::begin()).
// Task polls every 20 ms: multiple NMEA sentences are consumed without
// accumulation in the UART buffer.
//
// Hardware coupling (HardwareSerial, TinyGPSPlus, UART overflow ISR) is
// fully contained in SerialGpsWrapper.h — this task is hardware-agnostic.
#include "gps_task.h"
#include "IGpsProvider.h"
#include "globals.h"

void Task_GPS(void *pvParameters) {
    IGpsProvider* provider = static_cast<IGpsProvider*>(pvParameters);
    uint32_t last_overflow_log = 0;
    GpsData data = {};

    for (;;) {
        // Poll provider: drains UART, decodes NMEA, returns true on new fix.
        if (provider->update(data)) {
            if (xSemaphoreTake(gps_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                shared_gps_data = data;
                xSemaphoreGive(gps_mutex);
            }
        }

        // Log UART overflow events (incremented by the provider's ISR callback).
        uint32_t ovf = provider->getOverflowCount();
        if (ovf > last_overflow_log) {
            Serial.printf("[GPS] UART overflow detected: %u total\n", ovf);
            last_overflow_log = ovf;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz polling — well above the 10 Hz NMEA rate
    }
}
