// SPDX-License-Identifier: GPL-3.0-or-later
// imu_task.cpp — FreeRTOS Task_I2C (Core 0, priority 3)
//
// IMU read every exact 20 ms (vTaskDelayUntil: zero jitter).
// Also reads temperature here: the only context with exclusive I2C bus access.
// This eliminates the race condition that existed in v0.5.0 where getTemp()
// was called in Task_Filter outside the mutex, contending with this task.
#include "imu_task.h"

#include <M5Unified.h>

#include "globals.h"

void Task_I2C(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(DT_MS);
  ImuRawData data;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    M5.Imu.getAccelData(&data.ax, &data.ay, &data.az);
    M5.Imu.getGyroData(&data.gx, &data.gy, &data.gz);
    // Temperature read in the same I2C context: no contention with Task_Filter.
    // Defaults to 0.0 f if the read fails (transient I2C error).
    data.temp_c = 0.0f;
    M5.Imu.getTemp(&data.temp_c);
    data.timestamp_us = esp_timer_get_time();
    // xQueueOverwrite: if Task_Filter is behind, discard old data and
    // insert the new one (real-time data, not FIFO).
    xQueueOverwrite(imuQueue, &data);
  }
}
