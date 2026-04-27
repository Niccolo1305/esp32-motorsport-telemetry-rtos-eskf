// SPDX-License-Identifier: GPL-3.0-or-later
// imu_task.cpp — FreeRTOS Task_I2C (Core 0, priority 3)
//
// IMU read every exact 20 ms (vTaskDelayUntil: zero jitter).
// Also reads temperature here: the only context with exclusive I2C bus access.
// This eliminates the race condition that existed in v0.5.0 where getTemp()
// was called in Task_Filter outside the mutex, contending with this task.
#include "imu_task.h"

#include "IImuProvider.h"
#include "globals.h"

void Task_I2C(void *pvParameters) {
  IImuProvider* imu = static_cast<IImuProvider*>(pvParameters);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(DT_MS);
  ImuRawData data;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    imu->update(data);
    if (IMU_QUEUE_DEPTH == 1) {
      // Mailbox mode: always keep the latest sample for realtime display/control.
      xQueueOverwrite(imuQueue, &data);
    } else if (xQueueSend(imuQueue, &data, 0) != pdTRUE) {
      // Diagnostic FIFO mode: if full, discard the oldest sample and keep going.
      ImuRawData dropped;
      (void)xQueueReceive(imuQueue, &dropped, 0);
      (void)xQueueSend(imuQueue, &data, 0);
    }
  }
}
