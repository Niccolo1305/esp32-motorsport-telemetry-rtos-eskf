// SPDX-License-Identifier: GPL-3.0-or-later
// sd_writer.cpp — FreeRTOS Task_SD_Writer (Core 1, priority 1)
//
// Opens the file in APPEND mode (created with FILE_WRITE in setup()).
// Pure binary writes: 127 bytes/record (v1.3.1), no formatting.
// Flush every 50 packets = 1 s at 50 Hz: durability/throughput trade-off.
#include "sd_writer.h"

#include <SD.h>

#include "globals.h"

void Task_SD_Writer(void *pvParameters) {
  File logFile = SD.open(current_log_filename, FILE_APPEND);
  if (!logFile) {
    Serial.println("[ERR] SD Task: failed to open file in APPEND mode!");
    vTaskDelete(NULL);
  }

  TelemetryRecord rec;
  int unsaved_packets = 0;
  int write_fail_count = 0;
  bool pending_retry = false; // BUG-9: true if rec holds a failed record to retry

  for (;;) {
    // BUG-9 fix: only dequeue a new record if we don't have a pending retry.
    // On write failure, the failed rec is retried after reopen instead of
    // being silently dropped.
    if (!pending_retry) {
      if (!xQueueReceive(sd_queue, &rec, portMAX_DELAY)) continue;
    }
    pending_retry = false;

    size_t written = logFile.write((const uint8_t *)&rec, sizeof(TelemetryRecord));
    if (written != sizeof(TelemetryRecord)) {
      // Write failed: card removed or filesystem corrupt.
      write_fail_count++;
      if (write_fail_count <= 3) {
        Serial.printf("[SD] Write failed (%d/3), attempting reopen...\n", write_fail_count);
        logFile.close();
        vTaskDelay(pdMS_TO_TICKS(100));
        logFile = SD.open(current_log_filename, FILE_APPEND);
        if (!logFile) {
          Serial.println("[SD] Reopen failed: SD removed?");
          sd_write_error = true;
          vTaskDelete(NULL);
        }
        pending_retry = true; // retry the same rec on next iteration
      } else {
        Serial.println("[SD] 3 consecutive write failures: SD declared lost.");
        sd_write_error = true;
        logFile.close();
        vTaskDelete(NULL);
      }
    } else {
      write_fail_count = 0;  // reset on successful write
      sd_records_written++;
      unsaved_packets++;
      if (unsaved_packets >= SD_FLUSH_EVERY) {
        logFile.flush();
        unsaved_packets = 0;
      }
    }
  }
}
