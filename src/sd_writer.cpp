// SPDX-License-Identifier: GPL-3.0-or-later
// sd_writer.cpp — FreeRTOS Task_SD_Writer (Core 1, priority 1)
//
// Opens the file in APPEND mode (created with FILE_WRITE in setup()).
// Pure binary writes; flushes every SD_FLUSH_EVERY packets (5 s at 50 Hz).
//
// Write outcome handling:
//   written == sizeof  → success, normal path
//   written == 0       → transient error (card removed?): close/reopen, retry up to 3x
//   0 < written < size → partial write: file alignment corrupted, unrecoverable → fatal
//
// Diagnostics exposed via globals (readable from loop() / MQTT heartbeat):
//   sd_records_written  — total records successfully written
//   sd_records_dropped  — records dropped at producer (queue full)
//   sd_flush_worst_us   — worst-case flush() duration in microseconds
//   sd_flush_count      — total flush() calls
//   sd_queue_hwm        — queue high-water mark (max pending records observed)
#include "sd_writer.h"

#include <SD.h>
#include <esp_timer.h>

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
  bool pending_retry = false; // true if rec holds a failed zero-write to retry

  for (;;) {
    // Only dequeue a new record if we don't have a pending zero-write retry.
    if (!pending_retry) {
      if (!xQueueReceive(sd_queue, &rec, portMAX_DELAY)) continue;
    }
    pending_retry = false;

    // Track queue depth high-water mark (one sample per iteration, not per drain).
    UBaseType_t pending = uxQueueMessagesWaiting(sd_queue);
    uint32_t prev_hwm = sd_queue_hwm.load(std::memory_order_relaxed);
    if ((uint32_t)pending > prev_hwm)
      sd_queue_hwm.store((uint32_t)pending, std::memory_order_relaxed);

    size_t written = logFile.write((const uint8_t *)&rec, sizeof(TelemetryRecord));

    if (written == sizeof(TelemetryRecord)) {
      // ── SUCCESS ──────────────────────────────────────────────────────────
      write_fail_count = 0;
      sd_records_written++;
      unsaved_packets++;
      if (unsaved_packets >= SD_FLUSH_EVERY) {
        uint64_t t0 = esp_timer_get_time();
        logFile.flush();
        uint32_t dt_us = (uint32_t)(esp_timer_get_time() - t0);
        sd_flush_count++;
        uint32_t prev_worst = sd_flush_worst_us.load(std::memory_order_relaxed);
        if (dt_us > prev_worst)
          sd_flush_worst_us.store(dt_us, std::memory_order_relaxed);
        unsaved_packets = 0;
      }

    } else if (written > 0) {
      // ── PARTIAL WRITE — unrecoverable ────────────────────────────────────
      // Some bytes were written but not all: the file's record alignment is
      // permanently broken. Any subsequent writes would start at a wrong
      // offset, making the rest of the file unparseable. Declare fatal.
      Serial.printf("[SD] FATAL: partial write (%d/%d bytes). File corrupted.\n",
                    (int)written, (int)sizeof(TelemetryRecord));
      sd_write_error = true;
      logFile.close();
      vTaskDelete(NULL);

    } else {
      // ── ZERO WRITE — transient error, safe to retry ──────────────────────
      // Zero bytes written: no alignment damage. Close, wait, reopen, retry.
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
    }
  }
}
