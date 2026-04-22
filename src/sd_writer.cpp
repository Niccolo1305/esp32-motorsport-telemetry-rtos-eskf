// SPDX-License-Identifier: GPL-3.0-or-later
// sd_writer.cpp — FreeRTOS Task_SD_Writer (Core 1, priority 1)
//
// Opens the file in APPEND mode (created with FILE_WRITE in setup()).
// Pure binary writes; flushes every SD_FLUSH_EVERY packets (5 s at 50 Hz).
//
// Write outcome handling:
//   full record written  → success, normal path
//   partial progress     → continue from the next byte of the same record
//   zero progress        → transient error: close/reopen and retry remaining bytes
//   over-reported write  → impossible API result: count it, clamp to remaining
//   sustained no-progress→ SD declared lost after SD_WRITE_STALL_TIMEOUT_MS
//
// Diagnostics exposed via globals (readable from loop() / MQTT heartbeat):
//   sd_records_written  — total records successfully written
//   sd_records_dropped  — records dropped at producer (queue full)
//   sd_flush_worst_us   — worst-case flush() duration in microseconds
//   sd_flush_count      — total flush() calls
//   sd_queue_hwm        — queue high-water mark (max pending records observed)
//   sd_write_overreport_count — impossible write() over-report diagnostics
#include "sd_writer.h"

#include <SD.h>
#include <esp_timer.h>

#include "globals.h"

void Task_SD_Writer(void *pvParameters) {
  File logFile = SD.open(current_log_filename, FILE_APPEND);
  if (!logFile) {
    Serial.println("[ERR] SD Task: failed to open file in APPEND mode!");
    sd_write_error = true;
    vTaskDelete(NULL);
  }

  TelemetryRecord rec;
  int unsaved_packets = 0;

  for (;;) {
    if (!xQueueReceive(sd_queue, &rec, portMAX_DELAY)) continue;

    // Track queue depth high-water mark (one sample per iteration, not per drain).
    UBaseType_t pending = uxQueueMessagesWaiting(sd_queue);
    uint32_t prev_hwm = sd_queue_hwm.load(std::memory_order_relaxed);
    if ((uint32_t)pending > prev_hwm)
      sd_queue_hwm.store((uint32_t)pending, std::memory_order_relaxed);

    const size_t expected = sizeof(TelemetryRecord);
    const uint8_t *record_bytes = reinterpret_cast<const uint8_t *>(&rec);
    size_t offset = 0;
    uint32_t no_progress_start_ms = 0;

    while (offset < expected) {
      const size_t remaining = expected - offset;
      size_t written = logFile.write(record_bytes + offset, remaining);

      if (written > remaining) {
        sd_write_overreport_count++;
        Serial.printf("[SD] WARN: write() over-reported %d/%d bytes; clamping to complete record.\n",
                      (int)written, (int)remaining);
        written = remaining;
      }

      if (written > 0) {
        if (written < remaining) {
          sd_partial_write_count++;
          Serial.printf("[SD] Partial write recovered: progress=%d remaining=%d\n",
                        (int)written, (int)remaining);
        }
        offset += written;
        no_progress_start_ms = 0;
        continue;
      }

      sd_stall_count++;
      const uint32_t now_ms = millis();
      if (no_progress_start_ms == 0) {
        no_progress_start_ms = now_ms;
      }
      const uint32_t stalled_ms = now_ms - no_progress_start_ms;
      uint32_t prev_worst_stall = sd_stall_worst_ms.load(std::memory_order_relaxed);
      if (stalled_ms > prev_worst_stall) {
        sd_stall_worst_ms.store(stalled_ms, std::memory_order_relaxed);
      }

      if (stalled_ms >= (uint32_t)SD_WRITE_STALL_TIMEOUT_MS) {
        Serial.printf("[SD] FATAL: no write progress for %lu ms with %d bytes remaining. SD declared lost.\n",
                      (unsigned long)stalled_ms, (int)remaining);
        sd_write_error = true;
        logFile.close();
        vTaskDelete(NULL);
      }

      Serial.printf("[SD] Write stalled for %lu ms, reopening and retrying %d remaining bytes...\n",
                    (unsigned long)stalled_ms, (int)remaining);
      logFile.close();
      unsaved_packets = 0; // close() flushes completed records already accepted by FS.
      vTaskDelay(pdMS_TO_TICKS(SD_WRITE_RETRY_DELAY_MS));
      sd_reopen_count++;
      logFile = SD.open(current_log_filename, FILE_APPEND);
      if (!logFile) {
        Serial.println("[SD] Reopen failed: SD removed?");
        sd_write_error = true;
        vTaskDelete(NULL);
      }
    }

    // ── SUCCESS: one complete, aligned TelemetryRecord appended ─────────────
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
  }
}
