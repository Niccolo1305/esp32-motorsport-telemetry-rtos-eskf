// SPDX-License-Identifier: GPL-3.0-or-later
// sd_writer.cpp - FreeRTOS Task_SD_Writer (Core 1, priority 1)
//
// The task owns only record batching and telemetry-record finalization.
// Durable storage semantics live in LogFileWriter so SD.h and SdFat backends
// share the same commit rule: write is accepted only after coherent size/pos.
#include "sd_writer.h"

#include <stdint.h>
#include <string.h>

#include <esp_timer.h>

#include "crc16.h"
#include "globals.h"
#include "storage_manager.h"

#ifdef USE_BMI270
static uint8_t saturate_u8_counter(uint32_t value) {
  return value > UINT8_MAX ? UINT8_MAX : (uint8_t)value;
}

static uint32_t record_seq_or_zero(const TelemetryRecord& rec) {
  return rec.seq;
}
#else
static uint32_t record_seq_or_zero(const TelemetryRecord&) {
  return 0;
}
#endif

#ifdef USE_BMI270
static constexpr uint32_t LOG_HEADER_BYTES = sizeof(FileHeaderV6);
#else
static constexpr uint32_t LOG_HEADER_BYTES = sizeof(FileHeader);
#endif

static void flush_log_file(LogFileWriter& logFile, int& unsaved_records) {
  logFile.sync();
  unsaved_records = 0;
}

static uint32_t elapsed_ms(uint64_t t0_us) {
  const uint64_t dt_ms = (esp_timer_get_time() - t0_us) / 1000ULL;
  return dt_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)dt_ms;
}

static void update_worst_ms(std::atomic<uint32_t>& target, uint32_t value) {
  uint32_t prev = target.load(std::memory_order_relaxed);
  if (value > prev) {
    target.store(value, std::memory_order_relaxed);
  }
}

static bool rollover_log_file(LogFileWriter& logFile,
                              int& unsaved_records,
                              int& unsaved_batches) {
  const uint64_t t0 = esp_timer_get_time();
  if (!logFile.finalize()) {
    Serial.println("[SD] Rollover failed: finalize/truncate failed.");
    return false;
  }
  logFile.close();

  char next_filename[sizeof(current_log_filename)] = {};
  if (!create_next_log_segment(next_filename, sizeof(next_filename))) {
    Serial.println("[SD] Rollover failed: next segment create/prealloc/header failed.");
    return false;
  }
  strncpy(current_log_filename, next_filename, sizeof(current_log_filename) - 1);
  current_log_filename[sizeof(current_log_filename) - 1] = '\0';

  if (!logFile.openAppend(current_log_filename, LOG_HEADER_BYTES)) {
    Serial.println("[SD] Rollover failed: reopen next segment failed.");
    return false;
  }

  unsaved_records = 0;
  unsaved_batches = 0;
  sd_rollover_count++;
  update_worst_ms(sd_rollover_worst_ms, elapsed_ms(t0));
  Serial.printf("[SD] Rolled over to %s capacity=%lu bytes.\n",
                current_log_filename,
                (unsigned long)logFile.allocatedSize());
  return true;
}

void Task_SD_Writer(void *pvParameters) {
  LogFileWriter logFile;
  if (!logFile.openAppend(current_log_filename, LOG_HEADER_BYTES)) {
    Serial.println("[ERR] SD Task: failed to open file in APPEND mode!");
    sd_write_error = true;
    vTaskDelete(NULL);
  }

  TelemetryRecord rec;
  uint8_t batch[sizeof(TelemetryRecord) * SD_WRITE_BATCH_RECORDS];
  size_t batch_records = 0;
  uint32_t batch_first_seq = 0;
  uint32_t batch_last_seq = 0;
  int unsaved_records = 0;
  int unsaved_batches = 0;

  for (;;) {
    BREADCRUMB_MARK(BREADCRUMB_PHASE_SD_WAIT, batch_last_seq);
    const TickType_t wait_ticks = (batch_records > 0)
        ? pdMS_TO_TICKS(SD_BATCH_IDLE_FLUSH_MS)
        : portMAX_DELAY;
    const BaseType_t received = xQueueReceive(sd_queue, &rec, wait_ticks);

    if (received == pdTRUE) {
      UBaseType_t pending = uxQueueMessagesWaiting(sd_queue);
      uint32_t prev_hwm = sd_queue_hwm.load(std::memory_order_relaxed);
      if ((uint32_t)pending > prev_hwm)
        sd_queue_hwm.store((uint32_t)pending, std::memory_order_relaxed);

#ifdef USE_BMI270
      rec.sd_queue_hwm =
          saturate_u8_counter(sd_queue_hwm.load(std::memory_order_relaxed));
      rec.record_magic = TELEMETRY_RECORD_MAGIC;
      rec.sd_records_dropped =
          saturate_u8_counter(sd_records_dropped.load(std::memory_order_relaxed));
      rec.sd_partial_write_count =
          saturate_u8_counter(sd_partial_write_count.load(std::memory_order_relaxed));
      rec.sd_stall_count =
          saturate_u8_counter(sd_stall_count.load(std::memory_order_relaxed));
      rec.sd_reopen_count =
          saturate_u8_counter(sd_reopen_count.load(std::memory_order_relaxed));
      rec.crc16 = 0;
      rec.crc16 = telemetry_crc16_ccitt(reinterpret_cast<const uint8_t *>(&rec),
                                        sizeof(TelemetryRecord) - sizeof(rec.crc16));
#endif

      const uint32_t seq = record_seq_or_zero(rec);
      if (batch_records == 0) {
        batch_first_seq = seq;
      }
      batch_last_seq = seq;
      memcpy(batch + batch_records * sizeof(TelemetryRecord),
             &rec,
             sizeof(TelemetryRecord));
      batch_records++;
      BREADCRUMB_MARK(BREADCRUMB_PHASE_SD_BATCH, batch_last_seq);

      if (batch_records < (size_t)SD_WRITE_BATCH_RECORDS) {
        continue;
      }
    } else if (batch_records == 0) {
      continue;
    }

    const bool partial_idle_batch = (received != pdTRUE);
    const size_t records_to_write = batch_records;
    const size_t expected = records_to_write * sizeof(TelemetryRecord);

    if (logFile.needsRollover(expected)) {
      if (!rollover_log_file(logFile, unsaved_records, unsaved_batches)) {
        sd_write_error = true;
        logFile.close();
        vTaskDelete(NULL);
      }
    }

    bool forced_sync = false;
    BREADCRUMB_MARK(BREADCRUMB_PHASE_SD_WRITE, batch_first_seq);
    if (!logFile.writeBytesWithCommit(batch,
                                      expected,
                                      batch_first_seq,
                                      batch_last_seq,
                                      SD_VERIFY_EVERY_BATCH || partial_idle_batch,
                                      forced_sync)) {
      Serial.println("[SD] FATAL: writeBytesWithCommit returned false.");
      sd_write_error = true;
      logFile.close();
      vTaskDelete(NULL);
    }

    sd_records_written.fetch_add((uint32_t)records_to_write,
                                 std::memory_order_relaxed);
    if (forced_sync) {
      unsaved_records = 0;
      unsaved_batches = 0;
    } else {
      unsaved_records += (int)records_to_write;
      unsaved_batches++;
    }
    batch_records = 0;

    if (!forced_sync &&
        (partial_idle_batch ||
         unsaved_batches >= SD_FLUSH_EVERY_BATCHES ||
         unsaved_records >= SD_FLUSH_EVERY)) {
      BREADCRUMB_MARK(BREADCRUMB_PHASE_SD_FLUSH, batch_last_seq);
      flush_log_file(logFile, unsaved_records);
      unsaved_batches = 0;
    }
  }
}
