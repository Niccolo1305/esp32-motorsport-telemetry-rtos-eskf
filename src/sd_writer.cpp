// SPDX-License-Identifier: GPL-3.0-or-later
// sd_writer.cpp - FreeRTOS Task_SD_Writer (Core 1, priority 1)
//
// Opens the file in APPEND mode (created with FILE_WRITE in setup()).
// Pure binary writes; batches SD_WRITE_BATCH_RECORDS records per File.write().
// With 256-byte AtomS3R records, the default 8-record batch is 2048 bytes
// (4 full SD sectors). Flush cadence is SD_FLUSH_EVERY_BATCHES batches.
//
// Write outcome handling:
//   full batch written   -> success, normal path
//   partial progress     -> keep batch offset and write only remaining bytes
//   zero progress        -> close/reopen, verify persistent size, retry missing bytes
//   over-reported write  -> impossible API result: count it, clamp to remaining
//   sustained zero stall -> SD declared lost after SD_WRITE_STALL_TIMEOUT_MS
#include "sd_writer.h"

#include <stdint.h>
#include <string.h>

#include <SD.h>
#include <esp_timer.h>

#include "crc16.h"
#include "globals.h"

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

static void fail_sd_writer(File& logFile, const char *message) {
  Serial.println(message);
  sd_write_error = true;
  logFile.close();
  vTaskDelete(NULL);
}

static void reopen_log_file(File& logFile) {
  logFile.close();
  vTaskDelay(pdMS_TO_TICKS(SD_WRITE_RETRY_DELAY_MS));
  sd_reopen_count++;
  logFile = SD.open(current_log_filename, FILE_APPEND);
  if (!logFile) {
    Serial.println("[SD] Reopen failed: SD removed?");
    sd_write_error = true;
    vTaskDelete(NULL);
  }
}

static uint32_t reopen_and_observe_size(File& logFile) {
  reopen_log_file(logFile);
  const uint32_t size = (uint32_t)logFile.size();
  sd_last_file_size.store(size, std::memory_order_relaxed);
  return size;
}

static void sync_log_file(File& logFile) {
  uint64_t t0 = esp_timer_get_time();
  logFile.flush();
  uint64_t dt_us_64 = esp_timer_get_time() - t0;
  uint32_t dt_us = (dt_us_64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)dt_us_64;
  sd_flush_count++;
  uint32_t prev_worst = sd_flush_worst_us.load(std::memory_order_relaxed);
  if (dt_us > prev_worst) {
    sd_flush_worst_us.store(dt_us, std::memory_order_relaxed);
  }
  sd_last_file_size.store((uint32_t)logFile.size(), std::memory_order_relaxed);
}

static bool write_bytes_with_retry(File& logFile,
                                   const uint8_t *bytes,
                                   size_t expected,
                                   uint32_t first_seq,
                                   uint32_t last_seq,
                                   bool verify_persistent_size,
                                   bool& forced_sync) {
  const uint32_t base_size = (uint32_t)logFile.size();
  const uint32_t target_size = base_size + (uint32_t)expected;
  size_t offset = 0;
  uint32_t stall_start_ms = 0;
  bool had_zero_progress = false;
  bool stall_reported = false;
  sd_last_file_size.store(base_size, std::memory_order_relaxed);

  for (;;) {
    if (offset >= expected) {
      if (!had_zero_progress && !verify_persistent_size) {
        break;
      }

      sync_log_file(logFile);
      forced_sync = true;
      const uint32_t actual_size = reopen_and_observe_size(logFile);
      if (actual_size == target_size) {
        break;
      }
      if (actual_size < base_size || actual_size > target_size) {
        sd_size_mismatch_count++;
        Serial.printf("[SD] FATAL: persistent size out of batch bounds after verify: base=%lu actual=%lu target=%lu seq=%lu..%lu.\n",
                      (unsigned long)base_size,
                      (unsigned long)actual_size,
                      (unsigned long)target_size,
                      (unsigned long)first_seq,
                      (unsigned long)last_seq);
        fail_sd_writer(logFile, "[SD] Persistent file size outside current batch bounds.");
      }

      sd_size_mismatch_count++;
      offset = (size_t)(actual_size - base_size);
      Serial.printf("[SD] Persistent size mismatch after batch verification: %d/%d byte(s) present; retrying missing tail seq=%lu..%lu.\n",
                    (int)offset,
                    (int)expected,
                    (unsigned long)first_seq,
                    (unsigned long)last_seq);
      stall_start_ms = 0;
      stall_reported = false;
      continue;
    }

    const size_t remaining = expected - offset;
    size_t written = logFile.write(bytes + offset, remaining);

    if (written > remaining) {
      sd_write_overreport_count++;
      Serial.printf("[SD] WARN: write() over-reported %d/%d bytes; clamping to remaining batch.\n",
                    (int)written, (int)remaining);
      written = remaining;
    }

    if (written > 0) {
      offset += written;
      stall_start_ms = 0;
      stall_reported = false;

      if (offset < expected) {
        sd_partial_write_count++;
        Serial.printf("[SD] Partial batch write accepted %d byte(s); %d/%d complete.\n",
                      (int)written, (int)offset, (int)expected);
      }
      continue;
    }

    had_zero_progress = true;
    sd_write_zero_count++;
    sd_last_stall_seq.store(first_seq, std::memory_order_relaxed);

    const uint32_t now_ms = millis();
    if (stall_start_ms == 0) {
      stall_start_ms = now_ms;
    }
    const uint32_t stalled_ms = now_ms - stall_start_ms;
    uint32_t prev_worst_stall = sd_stall_worst_ms.load(std::memory_order_relaxed);
    if (stalled_ms > prev_worst_stall) {
      sd_stall_worst_ms.store(stalled_ms, std::memory_order_relaxed);
    }

    if (!stall_reported) {
      stall_reported = true;
      sd_stall_count++;
      Serial.printf("[SD] Write stalled at batch offset %d/%d seq=%lu..%lu; reopening and verifying size.\n",
                    (int)offset,
                    (int)expected,
                    (unsigned long)first_seq,
                    (unsigned long)last_seq);
    }

    if (stalled_ms >= (uint32_t)SD_WRITE_STALL_TIMEOUT_MS) {
      Serial.printf("[SD] FATAL: batch write retry stalled for %lu ms. SD declared lost.\n",
                    (unsigned long)stalled_ms);
      fail_sd_writer(logFile, "[SD] Sustained zero-progress write stall.");
    }

    forced_sync = true;
    const uint32_t actual_size = reopen_and_observe_size(logFile);
    if (actual_size < base_size || actual_size > target_size) {
      sd_size_mismatch_count++;
      Serial.printf("[SD] FATAL: persistent size out of batch bounds after stall: base=%lu actual=%lu target=%lu seq=%lu..%lu.\n",
                    (unsigned long)base_size,
                    (unsigned long)actual_size,
                    (unsigned long)target_size,
                    (unsigned long)first_seq,
                    (unsigned long)last_seq);
      fail_sd_writer(logFile, "[SD] Persistent file size outside current batch bounds.");
    }

    const size_t persistent_offset = (size_t)(actual_size - base_size);
    if (persistent_offset != offset) {
      sd_size_mismatch_count++;
      Serial.printf("[SD] Write-return/persistent-size mismatch: returned offset=%d, persistent offset=%d/%d seq=%lu..%lu.\n",
                    (int)offset,
                    (int)persistent_offset,
                    (int)expected,
                    (unsigned long)first_seq,
                    (unsigned long)last_seq);
    }
    offset = persistent_offset;
    stall_start_ms = 0;
    stall_reported = false;
  }

  if (had_zero_progress) {
    sd_write_recovered_count++;
  }
  sd_last_written_seq.store(last_seq, std::memory_order_relaxed);
  sd_last_file_size.store(target_size, std::memory_order_relaxed);
  return true;
}

static void flush_log_file(File& logFile, int& unsaved_records) {
  sync_log_file(logFile);
  unsaved_records = 0;
}

void Task_SD_Writer(void *pvParameters) {
  File logFile = SD.open(current_log_filename, FILE_APPEND);
  if (!logFile) {
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

      if (batch_records < (size_t)SD_WRITE_BATCH_RECORDS) {
        continue;
      }
    } else if (batch_records == 0) {
      continue;
    }

    const bool partial_idle_batch = (received != pdTRUE);
    const size_t records_to_write = batch_records;
    const size_t expected = records_to_write * sizeof(TelemetryRecord);
    bool forced_sync = false;
    write_bytes_with_retry(logFile,
                           batch,
                           expected,
                           batch_first_seq,
                           batch_last_seq,
                           SD_VERIFY_EVERY_BATCH || partial_idle_batch,
                           forced_sync);

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
      flush_log_file(logFile, unsaved_records);
      unsaved_batches = 0;
    }
  }
}
