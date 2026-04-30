// SPDX-License-Identifier: GPL-3.0-or-later
// storage_manager.cpp - SD.h/SdFat backend abstraction for robust logging
#include "storage_manager.h"

#include <string.h>
#include <stddef.h>

#include <SPI.h>
#include <esp_timer.h>

#include "crc16.h"
#include "globals.h"

#if defined(LOG_BACKEND_SDFAT)
static SdFs sd_fs;
#endif

StorageManager storage;

static void update_worst_atomic(std::atomic<uint32_t>& target, uint32_t value) {
  uint32_t prev = target.load(std::memory_order_relaxed);
  if (value > prev) {
    target.store(value, std::memory_order_relaxed);
  }
}

static void mark_sync_duration(uint64_t dt_us_64) {
  const uint32_t dt_us = (dt_us_64 > UINT32_MAX)
      ? UINT32_MAX
      : (uint32_t)dt_us_64;
  sd_flush_count++;
  sd_sync_count++;
  update_worst_atomic(sd_flush_worst_us, dt_us);
  update_worst_atomic(sd_sync_worst_us, dt_us);
}

StorageReadFile::StorageReadFile() {}

bool StorageReadFile::open(const char *path) {
#if defined(LOG_BACKEND_SDFAT)
  close();
  return file_.open(path, O_RDONLY);
#else
  close();
  file_ = SD.open(path, FILE_READ);
  return (bool)file_;
#endif
}

int StorageReadFile::available() {
  return file_ ? file_.available() : 0;
}

int StorageReadFile::read() {
  return file_ ? file_.read() : -1;
}

void StorageReadFile::close() {
  if (file_) {
    file_.close();
  }
}

StorageReadFile::operator bool() const {
  return (bool)file_;
}

LogFileWriter::LogFileWriter()
    : allocated_size_(0),
      final_size_(0) {
  path_[0] = '\0';
}

bool LogFileWriter::openAppend(const char *path, uint32_t initial_logical_size) {
  close();
  strncpy(path_, path, sizeof(path_) - 1);
  path_[sizeof(path_) - 1] = '\0';
#if defined(LOG_BACKEND_SDFAT)
  if (!file_.open(path_, O_RDWR)) {
    return false;
  }
#if LOG_SDFAT_PREALLOCATE
  const uint64_t physical_size = (uint64_t)file_.size();
  allocated_size_ = physical_size > UINT32_MAX ? UINT32_MAX : (uint32_t)physical_size;
  final_size_ = initial_logical_size;
  if (!file_.seekSet(final_size_)) {
    file_.close();
    return false;
  }
#else
  if (!file_.seekEnd()) {
    file_.close();
    return false;
  }
  final_size_ = size();
  allocated_size_ = final_size_;
#endif
#else
  file_ = SD.open(path_, FILE_APPEND);
  if (!file_) {
    return false;
  }
  final_size_ = size();
  allocated_size_ = final_size_;
#endif
  sd_last_file_size.store(final_size_, std::memory_order_relaxed);
  sd_current_segment_bytes.store(final_size_, std::memory_order_relaxed);
  sd_current_segment_capacity.store(allocated_size_, std::memory_order_relaxed);
  return true;
}

bool LogFileWriter::reopen() {
  close();
  vTaskDelay(pdMS_TO_TICKS(SD_WRITE_RETRY_DELAY_MS));
  sd_reopen_count++;
#if defined(LOG_BACKEND_SDFAT)
  if (!file_.open(path_, O_RDWR)) {
    return false;
  }
#if LOG_SDFAT_PREALLOCATE
  if (!file_.seekSet(final_size_)) {
    file_.close();
    return false;
  }
#else
  if (!file_.seekEnd()) {
    file_.close();
    return false;
  }
#endif
  return true;
#else
  file_ = SD.open(path_, FILE_APPEND);
  return (bool)file_;
#endif
}

uint32_t LogFileWriter::reopenAndObserveSize() {
  if (!reopen()) {
    abortWithError("[SD] Reopen failed: SD removed?");
  }
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
  const uint32_t current_size = position();
#else
  const uint32_t current_size = size();
#endif
  sd_last_file_size.store(current_size, std::memory_order_relaxed);
  return current_size;
}

size_t LogFileWriter::writeRaw(const uint8_t *bytes, size_t len) {
  return file_ ? file_.write(bytes, len) : 0;
}

bool LogFileWriter::syncRaw() {
  if (!file_) {
    return false;
  }
  const uint64_t t0 = esp_timer_get_time();
#if defined(LOG_BACKEND_SDFAT)
  const bool ok = file_.sync();
#else
  file_.flush();
  const bool ok = true;
#endif
  mark_sync_duration(esp_timer_get_time() - t0);
  sd_last_file_size.store(size(), std::memory_order_relaxed);
  return ok;
}

void LogFileWriter::sync() {
  if (!syncRaw()) {
    abortWithError("[SD] Sync failed.");
  }
}

uint32_t LogFileWriter::size() {
  if (!file_) {
    return 0;
  }
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
  return final_size_;
#else
  const uint64_t current = (uint64_t)file_.size();
  return current > UINT32_MAX ? UINT32_MAX : (uint32_t)current;
#endif
}

bool LogFileWriter::ensurePreallocated(uint32_t target_size) {
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
  (void)target_size;
  return allocated_size_ > 0;
#else
  (void)target_size;
  return true;
#endif
}

bool LogFileWriter::seekSet(uint32_t new_position) {
  if (!file_) {
    return false;
  }
#if defined(LOG_BACKEND_SDFAT)
  return file_.seekSet(new_position);
#else
  return file_.seek(new_position);
#endif
}

uint32_t LogFileWriter::position() {
  if (!file_) {
    return 0;
  }
#if defined(LOG_BACKEND_SDFAT)
  const uint64_t current = (uint64_t)file_.curPosition();
#else
  const uint64_t current = (uint64_t)file_.position();
#endif
  return current > UINT32_MAX ? UINT32_MAX : (uint32_t)current;
}

bool LogFileWriter::verifyPreallocatedBytes(uint32_t base_position,
                                            const uint8_t *bytes,
                                            size_t expected) {
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
  close();
  if (!file_.open(path_, O_RDWR)) {
    return false;
  }
  if (!file_.seekSet(base_position)) {
    return false;
  }
  uint8_t verify_buf[64];
  size_t offset = 0;
  while (offset < expected) {
    const size_t remaining = expected - offset;
    const size_t chunk = remaining < sizeof(verify_buf) ? remaining : sizeof(verify_buf);
    const int got = file_.read(verify_buf, chunk);
    if (got != (int)chunk) {
      return false;
    }
    if (memcmp(verify_buf, bytes + offset, chunk) != 0) {
      return false;
    }
    offset += chunk;
  }
  return file_.seekSet(base_position + expected);
#else
  (void)base_position;
  (void)bytes;
  (void)expected;
  return true;
#endif
}

bool LogFileWriter::writeBytesWithCommit(const uint8_t *bytes,
                                         size_t expected,
                                         uint32_t first_seq,
                                         uint32_t last_seq,
                                         bool verify_persistent_size,
                                         bool& forced_sync) {
  const uint32_t base_size = size();
  const uint32_t target_size = base_size + (uint32_t)expected;
  size_t offset = 0;
  uint32_t stall_start_ms = 0;
  bool had_zero_progress = false;
  bool stall_reported = false;
  sd_last_file_size.store(base_size, std::memory_order_relaxed);

  if (!ensurePreallocated(target_size)) {
    abortWithError("[SD] Preallocation failed.");
  }

  for (;;) {
    if (offset >= expected) {
      if (!had_zero_progress && !verify_persistent_size) {
        break;
      }

      sync();
      forced_sync = true;
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
      if (verifyPreallocatedBytes(base_size, bytes, expected)) {
        break;
      }
      sd_size_mismatch_count++;
      Serial.printf("[SD] Preallocated read-back mismatch after batch verification; rewriting seq=%lu..%lu.\n",
                    (unsigned long)first_seq,
                    (unsigned long)last_seq);
      if (!seekSet(base_size)) {
        abortWithError("[SD] Failed to seek for preallocated rewrite.");
      }
      offset = 0;
      stall_start_ms = 0;
      stall_reported = false;
      continue;
#else
      const uint32_t actual_size = reopenAndObserveSize();
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
        abortWithError("[SD] Persistent file size outside current batch bounds.");
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
#endif
    }

    const size_t remaining = expected - offset;
    size_t written = writeRaw(bytes + offset, remaining);

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
    update_worst_atomic(sd_stall_worst_ms, stalled_ms);

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
      abortWithError("[SD] Sustained zero-progress write stall.");
    }

    forced_sync = true;
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
    if (!reopen() || !seekSet(base_size + (uint32_t)offset)) {
      abortWithError("[SD] Reopen failed during preallocated write stall.");
    }
    const uint32_t actual_size = base_size + (uint32_t)offset;
#else
    const uint32_t actual_size = reopenAndObserveSize();
    if (actual_size < base_size || actual_size > target_size) {
      sd_size_mismatch_count++;
      Serial.printf("[SD] FATAL: persistent size out of batch bounds after stall: base=%lu actual=%lu target=%lu seq=%lu..%lu.\n",
                    (unsigned long)base_size,
                    (unsigned long)actual_size,
                    (unsigned long)target_size,
                    (unsigned long)first_seq,
                    (unsigned long)last_seq);
      abortWithError("[SD] Persistent file size outside current batch bounds.");
    }
#endif

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
  final_size_ = target_size;
  sd_last_written_seq.store(last_seq, std::memory_order_relaxed);
  sd_last_file_size.store(target_size, std::memory_order_relaxed);
  sd_current_segment_bytes.store(final_size_, std::memory_order_relaxed);
  sd_current_segment_capacity.store(allocated_size_, std::memory_order_relaxed);
  return true;
}

bool LogFileWriter::finalize() {
  if (!file_) {
    return true;
  }
  sync();
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
  const bool truncated = file_.truncate(final_size_);
  sd_truncate_ok.store(truncated, std::memory_order_relaxed);
  if (!truncated) {
    return false;
  }
  return syncRaw();
#else
  sd_truncate_ok.store(true, std::memory_order_relaxed);
  return true;
#endif
}

void LogFileWriter::abortWithError(const char *message) {
  Serial.println(message);
  sd_write_error = true;
  close();
  vTaskDelete(NULL);
}

void LogFileWriter::close() {
  if (file_) {
    file_.close();
  }
}

bool LogFileWriter::isOpen() const {
  return (bool)file_;
}

bool LogFileWriter::needsRollover(size_t next_write_bytes) const {
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE
  if (next_write_bytes > UINT32_MAX) {
    return true;
  }
  return shouldRollover(final_size_, allocated_size_, (uint32_t)next_write_bytes);
#else
  (void)next_write_bytes;
  return false;
#endif
}

uint32_t LogFileWriter::logicalSize() const {
  return final_size_;
}

uint32_t LogFileWriter::allocatedSize() const {
  return allocated_size_;
}

bool StorageManager::begin() {
  if (mounted_) {
    return true;
  }
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
#if defined(LOG_BACKEND_SDFAT)
  mounted_ = sd_fs.begin(SdSpiConfig(CS_PIN, DEDICATED_SPI, SD_SPI_HZ, &SPI));
#else
  mounted_ = SD.begin(CS_PIN, SPI, SD_SPI_HZ);
#endif
  sd_backend_id.store(backendId(), std::memory_order_relaxed);
  return mounted_;
}

void StorageManager::end() {
#if defined(LOG_BACKEND_SDFAT)
  sd_fs.end();
#else
  SD.end();
#endif
  mounted_ = false;
}

bool StorageManager::isMounted() const {
  return mounted_;
}

bool StorageManager::exists(const char *path) {
#if defined(LOG_BACKEND_SDFAT)
  return sd_fs.exists(path);
#else
  return SD.exists(path);
#endif
}

bool StorageManager::openRead(const char *path, StorageReadFile& file) {
  return file.open(path);
}

bool StorageManager::createLogFile(const char *path,
                                   const uint8_t *header,
                                   size_t header_size) {
#if defined(LOG_BACKEND_SDFAT)
  FsFile f;
  if (!f.open(path, O_RDWR | O_CREAT | O_TRUNC)) {
    return false;
  }
#if LOG_SDFAT_PREALLOCATE
  const uint64_t prealloc_t0 = esp_timer_get_time();
  Serial.printf("[SD] Preallocating %lu byte(s) for %s.\n",
                (unsigned long)LOG_PREALLOC_BYTES_CFG,
                path);
  if (!f.preAllocate(LOG_PREALLOC_BYTES_CFG)) {
    f.close();
    return false;
  }
  const uint64_t prealloc_dt_ms = (esp_timer_get_time() - prealloc_t0) / 1000ULL;
  const uint32_t prealloc_ms = prealloc_dt_ms > UINT32_MAX
      ? UINT32_MAX
      : (uint32_t)prealloc_dt_ms;
  sd_prealloc_count++;
  update_worst_atomic(sd_prealloc_worst_ms, prealloc_ms);
  if (!f.seekSet(0)) {
    f.close();
    return false;
  }
  sd_prealloc_bytes.store(LOG_PREALLOC_BYTES_CFG, std::memory_order_relaxed);
  sd_current_segment_capacity.store(LOG_PREALLOC_BYTES_CFG, std::memory_order_relaxed);
#endif
  const size_t written = f.write(header, header_size);
  const uint64_t t0 = esp_timer_get_time();
  const bool synced = f.sync();
  mark_sync_duration(esp_timer_get_time() - t0);
#if LOG_SDFAT_PREALLOCATE
  const bool sized = f.curPosition() == header_size;
#else
  const bool sized = f.size() == header_size;
#endif
  f.close();
  sd_last_file_size.store((uint32_t)header_size, std::memory_order_relaxed);
  sd_current_segment_bytes.store((uint32_t)header_size, std::memory_order_relaxed);
  return written == header_size && synced && sized;
#else
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    return false;
  }
  size_t offset = 0;
  while (offset < header_size) {
    const size_t written = f.write(header + offset, header_size - offset);
    if (written == 0) {
      f.close();
      return false;
    }
    offset += written;
  }
  const uint64_t t0 = esp_timer_get_time();
  f.flush();
  mark_sync_duration(esp_timer_get_time() - t0);
  const bool sized = f.size() == header_size;
  f.close();
  sd_last_file_size.store((uint32_t)header_size, std::memory_order_relaxed);
  return sized;
#endif
}

bool StorageManager::repairPreallocatedLog(const char *path) {
#if defined(LOG_BACKEND_SDFAT) && LOG_SDFAT_PREALLOCATE && defined(USE_BMI270)
  const uint64_t t0 = esp_timer_get_time();
  FsFile f;
  if (!f.open(path, O_RDWR)) {
    return false;
  }

  const uint64_t physical_size64 = (uint64_t)f.size();
  if (physical_size64 < sizeof(FileHeaderV6)) {
    f.close();
    return false;
  }

  FileHeaderV6 hdr = {};
  if (!f.seekSet(0) || f.read(&hdr, sizeof(hdr)) != (int)sizeof(hdr)) {
    f.close();
    return false;
  }

  if (hdr.base.magic[0] != 'T' || hdr.base.magic[1] != 'E' ||
      hdr.base.magic[2] != 'L' || hdr.base.header_version < 6 ||
      hdr.base.record_size != sizeof(TelemetryRecord) ||
      hdr.header_size != sizeof(FileHeaderV6) ||
      hdr.data_offset != sizeof(FileHeaderV6) ||
      hdr.endian_marker != TELEMETRY_ENDIAN_MARKER ||
      !(hdr.build_flags & FILE_HEADER_V6_FLAG_LOG_PREALLOCATED)) {
    f.close();
    return false;
  }

  FileHeaderV6 crc_hdr = hdr;
  crc_hdr.header_crc16 = 0;
  if (telemetry_crc16_ccitt(reinterpret_cast<const uint8_t *>(&crc_hdr),
                            sizeof(crc_hdr)) != hdr.header_crc16) {
    f.close();
    return false;
  }

  uint32_t valid_size = hdr.data_offset;
  uint32_t pos = hdr.data_offset;
  TelemetryRecord rec = {};
  while ((uint64_t)pos + sizeof(TelemetryRecord) <= physical_size64) {
    if (!f.seekSet(pos) || f.read(&rec, sizeof(rec)) != (int)sizeof(rec)) {
      break;
    }
    const uint16_t stored_crc = rec.crc16;
    rec.crc16 = 0;
    const bool record_ok =
        rec.record_magic == TELEMETRY_RECORD_MAGIC &&
        telemetry_crc16_ccitt(reinterpret_cast<const uint8_t *>(&rec),
                              sizeof(TelemetryRecord) - sizeof(rec.crc16)) == stored_crc;
    if (!record_ok) {
      break;
    }
    valid_size = pos + sizeof(TelemetryRecord);
    pos = valid_size;
  }

  if ((uint64_t)valid_size < physical_size64) {
    const uint64_t truncated64 = physical_size64 - valid_size;
    if (!f.truncate(valid_size) || !f.sync()) {
      f.close();
      return false;
    }
    sd_boot_repair_count++;
    const uint32_t truncated = (truncated64 > UINT32_MAX)
        ? UINT32_MAX
        : (uint32_t)truncated64;
    sd_boot_repair_truncated_bytes.fetch_add(truncated, std::memory_order_relaxed);
    const uint64_t dt_ms64 = (esp_timer_get_time() - t0) / 1000ULL;
    const uint32_t dt_ms = dt_ms64 > UINT32_MAX ? UINT32_MAX : (uint32_t)dt_ms64;
    update_worst_atomic(sd_boot_repair_worst_ms, dt_ms);
    Serial.printf("[SD] Repaired preallocated log %s: truncated %lu byte(s), valid_size=%lu.\n",
                  path,
                  (unsigned long)truncated,
                  (unsigned long)valid_size);
  }

  f.close();
  return true;
#else
  (void)path;
  return false;
#endif
}

uint64_t StorageManager::totalBytes() {
#if defined(LOG_BACKEND_SDFAT)
  return (uint64_t)sd_fs.clusterCount() * sd_fs.bytesPerCluster();
#else
  return SD.totalBytes();
#endif
}

uint64_t StorageManager::usedBytes() {
#if defined(LOG_BACKEND_SDFAT)
  const int32_t free_clusters = sd_fs.freeClusterCount();
  if (free_clusters < 0) {
    return 0;
  }
  const uint64_t total = totalBytes();
  const uint64_t free_bytes = (uint64_t)free_clusters * sd_fs.bytesPerCluster();
  return free_bytes > total ? 0 : total - free_bytes;
#else
  return SD.usedBytes();
#endif
}

uint32_t StorageManager::backendId() const {
#if defined(LOG_BACKEND_SDFAT)
  return STORAGE_BACKEND_SDFAT;
#else
  return STORAGE_BACKEND_SD_H;
#endif
}

const char *StorageManager::backendName() const {
#if defined(LOG_BACKEND_SDFAT)
  return "SdFat";
#else
  return "SD.h";
#endif
}
