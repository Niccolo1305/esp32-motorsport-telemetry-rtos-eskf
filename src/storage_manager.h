// SPDX-License-Identifier: GPL-3.0-or-later
// storage_manager.h - SD storage abstraction for telemetry logging
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#include "config.h"

#if !defined(LOG_BACKEND_SD_H) && !defined(LOG_BACKEND_SDFAT)
#define LOG_BACKEND_SD_H 1
#endif

#if defined(LOG_BACKEND_SD_H) && defined(LOG_BACKEND_SDFAT)
#error "Select only one storage backend: LOG_BACKEND_SD_H or LOG_BACKEND_SDFAT"
#endif

#if defined(LOG_BACKEND_SDFAT)
#include <SdFat.h>
#else
#include <FS.h>
#include <SD.h>
#endif

enum StorageBackendId : uint32_t {
  STORAGE_BACKEND_SD_H = 1,
  STORAGE_BACKEND_SDFAT = 2,
};

class StorageReadFile {
public:
  StorageReadFile();
  bool open(const char *path);
  int available();
  int read();
  void close();
  explicit operator bool() const;

private:
#if defined(LOG_BACKEND_SDFAT)
  FsFile file_;
#else
  File file_;
#endif
};

class LogFileWriter {
public:
  LogFileWriter();

  bool openAppend(const char *path, uint32_t initial_logical_size);
  bool writeBytesWithCommit(const uint8_t *bytes,
                            size_t expected,
                            uint32_t first_seq,
                            uint32_t last_seq,
                            bool verify_persistent_size,
                            bool& forced_sync);
  bool needsRollover(size_t next_write_bytes) const;
  uint32_t logicalSize() const;
  uint32_t allocatedSize() const;
  void sync();
  uint32_t size();
  bool finalize();
  void abortWithError(const char *message);
  void close();
  bool isOpen() const;

private:
  bool reopen();
  uint32_t reopenAndObserveSize();
  size_t writeRaw(const uint8_t *bytes, size_t len);
  bool syncRaw();
  bool ensurePreallocated(uint32_t target_size);
  bool seekSet(uint32_t position);
  uint32_t position();
  bool verifyPreallocatedBytes(uint32_t base_position,
                               const uint8_t *bytes,
                               size_t expected);

  char path_[64];
  uint32_t allocated_size_;
  uint32_t final_size_;
#if defined(LOG_BACKEND_SDFAT)
  FsFile file_;
#else
  File file_;
#endif
};

class StorageManager {
public:
  bool begin();
  void end();
  bool isMounted() const;
  bool exists(const char *path);
  bool openRead(const char *path, StorageReadFile& file);
  bool createLogFile(const char *path, const uint8_t *header, size_t header_size);
  bool scanTelemetryLogs(uint32_t& max_index, char *latest_path, size_t latest_path_size);
  bool repairPreallocatedLog(const char *path);
  uint64_t totalBytes();
  uint64_t usedBytes();
  uint32_t backendId() const;
  const char *backendName() const;

private:
  bool mounted_ = false;
};

extern StorageManager storage;
