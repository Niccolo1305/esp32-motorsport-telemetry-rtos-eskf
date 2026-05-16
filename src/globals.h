// SPDX-License-Identifier: GPL-3.0-or-later
// globals.h — Extern declarations for all shared global state
//
// Approach 1 (v1.1.0): all cross-module state lives here with extern linkage.
// Each variable is defined exactly once in globals.cpp.
// A future v2.0.0 may replace this with per-module encapsulation.
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <atomic>

#include "config.h"
#include "types.h"
#include "eskf.h"
#include "madgwick.h" // Created in Batch 2 — build requires all batches complete

// ── FreeRTOS Primitives ────────────────────────────────────────────────────
extern QueueHandle_t imuQueue;
extern QueueHandle_t sd_queue;
extern SemaphoreHandle_t telemetry_mutex;
extern SemaphoreHandle_t gps_mutex;

// ── Task Handles ───────────────────────────────────────────────────────────
extern TaskHandle_t TaskI2CHandle;
extern TaskHandle_t TaskFilterHandle;
extern TaskHandle_t TaskSDHandle;
extern TaskHandle_t TaskGPSHandle;

// Crash breadcrumb: survives ESP_RST_PANIC / watchdog resets in RTC no-init RAM
// and is printed at the next boot. Diagnostic only, not part of SD record format.
enum BootBreadcrumbPhase : uint16_t {
  BREADCRUMB_PHASE_NONE = 0,
  BREADCRUMB_PHASE_BOOT_SETUP = 1,
  BREADCRUMB_PHASE_BOOT_IMU = 2,
  BREADCRUMB_PHASE_BOOT_GPS = 3,
  BREADCRUMB_PHASE_BOOT_SD = 4,
  BREADCRUMB_PHASE_BOOT_TASKS = 5,
  BREADCRUMB_PHASE_LOOP = 10,
  BREADCRUMB_PHASE_GPS_UPDATE = 20,
  BREADCRUMB_PHASE_GPS_PUBLISH = 21,
  BREADCRUMB_PHASE_I2C_UPDATE = 30,
  BREADCRUMB_PHASE_I2C_QUEUE = 31,
  BREADCRUMB_PHASE_FILTER_WAIT = 40,
  BREADCRUMB_PHASE_FILTER_SAMPLE = 41,
  BREADCRUMB_PHASE_FILTER_GPS = 42,
  BREADCRUMB_PHASE_FILTER_SUPERVISOR = 43,
  BREADCRUMB_PHASE_FILTER_PREDICT = 44,
  BREADCRUMB_PHASE_FILTER_CORRECT = 45,
  BREADCRUMB_PHASE_FILTER_MOUNT_YAW = 46,
  BREADCRUMB_PHASE_FILTER_SD_ENQUEUE = 47,
  BREADCRUMB_PHASE_SD_WAIT = 60,
  BREADCRUMB_PHASE_SD_BATCH = 61,
  BREADCRUMB_PHASE_SD_WRITE = 62,
  BREADCRUMB_PHASE_SD_FLUSH = 63,
};

struct BootBreadcrumb {
  uint32_t magic;
  uint16_t version;
  uint16_t phase;
  uint32_t boot_count;
  uint32_t last_reset_reason;
  uint32_t uptime_ms;
  uint32_t seq;
  uint16_t line;
  uint16_t core_id;
  uint32_t heap_free;
  uint32_t heap_min;
  uint32_t stk_filter;
  uint32_t stk_i2c;
  uint32_t stk_sd;
  uint32_t sd_written;
  uint32_t sd_hwm;
  uint32_t sd_last_seq;
  uint32_t resource_sample_ms;
};

extern BootBreadcrumb boot_breadcrumb;

const char* breadcrumb_phase_name(uint16_t phase);
void breadcrumb_init_after_reset(uint32_t reset_reason);
void breadcrumb_mark(uint16_t phase, uint32_t seq, uint16_t line);

#define BREADCRUMB_MARK(phase, seq) \
  breadcrumb_mark((uint16_t)(phase), (uint32_t)(seq), (uint16_t)__LINE__)

// ── Shared Telemetry Data ──────────────────────────────────────────────────
extern FilteredTelemetry shared_telemetry;
extern GpsData shared_gps_data;

// ── ESKF Instances ─────────────────────────────────────────────────────────
extern ESKF2D eskf;
extern ESKF_6D eskf6;

// ── GPS ENU Origin ─────────────────────────────────────────────────────────
// First valid GPS coordinate after boot. All subsequent coordinates are
// converted to metres relative to this point.
extern double gps_origin_lat;
extern double gps_origin_lon;
extern bool gps_origin_set;

// ── Madgwick Instance ──────────────────────────────────────────────────────
extern MadgwickAHRS ahrs;

// ── Calibration State ──────────────────────────────────────────────────────
// Written by calibrate_alignment() under telemetry_mutex,
// read by Task_Filter under telemetry_mutex.
extern float sin_phi, cos_phi;
extern float sin_theta, cos_theta;
extern float bias_gx, bias_gy, bias_gz;
extern float bias_ax, bias_ay, bias_az;
extern float mag_ref_ut_x, mag_ref_ut_y, mag_ref_ut_z;
extern bool mag_ref_valid;

// ── Peripheral Objects ─────────────────────────────────────────────────────
extern WiFiClient wifiClient;
extern PubSubClient mqttClient;

// ── WiFi / MQTT Configuration ──────────────────────────────────────────────
// Loaded from /wifi_config.txt on SD at boot. If absent, WiFi is disabled.
extern WifiCredential wifi_networks[MAX_WIFI_NETWORKS];
extern int wifi_network_count;
extern char cfg_mqtt_broker[64];
extern int cfg_mqtt_port;
extern char cfg_mqtt_topic[128];
extern char cfg_mqtt_client_id[32];
extern bool mqtt_enabled;

// ── Atomic Flags (cross-core thread safety) ────────────────────────────────
extern std::atomic<bool> wifi_enabled;      // written by loop() (triple click)
extern std::atomic<bool> sd_write_error;          // true if SD write fails at runtime
extern std::atomic<uint32_t> sd_records_written;  // records successfully written
extern std::atomic<uint32_t> sd_records_dropped;  // records dropped (queue full)
extern std::atomic<uint32_t> sd_flush_worst_us;   // worst-case flush() duration [us]
extern std::atomic<uint32_t> sd_flush_count;      // total flush() calls
extern std::atomic<uint32_t> sd_sync_worst_us;    // worst-case backend sync() duration [us]
extern std::atomic<uint32_t> sd_sync_count;       // total durable sync calls
extern std::atomic<uint32_t> sd_queue_hwm;        // queue high-water mark [records]
extern std::atomic<uint32_t> sd_partial_write_count; // partial-write events recovered/attempted
extern std::atomic<uint32_t> sd_stall_count;      // zero-progress write stalls
extern std::atomic<uint32_t> sd_reopen_count;     // SD file reopen attempts after stalls
extern std::atomic<uint32_t> sd_stall_worst_ms;   // worst per-record stall window [ms]
extern std::atomic<uint32_t> sd_write_overreport_count; // write() returned more bytes than requested
extern std::atomic<uint32_t> sd_write_zero_count; // File.write() returned zero bytes
extern std::atomic<uint32_t> sd_write_recovered_count; // batch writes recovered after zero-progress stalls
extern std::atomic<uint32_t> sd_last_stall_seq;   // first seq in the batch that last hit zero-progress
extern std::atomic<uint32_t> sd_last_written_seq; // last seq durably accepted by Task_SD_Writer
extern std::atomic<uint32_t> sd_size_mismatch_count; // persistent file size did not match write return accounting
extern std::atomic<uint32_t> sd_last_file_size;   // last file size observed after flush/reopen verification
extern std::atomic<uint32_t> sd_backend_id;       // active storage backend identifier
extern std::atomic<uint32_t> sd_prealloc_bytes;   // bytes currently preallocated by backend, if any
extern std::atomic<bool> sd_truncate_ok;          // last finalize/truncate status
extern std::atomic<uint32_t> sd_rollover_count;   // completed preallocated segment rollovers
extern std::atomic<uint32_t> sd_rollover_worst_ms; // worst rollover duration [ms]
extern std::atomic<uint32_t> sd_prealloc_count;   // preallocation calls completed
extern std::atomic<uint32_t> sd_prealloc_worst_ms; // worst preallocation duration [ms]
extern std::atomic<uint32_t> sd_boot_repair_count; // preallocated logs repaired at boot
extern std::atomic<uint32_t> sd_boot_repair_truncated_bytes; // bytes removed by boot repair
extern std::atomic<uint32_t> sd_boot_repair_worst_ms; // worst boot repair duration [ms]
extern std::atomic<uint32_t> sd_current_segment_bytes; // logical bytes in current log segment
extern std::atomic<uint32_t> sd_current_segment_capacity; // allocated bytes in current log segment
extern std::atomic<uint32_t> sd_enqueue_fail_count; // Task_Filter/Calibration failed to enqueue to SD queue
extern std::atomic<uint32_t> imu_queue_drop_count; // Task_I2C dropped oldest sample in diagnostic FIFO mode
extern std::atomic<uint32_t> gps_mutex_timeout_count; // Task_Filter GPS snapshot lock timeouts
extern std::atomic<bool> gps_stale;         // true if a previously acquired fix is now stale/lost
extern std::atomic<int> system_state;       // 0=idle, 1=countdown, 2=racing
extern std::atomic<bool> recalibration_pending; // v1.3.2: set by calibrate_alignment(), cleared by Task_Filter

// ── SD State ───────────────────────────────────────────────────────────────
extern bool sd_mounted;
extern bool sd_low_space;
extern char current_log_filename[64];

// ── Loop / UI State ────────────────────────────────────────────────────────
extern int prev_system_state;
extern int mqtt_cycle_count;
extern int display_cycle_count;
extern int countdown_cycles;
extern bool btn_locked;
extern int btn_press_cycles;
extern int btn_release_cycles;
extern int btn_click_count;
extern bool display_off;
extern bool display_confirm_pending;
extern int display_confirm_cycles;
extern char buf[1536];
