// SPDX-License-Identifier: GPL-3.0-or-later
// globals.cpp — Definitions (storage) for all shared global state
//
// Every extern declared in globals.h is defined here exactly once.
// Initialization order within this file is guaranteed by the C++ standard.

#include "globals.h"

#include <esp_attr.h>

static constexpr uint32_t BOOT_BREADCRUMB_MAGIC = 0x54424C31UL; // "TBL1"
static constexpr uint16_t BOOT_BREADCRUMB_VERSION = 1;

// ── FreeRTOS Primitives ────────────────────────────────────────────────────
QueueHandle_t imuQueue;
QueueHandle_t sd_queue = NULL; // NULL until SD is mounted: guard in Task_Filter
SemaphoreHandle_t telemetry_mutex;
SemaphoreHandle_t gps_mutex;

// ── Task Handles ───────────────────────────────────────────────────────────
TaskHandle_t TaskI2CHandle;
TaskHandle_t TaskFilterHandle;
TaskHandle_t TaskSDHandle;
TaskHandle_t TaskGPSHandle;

RTC_NOINIT_ATTR BootBreadcrumb boot_breadcrumb;

const char* breadcrumb_phase_name(uint16_t phase) {
  switch (phase) {
  case BREADCRUMB_PHASE_BOOT_SETUP: return "BOOT_SETUP";
  case BREADCRUMB_PHASE_BOOT_IMU: return "BOOT_IMU";
  case BREADCRUMB_PHASE_BOOT_GPS: return "BOOT_GPS";
  case BREADCRUMB_PHASE_BOOT_SD: return "BOOT_SD";
  case BREADCRUMB_PHASE_BOOT_TASKS: return "BOOT_TASKS";
  case BREADCRUMB_PHASE_LOOP: return "LOOP";
  case BREADCRUMB_PHASE_GPS_UPDATE: return "GPS_UPDATE";
  case BREADCRUMB_PHASE_GPS_PUBLISH: return "GPS_PUBLISH";
  case BREADCRUMB_PHASE_I2C_UPDATE: return "I2C_UPDATE";
  case BREADCRUMB_PHASE_I2C_QUEUE: return "I2C_QUEUE";
  case BREADCRUMB_PHASE_FILTER_WAIT: return "FILTER_WAIT";
  case BREADCRUMB_PHASE_FILTER_SAMPLE: return "FILTER_SAMPLE";
  case BREADCRUMB_PHASE_FILTER_GPS: return "FILTER_GPS";
  case BREADCRUMB_PHASE_FILTER_SUPERVISOR: return "FILTER_SUPERVISOR";
  case BREADCRUMB_PHASE_FILTER_PREDICT: return "FILTER_PREDICT";
  case BREADCRUMB_PHASE_FILTER_CORRECT: return "FILTER_CORRECT";
  case BREADCRUMB_PHASE_FILTER_MOUNT_YAW: return "FILTER_MOUNT_YAW";
  case BREADCRUMB_PHASE_FILTER_SD_ENQUEUE: return "FILTER_SD_ENQUEUE";
  case BREADCRUMB_PHASE_SD_WAIT: return "SD_WAIT";
  case BREADCRUMB_PHASE_SD_BATCH: return "SD_BATCH";
  case BREADCRUMB_PHASE_SD_WRITE: return "SD_WRITE";
  case BREADCRUMB_PHASE_SD_FLUSH: return "SD_FLUSH";
  default: return "NONE";
  }
}

void breadcrumb_init_after_reset(uint32_t reset_reason) {
  const bool prev_valid =
      boot_breadcrumb.magic == BOOT_BREADCRUMB_MAGIC &&
      boot_breadcrumb.version == BOOT_BREADCRUMB_VERSION;
  const BootBreadcrumb prev = boot_breadcrumb;
  const uint32_t next_boot_count = prev_valid ? prev.boot_count + 1U : 1U;

  if (prev_valid) {
    Serial.printf("[BOOT] Prev breadcrumb: boot=%lu reset=%lu phase=%s(%u) "
                  "line=%u seq=%lu uptime=%lums heap=%lu min=%lu "
                  "stkF=%lu stkI2C=%lu stkSD=%lu sd_written=%lu sd_hwm=%lu sd_seq=%lu\n",
                  (unsigned long)prev.boot_count,
                  (unsigned long)reset_reason,
                  breadcrumb_phase_name(prev.phase),
                  (unsigned)prev.phase,
                  (unsigned)prev.line,
                  (unsigned long)prev.seq,
                  (unsigned long)prev.uptime_ms,
                  (unsigned long)prev.heap_free,
                  (unsigned long)prev.heap_min,
                  (unsigned long)prev.stk_filter,
                  (unsigned long)prev.stk_i2c,
                  (unsigned long)prev.stk_sd,
                  (unsigned long)prev.sd_written,
                  (unsigned long)prev.sd_hwm,
                  (unsigned long)prev.sd_last_seq);
  } else {
    Serial.printf("[BOOT] Prev breadcrumb: none reset=%lu\n",
                  (unsigned long)reset_reason);
  }

  boot_breadcrumb = {};
  boot_breadcrumb.magic = BOOT_BREADCRUMB_MAGIC;
  boot_breadcrumb.version = BOOT_BREADCRUMB_VERSION;
  boot_breadcrumb.boot_count = next_boot_count;
  boot_breadcrumb.last_reset_reason = reset_reason;
  boot_breadcrumb.phase = BREADCRUMB_PHASE_BOOT_SETUP;
  boot_breadcrumb.uptime_ms = millis();
  boot_breadcrumb.heap_free = ESP.getFreeHeap();
  boot_breadcrumb.heap_min = ESP.getMinFreeHeap();
}

void breadcrumb_mark(uint16_t phase, uint32_t seq, uint16_t line) {
  if (boot_breadcrumb.magic != BOOT_BREADCRUMB_MAGIC ||
      boot_breadcrumb.version != BOOT_BREADCRUMB_VERSION) {
    boot_breadcrumb.magic = BOOT_BREADCRUMB_MAGIC;
    boot_breadcrumb.version = BOOT_BREADCRUMB_VERSION;
  }

  const uint32_t now_ms = millis();
  boot_breadcrumb.phase = phase;
  boot_breadcrumb.seq = seq;
  boot_breadcrumb.line = line;
  boot_breadcrumb.core_id = (uint16_t)xPortGetCoreID();
  boot_breadcrumb.uptime_ms = now_ms;

  if (now_ms - boot_breadcrumb.resource_sample_ms >= 1000U ||
      boot_breadcrumb.resource_sample_ms == 0U) {
    boot_breadcrumb.resource_sample_ms = now_ms;
    boot_breadcrumb.heap_free = ESP.getFreeHeap();
    boot_breadcrumb.heap_min = ESP.getMinFreeHeap();
    boot_breadcrumb.stk_filter = TaskFilterHandle != NULL
        ? (uint32_t)uxTaskGetStackHighWaterMark(TaskFilterHandle) : 0U;
    boot_breadcrumb.stk_i2c = TaskI2CHandle != NULL
        ? (uint32_t)uxTaskGetStackHighWaterMark(TaskI2CHandle) : 0U;
    boot_breadcrumb.stk_sd = TaskSDHandle != NULL
        ? (uint32_t)uxTaskGetStackHighWaterMark(TaskSDHandle) : 0U;
    boot_breadcrumb.sd_written = sd_records_written.load(std::memory_order_relaxed);
    boot_breadcrumb.sd_hwm = sd_queue_hwm.load(std::memory_order_relaxed);
    boot_breadcrumb.sd_last_seq = sd_last_written_seq.load(std::memory_order_relaxed);
  }
}

// ── Shared Telemetry Data ──────────────────────────────────────────────────
FilteredTelemetry shared_telemetry;
GpsData shared_gps_data;

// ── ESKF Instances ─────────────────────────────────────────────────────────
ESKF2D eskf;
ESKF_6D eskf6;

// ── GPS ENU Origin ─────────────────────────────────────────────────────────
double gps_origin_lat = 0.0;
double gps_origin_lon = 0.0;
bool gps_origin_set = false;

// ── Madgwick Instance ──────────────────────────────────────────────────────
MadgwickAHRS ahrs(DT, 0.1f);

// ── Calibration State ──────────────────────────────────────────────────────
float sin_phi = 0.0f, cos_phi = 1.0f;
float sin_theta = 0.0f, cos_theta = 1.0f;
float bias_gx = 0.0f, bias_gy = 0.0f, bias_gz = 0.0f;
float bias_ax = 0.0f, bias_ay = 0.0f, bias_az = 0.0f;
float mag_ref_ut_x = 0.0f, mag_ref_ut_y = 0.0f, mag_ref_ut_z = 0.0f;
bool mag_ref_valid = false;

// ── EMA Previous Values ────────────────────────────────────────────────────
float prev_ax = 0.0f, prev_ay = 0.0f, prev_az = 0.0f;
float prev_gx = 0.0f, prev_gy = 0.0f, prev_gz = 0.0f;

// ── Peripheral Objects ─────────────────────────────────────────────────────
// wifiClient must be defined before mqttClient (constructor dependency).
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ── WiFi / MQTT Configuration ──────────────────────────────────────────────
WifiCredential wifi_networks[MAX_WIFI_NETWORKS];
int wifi_network_count = 0;
char cfg_mqtt_broker[64] = "";
int cfg_mqtt_port = 1883;
char cfg_mqtt_topic[128] = "";
char cfg_mqtt_client_id[32] = "";
bool mqtt_enabled = false;

// ── Atomic Flags (cross-core thread safety) ────────────────────────────────
std::atomic<bool> wifi_enabled{true};
std::atomic<bool> sd_write_error{false};
std::atomic<uint32_t> sd_records_written{0};
std::atomic<uint32_t> sd_records_dropped{0};
std::atomic<uint32_t> sd_flush_worst_us{0};
std::atomic<uint32_t> sd_flush_count{0};
std::atomic<uint32_t> sd_sync_worst_us{0};
std::atomic<uint32_t> sd_sync_count{0};
std::atomic<uint32_t> sd_queue_hwm{0};
std::atomic<uint32_t> sd_partial_write_count{0};
std::atomic<uint32_t> sd_stall_count{0};
std::atomic<uint32_t> sd_reopen_count{0};
std::atomic<uint32_t> sd_stall_worst_ms{0};
std::atomic<uint32_t> sd_write_overreport_count{0};
std::atomic<uint32_t> sd_write_zero_count{0};
std::atomic<uint32_t> sd_write_recovered_count{0};
std::atomic<uint32_t> sd_last_stall_seq{0};
std::atomic<uint32_t> sd_last_written_seq{0};
std::atomic<uint32_t> sd_size_mismatch_count{0};
std::atomic<uint32_t> sd_last_file_size{0};
std::atomic<uint32_t> sd_backend_id{0};
std::atomic<uint32_t> sd_prealloc_bytes{0};
std::atomic<bool> sd_truncate_ok{false};
std::atomic<uint32_t> sd_rollover_count{0};
std::atomic<uint32_t> sd_rollover_worst_ms{0};
std::atomic<uint32_t> sd_prealloc_count{0};
std::atomic<uint32_t> sd_prealloc_worst_ms{0};
std::atomic<uint32_t> sd_boot_repair_count{0};
std::atomic<uint32_t> sd_boot_repair_truncated_bytes{0};
std::atomic<uint32_t> sd_boot_repair_worst_ms{0};
std::atomic<uint32_t> sd_current_segment_bytes{0};
std::atomic<uint32_t> sd_current_segment_capacity{0};
std::atomic<uint32_t> sd_enqueue_fail_count{0};
std::atomic<uint32_t> imu_queue_drop_count{0};
std::atomic<uint32_t> gps_mutex_timeout_count{0};
std::atomic<bool> gps_stale{false};
std::atomic<int> system_state{0};
std::atomic<bool> recalibration_pending{false}; // v1.3.2

// ── SD State ───────────────────────────────────────────────────────────────
bool sd_mounted = false;
bool sd_low_space = false;
char current_log_filename[64] = "";

// ── Loop / UI State ────────────────────────────────────────────────────────
int prev_system_state = -1;
int mqtt_cycle_count = 0;
int display_cycle_count = 0;
int countdown_cycles = 0;
bool btn_locked = false;
int btn_press_cycles = 0;
int btn_release_cycles = 0;
int btn_click_count = 0;
bool display_off = false;
bool display_confirm_pending = false;
int display_confirm_cycles = 0;
char buf[1536];
