// SPDX-License-Identifier: GPL-3.0-or-later
// globals.cpp — Definitions (storage) for all shared global state
//
// Every extern declared in globals.h is defined here exactly once.
// Initialization order within this file is guaranteed by the C++ standard.

#include "globals.h"

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
std::atomic<uint32_t> sd_queue_hwm{0};
std::atomic<uint32_t> sd_partial_write_count{0};
std::atomic<uint32_t> sd_stall_count{0};
std::atomic<uint32_t> sd_reopen_count{0};
std::atomic<uint32_t> sd_stall_worst_ms{0};
std::atomic<uint32_t> sd_write_overreport_count{0};
std::atomic<uint32_t> gps_mutex_timeout_count{0};
std::atomic<bool> gps_stale{false};
std::atomic<int> system_state{0};
std::atomic<bool> recalibration_pending{false}; // v1.3.2

// ── SD State ───────────────────────────────────────────────────────────────
bool sd_mounted = false;
bool sd_low_space = false;
char current_log_filename[32] = "";

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
char buf[512];
