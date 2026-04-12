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
#include <TinyGPSPlus.h>
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

// ── EMA Previous Values ────────────────────────────────────────────────────
// Read/written by Task_Filter under telemetry_mutex.
extern float prev_ax, prev_ay, prev_az;
extern float prev_gx, prev_gy, prev_gz;

// ── Peripheral Objects ─────────────────────────────────────────────────────
extern WiFiClient wifiClient;
extern PubSubClient mqttClient;
extern TinyGPSPlus gps;
extern HardwareSerial gpsSerial;

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
extern std::atomic<bool> sd_write_error;    // true if SD write fails at runtime
extern std::atomic<uint32_t> sd_records_written; // records-written counter (MQTT heartbeat)
extern std::atomic<uint32_t> gps_uart_overflow_count; // GPS UART overflow (ISR callback)
extern std::atomic<bool> gps_stale;         // true if last GPS fix > 5 s ago
extern std::atomic<int> system_state;       // 0=idle, 1=countdown, 2=racing
extern std::atomic<bool> recalibration_pending; // v1.3.2: set by calibrate_alignment(), cleared by Task_Filter

// ── SD State ───────────────────────────────────────────────────────────────
extern bool sd_mounted;
extern bool sd_low_space;
extern char current_log_filename[32];

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
extern char buf[384];
