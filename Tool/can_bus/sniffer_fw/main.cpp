// SPDX-License-Identifier: GPL-3.0-or-later
// CAN Bus Sniffer Firmware for M5Stack AtomS3R + Unit CAN (ISO)
//
// Phase 1 discovery tool: runs in TWAI_MODE_LISTEN_ONLY at 500 kbps.
// Logs all CAN frames to Serial in CSV format and optionally to SD card.
//
// Build: pio run -e can_sniffer
// Flash: pio run -e can_sniffer -t upload
// Log:   pio device monitor | tee can_capture.csv
//
// Output format:
//   timestamp_us,CAN_ID,DLC,B0,B1,...,B7
//
// Capture procedure (see CAN_Bus_Implementation_Manual.md Sections 10.2):
//   Session 1: key ON, engine OFF — identify steering angle ID
//   Session 2: engine idle — identify RPM ID
//   Session 3: blip throttle in neutral — correlate RPM + throttle
//   Session 4: slow drive with turns — identify wheel speeds
//   Session 5: road drive — full validation

#include <M5Unified.h>
#include "driver/twai.h"
#include <SD.h>
#include <SPI.h>

// ── GPIO Configuration ──────────────────────────────────────────────────────
// IMPORTANT: verify these match your Hub Grove wiring.
// If GPS occupies GPIO 1/2 (Grove Port A), CAN must use another pair.
// AtomS3R Grove port B candidates: GPIO 5/6, 7/8, etc.
// Adjust before compiling!
#define CAN_TX_PIN  GPIO_NUM_5   // Grove IO1 -> TXD of CA-IS3050G
#define CAN_RX_PIN  GPIO_NUM_6   // Grove IO2 -> RXD of CA-IS3050G

// SD card SPI (AtomS3R)
#define SD_SCK   7
#define SD_MISO  8
#define SD_MOSI  6
#define SD_CS    5

// ── Globals ──────────────────────────────────────────────────────────────────
static bool sd_logging = false;
static File log_file;
static uint32_t frame_count = 0;
static uint32_t last_status_ms = 0;

// Unique CAN ID tracker for summary
static constexpr int MAX_UNIQUE_IDS = 256;
static uint32_t seen_ids[MAX_UNIQUE_IDS];
static uint32_t seen_counts[MAX_UNIQUE_IDS];
static int unique_id_count = 0;

static void track_id(uint32_t id) {
    for (int i = 0; i < unique_id_count; i++) {
        if (seen_ids[i] == id) {
            seen_counts[i]++;
            return;
        }
    }
    if (unique_id_count < MAX_UNIQUE_IDS) {
        seen_ids[unique_id_count] = id;
        seen_counts[unique_id_count] = 1;
        unique_id_count++;
    }
}

// ── SD Init ──────────────────────────────────────────────────────────────────
static void init_sd() {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
        Serial.println("[SD] Mount failed — logging to Serial only");
        return;
    }

    // Find next available filename
    char fname[32];
    for (int i = 1; i < 1000; i++) {
        snprintf(fname, sizeof(fname), "/can_sniff_%03d.csv", i);
        if (!SD.exists(fname)) {
            log_file = SD.open(fname, FILE_WRITE);
            if (log_file) {
                log_file.println("timestamp_us,CAN_ID,DLC,B0,B1,B2,B3,B4,B5,B6,B7");
                sd_logging = true;
                Serial.printf("[SD] Logging to %s\n", fname);
            }
            break;
        }
    }
}

// ── TWAI Init ────────────────────────────────────────────────────────────────
static bool init_twai() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    g_config.rx_queue_len = 32;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[CAN] Driver install failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[CAN] Start failed: %s\n", esp_err_to_name(err));
        return false;
    }

    Serial.println("[CAN] Sniffer started in LISTEN_ONLY mode at 500 kbps");
    return true;
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    delay(500);

    Serial.println("===========================================");
    Serial.println("  CAN Bus Sniffer — Phase 1 Discovery");
    Serial.println("  Qashqai J10 / Renault-Nissan platform");
    Serial.println("  Mode: LISTEN_ONLY (zero TX, zero ACK)");
    Serial.println("===========================================");

    init_sd();

    if (!init_twai()) {
        Serial.println("[FATAL] TWAI init failed — halting");
        M5.Display.fillScreen(TFT_RED);
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.drawString("CAN FAIL", 64, 64);
        while (true) { delay(1000); }
    }

    // Display
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_GREEN);
    M5.Display.setTextSize(1);
    M5.Display.drawString("CAN SNIFF", 5, 5);
    M5.Display.drawString("LISTEN_ONLY", 5, 20);

    // Print CSV header on Serial
    Serial.println("timestamp_us,CAN_ID,DLC,B0,B1,B2,B3,B4,B5,B6,B7");

    last_status_ms = millis();
}

// ── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
    twai_message_t msg;

    // Drain all available frames
    while (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        uint64_t now_us = esp_timer_get_time();
        frame_count++;
        track_id(msg.identifier);

        // Build CSV line
        char line[128];
        int pos = snprintf(line, sizeof(line), "%llu,0x%03X,%d",
                           (unsigned long long)now_us,
                           (unsigned)msg.identifier,
                           msg.data_length_code);
        for (int i = 0; i < 8; i++) {
            if (i < msg.data_length_code) {
                pos += snprintf(line + pos, sizeof(line) - pos, ",%02X", msg.data[i]);
            } else {
                pos += snprintf(line + pos, sizeof(line) - pos, ",00");
            }
        }

        Serial.println(line);

        if (sd_logging) {
            log_file.println(line);
        }
    }

    // Periodic SD flush and status update
    uint32_t now_ms = millis();
    if (now_ms - last_status_ms >= 2000) {
        last_status_ms = now_ms;

        if (sd_logging) {
            log_file.flush();
        }

        // Display status
        M5.Display.fillRect(0, 40, 128, 88, TFT_BLACK);
        char status[64];
        snprintf(status, sizeof(status), "Frames: %lu", (unsigned long)frame_count);
        M5.Display.drawString(status, 5, 45);
        snprintf(status, sizeof(status), "IDs: %d", unique_id_count);
        M5.Display.drawString(status, 5, 60);
        if (sd_logging) {
            M5.Display.setTextColor(TFT_GREEN);
            M5.Display.drawString("SD: OK", 5, 75);
        } else {
            M5.Display.setTextColor(TFT_RED);
            M5.Display.drawString("SD: OFF", 5, 75);
        }
        M5.Display.setTextColor(TFT_GREEN);

        // Serial summary
        Serial.printf("# STATUS: %lu frames, %d unique IDs\n",
                       (unsigned long)frame_count, unique_id_count);
    }

    // Button press: print summary of all seen IDs
    M5.update();
    if (M5.BtnA.wasPressed()) {
        Serial.println("\n# ── CAN ID Summary ──────────────────────");
        Serial.println("# CAN_ID     Count");
        for (int i = 0; i < unique_id_count; i++) {
            Serial.printf("# 0x%03X      %lu\n",
                          (unsigned)seen_ids[i],
                          (unsigned long)seen_counts[i]);
        }
        Serial.println("# ────────────────────────────────────────\n");
    }
}
