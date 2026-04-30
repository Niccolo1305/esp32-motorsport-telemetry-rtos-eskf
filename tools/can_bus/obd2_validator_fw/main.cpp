// SPDX-License-Identifier: GPL-3.0-or-later
// OBD-II PID Validator Firmware for M5Stack AtomS3R + Unit CAN (ISO)
//
// Phase 3 tool: runs in TWAI_MODE_NORMAL to send OBD-II PID requests
// and validate ECU responses.
//
// Build: pio run -e can_obd2_validator
// Flash: pio run -e can_obd2_validator -t upload
// Log:   pio device monitor
//
// WARNING: this firmware TRANSMITS on the CAN bus.
// Only use after Phase 1/2 sniffing is complete and validated.
// Test with engine running but vehicle stationary.

#include <M5Unified.h>
#include "driver/twai.h"

// ── GPIO Configuration ──────────────────────────────────────────────────────
// IMPORTANT: same GPIO pair as sniffer. Verify with your wiring.
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_6

// ── OBD-II PID Table ─────────────────────────────────────────────────────────
struct Obd2Pid {
    uint8_t pid;
    const char *name;
    const char *unit;
    bool two_byte;  // true if response uses A+B bytes
};

static const Obd2Pid pid_table[] = {
    {0x04, "Engine Load",       "%",    false},
    {0x05, "Coolant Temp",      "C",    false},
    {0x0B, "MAP",               "kPa",  false},
    {0x0C, "Engine RPM",        "rpm",  true },
    {0x0D, "Vehicle Speed",     "km/h", false},
    {0x0F, "Intake Air Temp",   "C",    false},
    {0x10, "MAF",               "g/s",  true },
    {0x11, "Throttle Pos",      "%",    false},
    {0x2F, "Fuel Level",        "%",    false},
    {0x33, "Barometric",        "kPa",  false},
    {0x46, "Ambient Temp",      "C",    false},
    {0x5A, "Accel Pedal",       "%",    false},
    {0x5C, "Oil Temp",          "C",    false},
    {0x5E, "Fuel Rate",         "L/h",  true },
};
static constexpr int NUM_PIDS = sizeof(pid_table) / sizeof(pid_table[0]);

// ── State ────────────────────────────────────────────────────────────────────
static int current_phase = 0;  // 0 = PID discovery, 1 = polling
static int poll_idx = 0;
static uint32_t supported_pids[3] = {0, 0, 0};  // PIDs 01-20, 21-40, 41-60
static uint32_t last_request_ms = 0;
static bool discovery_done = false;

// ── TWAI Init ────────────────────────────────────────────────────────────────
static bool init_twai() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
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

    Serial.println("[CAN] TWAI started at 500 kbps (NORMAL mode)");
    return true;
}

// ── OBD-II Request ──────────────────────────────────────────────────────────
static bool obd2_request(uint8_t mode, uint8_t pid) {
    twai_message_t req = {};
    req.identifier = 0x7DF;  // broadcast
    req.data_length_code = 8;
    req.data[0] = 0x02;      // 2 payload bytes
    req.data[1] = mode;
    req.data[2] = pid;

    esp_err_t err = twai_transmit(&req, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        Serial.printf("[OBD] TX failed for PID 0x%02X: %s\n",
                      pid, esp_err_to_name(err));
        return false;
    }
    return true;
}

// ── Decode PID Support Bitmask ──────────────────────────────────────────────
static void decode_pid_support(uint8_t base_pid, uint32_t bitmask) {
    Serial.printf("\n[OBD] Supported PIDs 0x%02X-0x%02X: 0x%08X\n",
                  base_pid + 1, base_pid + 0x20, (unsigned)bitmask);
    for (int i = 0; i < 32; i++) {
        if (bitmask & (1U << (31 - i))) {
            uint8_t pid = base_pid + 1 + i;
            Serial.printf("  PID 0x%02X: supported\n", pid);
        }
    }
}

// ── Decode PID Value ─────────────────────────────────────────────────────────
static void decode_pid_value(uint8_t pid, uint8_t a, uint8_t b) {
    float value = 0.0f;
    const char *name = "Unknown";
    const char *unit = "";

    switch (pid) {
        case 0x04: value = a * 100.0f / 255.0f; name = "Engine Load"; unit = "%"; break;
        case 0x05: value = a - 40.0f; name = "Coolant Temp"; unit = "C"; break;
        case 0x0B: value = (float)a; name = "MAP"; unit = "kPa"; break;
        case 0x0C: value = (a * 256.0f + b) / 4.0f; name = "Engine RPM"; unit = "rpm"; break;
        case 0x0D: value = (float)a; name = "Vehicle Speed"; unit = "km/h"; break;
        case 0x0F: value = a - 40.0f; name = "Intake Air Temp"; unit = "C"; break;
        case 0x10: value = (a * 256.0f + b) / 100.0f; name = "MAF"; unit = "g/s"; break;
        case 0x11: value = a * 100.0f / 255.0f; name = "Throttle"; unit = "%"; break;
        case 0x2F: value = a * 100.0f / 255.0f; name = "Fuel Level"; unit = "%"; break;
        case 0x33: value = (float)a; name = "Barometric"; unit = "kPa"; break;
        case 0x46: value = a - 40.0f; name = "Ambient Temp"; unit = "C"; break;
        case 0x5A: value = a * 100.0f / 255.0f; name = "Accel Pedal"; unit = "%"; break;
        case 0x5C: value = a - 40.0f; name = "Oil Temp"; unit = "C"; break;
        case 0x5E: value = (a * 256.0f + b) / 20.0f; name = "Fuel Rate"; unit = "L/h"; break;
        default:
            Serial.printf("[OBD] PID 0x%02X: A=0x%02X B=0x%02X (unknown)\n", pid, a, b);
            return;
    }
    Serial.printf("[OBD] %s: %.1f %s\n", name, value, unit);
}

// ── Process RX ──────────────────────────────────────────────────────────────
static void process_responses() {
    twai_message_t rx;
    while (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK) {
        // Only process OBD-II responses (0x7E8-0x7EF)
        if (rx.identifier < 0x7E8 || rx.identifier > 0x7EF) continue;
        if (rx.data_length_code < 3) continue;

        uint8_t resp_mode = rx.data[1];
        uint8_t resp_pid  = rx.data[2];

        if (resp_mode == 0x41) {
            // Mode 01 response
            if (resp_pid == 0x00 || resp_pid == 0x20 || resp_pid == 0x40) {
                // PID support bitmask
                if (rx.data_length_code >= 6) {
                    uint32_t bitmask = ((uint32_t)rx.data[3] << 24) |
                                      ((uint32_t)rx.data[4] << 16) |
                                      ((uint32_t)rx.data[5] << 8)  |
                                       (uint32_t)rx.data[6];
                    int idx = resp_pid / 0x20;
                    if (idx < 3) supported_pids[idx] = bitmask;
                    decode_pid_support(resp_pid, bitmask);
                }
            } else {
                // Data PID
                decode_pid_value(resp_pid, rx.data[3],
                                 rx.data_length_code > 4 ? rx.data[4] : 0);
            }
        }

        Serial.printf("[RAW] 0x%03X [%d] ", (unsigned)rx.identifier, rx.data_length_code);
        for (int i = 0; i < rx.data_length_code; i++) {
            Serial.printf("%02X ", rx.data[i]);
        }
        Serial.println();
    }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);
    delay(500);

    Serial.println("===========================================");
    Serial.println("  OBD-II PID Validator — Phase 3");
    Serial.println("  Qashqai J10 / Renault-Nissan platform");
    Serial.println("  Mode: NORMAL (TX + RX)");
    Serial.println("  WARNING: this firmware TRANSMITS on CAN");
    Serial.println("===========================================");

    if (!init_twai()) {
        Serial.println("[FATAL] TWAI init failed — halting");
        M5.Display.fillScreen(TFT_RED);
        M5.Display.drawString("CAN FAIL", 64, 64);
        while (true) { delay(1000); }
    }

    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_YELLOW);
    M5.Display.drawString("OBD-II", 5, 5);
    M5.Display.drawString("VALIDATOR", 5, 20);

    // Start with PID discovery
    Serial.println("\n[PHASE 0] Discovering supported PIDs...");
    last_request_ms = millis();
}

// ── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
    M5.update();
    uint32_t now_ms = millis();

    if (current_phase == 0) {
        // PID discovery phase: query PID 0x00, 0x20, 0x40
        static int discovery_step = 0;
        static const uint8_t discovery_pids[] = {0x00, 0x20, 0x40};

        if (now_ms - last_request_ms >= 500) {
            last_request_ms = now_ms;

            if (discovery_step < 3) {
                Serial.printf("[OBD] Querying PID support 0x%02X...\n",
                              discovery_pids[discovery_step]);
                obd2_request(0x01, discovery_pids[discovery_step]);
                discovery_step++;
            } else {
                Serial.println("\n[PHASE 0] Discovery complete.");
                Serial.println("[PHASE 1] Starting round-robin polling...");
                Serial.println("          Press button to print summary.\n");
                current_phase = 1;
                discovery_done = true;
            }
        }

        process_responses();

    } else if (current_phase == 1) {
        // Round-robin polling of known PIDs
        if (now_ms - last_request_ms >= 100) {  // ~10 Hz per PID cycle
            last_request_ms = now_ms;
            obd2_request(0x01, pid_table[poll_idx].pid);
            poll_idx = (poll_idx + 1) % NUM_PIDS;
        }

        process_responses();

        // Display update
        static uint32_t last_display_ms = 0;
        if (now_ms - last_display_ms >= 1000) {
            last_display_ms = now_ms;
            M5.Display.fillRect(0, 40, 128, 88, TFT_BLACK);
            M5.Display.setTextColor(TFT_GREEN);
            char buf[32];
            snprintf(buf, sizeof(buf), "Poll: %d/%d", poll_idx + 1, NUM_PIDS);
            M5.Display.drawString(buf, 5, 45);
            M5.Display.drawString("Running...", 5, 60);
        }
    }

    // Button: print full supported PID summary
    if (M5.BtnA.wasPressed() && discovery_done) {
        Serial.println("\n# ── Supported PID Summary ───────────────");
        for (int range = 0; range < 3; range++) {
            uint8_t base = range * 0x20;
            for (int i = 0; i < 32; i++) {
                if (supported_pids[range] & (1U << (31 - i))) {
                    uint8_t pid = base + 1 + i;
                    // Check if it's in our known table
                    const char *name = "?";
                    for (int j = 0; j < NUM_PIDS; j++) {
                        if (pid_table[j].pid == pid) {
                            name = pid_table[j].name;
                            break;
                        }
                    }
                    Serial.printf("#  PID 0x%02X: %s\n", pid, name);
                }
            }
        }
        Serial.println("# ────────────────────────────────────────\n");
    }
}
