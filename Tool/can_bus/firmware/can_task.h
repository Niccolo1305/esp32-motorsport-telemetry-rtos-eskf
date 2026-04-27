// SPDX-License-Identifier: GPL-3.0-or-later
// can_task.h — CAN bus acquisition task header
//
// Snapshot dei dati ECU via CAN bus, protetto da can_mutex.
// Aggiornato da Task_CAN (Core 0), letto da Task_Filter (Core 1).
//
// Integration:
//   1. Copy can_task.h, can_task.cpp, can_decode.h into src/
//   2. Add CAN globals to globals.h / globals.cpp
//   3. Add TWAI init + xTaskCreatePinnedToCore in Telemetria.ino setup()
//   4. Read CanData snapshot in filter_task.cpp Phase 2
#pragma once

#include <stdint.h>

struct CanData {
    // Motore
    float engine_rpm;           // [rpm]
    float throttle_pct;         // [%] posizione pedale acceleratore
    float coolant_temp_c;       // [C]
    float intake_air_temp_c;    // [C]
    float engine_load_pct;      // [%]

    // Velocita ruote individuali [km/h]
    float wheel_fl_kmh;         // anteriore sinistra
    float wheel_fr_kmh;         // anteriore destra
    float wheel_rl_kmh;         // posteriore sinistra
    float wheel_rr_kmh;         // posteriore destra

    // Sterzo
    float steering_angle_deg;   // [gradi] CW positivo, centro = 0

    // Freni
    bool brake_pressed;

    // Diagnostica CAN
    uint32_t frames_received;   // totale frame ricevuti
    uint32_t decode_errors;     // frame con ID noto ma decodifica fallita
    uint64_t last_frame_us;     // timestamp ultimo frame ricevuto
    bool bus_active;            // true se frame ricevuti negli ultimi 500 ms

    // Timestamp per freshness check
    uint64_t rpm_us;
    uint64_t wheels_us;
    uint64_t steering_us;
};

// Task entry point — pin to Core 0, priority 2, stack 4096
void Task_CAN(void *pvParameters);
