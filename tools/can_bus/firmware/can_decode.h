// SPDX-License-Identifier: GPL-3.0-or-later
// can_decode.h — CAN ID decode table for Nissan Qashqai J10 (K9K / M9R)
//
// !! DISCLAIMER !!
// All CAN IDs, byte layouts, and scale factors come from community
// reverse-engineering. They MUST be verified on-vehicle via the sniffer
// firmware (Phase 1) before trusting in production.
//
// Confidence levels:
//   ALTA  — confirmed on multiple vehicles of the same platform
//   MEDIA — reported on related vehicles, layout probable but unverified
//   BASSA — single source or generic Nissan platform
//
// After running Phase 1 sniffing sessions, update the CAN_ID_* constants
// below with the verified values from your specific vehicle.
#pragma once

#include <stdint.h>

// ── CAN ID Constants ─────────────────────────────────────────────────────────
// Update these after Phase 1 on-vehicle sniffing confirms the real IDs.

// Motore / Powertrain
static constexpr uint32_t CAN_ID_RPM           = 0x180;  // ALTA  — B0-B1: (val)/4 = rpm
static constexpr uint32_t CAN_ID_THROTTLE      = 0x280;  // MEDIA — B0-B1: val*100/1024 = %
static constexpr uint32_t CAN_ID_TEMP          = 0x182;  // MEDIA — B0: coolant-40, B1: intake-40
static constexpr uint32_t CAN_ID_VEHICLE_SPEED = 0x354;  // MEDIA — B0-B1: val/100 = km/h

// Velocita ruote (ABS/ESP — Bosch ABS 8.x layout)
static constexpr uint32_t CAN_ID_WHEELS_FRONT  = 0x284;  // ALTA  — B0-B1: FL, B2-B3: FR, *0.01 km/h
static constexpr uint32_t CAN_ID_WHEELS_REAR   = 0x285;  // ALTA  — B0-B1: RL, B2-B3: RR, *0.01 km/h

// Angolo sterzo (SAS / EPS)
static constexpr uint32_t CAN_ID_STEERING      = 0x002;  // MEDIA — B0-B1: int16*0.1 = gradi

// Freni / Stabilita
static constexpr uint32_t CAN_ID_BRAKE         = 0x35D;  // MEDIA — B0 bit 0: freno premuto
static constexpr uint32_t CAN_ID_YAW_LAT       = 0x286;  // BASSA — B0-B1: yaw*0.01, B2-B3: lat_a*0.01

// Trasmissione
static constexpr uint32_t CAN_ID_GEAR          = 0x421;  // BASSA — B0: numero marcia
static constexpr uint32_t CAN_ID_ATF_TEMP      = 0x174;  // BASSA — B0: val-40 (solo automatico)

// ── OBD-II Diagnostic IDs ────────────────────────────────────────────────────
static constexpr uint32_t OBD2_REQUEST_BROADCAST = 0x7DF;
static constexpr uint32_t OBD2_REQUEST_ECU       = 0x7E0;
static constexpr uint32_t OBD2_RESPONSE_ECU      = 0x7E8;

// ── Costanti fisiche Qashqai J10 ────────────────────────────────────────────
static constexpr float QJ10_WHEELBASE_M      = 2.630f;  // passo [m]
static constexpr float QJ10_TRACK_WIDTH_M    = 1.540f;  // carreggiata [m]
static constexpr float QJ10_STEERING_RATIO   = 16.0f;   // rapporto sterzo (verificare)
static constexpr float QJ10_WHEEL_DIAMETER_M = 0.686f;  // 215/65 R16
