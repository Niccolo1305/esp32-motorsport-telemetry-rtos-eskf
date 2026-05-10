// SPDX-License-Identifier: GPL-3.0-or-later
// gps_nav2.h - Shared NAV2-PVH validity and frame-conversion helpers.
#pragma once

#include <math.h>
#include <stdint.h>

#include "types.h"

static constexpr uint64_t NAV2_SPEED_FRESH_US = 150000ULL;
static constexpr float NAV2_MAX_SPEED_MS = 140.0f;
static constexpr float NAV2_MAX_S_ACC_MS = 10.0f;
static constexpr float NAV2_MAX_H_ACC_M = 100.0f;

enum GpsSupervisorState : uint8_t {
  GPS_SUPERVISOR_OK = 0,
  GPS_SUPERVISOR_SUSPECT = 1,
  GPS_SUPERVISOR_QUARANTINE = 2,
  GPS_SUPERVISOR_RECOVERING = 3,
};

enum GpsSupervisorReason : uint16_t {
  GPS_SUP_REASON_NONE = 0x0000,
  GPS_SUP_REASON_NMEA_NAV2_POS_MISMATCH = 0x0001,
  GPS_SUP_REASON_IMPLIED_SPEED_BAD = 0x0002,
  GPS_SUP_REASON_LOW_SPEED_POSITION_JUMP = 0x0004,
  GPS_SUP_REASON_BAD_FIXFLAGS = 0x0008,
  GPS_SUP_REASON_BAD_HACC_HDOP_SATS = 0x0010,
  GPS_SUP_REASON_ESKF_INNOVATION_BAD = 0x0020,
  GPS_SUP_REASON_POSITION_FROZEN_WHILE_MOVING = 0x0040,
  GPS_SUP_REASON_NAV2_STALE = 0x0080,
};

struct Nav2PvhSelection {
  bool ok = false;
  bool pos_ok = false;
  uint64_t age_us = UINT64_MAX;
  float speed_ms = 0.0f;
  float speed_kmh = 0.0f;
  float vel_e_ms = 0.0f;
  float vel_n_ms = 0.0f;
  double lat = 0.0;
  double lon = 0.0;
  float h_acc_m = 0.0f;
  uint8_t fix_flags = 0;
  // ESKF convention: angle from ENU East toward North, atan2(north, east).
  float heading_eskf_rad = 0.0f;
};

static inline Nav2PvhSelection evaluate_nav2_pvh(const GpsData& gps,
                                                 uint64_t now_us) {
  Nav2PvhSelection sel;
  sel.speed_ms = gps.nav_speed2d;
  sel.speed_kmh = gps.nav_speed2d * 3.6f;
  sel.vel_e_ms = gps.nav_vel_e;
  sel.vel_n_ms = gps.nav_vel_n;
  sel.lat = gps.nav_lat;
  sel.lon = gps.nav_lon;
  sel.h_acc_m = gps.nav_h_acc;
  sel.fix_flags = gps.nav_fix_flags;

  const bool vel_components_ok = isfinite(sel.vel_e_ms) && isfinite(sel.vel_n_ms);
  if (vel_components_ok) {
    sel.heading_eskf_rad = atan2f(sel.vel_n_ms, sel.vel_e_ms);
  }

  if (gps.nav_fix_us == 0ULL || now_us < gps.nav_fix_us) {
    return sel;
  }
  sel.age_us = now_us - gps.nav_fix_us;

  const bool speed_ok = isfinite(sel.speed_ms) &&
                        sel.speed_ms >= 0.0f &&
                        sel.speed_ms < NAV2_MAX_SPEED_MS;
  const bool acc_ok = isfinite(gps.nav_s_acc) &&
                      gps.nav_s_acc >= 0.0f &&
                      gps.nav_s_acc < NAV2_MAX_S_ACC_MS;
  const bool pos_fields_ok = isfinite(sel.lat) &&
                             isfinite(sel.lon) &&
                             fabs(sel.lat) > 1.0e-9 &&
                             fabs(sel.lon) > 1.0e-9 &&
                             isfinite(sel.h_acc_m) &&
                             sel.h_acc_m >= 0.0f &&
                             sel.h_acc_m < NAV2_MAX_H_ACC_M;

  sel.ok = gps.nav_vel_valid >= 6 &&
           sel.age_us <= NAV2_SPEED_FRESH_US &&
           speed_ok &&
           acc_ok &&
           vel_components_ok;
  sel.pos_ok = sel.fix_flags >= 6 &&
               sel.age_us <= NAV2_SPEED_FRESH_US &&
               pos_fields_ok;
  return sel;
}
