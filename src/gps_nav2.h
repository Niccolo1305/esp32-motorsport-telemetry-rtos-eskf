// SPDX-License-Identifier: GPL-3.0-or-later
// gps_nav2.h - Shared NAV2-PVH validity and frame-conversion helpers.
#pragma once

#include <math.h>
#include <stdint.h>

#include "types.h"

static constexpr uint64_t NAV2_SPEED_FRESH_US = 150000ULL;
static constexpr float NAV2_MAX_SPEED_MS = 140.0f;
static constexpr float NAV2_MAX_S_ACC_MS = 10.0f;

struct Nav2PvhSelection {
  bool ok = false;
  uint64_t age_us = UINT64_MAX;
  float speed_ms = 0.0f;
  float speed_kmh = 0.0f;
  float vel_e_ms = 0.0f;
  float vel_n_ms = 0.0f;
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

  sel.ok = gps.nav_vel_valid >= 6 &&
           sel.age_us <= NAV2_SPEED_FRESH_US &&
           speed_ok &&
           acc_ok &&
           vel_components_ok;
  return sel;
}
