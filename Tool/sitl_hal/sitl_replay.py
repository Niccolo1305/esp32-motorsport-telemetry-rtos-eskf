#!/usr/bin/env python3
"""
Offline SITL replay for Telemetria v1.4.x logs.

This script replays the same IMU pipeline implemented in `src/filter_task.cpp`
using the Level 1 sensor-frame channels logged in v1.4.0+:

  sensor_* -> ellipsoid -> rotate_3d -> mounting bias -> VGPL + Madgwick
            -> gravity removal -> ZARU -> ESKF 5D -> end-of-pipe EMA

Input can be either:
  - a CSV produced by `Tool/bin_to_csv.py`
  - a v1.4.x `.bin` file, converted on the fly through `bin_to_csv.py`

Typical usage:
  python Tool/sitl_replay.py Tool/tel_5_test.csv
  python Tool/sitl_replay.py Tool/tel_5.bin --output Tool/tel_5_sitl.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, TextIO, Tuple

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / "Tool"
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import bin_to_csv  # type: ignore  # local tool module


# Mirrors src/config.h / src/eskf.h
CALIB_B = np.array([-0.00125, +0.00429, -0.06491], dtype=np.float64)
CALIB_W = np.array(
    [
        [+1.000824, -0.000511, -0.001575],
        [-0.000511, +1.000989, -0.000132],
        [-0.001575, -0.000132, +1.003466],
    ],
    dtype=np.float64,
)

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
G_ACCEL = 9.80665
EARTH_RADIUS_M = 6371000.0
DT_DEFAULT = 0.02

ALPHA = 0.06
VAR_BUF_SIZE = 50
VAR_STILLNESS_GZ_THRESHOLD = 0.25
VAR_STILLNESS_GXY_THRESHOLD = 0.35
ZUPT_GPS_MAX_KMH = 2.0

NHC_R = 0.5
NHC_MIN_SPEED_MS = 1.4
NHC_MAX_LAT_G = 0.5

STRAIGHT_ALPHA = 0.01
STRAIGHT_COG_MAX_RAD = 0.10
STRAIGHT_MIN_SPEED_KMH = 40.0
STRAIGHT_MAX_LAT_G = 0.05
COG_MIN_BASELINE_M = 15.0

VGPL_NORM_GATE = 0.15
VGPL_RATE_LIMIT = 0.15
VGPL_BETA = 0.1
VGPL_BETA_FLOOR = 0.005
LEGACY_GPS_FIX_PERIOD_US = 90_000

INT_COLUMNS = {"t_us", "lap", "gps_sats", "zaru_flags", "gps_fix_us", "gps_valid"}
REQUIRED_COLUMNS = {
    "t_us",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "gps_lat",
    "gps_lon",
    "gps_speed_kmh",
    "gps_sats",
    "gps_hdop",
    "kf_x",
    "kf_y",
    "kf_vel",
    "kf_heading",
    "raw_ax",
    "raw_ay",
    "raw_az",
    "raw_gx",
    "raw_gy",
    "raw_gz",
    "sensor_ax",
    "sensor_ay",
    "sensor_az",
    "sensor_gx",
    "sensor_gy",
    "sensor_gz",
}

HEADER_UNIT_RE = re.compile(r"\s*\([^)]*\)\s*$")
CALIB_RE = re.compile(r"^#\s*(CALIB_BOOT|CALIB_UPDATE):\s*(.*)$")
CALIB_KV_RE = re.compile(r"([a-z_]+)=([+-]?(?:\d+(?:\.\d*)?|\.\d+))")


@dataclass
class CalibrationState:
    sin_phi: float
    cos_phi: float
    sin_theta: float
    cos_theta: float
    bias_ax: float
    bias_ay: float
    bias_az: float
    bias_gx: float
    bias_gy: float
    bias_gz: float

    @property
    def phi_rad(self) -> float:
        return math.atan2(self.sin_phi, self.cos_phi)

    @property
    def theta_rad(self) -> float:
        return math.atan2(self.sin_theta, self.cos_theta)


@dataclass
class GpsState:
    lat: float = 0.0
    lon: float = 0.0
    speed_kmh: float = 0.0
    alt_m: float = 0.0
    sats: int = 0
    hdop: float = 99.9
    valid: bool = False
    epoch: int = 0
    fix_us: int = 0


@dataclass
class RunningErrorStats:
    count: int = 0
    sum_abs: float = 0.0
    sum_sq: float = 0.0
    max_abs: float = 0.0

    def add(self, expected: float, actual: float, wrap_pi: bool = False) -> None:
        err = actual - expected
        if wrap_pi:
            err = wrap_angle(err)
        abs_err = abs(err)
        self.count += 1
        self.sum_abs += abs_err
        self.sum_sq += err * err
        if abs_err > self.max_abs:
            self.max_abs = abs_err

    @property
    def mae(self) -> float:
        return self.sum_abs / self.count if self.count else 0.0

    @property
    def rmse(self) -> float:
        return math.sqrt(self.sum_sq / self.count) if self.count else 0.0


def fast_inv_sqrt(x: float) -> float:
    if x <= 0.0:
        return 0.0
    return 1.0 / math.sqrt(x)


def wrap_angle(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def rate_limit(value: float, prev: float, limit: float) -> float:
    if value > prev + limit:
        return prev + limit
    if value < prev - limit:
        return prev - limit
    return value


def rotate_3d(x: float, y: float, z: float,
              calibration: CalibrationState) -> Tuple[float, float, float]:
    y1 = y * calibration.cos_phi - z * calibration.sin_phi
    z1 = y * calibration.sin_phi + z * calibration.cos_phi
    x2 = x * calibration.cos_theta + z1 * calibration.sin_theta
    z2 = -x * calibration.sin_theta + z1 * calibration.cos_theta
    return x2, y1, z2


def wgs84_to_enu(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    east_m = dlon * math.cos(math.radians(lat0)) * EARTH_RADIUS_M
    north_m = dlat * EARTH_RADIUS_M
    return east_m, north_m


class ESKF2D:
    """Numpy port of src/eskf.h::ESKF2D."""

    def __init__(self) -> None:
        self.qd = np.zeros((5, 5), dtype=np.float64)
        self.qd[2, 2] = 1.0e-2
        self.qd[3, 3] = 1.0e-2
        self.qd[4, 4] = 5.0e-4
        self.reset()

    def reset(self) -> None:
        self.x = np.zeros(5, dtype=np.float64)
        self.p = np.zeros((5, 5), dtype=np.float64)
        self.p[0, 0] = 100.0
        self.p[1, 1] = 100.0
        self.p[2, 2] = 1.0
        self.p[3, 3] = 1.0
        self.p[4, 4] = 0.1
        self.has_last_gps = False
        self.last_gps_east = 0.0
        self.last_gps_north = 0.0

    def predict(self, ax_body: float, ay_body: float, gz_rad: float,
                dt: float, is_stationary: bool) -> None:
        ax_ms2 = ax_body * G_ACCEL
        ay_ms2 = ay_body * G_ACCEL

        if is_stationary:
            self.x[2] = 0.0
            self.x[3] = 0.0

        vx, vy, theta = self.x[2], self.x[3], self.x[4]
        theta_new = wrap_angle(theta + gz_rad * dt)

        cos_th = math.cos(theta)
        sin_th = math.sin(theta)
        ax_enu = ax_ms2 * cos_th - ay_ms2 * sin_th
        ay_enu = ax_ms2 * sin_th + ay_ms2 * cos_th

        vx_new = vx + ax_enu * dt
        vy_new = vy + ay_enu * dt
        self.x[0] += vx * dt + 0.5 * ax_enu * dt * dt
        self.x[1] += vy * dt + 0.5 * ay_enu * dt * dt
        self.x[2] = vx_new
        self.x[3] = vy_new
        self.x[4] = theta_new

        dax_dth = -ax_ms2 * sin_th - ay_ms2 * cos_th
        day_dth = ax_ms2 * cos_th - ay_ms2 * sin_th

        fj = np.eye(5, dtype=np.float64)
        fj[0, 2] = dt
        fj[0, 4] = 0.5 * dax_dth * dt * dt
        fj[1, 3] = dt
        fj[1, 4] = 0.5 * day_dth * dt * dt
        fj[2, 4] = dax_dth * dt
        fj[3, 4] = day_dth * dt

        dt_scale = dt / DT_DEFAULT
        self.p = fj @ self.p @ fj.T
        self.p[2, 2] += self.qd[2, 2] * dt_scale
        self.p[3, 3] += self.qd[3, 3] * dt_scale
        self.p[4, 4] += self.qd[4, 4] * dt_scale
        self._symmetrize()

    def correct(self, gps_east_m: float, gps_north_m: float, gps_speed_kmh: float,
                hdop: float = 1.0) -> None:
        if hdop < 0.5 or hdop > 50.0:
            hdop = 1.0

        r_dynamic = 0.05 * (hdop * hdop)
        r_pos = np.diag([r_dynamic, r_dynamic]).astype(np.float64)

        z = np.array([gps_east_m, gps_north_m], dtype=np.float64)
        hx = self.x[:2].copy()
        y = z - hx
        s = self.p[:2, :2] + r_pos
        det = float(np.linalg.det(s))
        if abs(det) > 1e-10:
            s_inv = np.linalg.inv(s)
            d2 = float(y.T @ s_inv @ y)
            if d2 > 11.83:
                r_pos *= 50.0
                s = self.p[:2, :2] + r_pos
                det = float(np.linalg.det(s))
                if abs(det) <= 1e-10:
                    s = None
                else:
                    s_inv = np.linalg.inv(s)
            if s is not None:
                ph_t = self.p[:, :2]
                k = ph_t @ s_inv
                self.x = self.x + k @ y
                i_kh = np.eye(5, dtype=np.float64)
                i_kh[:, :2] -= k
                self.p = i_kh @ self.p @ i_kh.T + k @ r_pos @ k.T
                self._symmetrize()

        gps_speed_ms = gps_speed_kmh / 3.6
        eskf_speed = self.speed_ms()
        if gps_speed_kmh > 5.0 and eskf_speed > 0.1:
            h_v = np.zeros((1, 5), dtype=np.float64)
            h_v[0, 2] = self.x[2] / eskf_speed
            h_v[0, 3] = self.x[3] / eskf_speed
            y_v = gps_speed_ms - eskf_speed
            r_v = 1.0
            s_v = float((h_v @ self.p @ h_v.T)[0, 0]) + r_v
            if abs(s_v) > 1e-10:
                k_v = (self.p @ h_v.T)[:, 0] / s_v
                self.x = self.x + k_v * y_v
                i_kh_v = np.eye(5, dtype=np.float64) - np.outer(k_v, h_v[0])
                kk_t_v = np.outer(k_v, k_v)
                self.p = i_kh_v @ self.p @ i_kh_v.T + kk_t_v * r_v
                self._symmetrize()

        if self.has_last_gps and gps_speed_kmh > 5.0:
            dx = gps_east_m - self.last_gps_east
            dy = gps_north_m - self.last_gps_north
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 1.0:
                gps_heading = math.atan2(dy, dx)
                y_th = wrap_angle(gps_heading - self.x[4])
                h_th = np.zeros((1, 5), dtype=np.float64)
                h_th[0, 4] = 1.0
                sigma_pos = 1.2
                r_th = (sigma_pos * sigma_pos) / (dist * dist)
                r_th = max(0.005, min(0.300, r_th))
                s_th = float((h_th @ self.p @ h_th.T)[0, 0]) + r_th
                if abs(s_th) > 1e-10:
                    k_th = (self.p @ h_th.T)[:, 0] / s_th
                    self.x = self.x + k_th * y_th
                    self.x[4] = wrap_angle(self.x[4])
                    i_kh_th = np.eye(5, dtype=np.float64) - np.outer(k_th, h_th[0])
                    kk_t_th = np.outer(k_th, k_th)
                    self.p = i_kh_th @ self.p @ i_kh_th.T + kk_t_th * r_th
                    self._symmetrize()

        self.last_gps_east = gps_east_m
        self.last_gps_north = gps_north_m
        self.has_last_gps = True
        self.x[4] = wrap_angle(self.x[4])

    def correct_nhc(self, r_nhc: float) -> None:
        vx, vy, theta = self.x[2], self.x[3], self.x[4]
        sin_th = math.sin(theta)
        cos_th = math.cos(theta)

        v_lat = -vx * sin_th + vy * cos_th
        y = -v_lat

        h = np.zeros((1, 5), dtype=np.float64)
        h[0, 2] = -sin_th
        h[0, 3] = cos_th
        h[0, 4] = -(vx * cos_th + vy * sin_th)

        s = float((h @ self.p @ h.T)[0, 0]) + r_nhc
        if abs(s) < 1e-10:
            return

        k = (self.p @ h.T)[:, 0] / s
        self.x = self.x + k * y
        i_kh = np.eye(5, dtype=np.float64) - np.outer(k, h[0])
        kk_t = np.outer(k, k)
        self.p = i_kh @ self.p @ i_kh.T + kk_t * r_nhc
        self._symmetrize()

    def speed_ms(self) -> float:
        return math.sqrt(self.x[2] * self.x[2] + self.x[3] * self.x[3])

    def heading(self) -> float:
        return float(self.x[4])

    def _symmetrize(self) -> None:
        self.p = 0.5 * (self.p + self.p.T)


class MadgwickAHRS:
    """Port of src/madgwick.h."""

    def __init__(self, sampleperiod: float = DT_DEFAULT, beta: float = VGPL_BETA) -> None:
        self.sampleperiod = sampleperiod
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def reset(self) -> None:
        self.q[:] = (1.0, 0.0, 0.0, 0.0)

    def update_imu(self, gx: float, gy: float, gz: float,
                   ax: float, ay: float, az: float) -> None:
        q0, q1, q2, q3 = self.q
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3

        norm_accel_sq = ax * ax + ay * ay + az * az
        if norm_accel_sq < 1e-8:
            return
        inv_norm_accel = fast_inv_sqrt(norm_accel_sq)
        raw_norm = norm_accel_sq * inv_norm_accel
        ax *= inv_norm_accel
        ay *= inv_norm_accel
        az *= inv_norm_accel

        s0 = -_2q2 * (2.0 * q1 * q3 - _2q0 * q2 - ax) + _2q1 * (
            2.0 * q0 * q1 + _2q2 * q3 - ay
        )
        s1 = _2q3 * (2.0 * q1 * q3 - _2q0 * q2 - ax) + _2q0 * (
            2.0 * q0 * q1 + _2q2 * q3 - ay
        ) - 4.0 * q1 * (1.0 - 2.0 * q1 * q1 - 2.0 * q2 * q2 - az)
        s2 = -_2q0 * (2.0 * q1 * q3 - _2q0 * q2 - ax) + _2q3 * (
            2.0 * q0 * q1 + _2q2 * q3 - ay
        ) - 4.0 * q2 * (1.0 - 2.0 * q1 * q1 - 2.0 * q2 * q2 - az)
        s3 = _2q1 * (2.0 * q1 * q3 - _2q0 * q2 - ax) + _2q2 * (
            2.0 * q0 * q1 + _2q2 * q3 - ay
        )

        norm_s_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3
        if norm_s_sq < 1e-8:
            return
        inv_norm_s = fast_inv_sqrt(norm_s_sq)
        s0 *= inv_norm_s
        s1 *= inv_norm_s
        s2 *= inv_norm_s
        s3 *= inv_norm_s

        dist_from_1g = abs(raw_norm - 1.0)
        k_norm = max(0.0, 1.0 - (dist_from_1g / VGPL_NORM_GATE))
        beta_eff = max(VGPL_BETA_FLOOR, self.beta * k_norm)

        q_dot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        q_dot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        q_dot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        q_dot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        q0 += (q_dot0 - beta_eff * s0) * self.sampleperiod
        q1 += (q_dot1 - beta_eff * s1) * self.sampleperiod
        q2 += (q_dot2 - beta_eff * s2) * self.sampleperiod
        q3 += (q_dot3 - beta_eff * s3) * self.sampleperiod

        norm_q_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3
        if norm_q_sq > 0.0:
            inv_norm_q = fast_inv_sqrt(norm_q_sq)
            self.q[:] = (
                q0 * inv_norm_q,
                q1 * inv_norm_q,
                q2 * inv_norm_q,
                q3 * inv_norm_q,
            )

    def gravity_vector(self) -> Tuple[float, float, float]:
        q0, q1, q2, q3 = self.q
        gx_out = 2.0 * (q1 * q3 - q0 * q2)
        gy_out = 2.0 * (q0 * q1 + q2 * q3)
        gz_out = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
        return gx_out, gy_out, gz_out


class SITLReplayer:
    def __init__(self, calibration: Optional[CalibrationState] = None) -> None:
        self.calibration = calibration
        self.eskf = ESKF2D()
        self.ahrs = MadgwickAHRS()
        self.reset_dynamic_state(seed_next_ema=False)
        self.total_rows = 0

    def reset_dynamic_state(self, seed_next_ema: bool) -> None:
        self.gz_var_buf = [0.0] * VAR_BUF_SIZE
        self.gx_var_buf = [0.0] * VAR_BUF_SIZE
        self.gy_var_buf = [0.0] * VAR_BUF_SIZE
        self.gz_var_idx = 0
        self.gz_var_count = 0

        self.thermal_bias_gz = 0.0
        self.thermal_bias_gx = 0.0
        self.thermal_bias_gy = 0.0

        self.last_gps = GpsState()
        self.last_gps_key: Optional[Tuple[float, float, float, float, int, float]] = None
        self.last_eskf_epoch = 0
        self.last_processed_fix_us = 0
        self.using_logged_gps_timing: Optional[bool] = None
        self.legacy_timing_warning_emitted = False

        self.prev_cog_rad = 0.0
        self.cog_ref_east = 0.0
        self.cog_ref_north = 0.0
        self.has_cog_ref = False
        self.cog_variation = 99.0

        self.prev_cent_x = 0.0
        self.prev_long_y = 0.0
        self.prev_v_eskf = 0.0
        self.last_timestamp_us = 0

        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_az = 0.0
        self.prev_gx = 0.0
        self.prev_gy = 0.0
        self.prev_gz = 0.0
        self.first_sample_after_recalib = seed_next_ema

        self.gps_origin_set = False
        self.gps_origin_lat = 0.0
        self.gps_origin_lon = 0.0

        self.eskf.reset()
        self.ahrs.reset()

    def apply_calibration(self, calibration: CalibrationState, is_update: bool) -> None:
        self.calibration = calibration
        if is_update:
            self.reset_dynamic_state(seed_next_ema=True)

    def process_row(self, row: Dict[str, float]) -> Dict[str, float]:
        if self.calibration is None:
            raise RuntimeError("Missing CALIB_BOOT/CALIB_UPDATE before first data row.")

        self.total_rows += 1
        recalib_marker = self.first_sample_after_recalib

        t_us = int(row["t_us"])
        dt_real_sec = DT_DEFAULT
        if self.last_timestamp_us > 0:
            dt_us = t_us - self.last_timestamp_us
            dt_real_sec = dt_us / 1_000_000.0
            if dt_real_sec <= 0.0 or dt_real_sec > 1.0:
                dt_real_sec = DT_DEFAULT
        self.last_timestamp_us = t_us

        sensor_acc = np.array(
            [row["sensor_ax"], row["sensor_ay"], row["sensor_az"]],
            dtype=np.float64,
        )
        sensor_gyr = np.array(
            [row["sensor_gx"], row["sensor_gy"], row["sensor_gz"]],
            dtype=np.float64,
        )

        acc_cal = CALIB_W @ (sensor_acc - CALIB_B)
        ax_r_raw, ay_r_raw, az_r_raw = rotate_3d(
            acc_cal[0], acc_cal[1], acc_cal[2], self.calibration
        )
        ax_r = ax_r_raw - self.calibration.bias_ax
        ay_r = ay_r_raw - self.calibration.bias_ay
        az_r = az_r_raw - self.calibration.bias_az

        gx_clean = sensor_gyr[0] - self.calibration.bias_gx
        gy_clean = sensor_gyr[1] - self.calibration.bias_gy
        gz_clean = sensor_gyr[2] - self.calibration.bias_gz
        gx_r, gy_r, gz_r = rotate_3d(gx_clean, gy_clean, gz_clean, self.calibration)

        gx_rad = (gx_r - self.thermal_bias_gx) * DEG2RAD
        gy_rad = (gy_r - self.thermal_bias_gy) * DEG2RAD
        gz_rad = (gz_r - self.thermal_bias_gz) * DEG2RAD

        v_eskf = self.eskf.speed_ms()
        dv_dt = (v_eskf - self.prev_v_eskf) / dt_real_sec if dt_real_sec > 0.0 else 0.0
        self.prev_v_eskf = v_eskf

        v_var = self.eskf.p[2, 2] + self.eskf.p[3, 3]
        v_confidence = min(1.0, 1.0 / (1.0 + v_var))

        cent_x_raw = -(v_eskf * gz_rad) / G_ACCEL
        cent_x = rate_limit(cent_x_raw * v_confidence, self.prev_cent_x, VGPL_RATE_LIMIT)
        self.prev_cent_x = cent_x

        long_y_raw = dv_dt / G_ACCEL
        long_y = rate_limit(long_y_raw * v_confidence, self.prev_long_y, VGPL_RATE_LIMIT)
        self.prev_long_y = long_y

        ax_madg = ax_r - cent_x
        ay_madg = ay_r - long_y
        az_madg = az_r

        self.ahrs.sampleperiod = dt_real_sec
        self.ahrs.update_imu(gx_rad, gy_rad, gz_rad, ax_madg, ay_madg, az_madg)
        grav_x, grav_y, grav_z = self.ahrs.gravity_vector()
        lin_ax = ax_r - grav_x
        lin_ay = ay_r - grav_y
        lin_az = az_r - grav_z

        self._ingest_gps_row(row, t_us)
        self._update_variance_buffers(gx_r, gy_r, gz_r)
        var_gx, var_gy, var_gz, mean_gx, mean_gy, mean_gz = self._variance_snapshot()

        is_stationary = False
        if var_gz >= 0.0:
            gps_slow = (not self.last_gps.valid) or (self.last_gps.speed_kmh < ZUPT_GPS_MAX_KMH)
            is_stationary = (
                var_gz < VAR_STILLNESS_GZ_THRESHOLD
                and var_gx < VAR_STILLNESS_GXY_THRESHOLD
                and var_gy < VAR_STILLNESS_GXY_THRESHOLD
                and gps_slow
                and abs(mean_gz) < 2.5
            )

        if is_stationary:
            self.thermal_bias_gz = mean_gz
            self.thermal_bias_gx = mean_gx
            self.thermal_bias_gy = mean_gy

        straight_zaru_active = False
        if (not is_stationary) and self.last_gps.valid and self.last_gps.speed_kmh > STRAIGHT_MIN_SPEED_KMH:
            lat_gate = abs(lin_ay) < STRAIGHT_MAX_LAT_G
            gz_gate = abs(gz_r - self.thermal_bias_gz) < 2.0
            cog_gate = abs(self.cog_variation) < STRAIGHT_COG_MAX_RAD
            if lat_gate and gz_gate and cog_gate:
                self.thermal_bias_gz = (
                    STRAIGHT_ALPHA * mean_gz + (1.0 - STRAIGHT_ALPHA) * self.thermal_bias_gz
                )
                self.thermal_bias_gx = (
                    STRAIGHT_ALPHA * mean_gx + (1.0 - STRAIGHT_ALPHA) * self.thermal_bias_gx
                )
                self.thermal_bias_gy = (
                    STRAIGHT_ALPHA * mean_gy + (1.0 - STRAIGHT_ALPHA) * self.thermal_bias_gy
                )
                straight_zaru_active = True

        self.eskf.predict(lin_ax, lin_ay, gz_rad, dt_real_sec, is_stationary)

        nhc_active = False
        if self.eskf.speed_ms() > NHC_MIN_SPEED_MS and abs(lin_ay) < NHC_MAX_LAT_G:
            self.eskf.correct_nhc(NHC_R)
            nhc_active = True

        gps_is_stale = self.last_gps.valid and (t_us - self.last_gps.fix_us > 5_000_000)
        if self.using_logged_gps_timing:
            has_new_fix = self.last_gps.valid and (self.last_gps.fix_us != self.last_processed_fix_us)
        else:
            has_new_fix = self.last_gps.valid and (self.last_gps.epoch != self.last_eskf_epoch)

        if has_new_fix and not gps_is_stale:
            if not self.gps_origin_set:
                self.gps_origin_lat = self.last_gps.lat
                self.gps_origin_lon = self.last_gps.lon
                self.gps_origin_set = True
                self.eskf.reset()

            east_m, north_m = wgs84_to_enu(
                self.last_gps.lat,
                self.last_gps.lon,
                self.gps_origin_lat,
                self.gps_origin_lon,
            )
            self.eskf.correct(east_m, north_m, self.last_gps.speed_kmh, self.last_gps.hdop)
            if self.using_logged_gps_timing:
                self.last_processed_fix_us = self.last_gps.fix_us
            else:
                self.last_eskf_epoch = self.last_gps.epoch

            if self.last_gps.speed_kmh > 20.0:
                if not self.has_cog_ref:
                    self.cog_ref_east = east_m
                    self.cog_ref_north = north_m
                    self.has_cog_ref = True
                else:
                    de = east_m - self.cog_ref_east
                    dn = north_m - self.cog_ref_north
                    dist = math.sqrt(de * de + dn * dn)
                    if dist > COG_MIN_BASELINE_M:
                        current_cog = math.atan2(de, dn)
                        self.cog_variation = wrap_angle(current_cog - self.prev_cog_rad)
                        self.prev_cog_rad = current_cog
                        self.cog_ref_east = east_m
                        self.cog_ref_north = north_m

        tb_gx = self.thermal_bias_gx
        tb_gy = self.thermal_bias_gy
        tb_gz = self.thermal_bias_gz

        ema_ax = ALPHA * lin_ax + (1.0 - ALPHA) * self.prev_ax
        ema_ay = ALPHA * lin_ay + (1.0 - ALPHA) * self.prev_ay
        ema_az = ALPHA * lin_az + (1.0 - ALPHA) * self.prev_az
        ema_gx = ALPHA * (gx_r - tb_gx) + (1.0 - ALPHA) * self.prev_gx
        ema_gy = ALPHA * (gy_r - tb_gy) + (1.0 - ALPHA) * self.prev_gy
        ema_gz = ALPHA * (gz_r - tb_gz) + (1.0 - ALPHA) * self.prev_gz

        if self.first_sample_after_recalib:
            ema_ax = lin_ax
            ema_ay = lin_ay
            ema_az = lin_az
            ema_gx = gx_r - tb_gx
            ema_gy = gy_r - tb_gy
            ema_gz = gz_r - tb_gz
            self.first_sample_after_recalib = False

        self.prev_ax = ema_ax
        self.prev_ay = ema_ay
        self.prev_az = ema_az
        self.prev_gx = ema_gx
        self.prev_gy = ema_gy
        self.prev_gz = ema_gz

        sitl_flags = 0
        if is_stationary:
            sitl_flags |= 0x01
        if straight_zaru_active:
            sitl_flags |= 0x02
        if nhc_active:
            sitl_flags |= 0x04
        if recalib_marker:
            sitl_flags |= 0x08

        return {
            "t_us": float(t_us),
            "sitl_body_ax": ax_r,
            "sitl_body_ay": ay_r,
            "sitl_body_az": az_r,
            "sitl_body_gx": gx_r,
            "sitl_body_gy": gy_r,
            "sitl_body_gz": gz_r,
            "sitl_raw_ax": lin_ax,
            "sitl_raw_ay": lin_ay,
            "sitl_raw_az": lin_az,
            "sitl_raw_gx": gx_r,
            "sitl_raw_gy": gy_r,
            "sitl_raw_gz": gz_r,
            "sitl_ax": ema_ax,
            "sitl_ay": ema_ay,
            "sitl_az": ema_az,
            "sitl_gx": ema_gx,
            "sitl_gy": ema_gy,
            "sitl_gz": ema_gz,
            "sitl_kf_x": self.eskf.x[0],
            "sitl_kf_y": self.eskf.x[1],
            "sitl_kf_vel": self.eskf.speed_ms(),
            "sitl_kf_heading": self.eskf.heading(),
            "sitl_tbias_gx": tb_gx,
            "sitl_tbias_gy": tb_gy,
            "sitl_tbias_gz": tb_gz,
            "sitl_var_gx": var_gx,
            "sitl_var_gy": var_gy,
            "sitl_var_gz": var_gz,
            "sitl_mean_gx": mean_gx,
            "sitl_mean_gy": mean_gy,
            "sitl_mean_gz": mean_gz,
            "sitl_stationary": float(is_stationary),
            "sitl_straight_zaru": float(straight_zaru_active),
            "sitl_nhc": float(nhc_active),
            "sitl_zaru_flags": float(sitl_flags),
            "sitl_cent_x": cent_x,
            "sitl_long_y": long_y,
        }

    def _ingest_gps_row(self, row: Dict[str, float], t_us: int) -> None:
        lat = row["gps_lat"]
        lon = row["gps_lon"]
        speed_kmh = row["gps_speed_kmh"]
        alt_m = row.get("gps_alt_m", 0.0)
        sats = int(row["gps_sats"])
        hdop = row["gps_hdop"]
        has_logged_timing = "gps_fix_us" in row and "gps_valid" in row

        if has_logged_timing:
            if self.using_logged_gps_timing is None:
                self.using_logged_gps_timing = True
            valid = bool(int(row["gps_valid"]))
            fix_us = int(row["gps_fix_us"])
            epoch = self.last_gps.epoch
            if valid and fix_us != self.last_gps.fix_us:
                epoch += 1
            self.last_gps_key = None
        else:
            if self.using_logged_gps_timing is None:
                self.using_logged_gps_timing = False
            if not self.legacy_timing_warning_emitted:
                print(
                    "[SITL] Warning: pre-164 log detected; gps_fix_us/gps_valid missing, "
                    "using approximate GPS timing heuristics.",
                    file=sys.stderr,
                )
                self.legacy_timing_warning_emitted = True

            valid = sats > 0 and not (abs(lat) < 1e-12 and abs(lon) < 1e-12)
            epoch = self.last_gps.epoch
            fix_us = self.last_gps.fix_us
            key = (lat, lon, speed_kmh, alt_m, sats, hdop)

            if valid:
                if (
                    (not self.last_gps.valid)
                    or (self.last_gps_key != key)
                    or (
                        self.last_gps.fix_us > 0
                        and (t_us - self.last_gps.fix_us) >= LEGACY_GPS_FIX_PERIOD_US
                    )
                ):
                    epoch += 1
                    fix_us = t_us
                self.last_gps_key = key

        self.last_gps = GpsState(
            lat=lat,
            lon=lon,
            speed_kmh=speed_kmh,
            alt_m=alt_m,
            sats=sats,
            hdop=hdop,
            valid=valid,
            epoch=epoch,
            fix_us=fix_us,
        )

    def _update_variance_buffers(self, gx_r: float, gy_r: float, gz_r: float) -> None:
        idx = self.gz_var_idx
        self.gz_var_buf[idx] = gz_r
        self.gx_var_buf[idx] = gx_r
        self.gy_var_buf[idx] = gy_r
        self.gz_var_idx = (idx + 1) % VAR_BUF_SIZE
        if self.gz_var_count < VAR_BUF_SIZE:
            self.gz_var_count += 1

    def _variance_snapshot(self) -> Tuple[float, float, float, float, float, float]:
        if self.gz_var_count < VAR_BUF_SIZE:
            return -1.0, -1.0, -1.0, 0.0, 0.0, 0.0

        gz = np.array(self.gz_var_buf, dtype=np.float64)
        gx = np.array(self.gx_var_buf, dtype=np.float64)
        gy = np.array(self.gy_var_buf, dtype=np.float64)

        mean_gz = float(gz.mean())
        mean_gx = float(gx.mean())
        mean_gy = float(gy.mean())

        var_gz = max(0.0, float((gz * gz).mean() - mean_gz * mean_gz))
        var_gx = max(0.0, float((gx * gx).mean() - mean_gx * mean_gx))
        var_gy = max(0.0, float((gy * gy).mean() - mean_gy * mean_gy))
        return var_gx, var_gy, var_gz, mean_gx, mean_gy, mean_gz


def normalize_column(name: str) -> str:
    return HEADER_UNIT_RE.sub("", name).strip().lower().replace(" ", "_")


def parse_calibration_comment(line: str) -> Optional[Tuple[str, CalibrationState]]:
    match = CALIB_RE.match(line.strip())
    if not match:
        return None
    kind = match.group(1)
    values = {key: float(val) for key, val in CALIB_KV_RE.findall(match.group(2))}
    required = {
        "sin_phi",
        "cos_phi",
        "sin_theta",
        "cos_theta",
        "bias_ax",
        "bias_ay",
        "bias_az",
        "bias_gx",
        "bias_gy",
        "bias_gz",
    }
    missing = sorted(required - set(values))
    if missing:
        raise ValueError(f"Calibration comment missing fields: {', '.join(missing)}")
    ordered = {
        "sin_phi": values["sin_phi"],
        "cos_phi": values["cos_phi"],
        "sin_theta": values["sin_theta"],
        "cos_theta": values["cos_theta"],
        "bias_ax": values["bias_ax"],
        "bias_ay": values["bias_ay"],
        "bias_az": values["bias_az"],
        "bias_gx": values["bias_gx"],
        "bias_gy": values["bias_gy"],
        "bias_gz": values["bias_gz"],
    }
    return kind, CalibrationState(**ordered)


def parse_row(header: Sequence[str], raw_values: Sequence[str]) -> Dict[str, float]:
    if len(raw_values) != len(header):
        raise ValueError(f"CSV row has {len(raw_values)} columns, expected {len(header)}.")
    row: Dict[str, float] = {}
    for key, raw in zip(header, raw_values):
        if key in INT_COLUMNS:
            row[key] = int(raw)
        else:
            row[key] = float(raw)
    return row


def iter_csv_payload(header_line: str, lines: Iterable[str]) -> Iterator[Tuple[str, object]]:
    header = next(csv.reader([header_line]))
    normalized_header = [normalize_column(col) for col in header]
    missing = sorted(REQUIRED_COLUMNS - set(normalized_header))
    if missing:
        raise ValueError(
            "Input does not contain the v1.4.x SITL columns required by this script: "
            + ", ".join(missing)
        )

    for raw_line in lines:
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            parsed = parse_calibration_comment(line)
            if parsed is not None:
                yield ("calibration", parsed)
            continue
        row_values = next(csv.reader([line]))
        yield ("row", parse_row(normalized_header, row_values))


def open_csv_with_fallback(path: Path) -> TextIO:
    for encoding in ("utf-8-sig", "cp1252", "latin-1"):
        try:
            handle = path.open("r", encoding=encoding, newline="")
            handle.readline()
            handle.seek(0)
            return handle
        except UnicodeDecodeError:
            handle.close()
    raise UnicodeDecodeError("csv", b"", 0, 1, f"Unable to decode {path}")


def iter_input_events(path: Path) -> Iterator[Tuple[str, object]]:
    suffix = path.suffix.lower()
    if suffix == ".csv":
        with open_csv_with_fallback(path) as handle:
            header_line = handle.readline().strip()
            if not header_line:
                raise ValueError(f"Empty CSV: {path}")
            yield from iter_csv_payload(header_line, handle)
        return

    if suffix == ".bin":
        hdr, _ = bin_to_csv.read_file_header(str(path))
        record_size = hdr["record_size"] if hdr else bin_to_csv.CHUNK_SIZE_DEFAULT
        _, header_cols, _ = bin_to_csv.get_format(record_size)
        header_line = ",".join(header_cols)
        yield from iter_csv_payload(header_line, bin_to_csv.bin_to_csv_lines(str(path)))
        return

    raise ValueError(f"Unsupported input format: {path.suffix}")


def make_output_writer(path: Path) -> Tuple[TextIO, csv.writer]:
    handle = path.open("w", encoding="utf-8", newline="")
    writer = csv.writer(handle)
    writer.writerow(
        [
            "t_us",
            "sitl_body_ax",
            "sitl_body_ay",
            "sitl_body_az",
            "sitl_body_gx",
            "sitl_body_gy",
            "sitl_body_gz",
            "sitl_raw_ax",
            "sitl_raw_ay",
            "sitl_raw_az",
            "sitl_raw_gx",
            "sitl_raw_gy",
            "sitl_raw_gz",
            "sitl_ax",
            "sitl_ay",
            "sitl_az",
            "sitl_gx",
            "sitl_gy",
            "sitl_gz",
            "sitl_kf_x",
            "sitl_kf_y",
            "sitl_kf_vel",
            "sitl_kf_heading",
            "sitl_tbias_gx",
            "sitl_tbias_gy",
            "sitl_tbias_gz",
            "sitl_var_gx",
            "sitl_var_gy",
            "sitl_var_gz",
            "sitl_stationary",
            "sitl_straight_zaru",
            "sitl_nhc",
            "sitl_zaru_flags",
            "err_raw_ax",
            "err_raw_ay",
            "err_raw_az",
            "err_raw_gx",
            "err_raw_gy",
            "err_raw_gz",
            "err_ax",
            "err_ay",
            "err_az",
            "err_gx",
            "err_gy",
            "err_gz",
            "err_kf_x",
            "err_kf_y",
            "err_kf_vel",
            "err_kf_heading",
            "err_tbias_gz",
        ]
    )
    return handle, writer


def update_error_stats(stats: Dict[str, RunningErrorStats],
                       row: Dict[str, float],
                       replay: Dict[str, float]) -> None:
    comparisons = [
        ("raw_ax", "sitl_raw_ax", False),
        ("raw_ay", "sitl_raw_ay", False),
        ("raw_az", "sitl_raw_az", False),
        ("raw_gx", "sitl_raw_gx", False),
        ("raw_gy", "sitl_raw_gy", False),
        ("raw_gz", "sitl_raw_gz", False),
        ("ax", "sitl_ax", False),
        ("ay", "sitl_ay", False),
        ("az", "sitl_az", False),
        ("gx", "sitl_gx", False),
        ("gy", "sitl_gy", False),
        ("gz", "sitl_gz", False),
        ("kf_x", "sitl_kf_x", False),
        ("kf_y", "sitl_kf_y", False),
        ("kf_vel", "sitl_kf_vel", False),
        ("kf_heading", "sitl_kf_heading", True),
        ("tbias_gz", "sitl_tbias_gz", False),
    ]
    for log_key, replay_key, wrap_pi in comparisons:
        if log_key in row:
            stats[log_key].add(row[log_key], replay[replay_key], wrap_pi=wrap_pi)


def write_output_row(writer: csv.writer,
                     row: Dict[str, float],
                     replay: Dict[str, float]) -> None:
    err_heading = wrap_angle(replay["sitl_kf_heading"] - row["kf_heading"])
    writer.writerow(
        [
            int(replay["t_us"]),
            replay["sitl_body_ax"],
            replay["sitl_body_ay"],
            replay["sitl_body_az"],
            replay["sitl_body_gx"],
            replay["sitl_body_gy"],
            replay["sitl_body_gz"],
            replay["sitl_raw_ax"],
            replay["sitl_raw_ay"],
            replay["sitl_raw_az"],
            replay["sitl_raw_gx"],
            replay["sitl_raw_gy"],
            replay["sitl_raw_gz"],
            replay["sitl_ax"],
            replay["sitl_ay"],
            replay["sitl_az"],
            replay["sitl_gx"],
            replay["sitl_gy"],
            replay["sitl_gz"],
            replay["sitl_kf_x"],
            replay["sitl_kf_y"],
            replay["sitl_kf_vel"],
            replay["sitl_kf_heading"],
            replay["sitl_tbias_gx"],
            replay["sitl_tbias_gy"],
            replay["sitl_tbias_gz"],
            replay["sitl_var_gx"],
            replay["sitl_var_gy"],
            replay["sitl_var_gz"],
            int(replay["sitl_stationary"]),
            int(replay["sitl_straight_zaru"]),
            int(replay["sitl_nhc"]),
            int(replay["sitl_zaru_flags"]),
            replay["sitl_raw_ax"] - row["raw_ax"],
            replay["sitl_raw_ay"] - row["raw_ay"],
            replay["sitl_raw_az"] - row["raw_az"],
            replay["sitl_raw_gx"] - row["raw_gx"],
            replay["sitl_raw_gy"] - row["raw_gy"],
            replay["sitl_raw_gz"] - row["raw_gz"],
            replay["sitl_ax"] - row["ax"],
            replay["sitl_ay"] - row["ay"],
            replay["sitl_az"] - row["az"],
            replay["sitl_gx"] - row["gx"],
            replay["sitl_gy"] - row["gy"],
            replay["sitl_gz"] - row["gz"],
            replay["sitl_kf_x"] - row["kf_x"],
            replay["sitl_kf_y"] - row["kf_y"],
            replay["sitl_kf_vel"] - row["kf_vel"],
            err_heading,
            replay["sitl_tbias_gz"] - row.get("tbias_gz", 0.0),
        ]
    )


def print_summary(path: Path,
                  calibration: CalibrationState,
                  rows_processed: int,
                  stats: Dict[str, RunningErrorStats],
                  exact_flag_mismatches: int,
                  bit_mismatches: List[int],
                  logged_gps_timing: Optional[bool]) -> None:
    phi_deg = calibration.phi_rad * RAD2DEG
    theta_deg = calibration.theta_rad * RAD2DEG

    print(f"[SITL] Input: {path}")
    print(
        "[SITL] CALIB_BOOT:"
        f" phi={phi_deg:+.3f} deg"
        f" theta={theta_deg:+.3f} deg"
        f" bias_a=({calibration.bias_ax:+.5f}, {calibration.bias_ay:+.5f}, {calibration.bias_az:+.5f})"
        f" bias_g=({calibration.bias_gx:+.5f}, {calibration.bias_gy:+.5f}, {calibration.bias_gz:+.5f})"
    )
    print(f"[SITL] Rows processed: {rows_processed}")
    if logged_gps_timing is True:
        print("[SITL] GPS timing: exact replay from logged gps_fix_us/gps_valid")
    elif logged_gps_timing is False:
        print("[SITL] GPS timing: approximate heuristic fallback (pre-164 log)")
    else:
        print("[SITL] GPS timing: unavailable")
    print(
        f"[SITL] zaru_flags exact mismatch: {exact_flag_mismatches}/{rows_processed}"
        f" ({(100.0 * exact_flag_mismatches / rows_processed) if rows_processed else 0.0:.2f}%)"
    )
    print(
        "[SITL] zaru_flags bit mismatches:"
        f" static={bit_mismatches[0]}"
        f" straight={bit_mismatches[1]}"
        f" nhc={bit_mismatches[2]}"
        f" recalib={bit_mismatches[3]}"
    )
    print()
    print(f"{'channel':<12} {'mae':>12} {'rmse':>12} {'max_abs':>12}")
    print("-" * 52)
    ordered = [
        "raw_ax", "raw_ay", "raw_az",
        "raw_gx", "raw_gy", "raw_gz",
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "kf_x", "kf_y", "kf_vel", "kf_heading",
        "tbias_gz",
    ]
    for key in ordered:
        metric = stats[key]
        print(f"{key:<12} {metric.mae:12.6f} {metric.rmse:12.6f} {metric.max_abs:12.6f}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Replay the Telemetria SITL pipeline from a v1.4.x CSV or BIN log."
    )
    parser.add_argument("input", help="Path to a v1.4.x .csv or .bin file.")
    parser.add_argument(
        "--output",
        help="Optional CSV path for replayed SITL channels and per-sample errors.",
    )
    parser.add_argument(
        "--max-rows",
        type=int,
        default=0,
        help="Optional limit for quick debugging (0 = full file).",
    )
    args = parser.parse_args()

    input_path = Path(args.input).resolve()
    if not input_path.exists():
        raise SystemExit(f"Input file not found: {input_path}")

    replay = SITLReplayer()
    stats = {key: RunningErrorStats() for key in [
        "raw_ax", "raw_ay", "raw_az",
        "raw_gx", "raw_gy", "raw_gz",
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "kf_x", "kf_y", "kf_vel", "kf_heading",
        "tbias_gz",
    ]}

    rows_processed = 0
    exact_flag_mismatches = 0
    bit_mismatches = [0, 0, 0, 0]

    out_handle: Optional[TextIO] = None
    out_writer: Optional[csv.writer] = None
    if args.output:
        out_path = Path(args.output).resolve()
        out_handle, out_writer = make_output_writer(out_path)

    try:
        for event_type, payload in iter_input_events(input_path):
            if event_type == "calibration":
                kind, calibration = payload  # type: ignore[misc]
                replay.apply_calibration(calibration, is_update=(kind == "CALIB_UPDATE"))
                continue

            row = payload  # type: ignore[assignment]
            replay_row = replay.process_row(row)
            rows_processed += 1

            update_error_stats(stats, row, replay_row)

            if "zaru_flags" in row:
                logged_flags = int(row["zaru_flags"])
                sitl_flags = int(replay_row["sitl_zaru_flags"])
                if logged_flags != sitl_flags:
                    exact_flag_mismatches += 1
                for bit in range(4):
                    mask = 1 << bit
                    if bool(logged_flags & mask) != bool(sitl_flags & mask):
                        bit_mismatches[bit] += 1

            if out_writer is not None:
                write_output_row(out_writer, row, replay_row)

            if args.max_rows and rows_processed >= args.max_rows:
                break
    finally:
        if out_handle is not None:
            out_handle.close()

    if replay.calibration is None:
        raise SystemExit("No CALIB_BOOT/CALIB_UPDATE found in the input file.")

    print_summary(
        input_path,
        replay.calibration,
        rows_processed,
        stats,
        exact_flag_mismatches,
        bit_mismatches,
        replay.using_logged_gps_timing,
    )


if __name__ == "__main__":
    main()
