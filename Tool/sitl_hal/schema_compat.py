#!/usr/bin/env python3
"""
Schema compatibility helpers for Telemetria CSV/BIN tooling.

v5 AtomS3R logs renamed the old mixed-semantics columns into explicit groups:
  - pipe_lin_* / pipe_body_*      : zero-latency pipeline diagnostics
  - bmi_acc_* / bmi_gyr_*         : Bosch-direct physical channels in chip axes
  - bmi_raw_* / bmm_raw_* / ...   : acquisition truth

Older SITL/static tools still use legacy names:
  - raw_*     : zero-latency post-pipeline/pre-EMA outputs (pipe_*)
  - sensor_*  : pre-pipeline physical IMU channels remapped to pipeline axes

This module keeps the tooling honest by generating those aliases only inside
offline analysis, never in the logged schema itself.
"""

from __future__ import annotations

from typing import Iterable, MutableMapping, Sequence, Set, Tuple


LEGACY_RAW_AXES = {
    "raw_ax",
    "raw_ay",
    "raw_az",
    "raw_gx",
    "raw_gy",
    "raw_gz",
}

LEGACY_SENSOR_AXES = {
    "sensor_ax",
    "sensor_ay",
    "sensor_az",
    "sensor_gx",
    "sensor_gy",
    "sensor_gz",
}

V5_PIPE_AXES = {
    "pipe_lin_ax",
    "pipe_lin_ay",
    "pipe_lin_az",
    "pipe_body_gx",
    "pipe_body_gy",
    "pipe_body_gz",
}

V5_SENSOR_AXES = {
    "bmi_acc_x_g",
    "bmi_acc_y_g",
    "bmi_acc_z_g",
    "bmi_gyr_x_dps",
    "bmi_gyr_y_dps",
    "bmi_gyr_z_dps",
}

LEGACY_CALIB_B = (-0.00125, +0.00429, -0.06491)
LEGACY_CALIB_W = (
    (+1.000824, -0.000511, -0.001575),
    (-0.000511, +1.000989, -0.000132),
    (-0.001575, -0.000132, +1.003466),
)
BMI270_CALIB_B = (0.0, 0.0, 0.0)
BMI270_CALIB_W = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def remap_chip_axes_to_pipeline(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Mirror src/imu_axis_remap.h for AtomS3R Bosch-direct chip axes."""
    return y, -x, z


def expand_available_columns(columns: Iterable[str]) -> Set[str]:
    available = set(columns)
    if V5_PIPE_AXES.issubset(available):
        available.update(LEGACY_RAW_AXES)
    if V5_SENSOR_AXES.issubset(available):
        available.update(LEGACY_SENSOR_AXES)
    return available


def uses_bmi270_calibration(columns: Iterable[str]) -> bool:
    return V5_SENSOR_AXES.issubset(set(columns))


def ellipsoid_calibration_for_columns(
    columns: Iterable[str],
) -> Tuple[Sequence[float], Sequence[Sequence[float]]]:
    if uses_bmi270_calibration(columns):
        return BMI270_CALIB_B, BMI270_CALIB_W
    return LEGACY_CALIB_B, LEGACY_CALIB_W


def apply_runtime_aliases(row: MutableMapping[str, float]) -> MutableMapping[str, float]:
    """Add analysis-only compatibility aliases for v5 rows."""
    if "raw_ax" not in row and "pipe_lin_ax" in row:
        row["raw_ax"] = row["pipe_lin_ax"]
        row["raw_ay"] = row["pipe_lin_ay"]
        row["raw_az"] = row["pipe_lin_az"]
    if "raw_gx" not in row and "pipe_body_gx" in row:
        row["raw_gx"] = row["pipe_body_gx"]
        row["raw_gy"] = row["pipe_body_gy"]
        row["raw_gz"] = row["pipe_body_gz"]

    if "sensor_ax" not in row and "bmi_acc_x_g" in row:
        ax, ay, az = remap_chip_axes_to_pipeline(
            float(row["bmi_acc_x_g"]),
            float(row["bmi_acc_y_g"]),
            float(row["bmi_acc_z_g"]),
        )
        row["sensor_ax"] = ax
        row["sensor_ay"] = ay
        row["sensor_az"] = az
    if "sensor_gx" not in row and "bmi_gyr_x_dps" in row:
        gx, gy, gz = remap_chip_axes_to_pipeline(
            float(row["bmi_gyr_x_dps"]),
            float(row["bmi_gyr_y_dps"]),
            float(row["bmi_gyr_z_dps"]),
        )
        row["sensor_gx"] = gx
        row["sensor_gy"] = gy
        row["sensor_gz"] = gz
    return row
