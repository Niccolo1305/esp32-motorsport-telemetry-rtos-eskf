#!/usr/bin/env python3
"""Fail-fast guard for legacy one-off analysis scripts."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable


V5_COLUMNS = {
    "pipe_lin_ax",
    "pipe_lin_ay",
    "pipe_lin_az",
    "pipe_body_gx",
    "pipe_body_gy",
    "pipe_body_gz",
    "bmi_post_lpf20_prepipe_ax",
    "bmi_post_lpf20_prepipe_ay",
    "bmi_post_lpf20_prepipe_az",
    "bmi_gyr_x_dps",
    "bmm_ut_x",
    "fifo_frames_drained",
}
BUTTER_COLUMNS = {
    "butter_ax",
    "butter_ay",
    "butter_az",
    "butter_gx",
    "butter_gy",
    "butter_gz",
}

HEADER_UNIT_RE = re.compile(r"\s*\([^)]*\)\s*$")


def _normalize(name: object) -> str:
    return HEADER_UNIT_RE.sub("", str(name)).strip().lower().replace(" ", "_")


def reject_v5_columns(columns: Iterable[str], path: str | Path) -> None:
    normalized = {_normalize(col) for col in columns}
    if any(col in normalized for col in V5_COLUMNS):
        raise SystemExit(
            f"{path}: Telemetria v5 CSV detected. This legacy tools/script analysis is intentionally limited to <= v4 logs."
        )
    if any(col in normalized for col in BUTTER_COLUMNS):
        raise SystemExit(
            f"{path}: Butterworth CSV detected. This legacy tools/script analysis expects dated EMA-era schemas; use tools/sitl_hal or dashboard tooling instead."
        )
