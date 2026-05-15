"""
bin_to_csv.py — Binary telemetry → CSV converter with interactive split

Usage:
  python bin_to_csv.py                       # interactive menu
  python bin_to_csv.py tel_23.bin           # convert → split menu
  python bin_to_csv.py tel_23.csv           # split existing CSV
  python bin_to_csv.py tel_23.bin out.csv   # convert without split
  python bin_to_csv.py tel_23.bin out.csv --report
  python bin_to_csv.py --report-session data/session_dir
"""

import struct
import sys
import os
import time
import math
import json
import csv
import re
from datetime import datetime, timezone
from collections import Counter

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except ValueError:
        pass

# ── Configuration ─────────────────────────────────────────────────────────────
CHUNK_SIZE_DEFAULT = 122          # record size v0.9.8 (no header, 122 bytes)
HEADER_MAGIC = b'TEL'             # FileHeader magic bytes (v0.9.7+)
HEADER_V6_SIZE = 256              # v6: sector-aligned startup diagnostics header
RECORD_MAGIC_256_V6 = 0x364C4554  # "TEL6" little-endian
ENDIAN_MARKER = 0x1234
HEADER_V1_SIZE = 25               # v1: record_size is uint8_t  (25 bytes)
HEADER_V2_SIZE = 26               # v2: record_size is uint16_t (26 bytes)
HEADER_V3_SIZE = 66               # v3/v4: adds 40 bytes of calibration params
HEADER_V5_SIZE = 80               # v5: adds header_flags + mag_ref_ut_xyz
SENTINEL_CALIB = 0xFFFFFFFFFFFFFFFF  # uint64 max — CalibrationRecord marker
SENTINEL_MOUNT_YAW_OBS = 0xFFFFFFFFFFFFFFFE  # uint64 max-1 — Mount-yaw observation marker
SENTINEL_MOUNT_YAW_BOOT = 0xFFFFFFFFFFFFFFFD  # uint64 max-2 — Applied mount-yaw boot marker
PROGRESS_EVERY = 5000
TEXT_ENCODING_WRITE = 'utf-8-sig'
TEXT_ENCODINGS_READ = ('utf-8-sig', 'utf-8', 'latin1')
RESYNC_SCAN_BYTES = 4096
RESYNC_CONFIRM_RECORDS = 8
PREALLOC_TAIL_RESYNC_SCAN_BYTES = 1024 * 1024
MAX_TIMESTAMP_US = 7 * 24 * 60 * 60 * 1_000_000
MAX_TIMESTAMP_GAP_US = 10_000_000
FILE_HEADER_V6_FLAG_LOG_PREALLOCATED = 0x00000010
GPS_FRESH_MAX_US_REPORT = 1_500_000
REPORT_SCHEMA_VERSION = 1
REPORT_DEFAULT_KF_STEP_THRESHOLD_M = 100.0

GPS_SUPERVISOR_STATE_NAMES = {
    0: 'OK',
    1: 'SUSPECT',
    2: 'QUARANTINE',
    3: 'RECOVERING',
}

GPS_SUPERVISOR_REASON_NAMES = {
    0x0001: 'NMEA_NAV2_POS_MISMATCH',
    0x0002: 'IMPLIED_SPEED_BAD',
    0x0004: 'LOW_SPEED_POSITION_JUMP',
    0x0008: 'BAD_FIXFLAGS',
    0x0010: 'BAD_HACC_HDOP_SATS',
    0x0020: 'ESKF_INNOVATION_BAD',
    0x0040: 'POSITION_FROZEN_WHILE_MOVING',
    0x0080: 'NAV2_STALE',
}

# ── Format Registry ──────────────────────────────────────────────────────────
# 122-byte record (v0.9.8 — v1.3.0)
FMT_122 = '<I7fBddffBf4f6f5f'
HEADER_122 = [
    't_ms (ms)',
    'ax (G)', 'ay (G)', 'az (G)', 'gx (°/s)', 'gy (°/s)', 'gz (°/s)', 'temp_c (°C)',
    'lap',
    'gps_lat (°)', 'gps_lon (°)',
    'gps_sog_kmh (km/h)', 'gps_alt_m (m)',
    'gps_sats',
    'gps_hdop',
    'kf_x (m)', 'kf_y (m)', 'kf_vel (m/s)', 'kf_heading (rad)',
    'raw_ax (G)', 'raw_ay (G)', 'raw_az (G)', 'raw_gx (°/s)', 'raw_gy (°/s)', 'raw_gz (°/s)',
    'kf6_x (m)', 'kf6_y (m)', 'kf6_vel (m/s)', 'kf6_heading (rad)', 'kf6_bgz (rad/s)'
]
COL_FMT_122 = [
    '{:d}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.1f}',
    '{:d}',
    '{:.7f}', '{:.7f}',
    '{:.2f}', '{:.2f}',
    '{:d}',
    '{:.2f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}'
]

# 127-byte record (v1.3.1+): adds zaru_flags (uint8) + tbias_gz (float)
FMT_127 = '<I7fBddffBf4f6f5fBf'
HEADER_127 = HEADER_122 + ['zaru_flags', 'tbias_gz (°/s)']
COL_FMT_127 = COL_FMT_122 + ['{:d}', '{:.5f}']

# 155-byte record (v1.4.0): timestamp upgraded to uint64 µs + sensor-frame raw IMU
FMT_155 = '<Q7fBddffBf4f6f5fBf6f'  # Q = uint64 (was I = uint32)
HEADER_155 = [
    't_us (µs)',  # was t_ms (ms) — microsecond precision for SITL
    'ax (G)', 'ay (G)', 'az (G)', 'gx (°/s)', 'gy (°/s)', 'gz (°/s)', 'temp_c (°C)',
    'lap',
    'gps_lat (°)', 'gps_lon (°)',
    'gps_sog_kmh (km/h)', 'gps_alt_m (m)',
    'gps_sats',
    'gps_hdop',
    'kf_x (m)', 'kf_y (m)', 'kf_vel (m/s)', 'kf_heading (rad)',
    'raw_ax (G)', 'raw_ay (G)', 'raw_az (G)', 'raw_gx (°/s)', 'raw_gy (°/s)', 'raw_gz (°/s)',
    'kf6_x (m)', 'kf6_y (m)', 'kf6_vel (m/s)', 'kf6_heading (rad)', 'kf6_bgz (rad/s)',
    'zaru_flags', 'tbias_gz (°/s)',
    'sensor_ax (G)', 'sensor_ay (G)', 'sensor_az (G)',
    'sensor_gx (°/s)', 'sensor_gy (°/s)', 'sensor_gz (°/s)'
]
COL_FMT_155 = [
    '{:d}',  # t_us (uint64 → int)
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.1f}',
    '{:d}',
    '{:.7f}', '{:.7f}',
    '{:.2f}', '{:.2f}',
    '{:d}',
    '{:.2f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:d}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}'
]
assert struct.calcsize(FMT_155) == 155, f"FMT_155 mismatch: {struct.calcsize(FMT_155)}"

# 164-byte record (v1.4.2+): adds GPS timing metadata for deterministic SITL replay
# gps_fix_us: esp_timer timestamp of last GPS fix — same timebase as t_us.
# gps_valid:  mirrors GpsData.valid — needed for is_stationary gps_slow gate.
FMT_164 = '<Q7fBddffBf4f6f5fBf6fQB'  # appends Q (uint64) + B (uint8)
HEADER_164 = HEADER_155 + ['gps_fix_us (µs)', 'gps_valid']
COL_FMT_164 = COL_FMT_155 + ['{:d}', '{:d}']
assert struct.calcsize(FMT_164) == 164, f"FMT_164 mismatch: {struct.calcsize(FMT_164)}"

# 190-byte record (v1.5.0+): adds CASIC NAV/NAV2 binary velocity fields
# nav_speed2d: CASIC NAV/NAV2 ground speed [m/s]
# nav_s_acc:   speed accuracy estimate [m/s]
# nav_vel_n/e: ENU velocity components [m/s]
# nav_vel_valid: raw CASIC NAV/NAV2 velocity validity scale; firmware treats >=6 as velocity-valid
# gps_speed_source: 0=NMEA_SOG, 1=NAV/NAV2 binary (source actually used by firmware)
# nav_fix_us:  esp_timer when NAV/NAV2 was parsed [µs] (freshness/SITL)
FMT_190 = '<Q7fBddffBf4f6f5fBf6fQB4fBBQ'
HEADER_190 = HEADER_164 + [
    'nav_speed2d (m/s)', 'nav_s_acc (m/s)',
    'nav_vel_n (m/s)', 'nav_vel_e (m/s)',
    'nav_vel_valid', 'gps_speed_source',
    'nav_fix_us (µs)',
]
COL_FMT_190 = COL_FMT_164 + [
    '{:.5f}', '{:.6f}',
    '{:.5f}', '{:.5f}',
    '{:d}', '{:d}',
    '{:d}',
]
assert struct.calcsize(FMT_190) == 190, f"FMT_190 mismatch: {struct.calcsize(FMT_190)}"

# 202-byte record (v1.5.1+): adds NMEA DHV ground speed and timestamp
# dhv_gdspd: NMEA DHV horizontal ground speed [m/s]
# dhv_fix_us: esp_timer when DHV was parsed [µs]
FMT_202 = '<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ'
HEADER_202 = HEADER_190 + [
    'dhv_gdspd (m/s)',
    'dhv_fix_us (µs)',
]
COL_FMT_202 = COL_FMT_190 + [
    '{:.5f}',
    '{:d}',
]
assert struct.calcsize(FMT_202) == 202, f"FMT_202 mismatch: {struct.calcsize(FMT_202)}"

# 215-byte record (v1.6.0 legacy AtomS3R): M5Unified-scaled magnetometer output.
FMT_215 = '<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ3fB'
HEADER_215 = HEADER_202 + [
    'mag_mx_legacy (arb)',
    'mag_my_legacy (arb)',
    'mag_mz_legacy (arb)',
    'mag_valid_legacy',
]
COL_FMT_215 = COL_FMT_202 + ['{:.3f}', '{:.3f}', '{:.3f}', '{:d}']
assert struct.calcsize(FMT_215) == 215, f"FMT_215 mismatch: {struct.calcsize(FMT_215)}"

# 224-byte record (v1.6.1+): Bosch-direct AtomS3R magnetometer fields.
FMT_224 = '<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ3hH3fBB'
HEADER_224 = HEADER_202 + [
    'mag_raw_x (LSB)',
    'mag_raw_y (LSB)',
    'mag_raw_z (LSB)',
    'mag_rhall (LSB)',
    'mag_ut_x (uT)',
    'mag_ut_y (uT)',
    'mag_ut_z (uT)',
    'mag_valid',
    'mag_fresh',
]
COL_FMT_224 = COL_FMT_202 + ['{:d}', '{:d}', '{:d}', '{:d}', '{:.6f}', '{:.6f}', '{:.6f}', '{:d}', '{:d}']
assert struct.calcsize(FMT_224) == 224, f"FMT_224 mismatch: {struct.calcsize(FMT_224)}"

# 242-byte record (v1.7.0+): AtomS3R v5/v6 FIFO acquisition layout.
FMT_242 = '<Q7fBddffBf4f6f5fBf6h6f3hH3f8BQB4fBBQfQ'
HEADER_242 = [
    't_us (µs)',
    'ax (G)', 'ay (G)', 'az (G)', 'gx (°/s)', 'gy (°/s)', 'gz (°/s)', 'temp_c (°C)',
    'lap',
    'gps_lat (°)', 'gps_lon (°)',
    'gps_sog_kmh (km/h)', 'gps_alt_m (m)',
    'gps_sats',
    'gps_hdop',
    'kf_x (m)', 'kf_y (m)', 'kf_vel (m/s)', 'kf_heading (rad)',
    'pipe_lin_ax (G)', 'pipe_lin_ay (G)', 'pipe_lin_az (G)',
    'pipe_body_gx (°/s)', 'pipe_body_gy (°/s)', 'pipe_body_gz (°/s)',
    'kf6_x (m)', 'kf6_y (m)', 'kf6_vel (m/s)', 'kf6_heading (rad)', 'kf6_bgz (rad/s)',
    'zaru_flags', 'tbias_gz (°/s)',
    'bmi_post_lpf20_prepipe_ax (LSB)', 'bmi_post_lpf20_prepipe_ay (LSB)', 'bmi_post_lpf20_prepipe_az (LSB)',
    'bmi_post_lpf20_prepipe_gx (LSB)', 'bmi_post_lpf20_prepipe_gy (LSB)', 'bmi_post_lpf20_prepipe_gz (LSB)',
    'bmi_acc_x_g (G)', 'bmi_acc_y_g (G)', 'bmi_acc_z_g (G)',
    'bmi_gyr_x_dps (°/s)', 'bmi_gyr_y_dps (°/s)', 'bmi_gyr_z_dps (°/s)',
    'bmm_raw_x (LSB)', 'bmm_raw_y (LSB)', 'bmm_raw_z (LSB)',
    'bmm_rhall (LSB)',
    'bmm_ut_x (uT)', 'bmm_ut_y (uT)', 'bmm_ut_z (uT)',
    'mag_valid',
    'mag_sample_fresh',
    'mag_overflow',
    'imu_sample_fresh',
    'fifo_frames_drained',
    'fifo_backlog',
    'fifo_overrun',
    'reserved0',
    'gps_fix_us (µs)', 'gps_valid',
    'nav_speed2d (m/s)', 'nav_s_acc (m/s)', 'nav_vel_n (m/s)', 'nav_vel_e (m/s)',
    'nav_vel_valid', 'gps_speed_source', 'nav_fix_us (µs)',
    'dhv_gdspd (m/s)', 'dhv_fix_us (µs)',
]
COL_FMT_242 = [
    '{:d}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.1f}',
    '{:d}',
    '{:.7f}', '{:.7f}',
    '{:.2f}', '{:.2f}',
    '{:d}',
    '{:.2f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:d}', '{:.5f}',
    '{:d}', '{:d}', '{:d}', '{:d}', '{:d}', '{:d}',
    '{:.6f}', '{:.6f}', '{:.6f}', '{:.6f}', '{:.6f}', '{:.6f}',
    '{:d}', '{:d}', '{:d}',
    '{:d}',
    '{:.6f}', '{:.6f}', '{:.6f}',
    '{:d}', '{:d}', '{:d}', '{:d}', '{:d}', '{:d}', '{:d}', '{:d}',
    '{:d}', '{:d}',
    '{:.5f}', '{:.6f}', '{:.5f}', '{:.5f}',
    '{:d}', '{:d}', '{:d}',
    '{:.5f}', '{:d}',
]
assert struct.calcsize(FMT_242) == 242, f"FMT_242 mismatch: {struct.calcsize(FMT_242)}"

# 256-byte record (v1.7.6 header v5): sector-aligned tail with seq + uint16 SD diagnostics.
FMT_256_V5 = FMT_242 + 'I5H'
HEADER_256_V5 = HEADER_242 + [
    'seq',
    'sd_records_dropped',
    'sd_partial_write_count',
    'sd_stall_count',
    'sd_reopen_count',
    'sd_queue_hwm',
]
COL_FMT_256_V5 = COL_FMT_242 + ['{:d}'] * 6
assert struct.calcsize(FMT_256_V5) == 256, f"FMT_256_V5 mismatch: {struct.calcsize(FMT_256_V5)}"

# 256-byte record (header v6+): record magic + seq + compact SD diagnostics + CRC16.
FMT_256 = FMT_242 + 'II4BH'
HEADER_256_BASE = [
    ('sd_queue_hwm' if col == 'reserved0' else col)
    for col in HEADER_242
]
HEADER_256 = HEADER_256_BASE + [
    'record_magic',
    'seq',
    'sd_records_dropped',
    'sd_partial_write_count',
    'sd_stall_count',
    'sd_reopen_count',
    'crc16',
]
COL_FMT_256 = COL_FMT_242 + ['{:d}'] * 7
assert struct.calcsize(FMT_256) == 256, f"FMT_256 mismatch: {struct.calcsize(FMT_256)}"

# 320-byte record (v1.8.11 AtomS3R): GPS Supervisor shadow diagnostics.
FMT_320 = FMT_242 + 'ddfBBH3fBB26xII4BH'
HEADER_320 = HEADER_256_BASE + [
    'nav_lat (°)',
    'nav_lon (°)',
    'nav_h_acc (m)',
    'nav_fix_flags',
    'gps_supervisor_state',
    'gps_supervisor_reason',
    'gps_nmea_nav2_dist_m (m)',
    'gps_fix_step_speed_kmh (km/h)',
    'gps_eskf_innov_m (m)',
    'gps_supervisor_bad_count',
    'gps_supervisor_good_count',
    'record_magic',
    'seq',
    'sd_records_dropped',
    'sd_partial_write_count',
    'sd_stall_count',
    'sd_reopen_count',
    'crc16',
]
COL_FMT_320 = COL_FMT_242 + [
    '{:.7f}',
    '{:.7f}',
    '{:.3f}',
    '{:d}',
    '{:d}',
    '{:d}',
    '{:.3f}',
    '{:.2f}',
    '{:.3f}',
    '{:d}',
    '{:d}',
] + ['{:d}'] * 7
assert struct.calcsize(FMT_320) == 320, f"FMT_320 mismatch: {struct.calcsize(FMT_320)}"

# Default aliases (used for legacy files without header)
FMT_DEFAULT = FMT_122
HEADER = HEADER_122
COL_FMT = COL_FMT_122

def get_format(record_size, header_version=None):
    """Return (struct_fmt, header_list, col_fmt_list) for a given record size."""
    if record_size == 320:
        return FMT_320, HEADER_320, COL_FMT_320
    if record_size == 256:
        if header_version is not None and header_version < 6:
            return FMT_256_V5, HEADER_256_V5, COL_FMT_256_V5
        return FMT_256, HEADER_256, COL_FMT_256
    if record_size == 242:
        return FMT_242, HEADER_242, COL_FMT_242
    if record_size == 224:
        return FMT_224, HEADER_224, COL_FMT_224
    if record_size == 215:
        return FMT_215, HEADER_215, COL_FMT_215
    if record_size == 202:
        return FMT_202, HEADER_202, COL_FMT_202
    if record_size == 190:
        return FMT_190, HEADER_190, COL_FMT_190
    if record_size == 164:
        return FMT_164, HEADER_164, COL_FMT_164
    if record_size == 155:
        return FMT_155, HEADER_155, COL_FMT_155
    if record_size == 127:
        return FMT_127, HEADER_127, COL_FMT_127
    # Default: 122-byte format (also used for legacy files)
    return FMT_122, HEADER_122, COL_FMT_122

HEADER_LINE = ','.join(HEADER) + '\n'
CSV_PREAMBLE_LINES = []

# ── ANSI Colors ───────────────────────────────────────────────────────────────
BOLD  = '\033[1m'
DIM   = '\033[2m'
CYAN  = '\033[36m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
RED   = '\033[31m'
RESET = '\033[0m'


def banner():
    print(f"""
{BOLD}╔══════════════════════════════════════════════════════╗
║        TELEMETRIA — Converter & Splitter             ║
║        .bin (header+record) → split .csv             ║
╚══════════════════════════════════════════════════════╝{RESET}
""")


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def ask(prompt, default=None):
    """Input prompt with optional default shown in brackets."""
    suffix = f" [{default}]" if default else ""
    val = input(f"  {CYAN}▸{RESET} {prompt}{suffix}: ").strip()
    return val if val else (str(default) if default else "")


def ask_int(prompt, default=None, minimum=1):
    while True:
        val = ask(prompt, default)
        try:
            n = int(val)
            if n >= minimum:
                return n
            print(f"    {RED}Minimum: {minimum}{RESET}")
        except ValueError:
            print(f"    {RED}Please enter an integer.{RESET}")


def ask_float(prompt, default=None, minimum=0.1):
    while True:
        val = ask(prompt, default)
        try:
            n = float(val)
            if n >= minimum:
                return n
            print(f"    {RED}Minimum: {minimum}{RESET}")
        except ValueError:
            print(f"    {RED}Please enter a number.{RESET}")


def find_input_file():
    """Search for .bin and .csv files in the current directory.

    Keep this menu pass intentionally cheap. Older versions called
    bin_record_count() for every .bin found, which could scan large/preallocated
    logs just to print the selection list.
    """
    bins = []
    csvs = []
    with os.scandir('.') as entries:
        for entry in entries:
            if not entry.is_file():
                continue
            ext = os.path.splitext(entry.name)[1].lower()
            if ext not in ('.bin', '.csv'):
                continue
            try:
                size = entry.stat().st_size
            except OSError:
                continue
            if ext == '.bin':
                bins.append((entry.name, size))
            elif size > 1_000_000:  # only CSV >1MB
                csvs.append((entry.name, size))
    bins.sort(key=lambda item: item[0].lower())
    csvs.sort(key=lambda item: item[0].lower())

    if not bins and not csvs:
        print(f"  {RED}No .bin or .csv files found in the current directory.{RESET}")
        path = ask("File path")
        if not os.path.isfile(path):
            print(f"  {RED}File not found: {path}{RESET}")
            sys.exit(1)
        return path

    print(f"  {BOLD}Files found:{RESET}")
    options = []
    prealloc_seen = False
    for f, size in bins:
        size_mb = size / (1024 * 1024)
        records_text, detail_text, is_prealloc = describe_bin_for_menu(f, size)
        prealloc_seen = prealloc_seen or is_prealloc
        options.append(f)
        print(f"    {GREEN}{len(options)}.{RESET} {f}  "
              f"{DIM}({size_mb:.1f} MB, {records_text}, {detail_text}){RESET}")
    for f, size in csvs:
        size_mb = size / (1024 * 1024)
        options.append(f)
        print(f"    {GREEN}{len(options)}.{RESET} {f}  "
              f"{DIM}({size_mb:.1f} MB, CSV){RESET}")
    if prealloc_seen:
        print(f"  {DIM}Preallocated .bin counts are size-based in this menu; "
              f"exact valid rows are computed only after selection.{RESET}")
    print()

    if len(options) == 1:
        print(f"  Auto-selected: {BOLD}{options[0]}{RESET}")
        return options[0]

    choice = ask_int(f"Choose file (1-{len(options)})", 1, minimum=1)
    choice = min(choice, len(options))
    return options[choice - 1]


# ── FileHeader reader (v0.9.7+) ───────────────────────────────────────────────

def read_file_header(bin_path):
    """Read FileHeader if present (magic 'TEL'). Returns (header_dict, offset)
    or (None, 0) for legacy files without a header.

    Supports both header versions:
      v1 (25 bytes): record_size is uint8_t  — firmware <= v0.9.12
      v2 (26 bytes): record_size is uint16_t — firmware >= v0.9.13
      v3/v4 (66 bytes): adds boot calibration params
      v5 (80 bytes): adds header_flags + mag_ref_ut_xyz
    The header_version byte (offset 3) determines the layout.
    """
    with open(bin_path, 'rb') as f:
        preamble = f.read(4)  # magic (3) + header_version (1)
        if len(preamble) < 4 or preamble[:3] != HEADER_MAGIC:
            return None, 0

        hdr_ver = preamble[3]

        if hdr_ver >= 6:
            hdr_size = HEADER_V6_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            parts = struct.unpack('<3sB16sHI10fH3f', raw[:HEADER_V5_SIZE])
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = parts[:5]
            cal_params = parts[5:15]
            header_flags = parts[15]
            mag_ref = parts[16:19]
            ext = struct.unpack('<HHHHIIIHHHHH8B32s16s32s58s',
                                raw[HEADER_V5_SIZE:HEADER_V6_SIZE])
            (header_size, data_offset, endian_marker, header_crc16,
             build_flags, sd_spi_hz, imu_i2c_hz,
             sd_queue_depth, imu_queue_depth, sd_flush_every,
             record_magic_offset, record_crc_offset,
             reset_reason_cpu0, reset_reason_cpu1, board_id, imu_id,
             mag_id, gps_enabled, bmi_chip_id, bmm_chip_id,
             bmi_cfg_regs, bmm_cfg_regs, log_filename, _reserved) = ext
            crc_raw = bytearray(raw)
            crc_raw[HEADER_V5_SIZE + 6:HEADER_V5_SIZE + 8] = b'\x00\x00'
            header_crc_ok = crc16_ccitt(crc_raw) == header_crc16
            hdr_size = data_offset if data_offset else header_size
            v6_extension = {
                'header_size': header_size,
                'data_offset': data_offset,
                'endian_marker': endian_marker,
                'header_crc16': header_crc16,
                'header_crc_ok': header_crc_ok,
                'build_flags': build_flags,
                'sd_spi_hz': sd_spi_hz,
                'imu_i2c_hz': imu_i2c_hz,
                'sd_queue_depth': sd_queue_depth,
                'imu_queue_depth': imu_queue_depth,
                'sd_flush_every': sd_flush_every,
                'record_magic_offset': record_magic_offset,
                'record_crc_offset': record_crc_offset,
                'reset_reason_cpu0': reset_reason_cpu0,
                'reset_reason_cpu1': reset_reason_cpu1,
                'board_id': board_id,
                'imu_id': imu_id,
                'mag_id': mag_id,
                'gps_enabled': gps_enabled,
                'bmi_chip_id': bmi_chip_id,
                'bmm_chip_id': bmm_chip_id,
                'bmi_cfg_regs': bmi_cfg_regs,
                'bmm_cfg_regs': bmm_cfg_regs,
                'log_filename': log_filename.split(b'\x00')[0].decode('ascii', errors='replace'),
            }
        elif hdr_ver >= 5:
            hdr_size = HEADER_V5_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            parts = struct.unpack('<3sB16sHI10fH3f', raw)
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = parts[:5]
            cal_params = parts[5:15]
            header_flags = parts[15]
            mag_ref = parts[16:19]
            v6_extension = None
        elif hdr_ver >= 3:
            # v3/v4: 3s B 16s H I 10f = 3+1+16+2+4+40 = 66 bytes
            hdr_size = HEADER_V3_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            parts = struct.unpack('<3sB16sHI10f', raw)
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = parts[:5]
            cal_params = parts[5:]  # 10 float: sin_phi..bias_gz
            header_flags = 0
            mag_ref = None
            v6_extension = None
        elif hdr_ver >= 2:
            # v2: 3s B 16s H I = 3+1+16+2+4 = 26 bytes
            hdr_size = HEADER_V2_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = struct.unpack(
                '<3sB16sHI', raw)
            cal_params = None
            header_flags = 0
            mag_ref = None
            v6_extension = None
        else:
            # v1: 3s B 16s B I = 3+1+16+1+4 = 25 bytes
            hdr_size = HEADER_V1_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = struct.unpack(
                '<3sB16sBI', raw)
            cal_params = None
            header_flags = 0
            mag_ref = None
            v6_extension = None

        fw_str = fw_ver.split(b'\x00')[0].decode('ascii', errors='replace')
        result = {
            'header_version': hdr_ver_,
            'firmware_version': fw_str,
            'record_size': rec_size,
            'start_time_ms': start_ms,
            'header_flags': header_flags,
        }
        if cal_params is not None:
            result['calibration'] = cal_params
        if mag_ref is not None:
            result['mag_ref_ut'] = mag_ref
        if v6_extension is not None:
            result['v6'] = v6_extension
        return result, hdr_size


def describe_bin_for_menu(bin_path, size_bytes):
    """Return cheap display metadata for the interactive file picker."""
    try:
        hdr, offset = read_file_header(bin_path)
    except (OSError, struct.error, ValueError):
        hdr, offset = None, 0

    record_size = hdr.get('record_size') if hdr else CHUNK_SIZE_DEFAULT
    if not record_size:
        record_size = CHUNK_SIZE_DEFAULT
    max_records = max(0, size_bytes - offset) // record_size
    is_prealloc = _is_preallocated_log(hdr)

    if is_prealloc:
        records_text = f"prealloc <= {max_records:,} records"
    else:
        records_text = f"~{max_records:,} records"

    if hdr:
        detail_text = (
            f"binary, {hdr.get('firmware_version', 'unknown fw')}, "
            f"{record_size}B rec, header_v{hdr.get('header_version', '?')}"
        )
    else:
        detail_text = f"binary, legacy/default {record_size}B rec"

    return records_text, detail_text, is_prealloc


def open_text_read(path):
    """Open a text file with UTF-8 first, then legacy fallback for old CSVs."""
    last_exc = None
    for enc in TEXT_ENCODINGS_READ:
        try:
            f = open(path, 'r', encoding=enc)
            f.read(4096)
            f.seek(0)
            return f
        except UnicodeDecodeError as exc:
            last_exc = exc
            try:
                f.close()
            except Exception:
                pass
    if last_exc is not None:
        raise last_exc
    return open(path, 'r', encoding='utf-8')


def build_csv_preamble_lines(hdr):
    """Build metadata comment lines that must appear before the CSV header."""
    lines = []
    if hdr and 'calibration' in hdr:
        c = hdr['calibration']
        lines.append(
            f"# CALIB_BOOT: sin_phi={c[0]:.5f} cos_phi={c[1]:.5f} "
            f"sin_theta={c[2]:.5f} cos_theta={c[3]:.5f} "
            f"bias_ax={c[4]:.5f} bias_ay={c[5]:.5f} bias_az={c[6]:.5f} "
            f"bias_gx={c[7]:.5f} bias_gy={c[8]:.5f} bias_gz={c[9]:.5f}"
        )
    if hdr and hdr.get('record_size', 0) == 215:
        lines.append(
            "# COLUMN_NOTE: legacy AtomS3R 215B magnetometer columns are "
            "M5Unified-scaled arbitrary units, not Bosch-compensated physical uT."
        )
    if hdr and hdr.get('record_size', 0) == 224:
        lines.append(
            "# COLUMN_NOTE: mag_raw_* and mag_rhall are decoded native BMM150 "
            "register values; mag_ut_* are Bosch-compensated physical outputs in uT."
        )
        lines.append(
            "# COLUMN_NOTE: mag_valid=1 means the compensated reading is valid; "
            "mag_fresh=1 means the BMI270 AUX interface reported a fresh mag sample."
        )
    if hdr and hdr.get('record_size', 0) in (242, 256, 320):
        lines.append(
            "# COLUMN_NOTE: AtomS3R raw/FIFO schema. pipe_lin_* / pipe_body_* are zero-latency "
            "pipeline diagnostics; bmi_* / bmm_* are Bosch-direct acquisition truth."
        )
        lines.append(
            "# COLUMN_NOTE: mag_valid=1 means BMM150 compensation succeeded; "
            "mag_sample_fresh=1 means AUX delivered a fresh sample; mag_overflow/fifo_overrun flag sensor-side issues."
        )
        if hdr.get('record_size', 0) == 256:
            lines.append(
                "# COLUMN_NOTE: 256B v5 logs use seq + uint16 SD snapshots; header v6 logs add record_magic/crc16 and compact SD snapshots."
            )
        if hdr.get('record_size', 0) == 320:
            lines.append(
                "# COLUMN_NOTE: 320B v7 logs add NAV2-PVH position and GPS Supervisor shadow diagnostics; supervisor fields are diagnostic only."
            )
        if hdr.get('record_size', 0) in (256, 320):
            if hdr.get('header_version', 0) >= 6 and 'v6' in hdr:
                v6 = hdr['v6']
                lines.append(
                    f"# HEADER_V6: header_size={v6['header_size']} data_offset={v6['data_offset']} "
                    f"crc_ok={int(v6['header_crc_ok'])} sd_spi_hz={v6['sd_spi_hz']} "
                    f"sd_queue_depth={v6['sd_queue_depth']} imu_queue_depth={v6['imu_queue_depth']} "
                    f"sd_flush_every={v6['sd_flush_every']}"
                )
                lines.append(
                    f"# HEADER_V6_REGS: bmi_chip_id=0x{v6['bmi_chip_id']:02X} "
                    f"bmm_chip_id=0x{v6['bmm_chip_id']:02X} reset_cpu0={v6['reset_reason_cpu0']} "
                    f"build_flags=0x{v6['build_flags']:08X} log_filename={v6['log_filename']}"
                )
        if hdr.get('header_flags', 0) & 0x0001 and 'mag_ref_ut' in hdr:
            mx, my, mz = hdr['mag_ref_ut']
            lines.append(
                f"# MAG_REF_BOOT_UT: mx={mx:.6f} my={my:.6f} mz={mz:.6f}"
            )
        else:
            lines.append("# MAG_REF_BOOT_UT: invalid")
    return lines


def write_csv_preamble(fobj):
    """Write metadata comment lines followed by the CSV header."""
    for line in CSV_PREAMBLE_LINES:
        fobj.write(line + '\n')
    fobj.write(HEADER_LINE)


def is_csv_comment_line(line):
    """True for metadata/comment lines embedded in the CSV body."""
    return line.startswith('#')


def read_csv_preamble_and_header(csv_path):
    """Return (leading_comment_lines, header_line) from an existing CSV file."""
    preamble = []
    with open_text_read(csv_path) as f:
        for raw_line in f:
            stripped = raw_line.rstrip('\n\r')
            if not stripped:
                continue
            if stripped.startswith('#'):
                preamble.append(stripped)
                continue
            return preamble, stripped + '\n'
    raise ValueError(f"CSV header not found in {csv_path}")


# ── Binary → CSV row conversion ───────────────────────────────────────────────

def _base_col_name(name):
    return name.split(' (', 1)[0].strip()


def _field_indices(header_cols):
    return {_base_col_name(name): i for i, name in enumerate(header_cols)}


def _field_value(vals, fields, name):
    idx = fields.get(name)
    return vals[idx] if idx is not None else None


def _finite_abs_ok(value, limit):
    return isinstance(value, (int, float)) and math.isfinite(value) and abs(value) <= limit


def _range_ok(value, lo, hi):
    return isinstance(value, (int, float)) and math.isfinite(value) and lo <= value <= hi


def _timestamp_us(vals, fields):
    if 't_us' in fields:
        return int(vals[fields['t_us']])
    if 'timestamp_us' in fields:
        return int(vals[fields['timestamp_us']])
    if 't_ms' in fields:
        return int(vals[fields['t_ms']]) * 1000
    return None


def _record_seq(vals, fields):
    value = _field_value(vals, fields, 'seq')
    return int(value) if value is not None else None


def _record_integrity_ok(vals, fields, raw=None):
    value = _field_value(vals, fields, 'record_magic')
    if value is not None and value != RECORD_MAGIC_256_V6:
        return False

    value = _field_value(vals, fields, 'crc16')
    if value is not None and raw is not None:
        if crc16_ccitt(raw[:-2]) != value:
            return False

    return True


def _fmt_sentinel_float(value, digits=5):
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = float('nan')
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def _mount_yaw_obs_comment(vals, fields):
    """Decode the v1.8.3 mount-yaw observation sentinel payload."""
    yaw_deg = _field_value(vals, fields, 'ax')
    quality = _field_value(vals, fields, 'ay')
    duration_s = _field_value(vals, fields, 'az')
    mean_speed_kmh = _field_value(vals, fields, 'gx')
    mean_abs_acc_g = _field_value(vals, fields, 'gy')
    rms_gz_dps = _field_value(vals, fields, 'gz')
    rms_cog_rate_dps = _field_value(vals, fields, 'temp_c')
    obs_id = _field_value(vals, fields, 'lap')
    long_corr = _field_value(vals, fields, 'gps_sog_kmh')
    lat_rms_before_g = _field_value(vals, fields, 'gps_alt_m')
    lat_rms_after_g = _field_value(vals, fields, 'gps_hdop')
    lat_reduction_pct = _field_value(vals, fields, 'kf_x')
    rms_lat_expected_g = _field_value(vals, fields, 'kf_y')
    rms_az_g = _field_value(vals, fields, 'kf_vel')
    t_start_us = _field_value(vals, fields, 'gps_fix_us')
    t_end_us = _field_value(vals, fields, 'nav_fix_us')
    return (
        "# MOUNT_YAW_OBS: "
        f"obs_id={int(obs_id or 0)} "
        f"yaw_deg={_fmt_sentinel_float(yaw_deg)} "
        f"quality={_fmt_sentinel_float(quality, 3)} "
        f"t_start_us={int(t_start_us or 0)} "
        f"t_end_us={int(t_end_us or 0)} "
        f"duration_s={_fmt_sentinel_float(duration_s, 3)} "
        f"mean_speed_kmh={_fmt_sentinel_float(mean_speed_kmh, 2)} "
        f"mean_abs_acc_g={_fmt_sentinel_float(mean_abs_acc_g, 5)} "
        f"rms_gz_dps={_fmt_sentinel_float(rms_gz_dps, 3)} "
        f"rms_cog_rate_dps={_fmt_sentinel_float(rms_cog_rate_dps, 3)} "
        f"long_corr={_fmt_sentinel_float(long_corr, 3)} "
        f"lat_rms_before_g={_fmt_sentinel_float(lat_rms_before_g, 5)} "
        f"lat_rms_after_g={_fmt_sentinel_float(lat_rms_after_g, 5)} "
        f"lat_reduction_pct={_fmt_sentinel_float(lat_reduction_pct, 2)} "
        f"rms_lat_expected_g={_fmt_sentinel_float(rms_lat_expected_g, 5)} "
        f"rms_az_g={_fmt_sentinel_float(rms_az_g, 5)}"
    )


def _mount_yaw_boot_comment(vals, fields):
    """Decode the v1.8.4 accepted mount-yaw boot sentinel payload."""
    yaw_deg = _field_value(vals, fields, 'ax')
    consensus_std_deg = _field_value(vals, fields, 'ay')
    obs_count = _field_value(vals, fields, 'az')
    avg_quality = _field_value(vals, fields, 'gx')
    avg_long_corr = _field_value(vals, fields, 'gy')
    avg_stability_std_deg = _field_value(vals, fields, 'gz')
    sin_yaw = _field_value(vals, fields, 'gps_sog_kmh')
    cos_yaw = _field_value(vals, fields, 'gps_alt_m')
    emitted_us = _field_value(vals, fields, 'gps_fix_us')
    nav_fix_us = _field_value(vals, fields, 'nav_fix_us')
    return (
        "# MOUNT_YAW_BOOT: "
        f"yaw_deg={_fmt_sentinel_float(yaw_deg)} "
        f"consensus_std_deg={_fmt_sentinel_float(consensus_std_deg, 3)} "
        f"obs_count={int(obs_count or 0)} "
        f"avg_quality={_fmt_sentinel_float(avg_quality, 3)} "
        f"avg_long_corr={_fmt_sentinel_float(avg_long_corr, 3)} "
        f"avg_stability_std_deg={_fmt_sentinel_float(avg_stability_std_deg, 3)} "
        f"sin_yaw={_fmt_sentinel_float(sin_yaw, 6)} "
        f"cos_yaw={_fmt_sentinel_float(cos_yaw, 6)} "
        f"emitted_us={int(emitted_us or 0)} "
        f"nav_fix_us={int(nav_fix_us or 0)}"
    )


def _record_plausible(vals, fields, prev_ts_us=None, raw=None):
    """Heuristic guard used only to recover from byte-level SD log misalignment."""
    if not _record_integrity_ok(vals, fields, raw):
        return False

    ts_us = _timestamp_us(vals, fields)
    if ts_us in (SENTINEL_CALIB, SENTINEL_MOUNT_YAW_OBS, SENTINEL_MOUNT_YAW_BOOT):
        return True
    if ts_us is not None:
        if not (0 <= ts_us <= MAX_TIMESTAMP_US):
            return False
        if prev_ts_us is not None:
            dt_us = ts_us - prev_ts_us
            if not (0 < dt_us <= MAX_TIMESTAMP_GAP_US):
                return False

    for name in ('ax', 'ay', 'az', 'raw_ax', 'raw_ay', 'raw_az',
                 'pipe_lin_ax', 'pipe_lin_ay', 'pipe_lin_az',
                 'sensor_ax', 'sensor_ay', 'sensor_az',
                 'bmi_acc_x_g', 'bmi_acc_y_g', 'bmi_acc_z_g'):
        value = _field_value(vals, fields, name)
        if value is not None and not _finite_abs_ok(value, 20.0):
            return False

    for name in ('gx', 'gy', 'gz', 'raw_gx', 'raw_gy', 'raw_gz',
                 'pipe_body_gx', 'pipe_body_gy', 'pipe_body_gz',
                 'sensor_gx', 'sensor_gy', 'sensor_gz',
                 'bmi_gyr_x_dps', 'bmi_gyr_y_dps', 'bmi_gyr_z_dps'):
        value = _field_value(vals, fields, name)
        if value is not None and not _finite_abs_ok(value, 5000.0):
            return False

    for name in ('mag_ut_x', 'mag_ut_y', 'mag_ut_z',
                 'bmm_ut_x', 'bmm_ut_y', 'bmm_ut_z'):
        value = _field_value(vals, fields, name)
        if value is not None and not _finite_abs_ok(value, 10000.0):
            return False

    for name in ('nav_speed2d', 'nav_vel_n', 'nav_vel_e', 'dhv_gdspd'):
        value = _field_value(vals, fields, name)
        if value is not None and not _finite_abs_ok(value, 300.0):
            return False

    value = _field_value(vals, fields, 'nav_s_acc')
    if value is not None and not _range_ok(value, 0.0, 1000.0):
        return False

    value = _field_value(vals, fields, 'temp_c')
    if value is not None and not _range_ok(value, -40.0, 125.0):
        return False

    value = _field_value(vals, fields, 'gps_lat')
    if value is not None and value != 0.0 and not _range_ok(value, -90.0, 90.0):
        return False

    value = _field_value(vals, fields, 'gps_lon')
    if value is not None and value != 0.0 and not _range_ok(value, -180.0, 180.0):
        return False

    value = _field_value(vals, fields, 'nav_lat')
    if value is not None and value != 0.0 and not _range_ok(value, -90.0, 90.0):
        return False

    value = _field_value(vals, fields, 'nav_lon')
    if value is not None and value != 0.0 and not _range_ok(value, -180.0, 180.0):
        return False

    range_checks = (
        ('gps_sog_kmh', 0.0, 500.0),
        ('gps_alt_m', -1000.0, 10000.0),
        ('gps_sats', 0, 80),
        ('gps_hdop', 0.0, 100.0),
        ('nav_h_acc', 0.0, 10000.0),
        ('gps_nmea_nav2_dist_m', -1.0, 100000.0),
        ('gps_fix_step_speed_kmh', -1.0, 1000.0),
        ('gps_eskf_innov_m', -1.0, 100000.0),
        ('lap', 0, 255),
        ('fifo_frames_drained', 0, 64),
        ('fifo_backlog', 0, 255),
        ('nav_fix_flags', 0, 255),
        ('gps_supervisor_state', 0, 3),
        ('gps_supervisor_reason', 0, 65535),
        ('gps_supervisor_bad_count', 0, 255),
        ('gps_supervisor_good_count', 0, 255),
    )
    for name, lo, hi in range_checks:
        value = _field_value(vals, fields, name)
        if value is not None and not _range_ok(value, lo, hi):
            return False

    for name in ('mag_valid', 'mag_fresh', 'mag_valid_legacy',
                 'mag_sample_fresh', 'mag_overflow', 'imu_sample_fresh',
                 'fifo_overrun', 'gps_valid'):
        value = _field_value(vals, fields, name)
        if value is not None and value not in (0, 1):
            return False

    value = _field_value(vals, fields, 'reserved0')
    if value is not None and value != 0:
        return False

    value = _field_value(vals, fields, 'nav_vel_valid')
    if value is not None and not _range_ok(value, 0, 7):
        return False

    value = _field_value(vals, fields, 'gps_speed_source')
    if value is not None and value not in (0, 1, 2):
        return False

    return True


def _find_resync_skip(fobj, start_pos, fmt, chunk_size, fields,
                      prev_ts_us=None, scan_bytes=RESYNC_SCAN_BYTES):
    window_len = scan_bytes + RESYNC_CONFIRM_RECORDS * chunk_size
    fobj.seek(start_pos)
    window = fobj.read(window_len)
    max_skip = min(scan_bytes, len(window) - RESYNC_CONFIRM_RECORDS * chunk_size)
    if max_skip < 1:
        return None

    for skip in range(1, max_skip + 1):
        candidate_prev_ts_us = prev_ts_us
        ok = True
        for i in range(RESYNC_CONFIRM_RECORDS):
            off = skip + i * chunk_size
            vals = struct.unpack_from(fmt, window, off)
            raw = window[off:off + chunk_size]
            if not _record_plausible(vals, fields, candidate_prev_ts_us, raw):
                ok = False
                break
            ts_us = _timestamp_us(vals, fields)
            if ts_us is not None and ts_us not in (SENTINEL_CALIB, SENTINEL_MOUNT_YAW_OBS, SENTINEL_MOUNT_YAW_BOOT):
                candidate_prev_ts_us = ts_us
        if ok:
            return skip
    return None


def _is_preallocated_log(hdr):
    return (
        hdr is not None
        and hdr.get('header_version', 0) >= 6
        and hdr.get('record_size') in (256, 320)
        and (hdr.get('v6', {}).get('build_flags', 0) & FILE_HEADER_V6_FLAG_LOG_PREALLOCATED)
    )


def _tail_is_all_zero(fobj, start_pos, chunk_bytes=1024 * 1024):
    """Return True only if the remaining file tail is entirely preallocated zeros."""
    fobj.seek(start_pos)
    zero_chunk = b'\x00' * chunk_bytes
    while True:
        chunk = fobj.read(chunk_bytes)
        if not chunk:
            return True
        # Do not allocate/read the whole preallocated tail at once. A single
        # non-zero byte means this may be real corruption or later valid data,
        # so the caller must keep the physical file size.
        expected = zero_chunk if len(chunk) == chunk_bytes else b'\x00' * len(chunk)
        if chunk != expected:
            return False


def _record_valid_for_logical_scan(chunk, fmt, fields):
    vals = struct.unpack(fmt, chunk)
    ts_us = _timestamp_us(vals, fields)
    if ts_us in (SENTINEL_CALIB, SENTINEL_MOUNT_YAW_OBS, SENTINEL_MOUNT_YAW_BOOT):
        return True
    return _record_plausible(vals, fields, prev_ts_us=None, raw=chunk)


def _scan_preallocated_logical_end_fast(bin_path, hdr, offset, chunk_size, fmt, fields):
    """Binary-search the first invalid preallocated record.

    Firmware writes a contiguous prefix of valid records followed by unwritten
    preallocated space. This avoids an O(n) CRC pass over multi-hour 128 MiB
    logs while preserving the same logical EOF model used by boot repair.
    """
    if not _is_preallocated_log(hdr):
        return None

    file_size = os.path.getsize(bin_path)
    max_records = max(0, file_size - offset) // chunk_size
    if max_records <= 0:
        return offset, 0, max(0, file_size - offset)

    def valid_at(fobj, idx):
        pos = offset + idx * chunk_size
        fobj.seek(pos)
        chunk = fobj.read(chunk_size)
        if len(chunk) != chunk_size:
            return False
        return _record_valid_for_logical_scan(chunk, fmt, fields)

    with open(bin_path, 'rb') as f:
        lo = 0
        hi = max_records
        while lo < hi:
            mid = lo + (hi - lo) // 2
            if valid_at(f, mid):
                lo = mid + 1
            else:
                hi = mid

        # Local correction keeps the result conservative if the boundary is near
        # a sentinel or if the binary search lands a few records past EOF.
        start = max(0, lo - 16)
        end = min(max_records, lo + 64)
        first_invalid = lo
        for idx in range(start, end):
            if not valid_at(f, idx):
                first_invalid = idx
                break
        else:
            first_invalid = lo

    logical_end = offset + first_invalid * chunk_size
    return logical_end, first_invalid, file_size - logical_end


def _scan_preallocated_logical_end(bin_path, hdr, offset, chunk_size, fmt, fields):
    """Return (logical_end, valid_records, tail_bytes) for v6 preallocated logs.

    The scan stops at the first invalid magic/CRC/plausibility record. It treats
    the remaining bytes as preallocated tail if they are all zero. For v6 logs
    with stale non-zero preallocated cluster contents, a structurally invalid
    record (bad magic/CRC) after a valid sequence is also treated as logical EOF,
    unless a confirmed current-stream resync is found shortly after it.
    """
    if not _is_preallocated_log(hdr):
        size = os.path.getsize(bin_path)
        return size, max(0, size - offset) // chunk_size, 0

    fast_result = _scan_preallocated_logical_end_fast(
        bin_path, hdr, offset, chunk_size, fmt, fields
    )
    if fast_result is not None:
        return fast_result

    file_size = os.path.getsize(bin_path)
    logical_end = file_size
    valid_records = 0
    prev_ts_us = None
    invalid_pos = None
    invalid_integrity = False

    with open(bin_path, 'rb') as f:
        f.seek(offset)
        while True:
            record_pos = f.tell()
            chunk = f.read(chunk_size)
            if len(chunk) < chunk_size:
                invalid_pos = record_pos
                break
            vals = struct.unpack(fmt, chunk)
            integrity_ok = _record_integrity_ok(vals, fields, chunk)
            if not integrity_ok or not _record_plausible(vals, fields, prev_ts_us, chunk):
                invalid_pos = record_pos
                invalid_integrity = not integrity_ok
                break
            ts_us = _timestamp_us(vals, fields)
            if ts_us is not None and ts_us not in (SENTINEL_CALIB, SENTINEL_MOUNT_YAW_OBS, SENTINEL_MOUNT_YAW_BOOT):
                prev_ts_us = ts_us
            valid_records += 1

        if invalid_pos is None or invalid_pos >= file_size:
            return file_size, valid_records, 0

        if invalid_pos < file_size and _tail_is_all_zero(f, invalid_pos):
            logical_end = invalid_pos
            return logical_end, valid_records, file_size - logical_end

        if invalid_integrity and valid_records > 0:
            # Preallocated v6 files can expose stale non-zero cluster contents
            # after the last written record. Do not mistake a real mid-stream
            # corruption for EOF if the same stream resumes nearby.
            skip = _find_resync_skip(
                f, invalid_pos, fmt, chunk_size, fields,
                prev_ts_us=prev_ts_us,
                scan_bytes=PREALLOC_TAIL_RESYNC_SCAN_BYTES,
            )
            if skip is None:
                logical_end = invalid_pos
                return logical_end, valid_records, file_size - logical_end

    return file_size, max(0, file_size - offset) // chunk_size, 0


def bin_logical_layout(bin_path, quiet=False):
    hdr, offset = read_file_header(bin_path)
    chunk_size = hdr['record_size'] if hdr else CHUNK_SIZE_DEFAULT
    fmt, hdr_cols, _ = get_format(chunk_size, hdr.get('header_version') if hdr else None)
    fields = _field_indices(hdr_cols)
    logical_end, valid_records, tail_bytes = _scan_preallocated_logical_end(
        bin_path, hdr, offset, chunk_size, fmt, fields
    )
    if tail_bytes and not quiet:
        print(f"    {YELLOW}INFO:{RESET} preallocated tail ignored: "
              f"{tail_bytes:,} byte(s); logical size={logical_end:,} byte(s)")
    return {
        'hdr': hdr,
        'offset': offset,
        'chunk_size': chunk_size,
        'logical_end': logical_end,
        'valid_records': valid_records,
        'tail_bytes': tail_bytes,
    }


def repair_bin_preallocated_tail(bin_path):
    layout = bin_logical_layout(bin_path, quiet=True)
    tail_bytes = layout['tail_bytes']
    if tail_bytes <= 0:
        print(f"  {DIM}No preallocated tail to repair.{RESET}")
        return False
    os.truncate(bin_path, layout['logical_end'])
    print(f"  {GREEN}Repaired:{RESET} truncated {tail_bytes:,} byte(s); "
          f"new size={layout['logical_end']:,} byte(s)")
    return True


def bin_record_count(bin_path, layout=None):
    if layout is None:
        layout = bin_logical_layout(bin_path, quiet=True)
    return layout['valid_records']


def bin_to_csv_lines(bin_path, total=None, layout=None):
    """Yield CSV rows from a binary telemetry file (with or without FileHeader)."""
    if layout is None:
        layout = bin_logical_layout(bin_path)
    hdr = layout['hdr']
    offset = layout['offset']
    chunk_size = layout['chunk_size']
    logical_end = layout['logical_end']
    fmt, hdr_cols, col_fmt = get_format(chunk_size, hdr.get('header_version') if hdr else None)
    fields = _field_indices(hdr_cols)

    with open(bin_path, 'rb') as f:
        if offset > 0:
            f.seek(offset)
        count = 0
        resync_count = 0
        seq_gap_count = 0
        seq_missing_total = 0
        prev_ts_us = None
        prev_seq = None
        while True:
            record_pos = f.tell()
            if record_pos >= logical_end:
                break
            chunk = f.read(chunk_size)
            if len(chunk) < chunk_size:
                break
            vals = struct.unpack(fmt, chunk)
            if vals[0] == SENTINEL_CALIB:   # CalibrationRecord sentinel (uint64 max)
                yield f"# CALIB_UPDATE: sin_phi={vals[1]:.5f} cos_phi={vals[2]:.5f} sin_theta={vals[3]:.5f} cos_theta={vals[4]:.5f} bias_ax={vals[5]:.5f} bias_ay={vals[6]:.5f} bias_az={vals[7]:.5f} bias_gx={vals[11]:.5f} bias_gy={vals[12]:.5f} bias_gz={vals[14]:.5f}"
                prev_ts_us = None
                prev_seq = None
                continue
            if vals[0] == SENTINEL_MOUNT_YAW_OBS:
                yield _mount_yaw_obs_comment(vals, fields)
                prev_ts_us = None
                prev_seq = None
                continue
            if vals[0] == SENTINEL_MOUNT_YAW_BOOT:
                yield _mount_yaw_boot_comment(vals, fields)
                prev_ts_us = None
                prev_seq = None
                continue
            if not _record_plausible(vals, fields, prev_ts_us, chunk):
                skip = _find_resync_skip(f, record_pos, fmt, chunk_size, fields)
                if skip is None:
                    # Fallback: skip one record worth of bytes and keep trying
                    print(f"\n    {YELLOW}WARN:{RESET} no resync found at byte {record_pos:,}; "
                          f"skipping {chunk_size} bytes")
                    f.seek(record_pos + chunk_size)
                    prev_ts_us = None
                    continue
                resync_count += 1
                print(f"\n    {YELLOW}WARN:{RESET} resynced binary stream at byte {record_pos:,}; "
                      f"skipped {skip} byte(s)")
                f.seek(record_pos + skip)
                prev_ts_us = None
                prev_seq = None
                continue
            seq = _record_seq(vals, fields)
            if prev_seq is not None and seq is not None and seq != prev_seq + 1:
                missing = max(0, seq - prev_seq - 1)
                seq_gap_count += 1
                seq_missing_total += missing
                ts_us = _timestamp_us(vals, fields)
                dt_us = (ts_us - prev_ts_us) if ts_us is not None and prev_ts_us is not None else None
                dt_text = f", dt={dt_us / 1000000.0:.6f}s" if dt_us is not None else ""
                print(f"\n    {YELLOW}WARN:{RESET} sequence gap at byte {record_pos:,}: "
                      f"seq {prev_seq} -> {seq}, missing {missing}{dt_text}")
            line = ','.join(f.format(v) for f, v in zip(col_fmt, vals))
            yield line
            prev_ts_us = _timestamp_us(vals, fields)
            prev_seq = seq
            count += 1
            if count % PROGRESS_EVERY == 0 and total:
                pct = count * 100 // total
                print(f"    Read {count:>10,} / {total:,}  ({pct}%)", end='\r')
        if resync_count:
            print(f"\n    {YELLOW}Resync events:{RESET} {resync_count}")
        if seq_gap_count:
            print(f"\n    {YELLOW}Sequence gap events:{RESET} {seq_gap_count} "
                  f"(missing {seq_missing_total} record(s) by seq)")
    if total:
        status = "100%" if count >= total else "done"
        print(f"    Read {count:>10,} / {total:,}  ({status})    ")


def csv_to_lines(csv_path):
    """Yield CSV body lines, preserving inline metadata after the header."""
    total = count_csv_lines(csv_path)
    with open_text_read(csv_path) as f:
        header_seen = False
        count = 0
        for line in f:
            stripped = line.rstrip('\n\r')
            if not stripped:
                continue
            if not header_seen:
                if stripped.startswith('#'):
                    continue
                header_seen = True
                continue
            if stripped:
                yield stripped
                if not is_csv_comment_line(stripped):
                    count += 1
                if count and count % PROGRESS_EVERY == 0:
                    pct = count * 100 // total if total else 0
                    print(f"    Read {count:>10,} / {total:,}  ({pct}%)", end='\r')
    print(f"    Read {count:>10,} / {total:,}  (100%)    ")


def count_csv_lines(csv_path):
    """Count non-empty body lines, excluding leading metadata comments and header."""
    count = 0
    with open_text_read(csv_path) as f:
        header_seen = False
        for line in f:
            stripped = line.rstrip('\n\r')
            if not stripped:
                continue
            if not header_seen:
                if stripped.startswith('#'):
                    continue
                header_seen = True
                continue
            if not is_csv_comment_line(stripped):
                count += 1
    return count


# ── Diagnostic report ─────────────────────────────────────────────────────────

def _status_rank(status):
    return {'PASS': 0, 'WARN': 1, 'FAIL': 2}.get(status, 0)


def _worst_status(*statuses):
    return max(statuses or ('PASS',), key=_status_rank)


def _add_issue(issues, level, code, message):
    issues.append({'level': level, 'code': code, 'message': message})


def _status_from_issues(issues):
    if any(i['level'] == 'FAIL' for i in issues):
        return 'FAIL'
    if any(i['level'] == 'WARN' for i in issues):
        return 'WARN'
    return 'PASS'


def _safe_float(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _safe_int(value):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _ratio(numerator, denominator):
    return (float(numerator) / float(denominator)) if denominator else None


def _round_or_none(value, digits=6):
    return round(value, digits) if value is not None and math.isfinite(value) else None


def _top_event_insert(events, event, key='value', limit=10):
    events.append(event)
    events.sort(key=lambda item: item.get(key, 0), reverse=True)
    if len(events) > limit:
        del events[limit:]


def _parse_tel_index(path):
    name = os.path.basename(path)
    match = re.match(r'^tel_?(\d+)', name, re.IGNORECASE)
    return int(match.group(1)) if match else None


def _report_output_base_from_targets(targets):
    if len(targets) == 1 and os.path.isfile(targets[0]):
        return os.path.splitext(targets[0])[0]
    dirs = []
    for target in targets:
        dirs.append(target if os.path.isdir(target) else os.path.dirname(target) or os.getcwd())
    common = os.path.commonpath([os.path.abspath(d) for d in dirs]) if dirs else os.getcwd()
    return os.path.join(common, 'telemetry_session')


def _report_paths(output_base, report_format):
    if report_format not in ('both', 'json', 'md'):
        raise ValueError(f"Unsupported report format: {report_format}")
    paths = []
    if report_format in ('both', 'json'):
        paths.append(output_base + '.report.json')
    if report_format in ('both', 'md'):
        paths.append(output_base + '.report.md')
    return paths


def _resolve_report_targets(targets):
    files = []
    for target in targets:
        if os.path.isdir(target):
            for entry in os.scandir(target):
                if entry.is_file() and os.path.splitext(entry.name)[1].lower() in ('.bin', '.csv'):
                    files.append(entry.path)
        elif os.path.isfile(target):
            files.append(target)
        else:
            raise FileNotFoundError(target)
    files = sorted(
        files,
        key=lambda p: (
            _parse_tel_index(p) is None,
            _parse_tel_index(p) if _parse_tel_index(p) is not None else 10**9,
            os.path.basename(p).lower(),
        )
    )
    return files


def _header_report(hdr):
    if not hdr:
        return {
            'present': False,
            'firmware_version': None,
            'header_version': None,
            'record_size': None,
            'reset_reason_cpu0': None,
            'log_filename': None,
            'header_crc_ok': None,
        }
    v6 = hdr.get('v6') or {}
    return {
        'present': True,
        'firmware_version': hdr.get('firmware_version'),
        'header_version': hdr.get('header_version'),
        'record_size': hdr.get('record_size'),
        'start_time_ms': hdr.get('start_time_ms'),
        'header_flags': hdr.get('header_flags'),
        'reset_reason_cpu0': v6.get('reset_reason_cpu0'),
        'reset_reason_cpu1': v6.get('reset_reason_cpu1'),
        'log_filename': v6.get('log_filename'),
        'header_crc_ok': v6.get('header_crc_ok'),
        'build_flags': v6.get('build_flags'),
        'sd_spi_hz': v6.get('sd_spi_hz'),
        'sd_queue_depth': v6.get('sd_queue_depth'),
        'imu_queue_depth': v6.get('imu_queue_depth'),
        'sd_flush_every': v6.get('sd_flush_every'),
    }


def _new_metric_accumulators():
    return {
        'data_records': 0,
        'first_seq': None,
        'last_seq': None,
        'first_t_us': None,
        'last_t_us': None,
        'seq_gap_count': 0,
        'seq_missing_total': 0,
        'max_seq_gap_abs': 0,
        'timestamp_backsteps': 0,
        'min_dt_ms': None,
        'max_dt_ms': None,
        'max_sd_queue_hwm': 0,
        'max_sd_records_dropped': 0,
        'max_sd_partial_write_count': 0,
        'max_sd_stall_count': 0,
        'max_sd_reopen_count': 0,
        'gps_valid_count': 0,
        'gps_fresh_count': 0,
        'max_gps_fix_step_speed_kmh': None,
        'max_gps_eskf_innov_m': None,
        'max_gps_nmea_nav2_dist_m': None,
        'supervisor_state_counts': Counter(),
        'supervisor_reason_bit_counts': Counter(),
        'kf_step_over_threshold_count': 0,
        'max_kf_step_m': None,
        'top_kf_step_events': [],
        'sentinel_counts': {
            'CALIB_UPDATE': 0,
            'MOUNT_YAW_OBS': 0,
            'MOUNT_YAW_BOOT': 0,
        },
        'invalid_integrity_records': 0,
    }


def _update_metric_accumulators(acc, vals, fields, kf_threshold_m, prev):
    ts_us = _timestamp_us(vals, fields)
    if ts_us == SENTINEL_CALIB:
        acc['sentinel_counts']['CALIB_UPDATE'] += 1
        prev.clear()
        return
    if ts_us == SENTINEL_MOUNT_YAW_OBS:
        acc['sentinel_counts']['MOUNT_YAW_OBS'] += 1
        prev.clear()
        return
    if ts_us == SENTINEL_MOUNT_YAW_BOOT:
        acc['sentinel_counts']['MOUNT_YAW_BOOT'] += 1
        prev.clear()
        return

    acc['data_records'] += 1

    seq = _record_seq(vals, fields)
    if seq is not None:
        if acc['first_seq'] is None:
            acc['first_seq'] = seq
        acc['last_seq'] = seq
        prev_seq = prev.get('seq')
        if prev_seq is not None and seq != prev_seq + 1:
            gap_abs = abs(seq - prev_seq)
            missing = max(0, seq - prev_seq - 1)
            acc['seq_gap_count'] += 1
            acc['seq_missing_total'] += missing
            acc['max_seq_gap_abs'] = max(acc['max_seq_gap_abs'], gap_abs)
        prev['seq'] = seq

    if ts_us is not None:
        if acc['first_t_us'] is None:
            acc['first_t_us'] = ts_us
        acc['last_t_us'] = ts_us
        prev_t = prev.get('t_us')
        if prev_t is not None:
            dt_ms = (ts_us - prev_t) / 1000.0
            if dt_ms < 0:
                acc['timestamp_backsteps'] += 1
            else:
                acc['min_dt_ms'] = dt_ms if acc['min_dt_ms'] is None else min(acc['min_dt_ms'], dt_ms)
                acc['max_dt_ms'] = dt_ms if acc['max_dt_ms'] is None else max(acc['max_dt_ms'], dt_ms)
        prev['t_us'] = ts_us

    for field, metric in (
        ('sd_queue_hwm', 'max_sd_queue_hwm'),
        ('sd_records_dropped', 'max_sd_records_dropped'),
        ('sd_partial_write_count', 'max_sd_partial_write_count'),
        ('sd_stall_count', 'max_sd_stall_count'),
        ('sd_reopen_count', 'max_sd_reopen_count'),
    ):
        value = _field_value(vals, fields, field)
        if value is not None:
            acc[metric] = max(acc[metric], int(value))

    gps_valid = _field_value(vals, fields, 'gps_valid')
    if gps_valid == 1:
        acc['gps_valid_count'] += 1
        gps_fix_us = _field_value(vals, fields, 'gps_fix_us')
        if ts_us is not None and gps_fix_us is not None:
            age_us = ts_us - int(gps_fix_us)
            if 0 <= age_us <= GPS_FRESH_MAX_US_REPORT:
                acc['gps_fresh_count'] += 1

    for field, metric in (
        ('gps_fix_step_speed_kmh', 'max_gps_fix_step_speed_kmh'),
        ('gps_eskf_innov_m', 'max_gps_eskf_innov_m'),
        ('gps_nmea_nav2_dist_m', 'max_gps_nmea_nav2_dist_m'),
    ):
        value = _safe_float(_field_value(vals, fields, field))
        if value is not None:
            acc[metric] = value if acc[metric] is None else max(acc[metric], value)

    sup_state = _field_value(vals, fields, 'gps_supervisor_state')
    if sup_state is not None:
        state_id = int(sup_state)
        acc['supervisor_state_counts'][GPS_SUPERVISOR_STATE_NAMES.get(state_id, str(state_id))] += 1

    sup_reason = _field_value(vals, fields, 'gps_supervisor_reason')
    if sup_reason is not None:
        mask = int(sup_reason)
        for bit, name in GPS_SUPERVISOR_REASON_NAMES.items():
            if mask & bit:
                acc['supervisor_reason_bit_counts'][name] += 1

    kf_x = _safe_float(_field_value(vals, fields, 'kf_x'))
    kf_y = _safe_float(_field_value(vals, fields, 'kf_y'))
    prev_kf = prev.get('kf')
    if kf_x is not None and kf_y is not None and prev_kf is not None:
        step_m = math.hypot(kf_x - prev_kf[0], kf_y - prev_kf[1])
        acc['max_kf_step_m'] = step_m if acc['max_kf_step_m'] is None else max(acc['max_kf_step_m'], step_m)
        if step_m > kf_threshold_m:
            acc['kf_step_over_threshold_count'] += 1
            _top_event_insert(
                acc['top_kf_step_events'],
                {
                    'seq': seq,
                    't_us': ts_us,
                    'kf_step_m': _round_or_none(step_m, 3),
                    'kf_x_m': _round_or_none(kf_x, 3),
                    'kf_y_m': _round_or_none(kf_y, 3),
                },
                key='kf_step_m',
            )
    if kf_x is not None and kf_y is not None:
        prev['kf'] = (kf_x, kf_y)


def _finalize_file_report(report, acc, kf_threshold_m):
    data_records = acc['data_records']
    duration_s = None
    rate_hz = None
    if acc['first_t_us'] is not None and acc['last_t_us'] is not None:
        duration_s = (acc['last_t_us'] - acc['first_t_us']) / 1_000_000.0
        if duration_s > 0 and data_records > 1:
            rate_hz = (data_records - 1) / duration_s

    report['records'].update({
        'data_records': data_records,
        'first_seq': acc['first_seq'],
        'last_seq': acc['last_seq'],
        'first_t_us': acc['first_t_us'],
        'last_t_us': acc['last_t_us'],
        'duration_s': _round_or_none(duration_s, 6),
        'rate_hz': _round_or_none(rate_hz, 6),
        'min_dt_ms': _round_or_none(acc['min_dt_ms'], 3),
        'max_dt_ms': _round_or_none(acc['max_dt_ms'], 3),
        'seq_gap_count': acc['seq_gap_count'],
        'seq_missing_total': acc['seq_missing_total'],
        'max_seq_gap_abs': acc['max_seq_gap_abs'],
        'timestamp_backsteps': acc['timestamp_backsteps'],
        'sentinel_counts': acc['sentinel_counts'],
        'invalid_integrity_records': acc['invalid_integrity_records'],
    })

    report['sd'] = {
        'max_sd_queue_hwm': acc['max_sd_queue_hwm'],
        'max_sd_records_dropped': acc['max_sd_records_dropped'],
        'max_sd_partial_write_count': acc['max_sd_partial_write_count'],
        'max_sd_stall_count': acc['max_sd_stall_count'],
        'max_sd_reopen_count': acc['max_sd_reopen_count'],
    }

    report['gps'] = {
        'gps_valid_count': acc['gps_valid_count'],
        'gps_fresh_count': acc['gps_fresh_count'],
        'gps_valid_ratio': _round_or_none(_ratio(acc['gps_valid_count'], data_records), 6),
        'gps_fresh_ratio': _round_or_none(_ratio(acc['gps_fresh_count'], data_records), 6),
        'max_gps_fix_step_speed_kmh': _round_or_none(acc['max_gps_fix_step_speed_kmh'], 3),
        'max_gps_eskf_innov_m': _round_or_none(acc['max_gps_eskf_innov_m'], 3),
        'max_gps_nmea_nav2_dist_m': _round_or_none(acc['max_gps_nmea_nav2_dist_m'], 3),
    }

    report['gps_supervisor'] = {
        'state_counts': dict(acc['supervisor_state_counts']),
        'reason_bit_counts': dict(acc['supervisor_reason_bit_counts']),
        'top_reasons': [
            {'reason': reason, 'count': count}
            for reason, count in acc['supervisor_reason_bit_counts'].most_common(10)
        ],
    }

    report['eskf'] = {
        'kf_step_threshold_m': kf_threshold_m,
        'max_kf_step_m': _round_or_none(acc['max_kf_step_m'], 3),
        'kf_step_over_threshold_count': acc['kf_step_over_threshold_count'],
        'top_kf_step_events': acc['top_kf_step_events'],
    }

    issues = report['issues']
    hdr = report.get('header') or {}
    if hdr.get('header_crc_ok') is False:
        _add_issue(issues, 'FAIL', 'HEADER_CRC_BAD', 'Header CRC check failed.')
    if data_records == 0:
        _add_issue(issues, 'FAIL', 'NO_VALID_RECORDS', 'No data records were decoded.')
    if acc['invalid_integrity_records'] > 0:
        _add_issue(issues, 'FAIL', 'RECORD_INTEGRITY_BAD', 'One or more records failed magic/CRC checks.')
    if acc['seq_gap_count'] > 0:
        _add_issue(issues, 'FAIL', 'SEQ_GAP', 'Sequence gaps were found inside the file.')
    if acc['timestamp_backsteps'] > 0:
        _add_issue(issues, 'FAIL', 'TIMESTAMP_BACKSTEP', 'Timestamp moved backwards inside the file.')
    if acc['max_sd_records_dropped'] > 0:
        _add_issue(issues, 'FAIL', 'SD_DROPS', 'SD queue drops were recorded.')
    if acc['max_sd_partial_write_count'] > 0:
        _add_issue(issues, 'FAIL', 'SD_PARTIAL_WRITE', 'Partial SD writes were recorded.')
    if acc['max_sd_stall_count'] > 0:
        _add_issue(issues, 'FAIL', 'SD_STALL', 'SD stalls were recorded.')

    if report['file']['tail_bytes'] > 0:
        _add_issue(issues, 'WARN', 'PREALLOC_TAIL', 'File contains an unwritten preallocated tail.')
    if acc['max_sd_reopen_count'] > 0:
        _add_issue(issues, 'WARN', 'SD_REOPEN', 'SD reopen recovery was used.')
    queue_depth = hdr.get('sd_queue_depth')
    if queue_depth and acc['max_sd_queue_hwm'] >= 0.8 * queue_depth:
        _add_issue(issues, 'WARN', 'SD_QUEUE_HIGH_WATERMARK', 'SD queue high-water mark exceeded 80% of depth.')
    fresh_ratio = report['gps']['gps_fresh_ratio']
    if fresh_ratio is not None and fresh_ratio < 0.95:
        _add_issue(issues, 'WARN', 'GPS_FRESH_LOW', 'GPS fresh ratio is below 95%.')
    state_counts = acc['supervisor_state_counts']
    if state_counts.get('QUARANTINE', 0) or state_counts.get('RECOVERING', 0):
        _add_issue(issues, 'WARN', 'GPS_SUPERVISOR_ACTIVE', 'GPS supervisor entered QUARANTINE or RECOVERING.')
    if acc['kf_step_over_threshold_count'] > 0:
        _add_issue(issues, 'WARN', 'KF_STEP_LARGE', 'ESKF position step exceeded threshold.')

    report['status'] = _status_from_issues(issues)


def build_bin_health_report(bin_path, layout=None, kf_threshold_m=REPORT_DEFAULT_KF_STEP_THRESHOLD_M):
    if layout is None:
        layout = bin_logical_layout(bin_path, quiet=True)
    hdr = layout['hdr']
    chunk_size = layout['chunk_size']
    fmt, hdr_cols, _ = get_format(chunk_size, hdr.get('header_version') if hdr else None)
    fields = _field_indices(hdr_cols)

    report = {
        'path': bin_path,
        'type': 'bin',
        'tel_index': _parse_tel_index(bin_path),
        'status': 'PASS',
        'issues': [],
        'file': {
            'name': os.path.basename(bin_path),
            'physical_size_bytes': os.path.getsize(bin_path),
            'logical_size_bytes': layout['logical_end'],
            'tail_bytes': layout['tail_bytes'],
            'offset_bytes': layout['offset'],
            'record_size_bytes': chunk_size,
            'layout_valid_records': layout['valid_records'],
        },
        'header': _header_report(hdr),
        'records': {},
        'sd': {},
        'gps': {},
        'gps_supervisor': {},
        'eskf': {},
    }

    acc = _new_metric_accumulators()
    prev = {}
    with open(bin_path, 'rb') as f:
        f.seek(layout['offset'])
        while f.tell() < layout['logical_end']:
            chunk = f.read(chunk_size)
            if len(chunk) < chunk_size:
                break
            vals = struct.unpack(fmt, chunk)
            # bin_logical_layout() already validates the contiguous record prefix
            # through magic/CRC for v6 preallocated logs. Recomputing CRC for
            # every row here makes health reports on 128 MiB logs unnecessarily
            # slow, so this pass only computes diagnostics over the valid range.
            _update_metric_accumulators(acc, vals, fields, kf_threshold_m, prev)

    _finalize_file_report(report, acc, kf_threshold_m)
    return report


def build_csv_health_report(csv_path, kf_threshold_m=REPORT_DEFAULT_KF_STEP_THRESHOLD_M):
    preamble, header_line = read_csv_preamble_and_header(csv_path)
    header_cols = next(csv.reader([header_line.strip()]))
    fields = _field_indices(header_cols)
    report = {
        'path': csv_path,
        'type': 'csv',
        'tel_index': _parse_tel_index(csv_path),
        'status': 'PASS',
        'issues': [{'level': 'WARN', 'code': 'CSV_REDUCED_REPORT',
                    'message': 'CSV report lacks binary header, CRC, reset and preallocation metadata.'}],
        'file': {
            'name': os.path.basename(csv_path),
            'physical_size_bytes': os.path.getsize(csv_path),
            'logical_size_bytes': os.path.getsize(csv_path),
            'tail_bytes': None,
            'offset_bytes': None,
            'record_size_bytes': None,
            'layout_valid_records': None,
        },
        'header': {
            'present': bool(preamble),
            'firmware_version': None,
            'header_version': None,
            'record_size': None,
            'reset_reason_cpu0': None,
            'log_filename': None,
            'header_crc_ok': None,
        },
        'records': {},
        'sd': {},
        'gps': {},
        'gps_supervisor': {},
        'eskf': {},
    }
    acc = _new_metric_accumulators()
    prev = {}
    with open_text_read(csv_path) as f:
        header_seen = False
        for raw_line in f:
            stripped = raw_line.rstrip('\n\r')
            if not stripped:
                continue
            if not header_seen:
                if stripped.startswith('#'):
                    continue
                header_seen = True
                continue
            if stripped.startswith('#'):
                if stripped.startswith('# CALIB_UPDATE'):
                    acc['sentinel_counts']['CALIB_UPDATE'] += 1
                elif stripped.startswith('# MOUNT_YAW_OBS'):
                    acc['sentinel_counts']['MOUNT_YAW_OBS'] += 1
                elif stripped.startswith('# MOUNT_YAW_BOOT'):
                    acc['sentinel_counts']['MOUNT_YAW_BOOT'] += 1
                prev.clear()
                continue
            row = next(csv.reader([stripped]))
            vals = []
            for item in row:
                num = _safe_float(item)
                vals.append(num if num is not None else item)
            _update_metric_accumulators(acc, vals, fields, kf_threshold_m, prev)

    _finalize_file_report(report, acc, kf_threshold_m)
    return report


def build_file_health_report(path, layout=None, kf_threshold_m=REPORT_DEFAULT_KF_STEP_THRESHOLD_M):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.bin':
        return build_bin_health_report(path, layout=layout, kf_threshold_m=kf_threshold_m)
    if ext == '.csv':
        return build_csv_health_report(path, kf_threshold_m=kf_threshold_m)
    raise ValueError(f"Unsupported report input: {path}")


def build_session_health_report(paths, mode='session', kf_threshold_m=REPORT_DEFAULT_KF_STEP_THRESHOLD_M):
    files = [build_file_health_report(path, kf_threshold_m=kf_threshold_m) for path in paths]
    transitions = []
    for prev_report, next_report in zip(files, files[1:]):
        prev_seq = prev_report['records'].get('last_seq')
        next_seq = next_report['records'].get('first_seq')
        prev_t = prev_report['records'].get('last_t_us')
        next_t = next_report['records'].get('first_t_us')
        seq_delta = (next_seq - prev_seq) if prev_seq is not None and next_seq is not None else None
        time_gap_s = ((next_t - prev_t) / 1_000_000.0) if prev_t is not None and next_t is not None else None
        new_boot = next_seq == 0 if next_seq is not None else False
        continuous = seq_delta == 1
        issues = []
        if not continuous and not new_boot:
            _add_issue(issues, 'WARN', 'SESSION_SEQ_DISCONTINUITY', 'Adjacent files are not seq-continuous.')
        transition = {
            'prev_file': prev_report['file']['name'],
            'next_file': next_report['file']['name'],
            'seq_delta': seq_delta,
            'time_gap_s': _round_or_none(time_gap_s, 6),
            'continuous': continuous,
            'new_boot': new_boot,
            'status': _status_from_issues(issues),
            'issues': issues,
        }
        transitions.append(transition)

    status = 'PASS'
    for file_report in files:
        status = _worst_status(status, file_report['status'])
    for transition in transitions:
        status = _worst_status(status, transition['status'])

    total_records = sum((f['records'].get('data_records') or 0) for f in files)
    total_duration_s = sum((f['records'].get('duration_s') or 0.0) for f in files)
    return {
        'schema_version': REPORT_SCHEMA_VERSION,
        'generated_at': datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z'),
        'mode': mode,
        'status': status,
        'files': files,
        'transitions': transitions,
        'summary': {
            'file_count': len(files),
            'total_data_records': total_records,
            'total_duration_s': _round_or_none(total_duration_s, 6),
            'statuses': dict(Counter(f['status'] for f in files)),
            'transition_statuses': dict(Counter(t['status'] for t in transitions)),
        },
    }


def _md_table(headers, rows):
    out = []
    out.append('| ' + ' | '.join(headers) + ' |')
    out.append('| ' + ' | '.join('---' for _ in headers) + ' |')
    for row in rows:
        out.append('| ' + ' | '.join(str(item) for item in row) + ' |')
    return '\n'.join(out)


def render_health_report_markdown(report):
    lines = [
        f"# Telemetry Diagnostic Report",
        "",
        f"- Status: **{report['status']}**",
        f"- Mode: `{report['mode']}`",
        f"- Generated: `{report['generated_at']}`",
        "",
        "## Files",
    ]
    rows = []
    for f in report['files']:
        rows.append([
            f['file']['name'],
            f['status'],
            f['header'].get('firmware_version') or '',
            f['header'].get('reset_reason_cpu0') if f['header'].get('reset_reason_cpu0') is not None else '',
            f['records'].get('data_records') or 0,
            f['file'].get('tail_bytes') if f['file'].get('tail_bytes') is not None else '',
            f['records'].get('duration_s') or '',
            f['records'].get('rate_hz') or '',
            f['sd'].get('max_sd_queue_hwm') if f.get('sd') else '',
            f['gps'].get('gps_fresh_ratio') if f.get('gps') else '',
        ])
    lines.append(_md_table(
        ['file', 'status', 'firmware', 'reset', 'records', 'tail_bytes', 'duration_s', 'rate_hz', 'sd_hwm', 'gps_fresh'],
        rows,
    ))

    if report['transitions']:
        lines.extend(["", "## Transitions"])
        rows = []
        for t in report['transitions']:
            rows.append([t['prev_file'], t['next_file'], t['status'], t['seq_delta'], t['time_gap_s'], t['continuous'], t['new_boot']])
        lines.append(_md_table(['prev', 'next', 'status', 'seq_delta', 'time_gap_s', 'continuous', 'new_boot'], rows))

    all_issues = []
    for f in report['files']:
        for issue in f['issues']:
            all_issues.append([f['file']['name'], issue['level'], issue['code'], issue['message']])
    for t in report['transitions']:
        for issue in t['issues']:
            all_issues.append([f"{t['prev_file']} -> {t['next_file']}", issue['level'], issue['code'], issue['message']])
    if all_issues:
        lines.extend(["", "## Warnings And Failures", _md_table(['scope', 'level', 'code', 'message'], all_issues)])

    top_kf = []
    for f in report['files']:
        for event in f.get('eskf', {}).get('top_kf_step_events', []):
            top_kf.append((event.get('kf_step_m') or 0, f['file']['name'], event))
    top_kf.sort(key=lambda item: item[0], reverse=True)
    if top_kf:
        lines.extend(["", "## Top Kf Steps"])
        rows = []
        for _, file_name, event in top_kf[:10]:
            rows.append([file_name, event.get('seq'), event.get('t_us'), event.get('kf_step_m'), event.get('kf_x_m'), event.get('kf_y_m')])
        lines.append(_md_table(['file', 'seq', 't_us', 'kf_step_m', 'kf_x_m', 'kf_y_m'], rows))

    reason_rows = []
    reason_totals = Counter()
    for f in report['files']:
        reason_totals.update(f.get('gps_supervisor', {}).get('reason_bit_counts', {}))
    for reason, count in reason_totals.most_common(10):
        reason_rows.append([reason, count])
    if reason_rows:
        lines.extend(["", "## Top GPS Supervisor Reasons", _md_table(['reason', 'count'], reason_rows)])

    return '\n'.join(lines) + '\n'


def write_health_report_outputs(report, output_base, report_format='both'):
    written = []
    for path in _report_paths(output_base, report_format):
        if path.endswith('.json'):
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
                f.write('\n')
        elif path.endswith('.md'):
            with open(path, 'w', encoding='utf-8', newline='\n') as f:
                f.write(render_health_report_markdown(report))
        written.append(path)
    return written


def generate_health_report_for_targets(targets, output_base=None, report_format='both',
                                       mode='session',
                                       kf_threshold_m=REPORT_DEFAULT_KF_STEP_THRESHOLD_M):
    paths = _resolve_report_targets(targets)
    if not paths:
        raise ValueError('No .bin or .csv files found for report.')
    report = build_session_health_report(paths, mode=mode, kf_threshold_m=kf_threshold_m)
    if output_base is None:
        output_base = _report_output_base_from_targets(paths)
    written = write_health_report_outputs(report, output_base, report_format)
    print(f"  {GREEN}Report:{RESET} status={report['status']} files={len(paths)}")
    for path in written:
        print(f"    {GREEN}✓{RESET} {path}")
    return report, written


def parse_cli_options(raw_args):
    opts = {
        'repair_bin': False,
        'report': None,
        'report_format': 'both',
        'report_session': None,
        'kf_step_threshold': REPORT_DEFAULT_KF_STEP_THRESHOLD_M,
        'positionals': [],
    }
    i = 0
    while i < len(raw_args):
        arg = raw_args[i]
        if arg == '--repair-bin':
            opts['repair_bin'] = True
            i += 1
        elif arg == '--report':
            opts['report'] = True
            i += 1
        elif arg == '--no-report':
            opts['report'] = False
            i += 1
        elif arg == '--report-format':
            if i + 1 >= len(raw_args):
                raise ValueError('--report-format requires both, json, or md')
            value = raw_args[i + 1].lower()
            if value not in ('both', 'json', 'md'):
                raise ValueError('--report-format must be both, json, or md')
            opts['report_format'] = value
            i += 2
        elif arg == '--kf-step-threshold':
            if i + 1 >= len(raw_args):
                raise ValueError('--kf-step-threshold requires a numeric value')
            opts['kf_step_threshold'] = float(raw_args[i + 1])
            i += 2
        elif arg == '--report-session':
            targets = []
            i += 1
            while i < len(raw_args) and not raw_args[i].startswith('--'):
                targets.append(raw_args[i])
                i += 1
            if not targets:
                raise ValueError('--report-session requires at least one directory or file')
            opts['report_session'] = targets
        elif arg.startswith('--'):
            raise ValueError(f'Unknown option: {arg}')
        else:
            opts['positionals'].append(arg)
            i += 1
    return opts


# ── Split methods ─────────────────────────────────────────────────────────────

def write_split_files(lines_gen, base_name, split_fn, description):
    """
    Write rows into split output files.
    split_fn(part, line_count, file_size) → bool: True to open a new file.
    """
    t0 = time.perf_counter()
    part = 1
    line_count_total = 0
    line_count_part = 0
    file_size_part = 0
    preamble_text = ''.join(line + '\n' for line in CSV_PREAMBLE_LINES)
    header_bytes = len((preamble_text + HEADER_LINE).encode('utf-8-sig'))

    out_path = f"{base_name}_part{part}.csv"
    fout = open(out_path, 'w', encoding=TEXT_ENCODING_WRITE, newline='\n')
    write_csv_preamble(fout)
    file_size_part = header_bytes
    files_written = [out_path]

    for line in lines_gen:
        line_data = line + '\n'
        line_bytes = len(line_data.encode('utf-8'))

        # Check if a new file is needed BEFORE writing
        if split_fn(part, line_count_part, file_size_part + line_bytes):
            fout.close()
            size_mb = file_size_part / (1024 * 1024)
            print(f"    {GREEN}✓{RESET} {out_path}: {line_count_part:,} rows, {size_mb:.1f} MB")

            part += 1
            line_count_part = 0
            out_path = f"{base_name}_part{part}.csv"
            fout = open(out_path, 'w', encoding=TEXT_ENCODING_WRITE, newline='\n')
            write_csv_preamble(fout)
            file_size_part = header_bytes
            files_written.append(out_path)

        fout.write(line_data)
        file_size_part += line_bytes
        if not is_csv_comment_line(line):
            line_count_part += 1
            line_count_total += 1

    fout.close()
    size_mb = file_size_part / (1024 * 1024)
    print(f"    {GREEN}✓{RESET} {out_path}: {line_count_part:,} rows, {size_mb:.1f} MB")

    elapsed = time.perf_counter() - t0
    print(f"\n  {BOLD}Done:{RESET} {line_count_total:,} rows → "
          f"{part} files in {elapsed:.1f}s")
    return files_written


def split_by_max_size(lines_gen, base_name, max_mb):
    max_bytes = int(max_mb * 1024 * 1024)
    return write_split_files(
        lines_gen, base_name,
        lambda part, lc, fs: lc > 0 and fs > max_bytes,
        f"max {max_mb} MB per file"
    )


def split_by_n_parts(lines_gen, base_name, n_parts, total_lines):
    lines_per_part = math.ceil(total_lines / n_parts)
    return write_split_files(
        lines_gen, base_name,
        lambda part, lc, fs: part < n_parts and lc >= lines_per_part,
        f"{n_parts} equal parts"
    )


def split_by_rows(lines_gen, base_name, rows_per_part):
    return write_split_files(
        lines_gen, base_name,
        lambda part, lc, fs: lc >= rows_per_part,
        f"{rows_per_part:,} rows per file"
    )


def no_split(lines_gen, out_path):
    """Write everything into a single CSV file."""
    t0 = time.perf_counter()
    count = 0
    with open(out_path, 'w', encoding=TEXT_ENCODING_WRITE, newline='\n') as f:
        write_csv_preamble(f)
        for line in lines_gen:
            f.write(line + '\n')
            if not is_csv_comment_line(line):
                count += 1
    elapsed = time.perf_counter() - t0
    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"\n  {GREEN}✓{RESET} {out_path}: {count:,} rows, {size_mb:.1f} MB ({elapsed:.1f}s)")


# ── Split menu ────────────────────────────────────────────────────────────────

def split_menu(lines_gen_factory, base_name, total_lines, est_csv_mb):
    """
    Interactive menu to choose the split method.
    lines_gen_factory: callable that returns a fresh row generator.
    """
    print(f"\n  {BOLD}Dataset:{RESET} ~{total_lines:,} rows, ~{est_csv_mb:.1f} MB estimated CSV size")
    print()
    print(f"  {BOLD}Split method:{RESET}")
    print(f"    {GREEN}1.{RESET} Maximum file size (MB)")
    print(f"    {GREEN}2.{RESET} Number of equal parts")
    print(f"    {GREEN}3.{RESET} Number of rows per file")
    print(f"    {GREEN}4.{RESET} No split (single file)")
    print()

    choice = ask_int("Choice", 1, minimum=1)

    if choice == 1:
        max_mb = ask_float("Maximum size per part (MB)", 30, minimum=0.5)
        n_parts_est = math.ceil(est_csv_mb / max_mb)
        print(f"\n  Estimate: {YELLOW}~{n_parts_est} files{RESET} of ~{max_mb:.0f} MB each\n")
        split_by_max_size(lines_gen_factory(), base_name, max_mb)

    elif choice == 2:
        default_n = max(2, math.ceil(est_csv_mb / 30))
        n = ask_int("Number of parts", default_n, minimum=2)
        rows_each = math.ceil(total_lines / n)
        mb_each = est_csv_mb / n
        print(f"\n  → {YELLOW}{rows_each:,} rows/part{RESET}, ~{mb_each:.1f} MB each\n")
        split_by_n_parts(lines_gen_factory(), base_name, n, total_lines)

    elif choice == 3:
        # Estimate rows for ~30 MB
        bytes_per_row = (est_csv_mb * 1024 * 1024) / total_lines if total_lines else 150
        default_rows = int(30 * 1024 * 1024 / bytes_per_row)
        rows = ask_int("Rows per file", default_rows, minimum=100)
        n_parts_est = math.ceil(total_lines / rows)
        print(f"\n  Estimate: {YELLOW}~{n_parts_est} files{RESET} "
              f"of {rows:,} rows each\n")
        split_by_rows(lines_gen_factory(), base_name, rows)

    elif choice == 4:
        out = f"{base_name}.csv"
        print(f"\n  Output: {out}\n")
        no_split(lines_gen_factory(), out)

    else:
        print(f"  {RED}Invalid choice.{RESET}")
        sys.exit(1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global HEADER_LINE, CSV_PREAMBLE_LINES
    banner()

    try:
        cli = parse_cli_options(sys.argv[1:])
    except ValueError as exc:
        print(f"  {RED}ERROR:{RESET} {exc}")
        sys.exit(1)

    if cli['report_session'] is not None:
        try:
            generate_health_report_for_targets(
                cli['report_session'],
                report_format=cli['report_format'],
                mode='session',
                kf_threshold_m=cli['kf_step_threshold'],
            )
        except Exception as exc:
            print(f"  {RED}Report failed:{RESET} {exc}")
            sys.exit(1)
        return

    args = cli['positionals']
    repair_bin = cli['repair_bin']

    # Determine input
    if len(args) >= 2:
        # Direct mode: bin_to_csv.py input output
        bin_file = args[0]
        csv_file = args[1]
        if not os.path.isfile(bin_file):
            print(f"  {RED}File not found: {bin_file}{RESET}")
            sys.exit(1)
        if repair_bin and os.path.splitext(bin_file)[1].lower() == '.bin':
            repair_bin_preallocated_tail(bin_file)
        hdr, _ = read_file_header(bin_file)
        if hdr:
            print(f"  Firmware: {hdr['firmware_version']}, record_size={hdr['record_size']}B")
            _, hdr_cols, _ = get_format(hdr['record_size'], hdr.get('header_version'))
            HEADER_LINE = ','.join(hdr_cols) + '\n'
            CSV_PREAMBLE_LINES = build_csv_preamble_lines(hdr)
        print(f"  Direct conversion: {bin_file} → {csv_file}")
        layout = bin_logical_layout(bin_file)
        total = bin_record_count(bin_file, layout)
        no_split(bin_to_csv_lines(bin_file, total, layout), csv_file)
        if cli['report'] is True:
            try:
                generate_health_report_for_targets(
                    [bin_file],
                    output_base=os.path.splitext(csv_file)[0],
                    report_format=cli['report_format'],
                    mode='direct',
                    kf_threshold_m=cli['kf_step_threshold'],
                )
            except Exception as exc:
                print(f"  {RED}Report failed:{RESET} {exc}")
                sys.exit(1)
        return

    if len(args) == 1:
        input_file = args[0]
        if not os.path.isfile(input_file):
            print(f"  {RED}File not found: {input_file}{RESET}")
            sys.exit(1)
    else:
        input_file = find_input_file()

    ext = os.path.splitext(input_file)[1].lower()
    base = os.path.splitext(input_file)[0]

    if ext == '.bin':
        if repair_bin:
            repair_bin_preallocated_tail(input_file)
        # Detect FileHeader (v0.9.7+)
        hdr, hdr_offset = read_file_header(input_file)
        if hdr:
            print(f"  {GREEN}FileHeader detected:{RESET} firmware {BOLD}{hdr['firmware_version']}{RESET}, "
                  f"record_size={hdr['record_size']}B, "
                  f"header_v{hdr['header_version']}, "
                  f"start={hdr['start_time_ms']}ms")
            # Update HEADER_LINE for the detected record format
            _, hdr_cols, _ = get_format(hdr['record_size'], hdr.get('header_version'))
            HEADER_LINE = ','.join(hdr_cols) + '\n'
            CSV_PREAMBLE_LINES = build_csv_preamble_lines(hdr)
        else:
            print(f"  {DIM}Legacy file (no header) — record_size={CHUNK_SIZE_DEFAULT}B{RESET}")
            CSV_PREAMBLE_LINES = []

        layout = bin_logical_layout(input_file)
        total_lines = bin_record_count(input_file, layout)
        # Estimate CSV size: updated dynamically from the first record
        bytes_per_row_est = len(HEADER_LINE) * 0 + 150  # base heuristic
        est_csv_mb = (total_lines * bytes_per_row_est) / (1024 * 1024)

        # Sample one row for a better estimate
        for sample_line in bin_to_csv_lines(input_file, None, layout):
            bytes_per_row_est = len(sample_line.encode('utf-8')) + 1
            est_csv_mb = (total_lines * bytes_per_row_est) / (1024 * 1024)
            break

        print(f"  {BOLD}Input:{RESET} {input_file}")
        print(f"  {DIM}{os.path.getsize(input_file) / (1024*1024):.1f} MB binary, "
              f"{total_lines:,} records, ~{est_csv_mb:.1f} MB as CSV{RESET}")

        generate_report = False
        if cli['report'] is True:
            generate_report = True
        elif cli['report'] is None:
            confirm = ask("Generate diagnostic report? (y/n)", "y")
            generate_report = confirm.lower() in ('y', 'yes')
        if generate_report:
            try:
                report = build_session_health_report(
                    [input_file],
                    mode='file',
                    kf_threshold_m=cli['kf_step_threshold'],
                )
                written = write_health_report_outputs(report, base, cli['report_format'])
                print(f"  {GREEN}Report:{RESET} status={report['status']}")
                for path in written:
                    print(f"    {GREEN}✓{RESET} {path}")
            except Exception as exc:
                print(f"  {RED}Report failed:{RESET} {exc}")

        def gen_factory():
            return bin_to_csv_lines(input_file, total_lines, layout)

        split_menu(gen_factory, base, total_lines, est_csv_mb)

    elif ext == '.csv':
        file_mb = os.path.getsize(input_file) / (1024 * 1024)
        print(f"  {BOLD}Input:{RESET} {input_file} ({file_mb:.1f} MB)")
        CSV_PREAMBLE_LINES, HEADER_LINE = read_csv_preamble_and_header(input_file)
        print(f"  {DIM}Counting rows...{RESET}", end=' ', flush=True)
        total_lines = count_csv_lines(input_file)
        print(f"{total_lines:,} data rows")

        if cli['report'] is True:
            try:
                generate_health_report_for_targets(
                    [input_file],
                    output_base=base,
                    report_format=cli['report_format'],
                    mode='file',
                    kf_threshold_m=cli['kf_step_threshold'],
                )
            except Exception as exc:
                print(f"  {RED}Report failed:{RESET} {exc}")
                sys.exit(1)

        if file_mb < 31:
            print(f"\n  {GREEN}File is already under 31 MB — split not needed.{RESET}")
            confirm = ask("Split anyway? (y/n)", "n")
            if confirm.lower() not in ('y', 'yes'):
                return

        def gen_factory():
            return csv_to_lines(input_file)

        split_menu(gen_factory, base, total_lines, file_mb)

    else:
        print(f"  {RED}Unsupported format: {ext}{RESET}")
        print(f"  Use a .bin (binary telemetry) or .csv file")
        sys.exit(1)


if __name__ == '__main__':
    main()
