"""
bin_to_csv.py — Binary telemetry → CSV converter with interactive split

Usage:
  python bin_to_csv.py                       # interactive menu
  python bin_to_csv.py tel_23.bin           # convert → split menu
  python bin_to_csv.py tel_23.csv           # split existing CSV
  python bin_to_csv.py tel_23.bin out.csv   # convert without split
"""

import struct
import sys
import os
import time
import math

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
PROGRESS_EVERY = 5000
TEXT_ENCODING_WRITE = 'utf-8-sig'
TEXT_ENCODINGS_READ = ('utf-8-sig', 'utf-8', 'latin1')
RESYNC_SCAN_BYTES = 4096
RESYNC_CONFIRM_RECORDS = 8
MAX_TIMESTAMP_US = 7 * 24 * 60 * 60 * 1_000_000
MAX_TIMESTAMP_GAP_US = 10_000_000

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

# 190-byte record (v1.5.0+): adds NAV-PV velocity fields from CASIC binary
# nav_speed2d: CASIC NAV-PV ground speed [m/s]
# nav_s_acc:   speed accuracy estimate [m/s]
# nav_vel_n/e: ENU velocity components [m/s]
# nav_vel_valid: 0=invalid, 6=2D, 7=3D
# gps_speed_source: 0=NMEA_SOG, 1=NAV_PV (per-sample source flag)
# nav_fix_us:  esp_timer when NAV-PV was parsed [µs] (freshness/SITL)
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

# Default aliases (used for legacy files without header)
FMT_DEFAULT = FMT_122
HEADER = HEADER_122
COL_FMT = COL_FMT_122

def get_format(record_size, header_version=None):
    """Return (struct_fmt, header_list, col_fmt_list) for a given record size."""
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
    """Search for .bin and .csv files in the current directory."""
    bins = sorted([f for f in os.listdir('.') if f.endswith('.bin')])
    csvs = sorted([f for f in os.listdir('.') if f.endswith('.csv')
                   and os.path.getsize(f) > 1_000_000])  # only CSV >1MB

    if not bins and not csvs:
        print(f"  {RED}No .bin or .csv files found in the current directory.{RESET}")
        path = ask("File path")
        if not os.path.isfile(path):
            print(f"  {RED}File not found: {path}{RESET}")
            sys.exit(1)
        return path

    print(f"  {BOLD}Files found:{RESET}")
    options = []
    for f in bins:
        size_mb = os.path.getsize(f) / (1024 * 1024)
        n_records = bin_record_count(f)
        options.append(f)
        print(f"    {GREEN}{len(options)}.{RESET} {f}  "
              f"{DIM}({size_mb:.1f} MB, ~{n_records:,} records, binary){RESET}")
    for f in csvs:
        size_mb = os.path.getsize(f) / (1024 * 1024)
        options.append(f)
        print(f"    {GREEN}{len(options)}.{RESET} {f}  "
              f"{DIM}({size_mb:.1f} MB, CSV){RESET}")
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
    if hdr and hdr.get('record_size', 0) in (242, 256):
        lines.append(
            "# COLUMN_NOTE: v5 AtomS3R schema. pipe_lin_* / pipe_body_* are zero-latency "
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


def _record_plausible(vals, fields, prev_ts_us=None, raw=None):
    """Heuristic guard used only to recover from byte-level SD log misalignment."""
    value = _field_value(vals, fields, 'record_magic')
    if value is not None and value != RECORD_MAGIC_256_V6:
        return False

    value = _field_value(vals, fields, 'crc16')
    if value is not None and raw is not None:
        if crc16_ccitt(raw[:-2]) != value:
            return False

    ts_us = _timestamp_us(vals, fields)
    if ts_us == SENTINEL_CALIB:
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

    range_checks = (
        ('gps_sog_kmh', 0.0, 500.0),
        ('gps_alt_m', -1000.0, 10000.0),
        ('gps_sats', 0, 80),
        ('gps_hdop', 0.0, 100.0),
        ('lap', 0, 255),
        ('fifo_frames_drained', 0, 64),
        ('fifo_backlog', 0, 255),
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
    if value is not None and value not in (0, 6, 7):
        return False

    value = _field_value(vals, fields, 'gps_speed_source')
    if value is not None and value not in (0, 1, 2):
        return False

    return True


def _find_resync_skip(fobj, start_pos, fmt, chunk_size, fields):
    window_len = RESYNC_SCAN_BYTES + RESYNC_CONFIRM_RECORDS * chunk_size
    fobj.seek(start_pos)
    window = fobj.read(window_len)
    max_skip = min(RESYNC_SCAN_BYTES, len(window) - RESYNC_CONFIRM_RECORDS * chunk_size)
    if max_skip < 1:
        return None

    for skip in range(1, max_skip + 1):
        prev_ts_us = None
        ok = True
        for i in range(RESYNC_CONFIRM_RECORDS):
            off = skip + i * chunk_size
            vals = struct.unpack_from(fmt, window, off)
            raw = window[off:off + chunk_size]
            if not _record_plausible(vals, fields, prev_ts_us, raw):
                ok = False
                break
            ts_us = _timestamp_us(vals, fields)
            if ts_us is not None and ts_us != SENTINEL_CALIB:
                prev_ts_us = ts_us
        if ok:
            return skip
    return None


def bin_record_count(bin_path):
    hdr, offset = read_file_header(bin_path)
    chunk_size = hdr['record_size'] if hdr else CHUNK_SIZE_DEFAULT
    data_bytes = os.path.getsize(bin_path) - offset
    return data_bytes // chunk_size


def bin_to_csv_lines(bin_path, total=None):
    """Yield CSV rows from a binary telemetry file (with or without FileHeader)."""
    hdr, offset = read_file_header(bin_path)
    chunk_size = hdr['record_size'] if hdr else CHUNK_SIZE_DEFAULT
    fmt, hdr_cols, col_fmt = get_format(chunk_size, hdr.get('header_version') if hdr else None)
    fields = _field_indices(hdr_cols)

    with open(bin_path, 'rb') as f:
        if offset > 0:
            f.seek(offset)
        count = 0
        resync_count = 0
        prev_ts_us = None
        while True:
            record_pos = f.tell()
            chunk = f.read(chunk_size)
            if len(chunk) < chunk_size:
                break
            vals = struct.unpack(fmt, chunk)
            if vals[0] == SENTINEL_CALIB:   # CalibrationRecord sentinel (uint64 max)
                yield f"# CALIB_UPDATE: sin_phi={vals[1]:.5f} cos_phi={vals[2]:.5f} sin_theta={vals[3]:.5f} cos_theta={vals[4]:.5f} bias_ax={vals[5]:.5f} bias_ay={vals[6]:.5f} bias_az={vals[7]:.5f} bias_gx={vals[11]:.5f} bias_gy={vals[12]:.5f} bias_gz={vals[14]:.5f}"
                prev_ts_us = None
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
                continue
            line = ','.join(f.format(v) for f, v in zip(col_fmt, vals))
            yield line
            prev_ts_us = _timestamp_us(vals, fields)
            count += 1
            if count % PROGRESS_EVERY == 0 and total:
                pct = count * 100 // total
                print(f"    Read {count:>10,} / {total:,}  ({pct}%)", end='\r')
        if resync_count:
            print(f"\n    {YELLOW}Resync events:{RESET} {resync_count}")
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

    # Determine input
    if len(sys.argv) >= 3:
        # Direct mode: bin_to_csv.py input output
        bin_file = sys.argv[1]
        csv_file = sys.argv[2]
        if not os.path.isfile(bin_file):
            print(f"  {RED}File not found: {bin_file}{RESET}")
            sys.exit(1)
        hdr, _ = read_file_header(bin_file)
        if hdr:
            print(f"  Firmware: {hdr['firmware_version']}, record_size={hdr['record_size']}B")
            _, hdr_cols, _ = get_format(hdr['record_size'], hdr.get('header_version'))
            HEADER_LINE = ','.join(hdr_cols) + '\n'
            CSV_PREAMBLE_LINES = build_csv_preamble_lines(hdr)
        print(f"  Direct conversion: {bin_file} → {csv_file}")
        total = bin_record_count(bin_file)
        no_split(bin_to_csv_lines(bin_file, total), csv_file)
        return

    if len(sys.argv) == 2:
        input_file = sys.argv[1]
        if not os.path.isfile(input_file):
            print(f"  {RED}File not found: {input_file}{RESET}")
            sys.exit(1)
    else:
        input_file = find_input_file()

    ext = os.path.splitext(input_file)[1].lower()
    base = os.path.splitext(input_file)[0]

    if ext == '.bin':
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

        total_lines = bin_record_count(input_file)
        # Estimate CSV size: updated dynamically from the first record
        bytes_per_row_est = len(HEADER_LINE) * 0 + 150  # base heuristic

        # Sample one row for a better estimate
        for sample_line in bin_to_csv_lines(input_file, None):
            bytes_per_row_est = len(sample_line.encode('utf-8')) + 1
            est_csv_mb = (total_lines * bytes_per_row_est) / (1024 * 1024)
            break

        print(f"  {BOLD}Input:{RESET} {input_file}")
        print(f"  {DIM}{os.path.getsize(input_file) / (1024*1024):.1f} MB binary, "
              f"{total_lines:,} records, ~{est_csv_mb:.1f} MB as CSV{RESET}")

        def gen_factory():
            return bin_to_csv_lines(input_file, total_lines)

        split_menu(gen_factory, base, total_lines, est_csv_mb)

    elif ext == '.csv':
        file_mb = os.path.getsize(input_file) / (1024 * 1024)
        print(f"  {BOLD}Input:{RESET} {input_file} ({file_mb:.1f} MB)")
        CSV_PREAMBLE_LINES, HEADER_LINE = read_csv_preamble_and_header(input_file)
        print(f"  {DIM}Counting rows...{RESET}", end=' ', flush=True)
        total_lines = count_csv_lines(input_file)
        print(f"{total_lines:,} data rows")

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
