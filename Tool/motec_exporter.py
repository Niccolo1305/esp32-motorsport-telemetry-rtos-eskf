"""
motec_exporter.py — Converts ESP32 telemetry (.bin or .csv) to native MoTeC i2 Pro .ld

Usage:
  python motec_exporter.py tel_30.bin                     # → tel_30.ld
  python motec_exporter.py tel_30.csv                     # → tel_30.ld
  python motec_exporter.py tel_30.bin --venue "Mugello"   # with circuit name
  python motec_exporter.py tel_30.bin -o custom.ld        # custom output path

Inputs:
  .bin  — raw SD card recording (with or without v0.9.7+ FileHeader)
  .csv  — output of bin_to_csv.py

Output:
  .ld   — native MoTeC Log Data file, openable directly in i2 Pro

Channels exported:
  @ 50 Hz: Ground Speed (ESKF), G Force Long/Lat/Vert, Yaw/Roll/Pitch Rate, Sensor Temp
  @ 10 Hz: GPS Latitude, GPS Longitude, GPS Altitude, GPS Speed

Binary format source: reverse-engineered from github.com/gotzl/ldparser
"""

import struct
import sys
import os
import argparse
import math
from datetime import datetime

# ── Telemetry binary format ───────────────────────────────────────────────────
# Matches struct TelemetryRecord in Telemetria.ino (__attribute__((packed)))
# Supported sizes: 127 B (v1.3.1+), 122 B (v0.9.8–v1.3.0), 78 B (v0.8.0–v0.9.7)
#
#   Field order and types (little-endian):
#   t_ms            uint32    4 B   — IMU hardware timestamp (ms)
#   ax..gz          7×float   28 B  — EMA-filtered accel (G) + gyro (deg/s) + temp (°C)
#   lap             uint8     1 B   — lap counter
#   gps_lat/lon     2×double  16 B  — GPS coordinates (WGS84 degrees)
#   gps_speed_kmh   float     4 B   — GPS ground speed (km/h)
#   gps_alt_m       float     4 B   — GPS altitude (m)
#   gps_sats        uint8     1 B   — visible satellites
#   gps_hdop        float     4 B   — HDOP dilution
#   kf_x..heading   4×float   16 B  — ESKF2D output
#   raw_ax..raw_gz  6×float   24 B  — post-Madgwick linear accel + gyro (zero latency)
#   kf6_x..kf6_bgz 5×float   20 B  — ESKF_6D shadow output + gyro bias
#                            ────
#                            122 B  total

HEADER_MAGIC  = b'TEL'
RECORD_FMT_215 = '<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ3fB'  # 215-byte record (v1.6.0+, + magnetometer)
RECORD_FMT_202 = '<Q7fBddffBf4f6f5fBf6fQB4fBBQfQ'  # 202-byte record (v1.5.1+, + DHV speed/timestamp)
RECORD_FMT_190 = '<Q7fBddffBf4f6f5fBf6fQB4fBBQ'  # 190-byte record (v1.5.0+, + NAV-PV velocity)
RECORD_FMT_164 = '<Q7fBddffBf4f6f5fBf6fQB'  # 164-byte record (v1.4.2+, + GPS timing metadata)
RECORD_FMT_155 = '<Q7fBddffBf4f6f5fBf6f'  # 155-byte record (v1.4.0, uint64 ts + sensor raw)
RECORD_FMT_127 = '<I7fBddffBf4f6f5fBf'  # 127-byte record (v1.3.1+, + zaru_flags + tbias_gz)
RECORD_FMT_122 = '<I7fBddffBf4f6f5f'  # 122-byte record (v0.9.8+, raw IMU + 6D)
RECORD_FMT_78  = '<I7fBddffBf4f'       # 78-byte record (v0.8.0–v0.9.7, EMA only)
RECORD_SIZE_215 = struct.calcsize(RECORD_FMT_215)
RECORD_SIZE_202 = struct.calcsize(RECORD_FMT_202)
RECORD_SIZE_190 = struct.calcsize(RECORD_FMT_190)
RECORD_SIZE_164 = struct.calcsize(RECORD_FMT_164)
RECORD_SIZE_155 = struct.calcsize(RECORD_FMT_155)
RECORD_SIZE_127 = struct.calcsize(RECORD_FMT_127)
RECORD_SIZE_122 = struct.calcsize(RECORD_FMT_122)
RECORD_SIZE_78  = struct.calcsize(RECORD_FMT_78)
SENTINEL_CALIB = 0xFFFFFFFFFFFFFFFF  # uint64 max — CalibrationRecord marker

FIELD_NAMES_202 = [
    't_us',
    'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c',
    'lap',
    'gps_lat', 'gps_lon', 'gps_sog_kmh', 'gps_alt_m',
    'gps_sats', 'gps_hdop',
    'kf_x', 'kf_y', 'kf_vel', 'kf_heading',
    'raw_ax', 'raw_ay', 'raw_az', 'raw_gx', 'raw_gy', 'raw_gz',
    'kf6_x', 'kf6_y', 'kf6_vel', 'kf6_heading', 'kf6_bgz',
    'zaru_flags', 'tbias_gz',
    'sensor_ax', 'sensor_ay', 'sensor_az',
    'sensor_gx', 'sensor_gy', 'sensor_gz',
    'gps_fix_us', 'gps_valid',
    'nav_speed2d', 'nav_s_acc', 'nav_vel_n', 'nav_vel_e',
    'nav_vel_valid', 'gps_speed_source', 'nav_fix_us',
    'dhv_gdspd', 'dhv_fix_us',
]

FIELD_NAMES_215 = FIELD_NAMES_202 + ['mag_mx', 'mag_my', 'mag_mz', 'mag_valid']

FIELD_NAMES_190 = [
    't_us',
    'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c',
    'lap',
    'gps_lat', 'gps_lon', 'gps_sog_kmh', 'gps_alt_m',
    'gps_sats', 'gps_hdop',
    'kf_x', 'kf_y', 'kf_vel', 'kf_heading',
    'raw_ax', 'raw_ay', 'raw_az', 'raw_gx', 'raw_gy', 'raw_gz',
    'kf6_x', 'kf6_y', 'kf6_vel', 'kf6_heading', 'kf6_bgz',
    'zaru_flags', 'tbias_gz',
    'sensor_ax', 'sensor_ay', 'sensor_az',
    'sensor_gx', 'sensor_gy', 'sensor_gz',
    'gps_fix_us', 'gps_valid',  # SITL timing metadata
    'nav_speed2d', 'nav_s_acc', 'nav_vel_n', 'nav_vel_e',
    'nav_vel_valid', 'gps_speed_source', 'nav_fix_us',  # NAV-PV velocity (v1.5.0)
]

FIELD_NAMES_164 = [
    't_us',  # uint64 µs — IMU hardware clock
    'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c',
    'lap',
    'gps_lat', 'gps_lon', 'gps_sog_kmh', 'gps_alt_m',
    'gps_sats', 'gps_hdop',
    'kf_x', 'kf_y', 'kf_vel', 'kf_heading',
    'raw_ax', 'raw_ay', 'raw_az', 'raw_gx', 'raw_gy', 'raw_gz',
    'kf6_x', 'kf6_y', 'kf6_vel', 'kf6_heading', 'kf6_bgz',
    'zaru_flags', 'tbias_gz',
    'sensor_ax', 'sensor_ay', 'sensor_az',
    'sensor_gx', 'sensor_gy', 'sensor_gz',
    'gps_fix_us', 'gps_valid'  # SITL timing metadata — not exported as MoTeC channels
]

FIELD_NAMES_155 = [
    't_us',  # was t_ms — microsecond precision
    'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c',
    'lap',
    'gps_lat', 'gps_lon', 'gps_sog_kmh', 'gps_alt_m',
    'gps_sats', 'gps_hdop',
    'kf_x', 'kf_y', 'kf_vel', 'kf_heading',
    'raw_ax', 'raw_ay', 'raw_az', 'raw_gx', 'raw_gy', 'raw_gz',
    'kf6_x', 'kf6_y', 'kf6_vel', 'kf6_heading', 'kf6_bgz',
    'zaru_flags', 'tbias_gz',
    'sensor_ax', 'sensor_ay', 'sensor_az',
    'sensor_gx', 'sensor_gy', 'sensor_gz'
]

FIELD_NAMES_122 = [
    't_ms',
    'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c',
    'lap',
    'gps_lat', 'gps_lon', 'gps_sog_kmh', 'gps_alt_m',
    'gps_sats', 'gps_hdop',
    'kf_x', 'kf_y', 'kf_vel', 'kf_heading',
    'raw_ax', 'raw_ay', 'raw_az', 'raw_gx', 'raw_gy', 'raw_gz',
    'kf6_x', 'kf6_y', 'kf6_vel', 'kf6_heading', 'kf6_bgz',
]

FIELD_NAMES_127 = FIELD_NAMES_122 + ['zaru_flags', 'tbias_gz']

FIELD_NAMES_78 = [
    't_ms',
    'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'temp_c',
    'lap',
    'gps_lat', 'gps_lon', 'gps_sog_kmh', 'gps_alt_m',
    'gps_sats', 'gps_hdop',
    'kf_x', 'kf_y', 'kf_vel', 'kf_heading',
]

# ── MoTeC .ld binary format ───────────────────────────────────────────────────
# Reverse-engineered from github.com/gotzl/ldparser (MIT license).
# All multi-byte values are little-endian.
#
# File layout:
#   [0]               HEADER      (LD_HEAD_SIZE = 1762 B)
#   [LD_HEAD_SIZE]    EVENT BLOCK (LD_EVENT_SIZE = 1154 B)
#   [event + event_sz] CHAN META  (LD_CHAN_SIZE = 124 B × num_channels)
#   [chan_data_offset]  DATA      (float32 arrays, one per channel, contiguous)

LD_HEAD_FMT  = '<I4xII20xI24xHHHI8sHHI4x16s16x16s16x64s64s64x64s64x1024xI66x64s126x'
LD_EVENT_FMT = '<64s64s1024sH'
LD_CHAN_FMT  = '<IIIIHHHHhhhh32s8s12s40x'

LD_HEAD_SIZE  = struct.calcsize(LD_HEAD_FMT)    # 1762
LD_EVENT_SIZE = struct.calcsize(LD_EVENT_FMT)   # 1154
LD_CHAN_SIZE  = struct.calcsize(LD_CHAN_FMT)     # 124


def _pad_str(s, length):
    """Encode string as ASCII and pad/truncate to exactly `length` bytes."""
    return s.encode('ascii', errors='replace')[:length].ljust(length, b'\x00')


# ── Binary input reader ───────────────────────────────────────────────────────

def _read_bin_file_header(path):
    """Read FileHeader. Returns (header_dict, data_offset) or (None, 0)."""
    with open(path, 'rb') as f:
        magic = f.read(3)
        if magic != b'TEL':
            return None, 0
        
        hdr_ver_byte = f.read(1)
        if not hdr_ver_byte:
            return None, 0
        hdr_ver = struct.unpack('<B', hdr_ver_byte)[0]
        
        if hdr_ver >= 3:
            # Header v3 (v1.4.0+): 66 bytes (adds 40 bytes of calibration params)
            rest = f.read(62)  # 66 - 4 already read
            fw_ver_b, rec_size, start_ms = struct.unpack('<16sHI', rest[:22])
            # 40 bytes of calibration params skipped (not needed for MoTeC)
            data_offset = 66
        elif hdr_ver >= 2:
            # Header v2 (v1.0.0+): 26 bytes (record_size is uint16_t -> 'H')
            rest = f.read(22)
            fw_ver_b, rec_size, start_ms = struct.unpack('<16sHI', rest)
            data_offset = 26
        else:
            # Header v1 (v0.9.7): 25 bytes (record_size is uint8_t -> 'B')
            rest = f.read(21)
            fw_ver_b, rec_size, start_ms = struct.unpack('<16sBI', rest)
            data_offset = 25
            
        fw_str = fw_ver_b.split(b'\x00')[0].decode('ascii', errors='replace')
        return {'firmware_version': fw_str, 'record_size': rec_size, 'start_ms': start_ms}, data_offset


def read_bin(path):
    """
    Read a telemetry .bin file.
    Returns (records: list[dict], fw_version: str, has_raw: bool).
    """
    hdr, data_offset = _read_bin_file_header(path)

    if hdr:
        record_size = hdr['record_size']
        fw_version  = hdr['firmware_version']
        print(f'        FileHeader: firmware {fw_version}, '
              f'record_size={record_size}B, start={hdr["start_ms"]}ms')
    else:
        record_size = RECORD_SIZE_122   # assume latest format for legacy files
        fw_version  = 'unknown'
        print(f'        No FileHeader (legacy file) — assuming {record_size}B record size')

    if record_size == RECORD_SIZE_215:
        fmt        = RECORD_FMT_215
        fields     = FIELD_NAMES_215
        has_raw    = True
    elif record_size == RECORD_SIZE_202:
        fmt        = RECORD_FMT_202
        fields     = FIELD_NAMES_202
        has_raw    = True
    elif record_size == RECORD_SIZE_190:
        fmt        = RECORD_FMT_190
        fields     = FIELD_NAMES_190
        has_raw    = True
    elif record_size == RECORD_SIZE_164:
        fmt        = RECORD_FMT_164
        fields     = FIELD_NAMES_164
        has_raw    = True
    elif record_size == RECORD_SIZE_155:
        fmt        = RECORD_FMT_155
        fields     = FIELD_NAMES_155
        has_raw    = True
    elif record_size == RECORD_SIZE_127:
        fmt        = RECORD_FMT_127
        fields     = FIELD_NAMES_127
        has_raw    = True
    elif record_size == RECORD_SIZE_122:
        fmt        = RECORD_FMT_122
        fields     = FIELD_NAMES_122
        has_raw    = True
    elif record_size == RECORD_SIZE_78:
        fmt        = RECORD_FMT_78
        fields     = FIELD_NAMES_78
        has_raw    = False
    else:
        print(f'[MoTeC] WARNING: unknown record size {record_size}B — '
              f'attempting 122B parse, output may be corrupted.')
        fmt        = RECORD_FMT_122
        fields     = FIELD_NAMES_122
        has_raw    = True

    records = []
    with open(path, 'rb') as f:
        if data_offset:
            f.seek(data_offset)
        while True:
            chunk = f.read(record_size)
            if len(chunk) < record_size:
                break
            vals = struct.unpack(fmt, chunk)
            if vals[0] == SENTINEL_CALIB:
                continue  # CalibrationRecord sentinel (uint64 max)
            records.append(dict(zip(fields, vals)))

    return records, fw_version, has_raw


# ── CSV input reader ──────────────────────────────────────────────────────────

def read_csv(path):
    """
    Read a telemetry CSV file (output of bin_to_csv.py).
    Column headers like 'ax (G)' are stripped to 'ax' for uniform key access.
    Returns (records: list[dict], fw_version: str, has_raw: bool).
    """
    records = []
    with open(path, 'r') as f:
        raw_headers = f.readline().rstrip('\n').split(',')

        # Strip unit annotations: 'ax (G)' → 'ax', 'gps_lat (°)' → 'gps_lat'
        headers = []
        for h in raw_headers:
            h = h.strip()
            paren = h.find(' (')
            headers.append(h[:paren] if paren != -1 else h)

        has_raw = 'raw_ax' in headers

        for line in f:
            line = line.rstrip('\n')
            if not line:
                continue
            parts = line.split(',')
            rec = {}
            for key, val in zip(headers, parts):
                if not val:
                    rec[key] = 0.0
                else:
                    try:
                        rec[key] = float(val)
                    except ValueError:
                        rec[key] = 0.0
            records.append(rec)

    return records, 'unknown', has_raw


# ── MoTeC .ld writer ──────────────────────────────────────────────────────────

def _write_ld_file(output_path, channels_def, records, venue, driver, vehicle, comment):
    """
    Write a native MoTeC .ld file.

    channels_def: list of (name, short_name, unit, freq_hz, extractor_fn)
      where extractor_fn(record_dict) → float
      freq_hz must be a divisor of 50 (e.g. 50, 10, 5, 1).
    """
    n            = len(records)
    now          = datetime.now()
    date_str     = now.strftime('%d/%m/%Y')
    time_str     = now.strftime('%H:%M:%S')
    num_channels = len(channels_def)

    # Extract per-channel data arrays
    # Channels at freq < 50 Hz are decimated (take every n-th record from 50 Hz stream).
    channel_data = []
    for _name, _short, _unit, freq, extractor in channels_def:
        if freq < 50:
            step     = 50 // freq
            n_samples = n // step
            data = [extractor(records[i * step]) for i in range(n_samples)]
        else:
            data = [extractor(rec) for rec in records]
        channel_data.append(data)

    # File layout offsets
    event_offset     = LD_HEAD_SIZE
    chan_meta_offset = event_offset + LD_EVENT_SIZE
    chan_data_offset = chan_meta_offset + num_channels * LD_CHAN_SIZE

    data_offsets = []
    cur_offset = chan_data_offset
    for data in channel_data:
        data_offsets.append(cur_offset)
        cur_offset += len(data) * 4     # float32 = 4 bytes/sample

    # float32 clamp guard (prevents struct.pack overflow on NaN/Inf)
    F32_MAX = 3.4028235e+38

    with open(output_path, 'wb') as f:

        # ── HEADER (LD_HEAD_SIZE = 1762 B) ───────────────────────────────────
        f.write(struct.pack(
            LD_HEAD_FMT,
            0x40,                          # ldmarker
            chan_meta_offset,              # chann_meta_ptr
            chan_data_offset,              # chann_data_ptr
            event_offset,                  # event_ptr
            0x0001,                        # unknown1
            0x4240,                        # unknown2
            0x000F,                        # unknown3
            0x1F44,                        # device_serial
            _pad_str('ADL', 8),            # device_type
            420,                           # device_version
            0xADB0,                        # unknown4
            num_channels,                  # num_channs
            _pad_str(date_str, 16),        # date (DD/MM/YYYY)
            _pad_str(time_str, 16),        # time (HH:MM:SS)
            _pad_str(driver, 64),          # driver
            _pad_str(vehicle, 64),         # vehicleid
            _pad_str(venue, 64),           # venue
            0xC81A4,                       # pro_logging magic
            _pad_str(comment, 64),         # short_comment
        ))

        # ── EVENT BLOCK (LD_EVENT_SIZE = 1154 B) ─────────────────────────────
        f.write(struct.pack(
            LD_EVENT_FMT,
            _pad_str('Session', 64),
            _pad_str('Telemetria', 64),
            _pad_str(comment, 1024),
            0,                             # venue_ptr (no sub-block)
        ))

        # ── CHANNEL METADATA (LD_CHAN_SIZE = 124 B × num_channels) ───────────
        for i, (name, short, unit, freq, _) in enumerate(channels_def):
            prev_ptr = chan_meta_offset + (i - 1) * LD_CHAN_SIZE if i > 0           else 0
            next_ptr = chan_meta_offset + (i + 1) * LD_CHAN_SIZE if i < num_channels - 1 else 0
            f.write(struct.pack(
                LD_CHAN_FMT,
                prev_ptr,
                next_ptr,
                data_offsets[i],
                len(channel_data[i]),
                0x2EE1 + i,                # channel counter (ldparser convention)
                0x0007,                    # dtype_a (float)
                0x0004,                    # dtype_b (4 bytes → float32)
                freq,                      # rec_freq
                0,                         # shift
                1,                         # mul
                1,                         # scale
                0,                         # dec_places
                _pad_str(name, 32),
                _pad_str(short, 8),
                _pad_str(unit, 12),
            ))

        # ── CHANNEL DATA (float32, one contiguous block per channel) ─────────
        for data in channel_data:
            clamped = [
                max(-F32_MAX, min(F32_MAX, v)) if math.isfinite(v) else 0.0
                for v in data
            ]
            f.write(struct.pack(f'<{len(clamped)}f', *clamped))

    return channel_data


def export_motec_ld(records, has_raw, output_path, venue, fw_version, input_name):
    """Build channel definitions and write the .ld file."""
    n = len(records)
    if n == 0:
        print('[MoTeC] ERROR: no records to export.')
        return

    # IMU source: raw post-Madgwick channels (zero latency) when available,
    # EMA-filtered channels as fallback for legacy files.
    use_ema = not has_raw
    if use_ema:
        print('[MoTeC] WARNING: no raw IMU channels found — falling back to EMA '
              '(ax/ay/az, gx/gy/gz).')
        print('         EMA has ~313ms phase delay. For clean data use firmware v0.9.8+.')

    ax_key = 'ax'     if use_ema else 'raw_ax'
    ay_key = 'ay'     if use_ema else 'raw_ay'
    az_key = 'az'     if use_ema else 'raw_az'
    gx_key = 'gx'     if use_ema else 'raw_gx'
    gy_key = 'gy'     if use_ema else 'raw_gy'
    gz_key = 'gz'     if use_ema else 'raw_gz'

    channels_def = [
        # (name,           short,     unit,    Hz,  extractor)
        ('Ground Speed',   'GndSpd',  'km/h',  50,  lambda r: r.get('kf_vel', 0.0) * 3.6),
        ('G Force Long',   'GFrcLng', 'G',     50,  lambda r, k=ax_key: r.get(k, 0.0)),
        ('G Force Lat',    'GFrcLat', 'G',     50,  lambda r, k=ay_key: r.get(k, 0.0)),
        ('G Force Vert',   'GFrcVrt', 'G',     50,  lambda r, k=az_key: r.get(k, 0.0)),
        ('Yaw Rate',       'YawRt',   'deg/s', 50,  lambda r, k=gz_key: r.get(k, 0.0)),
        ('Roll Rate',      'RollRt',  'deg/s', 50,  lambda r, k=gx_key: r.get(k, 0.0)),
        ('Pitch Rate',     'PtchRt',  'deg/s', 50,  lambda r, k=gy_key: r.get(k, 0.0)),
        ('GPS Latitude',   'GPSLat',  'deg',   10,  lambda r: float(r.get('gps_lat', 0.0))),
        ('GPS Longitude',  'GPSLon',  'deg',   10,  lambda r: float(r.get('gps_lon', 0.0))),
        ('GPS Altitude',   'GPSAlt',  'm',     10,  lambda r: float(r.get('gps_alt_m',
                                                        r.get('gps_alt', 0.0)))),
        ('GPS Speed (SOG)', 'GPSSpd',  'km/h',  10,  lambda r: float(r.get('gps_sog_kmh', r.get('gps_speed_kmh', 0.0)))),
        ('GPS Speed (DHV)', 'DhvSpd',  'm/s',   10,  lambda r: float(r.get('dhv_gdspd', 0.0))),
        ('GPS Speed (nav)', 'NavSpd',  'm/s',   10,  lambda r: float(r.get('nav_speed2d', 0.0))),
        ('GPS Speed Acc',  'SpdAcc',  'm/s',  10, lambda r: float(r.get('nav_s_acc', 0.0))),
        ('Sensor Temp',    'Temp',    'C',     50,  lambda r: r.get('temp_c', 0.0)),
        ('Mag X',          'MagX',    'raw',   10,  lambda r: float(r.get('mag_mx', 0.0))),
        ('Mag Y',          'MagY',    'raw',   10,  lambda r: float(r.get('mag_my', 0.0))),
        ('Mag Z',          'MagZ',    'raw',   10,  lambda r: float(r.get('mag_mz', 0.0))),
    ]
    # Index reference for summary (GPS Latitude is at index 7)
    GPS_LAT_IDX = 7

    comment      = f'{fw_version} - {input_name}'
    channel_data = _write_ld_file(
        output_path, channels_def, records,
        venue=venue, driver='Niccolo', vehicle='VCU AtomS3', comment=comment,
    )

    # Summary
    duration_s    = (n - 1) * 0.020
    file_size     = os.path.getsize(output_path)
    max_speed     = max((r.get('kf_vel', 0.0) * 3.6 for r in records), default=0.0)
    gps_lat_data  = channel_data[GPS_LAT_IDX]
    gps_fix_count = sum(1 for v in gps_lat_data if abs(v) > 1e-6)

    print(f'[MoTeC] Exported: {output_path}')
    print(f'        Format:     native MoTeC .ld (i2 Pro)')
    print(f'        Samples:    {n} @ 50Hz  ({duration_s:.1f}s)')
    n50 = sum(1 for c in channels_def if c[3] == 50)
    n10 = sum(1 for c in channels_def if c[3] == 10)
    print(f'        Channels:   {len(channels_def)}  ({n50}@50Hz + {n10}@10Hz)')
    print(f'        GPS fixes:  {gps_fix_count} @ 10Hz')
    print(f'        Max speed:  {max_speed:.1f} km/h')
    print(f'        File size:  {file_size / 1024:.1f} KB')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Convert ESP32 telemetry (.bin/.csv) to native MoTeC i2 Pro .ld')
    parser.add_argument('input',
                        help='Input file (.bin binary recording or .csv from bin_to_csv.py)')
    parser.add_argument('-o', '--output',
                        help='Output .ld path  (default: <input>.ld in same directory)')
    parser.add_argument('--venue', default='Track',
                        help='Venue / circuit name written into the .ld header  (default: Track)')
    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f'[MoTeC] ERROR: file not found: {args.input}')
        sys.exit(1)

    base, ext = os.path.splitext(args.input)
    ext        = ext.lower()
    input_name = os.path.basename(base)
    output_path = args.output if args.output else f'{base}.ld'

    if ext == '.bin':
        print(f'[MoTeC] Reading binary: {args.input}')
        records, fw_version, has_raw = read_bin(args.input)
    elif ext == '.csv':
        print(f'[MoTeC] Reading CSV: {args.input}')
        records, fw_version, has_raw = read_csv(args.input)
    else:
        print(f'[MoTeC] ERROR: unsupported format "{ext}"  (use .bin or .csv)')
        sys.exit(1)

    if not has_raw:
        fmt_label = '78B (EMA only)'
    elif records and 'gps_fix_us' in records[0]:
        fmt_label = '164B (raw+6D+ZARU+SITL+GPS timing)'
    elif records and 'sensor_ax' in records[0]:
        fmt_label = '155B (raw+6D+ZARU+SITL)'
    elif records and 'zaru_flags' in records[0]:
        fmt_label = '127B (raw+6D+ZARU)'
    else:
        fmt_label = '122B (raw+6D)'
    print(f'        Records: {len(records)}, format: {fmt_label}')

    export_motec_ld(records, has_raw, output_path, args.venue, fw_version, input_name)


if __name__ == '__main__':
    main()
