#!/usr/bin/env python3
"""Convert and validate MPU6886 static bench binary logs.

Checks:
- file/header magic, header CRC, record size
- per-record CRC16
- fixed-record alignment and optional resync scan
- seq continuity and timestamp monotonicity
- drop estimate from seq gaps and timestamp gaps
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


HEADER_SIZE = 256
RECORD_SIZE = 256
HEADER_MAGIC = b"MPU6886B"
RECORD_MAGIC = 0x4236384D
RECORD_MAGIC_BYTES = struct.pack("<I", RECORD_MAGIC)

HEADER_STRUCT = struct.Struct("<8sHHHHIQ24s16s16sHHHHHH128s30sH")
RECORD_STRUCT = struct.Struct("<IIQHHHHIIhhhhhhhIIIIIIIIIIIIHHIII144sH")

ACC_LSB_PER_G = 4096.0
GYRO_LSB_PER_DPS = 16.384
TEMP_LSB_PER_C = 326.8
TEMP_OFFSET_C = 25.0
NOMINAL_PERIOD_US = 20_000


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cstr(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")


@dataclass
class Summary:
    input: str
    csv: str | None
    file_size: int
    header_ok: bool
    header_crc_ok: bool
    record_size: int
    payload_bytes: int
    payload_mod_record_size: int
    records_valid: int
    records_crc_bad: int
    resync_count: int
    stray_bytes: int
    seq_gaps: int
    seq_drop_estimate: int
    timestamp_nonmonotonic: int
    timestamp_gap_drop_estimate: int
    timestamp_dt_min_us: int | None
    timestamp_dt_max_us: int | None
    timestamp_dt_mean_us: float | None
    timestamp_jitter_rms_us: float | None
    sample_fresh_false: int
    fifo_overrun_records: int
    fifo_overrun_counter_final: int | None
    sd_records_written_final: int | None
    sd_records_dropped_final: int | None
    sd_partial_write_count_final: int | None
    sd_stall_count_final: int | None
    sd_reopen_count_final: int | None
    sd_flush_worst_us_final: int | None
    sd_queue_high_watermark_final: int | None
    read_error_count_final: int | None
    duration_s: float | None
    fs_mean_hz: float | None


def parse_header(blob: bytes) -> dict:
    if len(blob) < HEADER_SIZE:
        raise ValueError("file shorter than 256-byte header")
    values = HEADER_STRUCT.unpack(blob[:HEADER_SIZE])
    keys = (
        "magic",
        "header_version",
        "header_size",
        "record_size",
        "endian_marker",
        "firmware_build",
        "log_start_us",
        "firmware_version",
        "sensor_type",
        "board",
        "raw_odr_hz",
        "output_odr_hz",
        "decimation_ratio",
        "fifo_frame_size",
        "accel_range_g",
        "gyro_range_dps",
        "reg_dump",
        "reserved",
        "crc16",
    )
    header = dict(zip(keys, values))
    header["firmware_version"] = cstr(header["firmware_version"])
    header["sensor_type"] = cstr(header["sensor_type"])
    header["board"] = cstr(header["board"])
    header["header_crc_calc"] = crc16_ccitt(blob[: HEADER_SIZE - 2])
    header["header_crc_ok"] = header["header_crc_calc"] == header["crc16"]
    header["magic_ok"] = header["magic"] == HEADER_MAGIC
    return header


RECORD_KEYS = (
    "magic",
    "seq",
    "timestamp_us",
    "fifo_count_before",
    "fifo_count_after",
    "fifo_frames_drained",
    "decimation_counter",
    "flags",
    "read_error_count",
    "acc_x_raw",
    "acc_y_raw",
    "acc_z_raw",
    "gyro_x_raw",
    "gyro_y_raw",
    "gyro_z_raw",
    "temp_raw",
    "sd_queue_high_watermark",
    "sd_records_written",
    "sd_records_dropped",
    "sd_partial_write_count",
    "sd_stall_count",
    "sd_reopen_count",
    "sd_flush_worst_us",
    "sd_records_enqueued",
    "sd_partial_current_record",
    "fifo_overrun_count",
    "fifo_reset_count",
    "sd_stall_worst_ms",
    "int_status",
    "sample_period_us",
    "imu_loop_worst_us",
    "sd_reopen_fail_count",
    "sd_write_zero_count",
    "reserved",
    "crc16",
)


def parse_record(raw: bytes) -> dict:
    values = RECORD_STRUCT.unpack(raw)
    rec = dict(zip(RECORD_KEYS, values))
    rec["crc16_calc"] = crc16_ccitt(raw[:-2])
    rec["crc_ok"] = rec["crc16_calc"] == rec["crc16"]
    flags = rec["flags"]
    rec["sample_fresh"] = bool(flags & (1 << 0))
    rec["fifo_overrun"] = bool(flags & (1 << 1))
    rec["fifo_misaligned"] = bool(flags & (1 << 2))
    rec["i2c_error"] = bool(flags & (1 << 3))
    rec["sd_queue_drop_flag"] = bool(flags & (1 << 4))
    rec["acc_x_g"] = rec["acc_x_raw"] / ACC_LSB_PER_G
    rec["acc_y_g"] = rec["acc_y_raw"] / ACC_LSB_PER_G
    rec["acc_z_g"] = rec["acc_z_raw"] / ACC_LSB_PER_G
    rec["gyro_x_dps"] = rec["gyro_x_raw"] / GYRO_LSB_PER_DPS
    rec["gyro_y_dps"] = rec["gyro_y_raw"] / GYRO_LSB_PER_DPS
    rec["gyro_z_dps"] = rec["gyro_z_raw"] / GYRO_LSB_PER_DPS
    rec["temp_c"] = rec["temp_raw"] / TEMP_LSB_PER_C + TEMP_OFFSET_C
    rec.pop("reserved", None)
    return rec


def iter_records(payload: bytes):
    pos = 0
    resync_count = 0
    stray_bytes = 0
    while pos + RECORD_SIZE <= len(payload):
        chunk = payload[pos : pos + RECORD_SIZE]
        magic = struct.unpack_from("<I", chunk, 0)[0]
        if magic == RECORD_MAGIC:
            yield pos, chunk, resync_count, stray_bytes
            pos += RECORD_SIZE
            continue

        next_pos = payload.find(RECORD_MAGIC_BYTES, pos + 1)
        if next_pos < 0 or next_pos + RECORD_SIZE > len(payload):
            stray_bytes += len(payload) - pos
            break
        stray_bytes += next_pos - pos
        resync_count += 1
        pos = next_pos


def convert(path: Path, csv_path: Path | None, summary_json: Path | None) -> Summary:
    blob = path.read_bytes()
    header = parse_header(blob)
    if not header["magic_ok"]:
        raise ValueError(f"bad header magic: {header['magic']!r}")
    if header["record_size"] != RECORD_SIZE:
        raise ValueError(f"unsupported record size {header['record_size']}, expected {RECORD_SIZE}")

    payload = blob[HEADER_SIZE:]
    rows: list[dict] = []
    crc_bad = 0
    resync_count = 0
    stray_bytes = len(payload) % RECORD_SIZE

    for _pos, chunk, resync_seen, stray_seen in iter_records(payload):
        resync_count = max(resync_count, resync_seen)
        stray_bytes = max(stray_bytes, stray_seen)
        rec = parse_record(chunk)
        if not rec["crc_ok"]:
            crc_bad += 1
        rows.append(rec)

    if csv_path is not None:
        fieldnames = [k for k in RECORD_KEYS if k not in ("magic", "reserved", "crc16")]
        fieldnames += [
            "crc_ok",
            "sample_fresh",
            "fifo_overrun",
            "fifo_misaligned",
            "i2c_error",
            "sd_queue_drop_flag",
            "acc_x_g",
            "acc_y_g",
            "acc_z_g",
            "gyro_x_dps",
            "gyro_y_dps",
            "gyro_z_dps",
            "temp_c",
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(rows)

    seq_gaps = 0
    seq_drop_estimate = 0
    timestamp_nonmonotonic = 0
    timestamp_gap_drop_estimate = 0
    dts: list[int] = []

    for prev, cur in zip(rows, rows[1:]):
        expected = (prev["seq"] + 1) & 0xFFFFFFFF
        if cur["seq"] != expected:
            seq_gaps += 1
            if cur["seq"] > prev["seq"]:
                seq_drop_estimate += cur["seq"] - prev["seq"] - 1

        dt = cur["timestamp_us"] - prev["timestamp_us"]
        if dt <= 0:
            timestamp_nonmonotonic += 1
        else:
            dts.append(dt)
            if dt > int(NOMINAL_PERIOD_US * 1.5):
                timestamp_gap_drop_estimate += max(0, round(dt / NOMINAL_PERIOD_US) - 1)

    dt_mean = sum(dts) / len(dts) if dts else None
    jitter_rms = None
    if dts:
        jitter_rms = (sum((dt - NOMINAL_PERIOD_US) ** 2 for dt in dts) / len(dts)) ** 0.5

    duration_s = None
    fs_mean_hz = None
    if len(rows) >= 2 and rows[-1]["timestamp_us"] > rows[0]["timestamp_us"]:
        duration_s = (rows[-1]["timestamp_us"] - rows[0]["timestamp_us"]) / 1_000_000.0
        fs_mean_hz = (len(rows) - 1) / duration_s

    last = rows[-1] if rows else {}
    summary = Summary(
        input=str(path),
        csv=str(csv_path) if csv_path else None,
        file_size=len(blob),
        header_ok=header["magic_ok"],
        header_crc_ok=header["header_crc_ok"],
        record_size=header["record_size"],
        payload_bytes=len(payload),
        payload_mod_record_size=len(payload) % RECORD_SIZE,
        records_valid=len(rows) - crc_bad,
        records_crc_bad=crc_bad,
        resync_count=resync_count,
        stray_bytes=stray_bytes,
        seq_gaps=seq_gaps,
        seq_drop_estimate=seq_drop_estimate,
        timestamp_nonmonotonic=timestamp_nonmonotonic,
        timestamp_gap_drop_estimate=timestamp_gap_drop_estimate,
        timestamp_dt_min_us=min(dts) if dts else None,
        timestamp_dt_max_us=max(dts) if dts else None,
        timestamp_dt_mean_us=dt_mean,
        timestamp_jitter_rms_us=jitter_rms,
        sample_fresh_false=sum(1 for r in rows if not r["sample_fresh"]),
        fifo_overrun_records=sum(1 for r in rows if r["fifo_overrun"]),
        fifo_overrun_counter_final=last.get("fifo_overrun_count"),
        sd_records_written_final=last.get("sd_records_written"),
        sd_records_dropped_final=last.get("sd_records_dropped"),
        sd_partial_write_count_final=last.get("sd_partial_write_count"),
        sd_stall_count_final=last.get("sd_stall_count"),
        sd_reopen_count_final=last.get("sd_reopen_count"),
        sd_flush_worst_us_final=last.get("sd_flush_worst_us"),
        sd_queue_high_watermark_final=last.get("sd_queue_high_watermark"),
        read_error_count_final=last.get("read_error_count"),
        duration_s=duration_s,
        fs_mean_hz=fs_mean_hz,
    )

    if summary_json is not None:
        summary_json.write_text(json.dumps(asdict(summary), indent=2), encoding="utf-8")

    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bin", type=Path, help="Input MPU6886_###.BIN log")
    parser.add_argument("--csv", type=Path, help="Output CSV path; default: input suffix .csv")
    parser.add_argument("--no-csv", action="store_true", help="Validate only")
    parser.add_argument("--summary-json", type=Path, help="Optional JSON summary path")
    args = parser.parse_args(argv)

    csv_path = None if args.no_csv else (args.csv or args.bin.with_suffix(".csv"))
    summary = convert(args.bin, csv_path, args.summary_json)
    print(json.dumps(asdict(summary), indent=2))

    failed = (
        not summary.header_ok
        or not summary.header_crc_ok
        or summary.payload_mod_record_size != 0
        or summary.records_crc_bad != 0
        or summary.resync_count != 0
        or summary.seq_gaps != 0
        or summary.timestamp_nonmonotonic != 0
    )
    return 2 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
