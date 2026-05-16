"""Local Telemetria session server.

Imports data/samples/BIN logs into a compact Float32 channel-major cache and serves the
existing browser UI with a small FastAPI API.

Run:
    python tools/telemetry_server.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import socket
import struct
import tempfile
from pathlib import Path
from typing import Any, Iterable

import numpy as np

try:
    from fastapi import FastAPI, File, HTTPException, UploadFile
    from fastapi.responses import FileResponse
    from fastapi.staticfiles import StaticFiles
except ModuleNotFoundError:  # Dependencies may be installed after first checkout.
    FastAPI = None  # type: ignore[assignment]
    File = None  # type: ignore[assignment]
    HTTPException = None  # type: ignore[assignment]
    UploadFile = None  # type: ignore[assignment]
    FileResponse = None  # type: ignore[assignment]
    StaticFiles = None  # type: ignore[assignment]

try:  # Support both `python -m Tool.telemetry_server` and direct imports.
    from . import bin_to_csv
except ImportError:  # pragma: no cover - convenience for ad-hoc local runs.
    import bin_to_csv  # type: ignore


SCHEMA = "telemetria.float32.v1"
REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_ROOT = Path(__file__).resolve().parent
CACHE_ROOT = TOOL_ROOT / ".telemetry_cache"
UI_ROOT = REPO_ROOT / "design-system" / "telemetry_app"
DESIGN_ROOT = REPO_ROOT / "design-system"

CANONICAL_CHANNELS = [
    "idx",
    "t_s",
    "distance_m",
    "vel_kmh",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "raw_ax",
    "raw_ay",
    "raw_az",
    "raw_gx",
    "raw_gy",
    "raw_gz",
    "acc_mag",
    "raw_acc_mag",
    "gps_lat",
    "gps_lon",
    "kf_lat",
    "kf_lon",
    "kf_x",
    "kf_y",
    "lap",
]

REQUIRED_BASE = [
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "gps_lat",
    "gps_lon",
    "kf_vel",
    "kf_x",
    "kf_y",
]

CSV_ALIAS = {
    "t_us": "t_us",
    "timestamp_us": "t_us",
    "t_ms": "t_ms",
    "ax": "ax",
    "ay": "ay",
    "az": "az",
    "gx": "gx",
    "gy": "gy",
    "gz": "gz",
    "butter_ax": "ax",
    "butter_ay": "ay",
    "butter_az": "az",
    "butter_gx": "gx",
    "butter_gy": "gy",
    "butter_gz": "gz",
    "lap": "lap",
    "gps_lat": "gps_lat",
    "gps_lon": "gps_lon",
    "gps_sog_kmh": "gps_sog_kmh",
    "gps_speed_kmh": "gps_sog_kmh",
    "kf_x": "kf_x",
    "kf_y": "kf_y",
    "kf_vel": "kf_vel",
    "raw_ax": "raw_ax",
    "raw_ay": "raw_ay",
    "raw_az": "raw_az",
    "raw_gx": "raw_gx",
    "raw_gy": "raw_gy",
    "raw_gz": "raw_gz",
    "pipe_lin_ax": "pipe_lin_ax",
    "pipe_lin_ay": "pipe_lin_ay",
    "pipe_lin_az": "pipe_lin_az",
    "pipe_body_gx": "pipe_body_gx",
    "pipe_body_gy": "pipe_body_gy",
    "pipe_body_gz": "pipe_body_gz",
    "bmi_acc_x_g": "bmi_acc_x_g",
    "bmi_acc_y_g": "bmi_acc_y_g",
    "bmi_acc_z_g": "bmi_acc_z_g",
    "bmi_gyr_x_dps": "bmi_gyr_x_dps",
    "bmi_gyr_y_dps": "bmi_gyr_y_dps",
    "bmi_gyr_z_dps": "bmi_gyr_z_dps",
}


def _base_name(name: str) -> str:
    return name.split(" (", 1)[0].strip().lower()


def _hash_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _json_write(path: Path, payload: dict[str, Any]) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _write_f32(path: Path, stack: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    arr = np.asarray(stack, dtype="<f4", order="C")
    arr.tofile(tmp)
    tmp.replace(path)


def _read_csv_source(path: Path) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Read CSV into base-name arrays. Prefer Polars, fall back to pandas."""
    fallback_reason = ""
    try:
        import polars as pl  # type: ignore

        df = pl.read_csv(
            path,
            comment_prefix="#",
            infer_schema_length=10_000,
            ignore_errors=True,
            encoding="utf8-lossy",
        )
        out: dict[str, np.ndarray] = {}
        for col in df.columns:
            alias = CSV_ALIAS.get(_base_name(col))
            if not alias or alias in out:
                continue
            out[alias] = df[col].cast(pl.Float64, strict=False).to_numpy()
        return out, {"reader": "polars"}
    except ModuleNotFoundError:
        pass
    except Exception as exc:
        # Fall back to pandas for awkward legacy CSVs rather than failing early.
        fallback_reason = str(exc)

    import pandas as pd

    df = pd.read_csv(path, encoding="utf-8-sig", comment="#")
    out = {}
    for col in df.columns:
        alias = CSV_ALIAS.get(_base_name(col))
        if not alias or alias in out:
            continue
        out[alias] = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=np.float64)
    return out, {"reader": "pandas", "fallback_reason": fallback_reason}


def _read_bin_source(path: Path) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    layout = bin_to_csv.bin_logical_layout(str(path), quiet=True)
    hdr = layout["hdr"]
    offset = layout["offset"]
    record_size = layout["chunk_size"]
    logical_end = layout["logical_end"]
    fmt, header_cols, _ = bin_to_csv.get_format(
        record_size, hdr.get("header_version") if hdr else None
    )
    fields = {_base_name(c): i for i, c in enumerate(header_cols)}
    wanted = set(CSV_ALIAS.values()) | {"t_us", "t_ms"}
    buckets: dict[str, list[float]] = {name: [] for name in wanted if name in fields}
    rows = 0
    resync_count = 0
    skipped_records = 0
    prev_ts_us = None

    with path.open("rb") as f:
        f.seek(offset)
        while True:
            record_pos = f.tell()
            if record_pos >= logical_end:
                break
            chunk = f.read(record_size)
            if len(chunk) < record_size:
                break
            vals = struct.unpack(fmt, chunk)
            if vals[0] == bin_to_csv.SENTINEL_CALIB:
                prev_ts_us = None
                continue
            if not bin_to_csv._record_plausible(vals, fields, prev_ts_us, chunk):
                skip = bin_to_csv._find_resync_skip(f, record_pos, fmt, record_size, fields)
                if skip is None:
                    f.seek(record_pos + record_size)
                    skipped_records += 1
                else:
                    f.seek(record_pos + skip)
                    resync_count += 1
                prev_ts_us = None
                continue
            for name, values in buckets.items():
                values.append(float(vals[fields[name]]))
            prev_ts_us = bin_to_csv._timestamp_us(vals, fields)
            rows += 1

    arrays = {name: np.asarray(values, dtype=np.float64) for name, values in buckets.items()}
    meta = {
        "reader": "bin_to_csv_registry",
        "firmware_version": hdr.get("firmware_version") if hdr else "legacy",
        "record_size": record_size,
        "header_version": hdr.get("header_version") if hdr else None,
        "logical_end": logical_end,
        "tail_bytes": layout.get("tail_bytes", 0),
        "valid_records": layout.get("valid_records"),
        "resync_count": resync_count,
        "skipped_records": skipped_records,
        "records_read": rows,
    }
    return arrays, meta


def _take_valid_rows(source: dict[str, np.ndarray], mask: np.ndarray) -> dict[str, np.ndarray]:
    return {name: arr[mask] if len(arr) == len(mask) else arr for name, arr in source.items()}


def _required(source: dict[str, np.ndarray], name: str) -> np.ndarray:
    arr = source.get(name)
    if arr is None:
        raise ValueError(f"Missing required telemetry column: {name}")
    return arr.astype(np.float64, copy=False)


def _optional(source: dict[str, np.ndarray], name: str, n: int, fill: float = math.nan) -> np.ndarray:
    arr = source.get(name)
    if arr is None:
        return np.full(n, fill, dtype=np.float64)
    return arr.astype(np.float64, copy=False)


def _has_all(source: dict[str, np.ndarray], names: Iterable[str]) -> bool:
    return all(name in source for name in names)


def _convert_kf_to_wgs84(
    gps_lat: np.ndarray,
    gps_lon: np.ndarray,
    kf_x: np.ndarray,
    kf_y: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    valid = np.isfinite(gps_lat) & np.isfinite(gps_lon) & (gps_lat != 0) & (gps_lon != 0)
    if not np.any(valid):
        return np.zeros_like(gps_lat), np.zeros_like(gps_lon)

    i0 = int(np.argmax(valid))
    r_earth = 6_371_000.0
    gps_lat0 = float(gps_lat[i0])
    gps_lon0 = float(gps_lon[i0])
    cos_lat_approx = math.cos(math.radians(gps_lat0))
    lat0 = gps_lat0 - math.degrees(float(kf_y[i0]) / r_earth)
    lon0 = gps_lon0 - math.degrees(float(kf_x[i0]) / (r_earth * cos_lat_approx))
    cos_lat0 = math.cos(math.radians(lat0))
    kf_lat = lat0 + np.degrees(kf_y / r_earth)
    kf_lon = lon0 + np.degrees(kf_x / (r_earth * cos_lat0))
    return kf_lat, kf_lon


def _canonicalize(source: dict[str, np.ndarray]) -> tuple[np.ndarray, dict[str, Any]]:
    ts = source.get("t_us")
    ts_scale = 1_000_000.0
    if ts is None:
        ts = source.get("t_ms")
        ts_scale = 1_000.0
    if ts is None:
        raise ValueError("Missing timestamp column: expected t_us or t_ms")

    ts = ts.astype(np.float64, copy=False)
    mask = np.isfinite(ts)
    if not np.any(mask):
        raise ValueError("No valid timestamp samples found")
    source = _take_valid_rows(source, mask)
    ts = ts[mask]
    n = len(ts)

    for name in REQUIRED_BASE:
        _required(source, name)

    t_s = (ts - ts[0]) / ts_scale
    kf_vel = _required(source, "kf_vel")
    dt = np.diff(t_s, prepend=t_s[0])
    dt[0] = 0.0
    dt = np.maximum(dt, 0.0)
    distance_m = np.cumsum(np.abs(kf_vel) * dt)
    vel_kmh = kf_vel * 3.6

    ax = _required(source, "ax")
    ay = _required(source, "ay")
    az = _required(source, "az")
    gx = _required(source, "gx")
    gy = _required(source, "gy")
    gz = _required(source, "gz")

    raw_source = "none"
    if _has_all(source, ["raw_ax", "raw_ay", "raw_az", "raw_gx", "raw_gy", "raw_gz"]):
        raw_source = "legacy_raw"
        raw_ax, raw_ay, raw_az = source["raw_ax"], source["raw_ay"], source["raw_az"]
        raw_gx, raw_gy, raw_gz = source["raw_gx"], source["raw_gy"], source["raw_gz"]
    elif _has_all(source, ["pipe_lin_ax", "pipe_lin_ay", "pipe_lin_az", "pipe_body_gx", "pipe_body_gy", "pipe_body_gz"]):
        raw_source = "pipe_pre_presentation"
        raw_ax, raw_ay, raw_az = source["pipe_lin_ax"], source["pipe_lin_ay"], source["pipe_lin_az"]
        raw_gx, raw_gy, raw_gz = source["pipe_body_gx"], source["pipe_body_gy"], source["pipe_body_gz"]
    elif _has_all(source, ["bmi_acc_x_g", "bmi_acc_y_g", "bmi_acc_z_g", "bmi_gyr_x_dps", "bmi_gyr_y_dps", "bmi_gyr_z_dps"]):
        raw_source = "bmi_physical"
        raw_ax, raw_ay, raw_az = source["bmi_acc_x_g"], source["bmi_acc_y_g"], source["bmi_acc_z_g"]
        raw_gx, raw_gy, raw_gz = source["bmi_gyr_x_dps"], source["bmi_gyr_y_dps"], source["bmi_gyr_z_dps"]
    else:
        raw_ax = raw_ay = raw_az = raw_gx = raw_gy = raw_gz = np.full(n, math.nan)

    gps_lat = _required(source, "gps_lat")
    gps_lon = _required(source, "gps_lon")
    kf_x = _required(source, "kf_x")
    kf_y = _required(source, "kf_y")
    kf_lat, kf_lon = _convert_kf_to_wgs84(gps_lat, gps_lon, kf_x, kf_y)
    lap = _optional(source, "lap", n, 1.0)

    acc_mag = np.sqrt(ax**2 + ay**2)
    raw_acc_mag = np.sqrt(raw_ax**2 + raw_ay**2)
    idx = np.arange(n, dtype=np.float64)

    channel_arrays = {
        "idx": idx,
        "t_s": t_s,
        "distance_m": distance_m,
        "vel_kmh": vel_kmh,
        "ax": ax,
        "ay": ay,
        "az": az,
        "gx": gx,
        "gy": gy,
        "gz": gz,
        "raw_ax": raw_ax,
        "raw_ay": raw_ay,
        "raw_az": raw_az,
        "raw_gx": raw_gx,
        "raw_gy": raw_gy,
        "raw_gz": raw_gz,
        "acc_mag": acc_mag,
        "raw_acc_mag": raw_acc_mag,
        "gps_lat": gps_lat,
        "gps_lon": gps_lon,
        "kf_lat": kf_lat,
        "kf_lon": kf_lon,
        "kf_x": kf_x,
        "kf_y": kf_y,
        "lap": lap,
    }
    stack = np.vstack([channel_arrays[name] for name in CANONICAL_CHANNELS]).astype("<f4")
    sample_rate_hz = None
    if n > 1:
        diffs = np.diff(t_s)
        finite = diffs[np.isfinite(diffs) & (diffs > 0)]
        if finite.size:
            sample_rate_hz = float(1.0 / np.median(finite))
    meta = {
        "sample_count": int(n),
        "duration_s": float(t_s[-1]) if n else 0.0,
        "sample_rate_hz": sample_rate_hz,
        "raw_source": raw_source,
    }
    return stack, meta


def _lod_indices(raw: np.ndarray, channel_index: dict[str, int], step: int) -> np.ndarray:
    n = raw.shape[1]
    selected: set[int] = {0, n - 1}
    preserve = ["vel_kmh", "ax", "ay", "gz"]
    for start in range(0, n, step):
        end = min(n, start + step)
        selected.add(start)
        selected.add(end - 1)
        for name in preserve:
            values = raw[channel_index[name], start:end]
            finite = np.flatnonzero(np.isfinite(values))
            if finite.size == 0:
                continue
            finite_values = values[finite]
            selected.add(start + int(finite[np.argmin(finite_values)]))
            selected.add(start + int(finite[np.argmax(finite_values)]))
    return np.fromiter(sorted(selected), dtype=np.int64)


def _build_levels(raw: np.ndarray, session_dir: Path) -> list[dict[str, Any]]:
    levels_dir = session_dir / "levels"
    levels_dir.mkdir(parents=True, exist_ok=True)
    levels: list[dict[str, Any]] = []
    channel_index = {name: i for i, name in enumerate(CANONICAL_CHANNELS)}

    def write_level(indices: np.ndarray, step: int) -> None:
        level_name = f"l{len(levels)}"
        path = levels_dir / f"{level_name}.f32"
        _write_f32(path, raw[:, indices])
        levels.append(
            {
                "level": level_name,
                "sample_count": int(len(indices)),
                "step": int(step),
                "file": f"levels/{level_name}.f32",
            }
        )

    n = raw.shape[1]
    write_level(np.arange(n, dtype=np.int64), 1)
    previous_count = n
    step = 8
    while step < n:
        indices = _lod_indices(raw, channel_index, step)
        # Skip ineffective levels where peak preservation keeps nearly all samples.
        if len(indices) < previous_count * 0.92:
            write_level(indices, step)
            previous_count = len(indices)
        if previous_count <= 2_500:
            break
        step *= 2
    return levels


def _load_meta(session_dir: Path) -> dict[str, Any] | None:
    path = session_dir / "meta.json"
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def import_session(source_path: str | Path, source_name: str | None = None, force: bool = False) -> dict[str, Any]:
    source = Path(source_path)
    if not source.exists():
        raise FileNotFoundError(source)
    source_hash = _hash_file(source)
    session_id = source_hash[:16]
    source_name = source_name or source.name
    session_dir = CACHE_ROOT / session_id
    meta_path = session_dir / "meta.json"
    if not force:
        existing = _load_meta(session_dir)
        if existing and (session_dir / "raw.f32").exists():
            return existing

    work_dir = session_dir.with_name(session_dir.name + ".tmp")
    if work_dir.exists():
        shutil.rmtree(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)

    ext = source.suffix.lower()
    if ext == ".bin":
        arrays, read_meta = _read_bin_source(source)
    elif ext == ".csv":
        arrays, read_meta = _read_csv_source(source)
    else:
        raise ValueError(f"Unsupported telemetry file extension: {ext}")

    raw, canonical_meta = _canonicalize(arrays)
    _write_f32(work_dir / "raw.f32", raw)
    levels = _build_levels(raw, work_dir)

    meta = {
        "id": session_id,
        "schema": SCHEMA,
        "source_name": source_name,
        "source_hash": source_hash,
        "firmware_version": read_meta.get("firmware_version", "unknown"),
        "record_size": read_meta.get("record_size"),
        "channels": CANONICAL_CHANNELS,
        "levels": levels,
        **canonical_meta,
        "import": read_meta,
    }
    _json_write(work_dir / "meta.json", meta)

    if session_dir.exists():
        shutil.rmtree(session_dir)
    work_dir.replace(session_dir)
    return meta


def list_sessions() -> list[dict[str, Any]]:
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    sessions = []
    for child in sorted(CACHE_ROOT.iterdir()):
        if not child.is_dir():
            continue
        meta = _load_meta(child)
        if not meta:
            continue
        sessions.append(
            {
                "id": meta["id"],
                "source_name": meta.get("source_name", meta["id"]),
                "sample_count": meta.get("sample_count"),
                "duration_s": meta.get("duration_s"),
                "sample_rate_hz": meta.get("sample_rate_hz"),
                "raw_source": meta.get("raw_source"),
            }
        )
    return sessions


def _session_file(session_id: str, *parts: str) -> Path:
    root = (CACHE_ROOT / session_id).resolve()
    cache_root = CACHE_ROOT.resolve()
    if cache_root not in root.parents and root != cache_root:
        raise FileNotFoundError(session_id)
    path = root.joinpath(*parts).resolve()
    if root not in path.parents and path != root:
        raise FileNotFoundError(parts)
    return path


def create_app():
    if FastAPI is None:
        raise RuntimeError(
            "Missing server dependency. Run: pip install -r tools/requirements.txt"
        )

    app = FastAPI(title="Telemetria Local Server")
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)

    @app.get("/api/sessions")
    def api_sessions():
        return {"sessions": list_sessions()}

    @app.post("/api/sessions/import")
    async def api_import(file: UploadFile = File(...)):
        suffix = Path(file.filename or "upload.bin").suffix
        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(delete=False, suffix=suffix, dir=CACHE_ROOT) as tmp:
                tmp_path = Path(tmp.name)
                while True:
                    chunk = await file.read(1024 * 1024)
                    if not chunk:
                        break
                    tmp.write(chunk)
            meta = import_session(tmp_path, source_name=file.filename)
            return {"session": meta}
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        finally:
            if tmp_path and tmp_path.exists():
                tmp_path.unlink()

    @app.get("/api/sessions/{session_id}/meta")
    def api_meta(session_id: str):
        path = _session_file(session_id, "meta.json")
        if not path.exists():
            raise HTTPException(status_code=404, detail="Session not found")
        return FileResponse(path, media_type="application/json")

    @app.get("/api/sessions/{session_id}/raw.f32")
    def api_raw(session_id: str):
        path = _session_file(session_id, "raw.f32")
        if not path.exists():
            raise HTTPException(status_code=404, detail="Session data not found")
        return FileResponse(path, media_type="application/octet-stream")

    @app.get("/api/sessions/{session_id}/levels/{level}.f32")
    def api_level(session_id: str, level: str):
        path = _session_file(session_id, "levels", f"{level}.f32")
        if not path.exists():
            raise HTTPException(status_code=404, detail="Level not found")
        return FileResponse(path, media_type="application/octet-stream")

    @app.get("/colors_and_type.css")
    def colors_css():
        return FileResponse(DESIGN_ROOT / "colors_and_type.css", media_type="text/css")

    app.mount("/", StaticFiles(directory=UI_ROOT, html=True), name="telemetry-ui")
    return app


try:
    app = create_app()
except RuntimeError:
    app = None


def find_free_port(start: int = 8765, end: int = 8780) -> int:
    for port in range(start, end):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("127.0.0.1", port)) != 0:
                return port
    return start


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Telemetria local browser server")
    parser.add_argument("--import", dest="import_path", help="Import a .csv/.bin into the local cache and exit")
    parser.add_argument("--force", action="store_true", help="Rebuild cache even if session already exists")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0, help="Port, default: first free port from 8765")
    args = parser.parse_args(argv)

    if args.import_path:
        meta = import_session(args.import_path, force=args.force)
        print(json.dumps({"id": meta["id"], "source_name": meta["source_name"]}, indent=2))
        return

    if app is None:
        raise SystemExit("Missing server dependency. Run: pip install -r tools/requirements.txt")

    import uvicorn

    port = args.port or find_free_port()
    print(f"Telemetria local server: http://{args.host}:{port}")
    uvicorn.run(app, host=args.host, port=port)


if __name__ == "__main__":
    main()
