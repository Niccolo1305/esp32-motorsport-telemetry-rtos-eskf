# ──────────────────────────────────────────────────────────────────────────────
# Motorsport Telemetry Analysis Dashboard  v1.2.2
# ──────────────────────────────────────────────────────────────────────────────
# A high-performance, interactive telemetry viewer built with Dash and Plotly.
# Designed to visualize and analyze 200,000+ row CSV files from data-logger
# firmware running on ESP32-based hardware (IMU + GPS + Kalman Filter).
#
# Key capabilities:
#   - WebGL-accelerated rendering of all data points (zero downsampling)
#   - GPS track map with fused ESKF overlay
#   - G-G (friction circle) diagram in 2D and 3D
#   - Yaw / Roll / Grip rate overlay chart
#   - Velocity timeline with brake/accel/coast trace splitting
#   - Cross-graph cursor synchronization via Patch()
#   - 2-click sector selection with visual feedback
#
# Repository: 
# License:    
# ──────────────────────────────────────────────────────────────────────────────

from __future__ import annotations
import sys
import os
import numpy as np
import pandas as pd

from dash import (Dash, dcc, html, Input, Output, State,
                  callback, no_update, ctx, Patch)
import dash_bootstrap_components as dbc
import plotly.graph_objects as go
import socket

# ═══════════════════════════════════════════════════════════════════════════════
# 1.  INTERACTIVE CSV SELECTOR & DATA LOADING
# ═══════════════════════════════════════════════════════════════════════════════

# ── ANSI colour codes (for console output) ───────────────────────────────────
BOLD   = "\033[1m"
DIM    = "\033[2m"
CYAN   = "\033[36m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
RESET  = "\033[0m"


def _select_csv() -> str:
    """Interactive CSV file selector for the console.

    Resolution order:
      1. Command-line argument: ``python app.py myfile.csv``
      2. If exactly one CSV exists in the working directory → auto-select.
      3. Multiple CSVs found → display a numbered menu and prompt the user.
      4. No CSVs found → ask for a manual path.

    Returns:
        The filesystem path to the selected CSV file.
    """
    # 1. CLI argument
    if len(sys.argv) >= 2 and sys.argv[1].endswith(".csv"):
        path = sys.argv[1]
        if os.path.isfile(path):
            return path
        print(f"  {RED}File not found: {path}{RESET}")
        sys.exit(1)

    # 2–3. Scan working directory
    csvs = sorted(
        [f for f in os.listdir(".") if f.lower().endswith(".csv") and os.path.isfile(f)]
    )

    if not csvs:
        print(f"  {RED}No .csv files found in the current directory.{RESET}")
        path = input(f"  {CYAN}▸{RESET} Enter CSV path: ").strip()
        if not path or not os.path.isfile(path):
            print(f"  {RED}File not found.{RESET}")
            sys.exit(1)
        return path

    if len(csvs) == 1:
        print(f"  Auto-selected: {BOLD}{csvs[0]}{RESET}")
        return csvs[0]

    # Multiple CSVs → interactive menu
    print(f"\n  {BOLD}Available telemetry files:{RESET}")
    for i, f in enumerate(csvs, 1):
        size_mb = os.path.getsize(f) / (1024 * 1024)
        print(f"    {GREEN}{i}.{RESET} {f}  {DIM}({size_mb:.1f} MB){RESET}")
    print()

    while True:
        raw = input(f"  {CYAN}▸{RESET} Select file (1-{len(csvs)}): ").strip()
        try:
            choice = int(raw)
            if 1 <= choice <= len(csvs):
                selected = csvs[choice - 1]
                print(f"  Selected: {BOLD}{selected}{RESET}")
                return selected
        except ValueError:
            pass
        print(f"    {RED}Enter a number between 1 and {len(csvs)}.{RESET}")

# ── Column name mapping (legacy → canonical with units) ──────────────────────
# If the incoming CSV still uses legacy headers (v1.1 format without units),
# they are transparently renamed to the canonical format on load.  If the
# headers already match the canonical names, the rename is a no-op.
COL_MAP = {
    "t_us":          "t_us (µs)",
    "t_ms":          "t_ms (ms)",
    "ax":            "ax (G)",
    "ay":            "ay (G)",
    "az":            "az (G)",
    "gx":            "gx (°/s)",
    "gy":            "gy (°/s)",
    "gz":            "gz (°/s)",
    "temp_c":        "temp_c (°C)",
    "lap":           "lap",
    "gps_lat":       "gps_lat (°)",
    "gps_lon":       "gps_lon (°)",
    "gps_speed_kmh": "gps_speed_kmh (km/h)",
    "gps_alt_m":     "gps_alt_m (m)",
    "gps_sats":      "gps_sats",
    "gps_hdop":      "gps_hdop",
    "kf_x":          "kf_x (m)",
    "kf_y":          "kf_y (m)",
    "kf_vel":        "kf_vel (m/s)",
    "kf_heading":    "kf_heading (rad)",
    "raw_ax":        "raw_ax (G)",
    "raw_ay":        "raw_ay (G)",
    "raw_az":        "raw_az (G)",
    "raw_gx":        "raw_gx (°/s)",
    "raw_gy":        "raw_gy (°/s)",
    "raw_gz":        "raw_gz (°/s)",
    "kf6_x":         "kf6_x (m)",
    "kf6_y":         "kf6_y (m)",
    "kf6_vel":       "kf6_vel (m/s)",
    "kf6_heading":   "kf6_heading (rad)",
    "kf6_bgz":       "kf6_bgz (rad/s)",
    "sensor_ax":     "sensor_ax (G)",
    "sensor_ay":     "sensor_ay (G)",
    "sensor_az":     "sensor_az (G)",
    "sensor_gx":     "sensor_gx (°/s)",
    "sensor_gy":     "sensor_gy (°/s)",
    "sensor_gz":     "sensor_gz (°/s)",
}

REQUIRED_COLS = [
    "t_ms (ms)", "kf_vel (m/s)", "kf_x (m)", "kf_y (m)",
    "gps_lat (°)", "gps_lon (°)", "ax (G)", "ay (G)", "az (G)",
    "gz (°/s)", "gx (°/s)", "raw_ax (G)", "raw_ay (G)", "raw_az (G)",
    "raw_gz (°/s)", "raw_gx (°/s)",
]


def load_and_prepare(path: str) -> pd.DataFrame:
    """Load a telemetry CSV and compute all derived columns.

    Performs the following operations in order:
      1. Renames legacy column headers to the canonical format (with units).
      2. Computes elapsed time in seconds (``t_s``).
      3. Integrates ``abs(velocity)`` over time to produce a monotonic
         ``distance_m`` column (avoids gaps from negative velocity readings).
      4. Creates short-name acceleration aliases (``ax_g``, ``ay_g``, …) used
         throughout the figure builders.
      5. Computes 2-D acceleration magnitude for the Grip overlay.
      6. Converts Kalman-filter ENU coordinates to WGS-84 for map display.

    Args:
        path: Filesystem path to the CSV file.

    Returns:
        A fully augmented ``pd.DataFrame`` ready for visualization.

    Raises:
        FileNotFoundError: If *path* does not exist.
        KeyError: If required columns are missing after the rename step.
    """
    try:
        df = pd.read_csv(path, encoding='utf-8')
    except UnicodeDecodeError:
        df = pd.read_csv(path, encoding='latin1')

    # Rename legacy column names to the canonical format (idempotent)
    existing_old = {k: v for k, v in COL_MAP.items()
                    if k in df.columns and v not in df.columns}
    if existing_old:
        df.rename(columns=existing_old, inplace=True)

    # Normalize timestamp column: v1.4.0 uses t_us, older uses t_ms
    if "t_us (µs)" in df.columns and "t_ms (ms)" not in df.columns:
        df["t_ms (ms)"] = df["t_us (µs)"] / 1000.0  # backwards-compat alias

    missing = [c for c in REQUIRED_COLS if c not in df.columns]
    if missing:
        raise KeyError(
            f"Missing columns — wrong CSV format or firmware version < v0.9?\n"
            f"  Missing: {missing}"
        )

    # Elapsed time in seconds (origin-normalized)
    if "t_us (µs)" in df.columns:
        df["t_s"] = (df["t_us (µs)"] - df["t_us (µs)"].iloc[0]) / 1_000_000.0
    else:
        df["t_s"] = (df["t_ms (ms)"] - df["t_ms (ms)"].iloc[0]) / 1000.0

    # Cumulative distance via abs(velocity) integration (gap-free)
    dt = np.diff(df["t_s"].values, prepend=df["t_s"].values[0])
    dt[0] = 0.0
    df["distance_m"] = np.cumsum(np.abs(df["kf_vel (m/s)"].values) * dt)

    # Short-name acceleration aliases for internal use
    df["ax_g"]      = df["ax (G)"]
    df["ay_g"]      = df["ay (G)"]
    df["az_g"]      = df["az (G)"]
    df["raw_ax_g"]  = df["raw_ax (G)"]
    df["raw_ay_g"]  = df["raw_ay (G)"]
    df["raw_az_g"]  = df["raw_az (G)"]

    # 2-D acceleration magnitude (lateral + longitudinal) → Grip metric
    df["acc_mag"]     = np.sqrt(df["ax_g"] ** 2 + df["ay_g"] ** 2)
    df["raw_acc_mag"] = np.sqrt(df["raw_ax_g"] ** 2 + df["raw_ay_g"] ** 2)

    # Velocity in km/h (from Kalman-filter m/s output)
    df["vel_kmh"] = df["kf_vel (m/s)"] * 3.6

    # Gyroscope aliases for Yaw Rate / Roll Rate builders
    df["gz_filt"]  = df["gz (°/s)"]
    df["gx_filt"]  = df["gx (°/s)"]
    df["gz_raw"]   = df["raw_gz (°/s)"]
    df["gx_raw"]   = df["raw_gx (°/s)"]

    # Convert Kalman-filter ENU (x, y) → WGS-84 (lat, lon)
    df = _convert_kf_to_wgs84(df)
    df.reset_index(drop=True, inplace=True)
    return df


def _convert_kf_to_wgs84(df: pd.DataFrame) -> pd.DataFrame:
    """Project Kalman-filter (x, y) ENU metres back to WGS-84 latitude/longitude.

    Uses the same equirectangular (flat-earth) projection that the firmware
    uses in ``wgs84_to_enu()`` (eskf.h) so that the inverse transform is
    exact.  The firmware forward transform is:
        east  = dlon · cos(lat0) · R
        north = dlat · R
    so the inverse is:
        lat = lat0 + north / R
        lon = lon0 + east / (R · cos(lat0))

    The firmware ENU origin (lat0, lon0) is recovered from the first row that
    has a valid GPS fix by subtracting its kf_x/kf_y offset.  This makes the
    conversion correct even for part-files that start mid-session, where the
    first GPS coordinate is far from the original firmware origin.

    Args:
        df: DataFrame containing ``gps_lat (°)``, ``gps_lon (°)``,
            ``kf_x (m)`` and ``kf_y (m)`` columns.

    Returns:
        The same DataFrame with ``kf_lat`` and ``kf_lon`` columns appended.
    """
    valid = df[(df["gps_lat (°)"] != 0) & (df["gps_lon (°)"] != 0)]
    if valid.empty:
        df["kf_lat"] = 0.0; df["kf_lon"] = 0.0
        return df
    R = 6_371_000.0
    # Recover the firmware ENU origin: the GPS coordinate of this row minus
    # the ENU offset the firmware computed for it (kf_x, kf_y).
    row0 = valid.iloc[0]
    gps_lat0 = row0["gps_lat (°)"]
    gps_lon0 = row0["gps_lon (°)"]
    cos_lat_approx = np.cos(np.radians(gps_lat0))
    lat0 = gps_lat0 - np.degrees(row0["kf_y (m)"] / R)
    lon0 = gps_lon0 - np.degrees(row0["kf_x (m)"] / (R * cos_lat_approx))
    cos_lat0 = np.cos(np.radians(lat0))
    df["kf_lat"] = lat0 + np.degrees(df["kf_y (m)"].values / R)
    df["kf_lon"] = lon0 + np.degrees(df["kf_x (m)"].values / (R * cos_lat0))
    return df


print(f"\n{BOLD}╔══════════════════════════════════════════════════════╗{RESET}")
print(f"{BOLD}║       TELEMETRY DASHBOARD v1.2.2 — File Selector     ║{RESET}")
print(f"{BOLD}╚══════════════════════════════════════════════════════╝{RESET}\n")

CSV_PATH = _select_csv()

print(f"\n⏳  Loading {BOLD}{CSV_PATH}{RESET} …")
try:
    DF = load_and_prepare(CSV_PATH)
except FileNotFoundError:
    print(f"❌  CSV file not found: {CSV_PATH}")
    print("   Place a telemetry CSV in the working directory and update CSV_PATH.")
    sys.exit(1)
except KeyError as exc:
    print(f"❌  Missing required column in CSV: {exc}")
    sys.exit(1)

N = len(DF)
print(f"✅  {N:,} samples loaded — {DF['t_s'].iloc[-1]:.1f} s of data")

# ═══════════════════════════════════════════════════════════════════════════════
# 2.  CONSTANTS & THEME
# ═══════════════════════════════════════════════════════════════════════════════

ACCENT     = "#00e5ff"
BG_CARD    = "#1a1a2e"
BG_MAIN    = "#0f0e17"
BG_PLOT    = "#16213e"
GRID_COLOR = "#1e3a5f"
TEXT_COLOR = "#e0e0e0"
TEXT_MUTED = "#8892b0"
BRAKE_COL  = "#ff1744"
ACCEL_COL  = "#00e676"
COAST_COL  = "#546e7a"
HOVER_COL  = "#00e5ff"
CLICK_COL  = "#ffea00"
COLORSCALE = "Turbo"

CARD_S  = {"backgroundColor": BG_CARD, "border": f"1px solid {GRID_COLOR}",
           "borderRadius": "12px", "padding": "14px 18px", "textAlign": "center"}
GRAPH_S = {"backgroundColor": BG_CARD, "border": f"1px solid {GRID_COLOR}",
           "borderRadius": "12px", "padding": "6px"}

# Fixed cursor-trace indices per graph (layout is locked — no graph swapping).
# Map:      GPS(0) ESKF(1)  hover(2) click(3)
# G-G:      scatter(0)      hover(1) click(2)
# Yaw:      yaw(0) roll(1) grip(2)  hover(3) click(4)
# Timeline: brake(0) accel(1) coast(2) hover(3) click(4)
CUR_MAP  = (2, 3)
CUR_GG   = (1, 2)
CUR_YAW  = (3, 4)
CUR_TL   = (3, 4)

# Longitudinal acceleration thresholds for trace-splitting (G)
BRAKE_TH = -0.002
ACCEL_TH =  0.002


# ═══════════════════════════════════════════════════════════════════════════════
# 3.  HELPERS — cursor factories, base layout, customdata packing
# ═══════════════════════════════════════════════════════════════════════════════

def _base(**kw):
    """Return a dark-themed Plotly layout dict with sensible defaults.

    Keyword arguments are merged on top, allowing per-graph overrides.
    """
    lo = dict(paper_bgcolor=BG_PLOT, plot_bgcolor=BG_PLOT,
              font=dict(color=TEXT_COLOR, family="Inter,sans-serif", size=11),
              margin=dict(l=48, r=15, t=35, b=40),
              xaxis=dict(gridcolor=GRID_COLOR, zerolinecolor=GRID_COLOR),
              yaxis=dict(gridcolor=GRID_COLOR, zerolinecolor=GRID_COLOR),
              legend=dict(bgcolor="rgba(0,0,0,0.3)", font=dict(size=10)),
              dragmode="zoom",
              uirevision="stable")
    lo.update(kw)
    return lo


def _cmap(name, color, sz=14):
    """Create a sentinel Scattermap trace used as a cross-graph cursor."""
    return go.Scattermap(lat=[None], lon=[None], mode="markers",
                         marker=dict(size=sz, color=color, opacity=0.95),
                         name=name, showlegend=False, hoverinfo="skip")


def _cxy(name, color, sz=12):
    """Create a sentinel Scattergl trace used as a cross-graph cursor."""
    return go.Scattergl(x=[None], y=[None], mode="markers",
                        marker=dict(size=sz, color=color, symbol="diamond",
                                    line=dict(width=2, color="white")),
                        name=name, showlegend=False, hoverinfo="skip")


def _c3d(name, color, sz=8):
    """Create a sentinel Scatter3d trace used as a cross-graph cursor."""
    return go.Scatter3d(x=[None], y=[None], z=[None], mode="markers",
                        marker=dict(size=sz, color=color, symbol="diamond"),
                        name=name, showlegend=False, hoverinfo="skip")


def _cd4(sub):
    """Pack per-point metadata into a vectorized customdata array.

    Returns an (N, 4) float64 array with columns:
      [row_index, elapsed_time_s, cumulative_distance_m, speed_kmh]
    """
    return np.column_stack([
        sub.index.values.astype(np.float64),
        sub["t_s"].values,
        sub["distance_m"].values,
        sub["vel_kmh"].values,
    ])


# ═══════════════════════════════════════════════════════════════════════════════
# 4.  FIGURE BUILDERS
# ═══════════════════════════════════════════════════════════════════════════════

# ── 4a  GPS Track Map ─────────────────────────────────────────────────────────

def build_map(sub: pd.DataFrame):
    """Build the GPS track map with raw GPS scatter and fused ESKF line.

    Trace order (fixed):
      0 — GPS scatter (colour-coded by speed)
      1 — ESKF fused trajectory line
      2 — hover cursor sentinel
      3 — click cursor sentinel

    The map auto-centres and auto-zooms on the available position data.

    Args:
        sub: Sector-filtered DataFrame slice.

    Returns:
        A ``plotly.graph_objects.Figure``.
    """
    fig = go.Figure()
    vg = sub[(sub["gps_lat (°)"] != 0) & (sub["gps_lon (°)"] != 0)]
    vk = sub[(sub["kf_lat"] != 0) & (sub["kf_lon"] != 0)]

    if vg.empty and vk.empty:
        fig.update_layout(map=dict(style="dark", center=dict(lat=45, lon=8), zoom=5),
                          paper_bgcolor=BG_PLOT, margin=dict(l=0, r=0, t=30, b=0),
                          title=dict(text="GPS Map – No data", font=dict(size=13, color=TEXT_COLOR)))
        fig.add_trace(go.Scattermap(lat=[], lon=[], mode="markers", name="GPS"))
        fig.add_trace(go.Scattermap(lat=[], lon=[], mode="lines", name="ESKF"))
        fig.add_trace(_cmap("hov", HOVER_COL))
        fig.add_trace(_cmap("clk", CLICK_COL, 16))
        return fig

    spd_min = vg["vel_kmh"].min() if not vg.empty else 0
    spd_max = vg["vel_kmh"].max() if not vg.empty else 1

    # Trace 0: GPS scatter — all points, speed shown in tooltip
    cd_gps = _cd4(vg)
    fig.add_trace(go.Scattermap(
        lat=vg["gps_lat (°)"].values, lon=vg["gps_lon (°)"].values, mode="markers",
        marker=dict(size=3, color=vg["vel_kmh"].values, opacity=0.6,
                    colorscale=COLORSCALE, cmin=spd_min, cmax=spd_max,
                    colorbar=dict(title="km/h", len=0.5, thickness=12, x=1.02)),
        customdata=cd_gps,
        hovertemplate=(
            "<b>%{customdata[3]:.1f} km/h</b><br>"
            "t=%{customdata[1]:.1f}s  d=%{customdata[2]:.0f}m"
            "<extra>GPS</extra>"
        ),
        name="GPS",
    ))

    # Trace 1: ESKF fused trajectory
    if not vk.empty:
        fig.add_trace(go.Scattermap(
            lat=vk["kf_lat"].values, lon=vk["kf_lon"].values,
            mode="lines", line=dict(color="#ff1744", width=2.5),
            name="ESKF", hoverinfo="skip",
        ))
    else:
        fig.add_trace(go.Scattermap(lat=[], lon=[], mode="lines",
                                    name="ESKF", showlegend=False))

    # Traces 2–3: cursor sentinels
    fig.add_trace(_cmap("hov", HOVER_COL))
    fig.add_trace(_cmap("clk", CLICK_COL, 16))

    # Auto-centre and auto-zoom
    ref = vk if not vk.empty else vg
    lat_col = "kf_lat" if not vk.empty else "gps_lat (°)"
    lon_col = "kf_lon" if not vk.empty else "gps_lon (°)"
    lat_c, lon_c = ref[lat_col].median(), ref[lon_col].median()
    span = max(ref[lat_col].max() - ref[lat_col].min(),
               ref[lon_col].max() - ref[lon_col].min(), 0.0001)
    zoom = max(1, min(18, int(np.log2(360 / span)) - 1))

    fig.update_layout(
        map=dict(style="dark", center=dict(lat=lat_c, lon=lon_c), zoom=zoom),
        paper_bgcolor=BG_PLOT,
        font=dict(color=TEXT_COLOR, family="Inter,sans-serif", size=11),
        margin=dict(l=0, r=0, t=30, b=0),
        title=dict(text="GPS Map", font=dict(size=13)),
        showlegend=True,
        legend=dict(bgcolor="rgba(0,0,0,0.4)", font=dict(size=10, color=TEXT_COLOR)),
        uirevision="map",
    )
    return fig


# ── 4b  G-G Diagram ──────────────────────────────────────────────────────────

def build_gg(sub: pd.DataFrame, use_raw: bool, use_3d: bool,
             fixed_range: float | None = None):
    """Build a G-G (friction circle) scatter plot in 2-D or 3-D.

    Points are colour-coded by total acceleration magnitude using the Turbo
    colourscale.  When *fixed_range* is supplied the axes are locked to
    ``[-fixed_range, +fixed_range]`` regardless of sector data, enabling
    consistent visual comparison across sectors.

    Trace order (fixed):
      0 — G-G scatter
      1 — hover cursor sentinel
      2 — click cursor sentinel

    Args:
        sub:         Sector-filtered DataFrame slice.
        use_raw:     If ``True``, use unfiltered accelerometer data.
        use_3d:      If ``True``, render a Scatter3d with vertical-axis Az.
        fixed_range: Optional symmetric axis limit (G).  ``None`` → auto-fit.

    Returns:
        A ``plotly.graph_objects.Figure``.
    """
    ax_c = "raw_ax_g" if use_raw else "ax_g"
    ay_c = "raw_ay_g" if use_raw else "ay_g"
    mag_c = "raw_acc_mag" if use_raw else "acc_mag"
    fig = go.Figure()

    if sub.empty:
        if use_3d:
            fig.add_trace(go.Scatter3d(x=[], y=[], z=[], mode="markers", name="G-G"))
            fig.add_trace(_c3d("hov", HOVER_COL)); fig.add_trace(_c3d("clk", CLICK_COL))
        else:
            fig.add_trace(go.Scattergl(x=[], y=[], mode="markers", name="G-G"))
            fig.add_trace(_cxy("hov", HOVER_COL)); fig.add_trace(_cxy("clk", CLICK_COL))
        fig.update_layout(**_base(title=dict(text="G-G – No data")))
        return fig

    mag_min, mag_max = sub[mag_c].min(), sub[mag_c].max()
    cd = _cd4(sub)

    if use_3d:
        az_c = "raw_az_g" if use_raw else "az_g"
        fig.add_trace(go.Scatter3d(
            x=sub[ay_c].values, y=sub[ax_c].values, z=sub[az_c].values,
            mode="markers",
            marker=dict(size=2, color=sub[mag_c].values, colorscale=COLORSCALE,
                        cmin=mag_min, cmax=mag_max, opacity=0.5,
                        colorbar=dict(title="G", len=0.5, thickness=12)),
            customdata=cd,
            hovertemplate=(
                "Ay=%{x:.3f}G Ax=%{y:.3f}G Az=%{z:.3f}G<br>"
                "t=%{customdata[1]:.1f}s d=%{customdata[2]:.0f}m<extra></extra>"
            ),
            name="G-G",
        ))
        fig.add_trace(_c3d("hov", HOVER_COL)); fig.add_trace(_c3d("clk", CLICK_COL))
        scene_kw = dict(
            xaxis=dict(title="Ay (G)", gridcolor=GRID_COLOR, backgroundcolor=BG_PLOT),
            yaxis=dict(title="Ax (G)", gridcolor=GRID_COLOR, backgroundcolor=BG_PLOT),
            zaxis=dict(title="Az (G)", gridcolor=GRID_COLOR, backgroundcolor=BG_PLOT,
                       autorange="reversed"),
            bgcolor=BG_PLOT, aspectmode="cube",
        )
        if fixed_range is not None:
            fr = fixed_range
            scene_kw["xaxis"]["range"] = [-fr, fr]
            scene_kw["yaxis"]["range"] = [-fr, fr]
        fig.update_layout(
            scene=scene_kw,
            paper_bgcolor=BG_PLOT,
            font=dict(color=TEXT_COLOR, family="Inter,sans-serif", size=11),
            margin=dict(l=0, r=0, t=35, b=0),
            title=dict(text="G-G-Gz 3D" + (" [Fixed]" if fixed_range else ""),
                       font=dict(size=13)),
            uirevision="gg",
        )
    else:
        fig.add_trace(go.Scattergl(
            x=sub[ay_c].values, y=sub[ax_c].values, mode="markers",
            marker=dict(size=2, color=sub[mag_c].values, colorscale=COLORSCALE,
                        cmin=mag_min, cmax=mag_max, opacity=0.5,
                        colorbar=dict(title="G", len=0.5, thickness=12)),
            customdata=cd,
            hovertemplate=(
                "Ay=%{x:.3f}G Ax=%{y:.3f}G<br>"
                "t=%{customdata[1]:.1f}s d=%{customdata[2]:.0f}m<extra></extra>"
            ),
            name="G-G",
        ))
        fig.add_trace(_cxy("hov", HOVER_COL)); fig.add_trace(_cxy("clk", CLICK_COL))
        if fixed_range is not None:
            mr = fixed_range
        else:
            mr = max(sub[ay_c].abs().max(), sub[ax_c].abs().max(), 0.01) * 1.15
        fig.update_layout(**_base(
            title=dict(text="G-G Diagram" + (" [Fixed]" if fixed_range else ""),
                       font=dict(size=13)),
            xaxis=dict(title="Ay – Lat (G)", range=[-mr, mr], gridcolor=GRID_COLOR,
                       scaleanchor="y", scaleratio=1),
            yaxis=dict(title="Ax – Long (G)", range=[-mr, mr], gridcolor=GRID_COLOR),
            uirevision="gg",
        ))
    return fig


# ── 4c  Yaw Rate + Roll / Grip overlays ──────────────────────────────────────

def build_yaw(sub: pd.DataFrame, use_raw: bool, use_distance: bool,
              show_roll: bool, show_grip: bool, xrange=None):
    """Build the Yaw Rate line chart with optional Roll Rate and Grip overlays.

    Up to three Y-axes are managed:
      - Primary (left):  Yaw rate (°/s)
      - Secondary (right, y2): Roll rate (°/s) — orange
      - Tertiary  (right, y3): Grip magnitude (G) — green

    Only the primary axis renders grid lines; y2 and y3 always have
    ``showgrid=False`` and ``zeroline=False`` to prevent visual clutter.

    Trace order (fixed):
      0 — Yaw rate
      1 — Roll rate overlay (or empty placeholder)
      2 — Grip overlay (or empty placeholder)
      3 — hover cursor sentinel
      4 — click cursor sentinel

    Args:
        sub:          Sector-filtered DataFrame slice.
        use_raw:      Use unfiltered gyroscope data.
        use_distance: X-axis in metres instead of seconds.
        show_roll:    Toggle Roll Rate overlay visibility.
        show_grip:    Toggle Grip (acc_mag) overlay visibility.
        xrange:       Optional ``[x_min, x_max]`` to lock the X-axis.

    Returns:
        A ``plotly.graph_objects.Figure``.
    """
    gz_c = "gz_raw" if use_raw else "gz_filt"
    x_col = "distance_m" if use_distance else "t_s"
    x_tit = "Distance (m)" if use_distance else "Time (s)"
    fig = go.Figure()

    if sub.empty:
        fig.add_trace(go.Scattergl(x=[], y=[], name="Yaw"))
        fig.add_trace(go.Scattergl(x=[], y=[], name="Roll", yaxis="y2"))
        fig.add_trace(go.Scattergl(x=[], y=[], name="Grip", yaxis="y3"))
        fig.add_trace(_cxy("hov", HOVER_COL)); fig.add_trace(_cxy("clk", CLICK_COL))
        fig.update_layout(**_base(title=dict(text="Yaw – No data")))
        return fig

    cd = _cd4(sub)

    # Trace 0: Yaw Rate
    fig.add_trace(go.Scattergl(
        x=sub[x_col].values, y=sub[gz_c].values, mode="lines",
        line=dict(color="#7c4dff", width=1), name="Yaw Rate",
        customdata=cd,
        hovertemplate=(
            "gz=%{y:.2f}°/s<br>"
            "t=%{customdata[1]:.1f}s d=%{customdata[2]:.0f}m<extra></extra>"
        ),
    ))

    # Trace 1: Roll Rate overlay (always present; may be empty placeholder)
    gx_c = "gx_raw" if use_raw else "gx_filt"
    if show_roll:
        fig.add_trace(go.Scattergl(
            x=sub[x_col].values, y=sub[gx_c].values, mode="lines",
            line=dict(color="#ff9800", width=1), name="Roll Rate",
            yaxis="y2", customdata=cd,
            hovertemplate="gx=%{y:.2f}°/s<extra></extra>",
        ))
    else:
        fig.add_trace(go.Scattergl(x=[], y=[], visible=False,
                                   name="Roll Rate", yaxis="y2"))

    # Trace 2: Grip overlay (always present; may be empty placeholder)
    mag_c = "raw_acc_mag" if use_raw else "acc_mag"
    if show_grip:
        fig.add_trace(go.Scattergl(
            x=sub[x_col].values, y=sub[mag_c].values, mode="lines",
            line=dict(color="#00e676", width=1), name="Grip (G)",
            yaxis="y3", customdata=cd,
            hovertemplate="Grip=%{y:.4f}G<extra></extra>",
        ))
    else:
        fig.add_trace(go.Scattergl(x=[], y=[], visible=False,
                                   name="Grip (G)", yaxis="y3"))

    # Traces 3–4: cursor sentinels
    fig.add_trace(_cxy("hov", HOVER_COL)); fig.add_trace(_cxy("clk", CLICK_COL))

    lo = _base(
        title=dict(text="Yaw Rate (gz)", font=dict(size=13)),
        xaxis=dict(title=x_tit, gridcolor=GRID_COLOR),
        yaxis=dict(title="°/s (yaw)", gridcolor=GRID_COLOR, autorange=True),
        uirevision="yaw",
    )

    # y2 and y3 are ALWAYS defined with showgrid=False / zeroline=False
    # to prevent ghost grid lines even when the overlay is disabled.
    lo["yaxis2"] = dict(
        title=dict(text="°/s (roll)", font=dict(color="#ff9800")) if show_roll else None,
        overlaying="y", side="right",
        showgrid=False, zeroline=False, showticklabels=show_roll,
        tickfont=dict(color="#ff9800"),
        autorange=True,
        visible=show_roll,
    )
    lo["yaxis3"] = dict(
        title=dict(text="Grip (G)", font=dict(color="#00e676")) if show_grip else None,
        overlaying="y", side="right",
        showgrid=False, zeroline=False, showticklabels=show_grip,
        tickfont=dict(color="#00e676"),
        autorange=True,
        anchor="free", autoshift=True,
        visible=show_grip,
    )

    # Inherit sector X-range exactly
    if xrange is not None:
        lo["xaxis"]["range"] = list(xrange)
    fig.update_layout(**lo)
    return fig


# ── 4d  Velocity Timeline (trace-split + 2-click sector selection) ────────────

def build_timeline(full_df: pd.DataFrame, use_raw: bool, use_distance: bool,
                   xrange=None, sel_start=None, sel_end=None):
    """Build the velocity timeline with brake/accel/coast trace splitting.

    The longitudinal acceleration column is thresholded to split data into
    three separate ``Scattergl`` traces (red=brake, green=accel, grey=coast),
    avoiding the need to pass a 200k-element colour array to a single trace.

    If sector selection anchors are set, dashed vertical lines and a shaded
    region are drawn via Plotly shapes.

    Trace order (fixed):
      0 — Brake points
      1 — Accel points
      2 — Coast points
      3 — hover cursor sentinel
      4 — click cursor sentinel

    Args:
        full_df:      The **full** (unfiltered) DataFrame — the timeline
                      always shows the complete session.
        use_raw:      Use unfiltered accelerometer data for thresholding.
        use_distance: X-axis in metres instead of seconds.
        xrange:       Optional ``[x_min, x_max]`` to set the visible window.
        sel_start:    X-value of the first sector-selection click.
        sel_end:      X-value of the second sector-selection click.

    Returns:
        A ``plotly.graph_objects.Figure``.
    """
    ax_c = "raw_ax_g" if use_raw else "ax_g"
    x_col = "distance_m" if use_distance else "t_s"
    x_tit = "Distance (m)" if use_distance else "Time (s)"
    fig = go.Figure()

    if full_df.empty:
        for _ in range(3):
            fig.add_trace(go.Scattergl(x=[], y=[], mode="markers"))
        fig.add_trace(_cxy("hov", HOVER_COL)); fig.add_trace(_cxy("clk", CLICK_COL))
        fig.update_layout(**_base(title="Timeline – No data", height=220))
        return fig

    vel = full_df["vel_kmh"].values
    ax_vals = full_df[ax_c].values
    cd_all = _cd4(full_df)

    m_brake = ax_vals < BRAKE_TH
    m_accel = ax_vals > ACCEL_TH
    m_coast = ~m_brake & ~m_accel

    ht = "<b>%{y:.1f} km/h</b><br>t=%{customdata[1]:.1f}s d=%{customdata[2]:.0f}m<extra></extra>"

    for mask, color, name in [(m_brake, BRAKE_COL, "Brake"),
                              (m_accel, ACCEL_COL, "Accel"),
                              (m_coast, COAST_COL, "Coast")]:
        xs = full_df[x_col].values[mask]
        ys = vel[mask]
        cd = cd_all[mask]
        fig.add_trace(go.Scattergl(
            x=xs, y=ys, mode="markers",
            marker=dict(size=2, color=color, opacity=0.6),
            customdata=cd, hovertemplate=ht, name=name,
        ))

    # Traces 3–4: cursor sentinels
    fig.add_trace(_cxy("hov", HOVER_COL)); fig.add_trace(_cxy("clk", CLICK_COL))

    lo = _base(
        title=dict(text="Velocity Timeline", font=dict(size=13)),
        xaxis=dict(title=x_tit, gridcolor=GRID_COLOR),
        yaxis=dict(title="Speed (km/h)", gridcolor=GRID_COLOR, autorange=True),
        height=220, uirevision="timeline",
        hovermode="x unified",  # tooltip on the perpendicular
    )
    if xrange is not None:
        lo["xaxis"]["range"] = list(xrange)

    # Sector-selection visual feedback (vertical lines + shaded region)
    shapes = []
    if sel_start is not None:
        shapes.append(dict(type="line", x0=sel_start, x1=sel_start,
                           y0=0, y1=1, yref="paper",
                           line=dict(color="#00e5ff", width=2, dash="dash")))
    if sel_end is not None:
        shapes.append(dict(type="line", x0=sel_end, x1=sel_end,
                           y0=0, y1=1, yref="paper",
                           line=dict(color="#ffea00", width=2, dash="dash")))
    if sel_start is not None and sel_end is not None:
        x0, x1 = min(sel_start, sel_end), max(sel_start, sel_end)
        shapes.append(dict(type="rect", x0=x0, x1=x1, y0=0, y1=1,
                           yref="paper", fillcolor="rgba(0,229,255,0.08)",
                           line=dict(width=0)))
    lo["shapes"] = shapes

    fig.update_layout(**lo)
    return fig


# ═══════════════════════════════════════════════════════════════════════════════
# 5.  DASH APPLICATION & LAYOUT
# ═══════════════════════════════════════════════════════════════════════════════

app = Dash(__name__,
           external_stylesheets=[
               dbc.themes.VAPOR,
               "https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap",
           ],
           title="Telemetry Dashboard v1.2.2",
           update_title=None)


def _kpi(cid, icon, label):
    """Return a KPI card component for the header row."""
    return dbc.Col(html.Div([
        html.Div(icon, style={"fontSize": "22px"}),
        html.Div(label, style={"color": TEXT_MUTED, "fontSize": "11px",
                                "textTransform": "uppercase", "letterSpacing": "1px"}),
        html.Div(id=cid, style={"fontSize": "26px", "fontWeight": "700",
                                 "color": ACCENT}),
    ], style=CARD_S), md=4, xs=12)


def _graph(gid, h="440px"):
    """Return a configured dcc.Graph wrapper with standard modebar settings."""
    return dcc.Graph(id=gid, style={"height": h},
                     config={"scrollZoom": True, "displayModeBar": "hover",
                             "modeBarButtonsToRemove": ["lasso2d", "select2d",
                                 "autoScale2d", "toggleSpikelines"]})

app.layout = html.Div(style={
    "backgroundColor": BG_MAIN, "minHeight": "100vh",
    "fontFamily": "Inter,sans-serif", "color": TEXT_COLOR,
    "padding": "16px 24px",
}, children=[
    # ── Header ────────────────────────────────────────────────────────────
    html.H1(["Telemetry Dashboard ",
             html.Span("v1.2.2", style={
                 "fontSize": "14px", "fontWeight": "400", "color": ACCENT,
                 "verticalAlign": "super", "marginLeft": "8px",
                 "border": f"1px solid {ACCENT}", "borderRadius": "6px",
                 "padding": "2px 8px"})],
            style={"fontSize": "28px", "fontWeight": "700", "marginBottom": "12px"}),
    dbc.Row([
        _kpi("kpi-vmax", "🏎️", "Max Speed (km/h)"),
        _kpi("kpi-lat",  "⟵⟶", "Max Lat Acc (G)"),
        _kpi("kpi-long", "⬆⬇", "Max Long Acc (G)"),
    ], className="g-3", style={"marginBottom": "14px"}),

    # ── Controls ──────────────────────────────────────────────────────────
    html.Div(style={**CARD_S, "textAlign": "left", "marginBottom": "14px",
                    "display": "flex", "flexWrap": "wrap",
                    "alignItems": "center", "gap": "20px"}, children=[
        dbc.Switch(id="sw-raw",    label="Raw Data",       value=False),
        dbc.Switch(id="sw-3d",     label="3D G-G",         value=False),
        dbc.Switch(id="sw-xaxis",  label="X-Axis: Distance", value=False),
        dbc.Switch(id="sw-roll",   label="Show Roll Rate", value=False),
        dbc.Switch(id="sw-grip",   label="Show Grip",      value=False),
        dbc.Switch(id="sw-gg-fix", label="G-G Fixed Scale", value=False),
        html.Div(style={"borderLeft": f"1px solid {GRID_COLOR}", "paddingLeft": "16px",
                         "display": "flex", "alignItems": "center", "gap": "8px"}, children=[
            html.Span("Sector:", style={"fontWeight": "600", "fontSize": "12px"}),
            dcc.Input(id="in-start", type="number", placeholder="Start",
                      style={"width": "80px", "backgroundColor": BG_CARD, "color": "#fff",
                             "border": f"1px solid {GRID_COLOR}", "borderRadius": "6px",
                             "padding": "4px 8px", "fontSize": "12px"}),
            dcc.Input(id="in-end", type="number", placeholder="End",
                      style={"width": "80px", "backgroundColor": BG_CARD, "color": "#fff",
                             "border": f"1px solid {GRID_COLOR}", "borderRadius": "6px",
                             "padding": "4px 8px", "fontSize": "12px"}),
            dbc.Button("Apply Sector", id="btn-zoom", color="info", size="sm", outline=True),
            dbc.Button("Reset", id="btn-reset", color="secondary", size="sm", outline=True,
                       style={"marginLeft": "4px"}),
        ]),
        html.Div(style={"borderLeft": f"1px solid {GRID_COLOR}", "paddingLeft": "16px"}, children=[
            html.Span("💡 Click on the Timeline to set [Start] and [End]",
                       style={"fontSize": "11px", "color": TEXT_MUTED, "fontStyle": "italic"}),
        ]),
    ]),

    # ── Top row: Map, G-G, Yaw ───────────────────────────────────────────
    dbc.Row([
        dbc.Col(html.Div([_graph("graph-map")], style=GRAPH_S), lg=4, md=12),
        dbc.Col(html.Div([_graph("graph-gg")],  style=GRAPH_S), lg=4, md=6),
        dbc.Col(html.Div([_graph("graph-yaw")], style=GRAPH_S), lg=4, md=6),
    ], className="g-2"),

    # ── Bottom row: Timeline (full width) ─────────────────────────────────
    html.Div([
        html.Div([_graph("graph-tl", "220px")], style=GRAPH_S),
    ], style={"marginTop": "6px"}),

    # ── Client-side state stores ──────────────────────────────────────────
    dcc.Store(id="store-xrange", data=None),
    dcc.Store(id="store-sel", data={"start": None, "end": None}),
])


# ═══════════════════════════════════════════════════════════════════════════════
# 6.  CALLBACK — 2-click sector selection on the Timeline
# ═══════════════════════════════════════════════════════════════════════════════

@callback(
    Output("store-sel", "data"),
    Output("in-start", "value"),
    Output("in-end", "value"),
    Input("graph-tl", "clickData"),
    Input("btn-reset", "n_clicks"),
    State("store-sel", "data"),
    State("sw-xaxis", "value"),
    prevent_initial_call=True,
)
def sector_click(click_data, _reset, sel, use_dist):
    """Handle 2-click sector selection on the timeline.

    First click → sets the Start anchor.  Second click → sets the End anchor
    and automatically orders them so Start < End.  The Reset button clears
    both anchors.
    """
    trig = ctx.triggered_id

    if trig == "btn-reset":
        return {"start": None, "end": None}, None, None

    if trig != "graph-tl" or click_data is None:
        return no_update, no_update, no_update

    pts = click_data.get("points", [])
    if not pts:
        return no_update, no_update, no_update

    # Extract the x-value (time or distance) of the clicked point
    x_val = pts[0].get("x")
    if x_val is None:
        return no_update, no_update, no_update
    x_val = float(x_val)

    if sel is None:
        sel = {"start": None, "end": None}

    if sel["start"] is None:
        # First click → set Start
        return {"start": x_val, "end": None}, round(x_val, 1), None
    else:
        # Second click → set End, sort ascending
        s, e = sel["start"], x_val
        if s > e:
            s, e = e, s
        return {"start": s, "end": e}, round(s, 1), round(e, 1)


# ═══════════════════════════════════════════════════════════════════════════════
# 7.  CALLBACK — Apply sector → store-xrange
# ═══════════════════════════════════════════════════════════════════════════════

@callback(
    Output("store-xrange", "data"),
    Input("btn-zoom", "n_clicks"),
    Input("btn-reset", "n_clicks"),
    State("in-start", "value"),
    State("in-end", "value"),
    prevent_initial_call=True,
)
def apply_sector(_nz, _nr, man_s, man_e):
    """Commit the manual sector inputs to the shared x-range store.

    Triggered by the 'Apply Sector' button or the 'Reset' button.
    """
    trig = ctx.triggered_id
    if trig == "btn-reset":
        return None
    if trig == "btn-zoom":
        if man_s is not None and man_e is not None:
            s, e = float(man_s), float(man_e)
            if s > e:
                s, e = e, s
            return [s, e]
    return no_update


# ═══════════════════════════════════════════════════════════════════════════════
# 8.  CALLBACK (HEAVY) — Full figure reconstruction + KPI update
# ═══════════════════════════════════════════════════════════════════════════════

@callback(
    Output("graph-map", "figure"),
    Output("graph-gg", "figure"),
    Output("graph-yaw", "figure"),
    Output("graph-tl", "figure"),
    Output("kpi-vmax", "children"),
    Output("kpi-lat", "children"),
    Output("kpi-long", "children"),
    Input("store-xrange", "data"),
    Input("store-sel", "data"),
    Input("sw-raw", "value"),
    Input("sw-3d", "value"),
    Input("sw-xaxis", "value"),
    Input("sw-roll", "value"),
    Input("sw-grip", "value"),
    Input("sw-gg-fix", "value"),
)
def heavy(xrange, sel, use_raw, use_3d, use_dist, show_roll, show_grip, gg_fixed):
    """Rebuild all four figures and recompute KPIs from scratch.

    This is the "expensive" callback — it creates entirely new Figure objects
    whenever a toggle or sector range changes.  The lighter hover/click
    callbacks use ``Patch()`` to avoid this cost on every mouse move.
    """
    x_col = "distance_m" if use_dist else "t_s"

    if xrange is not None:
        mask = (DF[x_col] >= xrange[0]) & (DF[x_col] <= xrange[1])
        sub = DF.loc[mask]
    else:
        sub = DF

    # Compute KPIs for the active sector
    if sub.empty:
        kv, kl, kg = "–", "–", "–"
    else:
        ac = "raw_ax_g" if use_raw else "ax_g"
        lc = "raw_ay_g" if use_raw else "ay_g"
        kv = f"{sub['vel_kmh'].max():.1f}"
        kl = f"{sub[lc].abs().max():.4f}"
        kg = f"{sub[ac].abs().max():.4f}"

    # Retrieve sector-selection anchors from store
    sel = sel or {"start": None, "end": None}
    sel_s = sel.get("start")
    sel_e = sel.get("end")

    # G-G fixed scale: compute axis range from the full dataset envelope
    gg_fr = None
    if gg_fixed:
        ac_full = "raw_ax_g" if use_raw else "ax_g"
        lc_full = "raw_ay_g" if use_raw else "ay_g"
        gg_fr = max(DF[lc_full].abs().max(), DF[ac_full].abs().max(), 0.01) * 1.15

    fig_map = build_map(sub)
    fig_gg  = build_gg(sub, use_raw, use_3d, gg_fr)
    fig_yaw = build_yaw(sub, use_raw, use_dist, show_roll, show_grip, xrange)
    fig_tl  = build_timeline(DF, use_raw, use_dist, xrange, sel_s, sel_e)

    return fig_map, fig_gg, fig_yaw, fig_tl, kv, kl, kg


# ═══════════════════════════════════════════════════════════════════════════════
# 9.  CALLBACK (LIGHT) — Hover cross-filtering via Patch()
# ═══════════════════════════════════════════════════════════════════════════════

def _extract_idx(data):
    """Extract the DataFrame row index from a hover/click event's customdata."""
    if data is None:
        return None
    pts = data.get("points", [])
    if not pts:
        return None
    cd = pts[0].get("customdata")
    if cd is not None:
        idx = cd[0] if isinstance(cd, (list, np.ndarray)) else cd
        try:
            return int(idx)
        except (ValueError, TypeError):
            return None
    return None


def _coords(idx, use_raw, use_3d, use_dist):
    """Look up all graph coordinates for a given DataFrame row index.

    Returns a dict with keys consumed by the per-graph patch helpers:
      lat, lon, gg_x, gg_y, gg_z, lx, yaw_y, tl_y
    or ``None`` if the index is out of range.
    """
    if idx is None or idx < 0 or idx >= N:
        return None
    r = DF.iloc[idx]
    x_v = r["distance_m"] if use_dist else r["t_s"]
    ax_c = "raw_ax_g" if use_raw else "ax_g"
    ay_c = "raw_ay_g" if use_raw else "ay_g"
    lat = r["kf_lat"] if r["kf_lat"] != 0 else r["gps_lat (°)"]
    lon = r["kf_lon"] if r["kf_lon"] != 0 else r["gps_lon (°)"]
    return {
        "lat": lat if lat != 0 else None,
        "lon": lon if lon != 0 else None,
        "gg_x": r[ay_c], "gg_y": r[ax_c],
        "gg_z": r["raw_az_g" if use_raw else "az_g"],
        "lx": x_v,
        "yaw_y": r["gz_raw" if use_raw else "gz_filt"],
        "tl_y": r["vel_kmh"],
    }


def _patch_map(c, ci):
    """Return a Patch that updates the map cursor sentinel at trace index *ci*."""
    p = Patch()
    if c and c["lat"]:
        p["data"][ci]["lat"] = [c["lat"]]
        p["data"][ci]["lon"] = [c["lon"]]
    else:
        p["data"][ci]["lat"] = [None]
        p["data"][ci]["lon"] = [None]
    return p


def _patch_xy(c, ci, xk, yk):
    """Return a Patch that updates a Scattergl cursor sentinel at *ci*."""
    p = Patch()
    if c:
        p["data"][ci]["x"] = [c[xk]]
        p["data"][ci]["y"] = [c[yk]]
    else:
        p["data"][ci]["x"] = [None]
        p["data"][ci]["y"] = [None]
    return p


def _patch_3d(c, ci):
    """Return a Patch that updates a Scatter3d cursor sentinel at *ci*."""
    p = Patch()
    if c:
        p["data"][ci]["x"] = [c["gg_x"]]
        p["data"][ci]["y"] = [c["gg_y"]]
        p["data"][ci]["z"] = [c["gg_z"]]
    else:
        p["data"][ci]["x"] = [None]
        p["data"][ci]["y"] = [None]
        p["data"][ci]["z"] = [None]
    return p


def _build_all_patches(c, use_3d, cursor_type):
    """Generate four Patch objects (Map, G-G, Yaw, Timeline) for one cursor type.

    Args:
        c:           Coordinate dict from ``_coords()``, or ``None``.
        use_3d:      Whether the G-G chart is in 3-D mode.
        cursor_type: ``"hover"`` or ``"click"`` — selects the correct trace index.

    Returns:
        A 4-tuple of ``Patch`` objects.
    """
    ci_map = CUR_MAP[0 if cursor_type == "hover" else 1]
    ci_gg  = CUR_GG[0 if cursor_type == "hover" else 1]
    ci_yaw = CUR_YAW[0 if cursor_type == "hover" else 1]
    ci_tl  = CUR_TL[0 if cursor_type == "hover" else 1]

    p_map = _patch_map(c, ci_map)
    if use_3d:
        p_gg = _patch_3d(c, ci_gg)
    else:
        p_gg = _patch_xy(c, ci_gg, "gg_x", "gg_y")
    p_yaw = _patch_xy(c, ci_yaw, "lx", "yaw_y")
    p_tl  = _patch_xy(c, ci_tl, "lx", "tl_y")
    return p_map, p_gg, p_yaw, p_tl


@callback(
    Output("graph-map", "figure", allow_duplicate=True),
    Output("graph-gg", "figure", allow_duplicate=True),
    Output("graph-yaw", "figure", allow_duplicate=True),
    Output("graph-tl", "figure", allow_duplicate=True),
    Input("graph-map", "hoverData"),
    Input("graph-gg", "hoverData"),
    Input("graph-yaw", "hoverData"),
    Input("graph-tl", "hoverData"),
    State("sw-raw", "value"), State("sw-3d", "value"), State("sw-xaxis", "value"),
    prevent_initial_call=True,
)
def light_hover(h_map, h_gg, h_yaw, h_tl, use_raw, use_3d, use_dist):
    """Move the hover cursor across all four graphs (lightweight Patch)."""
    trig = ctx.triggered_id
    data_map = {"graph-map": h_map, "graph-gg": h_gg,
                "graph-yaw": h_yaw, "graph-tl": h_tl}
    data = data_map.get(trig)
    idx = _extract_idx(data)
    c = _coords(idx, use_raw, use_3d, use_dist)
    return _build_all_patches(c, use_3d, "hover")


# ═══════════════════════════════════════════════════════════════════════════════
# 10. CALLBACK (LIGHT) — Click cross-filtering via Patch()
# ═══════════════════════════════════════════════════════════════════════════════

@callback(
    Output("graph-map", "figure", allow_duplicate=True),
    Output("graph-gg", "figure", allow_duplicate=True),
    Output("graph-yaw", "figure", allow_duplicate=True),
    Output("graph-tl", "figure", allow_duplicate=True),
    Input("graph-map", "clickData"),
    Input("graph-gg", "clickData"),
    Input("graph-yaw", "clickData"),
    State("sw-raw", "value"), State("sw-3d", "value"), State("sw-xaxis", "value"),
    prevent_initial_call=True,
)
def light_click(c_map, c_gg, c_yaw, use_raw, use_3d, use_dist):
    """Move the click cursor across all four graphs (lightweight Patch).

    Note: Timeline clicks are handled separately by ``sector_click()`` for
    the 2-click sector selection workflow.
    """
    trig = ctx.triggered_id
    data_map = {"graph-map": c_map, "graph-gg": c_gg, "graph-yaw": c_yaw}
    data = data_map.get(trig)
    idx = _extract_idx(data)
    c = _coords(idx, use_raw, use_3d, use_dist)
    return _build_all_patches(c, use_3d, "click")


# ═══════════════════════════════════════════════════════════════════════════════
# 11. ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def find_free_port(start=8050, end=8060):
    """Return the first available port in [start, end), fall back to start."""
    for port in range(start, end):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(('127.0.0.1', port)) != 0:
                return port
    return start

if __name__ == "__main__":
    port = find_free_port()
    print(f"\n  Telemetry Dashboard v1.2.2")
    print(f"   {N:,} samples — {DF['t_s'].iloc[-1]:.1f} s / {DF['distance_m'].iloc[-1]:.0f} m")
    if port != 8050:
        print(f"   Port 8050 in use — using port {port} instead")
    print(f"   http://127.0.0.1:{port}\n")
    app.run(debug=False, host="127.0.0.1", port=port)
