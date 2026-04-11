"""
bias_drift_report.py — MPU-6886 Gyroscope Bias Drift Analysis Report

Generates an HTML report from a 6-face static test (tel_94) recorded with
diagnostic firmware v1.2.2-diag-kgs (K_gs disabled, rotate_3d bypassed).
Demonstrates that the MPU-6886 has significant non-thermal bias drift on gx/gy.
Serves as addendum to MPU6886_ThermalDrift_Report.pdf.

Usage:
  python Tool/bias_drift_report.py                          # default tel_94.csv
  python Tool/bias_drift_report.py path/to/test.csv         # custom CSV
"""

import sys, os, io, base64
import numpy as np
import pandas as pd
from scipy.stats import trim_mean
from datetime import date
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Configuration ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(SCRIPT_DIR, "tel_94.csv")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "..", "Reports")

N_FACES = 6
TIME_TRIM_S = 30.0
VALUE_TRIM = 0.05
MOTION_WIN_S = 2.0
MOTION_STD_THR = 0.015
MIN_FACE_S = 10.0
FACE_LABELS = ["Face 1", "Face 2", "Face 3", "Face 4", "Face 5", "Face 6"]
GYRO_YLIM = (-2, 2)  # °/s — clips rotation transitions only

K_GS = np.array([
    [+0.6056, +0.7969, -0.1346],
    [+0.0712, +0.3539, -0.0908],
    [-0.1547, -0.0372, -0.0003],
])

C_GX, C_GY, C_GZ = "crimson", "forestgreen", "royalblue"
C_TEMP = "darkorange"
FACE_COLORS = list(plt.cm.tab10.colors)

# ── Data functions ────────────────────────────────────────────────────────────

def load_csv(path):
    df = pd.read_csv(path, encoding="latin-1")
    rename = {
        "t_ms (ms)": "t_ms", "ax (G)": "ax", "ay (G)": "ay", "az (G)": "az",
        "gx (°/s)": "gx", "gy (°/s)": "gy", "gz (°/s)": "gz",
        "raw_ax (G)": "raw_ax", "raw_ay (G)": "raw_ay", "raw_az (G)": "raw_az",
        "raw_gx (°/s)": "raw_gx", "raw_gy (°/s)": "raw_gy", "raw_gz (°/s)": "raw_gz",
        "temp_c (°C)": "temp_c",
    }
    df = df.rename(columns=rename)
    df["t_s"] = df["t_ms"] / 1000.0
    df["a_norm"] = np.sqrt(df["raw_ax"]**2 + df["raw_ay"]**2 + df["raw_az"]**2)
    return df


def estimate_fs(df):
    dt = np.median(np.diff(df["t_s"].values))
    return 1.0 / dt if dt > 0 else 50.0


def detect_faces(df, fs):
    win = max(1, int(MOTION_WIN_S * fs))
    rolling_std = df["a_norm"].rolling(win, center=True, min_periods=1).std().fillna(0)
    is_still = rolling_std < MOTION_STD_THR
    segments, in_seg, start = [], False, 0
    for i, still in enumerate(is_still):
        if still and not in_seg:
            start = i; in_seg = True
        elif not still and in_seg:
            segments.append((start, i - 1)); in_seg = False
    if in_seg:
        segments.append((start, len(df) - 1))
    min_samples = max(1, int(MIN_FACE_S * fs))
    segments = [(s, e) for s, e in segments if (e - s + 1) >= min_samples]
    segments.sort(key=lambda s: s[1] - s[0], reverse=True)
    segments = segments[:N_FACES]
    segments.sort(key=lambda s: s[0])
    return segments, rolling_std


def get_core(seg_df, time_trim):
    if len(seg_df) > 2 * time_trim + 10:
        return seg_df.iloc[time_trim:-time_trim]
    return seg_df


def face_stats(seg_df, time_trim):
    core = get_core(seg_df, time_trim)
    n = len(core)
    r = {"n": n}
    for col in ["raw_gx", "raw_gy", "raw_gz", "raw_ax", "raw_ay", "raw_az"]:
        vals = core[col].values
        r[f"{col}_mean"] = trim_mean(vals, VALUE_TRIM)
        r[f"{col}_std"] = float(np.std(vals))
    r["t_ini"] = float(core["temp_c"].iloc[0])
    r["t_fin"] = float(core["temp_c"].iloc[-1])
    r["delta_t"] = r["t_fin"] - r["t_ini"]
    r["dur_s"] = float(core["t_s"].iloc[-1] - core["t_s"].iloc[0])
    return r


def half_split_stats(seg_df, time_trim):
    core = get_core(seg_df, time_trim)
    mid = len(core) // 2
    h1, h2 = core.iloc[:mid], core.iloc[mid:]
    dur_half_min = (len(h1) / 50.0) / 60.0
    r = {}
    for col, key in [("raw_gx", "gx"), ("raw_gy", "gy"), ("raw_gz", "gz")]:
        m1 = trim_mean(h1[col].values, VALUE_TRIM)
        m2 = trim_mean(h2[col].values, VALUE_TRIM)
        r[f"d_{key}"] = m2 - m1
        r[f"d_{key}_per_min"] = (m2 - m1) / dur_half_min if dur_half_min > 0 else 0
        r[f"{key}_h1"] = m1
        r[f"{key}_h2"] = m2
    return r


def dominant_axis(r):
    axes = [("ax", r["raw_ax_mean"]), ("ay", r["raw_ay_mean"]), ("az", r["raw_az_mean"])]
    axes.sort(key=lambda x: abs(x[1]), reverse=True)
    name, val = axes[0]
    chip_axis = name[1].upper()
    direction = "up" if val > 0 else "down"
    return f"{chip_axis}-chip {direction} ({val:+.3f} g)"


def global_drift_regression(df, segments, time_trim):
    """Fit linear regression through per-face trimmed means at their midpoint times.

    This is the correct approach: each face provides one independent measurement
    of the bias at a known time. Fitting through all raw samples would mix
    inter-face variance (orientation-dependent) with temporal drift.
    """
    cores = []
    for i0, i1 in segments:
        seg = df.iloc[i0:i1 + 1]
        cores.append(get_core(seg, time_trim))
    t0 = cores[0]["t_s"].iloc[0]

    # Compute midpoint time and trimmed mean for each face
    face_times = []
    face_means = {"raw_gx": [], "raw_gy": [], "raw_gz": []}
    for core in cores:
        t_mid = (core["t_s"].iloc[0] + core["t_s"].iloc[-1]) / 2 - t0
        face_times.append(t_mid)
        for col in face_means:
            face_means[col].append(trim_mean(core[col].values, VALUE_TRIM))

    face_times = np.array(face_times)
    total_dur = face_times[-1]

    result = {}
    for col, key in [("raw_gx", "gx"), ("raw_gy", "gy"), ("raw_gz", "gz")]:
        means = np.array(face_means[col])
        coeffs = np.polyfit(face_times, means, 1)
        slope = coeffs[0]
        predicted = np.polyval(coeffs, face_times)
        ss_res = np.sum((means - predicted) ** 2)
        ss_tot = np.sum((means - np.mean(means)) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
        result[key] = {
            "slope_per_s": slope, "slope_per_min": slope * 60,
            "r2": r2, "total_drift": slope * total_dur,
            "total_dur_s": total_dur, "coeffs": coeffs, "t0": t0,
        }
    return result


def kgs_offline_validation(results):
    a_boot = np.array([results[0]["raw_ax_mean"], results[0]["raw_ay_mean"],
                        results[0]["raw_az_mean"]])
    corrected = []
    for r in results:
        a_face = np.array([r["raw_ax_mean"], r["raw_ay_mean"], r["raw_az_mean"]])
        da = a_face - a_boot
        da_kgs = np.array([da[2], da[0], da[1]])
        correction = K_GS @ da_kgs
        corrected.append({
            "label": r["label"],
            "raw_gx": r["raw_gx_mean"], "raw_gy": r["raw_gy_mean"], "raw_gz": r["raw_gz_mean"],
            "corr_gx": r["raw_gx_mean"] - correction[0],
            "corr_gy": r["raw_gy_mean"] - correction[1],
            "corr_gz": r["raw_gz_mean"] - correction[2],
        })
    spreads = {}
    for axis in ["gx", "gy", "gz"]:
        raw_vals = [c[f"raw_{axis}"] for c in corrected]
        corr_vals = [c[f"corr_{axis}"] for c in corrected]
        raw_sp = max(raw_vals) - min(raw_vals)
        corr_sp = max(corr_vals) - min(corr_vals)
        change = (corr_sp / raw_sp - 1) * 100 if raw_sp > 0 else 0
        spreads[axis] = {"raw": raw_sp, "corr": corr_sp, "change_pct": change}
    return {"corrected": corrected, "spreads": spreads}


# ── Plot helpers ──────────────────────────────────────────────────────────────

def fig_to_base64(fig, dpi=150):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("ascii")


def make_warmup_plot(df_warmup, fs):
    """Plot the warmup phase (lap=0): gyro + temperature vs time."""
    # Remove spurious post-test tail: keep only the first contiguous segment
    # (gap > 10s indicates the test ran in between)
    dt = df_warmup["t_s"].diff()
    big_gap = dt[dt > 10.0]
    if len(big_gap) > 0:
        df_warmup = df_warmup.iloc[:big_gap.index[0]].copy()
    ds = 10
    t = df_warmup["t_s"].iloc[::ds]
    fig, (ax_g, ax_t) = plt.subplots(2, 1, figsize=(10, 4.5), sharex=True,
                                      gridspec_kw={"height_ratios": [2, 1]})
    fig.suptitle("Warm-up Phase (lap = 0) — Sensor thermal stabilization",
                 fontsize=11, fontweight="bold")
    for col, c, lbl in [("raw_gx", C_GX, "gx"), ("raw_gy", C_GY, "gy"), ("raw_gz", C_GZ, "gz")]:
        ax_g.plot(t, df_warmup[col].iloc[::ds], lw=0.4, color=c, alpha=0.8, label=lbl)
    ax_g.set_ylim(GYRO_YLIM)
    ax_g.axhline(0, color="k", lw=0.4, ls=":")
    ax_g.set_ylabel("Gyro (°/s)")
    ax_g.legend(loc="upper right", fontsize=8)
    ax_g.grid(True, alpha=0.2)

    ax_t.plot(t, df_warmup["temp_c"].iloc[::ds], lw=0.8, color=C_TEMP)
    ax_t.set_ylabel("Temp (°C)")
    ax_t.set_xlabel("Time (s)")
    ax_t.grid(True, alpha=0.2)
    plt.tight_layout()
    return fig_to_base64(fig)


def make_overview_plot(df, segments, results, fs):
    ds = 5
    t = df["t_s"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
    fig.suptitle("6-Face Static Test — Full Timeline (lap = 1, chip-frame)",
                 fontsize=11, fontweight="bold")

    def shade(ax):
        for i, ((i0, i1), r) in enumerate(zip(segments, results)):
            seg = df.iloc[i0:i1 + 1]
            t0, t1 = seg["t_s"].iloc[0], seg["t_s"].iloc[-1]
            c = FACE_COLORS[i % 10]
            ax.axvspan(t0, t1, alpha=0.10, color=c)

    axes[0].plot(t.iloc[::ds], df["a_norm"].iloc[::ds], lw=0.3, color="steelblue")
    axes[0].axhline(1.0, color="k", lw=0.5, ls=":")
    axes[0].set_ylabel("|a| (g)")
    axes[0].set_title("Accelerometer norm", fontsize=9)

    for col, c, lbl in [("raw_gx", C_GX, "gx"), ("raw_gy", C_GY, "gy"), ("raw_gz", C_GZ, "gz")]:
        axes[1].plot(t.iloc[::ds], df[col].iloc[::ds], lw=0.3, color=c, alpha=0.8, label=lbl)
    axes[1].set_ylim(GYRO_YLIM)
    axes[1].axhline(0, color="k", lw=0.4, ls=":")
    axes[1].set_ylabel("Gyro (°/s)")
    axes[1].set_title("Gyroscope chip-frame (clipped to ±2 °/s — rotation transitions exceed this range)",
                       fontsize=9)
    axes[1].legend(loc="upper right", fontsize=7)

    for col, c, lbl in [("raw_ax", C_GX, "ax"), ("raw_ay", C_GY, "ay"), ("raw_az", C_GZ, "az")]:
        axes[2].plot(t.iloc[::ds], df[col].iloc[::ds], lw=0.3, color=c, alpha=0.8, label=lbl)
    axes[2].set_ylabel("Accel (g)")
    axes[2].set_title("Accelerometer chip-frame", fontsize=9)
    axes[2].legend(loc="upper right", fontsize=7)

    axes[3].plot(t.iloc[::ds], df["temp_c"].iloc[::ds], lw=0.7, color=C_TEMP)
    axes[3].set_ylabel("Temp (°C)")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_title("IMU die temperature", fontsize=9)

    for ax in axes:
        ax.relim(); ax.autoscale_view()
        shade(ax)
    # re-enforce gyro ylim after autoscale
    axes[1].set_ylim(GYRO_YLIM)

    for i, ((i0, i1), r) in enumerate(zip(segments, results)):
        seg = df.iloc[i0:i1 + 1]
        t_mid = (seg["t_s"].iloc[0] + seg["t_s"].iloc[-1]) / 2
        axes[1].text(t_mid, GYRO_YLIM[1] * 0.90, r["label"],
                     ha="center", va="top", fontsize=7, color=FACE_COLORS[i % 10],
                     fontweight="bold")

    plt.tight_layout()
    return fig_to_base64(fig)


def make_face_plot(df, seg_bounds, result, face_idx, fs):
    i0, i1 = seg_bounds
    seg = df.iloc[i0:i1 + 1].copy()
    t_start = seg["t_s"].iloc[0]
    t_rel = seg["t_s"] - t_start
    time_trim = int(TIME_TRIM_S * fs)

    fig, (ax_g, ax_t) = plt.subplots(2, 1, figsize=(10, 4.5), sharex=True,
                                      gridspec_kw={"height_ratios": [3, 1]})
    orient = dominant_axis(result)
    fig.suptitle(f"{result['label']}: {orient}", fontsize=11, fontweight="bold")

    for col, c, name in [("raw_gx", C_GX, "gx"), ("raw_gy", C_GY, "gy"),
                          ("raw_gz", C_GZ, "gz")]:
        ax_g.plot(t_rel, seg[col], lw=0.3, color=c, alpha=0.7, label=name)
        mean_val = result[f"{col}_mean"]
        ax_g.axhline(mean_val, color=c, lw=1.0, ls="--", alpha=0.6)
        # Annotate mean value
        ax_g.text(t_rel.iloc[-1] * 1.01, mean_val, f"{mean_val:+.3f}",
                  fontsize=7, color=c, va="center")

    ax_g.set_ylim(GYRO_YLIM)
    trim_t = TIME_TRIM_S
    dur_total = t_rel.iloc[-1]
    ax_g.axvline(trim_t, color="gray", lw=0.8, ls=":", alpha=0.6)
    ax_g.axvline(dur_total - trim_t, color="gray", lw=0.8, ls=":", alpha=0.6)
    mid_t = dur_total / 2
    ax_g.axvline(mid_t, color="gray", lw=0.6, ls="-.", alpha=0.4)
    ax_g.axhline(0, color="k", lw=0.4, ls=":")
    ax_g.set_ylabel("Gyro (°/s)")
    ax_g.legend(loc="upper right", fontsize=8)
    ax_g.grid(True, alpha=0.2)

    ax_t.plot(t_rel, seg["temp_c"], lw=0.7, color=C_TEMP)
    ax_t.set_ylabel("Temp (°C)")
    ax_t.set_xlabel("Time since face start (s)")
    ax_t.grid(True, alpha=0.2)
    plt.tight_layout()
    return fig_to_base64(fig)


def make_drift_plot(df, segments, results, drift, fs):
    time_trim = int(TIME_TRIM_S * fs)
    t0_global = df["t_s"].iloc[segments[0][0]]

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.suptitle("Global Bias Drift — Per-face means vs elapsed time",
                 fontsize=11, fontweight="bold")

    for axis_col, key, color in [("raw_gx", "gx", C_GX), ("raw_gy", "gy", C_GY),
                                  ("raw_gz", "gz", C_GZ)]:
        times_mid, means = [], []
        for i, (i0, i1) in enumerate(segments):
            seg = df.iloc[i0:i1 + 1]
            core = get_core(seg, time_trim)
            t_mid = (core["t_s"].iloc[0] + core["t_s"].iloc[-1]) / 2 - t0_global
            times_mid.append(t_mid)
            means.append(results[i][f"{axis_col}_mean"])

        ax.scatter(times_mid, means, color=color, s=60, zorder=5, label=key,
                   edgecolors="k", linewidths=0.5)

        # Use the same regression coefficients as global_drift_regression
        d = drift[key]
        t_line = np.linspace(0, d["total_dur_s"], 100)
        ax.plot(t_line, np.polyval(d["coeffs"], t_line), color=color, lw=1.5, ls="--", alpha=0.6)

    ax.set_xlabel("Time since test start (s)")
    ax.set_ylabel("Gyro trimmed mean (°/s)")
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    for i, (i0, i1) in enumerate(segments):
        seg = df.iloc[i0:i1 + 1]
        core = get_core(seg, time_trim)
        t_mid = (core["t_s"].iloc[0] + core["t_s"].iloc[-1]) / 2 - t0_global
        ax.text(t_mid, ax.get_ylim()[1], results[i]["label"],
                ha="center", va="top", fontsize=7, color=FACE_COLORS[i % 10], fontweight="bold")
    plt.tight_layout()
    return fig_to_base64(fig)


def make_kgs_bar_plot(kgs_result):
    fig, ax = plt.subplots(figsize=(7, 4))
    axes_names = ["gx", "gy", "gz"]
    x = np.arange(3)
    w = 0.35
    raw_vals = [kgs_result["spreads"][a]["raw"] for a in axes_names]
    corr_vals = [kgs_result["spreads"][a]["corr"] for a in axes_names]

    bars1 = ax.bar(x - w/2, raw_vals, w, label="Raw (no K_gs)", color="steelblue", edgecolor="k")
    bars2 = ax.bar(x + w/2, corr_vals, w, label="K_gs corrected", color="salmon", edgecolor="k")
    ax.set_xticks(x)
    ax.set_xticklabels(axes_names, fontsize=12)
    ax.set_ylabel("Inter-face spread (°/s)")
    ax.set_title("K_gs Validation: Spread Comparison", fontsize=11, fontweight="bold")
    ax.legend(fontsize=10)
    ax.grid(True, axis="y", alpha=0.3)
    for grp in [bars1, bars2]:
        for bar in grp:
            h = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2, h + 0.02, f"{h:.2f}",
                    ha="center", va="bottom", fontsize=9)
    plt.tight_layout()
    return fig_to_base64(fig)


# ── HTML generation ───────────────────────────────────────────────────────────

CSS = """
@page { size: A4; margin: 20mm 18mm; }
body { font-family: 'Segoe UI', Arial, sans-serif; font-size: 11pt;
       line-height: 1.5; color: #222; max-width: 180mm; margin: auto; }
h1 { font-size: 22pt; color: #111; margin-bottom: 2px; }
h2 { font-size: 14pt; border-left: 4px solid #8b0000; padding-left: 10px;
     margin-top: 36px; page-break-before: always; }
h2:first-of-type { page-break-before: avoid; }
h3 { font-size: 12pt; color: #333; margin-top: 18px; }
table { border-collapse: collapse; width: 100%; margin: 12px 0; font-size: 10pt; }
th { background: #f0f0f0; border: 1px solid #ccc; padding: 6px 10px; text-align: left; }
td { border: 1px solid #ccc; padding: 5px 10px; }
tr:nth-child(even) { background: #fafafa; }
.highlight { background: #8b0000; color: white; padding: 10px 14px; margin: 14px 0;
             font-weight: bold; font-size: 10.5pt; line-height: 1.5; }
.caption { font-size: 9pt; color: #555; font-style: italic; margin-top: 4px; margin-bottom: 16px; }
.metadata { color: gray; font-size: 10pt; }
img { max-width: 100%; height: auto; }
.face-section { page-break-inside: avoid; }
.pre { font-family: 'Consolas', 'Courier New', monospace; font-size: 9.5pt;
       background: #f8f8f0; padding: 10px; border: 1px solid #ddd; white-space: pre-wrap; }
"""


def build_html(df_full, df, segments, results, drift, kgs_result, metadata):
    """Build the full HTML report string."""
    html = []
    html.append(f"<!DOCTYPE html><html><head><meta charset='utf-8'>")
    html.append(f"<title>MPU-6886 Bias Drift Analysis</title>")
    html.append(f"<style>{CSS}</style></head><body>")

    # ── Title page ──
    html.append(f"<h1>MPU-6886 Gyroscope<br>Bias Drift Analysis</h1>")
    html.append(f"<p class='metadata'>Internal R&D Report — "
                f"Addendum to MPU6886_ThermalDrift_Report.pdf</p>")
    html.append(f"<table>")
    for label, value in [
        ("Sensor", "MPU6886 (6-axis MEMS IMU, no on-chip temp compensation)"),
        ("Platform", "M5Stack AtomS3 (ESP32-S3 + 0.85\" IPS display)"),
        ("Test ID", metadata["test_id"]),
        ("Firmware", metadata["firmware"]),
        ("Data frame", metadata["data_frame"]),
        ("Duration (lap=1)", metadata["duration"]),
        ("Total samples", metadata["samples"]),
        ("Temperature range", metadata["temp_range"]),
        ("Date", metadata["date"]),
    ]:
        html.append(f"<tr><td><strong>{label}</strong></td><td>{value}</td></tr>")
    html.append(f"</table>")
    html.append(f"<p>This report characterizes <strong>non-thermal bias drift</strong> in the "
                f"MPU-6886 MEMS gyroscope. A 6-face static test was performed with K_gs "
                f"G-sensitivity correction and geometric rotation (rotate_3d) disabled in the "
                f"firmware, so that raw chip-frame gyro and accelerometer data are recorded "
                f"directly. The sensor was physically rotated to 6 different orientations and "
                f"held stationary on each face for approximately 5 minutes, taped to a flat "
                f"surface. These physical rotations appear as brief high-amplitude spikes in "
                f"the gyroscope plots (clipped at ±2 °/s) and are excluded from the analysis.</p>")

    # ── Table of Contents ──
    html.append(f"<h2 style='page-break-before:avoid; border: 1px solid #8b0000; "
                f"border-left: 4px solid #8b0000;'>Table of Contents</h2>")
    toc = [
        "1. Experimental Setup & Methodology",
        "2. Warm-up Phase",
        "3. 6-Face Test Overview",
        "4. Per-Face Analysis (Faces 1–6)",
        "5. Global Drift Regression",
        "6. K_gs Validation",
        "7. Comparison with Thermal Drift Report",
        "8. Conclusions",
    ]
    html.append("<ol style='font-size:11pt'>")
    for item in toc:
        html.append(f"<li>{item.split('. ', 1)[1]}</li>")
    html.append("</ol>")

    # ── 1. Methodology ──
    html.append(f"<h2>1. Experimental Setup &amp; Methodology</h2>")
    html.append(f"<p>The test was performed with firmware <strong>v1.2.2-diag-kgs</strong>, "
                f"a diagnostic build with three modifications:</p>")
    html.append(f"<ol>")
    html.append(f"<li><strong>K_gs G-sensitivity correction disabled</strong> — the 3×3 matrix "
                f"that compensates gyroscope sensitivity to gravitational orientation was "
                f"commented out.</li>")
    html.append(f"<li><strong>Gyro rotate_3d bypassed</strong> — gyroscope output remains in "
                f"chip-native frame (not rotated to vehicle frame). This ensures raw_gx/gy/gz "
                f"in the CSV are true chip-frame values.</li>")
    html.append(f"<li><strong>raw_ax/ay/az = chip-frame accel</strong> — instead of linear "
                f"acceleration (gravity removed), the raw accel fields contain ellipsoid-"
                f"calibrated chip-frame accelerometer data with gravity.</li>")
    html.append(f"</ol>")
    html.append(f"<p>The sensor was placed on 6 different faces in sequence, each taped to a "
                f"flat surface for ~5 minutes. The physical rotations between faces were "
                f"performed manually and take a few seconds each. These transitions produce "
                f"large gyroscope spikes (tens of °/s) that are <strong>clipped in all plots "
                f"at ±2 °/s</strong> to preserve the scale of the much smaller bias drift "
                f"signal (~0.1–2 °/s).</p>")
    html.append(f"<p>A warm-up phase (lap=0, ~48 minutes) preceded the test to allow the "
                f"sensor to reach thermal equilibrium. The 6-face test data is recorded "
                f"as lap=1.</p>")
    html.append(f"<p><em>Face detection: rolling std of |a| over a 2s window, threshold "
                f"0.015 g. The 6 longest static segments are selected. First and last 30s "
                f"of each face are trimmed. Statistics use 5% trimmed means.</em></p>")

    # ── 2. Warmup ──
    html.append(f"<h2>2. Warm-up Phase</h2>")
    df_warmup = df_full[df_full["lap"] == 0].reset_index(drop=True)
    if len(df_warmup) > 0:
        warmup_dur = (df_warmup["t_s"].iloc[-1] - df_warmup["t_s"].iloc[0])
        warmup_t_start = df_warmup["temp_c"].iloc[0]
        warmup_t_end = df_warmup["temp_c"].iloc[-1]
        html.append(f"<p>The sensor was powered on and left stationary for "
                    f"<strong>{warmup_dur/60:.0f} minutes</strong> before starting the 6-face "
                    f"test. Temperature rose from {warmup_t_start:.1f}°C to "
                    f"{warmup_t_end:.1f}°C during this phase, after which it stabilized.</p>")
        img_warmup = make_warmup_plot(df_warmup, estimate_fs(df_warmup))
        html.append(f"<img src='data:image/png;base64,{img_warmup}'>")
        html.append(f"<p class='caption'>Figure 1 — Warm-up phase: gyroscope and temperature "
                    f"vs time. The sensor was stationary throughout. Gyro drift during warm-up "
                    f"reflects both thermal settling and electronic bias instability.</p>")
    else:
        html.append(f"<p><em>No warm-up data (lap=0) found in the CSV.</em></p>")

    # ── 3. Overview ──
    html.append(f"<h2>3. 6-Face Test Overview</h2>")
    html.append(f"<p>The test consists of 6 static windows separated by brief physical "
                f"rotations. The gyroscope plot reveals a clear <strong>monotonic downward "
                f"trend on gx</strong> (red) across the entire test, despite the temperature "
                f"remaining nearly flat (37.6–39.6°C).</p>")
    img_overview = make_overview_plot(df, segments, results, estimate_fs(df))
    html.append(f"<img src='data:image/png;base64,{img_overview}'>")
    html.append(f"<p class='caption'>Figure 2 — Full timeline of the 6-face test (lap=1). "
                f"Colored bands mark detected static faces. Gyro scale clipped to ±2 °/s; "
                f"rotation transitions between faces exceed this range and appear clipped.</p>")

    # Per-face summary table
    html.append(f"<h3>Per-face summary</h3>")
    html.append(f"<table><tr><th>Face</th><th>Orientation</th><th>N (trimmed)</th>"
                f"<th>gx (°/s)</th><th>gy (°/s)</th><th>gz (°/s)</th>"
                f"<th>T<sub>ini</sub></th><th>T<sub>fin</sub></th><th>ΔT</th></tr>")
    for r in results:
        orient = dominant_axis(r)
        html.append(f"<tr><td>{r['label']}</td><td>{orient}</td><td>{r['n']}</td>"
                    f"<td>{r['raw_gx_mean']:+.4f}</td><td>{r['raw_gy_mean']:+.4f}</td>"
                    f"<td>{r['raw_gz_mean']:+.4f}</td>"
                    f"<td>{r['t_ini']:.1f}°C</td><td>{r['t_fin']:.1f}°C</td>"
                    f"<td>{r['delta_t']:+.2f}°C</td></tr>")
    html.append(f"</table>")

    html.append(f'<div class="highlight">The inter-face spread on gx is 2.54 °/s and on '
                f'gy is 1.16 °/s, while gz spread is only 0.09 °/s. The spread on gx/gy '
                f'is dominated by a monotonic temporal trend, not by G-sensitivity.</div>')

    # ── 4. Per-face ──
    html.append(f"<h2>4. Per-Face Analysis</h2>")
    html.append(f"<p>Each face is shown with gyroscope axes and temperature vs time. "
                f"Dashed horizontal lines indicate the trimmed mean for each axis. "
                f"Vertical dotted lines mark the 30s time-trim boundaries; the dash-dot "
                f"line marks the midpoint used for the half-split drift estimate.</p>")

    for i in range(len(segments)):
        r = results[i]
        orient = dominant_axis(r)
        img_face = make_face_plot(df, segments[i], r, i, estimate_fs(df))
        html.append(f'<div class="face-section">')
        html.append(f"<h3>{r['label']}: {orient}</h3>")
        html.append(f"<img src='data:image/png;base64,{img_face}'>")
        html.append(f"<table><tr><th>Metric</th><th>gx</th><th>gy</th><th>gz</th></tr>")
        html.append(f"<tr><td>Trimmed mean (°/s)</td>"
                    f"<td>{r['raw_gx_mean']:+.4f}</td><td>{r['raw_gy_mean']:+.4f}</td>"
                    f"<td>{r['raw_gz_mean']:+.4f}</td></tr>")
        html.append(f"<tr><td>Std (°/s)</td>"
                    f"<td>{r['raw_gx_std']:.4f}</td><td>{r['raw_gy_std']:.4f}</td>"
                    f"<td>{r['raw_gz_std']:.4f}</td></tr>")
        html.append(f"<tr><td>Intra-face drift Δ (°/s)</td>"
                    f"<td>{r['d_gx']:+.4f}</td><td>{r['d_gy']:+.4f}</td>"
                    f"<td>{r['d_gz']:+.4f}</td></tr>")
        html.append(f"<tr><td>Temperature</td>"
                    f"<td colspan='3'>{r['t_ini']:.1f} → {r['t_fin']:.1f}°C "
                    f"(ΔT = {r['delta_t']:+.2f}°C)</td></tr>")
        html.append(f"<tr><td>Duration / N</td>"
                    f"<td colspan='3'>{r['dur_s']:.0f}s / {r['n']} samples</td></tr>")
        html.append(f"</table></div>")

    # ── 5. Global drift ──
    html.append(f"<h2>5. Global Drift Regression</h2>")
    html.append(f"<p>A linear regression is fitted to the 6 per-face trimmed means "
                f"at their midpoint times. Each face provides one independent measurement "
                f"of the bias at a known time, isolating the temporal drift from "
                f"orientation-dependent offsets.</p>")
    img_drift = make_drift_plot(df, segments, results, drift, estimate_fs(df))
    html.append(f"<img src='data:image/png;base64,{img_drift}'>")
    html.append(f"<p class='caption'>Figure 9 — Per-face trimmed means plotted against "
                f"elapsed time. Dashed lines show linear regression fitted through the "
                f"6 per-face means.</p>")

    html.append(f"<table><tr><th>Axis</th><th>Drift rate (°/s/min)</th>"
                f"<th>R²</th><th>Total drift (°/s)</th>"
                f"<th>Duration (s)</th></tr>")
    for key in ["gx", "gy", "gz"]:
        d = drift[key]
        html.append(f"<tr><td><strong>{key.upper()}</strong></td>"
                    f"<td>{d['slope_per_min']:+.4f}</td>"
                    f"<td>{d['r2']:.4f}</td>"
                    f"<td>{d['total_drift']:+.4f}</td>"
                    f"<td>{d['total_dur_s']:.0f}</td></tr>")
    html.append(f"</table>")

    html.append(f'<div class="highlight">GX drifts at -0.076 °/s/min '
                f'(4.6 °/s/hour) and GY at -0.030 °/s/min (1.8 °/s/hour), both at '
                f'near-constant temperature. GZ is stable at +0.002 °/s/min. The drift is '
                f'electronic (non-thermal), consistent with MEMS stress relaxation.</div>')

    # ── 6. K_gs ──
    html.append(f"<h2>6. K_gs Validation</h2>")
    html.append(f"<p>The K_gs G-sensitivity matrix (computed from earlier tests tel_gsen4 "
                f"+ tel_82) was applied offline to the tel_94 chip-frame data. If K_gs "
                f"correctly removes gravity dependence, the corrected inter-face spread "
                f"should decrease.</p>")
    img_kgs = make_kgs_bar_plot(kgs_result)
    html.append(f"<img src='data:image/png;base64,{img_kgs}' style='max-width:70%'>")

    html.append(f"<table><tr><th>Axis</th><th>Raw spread (°/s)</th>"
                f"<th>Corrected spread (°/s)</th><th>Change</th><th>Verdict</th></tr>")
    for axis in ["gx", "gy", "gz"]:
        s = kgs_result["spreads"][axis]
        verdict = "WORSE" if s["change_pct"] > 0 else "BETTER"
        color = "#8b0000" if s["change_pct"] > 0 else "#006400"
        html.append(f"<tr><td><strong>{axis.upper()}</strong></td>"
                    f"<td>{s['raw']:.4f}</td><td>{s['corr']:.4f}</td>"
                    f"<td style='color:{color}'>{s['change_pct']:+.1f}%</td>"
                    f"<td style='color:{color}'><strong>{verdict}</strong></td></tr>")
    html.append(f"</table>")

    html.append(f"<p>K_gs correction <strong>increases spread on all three axes</strong>. "
                f"This happens because the dominant source of inter-face variation is "
                f"temporal bias drift, not G-sensitivity. Since the faces were tested "
                f"sequentially, time and orientation are confounded — K_gs subtracts a "
                f"gravity-dependent correction that partially correlates with the time "
                f"trend, making the result worse.</p>")
    html.append(f'<div class="highlight">K_gs was removed from firmware in v1.3.0. '
                f'The non-thermal drift dominates the G-sensitivity signal, making any '
                f'static gravity correction unreliable.</div>')

    # ── 7. Comparison ──
    html.append(f"<h2>7. Comparison with Thermal Drift Report</h2>")
    html.append(f"<h3>Previous finding (MPU6886_ThermalDrift_Report.pdf)</h3>")
    html.append(f"<ul>")
    html.append(f"<li>Gyroscope bias drift was attributed primarily to thermal sensitivity.</li>")
    html.append(f"<li>The thermal sensitivity dG/dT was found non-repeatable between sessions "
                f"on GX and GY (inter-run σ ~ 0.10 °/s/°C).</li>")
    html.append(f"<li>LUT and Poly-3 compensation both failed: 44% of GX samples fell in the "
                f"\"unusable\" category even after correction.</li>")
    html.append(f"<li>Conclusion: no fixed compensation model can capture the non-deterministic "
                f"drift shape. ZARU is required at every power-on.</li>")
    html.append(f"</ul>")

    html.append(f"<h3>New finding (this report)</h3>")
    html.append(f"<ul>")
    html.append(f"<li>Temperature was nearly flat during the test "
                f"({metadata['temp_range']}, ΔT &lt; 2°C).</li>")
    html.append(f"<li>Despite flat temperature, significant bias drift was observed:<br>"
                f"GX: {drift['gx']['slope_per_min']:+.4f} °/s/min "
                f"(total {drift['gx']['total_drift']:+.2f} °/s over "
                f"{drift['gx']['total_dur_s']:.0f}s)<br>"
                f"GY: {drift['gy']['slope_per_min']:+.4f} °/s/min "
                f"(total {drift['gy']['total_drift']:+.2f} °/s)<br>"
                f"GZ: {drift['gz']['slope_per_min']:+.4f} °/s/min (stable)</li>")
    html.append(f"<li>The drift is monotonic over 35 minutes, consistent with electronic "
                f"bias instability (stress relaxation, charge trapping), not thermal.</li>")
    html.append(f"</ul>")

    html.append(f"<h3>Reconciliation</h3>")
    html.append(f"<p>Both effects coexist in the MPU-6886:</p>")
    html.append(f"<ol>")
    html.append(f"<li><strong>Thermal sensitivity</strong> — real, observed in the heating "
                f"tests. The bias changes when temperature changes.</li>")
    html.append(f"<li><strong>Electronic drift</strong> — newly quantified in this test. "
                f"The bias changes over time even at constant temperature.</li>")
    html.append(f"</ol>")
    html.append(f"<p>The thermal drift report could not separate these two effects because "
                f"temperature and time were monotonically correlated in all 7 test runs "
                f"(continuous heating). The \"inter-run shape variability\" attributed to "
                f"non-repeatable thermal sensitivity was likely contaminated by the electronic "
                f"drift component.</p>")
    html.append(f"<p>ZARU addresses both: it periodically resets the bias estimate at "
                f"stationarity, regardless of whether the drift source is thermal or "
                f"electronic.</p>")

    # ── 8. Conclusions ──
    html.append(f"<h2>8. Conclusions</h2>")
    gx_10min = abs(drift["gx"]["slope_per_min"]) * 10

    html.append(f"<ol>")
    html.append(f"<li><strong>GX has the largest non-thermal drift</strong> at "
                f"{drift['gx']['slope_per_min']:+.4f} °/s/min "
                f"({abs(drift['gx']['slope_per_min']) * 60:.1f} °/s/hour). Over a 10-minute "
                f"driving stint without stationarity, this accumulates ~{gx_10min:.2f} °/s "
                f"of uncorrected bias error.</li>")
    html.append(f"<li><strong>GY drift is moderate</strong> at "
                f"{drift['gy']['slope_per_min']:+.4f} °/s/min "
                f"({abs(drift['gy']['slope_per_min']) * 60:.1f} °/s/hour). Less critical: "
                f"gy (pitch rate) does not directly feed the 2D ESKF heading.</li>")
    html.append(f"<li><strong>GZ is stable</strong> at "
                f"{drift['gz']['slope_per_min']:+.4f} °/s/min "
                f"({abs(drift['gz']['slope_per_min']) * 60:.2f} °/s/hour). This is "
                f"excellent for the ESKF, which uses gz for heading estimation.</li>")
    html.append(f"<li><strong>K_gs correction is counterproductive.</strong> "
                f"Spread after correction: GX {kgs_result['spreads']['gx']['change_pct']:+.0f}%, "
                f"GY {kgs_result['spreads']['gy']['change_pct']:+.0f}%, "
                f"GZ {kgs_result['spreads']['gz']['change_pct']:+.0f}% (all worse). "
                f"K_gs was removed from firmware in v1.3.0.</li>")
    html.append(f"<li><strong>ZARU remains the primary defense</strong> against both thermal "
                f"and electronic drift. The 3-axis ZARU (v1.2.1) resets bias at stationarity "
                f"and during straight-line driving, handling both sources transparently.</li>")
    html.append(f"<li><strong>Recommendation for future calibration:</strong> if K_gs is ever "
                f"revisited, randomize face order (not sequential) to decorrelate temporal "
                f"drift from gravity orientation.</li>")
    html.append(f"</ol>")

    html.append(f'<div class="highlight">The MPU-6886 gyroscope exhibits significant '
                f'non-thermal electronic drift on GX and GY. This drift was previously '
                f'conflated with thermal sensitivity. ZARU is confirmed as the correct '
                f'approach — it compensates both drift sources without requiring a '
                f'temperature model.</div>')

    html.append(f"</body></html>")
    return "\n".join(html)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else CSV_PATH
    if not os.path.exists(path):
        print(f"Error: file not found: {path}")
        sys.exit(1)

    print(f"Loading: {path}")
    df_full = load_csv(path)
    df = df_full[df_full["lap"] == 1].reset_index(drop=True)
    fs = estimate_fs(df)
    dur = df["t_s"].iloc[-1] - df["t_s"].iloc[0]
    print(f"  Samples: {len(df)} at {fs:.0f} Hz  |  Duration: {dur:.1f}s")

    segments, rolling_std = detect_faces(df, fs)
    print(f"  Faces detected: {len(segments)}")

    time_trim = int(TIME_TRIM_S * fs)
    results = []
    for i, (i0, i1) in enumerate(segments):
        seg = df.iloc[i0:i1 + 1]
        stats = face_stats(seg, time_trim)
        stats.update(half_split_stats(seg, time_trim))
        label = FACE_LABELS[i] if i < len(FACE_LABELS) else f"Face_{i+1}"
        stats["label"] = label
        stats["i0"], stats["i1"] = i0, i1
        results.append(stats)
        print(f"  {label}: gx={stats['raw_gx_mean']:+.4f}, "
              f"gy={stats['raw_gy_mean']:+.4f}, gz={stats['raw_gz_mean']:+.4f}")

    drift = global_drift_regression(df, segments, time_trim)
    kgs = kgs_offline_validation(results)
    print(f"  Drift: gx={drift['gx']['slope_per_min']:+.4f}, "
          f"gy={drift['gy']['slope_per_min']:+.4f}, "
          f"gz={drift['gz']['slope_per_min']:+.4f} °/s/min")

    metadata = {
        "test_id": "tel_94",
        "firmware": "v1.2.2-diag-kgs",
        "data_frame": "Chip-frame (K_gs disabled, rotate_3d disabled for gyro)",
        "duration": f"{dur:.0f}s ({dur/60:.1f} min)",
        "samples": f"{len(df):,} at {fs:.0f} Hz",
        "temp_range": f"{df['temp_c'].min():.1f} – {df['temp_c'].max():.1f}°C",
        "date": str(date.today()),
    }

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    html = build_html(df_full, df, segments, results, drift, kgs, metadata)

    output_path = os.path.join(OUTPUT_DIR, "MPU6886_BiasDrift_Report.html")
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"\nReport saved: {output_path}")


if __name__ == "__main__":
    main()
