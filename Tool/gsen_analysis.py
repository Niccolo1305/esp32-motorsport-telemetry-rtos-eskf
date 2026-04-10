"""
gsen_analysis.py — G-Sensitivity Test Analysis
Analizza il CSV prodotto da bin_to_csv.py durante il test a 6 posizioni statiche.

Usage:
  python Tool/gsen_analysis.py                        # usa path default
  python Tool/gsen_analysis.py path/to/gsen_test.csv
"""

import sys
import os
import numpy as np
import pandas as pd
from scipy.stats import trim_mean
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Parametri configurabili ────────────────────────────────────────────────────
CSV_PATH       = "gsen_test.csv"  # path default se non passato da riga di comando
N_FACES        = 6                # numero di facce da rilevare
TIME_TRIM_S    = 30.0             # [s] da scartare a inizio e fine di ogni finestra
VALUE_TRIM     = 0.05             # frazione trim_mean per parte (5% su ogni lato)
MOTION_WIN_S   = 2.0              # [s] finestra rolling per rilevare rotazione
MOTION_STD_THR = 0.015            # [g] soglia std rolling: sopra = in movimento
FACE_LABELS    = ["Z_up", "Z_down", "X_up", "X_down", "Y_up", "Y_down"]
# ──────────────────────────────────────────────────────────────────────────────


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, encoding="latin-1")
    rename = {
        "t_ms (ms)":    "t_ms",
        "raw_ax (G)":   "raw_ax",
        "raw_ay (G)":   "raw_ay",
        "raw_az (G)":   "raw_az",
        "raw_gx (°/s)": "raw_gx",
        "raw_gy (°/s)": "raw_gy",
        "raw_gz (°/s)": "raw_gz",
        "temp_c (°C)":  "temp_c",
    }
    df = df.rename(columns=rename)
    df["t_s"] = df["t_ms"] / 1000.0
    df["a_norm"] = np.sqrt(df["raw_ax"] ** 2 + df["raw_ay"] ** 2 + df["raw_az"] ** 2)
    return df


def estimate_fs(df: pd.DataFrame) -> float:
    dt = np.median(np.diff(df["t_s"].values))
    return 1.0 / dt if dt > 0 else 50.0


def detect_faces(df: pd.DataFrame, fs: float):
    """
    Rileva le N_FACES finestre statiche usando la rolling std di |a|.
    Ritorna lista di (i_start, i_end) ordinata per tempo e il rolling_std.
    """
    win = max(1, int(MOTION_WIN_S * fs))
    rolling_std = df["a_norm"].rolling(win, center=True, min_periods=1).std().fillna(0)
    is_still = rolling_std < MOTION_STD_THR

    # Raggruppa campioni still contigui in segmenti
    segments = []
    in_seg = False
    start = 0
    for i, still in enumerate(is_still):
        if still and not in_seg:
            start = i
            in_seg = True
        elif not still and in_seg:
            segments.append((start, i - 1))
            in_seg = False
    if in_seg:
        segments.append((start, len(df) - 1))

    if not segments:
        return [], rolling_std

    # Tieni i N_FACES più lunghi e riordina per posizione temporale
    segments.sort(key=lambda s: s[1] - s[0], reverse=True)
    segments = segments[:N_FACES]
    segments.sort(key=lambda s: s[0])

    return segments, rolling_std


def face_stats(seg: pd.DataFrame, time_trim: int) -> dict:
    """
    Applica time trim + trim_mean su una finestra segmento.
    Ritorna dizionario con medie, std e dati temperatura.
    """
    # Time trim
    if len(seg) > 2 * time_trim + 10:
        core = seg.iloc[time_trim:-time_trim]
    else:
        core = seg  # segmento troppo corto, usa tutto

    n = len(core)
    results = {"n": n}

    for col in ["raw_gx", "raw_gy", "raw_gz", "raw_ax", "raw_ay", "raw_az"]:
        vals = core[col].values
        results[f"{col}_mean"] = trim_mean(vals, VALUE_TRIM)
        results[f"{col}_std"] = float(np.std(vals))

    results["t_ini"] = float(core["temp_c"].iloc[0])
    results["t_fin"] = float(core["temp_c"].iloc[-1])
    results["delta_t"] = results["t_fin"] - results["t_ini"]

    return results


def print_table(results: list):
    W = 106
    print()
    print("=" * W)
    print("  G-SENSITIVITY TEST — Risultati per faccia (dati chip-frame, pre-rotate_3d)")
    print("=" * W)

    # Intestazione giroscopio
    print(f"\n{'GIROSCOPIO':}")
    print(f"  {'Faccia':<10} {'N':>6} │ "
          f"{'gx_mean':>10} {'σ_gx':>8} │ "
          f"{'gy_mean':>10} {'σ_gy':>8} │ "
          f"{'gz_mean':>10} {'σ_gz':>8}")
    print(f"  {'':10} {'':>6} │ "
          f"{'(°/s)':>10} {'(°/s)':>8} │ "
          f"{'(°/s)':>10} {'(°/s)':>8} │ "
          f"{'(°/s)':>10} {'(°/s)':>8}")
    print("  " + "-" * (W - 2))
    for r in results:
        flag = " *" if r["uncertain"] else ""
        print(f"  {r['label']+flag:<10} {r['n']:>6} │ "
              f"{r['raw_gx_mean']:>+10.4f} {r['raw_gx_std']:>8.4f} │ "
              f"{r['raw_gy_mean']:>+10.4f} {r['raw_gy_std']:>8.4f} │ "
              f"{r['raw_gz_mean']:>+10.4f} {r['raw_gz_std']:>8.4f}")

    # Intestazione accelerometro
    print(f"\n{'ACCELEROMETRO (conferma orientazione)':}")
    print(f"  {'Faccia':<10} │ "
          f"{'ax_mean':>10} {'σ_ax':>8} │ "
          f"{'ay_mean':>10} {'σ_ay':>8} │ "
          f"{'az_mean':>10} {'σ_az':>8}")
    print(f"  {'':10} │ "
          f"{'(g)':>10} {'(g)':>8} │ "
          f"{'(g)':>10} {'(g)':>8} │ "
          f"{'(g)':>10} {'(g)':>8}")
    print("  " + "-" * (W - 2))
    for r in results:
        flag = " *" if r["uncertain"] else ""
        print(f"  {r['label']+flag:<10} │ "
              f"{r['raw_ax_mean']:>+10.4f} {r['raw_ax_std']:>8.4f} │ "
              f"{r['raw_ay_mean']:>+10.4f} {r['raw_ay_std']:>8.4f} │ "
              f"{r['raw_az_mean']:>+10.4f} {r['raw_az_std']:>8.4f}")

    # Temperatura
    print(f"\n{'TEMPERATURA':}")
    print(f"  {'Faccia':<10} │ {'T_ini':>7} {'T_fin':>7} {'ΔT':>6}  {'Nota'}")
    print("  " + "-" * 50)
    for r in results:
        flag = " *" if r["uncertain"] else ""
        note = ""
        if abs(r["delta_t"]) > 1.0:
            note = "⚠ drift termico significativo"
        elif abs(r["delta_t"]) > 0.3:
            note = "~ lieve drift"
        print(f"  {r['label']+flag:<10} │ "
              f"{r['t_ini']:>7.2f} {r['t_fin']:>7.2f} {r['delta_t']:>+6.2f}°C  {note}")

    print()
    print("  * = ultima faccia (cavo USB/Grove, potenzialmente meno stabile)")
    print(f"  Parametri: time_trim={TIME_TRIM_S}s, value_trim={VALUE_TRIM*100:.0f}% per parte")
    print("=" * W)


def plot(df: pd.DataFrame, segments: list, results: list, rolling_std: pd.Series,
         fs: float, output_path: str):
    colors = plt.cm.tab10.colors
    face_colors = [colors[i % 10] for i in range(len(segments))]
    time_trim = int(TIME_TRIM_S * fs)
    t = df["t_s"]

    fig, axes = plt.subplots(4, 1, figsize=(15, 11), sharex=True)
    fig.suptitle("G-Sensitivity Test — Chip Frame Data (pre-rotate_3d, pre-Madgwick)",
                 fontsize=13, fontweight="bold")

    def shade(ax):
        for i, ((i0, i1), r) in enumerate(zip(segments, results)):
            seg = df.iloc[i0:i1 + 1]
            t0, t1 = seg["t_s"].iloc[0], seg["t_s"].iloc[-1]
            ax.axvspan(t0, t1, alpha=0.10, color=face_colors[i])
            trim_s = time_trim / fs
            ax.axvline(t0 + trim_s, color=face_colors[i], lw=0.9, ls="--", alpha=0.7)
            ax.axvline(t1 - trim_s, color=face_colors[i], lw=0.9, ls="--", alpha=0.7)
            # Etichetta faccia
            ax.text((t0 + t1) / 2, ax.get_ylim()[1] * 0.92, r["label"],
                    ha="center", va="top", fontsize=7.5, color=face_colors[i],
                    fontweight="bold")

    # 1) |a| nel tempo
    axes[0].plot(t, df["a_norm"], lw=0.5, color="steelblue", label="|a|")
    axes[0].axhline(1.0, color="k", lw=0.7, ls=":", label="1g")
    axes[0].set_ylabel("|a| (g)")
    axes[0].set_title("Norma accelerometro — transitori visibili come picchi")
    axes[0].legend(loc="upper right", fontsize=8)

    # 2) Giroscopio
    for col, c, lbl in zip(["raw_gx", "raw_gy", "raw_gz"],
                            ["crimson", "forestgreen", "royalblue"],
                            ["gx", "gy", "gz"]):
        axes[1].plot(t, df[col], lw=0.4, color=c, alpha=0.8, label=lbl)
    axes[1].axhline(0, color="k", lw=0.6, ls=":")
    axes[1].set_ylabel("Gyro (°/s)")
    axes[1].set_title("Giroscopio chip-frame — bias per faccia = segnale K_gs")
    axes[1].legend(loc="upper right", fontsize=8)

    # 3) Accelerometro
    for col, c, lbl in zip(["raw_ax", "raw_ay", "raw_az"],
                            ["crimson", "forestgreen", "royalblue"],
                            ["ax", "ay", "az"]):
        axes[2].plot(t, df[col], lw=0.4, color=c, alpha=0.8, label=lbl)
    axes[2].set_ylabel("Accel (g)")
    axes[2].set_title("Accelerometro chip-frame — asse dominante ≈ ±1g conferma orientazione")
    axes[2].legend(loc="upper right", fontsize=8)

    # 4) Temperatura
    axes[3].plot(t, df["temp_c"], lw=0.8, color="darkorange", label="temp_c")
    axes[3].set_ylabel("Temp (°C)")
    axes[3].set_title("Temperatura IMU — ΔT per faccia indica drift termico residuo")
    axes[3].set_xlabel("Tempo (s)")
    axes[3].legend(loc="upper right", fontsize=8)

    # Ombreggia segmenti dopo aver fissato ylim
    for ax in axes:
        ax.relim()
        ax.autoscale_view()
        shade(ax)

    # Legenda globale facce
    patches = [mpatches.Patch(color=face_colors[i], alpha=0.6,
                              label=results[i]["label"] + (" *" if results[i]["uncertain"] else ""))
               for i in range(len(results))]
    fig.legend(handles=patches, loc="upper right", bbox_to_anchor=(0.995, 0.97),
               fontsize=9, title="Facce", title_fontsize=9, framealpha=0.8)

    plt.tight_layout(rect=[0, 0, 0.93, 1])
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"  Plot salvato: {output_path}")
    plt.show()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else CSV_PATH
    if not os.path.exists(path):
        print(f"Errore: file non trovato: {path}")
        sys.exit(1)

    print(f"\nCaricamento: {path}")
    df = load_csv(path)
    fs = estimate_fs(df)
    dur = df["t_s"].iloc[-1] - df["t_s"].iloc[0]
    print(f"  Sample rate: {fs:.1f} Hz  |  Durata: {dur:.1f} s  |  Campioni: {len(df)}")

    segments, rolling_std = detect_faces(df, fs)
    n_found = len(segments)

    if n_found == 0:
        print("ERRORE: nessun segmento statico rilevato. "
              "Verifica MOTION_STD_THR o controlla il CSV.")
        sys.exit(1)
    if n_found < N_FACES:
        print(f"ATTENZIONE: rilevate {n_found} facce su {N_FACES} attese. "
              f"Considera di abbassare MOTION_STD_THR ({MOTION_STD_THR}).")

    time_trim = int(TIME_TRIM_S * fs)
    results = []
    for i, (i0, i1) in enumerate(segments):
        seg = df.iloc[i0:i1 + 1]
        stats = face_stats(seg, time_trim)
        label = FACE_LABELS[i] if i < len(FACE_LABELS) else f"Face_{i+1}"
        stats["label"] = label
        stats["uncertain"] = (i == n_found - 1)
        stats["i0"] = i0
        stats["i1"] = i1
        results.append(stats)

    print_table(results)

    png_path = os.path.splitext(path)[0] + "_gsen.png"
    plot(df, segments, results, rolling_std, fs, png_path)


if __name__ == "__main__":
    main()
