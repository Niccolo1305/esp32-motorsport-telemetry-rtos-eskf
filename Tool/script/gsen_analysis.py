"""
gsen_analysis.py — G-Sensitivity Test Analysis & K_gs Validation

Analizza il CSV prodotto da bin_to_csv.py durante il test a 6 posizioni statiche.
Tre modalità:
  - Pre-K_gs (default): misura la G-sensitivity grezza (dati chip-frame)
  - Post-K_gs (--validate): verifica che la correzione K_gs nel firmware riduca
    lo spread inter-faccia del giroscopio.
  - Offline (--validate-offline): per dati da firmware v1.2.2-diag-kgs (K_gs e
    rotate_3d disabilitate, raw_ax/ay/az = accel chip-frame). Applica K_gs in
    Python e confronta spread prima/dopo su dati indipendenti.

Usage:
  python Tool/gsen_analysis.py                                        # analisi grezza
  python Tool/gsen_analysis.py path/to/gsen_test.csv                  # analisi grezza
  python Tool/gsen_analysis.py --validate post_kgs.csv                # validazione firmware
  python Tool/gsen_analysis.py --validate-offline diag_kgs_test.csv   # validazione offline
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
MIN_FACE_S     = 10.0             # [s] segmento minimo per essere considerato una faccia
FACE_LABELS    = ["Z_up", "Z_down", "X_up", "X_down", "Y_up", "Y_down"]
# ──────────────────────────────────────────────────────────────────────────────


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path, encoding="latin-1")
    rename = {
        "t_ms (ms)":    "t_ms",
        "ax (G)":       "ax",
        "ay (G)":       "ay",
        "az (G)":       "az",
        "gx (°/s)":     "gx",
        "gy (°/s)":     "gy",
        "gz (°/s)":     "gz",
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


def detect_faces(df: pd.DataFrame, fs: float, use_gyro: bool = False):
    """
    Rileva le N_FACES finestre statiche.
    - use_gyro=False (default): rolling std di |a| (pre-K_gs, chip-frame data)
    - use_gyro=True (validate mode): rolling std di |g| (post-Madgwick, gravity removed)
      Works because during face rotations gyro axes spike, while at rest they are ~0.
    Ritorna lista di (i_start, i_end) ordinata per tempo e il rolling_std.
    """
    win = max(1, int(MOTION_WIN_S * fs))
    if use_gyro:
        g_norm = np.sqrt(df["raw_gx"] ** 2 + df["raw_gy"] ** 2 + df["raw_gz"] ** 2)
        rolling_std = g_norm.rolling(win, center=True, min_periods=1).std().fillna(0)
        threshold = 1.0  # °/s — gyro std during rotation is >> 1
    else:
        rolling_std = df["a_norm"].rolling(win, center=True, min_periods=1).std().fillna(0)
        threshold = MOTION_STD_THR
    is_still = rolling_std < threshold

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

    # Filter out segments shorter than MIN_FACE_S
    min_samples = max(1, int(MIN_FACE_S * fs))
    segments = [(s, e) for s, e in segments if (e - s + 1) >= min_samples]

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


def print_table(results: list, validate_mode: bool = False):
    W = 106
    print()
    print("=" * W)
    if validate_mode:
        print("  G-SENSITIVITY TEST — Risultati per faccia (vehicle-frame, post-K_gs)")
    else:
        print("  G-SENSITIVITY TEST — Risultati per faccia (dati chip-frame, pre-rotate_3d)")
    print("=" * W)

    # Intestazione giroscopio
    gyro_label = "GIROSCOPIO (raw_g*: post-K_gs, pre-ZARU)" if validate_mode else "GIROSCOPIO"
    print(f"\n{gyro_label:}")
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

    if not validate_mode:
        # Intestazione accelerometro (solo in raw mode — in validate mode
        # raw_ax/ay/az sono linear accel con gravita rimossa, non utili)
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


def validate_kgs_offline(results: list):
    """
    Offline K_gs validation for data from v1.2.2-diag-kgs firmware.
    Input: chip-frame data WITHOUT K_gs applied (raw_gx/gy/gz = chip-frame gyro,
           raw_ax/ay/az = chip-frame accel with gravity).
    Applies K_gs matrix in Python using per-face mean accel as gravity proxy,
    then compares spread before vs after correction.

    K_gs matrix [(deg/s)/g], chip frame:
                az(j=0)    ax(j=1)    ay(j=2)
      gx(i=0): +0.6056    +0.7969    -0.1346
      gy(i=1): +0.0712    +0.3539    -0.0908
      gz(i=2): -0.1547    -0.0372    -0.0003

    Correction: g_corr = g_raw - K_gs @ (a_face - a_boot)
    where a_boot = accel at boot face (first face = Z_up by convention).
    """
    K_gs = np.array([
        [+0.6056, +0.7969, -0.1346],  # gx row: az, ax, ay
        [+0.0712, +0.3539, -0.0908],  # gy row
        [-0.1547, -0.0372, -0.0003],  # gz row
    ])

    # Boot face = first face (sensor boots on this orientation)
    a_boot = np.array([
        results[0]["raw_ax_mean"],
        results[0]["raw_ay_mean"],
        results[0]["raw_az_mean"],
    ])

    W = 106
    print()
    print("=" * W)
    print("  K_gs OFFLINE VALIDATION — chip-frame data, K_gs applied in Python")
    print("=" * W)
    print(f"\n  Boot face: {results[0]['label']}  "
          f"a_boot = [{a_boot[0]:+.4f}, {a_boot[1]:+.4f}, {a_boot[2]:+.4f}] g")

    # Compute corrected gyro means per face
    corrected = []
    for r in results:
        a_face = np.array([r["raw_ax_mean"], r["raw_ay_mean"], r["raw_az_mean"]])
        da = a_face - a_boot  # [dax, day, daz] — but K_gs columns are [az, ax, ay]
        # K_gs column order is az(0), ax(1), ay(2); da is [ax, ay, az]
        da_kgs = np.array([da[2], da[0], da[1]])  # reorder to [daz, dax, day]
        correction = K_gs @ da_kgs
        corr_gx = r["raw_gx_mean"] - correction[0]
        corr_gy = r["raw_gy_mean"] - correction[1]
        corr_gz = r["raw_gz_mean"] - correction[2]
        corrected.append({
            "label": r["label"],
            "uncertain": r["uncertain"],
            "raw_gx": r["raw_gx_mean"], "raw_gy": r["raw_gy_mean"], "raw_gz": r["raw_gz_mean"],
            "corr_gx": corr_gx, "corr_gy": corr_gy, "corr_gz": corr_gz,
            "da": da_kgs,
        })

    # Per-face detail table
    print(f"\n  {'Faccia':<10} │ {'raw_gx':>8} {'raw_gy':>8} {'raw_gz':>8} │ "
          f"{'corr_gx':>8} {'corr_gy':>8} {'corr_gz':>8} │ "
          f"{'da_az':>7} {'da_ax':>7} {'da_ay':>7}")
    print(f"  {'':10} │ {'(°/s)':>8} {'(°/s)':>8} {'(°/s)':>8} │ "
          f"{'(°/s)':>8} {'(°/s)':>8} {'(°/s)':>8} │ "
          f"{'(g)':>7} {'(g)':>7} {'(g)':>7}")
    print("  " + "-" * (W - 2))
    for c in corrected:
        flag = " *" if c["uncertain"] else ""
        print(f"  {c['label']+flag:<10} │ "
              f"{c['raw_gx']:>+8.4f} {c['raw_gy']:>+8.4f} {c['raw_gz']:>+8.4f} │ "
              f"{c['corr_gx']:>+8.4f} {c['corr_gy']:>+8.4f} {c['corr_gz']:>+8.4f} │ "
              f"{c['da'][0]:>+7.3f} {c['da'][1]:>+7.3f} {c['da'][2]:>+7.3f}")

    # Spread comparison
    print(f"\n  {'Asse':<6} │ {'spread RAW':>12} │ {'spread CORR':>12} │ "
          f"{'riduzione':>10} │ {'Verdetto'}")
    print("  " + "-" * 80)

    all_ok = True
    for axis in ["gx", "gy", "gz"]:
        raw_means = [c[f"raw_{axis}"] for c in corrected]
        corr_means = [c[f"corr_{axis}"] for c in corrected]
        raw_spread = max(raw_means) - min(raw_means)
        corr_spread = max(corr_means) - min(corr_means)
        if raw_spread > 0:
            reduction = (1.0 - corr_spread / raw_spread) * 100
        else:
            reduction = 0.0

        if corr_spread < 0.15:
            verdict = "OTTIMO"
        elif corr_spread < 0.30:
            verdict = "OK"
        elif corr_spread < 0.50:
            verdict = "MARGINALE"
            all_ok = False
        else:
            verdict = "FALLITO"
            all_ok = False
        print(f"  {axis:<6} │ {raw_spread:>10.4f} °/s │ {corr_spread:>10.4f} °/s │ "
              f"{reduction:>+9.1f}% │ {verdict}")

    print()
    if all_ok:
        print("  RISULTATO: K_gs riduce lo spread su dati indipendenti — matrice valida.")
    else:
        print("  RISULTATO: K_gs non riduce sufficientemente lo spread.")
        print("  Azioni possibili:")
        print("    - Ricalcolare K_gs con kgs_calc.py usando i dati di questo test")
        print("    - Verificare drift termico nella tabella temperatura")
        print("    - Controllare ordine facce e orientazione")
    print("=" * W)


def validate_kgs(results: list):
    """
    K_gs validation: if the matrix is correct, gyro means across all faces
    should be nearly identical (G-sensitivity removed). The spread (max-min)
    of the per-face means is the metric: smaller = better correction.

    Uses raw_gx/gy/gz (post-K_gs, pre-ZARU) to avoid ZARU masking the result.
    """
    W = 106
    print()
    print("=" * W)
    print("  K_gs VALIDATION — Spread inter-faccia (dati vehicle-frame, post-K_gs, pre-ZARU)")
    print("=" * W)

    axes = [("gx", "raw_gx"), ("gy", "raw_gy"), ("gz", "raw_gz")]

    print(f"\n  {'Asse':<6} │ {'min(mean)':>10} {'max(mean)':>10} │ "
          f"{'spread':>10} │ {'Verdetto'}")
    print("  " + "-" * 70)

    all_ok = True
    for label, col in axes:
        means = [r[f"{col}_mean"] for r in results]
        mn, mx = min(means), max(means)
        spread = mx - mn
        # Threshold: spread < 0.3 °/s is good (pre-K_gs was 1-2 °/s typically)
        if spread < 0.15:
            verdict = "OTTIMO — spread < 0.15 °/s"
        elif spread < 0.30:
            verdict = "OK — spread < 0.30 °/s"
        elif spread < 0.50:
            verdict = "MARGINALE — possibile errore nella matrice"
            all_ok = False
        else:
            verdict = "FALLITO — K_gs probabilmente errata o contaminata"
            all_ok = False
        print(f"  {label:<6} │ {mn:>+10.4f} {mx:>+10.4f} │ {spread:>10.4f} │ {verdict}")

    # Per-face detail
    print(f"\n  {'Faccia':<10} │ {'raw_gx':>10} {'raw_gy':>10} {'raw_gz':>10} │ "
          f"{'gx(EMA)':>10} {'gy(EMA)':>10} {'gz(EMA)':>10}")
    print(f"  {'':10} │ {'(°/s)':>10} {'(°/s)':>10} {'(°/s)':>10} │ "
          f"{'(°/s)':>10} {'(°/s)':>10} {'(°/s)':>10}")
    print("  " + "-" * 90)
    for r in results:
        flag = " *" if r["uncertain"] else ""
        # gx/gy/gz (EMA+ZARU) columns may not exist in old CSVs
        gx_ema = r.get("gx_mean", float("nan"))
        gy_ema = r.get("gy_mean", float("nan"))
        gz_ema = r.get("gz_mean", float("nan"))
        print(f"  {r['label']+flag:<10} │ "
              f"{r['raw_gx_mean']:>+10.4f} {r['raw_gy_mean']:>+10.4f} {r['raw_gz_mean']:>+10.4f} │ "
              f"{gx_ema:>+10.4f} {gy_ema:>+10.4f} {gz_ema:>+10.4f}")

    print()
    if all_ok:
        print("  RISULTATO: matrice K_gs valida — spread inter-faccia accettabile.")
    else:
        print("  RISULTATO: matrice K_gs potenzialmente errata.")
        print("  Possibili cause:")
        print("    - Drift di bias durante il test originale (instabilita bias, non termico)")
        print("    - Rampa termica durante la misura (verificare colonna ΔT)")
        print("    - Errore nell'ordine o orientazione delle facce")
        print("  Azione: ripetere il 6-face test a temperatura stabile e ricalcolare K_gs.")
    print("=" * W)


def face_stats_validate(seg: pd.DataFrame, time_trim: int) -> dict:
    """
    Come face_stats ma include anche gx/gy/gz (EMA+ZARU) se presenti.
    """
    if len(seg) > 2 * time_trim + 10:
        core = seg.iloc[time_trim:-time_trim]
    else:
        core = seg

    n = len(core)
    results = {"n": n}

    for col in ["raw_gx", "raw_gy", "raw_gz", "raw_ax", "raw_ay", "raw_az"]:
        vals = core[col].values
        results[f"{col}_mean"] = trim_mean(vals, VALUE_TRIM)
        results[f"{col}_std"] = float(np.std(vals))

    # EMA+ZARU columns (v1.2.1+)
    for col in ["gx", "gy", "gz"]:
        if col in core.columns:
            vals = core[col].values
            results[f"{col}_mean"] = trim_mean(vals, VALUE_TRIM)
            results[f"{col}_std"] = float(np.std(vals))

    results["t_ini"] = float(core["temp_c"].iloc[0])
    results["t_fin"] = float(core["temp_c"].iloc[-1])
    results["delta_t"] = results["t_fin"] - results["t_ini"]

    return results


def main():
    # Parse args
    validate_mode = "--validate" in sys.argv
    validate_offline = "--validate-offline" in sys.argv
    # --lap N: filter by lap column value (default: 1 in validate mode)
    lap_filter = None
    args_clean = []
    skip_next = False
    for i, a in enumerate(sys.argv[1:]):
        if skip_next:
            skip_next = False
            continue
        if a in ("--validate", "--validate-offline"):
            continue
        if a == "--lap":
            if i + 1 < len(sys.argv) - 1:
                lap_filter = int(sys.argv[i + 2])
                skip_next = True
            continue
        args_clean.append(a)
    path = args_clean[0] if args_clean else CSV_PATH

    # Default: in validate mode, filter lap=1 (warm-up data excluded)
    if (validate_mode or validate_offline) and lap_filter is None:
        lap_filter = 1

    if not os.path.exists(path):
        print(f"Errore: file non trovato: {path}")
        sys.exit(1)

    print(f"\nCaricamento: {path}")
    if validate_offline:
        print("  Modalita: VALIDAZIONE OFFLINE K_gs (chip-frame, K_gs applicata in Python)")
    elif validate_mode:
        print("  Modalita: VALIDAZIONE K_gs (post-correzione)")
    df = load_csv(path)

    # Filter by lap if requested
    if lap_filter is not None and "lap" in df.columns:
        n_before = len(df)
        df = df[df["lap"] == lap_filter].reset_index(drop=True)
        print(f"  Filtro lap={lap_filter}: {n_before} -> {len(df)} campioni")

    # Sanity filter: remove rows with corrupted temperature (outside 0-100°C)
    if "temp_c" in df.columns:
        sane = (df["temp_c"] > 0) & (df["temp_c"] < 100)
        n_bad = (~sane).sum()
        if n_bad > 0:
            df = df[sane].reset_index(drop=True)
            print(f"  Rimossi {n_bad} campioni con temperatura fuori range (0-100°C)")
    fs = estimate_fs(df)
    dur = df["t_s"].iloc[-1] - df["t_s"].iloc[0]
    print(f"  Sample rate: {fs:.1f} Hz  |  Durata: {dur:.1f} s  |  Campioni: {len(df)}")

    segments, rolling_std = detect_faces(df, fs, use_gyro=validate_mode)
    n_found = len(segments)

    if n_found == 0:
        print("ERRORE: nessun segmento statico rilevato. "
              "Verifica MOTION_STD_THR o controlla il CSV.")
        sys.exit(1)
    if n_found < N_FACES:
        print(f"ATTENZIONE: rilevate {n_found} facce su {N_FACES} attese. "
              f"Considera di abbassare MOTION_STD_THR ({MOTION_STD_THR}).")

    time_trim = int(TIME_TRIM_S * fs)
    stats_fn = face_stats_validate if validate_mode else face_stats
    results = []
    for i, (i0, i1) in enumerate(segments):
        seg = df.iloc[i0:i1 + 1]
        stats = stats_fn(seg, time_trim)
        label = FACE_LABELS[i] if i < len(FACE_LABELS) else f"Face_{i+1}"
        stats["label"] = label
        stats["uncertain"] = (i == n_found - 1)
        stats["i0"] = i0
        stats["i1"] = i1
        results.append(stats)

    print_table(results, validate_mode and not validate_offline)

    if validate_offline:
        validate_kgs_offline(results)
    elif validate_mode:
        validate_kgs(results)

    png_path = os.path.splitext(path)[0] + "_gsen.png"
    plot(df, segments, results, rolling_std, fs, png_path)


if __name__ == "__main__":
    main()
