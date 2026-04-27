#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""
CAN Signal Correlator — Phase 2 reverse-engineering tool.

Cross-correlates unknown CAN bus signals with known reference signals
(GPS speed, IMU accel/gyro) from the telemetry system to identify
which CAN ID and byte layout corresponds to which physical signal.

Usage:
    python can_signal_correlator.py <can_log.csv> <telemetry.csv>
    python can_signal_correlator.py <can_log.csv> <telemetry.csv> --target-id 0x180
    python can_signal_correlator.py <can_log.csv> <telemetry.csv> --plot

Requires both captures to be from the same session (synchronized start).
"""

import csv
import sys
import argparse
from collections import defaultdict
from typing import List, Tuple, Optional

import numpy as np


def parse_can_log(filename: str) -> dict:
    """Parse CAN sniffer CSV into per-ID time-series."""
    ids = defaultdict(lambda: {'timestamps': [], 'data': []})

    with open(filename, encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 3 or row[0].startswith('#') or row[0].startswith('timestamp'):
                continue
            try:
                ts = int(row[0])
                can_id = row[1].strip()
                dlc = int(row[2])
            except (ValueError, IndexError):
                continue

            data_bytes = []
            for b in row[3:3 + dlc]:
                try:
                    data_bytes.append(int(b, 16))
                except ValueError:
                    data_bytes.append(0)

            ids[can_id]['timestamps'].append(ts)
            ids[can_id]['data'].append(data_bytes)

    return dict(ids)


def parse_telemetry_csv(filename: str) -> dict:
    """Parse telemetry CSV and extract reference signals."""
    signals = {
        'timestamps': [],
        'gps_speed_kmh': [],
        'accel_x': [],
        'accel_y': [],
        'gyro_z': [],
    }

    with open(filename, encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ts = float(row.get('timestamp_us', row.get('timestamp', 0)))
                signals['timestamps'].append(ts)
                signals['gps_speed_kmh'].append(
                    float(row.get('gps_sog_kmh', row.get('gps_speed', 0))))
                signals['accel_x'].append(
                    float(row.get('ax', row.get('accel_x', 0))))
                signals['accel_y'].append(
                    float(row.get('ay', row.get('accel_y', 0))))
                signals['gyro_z'].append(
                    float(row.get('gz', row.get('gyro_z', 0))))
            except (ValueError, KeyError):
                continue

    for key in signals:
        signals[key] = np.array(signals[key])

    return signals


def find_signal_in_frame(can_data: List[List[int]],
                         can_timestamps: np.ndarray,
                         reference_signal: np.ndarray,
                         reference_timestamps: np.ndarray,
                         ref_name: str) -> List[Tuple[float, str]]:
    """
    Try all byte combinations, encodings, and scale factors to find which
    signal in the CAN frame best correlates with a reference signal.

    Returns list of (correlation, config_description) sorted descending.
    """
    if len(can_data) < 20 or len(reference_signal) < 20:
        return []

    # Interpolate reference to CAN timestamps
    ref_interp = np.interp(can_timestamps, reference_timestamps, reference_signal)

    results = []
    dlc = max(len(d) for d in can_data)

    # Single byte (unsigned 8-bit)
    for i in range(dlc):
        values = np.array([d[i] if i < len(d) else 0 for d in can_data],
                          dtype=np.float64)

        for offset in [0, -40, -50, -128]:
            for scale in [1.0, 0.1, 0.01, 100.0 / 255.0]:
                signal = (values + offset) * scale
                if np.std(signal) < 1e-6:
                    continue
                corr = abs(np.corrcoef(signal, ref_interp)[0, 1])
                if np.isnan(corr):
                    continue
                if corr > 0.7:
                    results.append((
                        corr,
                        f"B{i} uint8, offset={offset}, scale={scale:.4f} "
                        f"-> {ref_name}"
                    ))

    # Byte pairs (uint16 / int16, big-endian)
    for i in range(dlc - 1):
        raw_unsigned = np.array(
            [(d[i] << 8) | d[i + 1] if i + 1 < len(d) else 0 for d in can_data],
            dtype=np.float64)
        raw_signed = np.array(
            [np.int16((d[i] << 8) | d[i + 1]) if i + 1 < len(d) else 0
             for d in can_data], dtype=np.float64)

        for values, enc_label in [(raw_unsigned, "uint16_BE"),
                                   (raw_signed, "int16_BE")]:
            for scale in [1.0, 0.1, 0.01, 0.25, 1.0 / 4.0, 0.001]:
                signal = values * scale
                if np.std(signal) < 1e-6:
                    continue
                corr = abs(np.corrcoef(signal, ref_interp)[0, 1])
                if np.isnan(corr):
                    continue
                if corr > 0.7:
                    results.append((
                        corr,
                        f"B{i}-B{i+1} {enc_label}, scale={scale:.4f} "
                        f"-> {ref_name}"
                    ))

    results.sort(key=lambda x: x[0], reverse=True)
    return results[:5]


def analyze_id(can_id: str, can_info: dict, telemetry: dict) -> dict:
    """Analyze a single CAN ID against all reference signals."""
    can_ts = np.array(can_info['timestamps'], dtype=np.float64)

    ref_signals = {
        'GPS Speed [km/h]': telemetry['gps_speed_kmh'],
        'Accel X [g]': telemetry['accel_x'],
        'Accel Y [g]': telemetry['accel_y'],
        'Gyro Z [deg/s]': telemetry['gyro_z'],
    }
    ref_ts = telemetry['timestamps']

    results = {}
    for ref_name, ref_signal in ref_signals.items():
        if np.std(ref_signal) < 1e-6:
            continue
        matches = find_signal_in_frame(
            can_info['data'], can_ts, ref_signal, ref_ts, ref_name)
        if matches:
            results[ref_name] = matches

    return results


def main():
    parser = argparse.ArgumentParser(
        description='Cross-correlate CAN signals with IMU/GPS reference')
    parser.add_argument('can_log', help='CAN sniffer CSV log')
    parser.add_argument('telemetry_csv', help='Telemetry CSV (from bin_to_csv.py)')
    parser.add_argument('--target-id', help='Analyze only this CAN ID (e.g. 0x180)')
    parser.add_argument('--min-corr', type=float, default=0.8,
                        help='Minimum correlation to report (default: 0.8)')
    parser.add_argument('--plot', action='store_true',
                        help='Plot best matches (requires matplotlib)')
    args = parser.parse_args()

    print(f"Loading CAN log: {args.can_log}")
    can_ids = parse_can_log(args.can_log)
    print(f"  {len(can_ids)} unique CAN IDs")

    print(f"Loading telemetry: {args.telemetry_csv}")
    telemetry = parse_telemetry_csv(args.telemetry_csv)
    print(f"  {len(telemetry['timestamps'])} samples")

    if len(telemetry['timestamps']) < 20:
        print("ERROR: telemetry CSV too short for correlation")
        sys.exit(1)

    # Filter to target ID if specified
    if args.target_id:
        target = args.target_id.strip()
        filtered = {k: v for k, v in can_ids.items()
                    if k.upper() == target.upper()}
        if not filtered:
            print(f"ERROR: CAN ID {target} not found in log")
            sys.exit(1)
        can_ids = filtered

    print(f"\nCorrelating {len(can_ids)} CAN IDs against reference signals...")
    print("=" * 80)

    best_matches = []

    for can_id in sorted(can_ids.keys()):
        info = can_ids[can_id]
        if len(info['data']) < 20:
            continue

        results = analyze_id(can_id, info, telemetry)
        if not results:
            continue

        has_good_match = False
        for ref_name, matches in results.items():
            for corr, config in matches:
                if corr >= args.min_corr:
                    if not has_good_match:
                        print(f"\n{can_id} ({len(info['data'])} frames):")
                        has_good_match = True
                    print(f"  corr={corr:.3f}  {config}")
                    best_matches.append((corr, can_id, config))

    if not best_matches:
        print(f"\nNo correlations above {args.min_corr} found.")
        print("Try lowering --min-corr or capturing with more dynamic activity.")
    else:
        print(f"\n{'='*80}")
        print("Top matches:")
        best_matches.sort(reverse=True)
        for corr, can_id, config in best_matches[:10]:
            print(f"  {can_id}: corr={corr:.3f}  {config}")

    if args.plot and best_matches:
        try:
            import matplotlib.pyplot as plt

            top_n = min(4, len(best_matches))
            fig, axes = plt.subplots(top_n, 1, figsize=(14, 3 * top_n),
                                     sharex=True)
            if top_n == 1:
                axes = [axes]

            for idx in range(top_n):
                _, can_id, config = best_matches[idx]
                info = can_ids[can_id]
                ts = np.array(info['timestamps'], dtype=np.float64)
                t0 = ts[0]
                ts_s = (ts - t0) / 1e6

                # Plot B0-B1 uint16 as a default view
                vals = [(d[0] << 8) | d[1] if len(d) >= 2 else 0
                        for d in info['data']]
                axes[idx].plot(ts_s, vals, linewidth=0.5)
                axes[idx].set_ylabel(f'{can_id}\n(raw B0-B1)')
                axes[idx].set_title(config, fontsize=9)
                axes[idx].grid(True, alpha=0.3)

            axes[-1].set_xlabel('Time [s]')
            fig.suptitle('Best CAN-Telemetry Correlations')
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("[WARN] matplotlib not installed — skipping plots")


if __name__ == '__main__':
    main()
