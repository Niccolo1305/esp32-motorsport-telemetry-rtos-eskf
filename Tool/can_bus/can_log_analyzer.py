#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""
CAN Bus Log Analyzer — Phase 1 post-capture analysis.

Reads CSV logs produced by the CAN sniffer firmware and produces:
  - Per-ID statistics: count, rate, DLC
  - Byte-level variation analysis (identifies dynamic fields)
  - Heuristic signal identification (RPM, wheels, steering, etc.)

Usage:
    python can_log_analyzer.py <capture.csv>
    python can_log_analyzer.py <capture.csv> --plot
"""

import csv
import sys
import argparse
from collections import defaultdict
from typing import Dict, List, Tuple


def parse_can_log(filename: str) -> Dict[str, dict]:
    """Parse a CAN sniffer CSV log and compute per-ID statistics."""
    ids = defaultdict(lambda: {
        'count': 0,
        'first_ts': None,
        'last_ts': None,
        'dlc': set(),
        'data_samples': [],
    })

    with open(filename, encoding='utf-8') as f:
        reader = csv.reader(f)
        for row in reader:
            # Skip header, comments, status lines
            if len(row) < 3:
                continue
            if row[0].startswith('#') or row[0].startswith('timestamp'):
                continue
            try:
                ts = int(row[0])
                can_id = row[1].strip()
                dlc = int(row[2])
            except (ValueError, IndexError):
                continue

            info = ids[can_id]
            info['count'] += 1
            info['dlc'].add(dlc)
            if info['first_ts'] is None:
                info['first_ts'] = ts
            info['last_ts'] = ts

            data_bytes = []
            for b in row[3:3 + dlc]:
                try:
                    data_bytes.append(int(b, 16))
                except ValueError:
                    data_bytes.append(0)
            if len(info['data_samples']) < 2000:
                info['data_samples'].append(data_bytes)

    return dict(ids)


def compute_byte_stats(samples: List[List[int]], dlc: int) -> List[dict]:
    """Compute min/max/range/std for each byte position."""
    stats = []
    for byte_idx in range(dlc):
        values = [s[byte_idx] for s in samples if byte_idx < len(s)]
        if not values:
            stats.append({'min': 0, 'max': 0, 'range': 0, 'std': 0.0})
            continue
        min_v = min(values)
        max_v = max(values)
        mean = sum(values) / len(values)
        variance = sum((v - mean) ** 2 for v in values) / len(values)
        stats.append({
            'min': min_v,
            'max': max_v,
            'range': max_v - min_v,
            'std': variance ** 0.5,
        })
    return stats


def classify_rate(rate_hz: float) -> str:
    """Classify signal type by broadcast rate."""
    if rate_hz > 20:
        return "CONTROL (RPM/wheels/steering)"
    elif rate_hz >= 1:
        return "STATE (throttle/temp)"
    else:
        return "EVENT (diagnostic)"


def heuristic_identify(can_id: str, rate_hz: float, byte_stats: List[dict],
                       samples: List[List[int]]) -> List[str]:
    """Apply heuristics to guess what a CAN ID might carry."""
    hints = []

    if len(byte_stats) < 2:
        return hints

    # High-rate IDs with 2-byte dynamic fields -> control signals
    if rate_hz > 30 and byte_stats[0]['range'] > 0 and byte_stats[1]['range'] > 0:
        # Check if B0-B1 look like RPM (~800 idle = 0x0320/4, or direct 0x0C80)
        u16_values = [(s[0] << 8) | s[1] for s in samples if len(s) >= 2]
        if u16_values:
            mean_val = sum(u16_values) / len(u16_values)
            # RPM/4 at idle: ~200 (0xC8)
            if 150 < mean_val < 400 and rate_hz > 30:
                hints.append(f"Possible RPM (val/4={mean_val/4:.0f} rpm)")
            # Wheel speed *0.01 at 0 km/h: ~0
            if mean_val < 50 and rate_hz > 30:
                hints.append("Possible wheel speed (near zero at standstill)")

    # Steering: high-rate, signed 16-bit, near-zero center
    if rate_hz > 40 and byte_stats[0]['range'] > 100:
        i16_values = []
        for s in samples:
            if len(s) >= 2:
                raw = (s[0] << 8) | s[1]
                if raw > 0x7FFF:
                    raw -= 0x10000
                i16_values.append(raw)
        if i16_values:
            mean_i16 = sum(i16_values) / len(i16_values)
            if abs(mean_i16) < 200:  # near center
                hints.append(f"Possible steering angle (mean={mean_i16*0.1:.1f} deg)")

    # Temperature: low rate, single byte, value 40-120 (after -40 offset)
    if rate_hz < 5 and byte_stats[0]['range'] < 60:
        mean_b0 = sum(s[0] for s in samples if s) / max(len(samples), 1)
        if 60 < mean_b0 < 160:  # -40 offset -> 20-120 C
            hints.append(f"Possible temperature (B0-40={mean_b0-40:.0f} C)")

    # 4-byte frames with high rate: front/rear wheels
    if rate_hz > 30 and len(byte_stats) >= 4:
        pairs_dynamic = (byte_stats[0]['range'] > 0 and byte_stats[2]['range'] > 0)
        if pairs_dynamic:
            hints.append("Possible 2x wheel speed pair (B0-B1, B2-B3)")

    # Brake: low dynamic range, single bit flipping
    if byte_stats[0]['range'] == 1 and byte_stats[0]['min'] == 0:
        hints.append("Possible boolean flag (brake/status)")

    return hints


def print_report(ids: Dict[str, dict], verbose: bool = True):
    """Print analysis report to stdout."""
    print(f"\n{'CAN ID':<10} {'Count':>8} {'Rate (Hz)':>10} {'DLC':>5}  {'Type'}")
    print("=" * 70)

    sorted_ids = sorted(ids.keys(),
                        key=lambda k: ids[k]['count'], reverse=True)

    for can_id in sorted_ids:
        info = ids[can_id]
        duration_s = 1.0
        if info['count'] > 1 and info['first_ts'] and info['last_ts']:
            duration_s = (info['last_ts'] - info['first_ts']) / 1e6
            if duration_s <= 0:
                duration_s = 1.0
        rate = info['count'] / duration_s
        dlc_str = ','.join(str(d) for d in sorted(info['dlc']))
        sig_type = classify_rate(rate)

        print(f"{can_id:<10} {info['count']:>8} {rate:>10.1f} {dlc_str:>5}  {sig_type}")

        if verbose and len(info['data_samples']) > 10:
            max_dlc = max(info['dlc'])
            byte_stats = compute_byte_stats(info['data_samples'], max_dlc)

            # Show dynamic bytes
            for i, bs in enumerate(byte_stats):
                if bs['range'] > 0:
                    print(f"  B{i}: min={bs['min']:3d} (0x{bs['min']:02X}) "
                          f"max={bs['max']:3d} (0x{bs['max']:02X}) "
                          f"range={bs['range']:3d}  std={bs['std']:.1f}")

            # Heuristic hints
            hints = heuristic_identify(can_id, rate, byte_stats,
                                       info['data_samples'])
            for hint in hints:
                print(f"  >>> {hint}")


def plot_signals(ids: Dict[str, dict]):
    """Plot time-series for the most active CAN IDs."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[WARN] matplotlib not installed — skipping plots")
        return

    # Pick top 6 most frequent IDs
    sorted_ids = sorted(ids.keys(),
                        key=lambda k: ids[k]['count'], reverse=True)[:6]

    fig, axes = plt.subplots(len(sorted_ids), 1, figsize=(14, 3 * len(sorted_ids)),
                             sharex=True)
    if len(sorted_ids) == 1:
        axes = [axes]

    for ax, can_id in zip(axes, sorted_ids):
        info = ids[can_id]
        samples = info['data_samples']
        if len(samples) < 2:
            continue

        # Reconstruct timestamps (approximate: evenly spaced)
        t0 = info['first_ts']
        t1 = info['last_ts']
        n = len(samples)
        ts = [t0 + i * (t1 - t0) / max(n - 1, 1) for i in range(n)]
        ts_s = [(t - t0) / 1e6 for t in ts]

        # Plot B0-B1 as uint16
        if all(len(s) >= 2 for s in samples):
            vals = [(s[0] << 8) | s[1] for s in samples]
            ax.plot(ts_s, vals, linewidth=0.5, label='B0-B1 uint16')
            ax.legend(loc='upper right', fontsize=8)

        ax.set_ylabel(can_id)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time [s]')
    fig.suptitle('CAN Bus Signal Overview (B0-B1 as uint16)')
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze CAN sniffer CSV logs')
    parser.add_argument('logfile', help='Path to CAN sniffer CSV log')
    parser.add_argument('--plot', action='store_true',
                        help='Show time-series plots (requires matplotlib)')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress byte-level details')
    args = parser.parse_args()

    print(f"Parsing {args.logfile}...")
    ids = parse_can_log(args.logfile)

    if not ids:
        print("No valid CAN frames found in the log file.")
        sys.exit(1)

    print_report(ids, verbose=not args.quiet)

    total_frames = sum(info['count'] for info in ids.values())
    first_ts = min(info['first_ts'] for info in ids.values()
                   if info['first_ts'] is not None)
    last_ts = max(info['last_ts'] for info in ids.values()
                  if info['last_ts'] is not None)
    duration = (last_ts - first_ts) / 1e6

    print(f"\n{'='*70}")
    print(f"Total: {total_frames} frames, {len(ids)} unique IDs, "
          f"{duration:.1f} s capture")
    print(f"Overall bus load: {total_frames/max(duration,0.001):.0f} frames/s")

    if args.plot:
        plot_signals(ids)


if __name__ == '__main__':
    main()
