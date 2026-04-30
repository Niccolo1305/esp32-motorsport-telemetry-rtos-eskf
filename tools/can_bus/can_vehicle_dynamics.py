#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Vehicle Dynamics Calculator — derived signals from CAN + IMU/GPS.

Computes from telemetry CSV (with CAN fields):
  - Differential yaw rate from wheel speeds
  - Instantaneous turn radius
  - Approximate slip angle
  - Understeer gradient
  - Lateral sliding diagnosis
  - Cross-validation CAN speed vs GPS speed

Usage:
    python can_vehicle_dynamics.py <telemetry_with_can.csv>
    python can_vehicle_dynamics.py <telemetry_with_can.csv> --plot
    python can_vehicle_dynamics.py <telemetry_with_can.csv> --output dynamics.csv
"""

import sys
import argparse
import math
from typing import Optional

import numpy as np
import pandas as pd


# ── Qashqai J10 Physical Constants ──────────────────────────────────────────
WHEELBASE_M = 2.630       # passo [m]
TRACK_WIDTH_M = 1.540     # carreggiata [m]
STEERING_RATIO = 16.0     # rapporto sterzo (verificare on-vehicle)
WHEEL_DIAMETER_M = 0.686  # 215/65 R16


def compute_dynamics(df: pd.DataFrame) -> pd.DataFrame:
    """Add derived vehicle dynamics columns to a telemetry DataFrame."""

    # Verify required CAN columns exist
    can_cols = ['can_wheel_fl_kmh', 'can_wheel_fr_kmh',
                'can_wheel_rl_kmh', 'can_wheel_rr_kmh']
    missing = [c for c in can_cols if c not in df.columns]
    if missing:
        print(f"WARNING: missing CAN columns: {missing}")
        print("Available columns:", list(df.columns))
        return df

    # Convert wheel speeds to m/s
    wfl = df['can_wheel_fl_kmh'].values / 3.6
    wfr = df['can_wheel_fr_kmh'].values / 3.6
    wrl = df['can_wheel_rl_kmh'].values / 3.6
    wrr = df['can_wheel_rr_kmh'].values / 3.6

    # Average speeds
    v_left = (wfl + wrl) / 2.0
    v_right = (wfr + wrr) / 2.0
    v_front = (wfl + wfr) / 2.0
    v_rear = (wrl + wrr) / 2.0
    v_avg = (wfl + wfr + wrl + wrr) / 4.0

    # CAN average speed in km/h
    df['can_speed_avg_kmh'] = v_avg * 3.6

    # ── Differential yaw rate [rad/s] ────────────────────────────────────────
    df['yaw_rate_wheel_rads'] = (v_right - v_left) / TRACK_WIDTH_M
    df['yaw_rate_wheel_degs'] = np.degrees(df['yaw_rate_wheel_rads'])

    # ── Instantaneous turn radius [m] ────────────────────────────────────────
    omega = df['yaw_rate_wheel_rads'].values
    with np.errstate(divide='ignore', invalid='ignore'):
        radius = np.where(np.abs(omega) > 0.001, v_avg / omega, np.inf)
    df['turn_radius_m'] = radius

    # ── Slip angle approximation [deg] ───────────────────────────────────────
    with np.errstate(divide='ignore', invalid='ignore'):
        beta_rad = np.where(
            v_rear > 0.5,  # only above ~1.8 km/h
            np.arctan((v_front - v_rear) / v_rear),
            0.0
        )
    df['slip_angle_deg'] = np.degrees(beta_rad)

    # ── Understeer gradient [deg] ────────────────────────────────────────────
    if 'can_steering_angle_deg' in df.columns:
        steer_rad = np.radians(df['can_steering_angle_deg'].values) / STEERING_RATIO
        with np.errstate(divide='ignore', invalid='ignore'):
            ackermann_rad = np.where(np.abs(radius) > 1.0,
                                     WHEELBASE_M / radius, 0.0)
        df['understeer_deg'] = np.degrees(steer_rad - ackermann_rad)
    else:
        df['understeer_deg'] = np.nan

    # ── Lateral sliding diagnosis ────────────────────────────────────────────
    df['delta_v_front_kmh'] = (wfr - wfl) * 3.6  # FR - FL
    df['delta_v_rear_kmh'] = (wrr - wrl) * 3.6   # RR - RL
    df['slide_indicator'] = df['delta_v_rear_kmh'] - df['delta_v_front_kmh']
    # positive = rear slides more than front (oversteer tendency)

    # ── GPS vs CAN speed cross-validation ────────────────────────────────────
    if 'gps_sog_kmh' in df.columns:
        df['speed_delta_kmh'] = df['can_speed_avg_kmh'] - df['gps_sog_kmh']
    elif 'gps_speed' in df.columns:
        df['speed_delta_kmh'] = df['can_speed_avg_kmh'] - df['gps_speed']

    # ── Gyro vs wheel yaw cross-validation ───────────────────────────────────
    if 'gz' in df.columns:
        df['yaw_delta_degs'] = df['yaw_rate_wheel_degs'] - df['gz']

    return df


def print_summary(df: pd.DataFrame):
    """Print dynamics summary statistics."""
    print("\n" + "=" * 60)
    print("Vehicle Dynamics Summary")
    print("=" * 60)

    cols_to_show = [
        ('can_speed_avg_kmh', 'CAN avg speed [km/h]'),
        ('yaw_rate_wheel_degs', 'Wheel yaw rate [deg/s]'),
        ('turn_radius_m', 'Turn radius [m]'),
        ('slip_angle_deg', 'Slip angle [deg]'),
        ('understeer_deg', 'Understeer gradient [deg]'),
        ('speed_delta_kmh', 'CAN-GPS speed delta [km/h]'),
        ('yaw_delta_degs', 'Wheel-gyro yaw delta [deg/s]'),
    ]

    for col, label in cols_to_show:
        if col in df.columns:
            valid = df[col].replace([np.inf, -np.inf], np.nan).dropna()
            if len(valid) > 0:
                print(f"\n{label}:")
                print(f"  mean={valid.mean():+.2f}  "
                      f"std={valid.std():.2f}  "
                      f"min={valid.min():+.2f}  "
                      f"max={valid.max():+.2f}")


def plot_dynamics(df: pd.DataFrame):
    """Plot dynamics time-series."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not installed — skipping plots")
        return

    if 'timestamp_us' in df.columns:
        t = (df['timestamp_us'] - df['timestamp_us'].iloc[0]) / 1e6
    else:
        t = np.arange(len(df)) / 50.0  # assume 50 Hz

    fig, axes = plt.subplots(5, 1, figsize=(14, 15), sharex=True)

    # 1. Wheel speeds
    ax = axes[0]
    for col, label in [('can_wheel_fl_kmh', 'FL'), ('can_wheel_fr_kmh', 'FR'),
                       ('can_wheel_rl_kmh', 'RL'), ('can_wheel_rr_kmh', 'RR')]:
        if col in df.columns:
            ax.plot(t, df[col], linewidth=0.7, label=label)
    if 'gps_sog_kmh' in df.columns:
        ax.plot(t, df['gps_sog_kmh'], 'k--', linewidth=1, label='GPS', alpha=0.5)
    ax.set_ylabel('Speed [km/h]')
    ax.legend(ncol=5, fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title('Wheel Speeds vs GPS')

    # 2. Yaw rate comparison
    ax = axes[1]
    ax.plot(t, df['yaw_rate_wheel_degs'], linewidth=0.7, label='Wheel diff')
    if 'gz' in df.columns:
        ax.plot(t, df['gz'], linewidth=0.7, label='Gyro Z', alpha=0.7)
    ax.set_ylabel('Yaw rate [deg/s]')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_title('Yaw Rate: Wheel Differential vs Gyroscope')

    # 3. Turn radius
    ax = axes[2]
    radius_clipped = df['turn_radius_m'].clip(-500, 500)
    ax.plot(t, radius_clipped, linewidth=0.5)
    ax.set_ylabel('Radius [m]')
    ax.set_ylim(-200, 200)
    ax.grid(True, alpha=0.3)
    ax.set_title('Instantaneous Turn Radius')

    # 4. Slip angle
    ax = axes[3]
    ax.plot(t, df['slip_angle_deg'], linewidth=0.7)
    ax.set_ylabel('Slip [deg]')
    ax.grid(True, alpha=0.3)
    ax.set_title('Estimated Slip Angle')

    # 5. Understeer
    ax = axes[4]
    if 'understeer_deg' in df.columns:
        us = df['understeer_deg'].clip(-10, 10)
        ax.plot(t, us, linewidth=0.7)
        ax.axhline(0, color='k', linewidth=0.5)
        ax.fill_between(t, 0, us, where=us > 0, alpha=0.2, color='red',
                        label='Understeer')
        ax.fill_between(t, 0, us, where=us < 0, alpha=0.2, color='blue',
                        label='Oversteer')
        ax.legend(fontsize=8)
    ax.set_ylabel('Understeer [deg]')
    ax.set_xlabel('Time [s]')
    ax.grid(True, alpha=0.3)
    ax.set_title('Understeer Gradient')

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Compute derived vehicle dynamics from CAN + telemetry')
    parser.add_argument('csvfile', help='Telemetry CSV with CAN fields')
    parser.add_argument('--output', '-o', help='Save results to CSV')
    parser.add_argument('--plot', action='store_true', help='Show plots')
    args = parser.parse_args()

    print(f"Loading {args.csvfile}...")
    df = pd.read_csv(args.csvfile)
    print(f"  {len(df)} rows, {len(df.columns)} columns")

    df = compute_dynamics(df)
    print_summary(df)

    if args.output:
        df.to_csv(args.output, index=False)
        print(f"\nSaved to {args.output}")

    if args.plot:
        plot_dynamics(df)


if __name__ == '__main__':
    main()
