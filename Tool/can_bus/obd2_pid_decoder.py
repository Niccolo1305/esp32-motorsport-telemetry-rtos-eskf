#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""
OBD-II PID Support Decoder — offline bitmask analysis.

Decodes PID support bitmask responses from OBD-II PID 0x00/0x20/0x40
and cross-references with the known K9K/M9R PID database.

Usage:
    # Decode from hex response bytes
    python obd2_pid_decoder.py --bitmask 0xBE1FA813

    # Parse from OBD2 validator log
    python obd2_pid_decoder.py --logfile obd2_capture.log

    # Show full K9K/M9R PID database
    python obd2_pid_decoder.py --database
"""

import argparse
import re
import sys

# ── Known PID Database (K9K / M9R diesel) ────────────────────────────────────
PID_DATABASE = {
    # Mode 01 — Standard PIDs
    0x01: ("Monitor status since DTC cleared", "bitmask", None),
    0x03: ("Fuel system status", "enum", None),
    0x04: ("Engine load", "%", lambda a, b: a * 100 / 255),
    0x05: ("Coolant temperature", "C", lambda a, b: a - 40),
    0x06: ("Short term fuel trim (B1)", "%", lambda a, b: (a - 128) * 100 / 128),
    0x07: ("Long term fuel trim (B1)", "%", lambda a, b: (a - 128) * 100 / 128),
    0x0B: ("Intake manifold pressure (MAP)", "kPa", lambda a, b: a),
    0x0C: ("Engine RPM", "rpm", lambda a, b: (a * 256 + b) / 4),
    0x0D: ("Vehicle speed", "km/h", lambda a, b: a),
    0x0E: ("Timing advance", "deg", lambda a, b: a / 2 - 64),
    0x0F: ("Intake air temperature", "C", lambda a, b: a - 40),
    0x10: ("MAF air flow rate", "g/s", lambda a, b: (a * 256 + b) / 100),
    0x11: ("Throttle position", "%", lambda a, b: a * 100 / 255),
    0x1C: ("OBD standards compliance", "enum", None),
    0x1F: ("Run time since engine start", "s", lambda a, b: a * 256 + b),
    0x21: ("Distance with MIL on", "km", lambda a, b: a * 256 + b),
    0x2F: ("Fuel tank level", "%", lambda a, b: a * 100 / 255),
    0x30: ("Warm-ups since codes cleared", "count", lambda a, b: a),
    0x31: ("Distance since codes cleared", "km", lambda a, b: a * 256 + b),
    0x33: ("Barometric pressure", "kPa", lambda a, b: a),
    0x42: ("Control module voltage", "V", lambda a, b: (a * 256 + b) / 1000),
    0x46: ("Ambient air temperature", "C", lambda a, b: a - 40),
    0x49: ("Accelerator pedal position D", "%", lambda a, b: a * 100 / 255),
    0x4A: ("Accelerator pedal position E", "%", lambda a, b: a * 100 / 255),
    0x5A: ("Relative accelerator pedal", "%", lambda a, b: a * 100 / 255),
    0x5C: ("Engine oil temperature", "C", lambda a, b: a - 40),
    0x5E: ("Engine fuel rate", "L/h", lambda a, b: (a * 256 + b) / 20),
}

# PIDs expected on K9K / M9R
K9K_M9R_EXPECTED = {
    0x04: "Engine load",
    0x05: "Coolant temp",
    0x0B: "MAP",
    0x0C: "Engine RPM",
    0x0D: "Vehicle speed",
    0x0F: "Intake air temp",
    0x10: "MAF",
    0x11: "Throttle position",
    0x2F: "Fuel level",
    0x33: "Barometric pressure",
    0x46: "Ambient temp",
    0x5A: "Accel pedal",
    0x5C: "Oil temp (M9R confirmed, K9K inconsistent)",
    0x5E: "Fuel rate",
}


def decode_bitmask(bitmask: int, base_pid: int = 0x00):
    """Decode a 32-bit PID support bitmask."""
    supported = []
    for i in range(32):
        if bitmask & (1 << (31 - i)):
            pid = base_pid + 1 + i
            supported.append(pid)
    return supported


def print_supported_pids(supported: list):
    """Print decoded PID list with database cross-reference."""
    print(f"\n{'PID':>6}  {'Status':>10}  {'Name'}")
    print("-" * 60)
    for pid in supported:
        db_entry = PID_DATABASE.get(pid)
        expected = K9K_M9R_EXPECTED.get(pid)

        name = db_entry[0] if db_entry else "Unknown"
        status = "EXPECTED" if expected else "standard"

        print(f"  0x{pid:02X}  {status:>10}  {name}")

    # Check for missing expected PIDs
    missing = [pid for pid in K9K_M9R_EXPECTED if pid not in supported
               and pid <= max(supported, default=0)]
    if missing:
        print(f"\nMissing expected PIDs:")
        for pid in missing:
            print(f"  0x{pid:02X}: {K9K_M9R_EXPECTED[pid]}")


def parse_log_for_bitmask(filename: str) -> list:
    """Extract PID support bitmask responses from an OBD2 validator log."""
    bitmasks = []
    pattern = re.compile(
        r'\[OBD\] Supported PIDs 0x(\w+)-0x\w+: 0x(\w+)')

    with open(filename, encoding='utf-8') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                base = int(m.group(1), 16) - 1  # base_pid for decode_bitmask
                bitmask = int(m.group(2), 16)
                bitmasks.append((base, bitmask))

    return bitmasks


def print_database():
    """Print the full PID database."""
    print("\n" + "=" * 70)
    print("K9K / M9R OBD-II PID Database (Mode 01)")
    print("=" * 70)
    print(f"\n{'PID':>6}  {'Unit':>8}  {'K9K/M9R':>10}  {'Name'}")
    print("-" * 70)
    for pid in sorted(PID_DATABASE.keys()):
        name, unit, _ = PID_DATABASE[pid]
        is_expected = "YES" if pid in K9K_M9R_EXPECTED else ""
        print(f"  0x{pid:02X}  {unit:>8}  {is_expected:>10}  {name}")

    print(f"\n\nMode 22 Extended PIDs (Nissan Consult):")
    print("-" * 70)
    mode22 = [
        (0x1166, "Rail pressure (actual)", "bar"),
        (0x1167, "Rail pressure (target)", "bar"),
        (0x115C, "DPF differential pressure", "mbar"),
        (0x1154, "EGR valve position", "%"),
        (0x1160, "Boost pressure (actual)", "mbar"),
        (0x1161, "Boost pressure (target)", "mbar"),
        (0x1001, "Injector correction cyl.1", "mm^3"),
        (0x1002, "Injector correction cyl.2", "mm^3"),
        (0x1003, "Injector correction cyl.3", "mm^3"),
        (0x1004, "Injector correction cyl.4", "mm^3"),
        (0x1168, "DPF soot load", "g"),
        (0x1169, "DPF regen status", "flags"),
    ]
    for pid, name, unit in mode22:
        print(f"  0x{pid:04X}  {unit:>8}  {name}")


def main():
    parser = argparse.ArgumentParser(
        description='Decode OBD-II PID support bitmasks')
    parser.add_argument('--bitmask', help='Hex bitmask to decode (e.g. 0xBE1FA813)')
    parser.add_argument('--base', type=int, default=0,
                        help='Base PID for bitmask (0=PIDs 01-20, 0x20=21-40, etc.)')
    parser.add_argument('--logfile', help='OBD2 validator log file to parse')
    parser.add_argument('--database', action='store_true',
                        help='Print full PID database')
    args = parser.parse_args()

    if args.database:
        print_database()
        return

    if args.bitmask:
        bitmask = int(args.bitmask, 16)
        print(f"Decoding bitmask 0x{bitmask:08X} (base PID 0x{args.base:02X}):")
        supported = decode_bitmask(bitmask, args.base)
        print_supported_pids(supported)
        return

    if args.logfile:
        bitmasks = parse_log_for_bitmask(args.logfile)
        if not bitmasks:
            print("No PID support bitmasks found in log file")
            sys.exit(1)

        all_supported = []
        for base, bitmask in bitmasks:
            print(f"\nBitmask 0x{bitmask:08X} (base 0x{base:02X}):")
            supported = decode_bitmask(bitmask, base)
            all_supported.extend(supported)

        print("\n" + "=" * 60)
        print("Combined supported PIDs:")
        print_supported_pids(sorted(set(all_supported)))
        return

    parser.print_help()


if __name__ == '__main__':
    main()
