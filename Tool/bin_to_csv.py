"""
bin_to_csv.py — Binary telemetry → CSV converter with interactive split

Usage:
  python bin_to_csv.py                      # interactive menu
  python bin_to_csv.py tel_23.bin            # convert → split menu
  python bin_to_csv.py tel_23.csv            # split existing CSV
  python bin_to_csv.py tel_23.bin out.csv    # convert without split
"""

import struct
import sys
import os
import time
import math

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')
    except ValueError:
        pass

# ── Configuration ─────────────────────────────────────────────────────────────
CHUNK_SIZE_DEFAULT = 122          # record size v0.9.8 (no header, 122 bytes)
HEADER_MAGIC = b'TEL'             # FileHeader magic bytes (v0.9.7+)
HEADER_V1_SIZE = 25               # v1: record_size is uint8_t  (25 bytes)
HEADER_V2_SIZE = 26               # v2: record_size is uint16_t (26 bytes)
HEADER_V3_SIZE = 66               # v3: adds 40 bytes of calibration params
SENTINEL_CALIB = 0xFFFFFFFFFFFFFFFF  # uint64 max — CalibrationRecord marker
PROGRESS_EVERY = 5000

# ── Format Registry ──────────────────────────────────────────────────────────
# 122-byte record (v0.9.8 — v1.3.0)
FMT_122 = '<I7fBddffBf4f6f5f'
HEADER_122 = [
    't_ms (ms)',
    'ax (G)', 'ay (G)', 'az (G)', 'gx (°/s)', 'gy (°/s)', 'gz (°/s)', 'temp_c (°C)',
    'lap',
    'gps_lat (°)', 'gps_lon (°)',
    'gps_speed_kmh (km/h)', 'gps_alt_m (m)',
    'gps_sats',
    'gps_hdop',
    'kf_x (m)', 'kf_y (m)', 'kf_vel (m/s)', 'kf_heading (rad)',
    'raw_ax (G)', 'raw_ay (G)', 'raw_az (G)', 'raw_gx (°/s)', 'raw_gy (°/s)', 'raw_gz (°/s)',
    'kf6_x (m)', 'kf6_y (m)', 'kf6_vel (m/s)', 'kf6_heading (rad)', 'kf6_bgz (rad/s)'
]
COL_FMT_122 = [
    '{:d}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.1f}',
    '{:d}',
    '{:.7f}', '{:.7f}',
    '{:.2f}', '{:.2f}',
    '{:d}',
    '{:.2f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}'
]

# 127-byte record (v1.3.1+): adds zaru_flags (uint8) + tbias_gz (float)
FMT_127 = '<I7fBddffBf4f6f5fBf'
HEADER_127 = HEADER_122 + ['zaru_flags', 'tbias_gz (°/s)']
COL_FMT_127 = COL_FMT_122 + ['{:d}', '{:.5f}']

# 155-byte record (v1.4.0): timestamp upgraded to uint64 µs + sensor-frame raw IMU
FMT_155 = '<Q7fBddffBf4f6f5fBf6f'  # Q = uint64 (was I = uint32)
HEADER_155 = [
    't_us (µs)',  # was t_ms (ms) — microsecond precision for SITL
    'ax (G)', 'ay (G)', 'az (G)', 'gx (°/s)', 'gy (°/s)', 'gz (°/s)', 'temp_c (°C)',
    'lap',
    'gps_lat (°)', 'gps_lon (°)',
    'gps_speed_kmh (km/h)', 'gps_alt_m (m)',
    'gps_sats',
    'gps_hdop',
    'kf_x (m)', 'kf_y (m)', 'kf_vel (m/s)', 'kf_heading (rad)',
    'raw_ax (G)', 'raw_ay (G)', 'raw_az (G)', 'raw_gx (°/s)', 'raw_gy (°/s)', 'raw_gz (°/s)',
    'kf6_x (m)', 'kf6_y (m)', 'kf6_vel (m/s)', 'kf6_heading (rad)', 'kf6_bgz (rad/s)',
    'zaru_flags', 'tbias_gz (°/s)',
    'sensor_ax (G)', 'sensor_ay (G)', 'sensor_az (G)',
    'sensor_gx (°/s)', 'sensor_gy (°/s)', 'sensor_gz (°/s)'
]
COL_FMT_155 = [
    '{:d}',  # t_us (uint64 → int)
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.1f}',
    '{:d}',
    '{:.7f}', '{:.7f}',
    '{:.2f}', '{:.2f}',
    '{:d}',
    '{:.2f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}',
    '{:d}', '{:.5f}',
    '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}', '{:.5f}'
]
assert struct.calcsize(FMT_155) == 155, f"FMT_155 mismatch: {struct.calcsize(FMT_155)}"

# 164-byte record (v1.4.2+): adds GPS timing metadata for deterministic SITL replay
# gps_fix_us: esp_timer timestamp of last GPS fix — same timebase as t_us.
# gps_valid:  mirrors GpsData.valid — needed for is_stationary gps_slow gate.
FMT_164 = '<Q7fBddffBf4f6f5fBf6fQB'  # appends Q (uint64) + B (uint8)
HEADER_164 = HEADER_155 + ['gps_fix_us (µs)', 'gps_valid']
COL_FMT_164 = COL_FMT_155 + ['{:d}', '{:d}']
assert struct.calcsize(FMT_164) == 164, f"FMT_164 mismatch: {struct.calcsize(FMT_164)}"

# Default aliases (used for legacy files without header)
FMT_DEFAULT = FMT_122
HEADER = HEADER_122
COL_FMT = COL_FMT_122

def get_format(record_size):
    """Return (struct_fmt, header_list, col_fmt_list) for a given record size."""
    if record_size == 164:
        return FMT_164, HEADER_164, COL_FMT_164
    if record_size == 155:
        return FMT_155, HEADER_155, COL_FMT_155
    if record_size == 127:
        return FMT_127, HEADER_127, COL_FMT_127
    # Default: 122-byte format (also used for legacy files)
    return FMT_122, HEADER_122, COL_FMT_122

HEADER_LINE = ','.join(HEADER) + '\n'

# ── ANSI Colors ───────────────────────────────────────────────────────────────
BOLD  = '\033[1m'
DIM   = '\033[2m'
CYAN  = '\033[36m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
RED   = '\033[31m'
RESET = '\033[0m'


def banner():
    print(f"""
{BOLD}╔══════════════════════════════════════════════════════╗
║        TELEMETRIA — Converter & Splitter             ║
║        .bin (header+record) → split .csv             ║
╚══════════════════════════════════════════════════════╝{RESET}
""")


def ask(prompt, default=None):
    """Input prompt with optional default shown in brackets."""
    suffix = f" [{default}]" if default else ""
    val = input(f"  {CYAN}▸{RESET} {prompt}{suffix}: ").strip()
    return val if val else (str(default) if default else "")


def ask_int(prompt, default=None, minimum=1):
    while True:
        val = ask(prompt, default)
        try:
            n = int(val)
            if n >= minimum:
                return n
            print(f"    {RED}Minimum: {minimum}{RESET}")
        except ValueError:
            print(f"    {RED}Please enter an integer.{RESET}")


def ask_float(prompt, default=None, minimum=0.1):
    while True:
        val = ask(prompt, default)
        try:
            n = float(val)
            if n >= minimum:
                return n
            print(f"    {RED}Minimum: {minimum}{RESET}")
        except ValueError:
            print(f"    {RED}Please enter a number.{RESET}")


def find_input_file():
    """Search for .bin and .csv files in the current directory."""
    bins = sorted([f for f in os.listdir('.') if f.endswith('.bin')])
    csvs = sorted([f for f in os.listdir('.') if f.endswith('.csv')
                   and os.path.getsize(f) > 1_000_000])  # only CSV >1MB

    if not bins and not csvs:
        print(f"  {RED}No .bin or .csv files found in the current directory.{RESET}")
        path = ask("File path")
        if not os.path.isfile(path):
            print(f"  {RED}File not found: {path}{RESET}")
            sys.exit(1)
        return path

    print(f"  {BOLD}Files found:{RESET}")
    options = []
    for f in bins:
        size_mb = os.path.getsize(f) / (1024 * 1024)
        n_records = bin_record_count(f)
        options.append(f)
        print(f"    {GREEN}{len(options)}.{RESET} {f}  "
              f"{DIM}({size_mb:.1f} MB, ~{n_records:,} records, binary){RESET}")
    for f in csvs:
        size_mb = os.path.getsize(f) / (1024 * 1024)
        options.append(f)
        print(f"    {GREEN}{len(options)}.{RESET} {f}  "
              f"{DIM}({size_mb:.1f} MB, CSV){RESET}")
    print()

    if len(options) == 1:
        print(f"  Auto-selected: {BOLD}{options[0]}{RESET}")
        return options[0]

    choice = ask_int(f"Choose file (1-{len(options)})", 1, minimum=1)
    choice = min(choice, len(options))
    return options[choice - 1]


# ── FileHeader reader (v0.9.7+) ───────────────────────────────────────────────

def read_file_header(bin_path):
    """Read FileHeader if present (magic 'TEL'). Returns (header_dict, offset)
    or (None, 0) for legacy files without a header.

    Supports both header versions:
      v1 (25 bytes): record_size is uint8_t  — firmware <= v0.9.12
      v2 (26 bytes): record_size is uint16_t — firmware >= v0.9.13
    The header_version byte (offset 3) determines the layout.
    """
    with open(bin_path, 'rb') as f:
        preamble = f.read(4)  # magic (3) + header_version (1)
        if len(preamble) < 4 or preamble[:3] != HEADER_MAGIC:
            return None, 0

        hdr_ver = preamble[3]

        if hdr_ver >= 3:
            # v3: 3s B 16s H I 10f = 3+1+16+2+4+40 = 66 bytes
            hdr_size = HEADER_V3_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            parts = struct.unpack('<3sB16sHI10f', raw)
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = parts[:5]
            cal_params = parts[5:]  # 10 float: sin_phi..bias_gz
        elif hdr_ver >= 2:
            # v2: 3s B 16s H I = 3+1+16+2+4 = 26 bytes
            hdr_size = HEADER_V2_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = struct.unpack(
                '<3sB16sHI', raw)
            cal_params = None
        else:
            # v1: 3s B 16s B I = 3+1+16+1+4 = 25 bytes
            hdr_size = HEADER_V1_SIZE
            f.seek(0)
            raw = f.read(hdr_size)
            if len(raw) < hdr_size:
                return None, 0
            magic_, hdr_ver_, fw_ver, rec_size, start_ms = struct.unpack(
                '<3sB16sBI', raw)
            cal_params = None

        fw_str = fw_ver.split(b'\x00')[0].decode('ascii', errors='replace')
        result = {
            'header_version': hdr_ver_,
            'firmware_version': fw_str,
            'record_size': rec_size,
            'start_time_ms': start_ms,
        }
        if cal_params is not None:
            result['calibration'] = cal_params
        return result, hdr_size


# ── Binary → CSV row conversion ───────────────────────────────────────────────

def bin_record_count(bin_path):
    hdr, offset = read_file_header(bin_path)
    chunk_size = hdr['record_size'] if hdr else CHUNK_SIZE_DEFAULT
    data_bytes = os.path.getsize(bin_path) - offset
    return data_bytes // chunk_size


def bin_to_csv_lines(bin_path, total=None):
    """Yield CSV rows from a binary telemetry file (with or without FileHeader)."""
    hdr, offset = read_file_header(bin_path)
    chunk_size = hdr['record_size'] if hdr else CHUNK_SIZE_DEFAULT
    fmt, _, col_fmt = get_format(chunk_size)
    
    if hdr and 'calibration' in hdr:
        c = hdr['calibration']
        yield f"# CALIB_BOOT: sin_phi={c[0]:.5f} cos_phi={c[1]:.5f} sin_theta={c[2]:.5f} cos_theta={c[3]:.5f} bias_ax={c[4]:.5f} bias_ay={c[5]:.5f} bias_az={c[6]:.5f} bias_gx={c[7]:.5f} bias_gy={c[8]:.5f} bias_gz={c[9]:.5f}"

    with open(bin_path, 'rb') as f:
        if offset > 0:
            f.seek(offset)
        count = 0
        while True:
            chunk = f.read(chunk_size)
            if len(chunk) < chunk_size:
                break
            vals = struct.unpack(fmt, chunk)
            if vals[0] == SENTINEL_CALIB:   # CalibrationRecord sentinel (uint64 max)
                yield f"# CALIB_UPDATE: sin_phi={vals[1]:.5f} cos_phi={vals[2]:.5f} sin_theta={vals[3]:.5f} cos_theta={vals[4]:.5f} bias_ax={vals[5]:.5f} bias_ay={vals[6]:.5f} bias_az={vals[7]:.5f} bias_gx={vals[11]:.5f} bias_gy={vals[12]:.5f} bias_gz={vals[14]:.5f}"
                continue
            line = ','.join(f.format(v) for f, v in zip(col_fmt, vals))
            yield line
            count += 1
            if count % PROGRESS_EVERY == 0 and total:
                pct = count * 100 // total
                print(f"    Read {count:>10,} / {total:,}  ({pct}%)", end='\r')
    if total:
        print(f"    Read {count:>10,} / {total:,}  (100%)    ")


def csv_to_lines(csv_path):
    """Yield CSV rows (skips header line)."""
    total = sum(1 for _ in open(csv_path)) - 1
    with open(csv_path, 'r') as f:
        f.readline()  # skip header
        count = 0
        for line in f:
            stripped = line.rstrip('\n\r')
            if stripped:
                yield stripped
                count += 1
                if count % PROGRESS_EVERY == 0:
                    pct = count * 100 // total if total else 0
                    print(f"    Read {count:>10,} / {total:,}  ({pct}%)", end='\r')
    print(f"    Read {count:>10,} / {total:,}  (100%)    ")


def count_csv_lines(csv_path):
    """Count data rows (excluding header)."""
    count = 0
    with open(csv_path, 'r') as f:
        f.readline()  # skip header
        for _ in f:
            count += 1
    return count


# ── Split methods ─────────────────────────────────────────────────────────────

def write_split_files(lines_gen, base_name, split_fn, description):
    """
    Write rows into split output files.
    split_fn(part, line_count, file_size) → bool: True to open a new file.
    """
    t0 = time.perf_counter()
    part = 1
    line_count_total = 0
    line_count_part = 0
    file_size_part = 0
    header_bytes = len(HEADER_LINE.encode('utf-8'))

    out_path = f"{base_name}_part{part}.csv"
    fout = open(out_path, 'w')
    fout.write(HEADER_LINE)
    file_size_part = header_bytes
    files_written = [out_path]

    for line in lines_gen:
        line_data = line + '\n'
        line_bytes = len(line_data.encode('utf-8'))

        # Check if a new file is needed BEFORE writing
        if split_fn(part, line_count_part, file_size_part + line_bytes):
            fout.close()
            size_mb = file_size_part / (1024 * 1024)
            print(f"    {GREEN}✓{RESET} {out_path}: {line_count_part:,} rows, {size_mb:.1f} MB")

            part += 1
            line_count_part = 0
            out_path = f"{base_name}_part{part}.csv"
            fout = open(out_path, 'w')
            fout.write(HEADER_LINE)
            file_size_part = header_bytes
            files_written.append(out_path)

        fout.write(line_data)
        file_size_part += line_bytes
        line_count_part += 1
        line_count_total += 1

    fout.close()
    size_mb = file_size_part / (1024 * 1024)
    print(f"    {GREEN}✓{RESET} {out_path}: {line_count_part:,} rows, {size_mb:.1f} MB")

    elapsed = time.perf_counter() - t0
    print(f"\n  {BOLD}Done:{RESET} {line_count_total:,} rows → "
          f"{part} files in {elapsed:.1f}s")
    return files_written


def split_by_max_size(lines_gen, base_name, max_mb):
    max_bytes = int(max_mb * 1024 * 1024)
    return write_split_files(
        lines_gen, base_name,
        lambda part, lc, fs: lc > 0 and fs > max_bytes,
        f"max {max_mb} MB per file"
    )


def split_by_n_parts(lines_gen, base_name, n_parts, total_lines):
    lines_per_part = math.ceil(total_lines / n_parts)
    return write_split_files(
        lines_gen, base_name,
        lambda part, lc, fs: part < n_parts and lc >= lines_per_part,
        f"{n_parts} equal parts"
    )


def split_by_rows(lines_gen, base_name, rows_per_part):
    return write_split_files(
        lines_gen, base_name,
        lambda part, lc, fs: lc >= rows_per_part,
        f"{rows_per_part:,} rows per file"
    )


def no_split(lines_gen, out_path):
    """Write everything into a single CSV file."""
    t0 = time.perf_counter()
    count = 0
    with open(out_path, 'w') as f:
        f.write(HEADER_LINE)
        for line in lines_gen:
            f.write(line + '\n')
            count += 1
    elapsed = time.perf_counter() - t0
    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"\n  {GREEN}✓{RESET} {out_path}: {count:,} rows, {size_mb:.1f} MB ({elapsed:.1f}s)")


# ── Split menu ────────────────────────────────────────────────────────────────

def split_menu(lines_gen_factory, base_name, total_lines, est_csv_mb):
    """
    Interactive menu to choose the split method.
    lines_gen_factory: callable that returns a fresh row generator.
    """
    print(f"\n  {BOLD}Dataset:{RESET} ~{total_lines:,} rows, ~{est_csv_mb:.1f} MB estimated CSV size")
    print()
    print(f"  {BOLD}Split method:{RESET}")
    print(f"    {GREEN}1.{RESET} Maximum file size (MB)")
    print(f"    {GREEN}2.{RESET} Number of equal parts")
    print(f"    {GREEN}3.{RESET} Number of rows per file")
    print(f"    {GREEN}4.{RESET} No split (single file)")
    print()

    choice = ask_int("Choice", 1, minimum=1)

    if choice == 1:
        max_mb = ask_float("Maximum size per part (MB)", 30, minimum=0.5)
        n_parts_est = math.ceil(est_csv_mb / max_mb)
        print(f"\n  Estimate: {YELLOW}~{n_parts_est} files{RESET} of ~{max_mb:.0f} MB each\n")
        split_by_max_size(lines_gen_factory(), base_name, max_mb)

    elif choice == 2:
        default_n = max(2, math.ceil(est_csv_mb / 30))
        n = ask_int("Number of parts", default_n, minimum=2)
        rows_each = math.ceil(total_lines / n)
        mb_each = est_csv_mb / n
        print(f"\n  → {YELLOW}{rows_each:,} rows/part{RESET}, ~{mb_each:.1f} MB each\n")
        split_by_n_parts(lines_gen_factory(), base_name, n, total_lines)

    elif choice == 3:
        # Estimate rows for ~30 MB
        bytes_per_row = (est_csv_mb * 1024 * 1024) / total_lines if total_lines else 150
        default_rows = int(30 * 1024 * 1024 / bytes_per_row)
        rows = ask_int("Rows per file", default_rows, minimum=100)
        n_parts_est = math.ceil(total_lines / rows)
        print(f"\n  Estimate: {YELLOW}~{n_parts_est} files{RESET} "
              f"of {rows:,} rows each\n")
        split_by_rows(lines_gen_factory(), base_name, rows)

    elif choice == 4:
        out = f"{base_name}.csv"
        print(f"\n  Output: {out}\n")
        no_split(lines_gen_factory(), out)

    else:
        print(f"  {RED}Invalid choice.{RESET}")
        sys.exit(1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global HEADER_LINE
    banner()

    # Determine input
    if len(sys.argv) >= 3:
        # Direct mode: bin_to_csv.py input output
        bin_file = sys.argv[1]
        csv_file = sys.argv[2]
        if not os.path.isfile(bin_file):
            print(f"  {RED}File not found: {bin_file}{RESET}")
            sys.exit(1)
        hdr, _ = read_file_header(bin_file)
        if hdr:
            print(f"  Firmware: {hdr['firmware_version']}, record_size={hdr['record_size']}B")
            _, hdr_cols, _ = get_format(hdr['record_size'])
            HEADER_LINE = ','.join(hdr_cols) + '\n'
        print(f"  Direct conversion: {bin_file} → {csv_file}")
        total = bin_record_count(bin_file)
        no_split(bin_to_csv_lines(bin_file, total), csv_file)
        return

    if len(sys.argv) == 2:
        input_file = sys.argv[1]
        if not os.path.isfile(input_file):
            print(f"  {RED}File not found: {input_file}{RESET}")
            sys.exit(1)
    else:
        input_file = find_input_file()

    ext = os.path.splitext(input_file)[1].lower()
    base = os.path.splitext(input_file)[0]

    if ext == '.bin':
        # Detect FileHeader (v0.9.7+)
        hdr, hdr_offset = read_file_header(input_file)
        if hdr:
            print(f"  {GREEN}FileHeader detected:{RESET} firmware {BOLD}{hdr['firmware_version']}{RESET}, "
                  f"record_size={hdr['record_size']}B, "
                  f"header_v{hdr['header_version']}, "
                  f"start={hdr['start_time_ms']}ms")
            # Update HEADER_LINE for the detected record format
            _, hdr_cols, _ = get_format(hdr['record_size'])
            HEADER_LINE = ','.join(hdr_cols) + '\n'
        else:
            print(f"  {DIM}Legacy file (no header) — record_size={CHUNK_SIZE_DEFAULT}B{RESET}")

        total_lines = bin_record_count(input_file)
        # Estimate CSV size: updated dynamically from the first record
        bytes_per_row_est = len(HEADER_LINE) * 0 + 150  # base heuristic

        # Sample one row for a better estimate
        for sample_line in bin_to_csv_lines(input_file, None):
            bytes_per_row_est = len(sample_line.encode('utf-8')) + 1
            est_csv_mb = (total_lines * bytes_per_row_est) / (1024 * 1024)
            break

        print(f"  {BOLD}Input:{RESET} {input_file}")
        print(f"  {DIM}{os.path.getsize(input_file) / (1024*1024):.1f} MB binary, "
              f"{total_lines:,} records, ~{est_csv_mb:.1f} MB as CSV{RESET}")

        def gen_factory():
            return bin_to_csv_lines(input_file, total_lines)

        split_menu(gen_factory, base, total_lines, est_csv_mb)

    elif ext == '.csv':
        file_mb = os.path.getsize(input_file) / (1024 * 1024)
        print(f"  {BOLD}Input:{RESET} {input_file} ({file_mb:.1f} MB)")
        print(f"  {DIM}Counting rows...{RESET}", end=' ', flush=True)
        total_lines = count_csv_lines(input_file)
        print(f"{total_lines:,} data rows")

        if file_mb < 31:
            print(f"\n  {GREEN}File is already under 31 MB — split not needed.{RESET}")
            confirm = ask("Split anyway? (y/n)", "n")
            if confirm.lower() not in ('y', 'yes'):
                return

        def gen_factory():
            return csv_to_lines(input_file)

        split_menu(gen_factory, base, total_lines, file_mb)

    else:
        print(f"  {RED}Unsupported format: {ext}{RESET}")
        print(f"  Use a .bin (binary telemetry) or .csv file")
        sys.exit(1)


if __name__ == '__main__':
    main()
