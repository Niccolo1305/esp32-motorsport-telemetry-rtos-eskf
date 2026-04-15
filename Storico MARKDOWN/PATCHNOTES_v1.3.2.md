# Patch Notes — v1.3.2

> **Date:** 2026-04-13
> **Based on:** v1.3.1
> **Audit sources:** `data_pipeline_audit.md`, `cross_domain_audit_report.md`

---

## Summary

Four code fixes (1 major, 2 minor, 1 cosmetic) and two documentation updates. No changes to the 127-byte binary record format, SD file header, or MQTT payload structure — files recorded on v1.3.1 remain compatible.

---

## Changes

### [FIX] Recalibration now fully resets navigation state — `MAJOR`

**Files:** `src/globals.h`, `src/globals.cpp`, `src/calibration.cpp`, `src/filter_task.cpp`

**Problem:** A mid-session manual recalibration (long-press ≥1.5 s) reset the Madgwick quaternion, gyro/accel biases, and EMA previous values, but left the following stale:
- ESKF 5D and 6D state matrices (`X`, `P`)
- GPS ENU origin (`gps_origin_set`, `gps_origin_lat/lon`)
- ZARU variance buffers (`gz/gx/gy_var_buf`, `gz_var_idx`, `gz_var_count`)
- Thermal biases (`thermal_bias_gz/gx/gy`)
- COG tracking state (`cog_ref_east/north`, `prev_cog_rad`, `has_cog_ref`, `cog_variation`)
- EMA previous values would reset to 0.0f, causing a single-sample spike of up to ~0.5g

**Effect of the bug:** After recalibration, the ESKF heading was desynchronised from the new Madgwick quaternion (old heading in X[4], new quaternion). The ZARU variance buffer contained pre-calibration data for up to 1 second, potentially triggering a false stationary condition. The trajectory SD log showed a visible artefact until the next GPS correction re-anchored the ESKF.

**Fix:**
- Added `std::atomic<bool> recalibration_pending` global flag
- `calibrate_alignment()` sets the flag under `telemetry_mutex` after writing new biases (before releasing the mutex)
- All 14 formerly-static locals in `Task_Filter` (variance buffers, COG tracking, thermal biases, GPS cache) promoted to function-scope so they are accessible from the reset block
- `Task_Filter` checks the flag at the top of each iteration (inside `telemetry_mutex`). On detection: resets ESKF state, GPS origin, all buffers and biases, COG tracking, timing variables; issues `continue` to discard the stale IMU sample that accumulated during the 2 s calibration window
- EMA is seeded from the first post-calibration raw sample instead of 0.0f, eliminating the single-sample output spike

**Behavioral change:** One IMU sample (20 ms) is discarded immediately after recalibration. The GPS origin is reset, so position resets to (0, 0) ENU until the next GPS fix establishes a new origin. This is the correct behaviour — a recalibration changes the sensor frame, making the old position meaningless.

---

### [FIX] ESKF `correct()` uses local R matrix — `MINOR`

**File:** `src/eskf.h` — both `ESKF2D::correct()` and `ESKF_6D::correct()`

**Problem:** The GPS position measurement noise matrix `R_` was a class member modified in-place during `correct()`. When the Mahalanobis Innovation Gate fired (GPS outlier, d² > 11.83), `R_` was inflated 50× and the inflated value persisted in the member until the next `correct()` call. This was safe because `R_` was always overwritten at the top of `correct()`, but any future code path reading `R_` between calls would inherit the inflated value.

**Fix:** Replaced member `R_` writes in `correct()` with a local `Matrix<2,2> R_pos`. The member `R_` is no longer mutated during correction. Behaviorally identical — same computation, same Joseph form covariance update.

---

### [FIX] Boot-time FSR register verification — `INFO→FIX`

**File:** `src/Telemetria.ino`

**Problem:** The firmware assumed M5Unified's `M5.begin()` sets the MPU-6886 to ±8g / ±2000 dps without verifying the register state. A library update changing the defaults would silently corrupt all acceleration and gyro readings with no indication.

**Fix:** After `Wire.begin()` and before the DLPF register writes, read-back GYRO_CONFIG (0x1B) and ACCEL_CONFIG (0x1C). Check bits 4:3 against expected values (0b11 = ±2000 dps, 0b10 = ±8g). If mismatched, log the error to Serial and display "FSR: ERROR!" on the LCD for 2 seconds. Non-blocking — the firmware continues to DLPF setup regardless.

---

### [FIX] `motec_exporter.py` format label for 127-byte files — `COSMETIC`

**File:** `Tool/motec_exporter.py`

**Problem:** The format label printed after reading a binary file only distinguished `'122B (raw+6D)'` and `'78B (EMA only)'`. Files recorded on v1.3.1+ (127-byte records) were silently labeled as 122B.

**Note:** The 127-byte parsing itself (`RECORD_FMT_127`, branching in `read_bin()`) was already correctly implemented in the current code — the audit that flagged CROSS-MINOR-1 was based on an older snapshot.

**Fix:** 3-way label check using presence of the `zaru_flags` key (unique to 127-byte field set) to correctly print `'127B (raw+6D+ZARU)'`.

---

### [DOC] Pipeline comment block updated to match v1.3.2

**File:** `src/Telemetria.ino` lines 66-135

The comment block documenting the IMU pipeline was last updated before v1.1.1 and contained multiple stale descriptions:
- Beta described as "linear" (was dual-gate since v1.1.1)
- ZARU described as 1-axis gz-only (was 3-axis since v1.2.1)
- Missing: Sanity Gate (v0.9.9), Straight-Line ZARU (v0.9.7→v1.3.1), NHC (v1.3.1), ESKF_6D shadow (v0.9.8), GPS staleness (v0.9.11), COG variation (v1.3.1), dynamic dt

Rewritten to accurately reflect the v1.3.2 pipeline with all active features documented.

---

### [DOC] `CLAUDE.md` version reference updated to v1.3.2

---

## Not Changed

| Finding | Reason |
|---------|--------|
| `telemetry_mutex` scope reduction (CROSS-MINOR-3) | Risk of race with `calibrate_alignment()` EMA access; 500µs hold is 2.5% of the 20ms period — no practical issue |
| Second mutex 2ms timeout (PIPELINE-MINOR-3) | 1-cycle stale data (20ms) — not observable |
| `prev_cog_rad` initialised to 0 (PIPELINE-MINOR-4) | One-shot transient; Straight-Line ZARU requires >40 km/h, cannot activate in first seconds |
| Automated stack overflow guard (CROSS-INFO-5) | Risk of false-positive task kill; MQTT heartbeat watermark reporting is sufficient |
| `norm_q_sq > 0.0f` exact comparison (CROSS-INFO-6) | Cannot be zero in practice (sum of 4 squares from active quaternion) |
| float32 ENU precision for long sessions (CROSS-MINOR-2) | Adequate for sessions ≤30 min; no fix without double FPU |
| `xQueueCreate(1)` + `xQueueOverwrite` comment (CROSS-MINOR-4) | Intentional design; depth=1 is the documented requirement for `xQueueOverwrite` |
