# Data Repository Migration Plan

This document defines how to move raw telemetry datasets out of the firmware
repository while preserving public traceability for reports and design choices.

## Goal

Keep this repository easy to clone and understand as an open source firmware
project, while moving heavy raw data to a dedicated evidence/data repository.

The firmware repository should keep:

- source code, firmware configuration, and core tools;
- curated reports and plots that explain design decisions;
- small fixtures only when needed for parser or SITL smoke checks;
- dataset manifests with hashes and links to the external data repository.

The data repository should keep:

- full CSV/BIN/JSON telemetry captures;
- long static-bench and diagnostic datasets;
- raw or intermediate analysis artifacts;
- original evidence-pack source files when they are useful for audit history.

## Current Problem

The current tracked tree is about 295 MB. Most of that weight is not firmware or
tooling.

Main contributors:

- `Reports/`: about 244 MB, mostly CSV files.
- `CSV/`: about 6 MB, raw/converted telemetry samples.
- `M5Stack_AT6668_Evidence_Pack_v1.5.2/`: duplicated legacy evidence pack.
- `analysis/reports/`: curated report outputs; relatively small and useful.
- `docs/evidence/`: organized evidence documentation; useful to keep.

The heavy CSV files are cited by reports, so they should not disappear without a
stable reference, hash, and retrieval path.

## New Data Repository

Suggested repository name:

```text
esp32-telemetry-evidence-data
```

Suggested structure:

```text
esp32-telemetry-evidence-data/
  README.md
  LICENSE
  manifests/
    datasets.csv
    datasets.sha256
    report-links.md
  raw/
    bias-drift/
      gsen_test1.csv
      tel_82.csv
      tel_86.csv
      tel_94.csv
      tel_gsen2.csv
      tel_gsen3.csv
      tel_gsen4.csv
    csv-thermal-log/
      test1-DisplaySpento.csv
      test1-displaySu.csv
      test2-DisplaySpento.csv
      test2-displaySu.csv
      test3-DisplaySpento.csv
      test3-displaySu.csv
    telemetry-samples/
      tel_47.bin
      tel_47.csv
      tel_48.bin
      tel_48.csv
    tumble/
      tumble_test_visivo.csv
    pendulum/
      Pendolo_telemetria_pulita5_calibrato.csv
  evidence-original/
    m5stack-at6668-v1.5.2/
      00_report/
      01_artifacts/
      03_public_sources/
  bench/
    static/
    sitl/
  notes/
    migration-notes.md
```

The data repo should include a manifest row for every moved dataset:

```text
dataset_id,path,sha256,size_bytes,source_report,description
bias-drift-tel-86,raw/bias-drift/tel_86.csv,<hash>,<bytes>,analysis/reports/MPU6886_BiasDrift_Report.pdf,Bias drift source capture
```

## Files To Move From Firmware Repo

Move these heavy raw datasets to the data repository.

### Bias Drift CSV

```text
Reports/Bias drift csv & bin/gsen_test1.csv
Reports/Bias drift csv & bin/tel_82.csv
Reports/Bias drift csv & bin/tel_86.csv
Reports/Bias drift csv & bin/tel_94.csv
Reports/Bias drift csv & bin/tel_gsen2.csv
Reports/Bias drift csv & bin/tel_gsen3.csv
Reports/Bias drift csv & bin/tel_gsen4.csv
```

Destination:

```text
raw/bias-drift/
```

### Thermal CSV Logs

```text
Reports/csv_termal_log/test1-DisplaySpento.csv
Reports/csv_termal_log/test1-displaySu.csv
Reports/csv_termal_log/test2-DisplaySpento.csv
Reports/csv_termal_log/test2-displaySu.csv
Reports/csv_termal_log/test3-DisplaySpento.csv
Reports/csv_termal_log/test3-displaySu.csv
```

Destination:

```text
raw/csv-thermal-log/
```

### Other CSV Data

```text
Reports/Pendolo_telemetria_pulita5_calibrato.csv
Reports/tumble_test_visivo.csv
CSV/tel_47.csv
CSV/tel_48.csv
```

Destination:

```text
raw/pendulum/
raw/tumble/
raw/telemetry-samples/
```

### Related BIN Samples

```text
CSV/tel_47.bin
CSV/tel_48.bin
```

Destination:

```text
raw/telemetry-samples/
```

### Legacy Evidence JSON

```text
M5Stack_AT6668_Evidence_Pack_v1.5.2/01_artifacts/M0_Telemetria.json
M5Stack_AT6668_Evidence_Pack_v1.5.2/01_artifacts/M1_Telemetria1.json
M5Stack_AT6668_Evidence_Pack_v1.5.2/01_artifacts/M2_Telemetria2.json
M5Stack_AT6668_Evidence_Pack_v1.5.2/01_artifacts/M3_Telemetria3.json
```

Destination:

```text
evidence-original/m5stack-at6668-v1.5.2/01_artifacts/
```

### Local Untracked Data Candidates

These are currently local/untracked in the firmware repo and should go directly
to the data repo if they are still useful:

```text
data/the yaw problem (mounting)/
tools/sitl_hal/bench-bin-tests-v1.5.0/
tools/sitl_hal/bench-bin-tests/
```

## Files To Keep In Firmware Repo

Keep these in the firmware repo because they support public understanding
without making the clone too heavy:

```text
analysis/reports/*.pdf
analysis/reports/bias-drift-csv-bin/*.png
analysis/reports/scratch_report.txt
docs/evidence/m5stack-at6668-v1.5.2/
docs/datasheets/
```

The report PDFs and plots should cite the moved datasets through
`docs/evidence/DATASETS.md`.

Small fixtures can stay only if they are used by repeatable validation. Current
candidate:

```text
data/samples/tel_47.ld
data/samples/tel_48.ld
```

If they are not used by validation, move them to:

```text
raw/telemetry-samples/
```

## Target Firmware Repo Structure

After migration, this repository should look like this at the top level:

```text
.
  assets/
  design-system/
  docs/
    datasheets/
    evidence/
      DATASETS.md
      m5stack-at6668-v1.5.2/
  firmware/
  src/
  tools/
  platformio.ini
  README.md
  LICENSE
  AGENTS.md
```

Remove these tracked legacy/raw-data roots from the firmware repo:

```text
CSV/
Reports/
M5Stack_AT6668_Evidence_Pack_v1.5.2/
```

Keep `analysis/reports/` only if it contains curated reports and plots, not raw
CSV/BIN/JSON datasets.

## Firmware Repo Changes To Make

1. Add `docs/evidence/DATASETS.md`.
2. Add dataset links and SHA256 values for every moved dataset.
3. Update report references that still point to `Reports/...` or `CSV/...`.
4. Update `README.md` to explain that full datasets live in the data repo.
5. Keep or add `.gitignore` rules for raw datasets:

```gitignore
*.bin
*.csv
*.json
Reports/
CSV/
data/raw/
tools/sitl_hal/bench-bin-tests*/
```

6. Remove moved files from tracking in the firmware repo.
7. Verify that firmware builds and parser tooling do not require removed files.
8. Keep only small validation fixtures if needed.

## Data Repo Changes To Make

1. Create the new repository.
2. Add a README explaining that it contains raw datasets for the firmware repo.
3. Copy the moved files into the target structure.
4. Generate `manifests/datasets.sha256`.
5. Generate `manifests/datasets.csv`.
6. Add `manifests/report-links.md` mapping reports to raw datasets.
7. Tag the initial import, for example:

```text
evidence-data-v1
```

8. Link that tag from the firmware repo's `docs/evidence/DATASETS.md`.

## Suggested Migration Procedure

### Phase 1: Prepare Data Repo

Create the repository and copy the raw files:

```powershell
mkdir C:\Users\1305n\Desktop\esp32-telemetry-evidence-data
cd C:\Users\1305n\Desktop\esp32-telemetry-evidence-data
git init
```

Copy the selected files preserving semantic groups, not the old root layout.

Generate hashes:

```powershell
Get-ChildItem -Recurse -File |
  Where-Object { $_.FullName -notmatch '\\.git\\' } |
  ForEach-Object {
    $hash = Get-FileHash -Algorithm SHA256 -LiteralPath $_.FullName
    "$($hash.Hash.ToLower())  $($_.FullName.Replace((Get-Location).Path + '\\', ''))"
  } | Set-Content manifests/datasets.sha256
```

Commit the data repo:

```powershell
git add .
git commit -m "Import telemetry evidence datasets"
git tag evidence-data-v1
```

### Phase 2: Add Dataset Manifest To Firmware Repo

Create:

```text
docs/evidence/DATASETS.md
```

For each moved dataset, include:

- dataset ID;
- original firmware-repo path;
- new data-repo path;
- SHA256;
- size;
- reports or plots derived from it;
- notes on whether a small fixture remains in this repo.

### Phase 3: Remove Raw Data From Firmware Repo

Remove only after the data repo commit/tag exists:

```powershell
git rm -r -- CSV Reports M5Stack_AT6668_Evidence_Pack_v1.5.2
```

If any curated report needs to stay, restore it before committing or move it
under `analysis/reports/` or `docs/evidence/`.

Commit:

```powershell
git add .gitignore README.md docs/evidence/DATASETS.md
git commit -m "Move raw telemetry datasets to evidence data repository"
```

### Phase 4: Validate

Run:

```powershell
git diff --check
```

Then run the project validation once `tools/validate.ps1` is committed:

```powershell
.\tools\validate.ps1 -Target docs
.\tools\validate.ps1 -Target python
```

If firmware logic is untouched, firmware builds are optional but useful before a
public cleanup push.

### Phase 5: Optional History Cleanup

Removing files with `git rm` makes the current branch lighter in GitHub's file
view, but old blobs remain in history. A normal clone can still fetch them.

For a truly light public repository, rewrite history after the move is verified:

- backup local branch;
- remove large raw paths from all history;
- verify final tree;
- force-push with `--force-with-lease`.

Candidate paths for history removal:

```text
Reports/
CSV/
M5Stack_AT6668_Evidence_Pack_v1.5.2/
```

Only do this after the data repository is complete and tagged.

## Open Decisions

- Keep or move `data/samples/tel_47.ld` and `data/samples/tel_48.ld`.
- Keep `analysis/reports/*.pdf` in the firmware repo or mirror them in the data
  repo as well.
- Decide whether the data repo should be public immediately or only after the
  firmware repo references and hashes are complete.
- Decide whether to rewrite history or accept that large blobs remain in old
  commits.
