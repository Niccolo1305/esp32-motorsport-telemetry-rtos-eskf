# Evidence Datasets

This firmware repository keeps source code, core tools, datasheets, curated
evidence, and final PDF reports. Full telemetry captures and generated CSV/BIN/
JSON datasets live in a separate evidence-data repository so this project stays
small enough to clone and review as firmware.

Planned external repository:

```text
esp32-telemetry-evidence-data
```

Local staging path used during this cleanup:

```text
F:\esp32-telemetry-evidence-data
```

Before publishing a public release of this cleanup, replace the local path above
with the final GitHub URL and tag, for example:

```text
https://github.com/Niccolo1305/esp32-telemetry-evidence-data/tree/evidence-data-v1
```

## Repository Policy

Keep in this firmware repository:

- firmware and tool source code;
- compact curated PDF reports in `analysis/reports/`;
- datasheets in `docs/datasheets/`;
- curated evidence packs in `docs/evidence/`;
- small metadata/manifest files that explain how to retrieve raw data.

Keep in the evidence-data repository:

- full telemetry CSV/BIN captures;
- generated analysis CSV/JSON files;
- intermediate plots and notes that are not needed for normal firmware review;
- archived legacy evidence packs.

## Data Repository Layout

The transferred data is organized as:

```text
esp32-telemetry-evidence-data/
  manifests/
    analysis-reports-copy-map.csv
    datasets.csv
    datasets.sha256
  raw/
    bias-drift/
    csv-thermal-log/
    telemetry-samples/
    pendulum/
    tumble/
  evidence-curated/
    m5stack-at6668-v1.5.2/
  evidence-original/
    m5stack-at6668-v1.5.2/
  reports/
    final/
    notes/
    plots/
```

## Dataset Groups

| Dataset group | Former firmware path | Evidence-data path | Used by |
|---|---|---|---|
| Bias-drift captures | `Reports/Bias drift csv & bin/*.csv` | `raw/bias-drift/` | `analysis/reports/MPU6886_BiasDrift_Report.pdf` |
| Bias-drift BIN/CSV working set | `analysis/reports/bias-drift-csv-bin/*.{bin,csv}` | `raw/bias-drift/` | Bias-drift plots and report checks |
| Bias-drift plots | `analysis/reports/bias-drift-csv-bin/*.png` | `reports/plots/bias-drift/` | README/report visual references |
| Thermal CSV logs | `Reports/csv_termal_log/*.csv`, `analysis/reports/thermal-csv-logs/*.csv` | `raw/csv-thermal-log/` | `analysis/reports/MPU6886_ThermalDrift_Report.pdf` |
| Pendulum validation | `Reports/Pendolo_telemetria_pulita5_calibrato.csv`, `analysis/reports/Pendolo_telemetria_pulita5_calibrato.csv` | `raw/pendulum/` | `analysis/reports/IMU_Calibration_and_Validation_Report.pdf` |
| Tumble validation | `Reports/tumble_test_visivo.csv`, `analysis/reports/tumble_test_visivo.csv` | `raw/tumble/` | `analysis/reports/Ellipsoid_fitting_Report.pdf` |
| Deprecated tel_47/tel_48 samples | `CSV/tel_47.*`, `CSV/tel_48.*`, `data/samples/tel_47.ld`, `data/samples/tel_48.ld` | `raw/telemetry-samples/` | Historical parser/sample archive |
| AT6668 curated evidence | `docs/evidence/m5stack-at6668-v1.5.2/` | `evidence-curated/m5stack-at6668-v1.5.2/` | GPS SOG/DHV/NAV-PV design rationale |
| AT6668 legacy evidence JSON | `M5Stack_AT6668_Evidence_Pack_v1.5.2/01_artifacts/*.json` | `evidence-original/m5stack-at6668-v1.5.2/01_artifacts/` | Archived original evidence pack |
| AtomS3 vs AtomS3R plots/notes | `analysis/reports/AtomS3_vs_AtomS3R_Sensor_Bench_Whitepaper_assets/`, related `.md/.pdf` | `reports/plots/atom-bench/`, `reports/final/`, `reports/notes/` | Sensor benchmark documentation |

## Files Remaining Here

The firmware repository intentionally keeps these final reports:

```text
analysis/reports/Ellipsoid_fitting_Report.pdf
analysis/reports/IMU_Calibration_and_Validation_Report.pdf
analysis/reports/MPU6886_BiasDrift_Report.pdf
analysis/reports/MPU6886_ThermalDrift_Report.pdf
```

The firmware repository also keeps the curated AT6668 evidence pack:

```text
docs/evidence/m5stack-at6668-v1.5.2/
```

That directory is the canonical in-repo AT6668 documentation. The older root
folder `M5Stack_AT6668_Evidence_Pack_v1.5.2/` was a smaller duplicate and has
been archived in the evidence-data repository before removal from this repo.

## Verification

During the migration, copied files were checked with SHA256 before staged
removal from this repository. Current copied groups verified with zero hash
mismatches:

- raw `Reports/` datasets;
- `analysis/reports/` datasets, plots, notes, and report mirrors;
- curated AT6668 evidence pack;
- deprecated `tel_47` and `tel_48` samples.

The evidence-data repository should publish:

```text
manifests/datasets.sha256
manifests/datasets.csv
manifests/analysis-reports-copy-map.csv
```

Use those manifests as the durable audit trail between report claims and raw
datasets.
