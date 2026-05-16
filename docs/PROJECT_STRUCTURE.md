# Project Structure

This repository is organized by purpose. Top-level folders should answer "what is this for?" without needing to open them.

```text
esp32-telemetry-clean/
|-- src/                         # Main PlatformIO firmware source
|-- firmware/
|   |-- test/                    # Sensor bench test firmwares
|   `-- benchmarks/              # Standalone benchmark projects
|-- tools/                       # Python converters, dashboards, SITL, CAN tools
|   |-- can_bus/                 # CAN analysis and helper firmwares
|   |-- script/                  # Small validation and engineering utilities
|   `-- sitl_hal/                # Offline replay and validation tools
|-- docs/
|   |-- datasheets/              # Hardware and sensor datasheets
|   `-- evidence/                # Curated evidence packs and dataset manifest
|-- analysis/
|   `-- reports/                 # Curated PDF reports kept in this repository
|-- assets/
|   `-- images/                  # README and documentation images
|-- design-system/               # UI kit and telemetry browser assets
|-- platformio.ini
|-- wifi_config.example.txt
|-- AGENTS.md
`-- README.md
```

## Folder Rules

- Put firmware code used by the main PlatformIO build in `src/`.
- Put standalone firmware experiments and benchmark projects in `firmware/`.
- Put executable Python tools in `tools/`.
- Put reusable project documentation in `docs/`.
- Keep raw telemetry datasets outside this repository; `docs/evidence/DATASETS.md`
  describes the external evidence-data repository.
- Put only curated final report PDFs in `analysis/reports/`.
- Put screenshots and documentation images in `assets/images/`.
- Keep repository-level agent guidance in `AGENTS.md`.
