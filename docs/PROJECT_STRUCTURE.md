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
|   |-- script/                  # Analysis scripts and bias reports
|   `-- sitl_hal/                # Offline replay and validation tools
|-- docs/
|   |-- guides/                  # Manuals, migration guides, protocols
|   |-- datasheets/              # Hardware and sensor datasheets
|   |-- evidence/                # Evidence packs and source material
|   |-- history/                 # Patch notes and historical audits
|   |-- pipeline/                # Mermaid pipeline diagrams
|   `-- llm-context/             # Repomix/context snapshots for LLM review
|-- data/
|   |-- samples/                 # Sample telemetry BIN/CSV/LD files
|   `-- test-drives/             # Test-drive datasets
|-- analysis/
|   `-- reports/                 # Generated reports and validation outputs
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
- Put raw or sample telemetry files in `data/`.
- Put generated reports, validation output, and analysis artifacts in `analysis/`.
- Put screenshots and documentation images in `assets/images/`.
- Keep repository-level agent guidance in `AGENTS.md`.
