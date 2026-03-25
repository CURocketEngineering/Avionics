# AGENTS.md

Guidance for AI coding agents working in this repository.

## Scope

These instructions apply to the entire repo unless a deeper `AGENTS.md` overrides them.

## General Guidance

- For broad safety-critical coding principles, refer to `context/P10.md`.

## Project Goals

- Deliver a reusable avionics toolbox that can be utilized across multiple rocket platforms.
- Maintain hardware-agnostic logic so it can be extended to any device running Arduino core, or run natively on a host machine via the custom HAL mocks in `hal/`.
- Prioritize simple, obvious, clear code over clever or over-engineered solutions. The project is maintained by students with beginner-to-intermediate C++ knowledge, and there is high turnover, so it must be easy for new members to start contributing without needing to understand complex design patterns or advanced C++ features.

## Project Overview

Avionics is a modular C++ (Arduino-core compatible) library used for rocket flight computers. Core areas:

- `include/state_estimation` + `src/state_estimation`: launch/burnout/apogee/state logic.
- `include/data_handling` + `src/data_handling`: logging, telemetry, sensor data handling.
- `hal/`: hardware abstraction layer for Arduino vs native host testing.
- `test/`: host-native Unity tests.
- `data/`: CSV fixtures used by many tests.

## Hard Rules

1. Never include Arduino headers directly (`<Arduino.h>`, etc.) from Avionics code.
2. Always include `hal/ArduinoHAL.h` for Arduino-facing APIs.
3. Keep host-native tests buildable; avoid adding dependencies that only exist on embedded targets.
4. Maintain `include/` and `src/` parity for non-header-only modules.
5. Keep changes compatible with the repo's C++ standard (`-std=c++11` in `platformio.ini`).

## Build, Test, and Checks

Run from repo root:

```bash
# Install toolchain (if needed)
pip install -U platformio

# Run unit tests (same environment as CI)
pio test -e native

# Run static analysis (same tool as CI)
pio check -e native --fail-on-defect=low --fail-on-defect=medium --fail-on-defect=high
```

Target a specific test directory when iterating:

```bash
pio test -e native -f test_apogee_detector
```

## Documentation and Doxygen

- Doxygen is configured via `Doxyfile` at the repo root.
- Generate docs locally with:

```bash
doxygen Doxyfile
python -m http.server --directory build/doxygen 8000
```

- Local output is `build/doxygen/` (open `build/doxygen/index.html`).
- Docs are deployed automatically by `.github/workflows/doxygen.yml`:
  - Trigger: pushes to `main` (and manual `workflow_dispatch`)
  - Deploy target: `gh-pages` branch
  - Published folder: `build/doxygen`
- If code changes affect public behavior/API, update doc comments and relevant Markdown docs in the same change.

## Data-Driven Tests

- Many tests require CSV files in `data/`; these are flight data from previous launches.
- If fixtures are missing, use the same dataset source CI uses (`CURocketEngineering/Rocket-Test-Data`, release tag `v1.0.0`) or project-approved local equivalents.
- Do not rename/remove existing CSV fixtures unless the task explicitly requires it.

## Change Guidelines

1. Prefer small, focused patches.
2. If behavior changes, update or add tests in `test/` to cover it.
3. Preserve public API names unless the task explicitly asks for a breaking change.
4. Keep docs in sync when changing module behavior:
   - top-level `README.md` for high-level behavior,
   - `docs/*.md` for detailed formats/protocols.

## Safety and Review Checklist

Before finishing, verify:

1. `pio test -e native` passes (or explain why not run/failing).
2. `pio check -e native ...` is clean for touched code (or explain remaining findings).
3. No direct Arduino includes were introduced.
4. New logic has test coverage for nominal and edge behavior.
