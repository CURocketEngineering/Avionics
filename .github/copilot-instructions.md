# Copilot Review Instructions

You are reviewing pull requests for this repository.

## Mandatory First Step
- Before reviewing any code, read `AGENTS.md` at the repository root.
- Treat `AGENTS.md` as required policy for this repo.
- If `AGENTS.md` cannot be read, stop and report that the review is blocked.

## How To Review
- Evaluate all changes against `AGENTS.md` first, then normal best practices.
- Call out violations explicitly and reference the relevant `AGENTS.md` rule.
- Prioritize safety and correctness issues over style suggestions.

## Rules To Enforce From AGENTS.md
- No direct Arduino includes in Avionics code; use `hal/ArduinoHAL.h`.
- Keep host-native compatibility (`pio test -e native` must remain viable).
- Keep C++11 compatibility.
- Preserve include/src parity for non-header-only modules.
- Enforce naming conventions:

## Expected Review Output
- List findings by severity.
- Include file and line references.
- Mention missing tests when behavior changes.
