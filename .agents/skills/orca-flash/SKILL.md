---
name: orca-flash
description: Build, flash, and serial-monitor the ORCA ESP32 firmware with PlatformIO. Use when the user wants to compile, upload to the board, open the serial monitor, run pio test, or diagnose a build/upload/port failure for orca-controller.
---

# Building & flashing ORCA firmware

Full workflow, command table, and failure diagnostics live in **[`docs/firmware-build.md`](../../../docs/firmware-build.md)** (vendor-neutral, also referenced from `AGENTS.md`). Read that file before acting.

Quick reminders:

- `pio` is at `~/.platformio/penv/bin/pio`, not on `PATH`. Run `export PATH="$HOME/.platformio/penv/bin:$PATH"` first, and work from `orca-controller/`.
- After any code change, run `pio run` (compile-only) before reporting it done.
- Don't claim an upload/flash succeeded unless `pio run -t upload` actually returned success — a board may not be attached.
