# Firmware build, flash & monitor

The firmware is a PlatformIO + Arduino project rooted at `orca-controller/`. Single build env: `esp32doit-devkit-v1` (ESP32 DevKit v1, 240 MHz dual-core).

## `pio` is not on PATH

PlatformIO ships with the VS Code extension. The CLI lives at `~/.platformio/penv/bin/pio`. Start every shell with:

```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"
cd orca-controller
```

If that path does not exist, PlatformIO Core is not installed — install the VS Code PlatformIO extension or run `pip install platformio`.

## Commands (run from `orca-controller/`)

| Goal | Command |
| --- | --- |
| Compile only (fast check after edits) | `pio run` |
| Compile + upload | `pio run -t upload` |
| Upload + open monitor | `pio run -t upload -t monitor` |
| Serial monitor only | `pio device monitor` |
| List serial ports | `pio device list` |
| Clean | `pio run -t clean` |
| Re-resolve libraries | `pio pkg install` |
| Unit tests | `pio test -e esp32doit-devkit-v1` |

Monitor baud is **115200** (set in `platformio.ini`). `main.cpp` prints a banner on boot and a telemetry line at 10 Hz once a controller connects.

## After making code changes

Always run `pio run` (compile-only) before treating a change as done — it is the only automated check in this repo. Do not claim a flash succeeded without actually running `-t upload` and seeing it return success; a board may not be connected.

## Common failures

- **`could not open port` / no port found** — board not plugged in, or another process (VS Code monitor, `screen`) holds the port. Run `pio device list`; close other monitors.
- **`A fatal error occurred: Failed to connect to ESP32`** — hold the BOOT button during upload, or the USB cable is power-only.
- **Undefined reference to a local lib class** — the lib needs `orca-controller/lib/<Name>/library.json`; PlatformIO only auto-links lib folders that have one.
- **Third-party lib not found** — add it to `lib_deps` in `platformio.ini` with a pinned `^version`, then `pio pkg install`.
- **Brownout / random reboots when motors run** — power issue, not firmware.

## Tests

`test/` currently has no test files, and `pio test` against the ESP32 needs a connected board. For host-compilable logic (PID math, kinematics), add an `[env:native]` to `platformio.ini` and Unity tests under `test/` so they run without hardware.
