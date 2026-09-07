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
| Unit tests (on connected board) | `pio test -e esp32doit-devkit-v1` |
| Unit test compile check (no board) | `pio test -e esp32doit-devkit-v1 --without-testing` |

Monitor baud is **115200** (set in `platformio.ini`). `main.cpp` prints a banner on boot and a telemetry line at 10 Hz once a controller connects.

## After making code changes

Always run `pio run` (compile-only) before treating a change as done — it is the only automated check in this repo. Do not claim a flash succeeded without actually running `-t upload` and seeing it return success; a board may not be connected.

## Common failures

- **`could not open port` / `[Errno 35] Could not exclusively lock port`** — board not plugged in, or another process (VS Code monitor, background `pio device monitor`, `screen`) holds the port. Find and terminate the holding process (`lsof /dev/cu.usbserial-*`, `kill <PID>`).
- **`pio test` hangs without output** — `pio test` waits for an attached serial board by default. Use `--without-testing` (e.g. `pio test -e esp32doit-devkit-v1 -f test_differential_drive --without-testing`) to compile tests without hardware.
- **`operation not permitted: pio` or `gpg: keyblock resource ... Permission denied`** — when running inside an isolated agent sandbox, `~/.platformio` and `~/.gnupg` reside outside the workspace. Run commands unsandboxed (bypass sandbox).
- **`A fatal error occurred: Failed to connect to ESP32`** — hold the BOOT button during upload, or the USB cable is power-only.
- **Undefined reference to a local lib class** — the lib needs `orca-controller/lib/<Name>/library.json`; PlatformIO only auto-links lib folders that have one.
- **Third-party lib not found** — add it to `lib_deps` in `platformio.ini` with a pinned `^version`, then `pio pkg install`.
- **Brownout / random reboots when motors run** — power issue, not firmware.

## Tests

On-device Unity tests run on the ESP32 via `pio test -e esp32doit-devkit-v1` (e.g. `test/test_bts7960`). Tests must verify actual hardware peripheral registers (such as LEDC channel duty cycles via `ledcRead()`) or pure mathematical logic. Before running tests, ensure no serial monitor process is holding the port. For host-compilable logic (PID math, kinematics), add an `[env:native]` to `platformio.ini` and Unity tests under `test/` so they run without hardware.
