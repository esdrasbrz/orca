# AGENTS.md

Guidance for AI coding assistants working in this repository. Tool-specific entry points (`CLAUDE.md`, `GEMINI.md`) are symlinks to this file.

Reusable task skills live in **`.agents/skills/`** (`orca-flash`, `orca-arch`) — each a `SKILL.md` that carries a trigger description and points at the detailed guide in `docs/`. `.claude/skills` is a symlink to `.agents/skills` so Claude Code and Antigravity both discover the same set.

## What this is

ORCA is a two-wheel differential-drive indoor robot. This repo currently contains **only the ESP32 low-level firmware** (`orca-controller/`, PlatformIO + Arduino framework) plus the engineering docs. Phase 2 (Raspberry Pi, ROS 2, LiDAR SLAM) is planned but not in this repo yet.

Design source of truth, keep in sync with code:

- `docs/orca-spec.md` — hardware selection, electrical topology, project roadmap.
- `docs/motor-control-architecture.md` — drivetrain software layers, control topology, exact C++ interface specs per class.

Working guides:

- `docs/firmware-build.md` — build, flash, serial-monitor, and test workflow.
- `docs/firmware-conventions.md` — layer boundaries, FreeRTOS core split, control-loop rules, safety invariants, GPIO constraints.

## Commands

`pio` is **not on `PATH`** (PlatformIO is installed via the VS Code extension). Add it first:

```bash
export PATH="$HOME/.platformio/penv/bin:$PATH"
```

All commands run from `orca-controller/`:

| Task                         | Command                           |
| ---------------------------- | --------------------------------- |
| Compile                      | `pio run`                         |
| Compile + flash              | `pio run -t upload`               |
| Serial monitor (115200 baud) | `pio device monitor`              |
| Flash + monitor              | `pio run -t upload -t monitor`    |
| Clean build                  | `pio run -t clean`                |
| Unit tests                   | `pio test -e esp32doit-devkit-v1` |

Single env only: `esp32doit-devkit-v1`. `test/` exists but has no tests yet. Always run `pio run` (compile-only) before reporting a firmware change as done — it is the only automated check in this repo. See `docs/firmware-build.md` for failure diagnostics.

## Architecture

### Layered drivetrain (planned — only the Xbox layer exists today)

`main.cpp` currently just reads the controller and prints telemetry. The full stack described in `docs/motor-control-architecture.md §5` is:

```
main.cpp (FreeRTOS tasks, state machine, teleop)
  → DifferentialDrive   (lib/DifferentialDrive)  — kinematics, heading-lock PID, slew limiter, BLE watchdog
  → ClosedLoopMotor     (lib/ClosedLoopMotor)    — per-wheel velocity PID, encoder feedback
  → BTS7960Motor        (lib/BTS7960Motor)       — LEDC PWM, direction, braking; no knowledge of geometry
```

A layer may only call the layer directly below it. Public interfaces for all three classes are already specified in `docs/motor-control-architecture.md §5` (each constructor takes a `*Config` struct) — implement to that signature; if a spec must change, change the doc in the same commit.

### Local libraries

Each `orca-controller/lib/<Name>/` with a `library.json` is auto-linked by PlatformIO — no `lib_deps` entry needed. Third-party deps go in `platformio.ini` `lib_deps` with a pinned `^version`. `XboxController` wraps `asukiaaa/XboxSeriesXControllerESP32`: it normalizes sticks to `[-1, +1]` with **forward = +1**, 0.05 deadzone, triggers to `[0, 1]`, and zeroes all inputs when disconnected or awaiting the first packet.

### ESP32 dual-core split

- **Core 1:** BLE stack (`XboxController`), Wi-Fi, serial logging.
- **Core 0:** one high-priority FreeRTOS task at a fixed 50–100 Hz (`vTaskDelayUntil`) calling `drivetrain.update()`. Keep it non-blocking: no `delay()`, no `Serial` output, no blocking I2C, no dynamic allocation.

### Cascaded control

Outer heading-lock PID (setpoint ω, feedback = BNO085 gyro Z rate) lives in `DifferentialDrive` and trims **both** wheel setpoints; it only engages for near-straight driving. Inner velocity PID (setpoint m/s, feedback = encoder ticks/s) lives per-wheel in `ClosedLoopMotor` with feedforward `Kff` + anti-windup.

### Safety invariants — never regress

- **BLE watchdog:** no drive command within `watchdogTimeoutMs` (default 250 ms) → immediate brake.
- **Slew-rate limiter:** duty/velocity changes ramp at `maxSlewRate`. Abrupt +1→−1 reversals draw 2.5–3.5 A/motor; the system fuse is 5–7.5 A and the pack BMS trips on overcurrent.
- Default state on fault / disconnect / boot is brake or coast, never "last value".

### Phase 2 seam

`DifferentialDrive::setTwist(linearX, angularZ)` mirrors `geometry_msgs/msg/Twist`; `getWheelStates(...)` feeds `/wheel/odom`. Keep these matching the ROS 2 message shapes so the Micro-ROS transition needs no changes above Layer 2.

## Hardware constraints (easy to get wrong in firmware)

- PWM must be **20 kHz, 10-bit via the ESP32 LEDC peripheral**, not `analogWrite`.
- Do not use strapping pins GPIO **0, 2, 12, 15** for motor PWM or encoders.
- Battery-voltage ADC must be on **ADC1** (GPIO 34/35/36/39); ADC2 pins are unreadable while BLE/Wi-Fi is active.
- GPIO **21/22** reserved for the BNO085 I2C bus.
- Full pin allocation table: `docs/motor-control-architecture.md §3` — update table and code together.

## Third-party library plan (architecture doc §6)

- Encoders: `madhephaestus/ESP32Encoder @ ^0.12.0` (hardware PCNT).
- PID: `dlloydev/QuickPID @ ^3.1.9` or a custom discrete PID.
- IMU: `Adafruit_BNO08x` / SparkFun BNO080 (SHTP over I2C, Game Rotation Vector mode, magnetometer disabled).
