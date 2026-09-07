---
name: orca-arch
description: Architecture and coding conventions for the ORCA ESP32 firmware — the layered drivetrain contract, ESP32 core split, control-loop rules, safety invariants, and GPIO constraints. Use when writing or reviewing firmware in orca-controller/ (motor drivers, encoders, PID, IMU, teleop, FreeRTOS tasks) or adding a library.
---

# ORCA firmware architecture & conventions

Full detail lives in **[`docs/firmware-conventions.md`](../../../docs/firmware-conventions.md)** (vendor-neutral, also referenced from `AGENTS.md`). Read that file, plus the design source of truth it points to: `docs/motor-control-architecture.md` (frozen interface specs) and `docs/orca-spec.md` (hardware, electrical, roadmap).

Non-negotiables to keep in mind:

- Layers call only one level down: `main.cpp` → `DifferentialDrive` → `ClosedLoopMotor` → `BTS7960Motor`. Implement class interfaces exactly as spec'd in `motor-control-architecture.md §5`; change the doc in the same commit if a signature must change.
- Core 0 control loop (50–100 Hz, `vTaskDelayUntil`) stays deterministic — no `delay()`, `Serial`, blocking I2C, or allocation. BLE/logging live on Core 1.
- Never regress the 250 ms deadman-brake watchdog or the slew-rate limiter.
- PWM = 20 kHz / 10-bit / LEDC. Avoid strapping pins 0/2/12/15; battery ADC on ADC1 only; GPIO 21/22 reserved for BNO085.
