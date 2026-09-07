# Firmware architecture & coding conventions

Design source of truth: `motor-control-architecture.md` (software layers + frozen interface specs) and `orca-spec.md` (hardware, electrical, roadmap). **Keep these docs in sync with the code** — if you change a pin, an interface, or a control parameter, update the doc in the same commit.

## Layered drivetrain — respect the boundaries

```
main.cpp                    FreeRTOS tasks, robot state machine, teleop mapping
  → DifferentialDrive       kinematics (unicycle → per-wheel), heading-lock PID,
     (lib/DifferentialDrive) slew-rate limiter, BLE deadman watchdog
  → ClosedLoopMotor         single-wheel velocity PID, encoder → m/s, feedforward
     (lib/ClosedLoopMotor)   + anti-windup
  → BTS7960Motor            LEDC PWM duty, direction pins, brake/coast
     (lib/BTS7960Motor)      — knows nothing about geometry, encoders, or the other wheel
```

Rules:
- A layer may only call the layer directly below it. `BTS7960Motor` has no encoder or track-width knowledge; `ClosedLoopMotor` has no track-width or heading knowledge.
- Public interfaces for all three classes are **already specified** in `motor-control-architecture.md §5`. Implement to that signature exactly (each constructor takes a `*Config` struct: `BTS7960Config`, `WheelPIDConfig`, `DrivetrainConfig`). If a spec needs to change, change the doc and say so.
- Each layer must be bring-up testable alone (open-loop `BTS7960Motor` on the bench before `ClosedLoopMotor` exists, etc.).

## ESP32 core split (FreeRTOS)

- **Core 1** — BLE (`XboxController`), Wi-Fi, all `Serial` logging.
- **Core 0** — ONE high-priority task, fixed 50–100 Hz via `vTaskDelayUntil` (not `vTaskDelay`), body = read gyro → heading trim → wheel PIDs → PWM out via `drivetrain.update(dt)`.
- The Core 0 loop must stay deterministic: no `delay()`, no blocking I2C, no `Serial.print`, no dynamic allocation. Pass telemetry out through a queue or shared struct read by Core 1.

## Control topology

- **Inner loop** (`ClosedLoopMotor`): setpoint = wheel m/s, feedback = `ESP32Encoder` PCNT ticks/s converted with `ticksPerMeter = (PPR * gearRatio) / (PI * wheelDiameter)`. Output = PWM duty `[-1, 1]`. Discrete PID + `Kff * v_target` feedforward + integral clamp anti-windup.
- **Outer loop** (`DifferentialDrive`): setpoint = target ω, feedback = BNO085 calibrated gyro Z rate. Output = Δω trim applied to BOTH wheel setpoints via `v_left = v - ω_eff·L/2`, `v_right = v + ω_eff·L/2`. Only engage heading-lock when |ω_target| ≈ 0.

## Safety invariants — never regress these

- **Deadman watchdog:** if `DifferentialDrive` receives no command within `watchdogTimeoutMs` (default 250 ms), it brakes immediately.
- **Slew-rate limiter:** all duty/velocity changes ramp at `maxSlewRate`. Abrupt reversals draw 2.5–3.5 A per motor; the system fuse is 5–7.5 A and the pack BMS trips on overcurrent.
- Default motor state on fault / disconnect / boot is **brake or coast**, never "last value".

## Units & conventions

- Normalized inputs `[-1, +1]`, **forward = +1**. Physical velocity in m/s, angles/rates in rad and rad/s. `Twist` uses `linearX` (m/s) and `angularZ` (rad/s).
- **Xbox Teleoperation Mapping:**
  - **Forward Throttle:** Right Trigger (`RT`, `[0.00, 1.00]`).
  - **Brake / Reverse:** Left Trigger (`LT`, `[0.00, 1.00]`).
  - **Longitudinal Demand:** `throttle = RT - LT` (`[-1.00, +1.00]`).
  - **Steering / Turn:** Left Stick X (`LX`, `[-1.00, +1.00]`, right = $+1.0$).
- **Kinematic Delegation & Slew Ramping:**
  - `driveArcade(throttle, turn)` delegates directly to `setTwist(throttle * maxLinearVel, -turn * maxAngularVel)` to keep synchronization, clamping, and watchdog timestamps in a single source of truth.
  - Slew rate limiting uses a shared `rampTowards(current, target, maxDelta)` pattern for both linear and angular setpoints.
- Header guards: `#ifndef __CLASS_NAME_H__`.
- Config-struct constructors; no setter soup. Keep `*Config` structs pure C++ aggregates (no in-class default member initializers) so brace/designated initialization works reliably across C++11/14 toolchains.
- `DifferentialDrive::setTwist()` and `getWheelStates()` are the Phase 2 Micro-ROS seam (`/cmd_vel`, `/wheel/odom`) — keep them matching `geometry_msgs/msg/Twist` and `nav_msgs/msg/Odometry`.

## Testing & verification rules

- **No "test theater":** Never write tests that only verify member variable assignments or trivial getters/setters. If a test does not verify real hardware state or pure mathematical algorithms, skip it.
- **Hardware register assertions:** On-device tests (`test/`) must inspect hardware peripheral registers directly (e.g. `ledcRead()` for PWM duty, `digitalRead()` for GPIO state).
- **LEDC timing & latching:** ESP32 LEDC duty updates latch on the next timer cycle (50 µs at 20 kHz); insert a 1–2 ms delay before checking `ledcRead()`. Note that Arduino-ESP32 HAL sets 100% duty on a 10-bit timer to 1024 (`max_duty + 1`) for full saturation.

## GPIO / hardware constraints

- PWM = **20 kHz, 10-bit, ESP32 LEDC peripheral** — never `analogWrite` (audible coil whine below ~20 kHz).
- Never use strapping pins GPIO **0, 2, 12, 15** for PWM or encoders.
- Battery-voltage ADC must be on **ADC1** (GPIO 34/35/36/39); ADC2 is dead while BLE/Wi-Fi is on.
- GPIO **21/22** reserved for BNO085 I2C (400 kHz).
- Canonical pin table: `motor-control-architecture.md §3` — edit table and code together.

## Adding a library

- Local lib: `orca-controller/lib/<Name>/` with `src/` and a `library.json` (PlatformIO auto-links only folders that have one). Wrap third-party APIs in a thin ORCA-named class like `XboxController` does.
- Third-party: add to `platformio.ini` `lib_deps` with a pinned `^version`. Planned set: `madhephaestus/ESP32Encoder @ ^0.12.0`, `dlloydev/QuickPID @ ^3.1.9`, `Adafruit_BNO08x`.
