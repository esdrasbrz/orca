# Orca Robot

Orca is a two-wheel differential drive indoor mobile robot designed for precise navigation, Xbox Bluetooth teleoperation, 2D LiDAR mapping (SLAM), and autonomous navigation.

---

## Overview

- **Locomotion:** 3WD differential drive (2 driven wheels + 1 omnidirectional caster wheel).
- **Low-Level Controller:** ESP32 DevKit running FreeRTOS/PlatformIO for motor PWM, encoder reading, IMU fusion, and Xbox BLE teleoperation.
- **High-Level Compute (Phase 2):** Raspberry Pi running ROS 2 for SLAM and Nav2 navigation.
- **Power:** 3S2P 18650 Li-ion pack (11.1V nominal, 12.6V max) with integrated BMS and XL4015 buck converter (5.15V logic rail).

For full hardware specifications and engineering decisions, see [docs/orca-spec.md](docs/orca-spec.md).

---

## Roadmap

- [x] **Architecture & Hardware Specification:** System design and component selection.
- [ ] **Phase 1 (Current): Low-Level Control & Teleoperation**
  - [x] Xbox BLE controller interface and live telemetry.
  - [ ] Dual BTS7960 motor driver control (20 kHz PWM).
  - [ ] MG310 quadrature encoder feedback via ESP32 PCNT peripheral.
  - [ ] BNO085 IMU integration (Game Rotation Vector mode).
  - [ ] Closed-loop PID speed and heading stabilization.
- [ ] **Phase 2: Autonomy & Perception**
  - [ ] Mount Raspberry Pi and 360° planar LiDAR.
  - [ ] Micro-ROS / serial bridge (`/cmd_vel`, `/wheel/odom`, `/imu/data`).
  - [ ] Sensor fusion (EKF) and SLAM mapping (Cartographer / SLAM Toolbox).
  - [ ] Nav2 autonomous path planning.
- [ ] **Phase 3: TBD**
