#ifndef __DIFFERENTIAL_DRIVE_H__
#define __DIFFERENTIAL_DRIVE_H__

#include <Arduino.h>

#include "BTS7960Motor.h"

struct DrivetrainConfig {
  float trackWidthMeters;      // Axle track gauge L (~0.20m for Orca chassis)
  float maxLinearVelocity;     // Maximum linear velocity in m/s (or 1.0f normalized)
  float maxAngularVelocity;    // Maximum angular velocity in rad/s
  float maxSlewRate;           // Max acceleration in (m/s)/s or duty/s (prevents current spikes)
  uint32_t watchdogTimeoutMs;  // Failsafe timeout in ms (Default: 250 ms)

  // Heading Lock PID parameters (Phase 1D seam)
  bool enableHeadingLock;
  float headingKp;
  float headingKi;
  float headingKd;
};

class DifferentialDrive {
 public:
  DifferentialDrive(BTS7960Motor& leftMotor, BTS7960Motor& rightMotor,
                    const DrivetrainConfig& config);

  void begin();

  // Runs on Core 0 at 50-100 Hz: updates watchdog, slew-rate ramping, and motor commands
  void update(float gyroYawRateZ = 0.0f);

  // Teleoperation input from Xbox thumbsticks/triggers [-1.0f, +1.0f]
  // throttle: +1.0f = forward, -1.0f = reverse
  // turn: +1.0f = steer right (CW), -1.0f = steer left (CCW)
  void driveArcade(float throttle, float turn);

  // Unicycle kinematics input (ROS 2 geometry_msgs/Twist equivalent)
  // linearX: m/s (forward positive)
  // angularZ: rad/s (CCW positive per ROS standard)
  void setTwist(float linearX, float angularZ);

  // Failsafes
  void emergencyStop();
  void coast();

  // Telemetry & diagnostics
  bool isWatchdogTriggered() const;
  float getRampedLinear() const;
  float getRampedAngular() const;

  // Odometry query (ticks & velocities for Micro-ROS wheel/odom - Phase 1C seam)
  void getWheelStates(float& outLeftVel, float& outRightVel, int64_t& outLeftTicks,
                      int64_t& outRightTicks) const;

 private:
  BTS7960Motor& _left;
  BTS7960Motor& _right;
  DrivetrainConfig _config;

  unsigned long _lastCommandTime;
  unsigned long _lastUpdateTime;

  float _targetLinear;
  float _targetAngular;
  float _rampedLinear;
  float _rampedAngular;
  float _halfTrack;
  bool _watchdogTriggered;

  portMUX_TYPE _mux;

  void resetTargets();
  void applyOutputs(float vLinear, float vAngular);
};

#endif  // __DIFFERENTIAL_DRIVE_H__
