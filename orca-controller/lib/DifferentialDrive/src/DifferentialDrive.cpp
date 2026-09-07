#include "DifferentialDrive.h"

#include <math.h>

static inline float clampFloat(float val, float minVal, float maxVal) {
  if (val < minVal)
    return minVal;
  if (val > maxVal)
    return maxVal;
  return val;
}

static inline float rampTowards(float current, float target, float maxDelta) {
  if (target > current + maxDelta)
    return current + maxDelta;
  if (target < current - maxDelta)
    return current - maxDelta;
  return target;
}

DifferentialDrive::DifferentialDrive(BTS7960Motor& leftMotor, BTS7960Motor& rightMotor,
                                     const DrivetrainConfig& config)
    : _left(leftMotor),
      _right(rightMotor),
      _config(config),
      _lastCommandTime(0),
      _lastUpdateTime(0),
      _targetLinear(0.0f),
      _targetAngular(0.0f),
      _rampedLinear(0.0f),
      _rampedAngular(0.0f),
      _halfTrack((config.trackWidthMeters > 0.01f) ? (config.trackWidthMeters * 0.5f) : 0.10f),
      _watchdogTriggered(true) {
  _mux = portMUX_INITIALIZER_UNLOCKED;
}

void DifferentialDrive::resetTargets() {
  portENTER_CRITICAL(&_mux);
  _targetLinear = 0.0f;
  _targetAngular = 0.0f;
  _rampedLinear = 0.0f;
  _rampedAngular = 0.0f;
  _watchdogTriggered = true;
  portEXIT_CRITICAL(&_mux);
}

void DifferentialDrive::begin() {
  _left.begin();
  _right.begin();

  portENTER_CRITICAL(&_mux);
  _lastCommandTime = millis();
  _lastUpdateTime = millis();
  _targetLinear = 0.0f;
  _targetAngular = 0.0f;
  _rampedLinear = 0.0f;
  _rampedAngular = 0.0f;
  _watchdogTriggered = false;
  portEXIT_CRITICAL(&_mux);

  _left.brake();
  _right.brake();
}

void DifferentialDrive::driveArcade(float throttle, float turn) {
  // Positive turn steers right (CW), mapping to negative angular velocity in ROS / CCW
  setTwist(clampFloat(throttle, -1.0f, 1.0f) * _config.maxLinearVelocity,
           -clampFloat(turn, -1.0f, 1.0f) * _config.maxAngularVelocity);
}

void DifferentialDrive::setTwist(float linearX, float angularZ) {
  float clampedLin = clampFloat(linearX, -_config.maxLinearVelocity, _config.maxLinearVelocity);
  float clampedAng = clampFloat(angularZ, -_config.maxAngularVelocity, _config.maxAngularVelocity);

  portENTER_CRITICAL(&_mux);
  _targetLinear = clampedLin;
  _targetAngular = clampedAng;
  _lastCommandTime = millis();
  _watchdogTriggered = false;
  portEXIT_CRITICAL(&_mux);
}

void DifferentialDrive::emergencyStop() {
  resetTargets();
  _left.brake();
  _right.brake();
}

void DifferentialDrive::coast() {
  resetTargets();
  _left.coast();
  _right.coast();
}

bool DifferentialDrive::isWatchdogTriggered() const {
  return _watchdogTriggered;
}

float DifferentialDrive::getRampedLinear() const {
  return _rampedLinear;
}

float DifferentialDrive::getRampedAngular() const {
  return _rampedAngular;
}

void DifferentialDrive::getWheelStates(float& outLeftVel, float& outRightVel, int64_t& outLeftTicks,
                                       int64_t& outRightTicks) const {
  outLeftVel = _rampedLinear - (_rampedAngular * _halfTrack);
  outRightVel = _rampedLinear + (_rampedAngular * _halfTrack);
  outLeftTicks = 0;
  outRightTicks = 0;
}

void DifferentialDrive::update(float gyroYawRateZ) {
  unsigned long now = millis();

  // Atomically read latest heartbeat and setpoints in a single pass
  portENTER_CRITICAL(&_mux);
  unsigned long lastCmd = _lastCommandTime;
  float targetLin = _targetLinear;
  float targetAng = _targetAngular;
  portEXIT_CRITICAL(&_mux);

  // Deadman watchdog check
  if ((now - lastCmd) > _config.watchdogTimeoutMs) {
    emergencyStop();
    _lastUpdateTime = now;
    return;
  }

  // Calculate elapsed time dt in seconds
  float dt = (now - _lastUpdateTime) / 1000.0f;
  _lastUpdateTime = now;
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.02f;  // Fallback to nominal step on jitter or first tick
  }

  // Slew rate limiting for linear and angular velocities
  float maxAngularAcc = _config.maxSlewRate / _halfTrack;
  _rampedLinear = rampTowards(_rampedLinear, targetLin, _config.maxSlewRate * dt);
  _rampedAngular = rampTowards(_rampedAngular, targetAng, maxAngularAcc * dt);

  // Apply kinematic demux and output to motor drivers
  applyOutputs(_rampedLinear, _rampedAngular);
}

void DifferentialDrive::applyOutputs(float vLinear, float vAngular) {
  // Unicycle kinematic demux:
  // v_left  = v - (ω * L / 2)
  // v_right = v + (ω * L / 2)
  float vLeft = vLinear - (vAngular * _halfTrack);
  float vRight = vLinear + (vAngular * _halfTrack);

  // Convert to normalized duties [-1.0f, +1.0f]
  float maxVel = (_config.maxLinearVelocity > 0.001f) ? _config.maxLinearVelocity : 1.0f;
  float dutyLeft = vLeft / maxVel;
  float dutyRight = vRight / maxVel;

  // Desaturation: preserve curvature ratio when duties exceed +/-1.0
  float maxMag = fmaxf(fabsf(dutyLeft), fabsf(dutyRight));
  if (maxMag > 1.0f) {
    dutyLeft /= maxMag;
    dutyRight /= maxMag;
  }

  _left.setSpeed(dutyLeft);
  _right.setSpeed(dutyRight);
}
