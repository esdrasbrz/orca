#include "BTS7960Motor.h"

static uint8_t s_nextLedcChannel = 0;

BTS7960Motor::BTS7960Motor(const BTS7960Config& config)
    : _config(config),
      _currentSpeed(0.0f),
      _enabled(false),
      _maxDuty((1U << config.pwmResolution) - 1),
      _channelRPWM(0xFF),
      _channelLPWM(0xFF) {}

void BTS7960Motor::begin() {
  if (_channelRPWM == 0xFF || _channelLPWM == 0xFF) {
    _channelRPWM = s_nextLedcChannel++;
    _channelLPWM = s_nextLedcChannel++;
  }

  ledcSetup(_channelRPWM, _config.pwmFreq, _config.pwmResolution);
  ledcAttachPin(_config.pinRPWM, _channelRPWM);

  ledcSetup(_channelLPWM, _config.pwmFreq, _config.pwmResolution);
  ledcAttachPin(_config.pinLPWM, _channelLPWM);

  if (_config.pinEN >= 0) {
    pinMode(_config.pinEN, OUTPUT);
    digitalWrite(_config.pinEN, HIGH);
  }

  _enabled = true;
  brake();
}

void BTS7960Motor::setSpeed(float speed) {
  if (speed > 1.0f) {
    speed = 1.0f;
  } else if (speed < -1.0f) {
    speed = -1.0f;
  }

  _currentSpeed = speed;

  if (_config.pinEN >= 0 && !_enabled) {
    digitalWrite(_config.pinEN, HIGH);
    _enabled = true;
  }

  applyOutput(_currentSpeed);
}

void BTS7960Motor::brake() {
  _currentSpeed = 0.0f;
  if (_config.pinEN >= 0 && !_enabled) {
    digitalWrite(_config.pinEN, HIGH);
  }
  _enabled = true;
  applyOutput(0.0f);
}

void BTS7960Motor::coast() {
  _currentSpeed = 0.0f;
  applyOutput(0.0f);
  if (_config.pinEN >= 0) {
    digitalWrite(_config.pinEN, LOW);
  }
  _enabled = false;
}

float BTS7960Motor::getSpeed() const {
  return _currentSpeed;
}

bool BTS7960Motor::isEnabled() const {
  return _enabled;
}

uint8_t BTS7960Motor::getRPWMChannel() const {
  return _channelRPWM;
}

uint8_t BTS7960Motor::getLPWMChannel() const {
  return _channelLPWM;
}

void BTS7960Motor::applyOutput(float speed) {
  float effectiveSpeed = _config.inverted ? -speed : speed;

  if (effectiveSpeed > 0.0f) {
    uint32_t duty = static_cast<uint32_t>(effectiveSpeed * _maxDuty + 0.5f);
    if (duty > _maxDuty) duty = _maxDuty;
    ledcWrite(_channelRPWM, duty);
    ledcWrite(_channelLPWM, 0);
  } else if (effectiveSpeed < 0.0f) {
    uint32_t duty = static_cast<uint32_t>(-effectiveSpeed * _maxDuty + 0.5f);
    if (duty > _maxDuty) duty = _maxDuty;
    ledcWrite(_channelRPWM, 0);
    ledcWrite(_channelLPWM, duty);
  } else {
    ledcWrite(_channelRPWM, 0);
    ledcWrite(_channelLPWM, 0);
  }
}
