#ifndef __BTS7960_MOTOR_H__
#define __BTS7960_MOTOR_H__

#include <Arduino.h>

struct BTS7960Config {
  uint8_t pinRPWM;        // Forward PWM pin
  uint8_t pinLPWM;        // Reverse PWM pin
  int8_t pinEN;           // Enable pin (-1 if tied permanently to HIGH)
  bool inverted;          // Invert direction flag (for mirrored chassis mounting)
  uint32_t pwmFreq;       // PWM frequency in Hz (Default: 20000 Hz)
  uint8_t pwmResolution;  // Resolution in bits (Default: 10 bits -> 0..1023)
};

class BTS7960Motor {
 public:
  explicit BTS7960Motor(const BTS7960Config& config);

  void begin();

  // Set motor duty cycle in normalized range [-1.0f, +1.0f]
  //  +1.0f = Full Forward, 0.0f = Active Stop (Brake), -1.0f = Full Reverse
  void setSpeed(float speed);

  // Active electronic brake (shorts motor terminals to GND via low-side FETs)
  void brake();

  // Freewheel / high-impedance coast (disables H-bridge)
  void coast();

  float getSpeed() const;
  bool isEnabled() const;

  // LEDC hardware channel accessors
  uint8_t getRPWMChannel() const;
  uint8_t getLPWMChannel() const;

 private:
  BTS7960Config _config;
  float _currentSpeed;
  bool _enabled;
  uint32_t _maxDuty;
  uint8_t _channelRPWM;
  uint8_t _channelLPWM;

  void applyOutput(float speed);
};

#endif  // __BTS7960_MOTOR_H__
