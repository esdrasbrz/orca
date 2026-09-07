#ifndef __XBOX_CONTROLLER_H__
#define __XBOX_CONTROLLER_H__

#include <Arduino.h>

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

/**
 * Clean, lightweight interface for the Xbox Wireless Controller on ESP32.
 *
 * - Joysticks normalized to [-1.00, +1.00] with deadband and forward-positive Y.
 * - Triggers normalized to [0.00, 1.00].
 * - Clean boolean button getters.
 * - Built-in serial telemetry output.
 */
class XboxController {
 public:
  explicit XboxController(const String& targetMac = "");

  // Lifecycle
  void begin();
  void update();

  // Connection status
  bool isConnected() const;
  bool isNewlyConnected() const;
  bool isNewlyDisconnected() const;
  String getAddress() const;

  // Normalized Joysticks [-1.00, +1.00] (Y: Forward = +1.00, Back = -1.00)
  float leftX() const;
  float leftY() const;
  float rightX() const;
  float rightY() const;

  // Normalized Triggers [0.00, 1.00]
  float leftTrigger() const;
  float rightTrigger() const;

  // Buttons
  bool a() const;
  bool b() const;
  bool x() const;
  bool y() const;
  bool lb() const;
  bool rb() const;
  bool start() const;
  bool select() const;
  bool dpadUp() const;
  bool dpadDown() const;
  bool dpadLeft() const;
  bool dpadRight() const;

  // Telemetry output
  void printTelemetry() const;

 private:
  XboxSeriesXControllerESP32_asukiaaa::Core _core;
  String _targetMac;
  bool _wasConnected;
  bool _newlyConnected;
  bool _newlyDisconnected;

  // Cached normalized snapshot
  float _lx, _ly, _rx, _ry, _lt, _rt;
  bool _btnA, _btnB, _btnX, _btnY;
  bool _btnLB, _btnRB, _btnStart, _btnSelect;
  bool _dpadUp, _dpadDown, _dpadLeft, _dpadRight;

  static float normalizeAxis(uint16_t raw, bool invert);
  static float normalizeTrigger(uint16_t raw);
};

#endif  // __XBOX_CONTROLLER_H__
