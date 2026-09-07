#include "XboxController.h"

static const float DEADZONE = 0.05f;

XboxController::XboxController(const String& targetMac)
    : _core(targetMac),
      _targetMac(targetMac),
      _wasConnected(false),
      _newlyConnected(false),
      _newlyDisconnected(false),
      _lx(0.0f),
      _ly(0.0f),
      _rx(0.0f),
      _ry(0.0f),
      _lt(0.0f),
      _rt(0.0f),
      _btnA(false),
      _btnB(false),
      _btnX(false),
      _btnY(false),
      _btnLB(false),
      _btnRB(false),
      _btnStart(false),
      _btnSelect(false),
      _dpadUp(false),
      _dpadDown(false),
      _dpadLeft(false),
      _dpadRight(false) {}

void XboxController::begin() {
  _core.begin();
}

void XboxController::update() {
  _core.onLoop();

  bool connected = isConnected();
  _newlyConnected = (connected && !_wasConnected);
  _newlyDisconnected = (!connected && _wasConnected);
  _wasConnected = connected;

  if (connected && !_core.isWaitingForFirstNotification()) {
    // Left stick (Y inverted: 0 is forward on Xbox -> +1.0)
    _lx = normalizeAxis(_core.xboxNotif.joyLHori, false);
    _ly = normalizeAxis(_core.xboxNotif.joyLVert, true);

    // Right stick
    _rx = normalizeAxis(_core.xboxNotif.joyRHori, false);
    _ry = normalizeAxis(_core.xboxNotif.joyRVert, true);

    // Triggers
    _lt = normalizeTrigger(_core.xboxNotif.trigLT);
    _rt = normalizeTrigger(_core.xboxNotif.trigRT);

    // Action buttons
    _btnA = _core.xboxNotif.btnA;
    _btnB = _core.xboxNotif.btnB;
    _btnX = _core.xboxNotif.btnX;
    _btnY = _core.xboxNotif.btnY;
    _btnLB = _core.xboxNotif.btnLB;
    _btnRB = _core.xboxNotif.btnRB;
    _btnStart = _core.xboxNotif.btnStart;
    _btnSelect = _core.xboxNotif.btnSelect;

    // D-Pad
    _dpadUp = _core.xboxNotif.btnDirUp;
    _dpadDown = _core.xboxNotif.btnDirDown;
    _dpadLeft = _core.xboxNotif.btnDirLeft;
    _dpadRight = _core.xboxNotif.btnDirRight;
  } else {
    // Zero out when disconnected or awaiting first packet
    _lx = _ly = _rx = _ry = _lt = _rt = 0.0f;
    _btnA = _btnB = _btnX = _btnY = false;
    _btnLB = _btnRB = _btnStart = _btnSelect = false;
    _dpadUp = _dpadDown = _dpadLeft = _dpadRight = false;
  }
}

bool XboxController::isConnected() const {
  return const_cast<XboxSeriesXControllerESP32_asukiaaa::Core&>(_core).isConnected();
}

bool XboxController::isNewlyConnected() const {
  return _newlyConnected;
}

bool XboxController::isNewlyDisconnected() const {
  return _newlyDisconnected;
}

String XboxController::getAddress() const {
  if (isConnected()) {
    return const_cast<XboxSeriesXControllerESP32_asukiaaa::Core&>(_core).buildDeviceAddressStr();
  }
  return _targetMac;
}

float XboxController::leftX() const {
  return _lx;
}
float XboxController::leftY() const {
  return _ly;
}
float XboxController::rightX() const {
  return _rx;
}
float XboxController::rightY() const {
  return _ry;
}
float XboxController::leftTrigger() const {
  return _lt;
}
float XboxController::rightTrigger() const {
  return _rt;
}

bool XboxController::a() const {
  return _btnA;
}
bool XboxController::b() const {
  return _btnB;
}
bool XboxController::x() const {
  return _btnX;
}
bool XboxController::y() const {
  return _btnY;
}
bool XboxController::lb() const {
  return _btnLB;
}
bool XboxController::rb() const {
  return _btnRB;
}
bool XboxController::start() const {
  return _btnStart;
}
bool XboxController::select() const {
  return _btnSelect;
}
bool XboxController::dpadUp() const {
  return _dpadUp;
}
bool XboxController::dpadDown() const {
  return _dpadDown;
}
bool XboxController::dpadLeft() const {
  return _dpadLeft;
}
bool XboxController::dpadRight() const {
  return _dpadRight;
}

void XboxController::printTelemetry() const {
  char btnBuf[32] = "";
  if (_btnA)
    strcat(btnBuf, "A ");
  if (_btnB)
    strcat(btnBuf, "B ");
  if (_btnX)
    strcat(btnBuf, "X ");
  if (_btnY)
    strcat(btnBuf, "Y ");
  if (_btnLB)
    strcat(btnBuf, "LB ");
  if (_btnRB)
    strcat(btnBuf, "RB ");
  if (_btnStart)
    strcat(btnBuf, "START ");
  if (_btnSelect)
    strcat(btnBuf, "SELECT ");
  if (strlen(btnBuf) > 0 && btnBuf[strlen(btnBuf) - 1] == ' ') {
    btnBuf[strlen(btnBuf) - 1] = '\0';
  }

  char dpadBuf[24] = "";
  if (_dpadUp)
    strcat(dpadBuf, "UP ");
  if (_dpadDown)
    strcat(dpadBuf, "DOWN ");
  if (_dpadLeft)
    strcat(dpadBuf, "LEFT ");
  if (_dpadRight)
    strcat(dpadBuf, "RIGHT ");
  if (strlen(dpadBuf) > 0 && dpadBuf[strlen(dpadBuf) - 1] == ' ') {
    dpadBuf[strlen(dpadBuf) - 1] = '\0';
  }

  Serial.printf(
      "[Axes] LX:%+5.2f LY:%+5.2f | RX:%+5.2f RY:%+5.2f | LT:%4.2f RT:%4.2f | Buttons: [%s] D-Pad: "
      "[%s]\n",
      _lx, _ly, _rx, _ry, _lt, _rt, strlen(btnBuf) > 0 ? btnBuf : "-",
      strlen(dpadBuf) > 0 ? dpadBuf : "-");
}

float XboxController::normalizeAxis(uint16_t raw, bool invert) {
  float norm = ((float)raw - 32767.5f) / 32767.5f;
  if (invert)
    norm = -norm;
  if (norm > 1.0f)
    norm = 1.0f;
  if (norm < -1.0f)
    norm = -1.0f;
  if (fabs(norm) < DEADZONE)
    norm = 0.0f;
  return norm;
}

float XboxController::normalizeTrigger(uint16_t raw) {
  float norm = (float)raw / 1023.0f;
  if (norm > 1.0f)
    norm = 1.0f;
  if (norm < 0.0f)
    norm = 0.0f;
  return norm;
}
