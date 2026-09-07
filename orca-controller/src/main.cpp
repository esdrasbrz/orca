#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// On-board LED for visual connection feedback (GPIO 2 on most ESP32 DevKits)
#define LED_PIN 2

// Deadzone threshold around joystick center (ignore slight mechanical drift)
#define JOYSTICK_DEADZONE 0.05f

// Create the controller instance.
// Leave parameter empty to auto-scan and connect to any nearby Xbox controller in pairing mode.
// Once you know your controller's MAC address, you can specify it for instant connection:
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("xx:xx:xx:xx:xx:xx");
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

bool wasConnected = false;
unsigned long lastPrintTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

/**
 * Normalizes raw 16-bit joystick input (0..65535, center ~32768)
 * to a standardized floating-point range [-1.0, 1.0].
 *
 * @param raw Raw 16-bit unsigned value from controller parser.
 * @param invert Set true if the axis should be flipped (standard for Y-axis forward = positive).
 * @param deadzone Deadband around zero to prevent drift.
 */
float normalizeAxis(uint16_t raw, bool invert = false, float deadzone = JOYSTICK_DEADZONE) {
  // Center point is 32767.5 (halfway of 0xFFFF)
  float norm = ((float)raw - 32767.5f) / 32767.5f;

  if (invert) {
    norm = -norm;
  }

  // Clamp within [-1.0, 1.0]
  if (norm > 1.0f) norm = 1.0f;
  if (norm < -1.0f) norm = -1.0f;

  // Apply deadband
  if (fabs(norm) < deadzone) {
    norm = 0.0f;
  }

  return norm;
}

/**
 * Normalizes raw trigger input (0..1023) to [0.0, 1.0].
 */
float normalizeTrigger(uint16_t raw) {
  float norm = (float)raw / (float)XboxControllerNotificationParser::maxTrig;
  if (norm > 1.0f) norm = 1.0f;
  if (norm < 0.0f) norm = 0.0f;
  return norm;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=======================================================");
  Serial.println("         Orca Robot — Xbox BLE Telemetry Reader        ");
  Serial.println("=======================================================");
  Serial.println("[Orca] Initializing BLE stack...");
  xboxController.begin();
  Serial.println("[Orca] Scanning for Xbox Wireless Controller...");
  Serial.println("[Orca] >> Press and hold the Pair button on your Xbox");
  Serial.println("[Orca] >> controller until the Xbox logo flashes fast.");
  Serial.println("=======================================================\n");
}

void loop() {
  // Mandatory: drives the NimBLE connection state machine
  xboxController.onLoop();

  if (xboxController.isConnected()) {
    // 1. Connection Event Handling
    if (!wasConnected) {
      wasConnected = true;
      digitalWrite(LED_PIN, HIGH); // Solid ON when connected

      String macAddr = xboxController.buildDeviceAddressStr();
      Serial.println("\n=======================================================");
      Serial.println("[Orca] >>> Xbox Controller CONNECTED! <<<");
      Serial.printf("[Orca] Controller MAC Address: %s\n", macAddr.c_str());
      Serial.println("[Orca] Normalized range: [-1.00, +1.00]");
      Serial.println("[Orca] (Left Stick Y inverted: Forward = +1.00, Back = -1.00)");
      Serial.println("=======================================================\n");
    }

    if (xboxController.isWaitingForFirstNotification()) {
      if (millis() - lastPrintTime >= 500) {
        lastPrintTime = millis();
        Serial.println("[Orca] Negotiating HID services and awaiting initial report...");
      }
    } else {
      // 2. Telemetry Streaming (Throttled to 10 Hz / 100 ms)
      if (millis() - lastPrintTime >= 100) {
        lastPrintTime = millis();

        // Convert raw axes to normalized [-1.0, 1.0] range
        // Note: For Y axes, 0 is fully up/forward on Xbox controllers,
        // so invert = true ensures forward stick produces positive (+1.0) velocity.
        float normLX = normalizeAxis(xboxController.xboxNotif.joyLHori, false);
        float normLY = normalizeAxis(xboxController.xboxNotif.joyLVert, true);
        float normRX = normalizeAxis(xboxController.xboxNotif.joyRHori, false);
        float normRY = normalizeAxis(xboxController.xboxNotif.joyRVert, true);
        float normLT = normalizeTrigger(xboxController.xboxNotif.trigLT);
        float normRT = normalizeTrigger(xboxController.xboxNotif.trigRT);

        // Build button status string
        char btnBuf[40] = "";
        if (xboxController.xboxNotif.btnA) strcat(btnBuf, "A ");
        if (xboxController.xboxNotif.btnB) strcat(btnBuf, "B ");
        if (xboxController.xboxNotif.btnX) strcat(btnBuf, "X ");
        if (xboxController.xboxNotif.btnY) strcat(btnBuf, "Y ");
        if (xboxController.xboxNotif.btnLB) strcat(btnBuf, "LB ");
        if (xboxController.xboxNotif.btnRB) strcat(btnBuf, "RB ");
        if (xboxController.xboxNotif.btnLS) strcat(btnBuf, "LS ");
        if (xboxController.xboxNotif.btnRS) strcat(btnBuf, "RS ");
        if (xboxController.xboxNotif.btnStart) strcat(btnBuf, "START ");
        if (xboxController.xboxNotif.btnSelect) strcat(btnBuf, "SELECT ");

        // Build D-Pad status string
        char dpadBuf[20] = "";
        if (xboxController.xboxNotif.btnDirUp) strcat(dpadBuf, "UP ");
        if (xboxController.xboxNotif.btnDirDown) strcat(dpadBuf, "DOWN ");
        if (xboxController.xboxNotif.btnDirLeft) strcat(dpadBuf, "LEFT ");
        if (xboxController.xboxNotif.btnDirRight) strcat(dpadBuf, "RIGHT ");

        // Print structured telemetry line
        Serial.printf("[Axes] LX:%+5.2f LY:%+5.2f | RX:%+5.2f RY:%+5.2f | LT:%4.2f RT:%4.2f | Buttons: [%s] D-Pad: [%s]\n",
                      normLX, normLY, normRX, normRY, normLT, normRT,
                      strlen(btnBuf) > 0 ? btnBuf : "-",
                      strlen(dpadBuf) > 0 ? dpadBuf : "-");
      }
    }
  } else {
    // 3. Disconnection / Scanning State Handling
    if (wasConnected) {
      wasConnected = false;
      Serial.println("\n[Orca] Controller DISCONNECTED! Resuming scan...\n");
    }

    // Blink LED slowly while searching
    if (millis() - lastBlinkTime >= 500) {
      lastBlinkTime = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }

    // Periodic heartbeat to terminal
    if (millis() - lastPrintTime >= 3000) {
      lastPrintTime = millis();
      Serial.println("[Orca] Still scanning for Xbox controller in pairing mode...");
    }
  }
}
