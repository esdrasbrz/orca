#include <Arduino.h>
#include <XboxController.h>

#define LED_PIN 2

XboxController controller;

unsigned long lastPrintTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=======================================================");
  Serial.println("         Orca Robot — Xbox BLE Telemetry Reader        ");
  Serial.println("=======================================================");
  Serial.println("[Orca] Initializing BLE stack...");
  controller.begin();
  Serial.println("[Orca] Scanning for Xbox Wireless Controller...");
  Serial.println("[Orca] >> Press and hold the Pair button on your Xbox");
  Serial.println("[Orca] >> controller until the Xbox logo flashes fast.");
  Serial.println("=======================================================\n");
}

void connected() {
  if (controller.isNewlyConnected()) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("\n=======================================================");
    Serial.println("[Orca] >>> Xbox Controller CONNECTED! <<<");
    Serial.printf("[Orca] Controller MAC Address: %s\n", controller.getAddress().c_str());
    Serial.println("[Orca] Normalized range: [-1.00, +1.00] (Y forward = +1.00)");
    Serial.println("=======================================================\n");
  }

  // Stream telemetry line at 10 Hz (every 100 ms)
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();
    controller.printTelemetry();
  }
}

void disconnected() {
  if (controller.isNewlyDisconnected()) {
    Serial.println("\n[Orca] Controller DISCONNECTED! Resuming scan...\n");
  }

  // Blink onboard LED while scanning
  if (millis() - lastBlinkTime >= 500) {
    lastBlinkTime = millis();
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  if (millis() - lastPrintTime >= 3000) {
    lastPrintTime = millis();
    Serial.println("[Orca] Still scanning for Xbox controller in pairing mode...");
  }
}

void loop() {
  controller.update();

  if (controller.isConnected()) {
    connected();
  } else {
    disconnected();
  }
}
