#include <Arduino.h>
#include <BTS7960Motor.h>
#include <DifferentialDrive.h>
#include <XboxController.h>

#define LED_PIN 2

// Motor pin allocation based on docs/motor-control-architecture.md §3
#define LEFT_RPWM_PIN 18
#define LEFT_LPWM_PIN 19
#define LEFT_EN_PIN 5

#define RIGHT_RPWM_PIN 25
#define RIGHT_LPWM_PIN 26
#define RIGHT_EN_PIN 23

// Actuator configurations
static BTS7960Config leftMotorConfig = {.pinRPWM = LEFT_RPWM_PIN,
                                        .pinLPWM = LEFT_LPWM_PIN,
                                        .pinEN = LEFT_EN_PIN,
                                        .inverted = false,
                                        .pwmFreq = 20000,
                                        .pwmResolution = 10};

static BTS7960Config rightMotorConfig = {
    .pinRPWM = RIGHT_RPWM_PIN,
    .pinLPWM = RIGHT_LPWM_PIN,
    .pinEN = RIGHT_EN_PIN,
    .inverted = true,  // Right motor mounted 180-deg mirrored on chassis
    .pwmFreq = 20000,
    .pwmResolution = 10};

// Drivetrain kinematics & safety configuration
static DrivetrainConfig drivetrainConfig = {
    .trackWidthMeters = 0.20f,
    .maxLinearVelocity = 1.0f,
    .maxAngularVelocity = 5.0f,
    .maxSlewRate = 2.5f,       // Smooth ramp to protect 5A fuse & BMS
    .watchdogTimeoutMs = 250,  // Deadman brake if no BLE packet within 250ms
    .enableHeadingLock = false,
    .headingKp = 0.0f,
    .headingKi = 0.0f,
    .headingKd = 0.0f};

// Hardware and controller instances
BTS7960Motor leftMotor(leftMotorConfig);
BTS7960Motor rightMotor(rightMotorConfig);
DifferentialDrive drivetrain(leftMotor, rightMotor, drivetrainConfig);
XboxController controller;

TaskHandle_t controlTaskHandle = NULL;
unsigned long lastPrintTime = 0;
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Deterministic 100 Hz control loop running on Core 0
void controlLoopTask(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100 Hz

  for (;;) {
    // Non-blocking: updates watchdog, slew-rate ramping, and motor PWM registers
    drivetrain.update();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("\n=======================================================");
  Serial.println("         Orca Robot — Phase 1A Open-Loop Teleop        ");
  Serial.println("=======================================================");

  // Initialize drivetrain actuators
  drivetrain.begin();
  Serial.println("[Orca] Dual BTS7960 motors initialized (20 kHz PWM).");
  Serial.println("[Orca] DifferentialDrive initialized with 250ms watchdog.");

  // Pin deterministic control loop to Core 0
  xTaskCreatePinnedToCore(controlLoopTask, "ControlLoop", 4096, NULL, configMAX_PRIORITIES - 1,
                          &controlTaskHandle,
                          0  // Core 0
  );
  Serial.println("[Orca] Core 0 control task started (100 Hz deterministic).");

  // Initialize BLE teleoperation on Core 1
  Serial.println("[Orca] Initializing BLE stack on Core 1...");
  controller.begin();
  Serial.println("[Orca] Scanning for Xbox Wireless Controller...");
  Serial.println("[Orca] >> Press and hold Pair on your Xbox controller.");
  Serial.println("=======================================================\n");
}

void connected() {
  if (controller.isNewlyConnected()) {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("\n=======================================================");
    Serial.println("[Orca] >>> Xbox Controller CONNECTED! <<<");
    Serial.printf("[Orca] Controller MAC Address: %s\n", controller.getAddress().c_str());
    Serial.println("[Orca] Controls:");
    Serial.println("[Orca]   • Right Trigger (RT): Forward Throttle [0.00 to 1.00]");
    Serial.println("[Orca]   • Left Trigger (LT):  Brake / Reverse  [0.00 to 1.00]");
    Serial.println("[Orca]   • Left Stick X (LX):  Steer / Drive    [-1.00 to +1.00]");
    Serial.println("=======================================================\n");
  }

  // Map controls per user specification:
  // RT = forward throttle, LT = reverse/brake, LX = steering
  float throttle = controller.rightTrigger() - controller.leftTrigger();
  float turn = controller.leftX();

  // Feed teleop input to drivetrain (feeds deadman watchdog heartbeat)
  drivetrain.driveArcade(throttle, turn);

  // Stream telemetry at 10 Hz
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();
    Serial.printf(
        "[Drive] Throttle:%+5.2f Turn:%+5.2f | RampedLin:%+5.2f RampedAng:%+5.2f | WD:%s\n",
        throttle, turn, drivetrain.getRampedLinear(), drivetrain.getRampedAngular(),
        drivetrain.isWatchdogTriggered() ? "TRIGGERED" : "OK");
  }
}

void disconnected() {
  if (controller.isNewlyDisconnected()) {
    Serial.println("\n[Orca] Controller DISCONNECTED! Triggering emergency stop...\n");
  }

  // Failsafe: command immediate active brake
  drivetrain.emergencyStop();

  // Blink onboard LED while scanning
  if (millis() - lastBlinkTime >= 500) {
    lastBlinkTime = millis();
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  if (millis() - lastPrintTime >= 3000) {
    lastPrintTime = millis();
    Serial.println("[Orca] Scanning for Xbox controller in pairing mode...");
  }
}

void loop() {
  // Core 1 loop handles BLE notifications and teleoperation dispatch
  controller.update();

  if (controller.isConnected()) {
    connected();
  } else {
    disconnected();
  }
}
