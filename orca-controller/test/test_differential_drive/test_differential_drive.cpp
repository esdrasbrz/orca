#include <Arduino.h>
#include <BTS7960Motor.h>
#include <DifferentialDrive.h>
#include <unity.h>

// Pins based on docs/motor-control-architecture.md §3
#define LEFT_RPWM_PIN 18
#define LEFT_LPWM_PIN 19
#define LEFT_EN_PIN 5

#define RIGHT_RPWM_PIN 25
#define RIGHT_LPWM_PIN 26
#define RIGHT_EN_PIN 23

static BTS7960Config leftConfig = {.pinRPWM = LEFT_RPWM_PIN,
                                   .pinLPWM = LEFT_LPWM_PIN,
                                   .pinEN = LEFT_EN_PIN,
                                   .inverted = false,
                                   .pwmFreq = 20000,
                                   .pwmResolution = 10};

static BTS7960Config rightConfig = {.pinRPWM = RIGHT_RPWM_PIN,
                                    .pinLPWM = RIGHT_LPWM_PIN,
                                    .pinEN = RIGHT_EN_PIN,
                                    .inverted = true,
                                    .pwmFreq = 20000,
                                    .pwmResolution = 10};

static DrivetrainConfig dtConfig = {.trackWidthMeters = 0.20f,
                                    .maxLinearVelocity = 1.0f,
                                    .maxAngularVelocity = 5.0f,
                                    .maxSlewRate = 2.0f,       // 2.0 units/sec^2
                                    .watchdogTimeoutMs = 250,  // 250 ms failsafe
                                    .enableHeadingLock = false,
                                    .headingKp = 0.0f,
                                    .headingKi = 0.0f,
                                    .headingKd = 0.0f};

static BTS7960Motor leftMotor(leftConfig);
static BTS7960Motor rightMotor(rightConfig);
static DifferentialDrive drivetrain(leftMotor, rightMotor, dtConfig);

void setUp(void) {
  drivetrain.begin();
}

void tearDown(void) {
  drivetrain.coast();
}

void test_initialization_state(void) {
  TEST_ASSERT_FALSE(drivetrain.isWatchdogTriggered());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drivetrain.getRampedLinear());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drivetrain.getRampedAngular());

  // Hardware LEDC registers must be zero
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(leftMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(leftMotor.getLPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(rightMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(rightMotor.getLPWMChannel()));

  // Active brake ensures enable pins are driven HIGH
  TEST_ASSERT_EQUAL(HIGH, digitalRead(LEFT_EN_PIN));
  TEST_ASSERT_EQUAL(HIGH, digitalRead(RIGHT_EN_PIN));
}

void test_slew_rate_limiter(void) {
  // Command full forward throttle
  drivetrain.driveArcade(1.0f, 0.0f);

  // First step with small dt (e.g. 20 ms -> 0.02s)
  delay(20);
  drivetrain.update();

  // Max ramp in 20 ms at 2.0 units/s^2 is ~0.04 units (+ a small tolerance for delay jitter)
  float ramped = drivetrain.getRampedLinear();
  TEST_ASSERT_TRUE_MESSAGE(ramped < 0.20f, "Slew rate limiter failed: jump was too large");
  TEST_ASSERT_TRUE_MESSAGE(ramped > 0.01f, "Slew rate limiter failed: no acceleration");

  // Run update loop over ~550 ms to reach 1.0f
  for (int i = 0; i < 30; i++) {
    delay(20);
    drivetrain.driveArcade(1.0f, 0.0f);  // keep heartbeat alive
    drivetrain.update();
  }

  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, drivetrain.getRampedLinear());
  delay(2);
  // Full forward on Left (normal): RPWM saturates (1024)
  TEST_ASSERT_EQUAL_UINT32(1024, ledcRead(leftMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(leftMotor.getLPWMChannel()));
  // Full forward on Right (inverted): LPWM saturates (1024)
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(rightMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(1024, ledcRead(rightMotor.getLPWMChannel()));
}

void test_deadman_watchdog_timeout(void) {
  // Drive briefly
  drivetrain.driveArcade(0.8f, 0.0f);
  for (int i = 0; i < 15; i++) {
    delay(20);
    drivetrain.driveArcade(0.8f, 0.0f);
    drivetrain.update();
  }
  TEST_ASSERT_FALSE(drivetrain.isWatchdogTriggered());

  // Stop sending commands and wait longer than watchdogTimeoutMs (250 ms)
  delay(270);
  drivetrain.update();

  // Watchdog should now be triggered and motors actively braked
  TEST_ASSERT_TRUE(drivetrain.isWatchdogTriggered());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drivetrain.getRampedLinear());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drivetrain.getRampedAngular());

  delay(2);
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(leftMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(leftMotor.getLPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(rightMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(rightMotor.getLPWMChannel()));
}

void test_emergency_stop_and_coast(void) {
  drivetrain.driveArcade(0.5f, 0.0f);
  delay(50);
  drivetrain.update();

  // Emergency stop must immediately brake both motors
  drivetrain.emergencyStop();
  delay(2);
  TEST_ASSERT_TRUE(drivetrain.isWatchdogTriggered());
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(leftMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(leftMotor.getLPWMChannel()));
  TEST_ASSERT_EQUAL(HIGH, digitalRead(LEFT_EN_PIN));
  TEST_ASSERT_EQUAL(HIGH, digitalRead(RIGHT_EN_PIN));

  // Coast must put drivers in high impedance
  drivetrain.coast();
  delay(2);
  TEST_ASSERT_EQUAL(LOW, digitalRead(LEFT_EN_PIN));
  TEST_ASSERT_EQUAL(LOW, digitalRead(RIGHT_EN_PIN));
}

void test_arcade_kinematics_and_curvature_desaturation(void) {
  // Command simultaneous full throttle (+1.0) and full turn (+1.0 steer right)
  // Without desaturation:
  // vLeft  = 1.0 - (-5.0 * 0.1) = 1.5
  // vRight = 1.0 + (-5.0 * 0.1) = 0.5
  // Max magnitude = 1.5 > 1.0 -> scaled down by 1.5:
  // dutyLeft = 1.5 / 1.5 = 1.0
  // dutyRight = 0.5 / 1.5 = 0.333
  // This verifies that curvature (ratio 3:1) is preserved and output is clamped to 1.0
  drivetrain.driveArcade(1.0f, 1.0f);

  // Allow slew limiter to ramp up
  for (int i = 0; i < 40; i++) {
    delay(20);
    drivetrain.driveArcade(1.0f, 1.0f);
    drivetrain.update();
  }

  // Left motor should be full forward (+1.0f -> 1024 RPWM)
  // Right motor should be partial forward (+0.333f -> ~341 LPWM because inverted)
  delay(2);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, leftMotor.getSpeed());
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.333f, rightMotor.getSpeed());
  TEST_ASSERT_EQUAL_UINT32(1024, ledcRead(leftMotor.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(rightMotor.getRPWMChannel()));
  TEST_ASSERT_UINT32_WITHIN(30, 341, ledcRead(rightMotor.getLPWMChannel()));
}

void setup() {
  delay(2000);
  UNITY_BEGIN();

  RUN_TEST(test_initialization_state);
  RUN_TEST(test_slew_rate_limiter);
  RUN_TEST(test_deadman_watchdog_timeout);
  RUN_TEST(test_emergency_stop_and_coast);
  RUN_TEST(test_arcade_kinematics_and_curvature_desaturation);

  UNITY_END();
}

void loop() {
  // Unity on ESP32 does not loop
}
