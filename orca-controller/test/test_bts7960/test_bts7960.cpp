#include <Arduino.h>
#include <unity.h>
#include <BTS7960Motor.h>

// Pins based on docs/motor-control-architecture.md §3
#define LEFT_RPWM_PIN  18
#define LEFT_LPWM_PIN  19
#define LEFT_EN_PIN    5

#define RIGHT_RPWM_PIN 25
#define RIGHT_LPWM_PIN 26
#define RIGHT_EN_PIN   23

BTS7960Config normalConfig = {
  .pinRPWM = LEFT_RPWM_PIN,
  .pinLPWM = LEFT_LPWM_PIN,
  .pinEN = LEFT_EN_PIN,
  .inverted = false,
  .pwmFreq = 20000,
  .pwmResolution = 10
};

BTS7960Config invertedConfig = {
  .pinRPWM = RIGHT_RPWM_PIN,
  .pinLPWM = RIGHT_LPWM_PIN,
  .pinEN = RIGHT_EN_PIN,
  .inverted = true,
  .pwmFreq = 20000,
  .pwmResolution = 10
};

BTS7960Motor motorNormal(normalConfig);
BTS7960Motor motorInverted(invertedConfig);

void setUp(void) {
  // Reset states before each test if needed
}

void tearDown(void) {
  motorNormal.coast();
  motorInverted.coast();
}

void test_initialization_and_hardware_registers(void) {
  motorNormal.begin();

  TEST_ASSERT_TRUE(motorNormal.isEnabled());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, motorNormal.getSpeed());

  // Hardware LEDC registers must be zero on initial brake
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getLPWMChannel()));

  // Enable pin must be driven HIGH for active dynamic brake
  TEST_ASSERT_EQUAL(HIGH, digitalRead(LEFT_EN_PIN));
}

void test_forward_pwm_duty_cycle(void) {
  motorNormal.setSpeed(0.5f);
  delay(2);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, motorNormal.getSpeed());
  // 50% of 1023 = 512
  TEST_ASSERT_EQUAL_UINT32(512, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getLPWMChannel()));

  motorNormal.setSpeed(1.0f);
  delay(2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, motorNormal.getSpeed());
  // On ESP32 LEDC, 100% duty (max_duty = 1023) is latched as max_duty + 1 = 1024 for full ON
  TEST_ASSERT_EQUAL_UINT32(1024, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getLPWMChannel()));
}

void test_reverse_pwm_duty_cycle(void) {
  motorNormal.setSpeed(-0.25f);
  delay(2);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.25f, motorNormal.getSpeed());
  // Forward channel must be 0
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getRPWMChannel()));
  // 25% of 1023 = 256
  TEST_ASSERT_EQUAL_UINT32(256, ledcRead(motorNormal.getLPWMChannel()));

  motorNormal.setSpeed(-1.0f);
  delay(2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, motorNormal.getSpeed());
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(1024, ledcRead(motorNormal.getLPWMChannel()));
}

void test_clamping_out_of_bounds_inputs(void) {
  motorNormal.setSpeed(3.5f);
  delay(2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, motorNormal.getSpeed());
  TEST_ASSERT_EQUAL_UINT32(1024, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getLPWMChannel()));

  motorNormal.setSpeed(-4.0f);
  delay(2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, motorNormal.getSpeed());
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(1024, ledcRead(motorNormal.getLPWMChannel()));
}

void test_dynamic_brake(void) {
  motorNormal.setSpeed(0.8f);
  delay(2);
  TEST_ASSERT_TRUE(motorNormal.isEnabled());

  motorNormal.brake();
  delay(2);
  TEST_ASSERT_TRUE(motorNormal.isEnabled());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, motorNormal.getSpeed());

  // Both terminals shorted to GND via low-side FETs
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getLPWMChannel()));
  TEST_ASSERT_EQUAL(HIGH, digitalRead(LEFT_EN_PIN));
}

void test_coast_and_reenable(void) {
  motorNormal.setSpeed(0.6f);
  delay(2);
  motorNormal.coast();
  delay(2);

  TEST_ASSERT_FALSE(motorNormal.isEnabled());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, motorNormal.getSpeed());
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorNormal.getLPWMChannel()));
  // Enable pin pulled LOW for high impedance
  TEST_ASSERT_EQUAL(LOW, digitalRead(LEFT_EN_PIN));

  // Calling setSpeed must re-enable the driver
  motorNormal.setSpeed(0.4f);
  delay(2);
  TEST_ASSERT_TRUE(motorNormal.isEnabled());
  TEST_ASSERT_EQUAL(HIGH, digitalRead(LEFT_EN_PIN));
  TEST_ASSERT_EQUAL_UINT32(409, ledcRead(motorNormal.getRPWMChannel()));
}

void test_inverted_direction_logic(void) {
  motorInverted.begin();

  // Positive forward speed must route to LPWM when inverted = true
  motorInverted.setSpeed(0.5f);
  delay(2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, motorInverted.getSpeed());
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorInverted.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(512, ledcRead(motorInverted.getLPWMChannel()));

  // Negative reverse speed must route to RPWM when inverted = true
  motorInverted.setSpeed(-0.75f);
  delay(2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.75f, motorInverted.getSpeed());
  TEST_ASSERT_EQUAL_UINT32(767, ledcRead(motorInverted.getRPWMChannel()));
  TEST_ASSERT_EQUAL_UINT32(0, ledcRead(motorInverted.getLPWMChannel()));
}

void setup() {
  delay(2000); // Allow board to stabilize after serial connect
  UNITY_BEGIN();

  RUN_TEST(test_initialization_and_hardware_registers);
  RUN_TEST(test_forward_pwm_duty_cycle);
  RUN_TEST(test_reverse_pwm_duty_cycle);
  RUN_TEST(test_clamping_out_of_bounds_inputs);
  RUN_TEST(test_dynamic_brake);
  RUN_TEST(test_coast_and_reenable);
  RUN_TEST(test_inverted_direction_logic);

  UNITY_END();
}

void loop() {
  // No-op
}
