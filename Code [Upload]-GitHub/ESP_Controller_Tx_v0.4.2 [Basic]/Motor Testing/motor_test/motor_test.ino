/*
 * ============================================================
 *  URC MOTOR TEST — XIAO ESP32-C6 + TB6612FNG
 *  No radio needed — standalone motor sequence test
 *  Sequence: FORWARD → BACKWARD → LEFT → RIGHT → repeat
 * ============================================================
 *
 *  PINOUT (identical to RX v0.6 — no rewiring needed):
 *  ─────────────────────────────────────────────────────────
 *  D0  (GPIO0)  → AIN1   Left  motor dir A
 *  D1  (GPIO1)  → AIN2   Left  motor dir B
 *  D2  (GPIO2)  → PWMA   Left  motor speed
 *  D3  (GPIO3)  → BIN1   Right motor dir A
 *  D6  (GPIO6)  → BIN2   Right motor dir B
 *  D7  (GPIO7)  → PWMB   Right motor speed
 *  D10 (GPIO10) → STBY   HIGH = driver active
 *  D8  (GPIO8)  → Green LED (anode + 220Ω to GND)
 *  D9  (GPIO9)  → Red   LED (anode + 220Ω to GND)
 * ============================================================
 */

#include <Arduino.h>

// ── Pin definitions ─────────────────────────────────────────
#define PIN_AIN1   0    // Left  motor dir A
#define PIN_AIN2   1    // Left  motor dir B
#define PIN_PWMA   2    // Left  motor PWM

#define PIN_BIN1   3    // Right motor dir A
#define PIN_BIN2   8    // Right motor dir B
#define PIN_PWMB   9    // Right motor PWM

#define PIN_STBY   4   // TB6612 standby — HIGH = active

#define PIN_LED_G  6    // Green LED
#define PIN_LED_R  5    // Red   LED

// ── Config ──────────────────────────────────────────────────
#define PWM_FREQ   20000   // 20 kHz (silent)
#define PWM_RES    8       // 8-bit → 0–255
#define TEST_SPEED 220     // ~70% duty — safe for 3V N20 on 3.7V LiPo
                           // lower this (e.g. 120) if motors feel too hot // 220
#define STEP_MS    1000    // how long each direction runs (ms)
#define PAUSE_MS   300     // brief coast pause between steps

// ── Motor helper ────────────────────────────────────────────
//  speed: -255 to +255
//  positive = forward, negative = reverse, 0 = coast
void driveMotor(bool isLeft, int speed)
{
  int in1 = isLeft ? PIN_AIN1 : PIN_BIN1;
  int in2 = isLeft ? PIN_AIN2 : PIN_BIN2;
  int pwm = isLeft ? PIN_PWMA : PIN_PWMB;

  int duty = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    duty = 0;
  }
  ledcWrite(pwm, duty);
}

void stopMotors()
{
  driveMotor(true,  0);
  driveMotor(false, 0);
}

// ── Named movement functions ────────────────────────────────
//
//  Tank drive wiring convention (adjust sign if a motor is flipped):
//    Forward  : both motors positive
//    Backward : both motors negative
//    Left     : left motor reverse, right motor forward (spin left)
//    Right    : left motor forward, right motor reverse (spin right)

void moveForward()  { driveMotor(true,  TEST_SPEED); driveMotor(false,  TEST_SPEED); }
void moveBackward() { driveMotor(true, -TEST_SPEED); driveMotor(false, -TEST_SPEED); }
void turnLeft()     { driveMotor(true, -TEST_SPEED); driveMotor(false,  TEST_SPEED); }
void turnRight()    { driveMotor(true,  TEST_SPEED); driveMotor(false, -TEST_SPEED); }

// ── Setup ───────────────────────────────────────────────────
void setup()
{
  delay(500);
  Serial.begin(115200);
  Serial.println("\n=== URC MOTOR TEST ===");

  // Motor direction pins
  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);

  // Standby — keep LOW until ready
  pinMode(PIN_STBY, OUTPUT); 
 // digitalWrite(PIN_STBY, LOW);

  // LEDC PWM (Arduino Core 3.x API)
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);

  // LEDs
  pinMode(PIN_LED_G, OUTPUT); digitalWrite(PIN_LED_G, LOW);
  pinMode(PIN_LED_R, OUTPUT); digitalWrite(PIN_LED_R, HIGH); // red while init

  Serial.println("[INIT] Pins OK. Starting in 2 seconds...");
  delay(2000);   // give you time to place the robot on the floor

  // Enable TB6612 driver
  digitalWrite(PIN_STBY, HIGH);
  digitalWrite(PIN_LED_R, LOW);
  digitalWrite(PIN_LED_G, HIGH); // green = running
  Serial.println("[RUN]  TB6612 enabled — test loop starting\n");
}

// ── Loop ────────────────────────────────────────────────────
void loop()
{
  // ── FORWARD ──────────────────────────────────────────────
  Serial.println("[TEST] FORWARD");
  moveForward();
  //delay(STEP_MS);

  //stopMotors();
  //delay(PAUSE_MS);

  // ── BACKWARD ─────────────────────────────────────────────
  //Serial.println("[TEST] BACKWARD");
  //moveBackward();
  //delay(STEP_MS);

  //stopMotors();
  //delay(PAUSE_MS);

  // ── LEFT ─────────────────────────────────────────────────
 //Serial.println("[TEST] LEFT (spin)");
  //turnLeft();
  //delay(STEP_MS);

  //stopMotors();
  //delay(PAUSE_MS);

  // ── RIGHT ────────────────────────────────────────────────
  //Serial.println("[TEST] RIGHT (spin)");
  //turnRight();
  //delay(STEP_MS);

  //stopMotors();
  //delay(PAUSE_MS);

  //Serial.println("── cycle complete ──\n");
}
