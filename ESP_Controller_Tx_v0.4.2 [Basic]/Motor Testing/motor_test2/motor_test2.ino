/*
 * ============================================================
 *  URC MOTOR TEST — XIAO ESP32-C6 + TB6612FNG
 *  Standalone motor sequence — no radio needed
 *  Sequence: FORWARD → BACKWARD → LEFT → RIGHT → repeat
 *
 *  STBY hardwired to 3V3 (no GPIO needed for standby)
 * ============================================================
 *
 *  PINOUT:
 *  D0  (GPIO0)  → AIN1   Left  motor dir A
 *  D1  (GPIO1)  → AIN2   Left  motor dir B
 *  D2  (GPIO2)  → PWMA   Left  motor speed
 *  D3  (GPIO3)  → BIN1   Right motor dir A
 *  D6  (GPIO6)  → BIN2   Right motor dir B
 *  D7  (GPIO7)  → PWMB   Right motor speed
 *  3V3          → STBY   (hardwired — always active)
 *  D8  (GPIO8)  → Green LED
 *  D9  (GPIO9)  → Red   LED
 * ============================================================
 */

#include <Arduino.h>

// ── Pin definitions ─────────────────────────────────────────
#define PIN_AIN1   0
#define PIN_AIN2   1
#define PIN_PWMA   2

#define PIN_BIN1   3
#define PIN_BIN2   6
#define PIN_PWMB   7

// PIN_STBY removed — wire STBY directly to 3V3 on the board

#define PIN_LED_G  8
#define PIN_LED_R  9

// ── Config ──────────────────────────────────────────────────
#define PWM_FREQ   20000
#define PWM_RES    8
#define TEST_SPEED 180     // ~70% duty — lower if motors run hot
#define STEP_MS    1000    // duration per direction (ms)
#define PAUSE_MS   300     // coast pause between steps (ms)

// ── Motor driver ────────────────────────────────────────────
void driveMotor(bool isLeft, int speed)
{
  int in1  = isLeft ? PIN_AIN1 : PIN_BIN1;
  int in2  = isLeft ? PIN_AIN2 : PIN_BIN2;
  int pwmP = isLeft ? PIN_PWMA : PIN_PWMB;

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
  ledcWrite(pwmP, duty);
}

void stopMotors()
{
  driveMotor(true,  0);
  driveMotor(false, 0);
}

// ── Movements ───────────────────────────────────────────────
void moveForward()  { driveMotor(true,  TEST_SPEED); driveMotor(false,  TEST_SPEED); }
void moveBackward() { driveMotor(true, -TEST_SPEED); driveMotor(false, -TEST_SPEED); }
void turnLeft()     { driveMotor(true, -TEST_SPEED); driveMotor(false,  TEST_SPEED); }
void turnRight()    { driveMotor(true,  TEST_SPEED); driveMotor(false, -TEST_SPEED); }

// ── Setup ────────────────────────────────────────────────────
void setup()
{
  delay(500);
  Serial.begin(115200);
  Serial.println("\n=== URC MOTOR TEST ===");
  Serial.println("[INFO] STBY hardwired to 3V3 — no GPIO standby control");

  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);

  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);

  pinMode(PIN_LED_G, OUTPUT); digitalWrite(PIN_LED_G, LOW);
  pinMode(PIN_LED_R, OUTPUT); digitalWrite(PIN_LED_R, HIGH);

  Serial.println("[INIT] Pins OK. Starting in 2 seconds...");
  delay(2000);

  digitalWrite(PIN_LED_R, LOW);
  digitalWrite(PIN_LED_G, HIGH);
  Serial.println("[RUN]  Test loop starting\n");
}

// ── Loop ─────────────────────────────────────────────────────
void loop()
{
  Serial.println("[TEST] FORWARD");
  moveForward();
  delay(STEP_MS);
  stopMotors();
  delay(PAUSE_MS);

  Serial.println("[TEST] BACKWARD");
  moveBackward();
  delay(STEP_MS);
  stopMotors();
  delay(PAUSE_MS);

  Serial.println("[TEST] LEFT");
  turnLeft();
  delay(STEP_MS);
  stopMotors();
  delay(PAUSE_MS);

  Serial.println("[TEST] RIGHT");
  turnRight();
  delay(STEP_MS);
  stopMotors();
  delay(PAUSE_MS);

  Serial.println("── cycle complete ──\n");
}
