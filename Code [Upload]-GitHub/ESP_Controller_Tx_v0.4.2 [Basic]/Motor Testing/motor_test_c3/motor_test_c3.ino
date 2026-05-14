/* Working Test MOdel for ESP-C3 Mini + TB6612 + 2x N20 5v Motor
 * ============================================================
 *  MOTOR TEST — ESP32-C3 Mini + TB6612FNG + 2x N20 3V
 *  Sequence: FORWARD → BACKWARD → RIGHT → LEFT → repeat
 *
 *  WIRING (matches RX v0.6 pinout — no rewiring needed):
 *  ─────────────────────────────────────────────────────────
 *  GPIO 2  → AIN1   Left  motor dir A
 *  GPIO 3  → AIN2   Left  motor dir B
 *  GPIO 4  → PWMA   Left  motor speed
 *  GPIO 5  → BIN1   Right motor dir A
 *  GPIO 6  → BIN2   Right motor dir B
 *  GPIO 7  → PWMB   Right motor speed
 *  GPIO 8  → STBY   TB6612 enable/disable  // OR hardwire STBY to 3V3
 *  3V3     → VCC    TB6612 logic supply
 *  GND     → GND    common ground
 *  LiPo+   → VM     motor power rail
 * ============================================================
 */

#include <Arduino.h>

// ╔══════════════════════════════════════════════════════════╗
// ║                    PIN DEFINITIONS                      ║
// ╚══════════════════════════════════════════════════════════╝

#define PIN_AIN1 2  // Left  motor direction A
#define PIN_AIN2 3  // Left  motor direction B
#define PIN_PWMA 4  // Left  motor PWM speed

#define PIN_BIN1 5  // Right motor direction A
#define PIN_BIN2 6  // Right motor direction B
#define PIN_PWMB 7  // Right motor PWM speed

#define PIN_STBY 10  // TB6612 standby — HIGH=active
                     // If STBY is hardwired to 3V3, comment out
                     // all PIN_STBY lines below

// ╔══════════════════════════════════════════════════════════╗
// ║                ★ TWEAK THESE ★                          ║
// ╚══════════════════════════════════════════════════════════╝

// ── Speed ────────────────────────────────────────────────
//  Range: 0 (stop) to 255 (full speed)
//  For 3V N20 motors on a 3.7V LiPo, stay under 220
//  to avoid overheating. Start at 150 and go up slowly.
#define DRIVE_SPEED 255  // straight forward/backward speed
#define TURN_SPEED 160   // spin speed for left/right turns \
                         // lower than drive to avoid wheel slip

// ── Timing ───────────────────────────────────────────────
//  All values in milliseconds (1000ms = 1 second)
#define MOVE_TIME 1000  // how long each movement lasts
#define PAUSE_TIME 800  // coast pause between movements \
                        // increase if robot needs more settling time

// ── PWM ──────────────────────────────────────────────────
#define PWM_FREQ 20000  // 20kHz — above hearing range (silent)
#define PWM_RES 8       // 8-bit → duty 0–255, don't change this

// ╔══════════════════════════════════════════════════════════╗
// ║                    MOTOR DRIVER                         ║
// ╚══════════════════════════════════════════════════════════╝

/*
 * driveMotor(isLeft, speed)
 *   isLeft : true = left motor (channel A), false = right motor (channel B)
 *   speed  : -255 to +255
 *             positive = forward
 *             negative = reverse
 *             zero     = coast (both IN pins LOW)
 */
void driveMotor(bool isLeft, int speed) {
  int in1 = isLeft ? PIN_AIN1 : PIN_BIN1;
  int in2 = isLeft ? PIN_AIN2 : PIN_BIN2;
  int pwmP = isLeft ? PIN_PWMA : PIN_PWMB;

  int duty = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);  // coast — swap both HIGH for active brake
    digitalWrite(in2, LOW);
    duty = 0;
  }
  ledcWrite(pwmP, duty);
}

void stopMotors() {
  driveMotor(true, 0);
  driveMotor(false, 0);
}

// ╔══════════════════════════════════════════════════════════╗
// ║                   MOVEMENT FUNCTIONS                    ║
// ║                                                         ║
// ║  If a motor spins the wrong way, swap the sign on       ║
// ║  that motor in the function below. e.g. if forward      ║
// ║  makes the right motor go backward, change:             ║
// ║    driveMotor(false,  DRIVE_SPEED)                      ║
// ║  to:                                                    ║
// ║    driveMotor(false, -DRIVE_SPEED)                      ║
// ╚══════════════════════════════════════════════════════════╝

void moveForward() {
  driveMotor(true, DRIVE_SPEED);   // left  motor forward
  driveMotor(false, DRIVE_SPEED);  // right motor forward
}

void moveBackward() {
  driveMotor(true, -DRIVE_SPEED);   // left  motor reverse
  driveMotor(false, -DRIVE_SPEED);  // right motor reverse
}

void turnRight() {
  // Tank spin: left forward, right reverse
  driveMotor(true, -TURN_SPEED);
  driveMotor(false, TURN_SPEED);
}

void turnLeft() {
  // Tank spin: left reverse, right forward
  driveMotor(true, TURN_SPEED);
  driveMotor(false, -TURN_SPEED);
}

// ╔══════════════════════════════════════════════════════════╗
// ║                        SETUP                            ║
// ╚══════════════════════════════════════════════════════════╝

void setup() {
  delay(500);
  Serial.begin(115200);
  Serial.println("\n=== MOTOR TEST — ESP32-C3 Mini ===");

  // Direction pins — all LOW (safe state) before enabling driver
  pinMode(PIN_AIN1, OUTPUT);
  digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT);
  digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT);
  digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT);
  digitalWrite(PIN_BIN2, LOW);

  // STBY LOW until ready — keeps motors off during boot
  // Comment these two lines out if STBY is hardwired to 3V3
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);

  // LEDC PWM — ESP32 Arduino Core 3.x API
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);

  Serial.println("[INIT] Pins OK. Waiting 2 seconds before start...");
  Serial.println("[INFO] Place robot on floor now.");
  delay(2000);  // time to put robot down before it moves

  // Enable TB6612 driver
  // Comment this line out if STBY is hardwired to 3V3
  digitalWrite(PIN_STBY, HIGH);

  Serial.println("[RUN]  Starting test loop...\n");
}

// ╔══════════════════════════════════════════════════════════╗
// ║                         LOOP                            ║
// ╚══════════════════════════════════════════════════════════╝

void loop() {
  // ── FORWARD ──────────────────────────────────────────────
  Serial.println("[TEST] FORWARD");
  moveForward();
  delay(MOVE_TIME);

  stopMotors();
  delay(PAUSE_TIME);

  // ── BACKWARD ─────────────────────────────────────────────
  Serial.println("[TEST] BACKWARD");
  moveBackward();
  delay(MOVE_TIME);

  stopMotors();
  delay(PAUSE_TIME);

  // ── RIGHT ────────────────────────────────────────────────
  Serial.println("[TEST] RIGHT (spin)");
  turnRight();
  delay(MOVE_TIME);

  stopMotors();
  delay(PAUSE_TIME);

  // ── LEFT ─────────────────────────────────────────────────
  Serial.println("[TEST] LEFT (spin)");
  turnLeft();
  delay(MOVE_TIME);

  stopMotors();
  delay(PAUSE_TIME);

  Serial.println("── cycle complete ──\n");
}
