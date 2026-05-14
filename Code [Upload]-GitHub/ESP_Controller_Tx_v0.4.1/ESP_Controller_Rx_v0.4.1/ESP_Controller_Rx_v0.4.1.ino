/*
 * ============================================================
 *  URC RX v0.5 — Combat Robot Driver
 *  Board  : ESP32-C3 Mini
 *  Core   : ESP32 Arduino Core 3.x
 *  Radio  : ESP-NOW receiver
 *  Drive  : TB6612FNG + 2x N20 3V DC motors (tank drive)
 *  Power  : 3.7V LiPo
 * ============================================================
 *
 *  ┌─────────────────────────────────────────────────────────┐
 *  │                   WIRING DIAGRAM                        │
 *  ├─────────────────────────────────────────────────────────┤
 *  │                                                         │
 *  │  ── TB6612FNG MOTOR DRIVER ────────────────────────     │
 *  │  AIN1  → GPIO 2   (left  motor direction bit A)        │
 *  │  AIN2  → GPIO 3   (left  motor direction bit B)        │
 *  │  PWMA  → GPIO 4   (left  motor speed — LEDC PWM)       │
 *  │  BIN1  → GPIO 5   (right motor direction bit A)        │
 *  │  BIN2  → GPIO 6   (right motor direction bit B)        │
 *  │  PWMB  → GPIO 7   (right motor speed — LEDC PWM)       │
 *  │  STBY  → GPIO 8   (HIGH=driver enabled, LOW=standby)   │
 *  │  VCC   → 3.3V     (logic supply)                       │
 *  │  GND   → GND                                           │
 *  │  VM    → LiPo+    (motor supply — 3.7V direct)         │
 *  │                                                         │
 *  │  ── MOTORS ────────────────────────────────────────     │
 *  │  Left  N20 → AO1, AO2 on TB6612FNG                     │
 *  │  Right N20 → BO1, BO2 on TB6612FNG                     │
 *  │  If a motor spins the wrong direction, swap its two     │
 *  │  output wires (AO1↔AO2 or BO1↔BO2)                     │
 *  │                                                         │
 *  │  ── STATUS LEDs ───────────────────────────────────     │
 *  │  Green LED anode → GPIO 9  → 220Ω resistor → GND      │
 *  │  Red   LED anode → GPIO 10 → 220Ω resistor → GND      │
 *  │                                                         │
 *  │  ── POWER ─────────────────────────────────────────     │
 *  │  LiPo+ → TB6612FNG VM  (motor rail)                    │
 *  │  LiPo+ → 3.3V regulator → ESP32-C3 3V3 pin            │
 *  │  LiPo- → GND (common ground with everything)           │
 *  │                                                         │
 *  │  ── NOTE ON N20 3V MOTORS AT 3.7V ─────────────────    │
 *  │  N20s will run slightly over their rated voltage.       │
 *  │  They handle it fine for combat use. Avoid running      │
 *  │  at 100% PWM continuously — use 85–90% max if you      │
 *  │  want to protect motor longevity.                       │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  ── FIRST TIME SETUP ──────────────────────────────────────
 *  1. Flash this file to the ESP32-C3 Mini
 *  2. Open Serial Monitor (115200 baud)
 *  3. Copy the MAC address printed on boot
 *  4. Paste it into RX_MAC[] in the TX file
 *  5. Flash the TX file to the 30-pin ESP32
 *
 *  ── TANK DRIVE MAPPING ────────────────────────────────────
 *  LY (left  stick Y) → left  motor speed & direction
 *  RY (right stick Y) → right motor speed & direction
 *  Stick forward (LY negative) → motor forward
 *  Stick back    (LY positive) → motor reverse
 *  Mixed LY/RY = turns, spins, differential steering
 *
 *  ── BUTTON HOOKUP POINTS ──────────────────────────────────
 *  data.LB = left  push button on TX
 *  data.RB = right push button on TX
 *  data.LS = left  stick click on TX
 *  data.RS = right stick click on TX
 *  All four are received every packet — wire + uncomment
 *  whichever action you want in the BUTTON HOOKUP section.
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ╔══════════════════════════════════════════════════════════╗
// ║              PIN DEFINITIONS — TB6612FNG                ║
// ╚══════════════════════════════════════════════════════════╝

// ── Left motor (TB6612FNG channel A) ───────────────────────
#define PIN_AIN1  2    // direction bit A
#define PIN_AIN2  3    // direction bit B
#define PIN_PWMA  4    // PWM speed — LEDC channel 0

// ── Right motor (TB6612FNG channel B) ──────────────────────
#define PIN_BIN1  5    // direction bit A
#define PIN_BIN2  6    // direction bit B
#define PIN_PWMB  7    // PWM speed — LEDC channel 1

// ── TB6612FNG standby ──────────────────────────────────────
#define PIN_STBY  8    // HIGH = driver active │ LOW = standby (coasting)

// ── Status LEDs ────────────────────────────────────────────
#define PIN_LED_G 9    // Green → link active
#define PIN_LED_R 10   // Red   → no link / waiting

// ╔══════════════════════════════════════════════════════════╗
// ║                     CONFIGURATION                       ║
// ╚══════════════════════════════════════════════════════════╝
#define ACK_TIMEOUT_MS  500    // ms without packet → link lost, motors stop

// PWM config — 20kHz puts switching noise above hearing range
// 8-bit resolution → 0–255 duty cycle values
#define PWM_FREQ   20000
#define PWM_RES    8

// Motor speed cap — protects N20 3V motors running on 3.7V
// 255 = full voltage, 217 ≈ 85% duty ≈ ~3.1V effective
// Set to 255 to remove the cap entirely
#define MAX_PWM    217

// ╔══════════════════════════════════════════════════════════╗
// ║                   CONTROLLER DATA STRUCT                ║
// ║   Must be byte-identical to TX — do not change          ║
// ╚══════════════════════════════════════════════════════════╝
typedef struct {
  int  LX, LY;   // Left  stick axes — range: -2047 .. +2047
  int  RX, RY;   // Right stick axes — range: -2047 .. +2047
  bool LS, RS;   // Stick clicks     — true = pressed
  bool LB, RB;   // Push buttons     — true = pressed
} ControllerData;

ControllerData data;

// ╔══════════════════════════════════════════════════════════╗
// ║               BUTTON OUTPUT PIN EXAMPLES                ║
// ║  Define output pins here when you decide what to wire.  ║
// ║  Uncomment the ones you use.                            ║
// ╚══════════════════════════════════════════════════════════╝
// #define PIN_LB_OUT  1    // e.g. LED strip, relay, beeper
// #define PIN_RB_OUT  0    // e.g. servo signal, LED, buzzer
// #define PIN_LS_OUT  20   // e.g. weapon trigger, light
// #define PIN_RS_OUT  21   // e.g. full-stop latch, alarm

// ╔══════════════════════════════════════════════════════════╗
// ║                       GLOBALS                           ║
// ╚══════════════════════════════════════════════════════════╝
volatile bool          newPacket = false;
volatile unsigned long lastRx    = 0;
bool                   connected = false;

// ╔══════════════════════════════════════════════════════════╗
// ║            ESP-NOW RECEIVE CALLBACK (Core 3.x)          ║
// ║                                                         ║
// ║  Core 2.x: const uint8_t *mac, const uint8_t *data     ║
// ║  Core 3.x: const esp_now_recv_info_t *info  ←          ║
// ╚══════════════════════════════════════════════════════════╝
void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *inData, int len) {
  if (len == sizeof(ControllerData)) {
    memcpy(&data, inData, sizeof(data));
    newPacket = true;
    lastRx    = millis();
  }
}

// ╔══════════════════════════════════════════════════════════╗
// ║                     MOTOR DRIVER                        ║
// ╚══════════════════════════════════════════════════════════╝

/*
 * driveMotor()
 * Sets direction and PWM speed for one motor channel.
 *
 * isLeft = true  → channel A (AIN1, AIN2, PWMA)
 * isLeft = false → channel B (BIN1, BIN2, PWMB)
 * speed  = -2047..+2047  (negative = reverse, 0 = coast)
 *
 * TB6612FNG truth table:
 *   IN1=H IN2=L → forward
 *   IN1=L IN2=H → reverse
 *   IN1=L IN2=L → coast  (both LOW)
 *   IN1=H IN2=H → brake  (both HIGH) ← swap coast lines to use
 */
void driveMotor(bool isLeft, int speed) {
  int ain1 = isLeft ? PIN_AIN1 : PIN_BIN1;
  int ain2 = isLeft ? PIN_AIN2 : PIN_BIN2;
  int pwmP = isLeft ? PIN_PWMA : PIN_PWMB;

  // Map |speed| 0–2047 → 0–MAX_PWM duty
  int pwm = map(abs(speed), 0, 2047, 0, MAX_PWM);
  pwm = constrain(pwm, 0, MAX_PWM);

  if (speed > 0) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else if (speed < 0) {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  } else {
    // Coast — change both to HIGH for active brake instead
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
    pwm = 0;
  }
  ledcWrite(pwmP, pwm);
}

/*
 * stopMotors()
 * Coasts both motors to a stop. Called on link loss.
 * Also sets STBY low via the watchdog — belt-and-suspenders.
 */
void stopMotors() {
  driveMotor(true,  0);
  driveMotor(false, 0);
}

// ╔══════════════════════════════════════════════════════════╗
// ║                         SETUP                           ║
// ╚══════════════════════════════════════════════════════════╝
void setup() {
  Serial.begin(115200);

  // ── Motor direction pins ──────────────────────────────────
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);    // driver disabled until link established

  // ── LEDC PWM — ESP32 Arduino Core 3.x API ─────────────────
  // Core 3.x: ledcAttach(pin, freq, resolution) — no channel numbers
  // Core 2.x: ledcSetup(ch, freq, res) + ledcAttachPin(pin, ch)
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);

  // ── Status LEDs ───────────────────────────────────────────
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);  // red on = waiting for link
  digitalWrite(PIN_LED_G, LOW);

  // ── Button output pins ────────────────────────────────────
  // Uncomment whichever you wire up
  // pinMode(PIN_LB_OUT, OUTPUT);
  // pinMode(PIN_RB_OUT, OUTPUT);
  // pinMode(PIN_LS_OUT, OUTPUT);
  // pinMode(PIN_RS_OUT, OUTPUT);

  // ── ESP-NOW init ──────────────────────────────────────────
  WiFi.mode(WIFI_STA);

  // Print MAC address — copy this into RX_MAC[] on the TX side
  Serial.print("RX MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    // Blink red rapidly to signal fault
    while (1) {
      digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R));
      delay(150);
    }
  }

  // Register receive callback — Core 3.x signature
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("RX ready — waiting for controller...");
}

// ╔══════════════════════════════════════════════════════════╗
// ║                          LOOP                           ║
// ╚══════════════════════════════════════════════════════════╝
void loop() {
  unsigned long now = millis();

  // ── Connection watchdog ──────────────────────────────────
  // If no packet arrives within ACK_TIMEOUT_MS → link lost.
  // Motors stop and STBY pulls the driver into standby.
  bool wasConnected = connected;
  connected = (now - lastRx < ACK_TIMEOUT_MS);

  if (connected != wasConnected) {
    digitalWrite(PIN_LED_G, connected ? HIGH : LOW);
    digitalWrite(PIN_LED_R, connected ? LOW  : HIGH);
    digitalWrite(PIN_STBY,  connected ? HIGH : LOW);
    if (!connected) stopMotors();
    Serial.println(connected ? "Link UP" : "Link LOST — motors stopped");
  }

  // ── Process incoming packet ──────────────────────────────
  if (newPacket && connected) {
    newPacket = false;

    // ── Tank drive ────────────────────────────────────────
    // LY → left motor │ RY → right motor
    // Negated: stick pushed forward (negative LY) → forward motion
    // Swap the negate sign if a motor runs backwards
    driveMotor(true,  -data.LY);   // left  motor
    driveMotor(false, -data.RY);   // right motor

    // ╔══════════════════════════════════════════════════════╗
    // ║              BUTTON HOOKUP POINTS                   ║
    // ║                                                     ║
    // ║  Four signals arrive every packet:                  ║
    // ║    data.LB  — left  push button on TX               ║
    // ║    data.RB  — right push button on TX               ║
    // ║    data.LS  — left  stick click on TX               ║
    // ║    data.RS  — right stick click on TX               ║
    // ║                                                     ║
    // ║  Wire your output to a free GPIO on the C3 mini,    ║
    // ║  define its PIN above, then uncomment the block.    ║
    // ╚══════════════════════════════════════════════════════╝

    // ── LB: e.g. toggle relay, LED strip, beeper ──────────
    // digitalWrite(PIN_LB_OUT, data.LB ? HIGH : LOW);

    // ── RB: e.g. trigger buzzer, fire a servo pulse ───────
    // digitalWrite(PIN_RB_OUT, data.RB ? HIGH : LOW);

    // ── LS (left stick click): e.g. spin in place ─────────
    // Overrides tank drive while held — rotate on the spot
    // if (data.LS) {
    //   driveMotor(true,   900);   // left  forward
    //   driveMotor(false, -900);   // right reverse → CW spin
    // }

    // ── RS (right stick click): e.g. emergency full stop ──
    // if (data.RS) {
    //   driveMotor(true,  0);
    //   driveMotor(false, 0);
    // }

    // ── Combined example: LB+RB together = turbo mode ─────
    // if (data.LB && data.RB) {
    //   // temporarily override MAX_PWM cap — use with caution
    //   ledcWrite(PIN_PWMA, 255);
    //   ledcWrite(PIN_PWMB, 255);
    // }
  }
}
