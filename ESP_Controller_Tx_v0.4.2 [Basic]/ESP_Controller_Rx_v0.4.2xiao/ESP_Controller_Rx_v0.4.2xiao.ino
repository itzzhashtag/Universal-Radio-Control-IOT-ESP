/*
XIAO ESP32-C6          TB6612FNG
─────────────          ─────────
D0  (GPIO0)  ────────→ AIN1
D1  (GPIO1)  ────────→ AIN2
D2  (GPIO2)  ────────→ PWMA
D3  (GPIO3)  ────────→ BIN1
D6  (GPIO6)  ────────→ BIN2
D7  (GPIO7)  ────────→ PWMB
D10 (GPIO10) ────────→ STBY
3V3          ────────→ VCC
GND          ────────→ GND

LiPo+        ────────→ VM  (motor power rail)
LiPo-        ────────→ GND

AO1, AO2     ────────→ Left  N20 motor
BO1, BO2     ────────→ Right N20 motor

Status LEDs (with 220Ω to GND):
D8 (GPIO8)  → Green LED anode
D9 (GPIO9)  → Red   LED anode
*/
/*
 * ============================================================
 *  URC RX v0.6 — Combat Robot Driver
 *  Board  : Seeed Studio XIAO ESP32-C6
 *  Pins   : D0-D10 (GPIO0-GPIO10)
 *  Core   : ESP32 Arduino Core 3.x
 *  Radio  : ESP-NOW receiver
 *  Drive  : TB6612FNG + 2x N20 3V DC motors (tank drive)
 *  Power  : 3.7V LiPo
 * ============================================================
 *
 *  WIRING SUMMARY
 *  ──────────────────────────────────────────────────────────
 *  XIAO D0  (GPIO0)  → TB6612 AIN1   left  motor dir A
 *  XIAO D1  (GPIO1)  → TB6612 AIN2   left  motor dir B
 *  XIAO D2  (GPIO2)  → TB6612 PWMA   left  motor speed
 *  XIAO D3  (GPIO3)  → TB6612 BIN1   right motor dir A
 *  XIAO D6  (GPIO6)  → TB6612 BIN2   right motor dir B
 *  XIAO D7  (GPIO7)  → TB6612 PWMB   right motor speed
 *  XIAO D10 (GPIO10) → TB6612 STBY   HIGH=active LOW=standby
 *  XIAO 3V3          → TB6612 VCC    logic supply
 *  XIAO GND          → TB6612 GND    common ground
 *  LiPo+             → TB6612 VM     motor power
 *
 *  TB6612 AO1,AO2    → Left  N20 motor terminals
 *  TB6612 BO1,BO2    → Right N20 motor terminals
 *  (wrong direction? swap the two wires on that motor)
 *
 *  Green LED + 220Ω  → D8 (GPIO8)  → GND   link OK
 *  Red   LED + 220Ω  → D9 (GPIO9)  → GND   no link
 *
 *  ── FIRST TIME SETUP ──────────────────────────────────────
 *  1. Flash this to XIAO ESP32-C6
 *  2. Open Serial Monitor at 115200 baud
 *  3. Copy MAC printed at boot
 *  4. Paste into RX_MAC[] in your TX file and reflash TX
 *  5. Link LED goes green within ~1 second
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ╔══════════════════════════════════════════════════════════╗
// ║       PIN DEFINITIONS — XIAO ESP32-C6 (GPIO 0-10)      ║
// ╚══════════════════════════════════════════════════════════╝

// ── Left motor — TB6612 channel A ──────────────────────────
#define PIN_AIN1   0    // D0  — direction bit A
#define PIN_AIN2   1    // D1  — direction bit B
#define PIN_PWMA   2    // D2  — PWM speed (LEDC)

// ── Right motor — TB6612 channel B ─────────────────────────
#define PIN_BIN1   3    // D3  — direction bit A
#define PIN_BIN2   6    // D6  — direction bit B
#define PIN_PWMB   7    // D7  — PWM speed (LEDC)

// ── TB6612 standby ──────────────────────────────────────────
#define PIN_STBY   10   // D10 — HIGH=driver active, LOW=coast

// ── Status LEDs ─────────────────────────────────────────────
#define PIN_LED_G  8    // D8  — green (link OK)
#define PIN_LED_R  9    // D9  — red   (no link / waiting)

// ── Button output pins — define when you wire them ──────────
// #define PIN_LB_OUT  4   // D4 — e.g. LED strip, relay
// #define PIN_RB_OUT  5   // D5 — e.g. buzzer, weapon

// ╔══════════════════════════════════════════════════════════╗
// ║                     CONFIGURATION                       ║
// ╚══════════════════════════════════════════════════════════╝

#define ACK_TIMEOUT_MS        500   // ms without packet → link lost
#define PWM_FREQ            20000   // 20kHz — above hearing range
#define PWM_RES                 8   // 8-bit → duty 0–255
#define MAX_PWM               217   // 85% cap for 3V N20s at 3.7V
                                    // set to 255 to remove cap
#define SERIAL_PRINT_INTERVAL 500   // ms between serial snapshots

// ╔══════════════════════════════════════════════════════════╗
// ║              CONTROLLER DATA STRUCT                     ║
// ║  Must be byte-identical to TX — do not reorder fields   ║
// ╚══════════════════════════════════════════════════════════╝

typedef struct {
  int  LX, LY;   // Left  stick  -2047..+2047
  int  RX, RY;   // Right stick  -2047..+2047
  bool LS, RS;   // Stick clicks  true=pressed
  bool LB, RB;   // Push buttons  true=pressed
} ControllerData;

ControllerData data;

// ╔══════════════════════════════════════════════════════════╗
// ║                        GLOBALS                          ║
// ╚══════════════════════════════════════════════════════════╝

volatile bool          newPacket       = false;
volatile unsigned long lastRx          = 0;
bool                   connected       = false;
unsigned long          lastSerialPrint = 0;
unsigned long          packetCount     = 0;

bool prevLB = false, prevRB = false;
bool prevLS = false, prevRS = false;

// ╔══════════════════════════════════════════════════════════╗
// ║         ESP-NOW RECEIVE CALLBACK — Core 3.x             ║
// ╚══════════════════════════════════════════════════════════╝

void onDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *inData, int len)
{
  if (len == sizeof(ControllerData)) {
    memcpy(&data, inData, sizeof(data));
    newPacket = true;
    lastRx    = millis();
    packetCount++;
  } else {
    Serial.printf("[ESP-NOW] Size mismatch: got %d bytes, expected %d\n",
                  len, sizeof(ControllerData));
    Serial.println("[ESP-NOW] ControllerData struct must match TX exactly");
  }
}

// ╔══════════════════════════════════════════════════════════╗
// ║                     MOTOR DRIVER                        ║
// ╚══════════════════════════════════════════════════════════╝

/*
 * driveMotor()
 * isLeft = true  → channel A (AIN1/AIN2/PWMA) — left  motor
 * isLeft = false → channel B (BIN1/BIN2/PWMB) — right motor
 * speed = -2047..+2047
 *   positive → forward  (IN1=H IN2=L)
 *   negative → reverse  (IN1=L IN2=H)
 *   zero     → coast    (IN1=L IN2=L, PWM=0)
 */
void driveMotor(bool isLeft, int speed)
{
  int in1  = isLeft ? PIN_AIN1 : PIN_BIN1;
  int in2  = isLeft ? PIN_AIN2 : PIN_BIN2;
  int pwmP = isLeft ? PIN_PWMA : PIN_PWMB;

  int pwm = map(abs(speed), 0, 2047, 0, MAX_PWM);
  pwm = constrain(pwm, 0, MAX_PWM);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwm = 0;
  }
  ledcWrite(pwmP, pwm);
}

void stopMotors()
{
  driveMotor(true,  0);
  driveMotor(false, 0);
  Serial.println("[MOTOR] Both motors stopped (coast)");
}

// ╔══════════════════════════════════════════════════════════╗
// ║                  SERIAL SNAPSHOT                        ║
// ╚══════════════════════════════════════════════════════════╝

void printSerialData()
{
  // Scale raw ±2047 → ±99 for display (matches TX output format)
  auto scl    = [](int v) -> int { return v * 99 / 2047; };
  auto motDir = [](int val) -> const char* {
    if (val < -300) return "FWD";
    if (val >  300) return "REV";
    return "STP";
  };

  int leftPWM  = (abs(data.LY) > 300)
                   ? constrain(map(abs(data.LY), 0, 2047, 0, MAX_PWM), 0, MAX_PWM)
                   : 0;
  int rightPWM = (abs(data.RY) > 300)
                   ? constrain(map(abs(data.RY), 0, 2047, 0, MAX_PWM), 0, MAX_PWM)
                   : 0;

  Serial.printf(
    "[RX] L(%+3d,%+3d) R(%+3d,%+3d) | LS:%d RS:%d LB:%d RB:%d"
    " | MOT L:%s/%3d R:%s/%3d | PKT:%lu | Link:%s\n",
    scl(data.LX), scl(data.LY),
    scl(data.RX), scl(data.RY),
    data.LS ? 1 : 0, data.RS ? 1 : 0,
    data.LB ? 1 : 0, data.RB ? 1 : 0,
    motDir(data.LY), leftPWM,
    motDir(data.RY), rightPWM,
    packetCount,
    connected ? "OK" : "DEAD"
  );
}

// ╔══════════════════════════════════════════════════════════╗
// ║                         SETUP                           ║
// ╚══════════════════════════════════════════════════════════╝

void setup()
{
  delay(500);             // let USB CDC enumerate before printing
  Serial.begin(115200);

  Serial.println("\n\n========================================");
  Serial.println("  URC RX v0.6 — XIAO ESP32-C6");
  Serial.println("========================================");

  // ── Motor direction + standby pins ───────────────────────
  pinMode(PIN_AIN1, OUTPUT); digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT); digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT); digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT); digitalWrite(PIN_BIN2, LOW);
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);  // standby until link
  Serial.println("[INIT] Motor pins OK — TB6612 in STANDBY");

  // ── LEDC PWM — Core 3.x API (no channel numbers needed) ──
  // Core 2.x used ledcSetup(ch) + ledcAttachPin(pin, ch)
  // Core 3.x uses ledcAttach(pin, freq, res) directly
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);
  Serial.println("[INIT] LEDC PWM OK (20kHz, 8-bit)");

  // ── Status LEDs ───────────────────────────────────────────
  pinMode(PIN_LED_G, OUTPUT); digitalWrite(PIN_LED_G, LOW);
  pinMode(PIN_LED_R, OUTPUT); digitalWrite(PIN_LED_R, HIGH); // red = waiting
  Serial.println("[INIT] Status LEDs OK — red on (waiting for link)");

  // ── Button output pins — uncomment when wired ─────────────
  // pinMode(PIN_LB_OUT, OUTPUT); digitalWrite(PIN_LB_OUT, LOW);
  // pinMode(PIN_RB_OUT, OUTPUT); digitalWrite(PIN_RB_OUT, LOW);

  // ── WiFi + ESP-NOW ────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  Serial.print("[INIT] *** RX MAC: ");
  Serial.print(WiFi.macAddress());
  Serial.println("  ← paste this into TX RX_MAC[]");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAILED");
    while (1) {                          // blink red fast = fatal error
      digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R));
      delay(150);
    }
  }
  Serial.println("[INIT] ESP-NOW OK");

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("[INIT] Receive callback registered (Core 3.x)");

  Serial.printf("[INFO] ControllerData: %d bytes (must match TX)\n",
                sizeof(ControllerData));
  Serial.println("[INIT] Waiting for TX...");
  Serial.println("========================================\n");
}

// ╔══════════════════════════════════════════════════════════╗
// ║                          LOOP                           ║
// ╚══════════════════════════════════════════════════════════╝

void loop()
{
  unsigned long now = millis();

  // ── Link watchdog ────────────────────────────────────────
  // No packet for ACK_TIMEOUT_MS → declare link lost, stop motors
  bool wasConnected = connected;
  connected = (now - lastRx < ACK_TIMEOUT_MS);

  if (connected != wasConnected) {
    if (connected) {
      Serial.println("[LINK] ✓ LINK UP — driver enabled, motors ready");
      digitalWrite(PIN_STBY, HIGH);   // TB6612 active
      digitalWrite(PIN_LED_G, HIGH);
      digitalWrite(PIN_LED_R, LOW);
    } else {
      Serial.println("[LINK] ✗ LINK LOST — stopping motors, driver standby");
      stopMotors();
      digitalWrite(PIN_STBY, LOW);    // TB6612 standby → motors coast
      digitalWrite(PIN_LED_G, LOW);
      digitalWrite(PIN_LED_R, HIGH);
    }
  }

  // ── Process packet ───────────────────────────────────────
  if (newPacket && connected) {
    newPacket = false;

    // ── Drive mixer ────────────────────────────────────────
    //
    //  Input sticks (all -2047..+2047):
    //
    //  LY → forward/backward
    //       LY negative = stick pushed forward
    //       Negate so positive base = forward
    //
    //  LX → spin/rotate
    //       LX positive = rotate right
    //         left  motor += LX  (faster)
    //         right motor -= LX  (slower)
    //
    //  RX → drift/curve (both wheels same direction, diff speed)
    //       RX positive = drift right
    //         left  wheel += RX      (dominant)
    //         right wheel += RX / 2  (less)
    //
    //  All three sum together, clamped to ±2047 before driving.

    int base = -data.LY;   // negate: fwd stick push → positive base

    int leftSpeed  = base + data.LX;
    int rightSpeed = base - data.LX;

    if (data.RX >= 0) {
      leftSpeed  += data.RX;
      rightSpeed += data.RX / 2;
    } else {
      leftSpeed  += data.RX / 2;
      rightSpeed += data.RX;
    }

    leftSpeed  = constrain(leftSpeed,  -2047, 2047);
    rightSpeed = constrain(rightSpeed, -2047, 2047);

    driveMotor(true,  leftSpeed);
    driveMotor(false, rightSpeed);

    // ── Button hookup points ───────────────────────────────
    // Uncomment the block when you decide what to wire to each button.
    // Define the output pin at the top first.

    // LB → LED strip, headlight, relay
    // digitalWrite(PIN_LB_OUT, data.LB ? HIGH : LOW);

    // RB → buzzer, beeper, weapon motor relay
    // digitalWrite(PIN_RB_OUT, data.RB ? HIGH : LOW);

    // LS → spin in place clockwise at fixed speed
    // if (data.LS) { driveMotor(true, 900); driveMotor(false, -900); }

    // RS → emergency full stop override
    // if (data.RS) { driveMotor(true, 0); driveMotor(false, 0); }

    // ── Button state change → print immediately ────────────
    if (data.LB != prevLB) {
      Serial.printf("[BTN] LB %s\n", data.LB ? "PRESSED" : "released");
      prevLB = data.LB;
    }
    if (data.RB != prevRB) {
      Serial.printf("[BTN] RB %s\n", data.RB ? "PRESSED" : "released");
      prevRB = data.RB;
    }
    if (data.LS != prevLS) {
      Serial.printf("[BTN] LS %s\n", data.LS ? "PRESSED" : "released");
      prevLS = data.LS;
    }
    if (data.RS != prevRS) {
      Serial.printf("[BTN] RS %s\n", data.RS ? "PRESSED" : "released");
      prevRS = data.RS;
    }
  }

  // ── Serial snapshot every 500ms ──────────────────────────
  if (now - lastSerialPrint >= SERIAL_PRINT_INTERVAL) {
    printSerialData();
    lastSerialPrint = now;
  }
}