/*
 * ============================================================
 *  URC RX v0.6 — Combat Robot Driver
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
 *  │  GND   → GND      (common ground)                      │
 *  │  VM    → LiPo+    (motor supply — 3.7V direct)         │
 *  │                                                         │
 *  │  ── MOTORS ────────────────────────────────────────     │
 *  │  Left  N20 → AO1, AO2 on TB6612FNG                     │
 *  │  Right N20 → BO1, BO2 on TB6612FNG                     │
 *  │  If a motor spins the wrong way, swap its two wires     │
 *  │  (AO1↔AO2 or BO1↔BO2) — no code change needed         │
 *  │                                                         │
 *  │  ── STATUS LEDs ───────────────────────────────────     │
 *  │  Green LED anode → GPIO 9  → 220Ω resistor → GND      │
 *  │  Red   LED anode → GPIO 10 → 220Ω resistor → GND      │
 *  │                                                         │
 *  │  ── BUTTON OUTPUT HOOKUPS (wire when ready) ───────     │
 *  │  LB output  → GPIO 1   (e.g. LED strip, relay)         │
 *  │  RB output  → GPIO 0   (e.g. buzzer, beeper)           │
 *  │  (define pins below, uncomment pinMode + logic)         │
 *  │                                                         │
 *  │  ── POWER ─────────────────────────────────────────     │
 *  │  LiPo+ → TB6612FNG VM  (motor power rail)              │
 *  │  LiPo+ → 3.3V regulator → ESP32-C3 3V3 pin            │
 *  │  LiPo- → GND (common ground for everything)            │
 *  │                                                         │
 *  │  ── NOTE ON N20 3V MOTORS AT 3.7V ─────────────────    │
 *  │  N20s run slightly over rated voltage at 3.7V.          │
 *  │  Fine for combat use. MAX_PWM is set to 217 (85%)       │
 *  │  to protect motor longevity. Set to 255 to remove cap.  │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  ── TANK DRIVE EXPLAINED ──────────────────────────────────
 *
 *  This robot uses TANK DRIVE (skid steer):
 *  • Left  stick Y (LY) controls LEFT  motor speed/direction
 *  • Right stick Y (RY) controls RIGHT motor speed/direction
 *
 *  Stick value range received: -2047 (full back) to +2047 (full fwd)
 *  Dead-zone applied on TX side — values near 0 are already 0.
 *
 *  Motion summary:
 *  ┌──────────────┬───────────┬───────────┐
 *  │  Movement    │ LY        │ RY        │
 *  ├──────────────┼───────────┼───────────┤
 *  │  Forward     │ negative  │ negative  │
 *  │  Backward    │ positive  │ positive  │
 *  │  Turn Left   │ zero/slow │ negative  │
 *  │  Turn Right  │ negative  │ zero/slow │
 *  │  Spin Left   │ positive  │ negative  │
 *  │  Spin Right  │ negative  │ positive  │
 *  │  Stop        │ zero      │ zero      │
 *  └──────────────┴───────────┴───────────┘
 *
 *  Speed is proportional — half stick = half speed.
 *  Mapped from ±2047 → 0–MAX_PWM (0–217 by default).
 *
 *  ── BUTTON HOOKUP POINTS ──────────────────────────────────
 *  data.LB = left  push button on TX controller
 *  data.RB = right push button on TX controller
 *  data.LS = left  stick click on TX controller
 *  data.RS = right stick click on TX controller
 *
 *  All four arrive in every packet. Wire an output to a free
 *  GPIO, define it below, then uncomment the action block.
 *
 *  ── FIRST TIME SETUP ──────────────────────────────────────
 *  1. Flash this file to the ESP32-C3 Mini
 *  2. Open Serial Monitor (115200 baud)
 *  3. Copy the MAC address that prints on boot
 *  4. Paste it into RX_MAC[] in the TX file
 *  5. Flash the TX file to the 30-pin ESP32
 *
 *  ── SERIAL DEBUG OUTPUT (115200 baud) ─────────────────────
 *  Boot  : board MAC, ESP-NOW init status
 *  Loop  : received packet values, motor commands, link events
 *          button state changes printed immediately
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
#define PIN_PWMA  4    // PWM speed — LEDC

// ── Right motor (TB6612FNG channel B) ──────────────────────
#define PIN_BIN1  5    // direction bit A
#define PIN_BIN2  6    // direction bit B
#define PIN_PWMB  7    // PWM speed — LEDC

// ── TB6612FNG standby ──────────────────────────────────────
#define PIN_STBY  8    // HIGH = driver active │ LOW = standby (coast)

// ── Status LEDs ────────────────────────────────────────────
#define PIN_LED_G 9    // Green → link active
#define PIN_LED_R 10   // Red   → no link / waiting

// ╔══════════════════════════════════════════════════════════╗
// ║         BUTTON OUTPUT PINS — define when ready          ║
// ║  Uncomment the define AND the pinMode in setup()        ║
// ║  AND the digitalWrite in the button hookup section      ║
// ╚══════════════════════════════════════════════════════════╝
// #define PIN_LB_OUT  1    // LB → e.g. LED strip, relay, light
// #define PIN_RB_OUT  0    // RB → e.g. buzzer, beeper, weapon
// #define PIN_LS_OUT  20   // LS → e.g. secondary weapon, siren
// #define PIN_RS_OUT  21   // RS → e.g. brake light, indicator

// ╔══════════════════════════════════════════════════════════╗
// ║                     CONFIGURATION                       ║
// ╚══════════════════════════════════════════════════════════╝
#define ACK_TIMEOUT_MS  500    // ms without packet → link lost, motors stop

// PWM — 20kHz puts motor switching noise above hearing range
// 8-bit resolution → duty cycle 0–255
#define PWM_FREQ   20000
#define PWM_RES    8

// Motor speed cap — protects N20 3V motors running at 3.7V
// 255 = full voltage, 217 ≈ 85% duty ≈ ~3.1V effective
// Set to 255 to remove cap entirely
#define MAX_PWM    217

// Serial print interval — packet data printed every N ms
// Not every received packet (arrives at ~50Hz = too fast to read)
#define SERIAL_PRINT_INTERVAL 500

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
// ║                       GLOBALS                           ║
// ╚══════════════════════════════════════════════════════════╝
volatile bool          newPacket    = false;
volatile unsigned long lastRx       = 0;
bool                   connected    = false;
unsigned long          lastSerialPrint = 0;
unsigned long          packetCount  = 0;   // total packets received

// Track previous button states to print only on change
bool prevLB = false, prevRB = false;
bool prevLS = false, prevRS = false;

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
    packetCount++;
  } else {
    // Packet size mismatch — struct probably differs between TX and RX
    Serial.print("[ESP-NOW] Received packet with wrong size: ");
    Serial.print(len); Serial.print(" bytes (expected ");
    Serial.print(sizeof(ControllerData)); Serial.println(")");
    Serial.println("[ESP-NOW] Check ControllerData struct matches TX exactly");
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
 * speed  = -2047..+2047
 *          negative = reverse
 *          zero     = coast (both IN pins LOW)
 *          positive = forward
 *
 * TB6612FNG truth table:
 *   IN1=H IN2=L → forward
 *   IN1=L IN2=H → reverse
 *   IN1=L IN2=L → coast  ← used here for zero speed
 *   IN1=H IN2=H → active brake (swap coast lines below to use)
 */
void driveMotor(bool isLeft, int speed) {
  int ain1 = isLeft ? PIN_AIN1 : PIN_BIN1;
  int ain2 = isLeft ? PIN_AIN2 : PIN_BIN2;
  int pwmP = isLeft ? PIN_PWMA : PIN_PWMB;

  // Map absolute speed 0–2047 → 0–MAX_PWM duty cycle
  int pwm = map(abs(speed), 0, 2047, 0, MAX_PWM);
  pwm = constrain(pwm, 0, MAX_PWM);

  if (speed > 0) {
    // Forward
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else if (speed < 0) {
    // Reverse
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  } else {
    // Coast — swap both HIGH for active brake if preferred
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
    pwm = 0;
  }
  ledcWrite(pwmP, pwm);
}

/*
 * stopMotors()
 * Coasts both motors immediately.
 * Called on link loss — belt-and-suspenders with STBY pin.
 */
void stopMotors() {
  driveMotor(true,  0);
  driveMotor(false, 0);
  Serial.println("[MOTOR] Both motors stopped (coast)");
}

/*
 * getMotionLabel()
 * Returns a human-readable string describing current motion.
 * Used only for Serial output — no impact on drive logic.
 *
 * Threshold: 300 = ~15% of full stick range
 * Values below this are treated as "zero" for labelling purposes.
 */
const char* getMotionLabel(int ly, int ry) {
  const int T = 300;
  bool lFwd  = (ly < -T);
  bool lBack = (ly >  T);
  bool rFwd  = (ry < -T);
  bool rBack = (ry >  T);
  bool lZero = (abs(ly) <= T);
  bool rZero = (abs(ry) <= T);

  if (lZero && rZero)           return "STOP";
  if (lFwd  && rFwd)            return "FORWARD";
  if (lBack && rBack)           return "BACKWARD";
  if (lFwd  && rBack)           return "SPIN RIGHT";
  if (lBack && rFwd)            return "SPIN LEFT";
  if (lZero && rFwd)            return "TURN LEFT (pivot)";
  if (lFwd  && rZero)           return "TURN RIGHT (pivot)";
  if (lZero && rBack)           return "TURN LEFT BACK";
  if (lBack && rZero)           return "TURN RIGHT BACK";
  if (lFwd  && rFwd)            return "FORWARD (curve)";
                                 return "MOVING";
}

/*
 * printSerialData()
 * Single-line serial output matching TX format
 *
 * Format:
 * [RX←IN] L( LX, LY) R( RX, RY) | LA:0 RA:0 LB:0 RB:0 | MOT L:FWD/217 R:STP/0 | Link: OK
 *          └─ scaled ±99 ──────┘   └─ stick clicks ─┘ └─ push buttons ─┘  └─ motor state ──┘
 *
 * MOT L/R:  FWD = forward   REV = reverse   STP = stopped
 * PWM value shown after / — 0 to MAX_PWM (217 by default)
 * Link: OK = receiving packets   Link: Dead = timeout
 */
void printSerialData() {
  auto scl = [](int v) -> int { return v * 99 / 2047; };

  // Motor direction labels
  auto motDir = [](int val) -> const char* {
    if (val < -300) return "FWD";
    if (val >  300) return "REV";
    return "STP";
  };

  // Motor PWM values — same calc as driveMotor()
  int leftPWM  = (abs(data.LY) > 300) ? constrain(map(abs(data.LY), 0, 2047, 0, MAX_PWM), 0, MAX_PWM) : 0;
  int rightPWM = (abs(data.RY) > 300) ? constrain(map(abs(data.RY), 0, 2047, 0, MAX_PWM), 0, MAX_PWM) : 0;

  Serial.printf("[RX←IN ] L(%+3d,%+3d) R(%+3d,%+3d) | LA:%d RA:%d LB:%d RB:%d | MOT L:%s/%3d R:%s/%3d | Link: %s\n",
    scl(data.LX), scl(data.LY),
    scl(data.RX), scl(data.RY),
    data.LS ? 1 : 0,
    data.RS ? 1 : 0,
    data.LB ? 1 : 0,
    data.RB ? 1 : 0,
    motDir(data.LY), leftPWM,
    motDir(data.RY), rightPWM,
    connected ? "OK" : "Dead"
  );
}

// ╔══════════════════════════════════════════════════════════╗
// ║                         SETUP                           ║
// ╚══════════════════════════════════════════════════════════╝
void setup() {
  Serial.begin(115200);
  delay(500);   // let Serial settle

  Serial.println("\n\n========================================");
  Serial.println("   URC RX v0.6 — Combat Robot Driver");
  Serial.println("========================================");

  // ── Motor direction pins ──────────────────────────────────
  Serial.println("[INIT] Setting up motor pins...");
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);    // driver disabled until link established
  Serial.println("[INIT] Motor pins OK — driver in STANDBY");

  // ── LEDC PWM — ESP32 Arduino Core 3.x API ─────────────────
  // Core 3.x: ledcAttach(pin, freq, resolution)
  // Core 2.x: ledcSetup(ch, freq, res) + ledcAttachPin(pin, ch) ← old way
  Serial.println("[INIT] Setting up LEDC PWM (20kHz, 8-bit)...");
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);
  Serial.println("[INIT] LEDC PWM OK");

  // ── Status LEDs ───────────────────────────────────────────
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);  // red on = waiting for link
  digitalWrite(PIN_LED_G, LOW);

  // ── Button output pins — uncomment when you wire them ─────
  // pinMode(PIN_LB_OUT, OUTPUT); digitalWrite(PIN_LB_OUT, LOW);
  // pinMode(PIN_RB_OUT, OUTPUT); digitalWrite(PIN_RB_OUT, LOW);
  // pinMode(PIN_LS_OUT, OUTPUT); digitalWrite(PIN_LS_OUT, LOW);
  // pinMode(PIN_RS_OUT, OUTPUT); digitalWrite(PIN_RS_OUT, LOW);

  // ── ESP-NOW init ──────────────────────────────────────────
  Serial.println("[INIT] Starting WiFi in STA mode...");
  WiFi.mode(WIFI_STA);

  // >>> COPY THIS MAC ADDRESS → paste into RX_MAC[] in TX file <<<
  Serial.print("[INIT] *** RX MAC Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println(" ← copy this into TX RX_MAC[]");

  Serial.println("[INIT] Initialising ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAILED");
    while (1) {
      digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R));
      delay(150);
    }
  }
  Serial.println("[INIT] ESP-NOW OK");

  // Register receive callback — Core 3.x signature
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("[INIT] Receive callback registered (Core 3.x signature)");

  Serial.println("[INIT] ── Setup complete. Waiting for TX packets... ──");
  Serial.print("[INFO] ControllerData struct size: ");
  Serial.print(sizeof(ControllerData));
  Serial.println(" bytes (must match TX)");
  Serial.println("========================================\n");
}

// ╔══════════════════════════════════════════════════════════╗
// ║                          LOOP                           ║
// ╚══════════════════════════════════════════════════════════╝
void loop() {
  unsigned long now = millis();

  // ── Connection watchdog ──────────────────────────────────
  // If no packet arrives within ACK_TIMEOUT_MS → link lost.
  // STBY pulls TB6612FNG into standby — motors coast to stop.
  bool wasConnected = connected;
  connected = (now - lastRx < ACK_TIMEOUT_MS);

  if (connected != wasConnected) {
    if (connected) {
      Serial.println("\n[LINK] ✓ LINK ESTABLISHED — motors enabled");
      digitalWrite(PIN_STBY, HIGH);   // enable motor driver
    } else {
      Serial.println("\n[LINK] ✗ LINK LOST — stopping motors, driver to standby");
      stopMotors();
      digitalWrite(PIN_STBY, LOW);    // standby — coasts motors
    }
    digitalWrite(PIN_LED_G, connected ? HIGH : LOW);
    digitalWrite(PIN_LED_R, connected ? LOW  : HIGH);
  }

  // ── Process incoming packet ──────────────────────────────
  if (newPacket && connected) {
    newPacket = false;

    // ╔══════════════════════════════════════════════════════╗
    // ║                   DRIVE MIXER                       ║
    // ║                                                     ║
    // ║  Three inputs, all in range -2047 .. +2047:         ║
    // ║                                                     ║
    // ║  LY  (Left  stick Y) → forward / backward           ║
    // ║       negative = forward,  positive = backward      ║
    // ║                                                     ║
    // ║  LX  (Left  stick X) → rotate left / right         ║
    // ║       negative = rotate left (spin),               ║
    // ║       positive = rotate right (spin)               ║
    // ║       Motors run opposite directions for spin       ║
    // ║                                                     ║
    // ║  RX  (Right stick X) → drift left / right          ║
    // ║       One wheel gets full RX, other gets 50%        ║
    // ║       Both wheels same direction → curves, not spin ║
    // ║       negative = drift left,  positive = drift right║
    // ║                                                     ║
    // ║  RY  → unused for now                              ║
    // ║                                                     ║
    // ║  All three mix into final L and R motor values.     ║
    // ║  Result clamped to ±2047 before going to motors.   ║
    // ╚══════════════════════════════════════════════════════╝

    int ly = data.LY;   // forward/backward  — negate: fwd stick = negative LY
    int lx = data.LX;   // rotate            — positive = rotate right
    int rx = data.RX;   // drift             — positive = drift right

    // ── Step 1: Forward / Backward base ──────────────────
    // LY negative = forward → we negate so forward = positive internally
    int base = -ly;   // now: positive base = forward, negative = backward

    // ── Step 2: Rotation from LX ─────────────────────────
    // Positive LX = rotate right:
    //   left  motor speeds up   (+lx)
    //   right motor slows down  (-lx)
    // Negative LX = rotate left:
    //   left  motor slows down
    //   right motor speeds up
    int leftSpeed  = base + lx;
    int rightSpeed = base - lx;

    // ── Step 3: Drift from RX ────────────────────────────
    // Drift right (RX positive):
    //   Right wheel gets full RX contribution
    //   Left  wheel gets 50% of RX (same direction → curves right)
    // Drift left (RX negative):
    //   Left  wheel gets full RX contribution (negated below)
    //   Right wheel gets 50%
    //
    // Both wheels move the same direction but at different speeds
    // → robot arcs/curves rather than spinning
    //
    // RX positive = drift right:
    //   left  motor += RX       (more speed on left  → pushes right)
    //   right motor += RX * 0.5 (less speed on right → curves)
    //
    // RX negative = drift left:
    //   left  motor += RX * 0.5
    //   right motor += RX

    if (rx >= 0) {
      // Drift right — left wheel dominant
      leftSpeed  += rx;
      rightSpeed += rx / 2;
    } else {
      // Drift left — right wheel dominant
      leftSpeed  += rx / 2;
      rightSpeed += rx;
    }

    // ── Step 4: Clamp to valid motor range ───────────────
    // All three inputs can stack up beyond ±2047 — clamp before driving
    leftSpeed  = constrain(leftSpeed,  -2047, 2047);
    rightSpeed = constrain(rightSpeed, -2047, 2047);

    // ── Step 5: Drive motors ─────────────────────────────
    driveMotor(true,  leftSpeed);
    driveMotor(false, rightSpeed);

    // ╔══════════════════════════════════════════════════════╗
    // ║              BUTTON HOOKUP POINTS                   ║
    // ║                                                     ║
    // ║  Uncomment the block you want when you decide       ║
    // ║  what to wire. Define the output pin above first.   ║
    // ╚══════════════════════════════════════════════════════╝

    // ── LB: Left button → e.g. LED strip, headlights ──────
    // digitalWrite(PIN_LB_OUT, data.LB ? HIGH : LOW);

    // ── RB: Right button → e.g. buzzer, beeper ────────────
    // digitalWrite(PIN_RB_OUT, data.RB ? HIGH : LOW);

    // ── LS: Left stick click → e.g. spin in place CW ──────
    // if (data.LS) {
    //   driveMotor(true,   900);
    //   driveMotor(false, -900);
    // }

    // ── RS: Right stick click → e.g. emergency full stop ───
    // if (data.RS) {
    //   driveMotor(true,  0);
    //   driveMotor(false, 0);
    // }

    // ── Button state change — print immediately on change ───
    if (data.LB != prevLB) {
      Serial.print("[BTN] LB "); Serial.println(data.LB ? "PRESSED  → action: (not wired yet)" : "released");
      prevLB = data.LB;
    }
    if (data.RB != prevRB) {
      Serial.print("[BTN] RB "); Serial.println(data.RB ? "PRESSED  → action: (not wired yet)" : "released");
      prevRB = data.RB;
    }
    if (data.LS != prevLS) {
      Serial.print("[BTN] LS "); Serial.println(data.LS ? "PRESSED  → action: (not wired yet)" : "released");
      prevLS = data.LS;
    }
    if (data.RS != prevRS) {
      Serial.print("[BTN] RS "); Serial.println(data.RS ? "PRESSED  → action: (not wired yet)" : "released");
      prevRS = data.RS;
    }
  }

  // ── Periodic serial snapshot ─────────────────────────────
  if (now - lastSerialPrint >= SERIAL_PRINT_INTERVAL) {
    printSerialData();
    lastSerialPrint = now;
  }
}