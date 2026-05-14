// ================================================================
//  URC RX v0.9.1 — Robot Receiver
//  Role    : Slave / Receiver
//  Board   : ESP32-C3 Mini
//  Remote 1 : { 0xA4, 0xF0, 0x0F, 0x82, 0x6C, 0xA0 }

//  Hardware:
//    TB6612FNG dual motor driver
//    2x N20 motors (or similar DC motors)
//    ESP-NOW radio (receives from URC TX)
//
//  ── WIRING ──────────────────────────────────────────────────
//  GPIO 2  → AIN1   Left  motor direction A
//  GPIO 3  → AIN2   Left  motor direction B
//  GPIO 4  → PWMA   Left  motor speed (PWM)
//  GPIO 5  → BIN1   Right motor direction A
//  GPIO 6  → BIN2   Right motor direction B
//  GPIO 7  → PWMB   Right motor speed (PWM)
//  GPIO 8  → STBY   TB6612 enable (HIGH = active, LOW = off)
//  3V3     → VCC    TB6612 logic power
//  GND     → GND    Common ground
//  LiPo+   → VM     Motor power rail (separate from logic)
//
//  ── CONTROL MAPPING ─────────────────────────────────────────
//  Ly  (-99…+99) → Forward / Backward speed
//                   +99 = full forward, -99 = full reverse
//  Rx  (-99…+99) → Left / Right steering (tank mix)
//                   +99 = full right,   -99 = full left
//  LBt (bool)    → ARM / DISARM  TB6612 STBY pin
//                   true = armed (motors active)
//                   false = disarmed (motors OFF, safe)
//  LABt (bool)   → Stub — add your feature here (lights, horn…)
//  RABt (bool)   → Stub — add your feature here
//  RBt  (bool)   → Stub — reserved
//
//  ── HOW MIXING WORKS ────────────────────────────────────────
//  leftSpeed  = Ly + Rx     (clamp to ±255)
//  rightSpeed = Ly - Rx     (clamp to ±255)
//  This gives arcade-style tank mixing:
//    Full forward + turn right → right motor slows / reverses
//    Stick neutral + right → on-spot right spin
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <ESP32Servo.h>

// ================================================================
//  MOTOR DRIVER PINS  (TB6612FNG)
// ================================================================
#define PIN_AIN1 2  // Left  motor direction A
#define PIN_AIN2 3  // Left  motor direction B
#define PIN_PWMA 4  // Left  motor PWM speed

#define PIN_BIN1 5  // Right motor direction A
#define PIN_BIN2 6  // Right motor direction B
#define PIN_PWMB 7  // Right motor PWM speed

#define PIN_STBY 8  // TB6612 standby: HIGH=active, LOW=off \
                    // Controlled by LBt (arm/disarm)
#define SERVO_PIN 10
// ================================================================
//  MOTOR TUNING
//  ── Speed ───────────────────────────────────────────────────
//  MAX_PWM   : Maximum PWM value sent to motors.
//              255 = full voltage.  Reduce if motors overheat
//              or for 3V motors on a higher voltage supply.
//              Recommended: 200 for 3V N20 on 3.7V LiPo.
//
//  MIN_PWM   : Minimum PWM to actually move the motor.
//              Below this threshold the motor just hums.
//              Tune this if motors twitch but don't spin.
//              Try 40–80 depending on your motor + load.
//
//  ── Steering ────────────────────────────────────────────────
//  TURN_MIX  : How strongly Rx (steering) affects speed.
//              1.0 = full mix (sharp turns, may spin in place)
//              0.5 = gentle mix (wide arcs, less aggressive)
//              Range: 0.0–1.0
//
//  ── PWM ─────────────────────────────────────────────────────
//  PWM_FREQ  : 20 kHz = above hearing range (silent operation)
//              Lower to 1000 Hz if your motor driver needs it.
//  PWM_RES   : 8-bit → 0–255 range.  Don't change.
// ================================================================
#define MAX_PWM 220     // Max PWM duty (0–255).  Try 200 for 3V motors.
#define MIN_PWM 50      // Deadband: below this, motor won't spin. Tune per motor.
#define PWM_FREQ 20000  // Hz.  20kHz = silent.
#define PWM_RES 8       // Bits.  8 = 0–255 range.

// ── Steering mix strength (0.0 = no steering, 1.0 = full tank mix) ──
#define TURN_MIX 1.0f  // Try 0.7 for gentler cornering

// ================================================================
//  SAFETY TIMEOUT
//  If no ESP-NOW packet arrives within this many ms,
//  motors are stopped automatically.
//  Increase if you're getting false stops at range.
//  Decrease for faster emergency cut on signal loss.
// ================================================================
#define LINK_TIMEOUT_MS 2000  // ms.  300ms = ~15 missed packets at 50Hz.

// ================================================================
//  TX MAC ADDRESS  — put the URC transmitter MAC here
//  Read it from TX Serial Monitor at boot: "[INIT] TX MAC: XX:XX:…"
//  Leave as broadcast during bench testing (0xFF×6)
// ================================================================
// uint8_t txMAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t txMAC[] = { 0xA4, 0xF0, 0x0F, 0x82, 0x6C, 0xA0 };

// ================================================================
//  DATA STRUCT — MUST be byte-identical to TX side
// ================================================================
typedef struct {
  int Lx, Ly;
  int Rx, Ry;
  bool LBt, RBt;
  bool LABt, RABt;
  int BAT;
} ControllerData;

typedef struct {
  bool alive;
} AckData;

// ================================================================
//  RUNTIME STATE
// ================================================================
ControllerData rxData;
AckData ackPacket = { true };

Servo myServo;
bool servoArmed = false;
unsigned long lastPacketTime = 0;  // timestamp of last good packet
bool armed = false;                // mirrors LBt state (STBY pin)
bool wasArmed = false;             // previous arm state for edge detection

// ================================================================
//  MOTOR DRIVER — driveMotor()
//  isLeft : true = left motor (AIN/PWMA), false = right (BIN/PWMB)
//  speed  : -255 … +255
//            positive = forward
//            negative = reverse
//            zero     = coast
// ================================================================
void driveMotor(bool isLeft, int speed) {
  int pinIn1 = isLeft ? PIN_AIN1 : PIN_BIN1;
  int pinIn2 = isLeft ? PIN_AIN2 : PIN_BIN2;
  int pinPWM = isLeft ? PIN_PWMA : PIN_PWMB;

  int duty = constrain(abs(speed), 0, 255);

  // Apply minimum PWM threshold — below MIN_PWM just coast instead of stall
  if (duty > 0 && duty < MIN_PWM) duty = MIN_PWM;

  if (speed > 0) {
    digitalWrite(pinIn1, HIGH);
    digitalWrite(pinIn2, LOW);
  } else if (speed < 0) {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, HIGH);
  } else {
    // Coast (both LOW).  Swap both HIGH for active brake if needed.
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    duty = 0;
  }
  ledcWrite(pinPWM, duty);
}

// Stop both motors (coast)
void stopMotors() {
  driveMotor(true, 0);
  driveMotor(false, 0);
}

// ================================================================
//  ARCADE TANK MIX
//  Converts Ly (throttle -99..+99) and Rx (steering -99..+99)
//  into individual left/right motor PWM values.
//
//  leftSpeed  = (Ly + Rx * TURN_MIX) scaled to MAX_PWM
//  rightSpeed = (Ly - Rx * TURN_MIX) scaled to MAX_PWM
//
//  If motors are reversed (robot drives backward when commanded forward),
//  negate the speed passed to driveMotor() for that side.
// ================================================================
void applyDrive(int ly, int rx) {
  // Scale from -99..+99 to -MAX_PWM..+MAX_PWM
  float scale = (float)MAX_PWM / 99.0f;

  float throttle = ly * scale;
  float steer = rx * scale * TURN_MIX;

  int leftPWM = (int)constrain(throttle + steer, -255.0f, 255.0f);
  int rightPWM = (int)constrain(throttle - steer, -255.0f, 255.0f);

  driveMotor(true, leftPWM);    // LEFT  motor  (swap — fixes reversed steering)
  driveMotor(false, rightPWM);  // RIGHT motor
  // Uncomment for real-time drive debug (verbose — disable at speed):
  // Serial.printf("[DRIVE] Ly:%+3d Rx:%+3d → L:%+4d R:%+4d\n", ly, rx, leftPWM, rightPWM);
}

// ================================================================
//  LABt ACTION — Left stick click
//  Add your feature here: LED strip, horn tone, servo, etc.
// ================================================================
void onLABt() {
  Serial.println("[ACTION] LABt triggered — add your feature here");
  // Example: digitalWrite(PIN_HORN, HIGH);
}

// ================================================================
//  RABt ACTION — Right stick click
//  Add your feature here
// ================================================================

void onRABt(bool pressed) {
  if (!servoArmed) return;
  if (pressed == 1) {
    myServo.write(90);
    Serial.println("[SERVO] 90");
  } else {
    myServo.write(200);
    Serial.println("[SERVO] 180");
  }
}

// ================================================================
//  RBt ACTION — Right push button
//  Add your feature here
// ================================================================
void onRBt(bool pressed) {
  if (pressed == 1) {
    servoArmed = true;
    myServo.attach(SERVO_PIN, 500, 2400);
    myServo.write(180);
    Serial.print("[ACTION] RBt pressed — ARMED: ");
    Serial.println(servoArmed);
  } else {
    servoArmed = false;
    myServo.detach();  // remove delay(300) — it was blocking ESP-NOW
    Serial.print("[ACTION] RBt released — DISARMED: ");
    Serial.println(servoArmed);
  }
}
// ================================================================
//  ESP-NOW RECEIVE CALLBACK
//  Runs on radio task (not loop()) — keep it short and fast.
//  Copy data, update timestamp, send ACK.
// ================================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) 
{
  // ── FILTER UNKNOWN SENDERS ─────────────────────────────
  if (memcmp(info->src_addr, txMAC, 6) != 0) 
  {
    Serial.print("[BLOCKED] Packet from: ");

    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X", info->src_addr[i]);
      if (i < 5) Serial.print(":");
    }

    Serial.println("  -> ignored");
    return;
  }

  // ── SIZE CHECK ─────────────────────────────────────────
  if (len != sizeof(ControllerData)) {
    Serial.printf("[RX] Bad packet size: got %d expected %d\n",
                  len,
                  (int)sizeof(ControllerData));
    return;
  }

  // ── VALID PACKET ───────────────────────────────────────
  memcpy(&rxData, incomingData, sizeof(rxData));
  lastPacketTime = millis();

  // ACK back only to valid TX
  esp_now_send(info->src_addr,
               (uint8_t *)&ackPacket,
               sizeof(ackPacket));
}

// ================================================================
//  PROCESS INPUTS
//  Called every loop tick.
//  Handles arm/disarm, drive mixing, and button stubs.
// ================================================================
void processInputs() {
  static bool prevLABt = false, prevRABt = false;
  static bool prevRBt = false;

  // ── ARM / DISARM via LBt ──────────────────────────────────
  armed = rxData.LBt;
  if (armed != wasArmed) {
    digitalWrite(PIN_STBY, armed ? HIGH : LOW);
    Serial.printf("[ARM] Motors %s\n", armed ? "ARMED — STBY HIGH" : "DISARMED — STBY LOW");
    if (!armed) stopMotors();  // immediate stop on disarm
    wasArmed = armed;
  }

  // ── DRIVE  (only if armed) ───────────────────────────────
  if (armed) {
    applyDrive(rxData.Ly, rxData.Rx);
  } else {
    stopMotors();
  }

  // ── LABt stub — edge detect (press only) ─────────────────
  if (rxData.LABt && !prevLABt) onLABt();
  prevLABt = rxData.LABt;

  // ── RABt — both edges ────────────────────────────────────
  if (rxData.RABt != prevRABt) {
    onRABt(rxData.RABt);
    prevRABt = rxData.RABt;
  }
  // ── RBt stub — both edges ────────────────────────────────
  if (rxData.RBt != prevRBt) {
    onRBt(rxData.RBt);
    prevRBt = rxData.RBt;
  }
}

// ================================================================
//  LINK WATCHDOG
//  If no packet arrives within LINK_TIMEOUT_MS, disarm and stop.
//  This prevents the robot from driving away if TX loses power.
// ================================================================
void linkWatchdog() {
  static bool wasLinked = true;
  static unsigned long lostSince = 0;

  bool linked = (millis() - lastPacketTime < LINK_TIMEOUT_MS);

  if (!linked && wasLinked) {
    // Signal just dropped — start the timer but don't act yet
    lostSince = millis();
    wasLinked = false;
  }

  if (!wasLinked && !linked) {
    // Still lost — check if we've been lost long enough to act
    if (millis() - lostSince > LINK_TIMEOUT_MS) {
      //Serial.println("[LINK] !! SIGNAL LOST — stopping motors !!");
      rxData.Lx = rxData.Ly = rxData.Rx = rxData.Ry = 0;
      rxData.LABt = rxData.RABt = false;
      stopMotors();
      digitalWrite(PIN_STBY, LOW);
      armed = false;
    }
  }

  if (linked && !wasLinked) {
    //Serial.println("[LINK] Signal restored");
    wasLinked = true;
    lostSince = 0;
  }

  // Periodic heartbeat
  static unsigned long lastPrint = 0;
  if (linked && millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("[RX] Ly:%+3d Rx:%+3d | LBt:%d RBt:%d | LABt:%d RABt:%d | Armed:%d | BAT(TX):%d%%\n",
                  rxData.Ly, rxData.Rx,
                  (int)rxData.LBt, (int)rxData.RBt,
                  (int)rxData.LABt, (int)rxData.RABt,
                  (int)armed, rxData.BAT);
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  delay(300);
  Serial.begin(115200);
  //Serial.setTxTimeoutMs(0); // This prevents the "slow" lag when not connected
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(90);
  Serial.println("[SERVO] Ready at 90");
  Serial.println("\n==============================");
  Serial.println("  URC RX v0.9.2 — Robot Boot(Seovo baesd)");
  Serial.println("==============================");
  WiFi.mode(WIFI_STA);

  // Print MAC address
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  // ── Motor direction pins — LOW (safe) before enabling TB6612 ──
  pinMode(PIN_AIN1, OUTPUT);
  digitalWrite(PIN_AIN1, LOW);
  pinMode(PIN_AIN2, OUTPUT);
  digitalWrite(PIN_AIN2, LOW);
  pinMode(PIN_BIN1, OUTPUT);
  digitalWrite(PIN_BIN1, LOW);
  pinMode(PIN_BIN2, OUTPUT);
  digitalWrite(PIN_BIN2, LOW);

  // ── STBY LOW at boot — TB6612 disabled until TX arms it ──────
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, LOW);
  Serial.println("[INIT] STBY LOW — motors disarmed at boot");

  // ── LEDC PWM (ESP32 Arduino Core 3.x API) ────────────────────
  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);
  Serial.println("[INIT] PWM channels attached");

  // ── WiFi + ESP-NOW ────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  Serial.print("[INIT] RX MAC (give this to TX): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAILED — halting");
    while (1) delay(100);
  }
  Serial.println("[INIT] ESP-NOW OK");

  esp_now_register_recv_cb(OnDataRecv);

  // ── Add TX as peer so we can send ACK back ────────────────────
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, txMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[WARN] Could not add TX peer — ACK will not reach TX");
    Serial.println("[WARN]  → Update txMAC[] with your TX MAC address");
  } else {
    Serial.println("[INIT] TX peer added — ACK enabled");
  }

  Serial.println("[INIT] Waiting for TX signal...");
  Serial.println("  → Disarmed at boot.  Press LBt on TX to arm motors.");
  Serial.println("==============================\n");
}

// ================================================================
//  MAIN LOOP — runs as fast as possible (no delay)
//  All timing handled internally by linkWatchdog()
// ================================================================
void loop() {
  linkWatchdog();   // check for signal loss, stop motors if link dead
  processInputs();  // apply drive, arm state, button stubs
  // No delay here — keeps latency as low as possible.
  // ESP-NOW callbacks run on their own task independently.
}
