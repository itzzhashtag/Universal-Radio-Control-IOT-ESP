// ================================================================
//  ESP32 Universal RC Controller — TX v2.0
//  MAC : D4:E9:F4:A4:C9:7C
//  Role    : Transmitter / Master
//  Board   : ESP32 (38-pin devkit)
//  Protocol: ESP-NOW (broadcast by default) + UART to Nano display
//
//  ── What changed from v1.9 ──────────────────────────────────────
//  • Removed all U8g2 / SPI LCD code — Nano handles the display now
//  • Removed verbose Serial debug spam (kept key boot/bat lines)
//  • Serial now sends structured 18-byte packets to Nano only
//  • Packet includes screenID + caution-flags so Nano renders the
//    correct screen (boot anim / caution gate / low-bat / normal)
//  • LCD_FPS and BOOT_SETTLE_MS are tunable at the top of the file
//
//  ── Hardware ────────────────────────────────────────────────────
//    2× Analog Joystick   LX=35  LY=32  / RX=33  RY=34
//    4× Push Buttons      LABt=26  RABt=27  LBt=25  RBt=14
//    2× Toggle Switch     TSW1=18  TSW2=19
//    1× Slider Pot        Pot1=39   (ADC1, input-only)
//    1× Slider Pot        Pot2=36   (ADC1, input-only)
//    1× ADS1115           I²C  A0 → battery voltage divider
//    1× RED  LED          GPIO 13   (100Ω → GND)
//    1× GREEN LED         GPIO 12   (100Ω → GND)
//    1× Passive Buzzer    GPIO 23   (100Ω → GND)
//
//  ── UART to Nano ─────────────────────────────────────────────────
//    ESP32 GPIO17 (TX2) → Nano RX0
//    ESP32 GPIO16 (RX2) ← Nano TX1 (optional)
//    ESP32 GND          ↔ Nano GND
//    Baud rate must match UART_BAUD below on both ends.
//
//  ── Connection Detection ─────────────────────────────────────────
//    Receiver sends AckData packet back after each ESP-NOW packet.
//    OnDataRecv() stamps lastAckTime.
//    updateConnectionLED() sets gConnected = true within ACK_TIMEOUT.
//    GREEN = link alive  |  RED = no ACK in last ACK_TIMEOUT ms
//
//  ── MAC Addresses ────────────────────────────────────────────────
//    BROADCAST 1 → sends to FF:FF:FF:FF:FF:FF (any nearby receiver)
//    BROADCAST 0 → sends only to ROBOT_MAC_BYTES
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ================================================================
//  ★  USER-TUNABLE CONSTANTS  ★
//  Adjust these without digging into the rest of the code.
// ================================================================

// How often the main loop runs (display refresh + send rate).
// 20 Hz = 50 ms delay.  30 Hz = 33 ms.  50 Hz = 20 ms.
// Higher = more responsive sticks, but slightly more radio traffic.
#define LCD_FPS 60  // Hz — sets loop() delay

// How long (ms) to show the "URC Booting / Please Wait" animation
// while capacitors charge up on cold start.
// Measured from the serial output: ~3–4 s before voltage stabilises.
#define BOOT_SETTLE_MS 4000  // ms — boot animation duration

// Joystick dead-band in mapped units (−99..+99).
// Stick values inside ±DEADZONE are reported as 0 (centre).
#define DEADZONE 15

// Minimum axis delta before updating txData (kills ADC jitter).
#define CHANGE_THRESH 2

// Raw ADC threshold for "pot at minimum" check in startup gate.
// analogRead must return >= (4095 − POT_ZERO_THRESH) to pass.
#define POT_ZERO_THRESH 80

// Battery voltage thresholds for a 2S Li-Ion pack.
#define BAT_FULL 8.4f          // V → 100 %
#define BAT_EMPTY 6.6f         // V →   0 %
#define BAT_LOW_V 6.8f         // V → triggers safe mode (radio off)
#define BAT_LOW_PCT 10         // % → startup gate blocks below this
#define BAT_CAL_FACTOR 1.012f  // Trim factor for voltage divider error

// ADS1115 voltage divider ratio: VBAT –100kΩ– A0 –47kΩ– GND
// vBat = vADC × (100+47)/47
#define BAT_DIVIDER_RATIO (147.0f / 47.0f)

// Buzzer
#define BEEP_FREQ 1200  // Hz
#define BEEP_DUR 40     // ms

// ACK watchdog: no ACK within this window → mark as disconnected.
#define ACK_TIMEOUT 500  // ms

// ================================================================
//  ★  BROADCAST / UNICAST TARGET  ★
// ================================================================
#define BROADCAST 1  // 1 = broadcast FF:FF:…  |  0 = unicast to ROBOT_MAC_BYTES
#define ROBOT_MAC_BYTES 0x20, 0xE7, 0xC8, 0x9F, 0x47, 0xF8

#if BROADCAST
uint8_t peerMAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#else
uint8_t peerMAC[] = { ROBOT_MAC_BYTES };
#endif

// ================================================================
//  PIN DEFINITIONS
// ================================================================

// ── Joystick axes (ADC1 only — 34/35/39 are input-only) ─────────
#define PIN_RX 33
#define PIN_RY 34
#define PIN_LX 35
#define PIN_LY 32

// ── Digital inputs (INPUT_PULLUP, active LOW) ────────────────────
#define PIN_LABt 26  // Left  joystick click
#define PIN_RABt 27  // Right joystick click
#define PIN_LBt 25   // Left  shoulder button
#define PIN_RBt 13   // Right shoulder button
#define PIN_TSW1 18  // Toggle switch 1
#define PIN_TSW2 19  // Toggle switch 2

// ── Analog inputs ────────────────────────────────────────────────
#define PIN_POT1 39  // Pot 1 (ADC1, input-only)
#define PIN_POT2 36  // Pot 2 (ADC1, input-only — also ADS1115 A0 for bat)

// ── Outputs ──────────────────────────────────────────────────────
#define PIN_LED_R 2   // Red  LED  HIGH = ON
#define PIN_LED_G 4  // Green LED HIGH = ON
#define PIN_BUZZ 23   // Passive buzzer

// ================================================================
//  PACKET STRUCTURES
//  Must be identical on TX and every ESP-NOW receiver.
//  Field order matters — changing it silently breaks compatibility.
// ================================================================

// Outgoing control data → robot
typedef struct {
  int Lx, Ly;       // Left  joystick X/Y  (−99 .. +99)
  int Rx, Ry;       // Right joystick X/Y  (−99 .. +99)
  bool LABt, RABt;  // Joystick click buttons
  bool LBt, RBt;    // Shoulder buttons
  bool TSW1, TSW2;  // Toggle switches
  int Pot1, Pot2;   // Potentiometers (0 .. 100)
  int BAT;          // Battery % for robot to log/display
} ControllerData;

// Incoming ACK from robot → confirms packet was received
typedef struct {
  bool alive;  // Robot sets this true on every ACK
} AckData;

// ================================================================
//  UART PACKET TO NANO  (18 bytes)
//
//  byte  0   : 0xAA  start marker
//  byte  1   : screenID  (see SCREEN_* constants below)
//  byte  2   : cautionFlags  (bit-field, used on SCREEN_CAUTION)
//                bit0 = sw1OK   bit1 = sw2OK
//                bit2 = p1OK    bit3 = batOK
//  byte  3   : Lx + 99   (0..198, avoids negatives over wire)
//  byte  4   : Ly + 99
//  byte  5   : Rx + 99
//  byte  6   : Ry + 99
//  byte  7   : Pot1  (0..100)
//  byte  8   : Pot2  (0..100)
//  byte  9   : BAT % (0..100)
//  byte 10   : button bitmask
//                bit0=LABt  bit1=RABt  bit2=LBt  bit3=RBt
//                bit4=TSW1  bit5=TSW2  bit6=gConnected
//  bytes 11-16 : reserved (0x00)
//  byte 17   : XOR checksum of bytes 1..16
// ================================================================
#define PKT_LEN 18

// Screen IDs sent in byte[1] of every UART packet.
// Nano uses this to decide which screen to draw.
#define SCREEN_BOOT 0x01     // Booting animation (cap charge wait)
#define SCREEN_CAUTION 0x02  // Safety gate (startup checks)
#define SCREEN_LOWBAT 0x03   // Low battery safe mode
#define SCREEN_NORMAL 0x04   // Normal operation

// ================================================================
//  GLOBALS
// ================================================================
ControllerData txData;  // Live outgoing ESP-NOW packet
Adafruit_ADS1115 ads;   // ADS1115 for battery voltage

int gBatPct = 100;        // Battery %, updated by batteryManager()
bool gConnected = false;  // true = ACK received within ACK_TIMEOUT

static unsigned long lastAckTime = 0;  // millis() of last received ACK
static float batV = 8.4f;              // Last measured battery voltage

// Loop delay in ms, derived from LCD_FPS
static const uint16_t LOOP_DELAY_MS = 1000 / LCD_FPS;

// ================================================================
//  HELPERS
// ================================================================

// Map raw ADC (0–4095) → axis value (−99..+99) with centre dead-band.
int mapAxis(int raw) {
  int v = map(raw, 0, 4095, 99, -99);
  return (abs(v) < DEADZONE) ? 0 : v;
}

// Map raw ADC (0–4095) → pot value (0..100).
int mapPot(int raw) {
  return map(raw, 0, 4095, 0, 100);
}

// Read battery voltage via ADS1115 A0, return % (0–100).
// vOut is set to the computed pack voltage for threshold comparisons.
int readBatPct(float &vOut) {
  // ADS1115 LSB at GAIN_TWOTHIRDS = 0.1875 mV
  float vADC = ads.readADC_SingleEnded(0) * 0.1875f / 1000.0f;
  vOut = vADC * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR;

  if (vOut >= BAT_FULL) return 100;
  if (vOut <= BAT_EMPTY) return 0;
  return (int)((vOut - BAT_EMPTY) * 100.0f / (BAT_FULL - BAT_EMPTY));
}

// Single short beep — non-blocking (ESP32 timer-based tone()).
void beep() {
  tone(PIN_BUZZ, BEEP_FREQ, BEEP_DUR);
}

// ================================================================
//  BEEP ON EDGE + POT PITCH FEEDBACK
//  Fires one beep per LOW→HIGH transition on each digital input.
//  Must be called AFTER txData has been freshly updated.
// ================================================================
void checkBeeps() {
  static bool pLABt = 0, pRABt = 0, pLBt = 0, pRBt = 0, pTSW1 = 0, pTSW2 = 0;
  static int pPot1Raw = -1, pPot2Raw = -1;

  // ── Joystick clicks — soft futuristic tap ──────────────────
  if (txData.LABt && !pLABt) soundJoyClick();
  if (txData.RABt && !pRABt) soundJoyClick();

  // ── Shoulder buttons — heavier thunk + pip ─────────────────
  if (txData.LBt && !pLBt) soundShoulderBtn();
  if (txData.RBt && !pRBt) soundShoulderBtn();

  // ── Toggle switches — direction-aware blip ─────────────────
  if (txData.TSW1 != pTSW1) txData.TSW1 ? soundToggleOn() : soundToggleOff();
  if (txData.TSW2 != pTSW2) txData.TSW2 ? soundToggleOn() : soundToggleOff();

  // ── Pot ticks — chirp at pitch matching position ───────────
  int rawP1 = analogRead(PIN_POT1);
  int rawP2 = analogRead(PIN_POT2);

  if (pPot1Raw != -1 && abs(rawP1 - pPot1Raw) > 80) {
    soundPotTick(rawP1);
    pPot1Raw = rawP1;
  }
  if (pPot2Raw != -1 && abs(rawP2 - pPot2Raw) > 80) {
    soundPotTick(rawP2);
    pPot2Raw = rawP2;
  }

  if (pPot1Raw == -1) pPot1Raw = rawP1;
  if (pPot2Raw == -1) pPot2Raw = rawP2;

  pLABt = txData.LABt;
  pRABt = txData.RABt;
  pLBt  = txData.LBt;
  pRBt  = txData.RBt;
  pTSW1 = txData.TSW1;
  pTSW2 = txData.TSW2;
}
// ================================================================
//  CONNECTION TONES  — tuned for 5V speaker
//  Inspired by real RC systems / sci-fi UI sounds
// ================================================================

// ================================================================
//  SOUND ENGINE  — GPIO23 square wave, sweep simulation
//  Simulates Radiomaster-style sounds by stepping frequency
//  rapidly through a curve instead of a single fixed tone.
// ================================================================

// ── Smooth frequency sweep using rapid tone() steps ──────────
//  startFreq → endFreq over durationMs in 'steps' increments
//  Lower steps = grainier, higher = smoother (max ~40 useful)
void playSweep(int startFreq, int endFreq, int durationMs, int steps = 30) {
  int stepDelay = durationMs / steps;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    // Ease in-out curve: feels smoother than linear
    float eased = t * t * (3.0f - 2.0f * t);
    int freq = startFreq + (int)((endFreq - startFreq) * eased);
    tone(PIN_BUZZ, freq, stepDelay + 2);  // +2 overlap avoids gaps
    delay(stepDelay);
  }
}

// ── Single tone with optional gap after ──────────────────────
void playTone(int freq, int durationMs, int gapMs = 0) {
  tone(PIN_BUZZ, freq, durationMs);
  delay(durationMs + gapMs);
}

// ================================================================
//  SOUND PROFILES  — Radiomaster-style on square wave speaker
// ================================================================

void toneConnected() {
  // "Link Acquired"
  // Fast rising sweep → silence snap → two confident pips
  playSweep(400, 2000, 160, 35);   // whooooop
  noTone(PIN_BUZZ); delay(25);     // snap silence = punch
  playTone(2200, 70,  35);         // bip
  playTone(2600, 110, 0);          // BIP (longer = confident)
  noTone(PIN_BUZZ);
}

void toneDisconnected() {
  // "Link Lost"
  // Sharp pip → falling wail → low double thud
  playTone(1800, 60, 20);          // sharp alert pip
  playSweep(1600, 280, 220, 35);   // wheeeeer (falling wail)
  noTone(PIN_BUZZ); delay(40);
  playTone(420, 90,  55);          // low thud 1
  playTone(320, 130, 0);           // low thud 2 (lower = ominous)
  noTone(PIN_BUZZ);
}

void toneBoot() {
  // "System Online"
  // Three-stage rising sweep → pause → final sustain pip
  playSweep(250, 700,  110, 25);   // low warm rise
  playSweep(700, 1500, 130, 30);   // mid sweep
  playSweep(1500, 2200, 90, 25);   // top sweep
  noTone(PIN_BUZZ); delay(35);     // breath before final note
  playTone(2200, 220, 0);          // final sustain
  noTone(PIN_BUZZ);
}

void toneLowBat() {
  // "Battery Warning"
  // Three descending stabs — urgent and hard to ignore
  playTone(900, 75, 55);
  playTone(750, 75, 55);
  playTone(550, 180, 0);
  noTone(PIN_BUZZ);
}
// ================================================================
//  MODERN UI BUTTON SOUNDS
//  Different feel for each input type
// ================================================================

// ── Micro sweep tap — for joystick clicks (LABt / RABt) ──────
// Feels like a soft futuristic "tik" — light and satisfying
void soundJoyClick() {
  playSweep(1800, 2400, 35, 8);
  noTone(PIN_BUZZ);
}

// ── Shoulder button — heavier "thunk-pip" ────────────────────
// Feels like a mechanical switch with a confirm tone
void soundShoulderBtn() {
  playTone(600, 18, 0);             // low thunk
  noTone(PIN_BUZZ); delay(10);
  playSweep(1200, 1800, 40, 10);    // rising pip
  noTone(PIN_BUZZ);
}

// ── Toggle switch ON — ascending blip ────────────────────────
void soundToggleOn() {
  playSweep(800, 1600, 55, 12);
  noTone(PIN_BUZZ);
}

// ── Toggle switch OFF — descending blip ──────────────────────
void soundToggleOff() {
  playSweep(1600, 800, 55, 12);
  noTone(PIN_BUZZ);
}

// ── Pot movement — smooth pitch tick ─────────────────────────
// Short sine-like chirp at the frequency matching pot position
void soundPotTick(int rawValue) {
  int freq = map(rawValue, 0, 4095, 500, 2200);
  playSweep(freq - 80, freq + 80, 30, 8);  // tiny sweep around note
  noTone(PIN_BUZZ);
}
// ================================================================
//  ESP-NOW CALLBACKS
// ================================================================

// Fires after each esp_now_send() attempt.
// A successful status only means the packet left our antenna —
// it does NOT confirm the receiver got it. ACK reception does that.
// (For ESP32 Arduino Core < 3.x, remove the wifi_tx_info_t signature
//  and use: void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status))
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  (void)status;  // not used in ACK-based connection detection
}

// Fires when the robot sends back an AckData heartbeat packet.
// Resets the ACK watchdog timer — this is how we detect link status.
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *buf, int len) {
  if (len == sizeof(AckData)) {
    AckData ack;
    memcpy(&ack, buf, sizeof(ack));
    if (ack.alive)
      lastAckTime = millis();  // reset watchdog
  }
  (void)info;
}

// ================================================================
//  CONNECTION LED
//  Call every loop tick.
//  GREEN = receiver alive within ACK_TIMEOUT ms
//  RED   = no recent ACK
// ================================================================
void updateConnectionLED() {
  static bool prevConnected = false;

  gConnected = (millis() - lastAckTime < ACK_TIMEOUT);

  // Fire tone only on state change, not every loop tick
  if (gConnected && !prevConnected) {
    toneConnected();
  } else if (!gConnected && prevConnected) {
    toneDisconnected();
  }

  prevConnected = gConnected;

  digitalWrite(PIN_LED_G, gConnected ? HIGH : LOW);
  digitalWrite(PIN_LED_R, gConnected ? LOW  : HIGH);
}

// ================================================================
//  SEND UART PACKET TO NANO
//  Packs controller state + screenID + cautionFlags into 18 bytes
//  and writes them to Serial (UART0, GPIO1 TX).
//
//  screenID     : one of SCREEN_* constants
//  cautionFlags : only meaningful on SCREEN_CAUTION
//                 bit0=sw1OK  bit1=sw2OK  bit2=p1OK  bit3=batOK
// ================================================================
void sendDisplayPacket(uint8_t screenID, uint8_t cautionFlags = 0) {
  uint8_t pkt[PKT_LEN];

  pkt[0] = 0xAA;          // start marker (never included in checksum)
  pkt[1] = screenID;      // which screen Nano should draw
  pkt[2] = cautionFlags;  // startup gate status flags

  // Joystick axes offset +99 so range is 0–198 (no signed bytes over wire)
  pkt[3] = (uint8_t)(txData.Lx + 99);
  pkt[4] = (uint8_t)(txData.Ly + 99);
  pkt[5] = (uint8_t)(txData.Rx + 99);
  pkt[6] = (uint8_t)(txData.Ry + 99);

  pkt[7] = (uint8_t)txData.Pot1;
  pkt[8] = (uint8_t)txData.Pot2;
  pkt[9] = (uint8_t)txData.BAT;  // battery % for Nano to display

  // Pack all boolean inputs into one byte
  pkt[10] = (txData.LABt ? 0x01 : 0)
            | (txData.RABt ? 0x02 : 0)
            | (txData.LBt ? 0x04 : 0)
            | (txData.RBt ? 0x08 : 0)
            | (txData.TSW1 ? 0x10 : 0)
            | (txData.TSW2 ? 0x20 : 0)
            | (gConnected ? 0x40 : 0);

  // Reserved bytes — zero-fill for future use
  for (uint8_t i = 11; i < 17; i++) pkt[i] = 0x00;

  // XOR checksum of bytes 1..16 (excludes start marker and checksum itself)
  uint8_t cs = 0;
  for (uint8_t i = 1; i < 17; i++) cs ^= pkt[i];
  pkt[17] = cs;

  Serial2.write(pkt, PKT_LEN);
}

// ================================================================
//  READ ALL INPUTS + SEND ESP-NOW
//  Samples joystick axes, buttons, pots → fills txData → sends.
// ================================================================
void readAndSend() {
  // ── Joystick axes: only update if change exceeds threshold ───
  // This prevents ADC noise from generating unnecessary traffic.
  int nLx = mapAxis(analogRead(PIN_LX));
  int nLy = mapAxis(analogRead(PIN_LY));
  int nRx = mapAxis(analogRead(PIN_RX));
  int nRy = mapAxis(analogRead(PIN_RY));

  if (abs(nLx - txData.Lx) > CHANGE_THRESH) txData.Lx = nLx;
  if (abs(nLy - txData.Ly) > CHANGE_THRESH) txData.Ly = nLy;
  if (abs(nRx - txData.Rx) > CHANGE_THRESH) txData.Rx = nRx;
  if (abs(nRy - txData.Ry) > CHANGE_THRESH) txData.Ry = nRy;

  // ── Digital inputs: pins are INPUT_PULLUP, active LOW ────────
  // Inverted here → txData uses active HIGH (true = pressed)
  txData.LABt = !digitalRead(PIN_LABt);
  txData.RABt = !digitalRead(PIN_RABt);
  txData.LBt = !digitalRead(PIN_LBt);
  txData.RBt = !digitalRead(PIN_RBt);
  txData.TSW1 = !digitalRead(PIN_TSW1);
  txData.TSW2 = !digitalRead(PIN_TSW2);

  // ── Potentiometers ────────────────────────────────────────────
  txData.Pot1 = mapPot(analogRead(PIN_POT1));
  txData.Pot2 = mapPot(analogRead(PIN_POT2));

  // ── Attach current battery % so robot can log/display it ─────
  txData.BAT = gBatPct;

  // ── Beep on button/switch press edges ────────────────────────
  checkBeeps();
  // ── Serial debug line ────────────────────────────────────────
  Serial.printf("[TX→OUT] L(%+3d,%+3d) R(%+3d,%+3d) | LA:%d RA:%d LB:%d RB:%d | S1:%d S2:%d | P1:%3d P2:%3d | BAT: %.2fV %3d%% | Link: %s\n",
                txData.Lx, txData.Ly, txData.Rx, txData.Ry,
                txData.LABt, txData.RABt, txData.LBt, txData.RBt,
                txData.TSW1, txData.TSW2, txData.Pot1, txData.Pot2, batV, txData.BAT, gConnected ? "Connected" : "Dead");
  // ── Fire ESP-NOW packet to peerMAC ───────────────────────────
  esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));
}

// ================================================================
//  BATTERY MANAGER
//  Reads voltage every 250 ms (non-blocking, static timer).
//  Returns true  → low-battery safe mode active (caller must block)
//  Returns false → battery OK, proceed normally
// ================================================================
bool batteryManager() {
  static unsigned long lastRead = 0;
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  static bool killSent = false;

  // Re-read ADC every 250 ms to avoid blocking
  if (millis() - lastRead > 250) {
    lastRead = millis();
    gBatPct = readBatPct(batV);
    //Serial.printf("[BAT] %.2fV  %d%%\n", batV, gBatPct);
  }

  if (batV <= BAT_LOW_V) {
    // First time crossing threshold: zero all outputs and kill radio
    if (!killSent) {
      memset(&txData, 0, sizeof(txData));
      esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));  // tell robot to stop
      Serial.printf("[BAT] !! SAFE MODE !! %.2fV %d%% — radio KILLED\n", batV, gBatPct);
      killSent = true;
    }

    // Fast-blink red LED at ~1.7 Hz while in safe mode
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(PIN_LED_R, ledState);
    }
    digitalWrite(PIN_LED_G, LOW);
    return true;  // caller should render LOWBAT screen
  }

  // Voltage is acceptable — clear the kill flag for next time
  killSent = false;
  return false;
}

// ================================================================
//  STARTUP SAFETY GATE
//  Blocks until ALL conditions are met simultaneously:
//    TSW1 = OFF   (digital HIGH = switch open = pullup held HIGH)
//    TSW2 = OFF
//    Pot1 at physical minimum  (raw ADC ≥ 4095 − POT_ZERO_THRESH)
//    Battery ≥ BAT_LOW_PCT (10 %)
//
//  Reason: prevents the robot lurching when you power on with
//  sticks/pots in unknown positions.
//
//  While blocked → sends SCREEN_CAUTION packet with live flags.
//  When all clear → sends ascending two-tone beep, returns.
// ================================================================
void startup() {
  Serial.println("[BOOT] Safety gate: waiting for safe conditions…");

  while (true) {
    gBatPct = readBatPct(batV);
    txData.BAT = gBatPct;

    bool sw1OK = (bool)digitalRead(PIN_TSW1);  // LOW = switch open
    bool sw2OK = (bool)digitalRead(PIN_TSW2);
    bool p1OK = (analogRead(PIN_POT1) <= (POT_ZERO_THRESH));
    bool p2OK = (analogRead(PIN_POT2) <= (POT_ZERO_THRESH));
    bool batOK = (gBatPct >= BAT_LOW_PCT);

    // Pack flags into one byte for the Nano packet
    uint8_t flags = (sw1OK ? 0x01 : 0)
                    | (sw2OK ? 0x02 : 0)
                    | (p1OK ? 0x04 : 0)
                    | (p2OK ? 0x08 : 0)
                    | (batOK ? 0x10 : 0);

    Serial.printf(
      "[BOOT] SW1:%s SW2:%s P1:%s P2:%s BAT:%s(%d%%)\n",
      sw1OK ? "OK" : "WAIT",
      sw2OK ? "OK" : "WAIT",
      p1OK ? "OK" : "WAIT",
      p2OK ? "OK" : "WAIT",
      batOK ? "OK" : "LOW",
      gBatPct);

    if (sw1OK && sw2OK && p1OK && p2OK && batOK) {
      // All conditions satisfied — tell Nano to show ALL CLEAR
      sendDisplayPacket(SCREEN_NORMAL, 0x0F);  // all flags set = all green
      Serial.println("[BOOT] All clear — proceeding");

      // Ascending two-tone boot beep
      toneBoot();
      delay(1000); // let Nano linger on the normal screen a moment
      return;
    }

    // Not yet safe — tell Nano which checks are failing
    sendDisplayPacket(SCREEN_CAUTION, flags);
    delay(150);
  }
}

// ================================================================
//  SYSTEM CONTROL
//  Wraps batteryManager() with a blocking low-bat recovery loop.
//  If voltage dips below BAT_LOW_V and then recovers, calls
//  startup() again so the operator must confirm safe state.
// ================================================================
void systemControl() {
  static bool wasLow = false;

  bool isLow = batteryManager();

  if (isLow) {
    wasLow = true;

    // Block here sending LOWBAT packets until voltage recovers
    while (batteryManager()) {
      sendDisplayPacket(SCREEN_LOWBAT);
      delay(LOOP_DELAY_MS);
    }
  }

  if (wasLow) {
    wasLow = false;
    startup();  // require operator to confirm safe state again after recovery
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  // Serial  = USB debug
  // Serial2 = Nano display UART
  // Remove the USB cable during normal operation to avoid garbage on Nano RX.
  Serial.begin(115200);                       // USB debug monitor
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // UART2 to Nano

  Serial.println("\n================================================");
  Serial.println(" ESP32 Universal RC Controller — TX v2.0");
  Serial.println("================================================");

  // ── I²C + ADS1115 for battery voltage ────────────────────────
  Wire.begin();
  ads.begin();
  ads.setGain(GAIN_TWOTHIRDS);  // ±6.144 V range, 0.1875 mV/LSB

  // ── Output pins ──────────────────────────────────────────────
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);  // red on during boot
  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_BUZZ, LOW);

  // ── Digital input pins (active LOW with internal pull-up) ────
  pinMode(PIN_LABt, INPUT_PULLUP);
  pinMode(PIN_RABt, INPUT_PULLUP);
  pinMode(PIN_LBt, INPUT_PULLUP);
  pinMode(PIN_RBt, INPUT_PULLUP);
  pinMode(PIN_TSW1, INPUT_PULLUP);
  pinMode(PIN_TSW2, INPUT_PULLUP);
  // Analog pins (32–36, 39) default to input — no pinMode needed

  // ── ADC attenuation: 11 dB → full 0–3.3 V range on ADC1 ─────
  // Without this, ADC saturates at ~1.1 V
  analogSetAttenuation(ADC_11db);

  Serial.println("[BOOT] GPIO + ADC configured");

  // ── WiFi station mode (required for ESP-NOW) ─────────────────
  WiFi.mode(WIFI_STA);
  Serial.printf("[BOOT] TX MAC: %s\n", WiFi.macAddress().c_str());
  delay(100);

  // ── ESP-NOW init ─────────────────────────────────────────────
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAILED — halting");
    while (1) {
      digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R));
      delay(200);
    }
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer (broadcast or unicast per #define above)
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;  // 0 = follow current WiFi channel
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.printf("[BOOT] ESP-NOW ready  mode:%s  peer:%02X:%02X:%02X:%02X:%02X:%02X\n",
                BROADCAST ? "BROADCAST" : "UNICAST",
                peerMAC[0], peerMAC[1], peerMAC[2],
                peerMAC[3], peerMAC[4], peerMAC[5]);

  // ── Boot animation phase ──────────────────────────────────────
  // Capacitors need a few seconds to charge before the voltage
  // reading is valid. Stream SCREEN_BOOT packets during this window
  // so the Nano shows "URC Booting / Please Wait" with the bouncing
  // box animation instead of a confusing blank or caution screen.
  Serial.printf("[BOOT] Boot settle wait: %d ms\n", BOOT_SETTLE_MS);
  unsigned long bootStart = millis();
  while (millis() - bootStart < BOOT_SETTLE_MS) {
    sendDisplayPacket(SCREEN_BOOT);
    delay(LOOP_DELAY_MS);
  }

  // ── Startup safety gate (blocks until controls are in safe pos) ─
  startup();

  // ── Boot complete ─────────────────────────────────────────────
  digitalWrite(PIN_LED_R, LOW);
  Serial.printf("[BOOT] Complete — running at %d Hz\n\n", LCD_FPS);
}

// ================================================================
//  LOOP  — runs at LCD_FPS Hz
// ================================================================
void loop() {
  // 1. Battery check — blocks + sends LOWBAT if voltage is critical.
  //    Re-runs startup() if battery recovers from safe mode.
  systemControl();

  // 2. Sample all inputs and transmit ESP-NOW packet to robot.
  readAndSend();

  // 3. Send updated display packet to Nano over UART.
  sendDisplayPacket(SCREEN_NORMAL);

  // 4. Update GREEN/RED LEDs based on ACK watchdog.
  updateConnectionLED();

  // 5. Wait to hit target frame rate.
  delay(LOOP_DELAY_MS);
}
