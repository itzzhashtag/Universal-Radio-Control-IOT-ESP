// ================================================================
//  URC v0.9.2 — ESP-NOW RC Controller  (20×4 LCD Edition)
//  Role    : Master / Transmitter
//  Board   : ESP32 38-pin DevKit
//
//  Hardware:
//    2× Analog Joystick     LX/LY (GPIO34/35), RX/RY (GPIO32/33)
//    2× Joystick Click      LABt (GPIO26), RABt (GPIO27)  INPUT_PULLUP active LOW
//    2× Push Button         LBt  (GPIO25), RBt  (GPIO14)  INPUT_PULLUP active LOW
//    1× Buzzer              GPIO18
//    20×4 I2C LCD           address 0x27
//    2× LED                 RED=GPIO13, GREEN=GPIO12
//    Battery divider        GPIO36  (100kΩ + 47kΩ, 2S LiPo)
//
//  LCD Layout (20 chars × 4 rows):
//  ┌────────────────────┐
//  │L +99 +99  R +99+99 │  row 0 — left & right stick axes (Lx,Ly,Rx,Ry)
//  │[d]   CONNECTED  [d]│  row 1 — Ldir-arrow, link status, Rdir-arrow
//  │LBt:0    100%  RBt:0│  row 2 — LBt, battery %, RBt
//  │LABt:0 RABt:0 v0.9.2│  row 3 — stick clicks + firmware version
//  └────────────────────┘
//  (d) = CGRAM direction arrow character
//
//  Row 1 status strings (always 9 chars):
//    "CONNECTED"  — robot ACK received within the last 500 ms
//    "-- NO LNK"  — no recent ACK (robot off or out of range)
//
//  Startup caution screen (20×4):
//  ┌────────────────────┐
//  │===== Caution! =====│  row 0  (fixed)
//  │  LBtn  : OK        │  row 1  "OK" or "HOLD BTN"
//  │  RBtn  : OK        │  row 2  "OK" or "HOLD BTN"
//  │  Batt  : OK ( 99%) │  row 3  "OK" or "LO" + percent
//  └────────────────────┘
//
//  All-clear screen (replaces rows 1–3 when ready):
//  ┌────────────────────┐
//  │===== Caution! =====│
//  │                    │
//  │    ** ALL CLEAR ** │
//  │                    │
//  └────────────────────┘
//
//  Changelog v0.9.2 (20×4 port):
//    - Ported from 16×2 back to 20×4 LCD
//    - lcd() init changed to LiquidCrystal_I2C(0x27, 20, 4)
//    - All snprintf buffers expanded to 21 chars (20 + null)
//    - LCD layout redesigned across all 4 rows:
//        Row 0: Stick axes (Lx, Ly, Rx, Ry)
//        Row 1: Direction arrows + connection status string
//        Row 2: Push buttons + battery percentage
//        Row 3: Stick clicks + firmware version label
//    - Added line2, line3 to LCD line cache (was line0, line1 only)
//    - startup() uses all 4 rows with per-check status column
//    - LOW BAT screen uses all 4 rows
//    - Boot splash updated to 20-char lines
//    - CGRAM custom chars retained (slots 0–7, same bitmaps)
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================================================================
//  LCD — 20 columns, 4 rows
//  Original 16×2 code used LiquidCrystal_I2C(0x27, 16, 2).
//  A 20×4 display with wrong dimensions will show garbled output
//  because the HD44780 DDRAM address map differs between sizes.
// ================================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ================================================================
//  PIN DEFINITIONS
// ================================================================
#define RED_LED      13
#define GREEN_LED    12
#define BUZZ_PIN     18

// Joystick analog axes — ADC1 pins only (safe when WiFi is active)
// ADC2 pins (GPIO0,2,4,12–15,25–27) are unusable when WiFi is on.
#define LX_PIN       34   // Left  stick X
#define LY_PIN       35   // Left  stick Y
#define RX_PIN       32   // Right stick X
#define RY_PIN       33   // Right stick Y

// Digital inputs — all INPUT_PULLUP, active LOW (pressed = LOW → reads false)
#define L_STICK_PIN  26   // Left  joystick click  (LABt)
#define R_STICK_PIN  27   // Right joystick click  (RABt)
#define L_BTN_PIN    16   // Left  toggle button   (LBt)
#define R_BTN_PIN    17   // Right toggle button   (RBt)

// Battery sense — 100kΩ : 47kΩ voltage divider, GPIO36 is input-only
#define BAT_PIN      36

// ================================================================
//  BATTERY CALIBRATION
//  Divider ratio: (100k + 47k) / 47k ≈ 3.128  →  multiply v_adc by that
//  BAT_CAL_FACTOR corrects any real-world component tolerance.
//
//  How to find your factor:
//    1. Measure real battery voltage with a multimeter  (e.g. 8.20 V)
//    2. Open Serial Monitor, check "[BAT]" line voltage (e.g. 7.80 V)
//    3. BAT_CAL_FACTOR = 8.20 / 7.80 ≈ 1.051
// ================================================================
#define FULL_BAT          8.4f    // 100% — 2S LiPo fully charged
#define EMPTY_BAT         6.6f    // 0%   — 2S LiPo lower cutoff
#define LOW_BAT_THRESHOLD 6.8f    // Safe-mode trigger voltage
#define LOW_BAT_PERCENT   10      // Startup blocks below this %
#define BAT_CAL_FACTOR    (8.2f / 7.8f)   // ← adjust with real multimeter

// ================================================================
//  JOYSTICK TUNING
//  DEADZONE         — raw units (0–99 scale) ignored around center.
//                     Prevents stick drift when you're not touching it.
//                     Increase (e.g. 20) if you see unwanted movement.
//  CHANGE_THRESHOLD — axis must move this much before the value updates.
//                     Filters ADC jitter, reduces unnecessary radio packets.
// ================================================================
#define DEADZONE          15
#define CHANGE_THRESHOLD   2

// ================================================================
//  BUZZER
// ================================================================
#define BEEP_FREQ   1200   // Hz
#define BEEP_DUR      40   // ms

// ================================================================
//  RECEIVER MAC
//  Replace with your robot ESP32C3 MAC address.
//  All 0xFF = broadcast (useful on the bench without the robot).
// ================================================================
uint8_t receiverMAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ================================================================
//  DATA STRUCT — byte-identical on TX (this file) and RX (robot)
//  If you add/remove/reorder fields, update BOTH sides.
// ================================================================
typedef struct {
  int  Lx, Ly;       // Left  stick  -99 … +99
  int  Rx, Ry;       // Right stick  -99 … +99
  bool LBt, RBt;     // Push buttons  true = held
  bool LABt, RABt;   // Stick clicks  true = pressed
  int  BAT;          // Battery %     0–100
} ControllerData;

typedef struct {
  bool alive;        // Robot sends this as heartbeat ACK
} AckData;

ControllerData data;   // current packet being built
ControllerData prev;   // previous packet (unused beyond struct declaration)

// ================================================================
//  RUNTIME STATE
// ================================================================
int           BaT         = 100;    // battery percent (0–100)
float         batteryVolt = 8.4f;   // calculated voltage
bool          Stat        = false;  // true if robot ACK received recently
unsigned long lastAckTime = 0;      // millis() timestamp of last ACK

// ================================================================
//  LCD LINE CACHE
//  Stores what was last written to each row.
//  printIfChanged() skips the I2C write if the string hasn't changed,
//  preventing flicker and reducing bus traffic.
//  Four strings now — one per row of the 20×4 display.
// ================================================================
String line0 = "";
String line1 = "";
String line2 = "";   // ← new for 20×4 (row 2: buttons + battery)
String line3 = "";   // ← new for 20×4 (row 3: stick clicks + version)

// ================================================================
//  CUSTOM LCD CHARACTERS (CGRAM slots 0–7)
//  Each byte is a row of 5 pixels (bits 4→0).
//  Slots:  0=↗  1=↑  2=↓  3=←  4=→  5=↙  6=↘  7=↖
//  These fit in a single character cell (5×8 pixels).
// ================================================================
byte UR_c[8] = { B00000, B00000, B01111, B00011, B00101, B01001, B10000, B00000 }; // ↗
byte U_c[8]  = { B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100 }; // ↑
byte D_c[8]  = { B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100 }; // ↓
byte L_c[8]  = { B00000, B00010, B00100, B01000, B11111, B01000, B00100, B00010 }; // ←
byte R_c[8]  = { B00000, B01000, B00100, B00010, B11111, B00010, B00100, B01000 }; // →
byte DL_c[8] = { B00000, B00000, B00001, B10010, B10100, B11000, B11110, B00000 }; // ↙
byte DR_c[8] = { B00000, B00000, B10000, B01001, B00101, B00011, B01111, B00000 }; // ↘
byte UL_c[8] = { B00000, B00000, B11110, B11000, B10100, B10010, B00001, B00000 }; // ↖

// ================================================================
//  HELPERS
// ================================================================

// One short beep
void beep() {
  tone(BUZZ_PIN, BEEP_FREQ, BEEP_DUR);
}

// Map raw ADC (0–4095) to -99…+99 with deadzone applied.
// analogRead() returns 0 at one extreme and 4095 at the other;
// map() flips it so pushing forward gives a positive Ly.
int mapAxis(int val) {
  int mapped = map(val, 0, 4095, 99, -99);
  if (abs(mapped) < DEADZONE) return 0;   // center dead zone → zero
  return mapped;
}

// Format axis value for display:
//   0 → "  0"   +50 → "+50"   -99 → "-99"
// We want exactly 3 chars for alignment.
String fmt(int v) {
  char buf[5];
  if (v == 0) sprintf(buf, "  0");
  else        sprintf(buf, "%+3d", v);  // %+3d always prints sign
  return String(buf);
}

// Write text to LCD only if it differs from the cached value.
// col/row = cursor position, text = new string, cache = last written value.
void printIfChanged(int col, int row, String text, String &cache) {
  if (text != cache) {
    lcd.setCursor(col, row);
    lcd.print(text);
    cache = text;
  }
}

// Return CGRAM slot (0–7) for the direction of a joystick.
// Returns 255 if the stick is within the deadzone (= centered).
// Diagonal is detected when BOTH axes exceed the deadzone.
byte getDirIcon(int x, int y) {
  bool xPos = (x >  DEADZONE);
  bool xNeg = (x < -DEADZONE);
  bool yPos = (y >  DEADZONE);
  bool yNeg = (y < -DEADZONE);

  if (yPos && xPos) return 0;  // ↗
  if (yPos && xNeg) return 7;  // ↖
  if (yNeg && xPos) return 6;  // ↘
  if (yNeg && xNeg) return 5;  // ↙
  if (yPos)         return 1;  // ↑
  if (yNeg)         return 2;  // ↓
  if (xNeg)         return 3;  // ←
  if (xPos)         return 4;  // →
  return 255;                  // centered
}

// ================================================================
//  BEEP POLICY
//  Push buttons (LBt, RBt)  → beep on press AND release  (toggle feel)
//  Stick clicks (LABt,RABt) → beep on press only          (click feel)
//  Edge detection via static booleans that remember last state.
// ================================================================
void checkBeeps() {
  static bool pLBt  = false, pRBt  = false;
  static bool pLABt = false, pRABt = false;

  if ( data.LBt  && !pLBt)  beep();  // LBt just pressed
  if (!data.LBt  &&  pLBt)  beep();  // LBt just released
  if ( data.RBt  && !pRBt)  beep();  // RBt just pressed
  if (!data.RBt  &&  pRBt)  beep();  // RBt just released
  if ( data.LABt && !pLABt) beep();  // LABt just pressed
  if ( data.RABt && !pRABt) beep();  // RABt just pressed

  // Save states for next call
  pLBt  = data.LBt;
  pRBt  = data.RBt;
  pLABt = data.LABt;
  pRABt = data.RABt;
}

// ================================================================
//  ESP-NOW CALLBACK — runs in WiFi task context when ACK arrives
//  We only accept packets whose size matches AckData exactly.
// ================================================================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(AckData)) {
    AckData ack;
    memcpy(&ack, incomingData, sizeof(ack));
    if (ack.alive) {
      lastAckTime = millis();
      Stat = true;
    }
  }
}

// ================================================================
//  DataWork — read inputs, build packet, transmit
//  Called every loop tick; no blocking code inside.
// ================================================================
void DataWork() {
  // --- Joystick axes ---
  int newLx = mapAxis(analogRead(LX_PIN));
  int newLy = mapAxis(analogRead(LY_PIN));
  int newRx = mapAxis(analogRead(RX_PIN));
  int newRy = mapAxis(analogRead(RY_PIN));

  // Apply CHANGE_THRESHOLD: only commit new value if the axis moved enough.
  // This prevents tiny ADC fluctuations from constantly updating the packet.
  if (abs(newLx - data.Lx) > CHANGE_THRESHOLD) data.Lx = newLx;
  if (abs(newLy - data.Ly) > CHANGE_THRESHOLD) data.Ly = newLy;
  if (abs(newRx - data.Rx) > CHANGE_THRESHOLD) data.Rx = newRx;
  if (abs(newRy - data.Ry) > CHANGE_THRESHOLD) data.Ry = newRy;

  // --- Buttons (INPUT_PULLUP: LOW when pressed → invert with !) ---
  data.LABt = !digitalRead(L_STICK_PIN);
  data.RABt = !digitalRead(R_STICK_PIN);
  data.LBt  = !digitalRead(L_BTN_PIN);
  data.RBt  = !digitalRead(R_BTN_PIN);
  data.BAT  = BaT;

  checkBeeps();  // edge detection — must run after data is updated

  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

  Serial.printf("L(%+3d,%+3d) R(%+3d,%+3d) | LBt:%d RBt:%d LABt:%d RABt:%d | %.2fV %d%% | %s\n",
    data.Lx, data.Ly, data.Rx, data.Ry,
    (int)data.LBt, (int)data.RBt, (int)data.LABt, (int)data.RABt,
    batteryVolt, BaT,
    Stat ? "TELEMETRY Connected" : "TELEMETRY Lost!");
}

// ================================================================
//  updateDisplay — refresh the 20×4 LCD
//
//  Row 0 layout (20 chars):
//    "L +99 +99  R +99 +99"
//     ^         ^
//     L-stick   R-stick
//    "L "(2) + Lx(3) + " "(1) + Ly(3) + "  R "(4) + Rx(3) + " "(1) + Ry(3) = 20
//
//  Row 1 layout (20 chars):
//    "[d]    CONNECTED     [d]"  — direction arrow, link status, direction arrow
//    col 0   : left direction arrow (CGRAM) or '+' if centered
//    cols 1–4: 4 spaces
//    cols 5–13: 9-char status  →  "CONNECTED" or "-- NO LNK"
//    cols 14–18: 5 spaces
//    col 19  : right direction arrow (CGRAM) or '+' if centered
//    Total = 1 + 4 + 9 + 5 + 1 = 20
//
//  Row 2 layout (20 chars):
//    "LBt:0    100%  RBt:0"
//    "LBt:0"(5) + "    "(4) + "%3d%%"(4) + "  "(2) + "RBt:0"(5) = 20
//
//  Row 3 layout (20 chars):
//    "LABt:0 RABt:0 v0.9.2"
//    "LABt:0"(6) + " "(1) + "RABt:0"(6) + " "(1) + "v0.9.2"(6) = 20
//
//  Direction arrow: if centered → prints '+', else prints CGRAM char.
// ================================================================
void updateDisplay() {
  // ── Row 0: axis values ──────────────────────────────────────────
  // fmt() returns exactly 3 chars each (e.g. "+99", "  0", "-12")
  // "L " + Lx(3) + " " + Ly(3) + "  R " + Rx(3) + " " + Ry(3) = 20 chars
  char l0[21];  // 20 chars + null terminator
  snprintf(l0, 21, "L %s %s  R %s %s",
           fmt(data.Lx).c_str(),
           fmt(data.Ly).c_str(),
           fmt(data.Rx).c_str(),
           fmt(data.Ry).c_str());

  printIfChanged(0, 0, String(l0), line0);

  // ── Row 1: direction arrows + connection status ─────────────────
  // Write as individual segments to handle CGRAM char() calls cleanly.
  // Status string is always exactly 9 chars so column positions stay fixed.
  byte iconL = getDirIcon(data.Lx, data.Ly);
  byte iconR = getDirIcon(data.Rx, data.Ry);
  char iconLchar = (iconL == 255) ? '+' : (char)iconL;
  char iconRchar = (iconR == 255) ? '+' : (char)iconR;
  const char* statusStr = Stat ? "RX Linked" : " RX Lost!";

  // Build a comparison string using the proxy chars for CGRAM slots.
  // This lets us skip re-drawing row 1 if nothing has changed.
  char l1cmp[21];
  snprintf(l1cmp, 21, "%c    %s     %c",
           iconLchar, statusStr, iconRchar);
  // layout: icon(1) + "    "(4) + status(9) + "     "(5) + icon(1) = 20

  if (String(l1cmp) != line1) {
    line1 = String(l1cmp);

    // col 0: left direction arrow
    lcd.setCursor(0, 1);
    if (iconL == 255) lcd.print('+');    // centered → neutral cross
    else              lcd.write(iconL);  // directional CGRAM glyph

    // cols 1–4: 4 spaces (gap between arrow and status)
    lcd.print("    ");

    // cols 5–13: 9-char connection status string
    lcd.print(statusStr);

    // cols 14–18: 5 spaces (gap between status and arrow)
    lcd.print("     ");

    // col 19: right direction arrow
    if (iconR == 255) lcd.print('+');
    else              lcd.write(iconR);
  }

  // ── Row 2: push buttons + battery ──────────────────────────────
  // "LBt:0    100%  RBt:0" — exactly 20 chars
  // "LBt:X"(5) + "    "(4) + "%3d%%"(4, e.g. " 99%") + "  "(2) + "RBt:X"(5) = 20
  char l2[21];
  snprintf(l2, 21, "Arm:%d   %3d%%   RBt:%d",
           (int)data.LBt, BaT, (int)data.RBt);

  printIfChanged(0, 2, String(l2), line2);

  // ── Row 3: stick clicks + firmware version ──────────────────────
  // "LABt:0 RABt:0 v0.9.2" — exactly 20 chars
  // "LABt:X"(6) + " "(1) + "RABt:X"(6) + " "(1) + "v0.9.2"(6) = 20
  char l3[21];
  snprintf(l3, 21, "LStk:%d RStk:%d v0.9.2",
           (int)data.LABt, (int)data.RABt);

  printIfChanged(0, 3, String(l3), line3);
}

// ================================================================
//  updateConnectionLED
//  GREEN = robot sent an ACK within the last 500 ms → link is live
//  RED   = no ACK recently → link lost or robot is off
// ================================================================
void updateConnectionLED() {
  if (millis() - lastAckTime < 500) {
    Stat = true;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED,   LOW);
  } else {
    Stat = false;
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED,   HIGH);
  }
}

// ================================================================
//  BatteryManager — non-blocking battery read + low-bat safe mode
//
//  Normal operation:
//    Reads BAT_PIN every 5 seconds, calculates voltage and percent,
//    stores in global BaT and batteryVolt.
//
//  Low battery safe mode (batteryVolt ≤ LOW_BAT_THRESHOLD):
//    1. Sends a zeroed packet to kill the robot drive.
//    2. Blinks the red LED.
//    3. Overwrites the LCD with a LOW BAT warning (all 4 rows).
//    4. Returns true so the caller knows we're in fault state.
//
//  20×4 LOW BAT screen layout:
//  ┌────────────────────┐
//  │                    │  row 0  (blank)
//  │  ***  LOW  BAT  ***│  row 1  (warning banner)
//  │    Radio is OFF    │  row 2  (action taken)
//  │                    │  row 3  (blank)
//  └────────────────────┘
//
//  Returns: true if safe mode is active, false otherwise.
// ================================================================
bool BatteryManager() {
  static unsigned long lastRead  = 0;
  static unsigned long lastBlink = 0;
  static bool ledState  = false;
  static bool killSent  = false;

  // Read battery every 5 seconds (millis()-based, non-blocking)
  if (millis() - lastRead > 5000) {
    lastRead = millis();

    int   raw   = analogRead(BAT_PIN);
    float v_adc = raw * (3.3f / 4095.0f);

    // Reverse the voltage divider: V_bat = V_adc × (R1 + R2) / R2
    // R1 = 100kΩ, R2 = 47kΩ  →  factor = 147/47 ≈ 3.128
    batteryVolt = v_adc * ((100.0f + 47.0f) / 47.0f) * BAT_CAL_FACTOR;

    // Clamp to 0–100%
    if      (batteryVolt >= FULL_BAT)  BaT = 100;
    else if (batteryVolt <= EMPTY_BAT) BaT = 0;
    else    BaT = (int)((batteryVolt - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));

    Serial.printf("[BAT] %.2fV  %d%%\n", batteryVolt, BaT);
  }

  // ── Low battery safe mode ───────────────────────────────────────
  if (batteryVolt <= LOW_BAT_THRESHOLD) {
    if (!killSent) {
      memset(&data, 0, sizeof(data));   // zero all axes and buttons
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
      Serial.printf("[BAT] SAFE MODE %.2fV %d%% — radio KILLED\n", batteryVolt, BaT);
      killSent = true;
    }

    // Blink red LED at ~1.7 Hz (300 ms half-period)
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState  = !ledState;
      digitalWrite(RED_LED, ledState);
    }
    digitalWrite(GREEN_LED, LOW);

    // 20×4 LOW BAT screen — each row is exactly 20 chars
    lcd.setCursor(0, 0); lcd.print("                    ");   // blank row
    lcd.setCursor(0, 1); lcd.print("  ***  LOW  BAT  ***");   // warning banner
    lcd.setCursor(0, 2); lcd.print("    Radio is OFF    ");   // action message
    lcd.setCursor(0, 3); lcd.print("                    ");   // blank row

    return true;   // caller should not do normal work
  }

  killSent = false;   // reset so we re-send kill if battery dips again
  return false;
}

// ================================================================
//  SystemControl — wraps BatteryManager; handles recovery restart
//  If battery was low and has now recovered, re-run the startup
//  safety gate so the operator confirms before driving again.
// ================================================================
void SystemControl() {
  static bool wasLow = false;
  bool isLow = BatteryManager();

  if (isLow) {
    wasLow = true;
    // Spin here (still calling BatteryManager) until voltage recovers
    while (BatteryManager()) { delay(50); }
  }

  if (wasLow) {
    wasLow = false;
    startup();               // re-run safety gate after recovery
    line0 = line1 = line2 = line3 = "";   // invalidate LCD cache so it fully redraws
  }
}

// ================================================================
//  startup — blocking safety check before the main loop starts
//
//  Blocks until ALL of these are satisfied:
//    • LBt  = not pressed  (INPUT_PULLUP HIGH = safe)
//    • RBt  = not pressed
//    • Battery ≥ LOW_BAT_PERCENT %
//
//  20×4 screen while waiting:
//  ┌────────────────────┐
//  │===== Caution! =====│   row 0  (fixed throughout)
//  │  LBtn  : OK        │   row 1  "OK" or "HOLD BTN" (10-char field)
//  │  RBtn  : OK        │   row 2  "OK" or "HOLD BTN"
//  │  Batt  : OK ( 99%) │   row 3  "OK"/"LO" + percent
//  └────────────────────┘
//
//  Column layout for rows 1–2:
//    "  LBtn  : %-10s"  →  "  LBtn  : "(10) + padded-status(10) = 20
//    Status values: "OK        " or "HOLD BTN  "
//
//  Column layout for row 3:
//    "  Batt  : %s (%3d%%) "
//    "  Batt  : "(10) + "OK"/"LO"(2) + " ("(2) + pct(3) + "%)"(2) + " "(1) = 20
//
//  When all clear:
//  ┌────────────────────┐
//  │===== Caution! =====│
//  │                    │
//  │    ** ALL CLEAR ** │
//  │                    │
//  └────────────────────┘
//  Holds for 800 ms then returns.
// ================================================================
void startup() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("===== Caution! =====");   // exactly 20 chars
  Serial.println("[STARTUP] Safety check — release LBt, RBt and check battery");

  while (true) {
    // Read battery (no calibration factor here — just a rough startup check)
    int   rawBat = analogRead(BAT_PIN);
    float v_adc  = rawBat * (3.3f / 4095.0f);
    float batV   = v_adc * ((100.0f + 47.0f) / 47.0f);   // no CAL_FACTOR intentional
    int   batPct = 0;
    if      (batV >= FULL_BAT)  batPct = 100;
    else if (batV <= EMPTY_BAT) batPct = 0;
    else    batPct = (int)((batV - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));

    // digitalRead HIGH (not pressed on pullup) = safe
    bool lbtSafe = digitalRead(L_BTN_PIN);   // HIGH → button released = OK
    bool rbtSafe = digitalRead(R_BTN_PIN);
    bool batSafe = (batPct >= LOW_BAT_PERCENT);

    if (lbtSafe && rbtSafe && batSafe) {
      // ── ALL CLEAR ──
      lcd.setCursor(0, 1); lcd.print("                    ");   // 20 chars — blank
      lcd.setCursor(0, 2); lcd.print("  ** ALL CLEAR! **  ");   // 20 chars — centred
      lcd.setCursor(0, 3); lcd.print("====================");   // 20 chars — blank
      Serial.println("[STARTUP] All clear — proceeding");
      delay(1500);
      lcd.clear();
      return;
    }

    // ── Still waiting — show which checks are failing ──
    // Row 1: "  LBtn  : OK        " (20 chars)  or "  LBtn  : HOLD BTN  "
    // Row 2: "  RBtn  : OK        " (20 chars)  or "  RBtn  : HOLD BTN  "
    // Row 3: "  Batt  : OK ( 99%) " (20 chars)  or "  Batt  : LO ( 99%) "
    //
    // "  LBtn  : "(10 chars) + %-10s(10 chars) = 20
    // "HOLD BTN" left-justified in a 10-char field → "HOLD BTN  "
    char row1[21], row2[21], row3[21];
    snprintf(row1, 21, "  LBtn  : %-10s", lbtSafe ? "OK" : "Dis-ARM");
    snprintf(row2, 21, "  RBtn  : %-10s", rbtSafe ? "OK" : "Dis-ARM");
    // "  Batt  : "(10) + "OK"/"LO"(2) + " ("(2) + pct(3) + "%)"(2) + " "(1) = 20
    snprintf(row3, 21, "  Batt  : %s (%3d%%) ", batSafe ? "OK" : "LO", batPct);

    lcd.setCursor(0, 1); lcd.print(row1);
    lcd.setCursor(0, 2); lcd.print(row2);
    lcd.setCursor(0, 3); lcd.print(row3);

    delay(150);   // poll at ~6 Hz while waiting
  }
}

// ================================================================
//  SETUP — runs once at power-on
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n==============================");
  Serial.println("  URC TX v0.9.2 — Booting...");
  Serial.println("==============================");

  // ── GPIO ──
  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZ_PIN,  OUTPUT);
  digitalWrite(RED_LED,   HIGH);   // red on during boot
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BUZZ_PIN,  LOW);

  // Joystick click and push button inputs with internal pull-ups.
  // Active LOW: button pressed → pin reads LOW → !LOW = true.
  pinMode(L_STICK_PIN, INPUT_PULLUP);
  pinMode(R_STICK_PIN, INPUT_PULLUP);
  pinMode(L_BTN_PIN,   INPUT_PULLUP);
  pinMode(R_BTN_PIN,   INPUT_PULLUP);
  pinMode(BAT_PIN,     INPUT);        // GPIO36 is input-only, no mode needed

  // Full 0–3.3 V range for all ADC channels.
  // ADC_11db allows reading up to 3.3 V (default is 0–1 V which would clip battery sense).
  analogSetAttenuation(ADC_11db);

  // ── LCD init ──
  lcd.init();          // initialise I2C and the HD44780 controller
  lcd.backlight();     // turn on backlight (off by default after init)

  // Load custom direction arrow characters into CGRAM slots 0–7.
  // CGRAM survives lcd.clear() but not power cycling.
  lcd.createChar(0, UR_c);  // ↗
  lcd.createChar(1, U_c);   // ↑
  lcd.createChar(2, D_c);   // ↓
  lcd.createChar(3, L_c);   // ←
  lcd.createChar(4, R_c);   // →
  lcd.createChar(5, DL_c);  // ↙
  lcd.createChar(6, DR_c);  // ↘
  lcd.createChar(7, UL_c);  // ↖
  lcd.home();

  // ── Startup safety gate (blocks until buttons released + battery OK) ──
  startup();

  // ── Boot splash while radio initialises ──
  // Each string is exactly 20 chars for the 20×4 display.
  lcd.setCursor(0, 0); lcd.print("--------------------");
  lcd.setCursor(0, 1); lcd.print("  Radio Booting...  ");
  lcd.setCursor(0, 2); lcd.print("  URC-NOW    v0.9.2 ");
  lcd.setCursor(0, 3); lcd.print("--------------------");

  // Two-tone ascending beep — signals successful startup
  tone(BUZZ_PIN, 1000, 80); delay(130);
  tone(BUZZ_PIN, 1400, 80); delay(350);

  // ── WiFi + ESP-NOW init ──
  WiFi.mode(WIFI_STA);   // Station mode required for ESP-NOW; AP mode not used
  Serial.print("[INIT] TX MAC: ");
  Serial.println(WiFi.macAddress());
  delay(1000);

  if (esp_now_init() != ESP_OK) {
    // Fatal: blink red forever so the operator knows
    Serial.println("[ERROR] ESP-NOW init FAILED");
    while (1) { digitalWrite(RED_LED, !digitalRead(RED_LED)); delay(200); }
  }
  Serial.println("[INIT] ESP-NOW OK");

  esp_now_register_recv_cb(OnDataRecv);   // register ACK handler

  // Add robot as a peer — without this esp_now_send() will fail
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;       // 0 = same channel as WiFi
  peerInfo.encrypt = false;   // no encryption (faster, simpler)
  esp_now_add_peer(&peerInfo);

  Serial.println("[INIT] Peer added — entering main loop");
  lcd.clear();   // wipe boot splash before main loop starts
}

// ================================================================
//  MAIN LOOP — target ~50 Hz  (20 ms/tick)
//  No blocking code here — everything is non-blocking except
//  BatteryManager's internal while() in safe-mode (via SystemControl).
// ================================================================
void loop() {
  SystemControl();        // 1. battery watchdog — safe mode if low
  DataWork();             // 2. read sticks/buttons, beep edges, send packet
  updateDisplay();        // 3. refresh LCD (cached writes, minimal flicker)
  updateConnectionLED();  // 4. green/red LED based on ACK timeout
  delay(20);              // 5. ~50 Hz pacing — reduce to 10 for faster response
}