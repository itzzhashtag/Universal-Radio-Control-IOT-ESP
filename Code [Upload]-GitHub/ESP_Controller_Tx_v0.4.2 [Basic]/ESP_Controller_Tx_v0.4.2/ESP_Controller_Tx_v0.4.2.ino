/*
 * ============================================================
 *  URC TX v0.6 — Combat Robot Controller
 *  Board  : ESP32 Dev Board (30-pin, Wroom/Wroom-32)
 *  Core   : ESP32 Arduino Core 3.x
 *  Radio  : ESP-NOW (unicast to RX robot)
 *  Display: 16x2 I2C LCD via hd44780 library
 * ============================================================
 *
 *  ┌─────────────────────────────────────────────────────────┐
 *  │                   WIRING DIAGRAM                        │
 *  ├─────────────────────────────────────────────────────────┤
 *  │                                                         │
 *  │  ── JOYSTICKS ─────────────────────────────────────     │
 *  │  Left  Stick VRx → GPIO 34  (ADC1_CH6, input-only)     │
 *  │  Left  Stick VRy → GPIO 35  (ADC1_CH7, input-only)     │
 *  │  Left  Stick SW  → GPIO 25  (stick click, pullup)      │
 *  │  Right Stick VRx → GPIO 32  (ADC1_CH4)                 │
 *  │  Right Stick VRy → GPIO 33  (ADC1_CH5)                 │
 *  │  Right Stick SW  → GPIO 26  (stick click, pullup)      │
 *  │  All   Stick VCC → 3.3V                                 │
 *  │  All   Stick GND → GND                                  │
 *  │                                                         │
 *  │  ── BUTTONS ───────────────────────────────────────     │
 *  │  Left  Button    → GPIO 18  (one leg) → GND (other)    │
 *  │  Right Button    → GPIO 19  (one leg) → GND (other)    │
 *  │  (uses INPUT_PULLUP — no external resistor needed)      │
 *  │                                                         │
 *  │  ── STATUS LEDs ───────────────────────────────────     │
 *  │  Green LED anode → GPIO 27 → 220Ω resistor → GND      │
 *  │  Red   LED anode → GPIO 14 → 220Ω resistor → GND      │
 *  │                                                         │
 *  │  ── BATTERY VOLTAGE DIVIDER ───────────────────────     │
 *  │  LiPo+ ──┬── 100kΩ ──┬── 100kΩ ── GND                 │
 *  │           │           │                                 │
 *  │         (bat)       GPIO 36  (ADC1_CH0, input-only)    │
 *  │  Divides battery voltage by 2 → safe for 3.3V ADC      │
 *  │  4.2V battery → 2.1V at GPIO 36  ✓                     │
 *  │                                                         │
 *  │  ── 16x2 I2C LCD ──────────────────────────────────     │
 *  │  LCD VCC → 5V  (or 3.3V if your module supports it)    │
 *  │  LCD GND → GND                                          │
 *  │  LCD SDA → GPIO 21  (ESP32 default I2C SDA)            │
 *  │  LCD SCL → GPIO 22  (ESP32 default I2C SCL)            │
 *  │  I2C address: 0x27 (most common) — if LCD stays blank  │
 *  │               run I2C scanner, may be 0x3F             │
 *  │  hd44780 library auto-detects address — no code change  │
 *  │                                                         │
 *  │  ── POWER ─────────────────────────────────────────     │
 *  │  ESP32 powered via USB or 5V pin from a regulator       │
 *  │  Do NOT feed LiPo directly into ESP32 3.3V pin          │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  ── LIBRARIES NEEDED (install via Library Manager) ────────
 *  • hd44780  by Bill Perry   (replaces LiquidCrystal_I2C)
 *
 *  ── LCD DISPLAY LAYOUT ────────────────────────────────────
 *  Row 0:  L[LX][LY] R[RX][RY]
 *          "L-99+99 R+99-99"     (16 chars)
 *  Row 1:  LB  Ldir  BAT%  Rdir  RB
 *          "L:0 [d] 99% [d] R:0" (16 chars)
 *
 *  dir chars: C0=center  U0=up  D0=down  L0=left  R0=right
 *             UL UR DL DR = diagonals (loaded on demand)
 *
 *  ── SERIAL DEBUG OUTPUT (115200 baud) ─────────────────────
 *  Boot  : board MAC, ESP-NOW init status, peer add status
 *  Loop  : packet data every 500ms, link status changes,
 *          battery voltage + %, send fail count
 *
 *  ── FIRST TIME SETUP ──────────────────────────────────────
 *  1. Flash rx_robot.ino to the C3 mini
 *  2. Open Serial Monitor on C3 → copy the MAC address printed
 *  3. Paste it into RX_MAC[] below
 *  4. Flash this file to the 30-pin ESP32
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>   // I2C i/o expander backpack

// ╔══════════════════════════════════════════════════════════╗
// ║                     PIN DEFINITIONS                     ║
// ╚══════════════════════════════════════════════════════════╝

// ── Joystick axes — ADC1 only ──────────────────────────────
// ADC2 is unusable while ESP-NOW / WiFi is active on ESP32
#define PIN_LX    34   // Left  stick X  │ ADC1_CH6 │ input-only pin
#define PIN_LY    35   // Left  stick Y  │ ADC1_CH7 │ input-only pin
#define PIN_RX_A  32   // Right stick X  │ ADC1_CH4
#define PIN_RY    33   // Right stick Y  │ ADC1_CH5

// ── Joystick stick-click (SW pin) ──────────────────────────
#define PIN_LS    25   // Left  stick click │ INPUT_PULLUP, active LOW
#define PIN_RS    26   // Right stick click │ INPUT_PULLUP, active LOW

// ── Push buttons ───────────────────────────────────────────
#define PIN_LB    18   // Left  button │ INPUT_PULLUP, active LOW
#define PIN_RB    19   // Right button │ INPUT_PULLUP, active LOW

// ── Status LEDs ────────────────────────────────────────────
#define PIN_LED_G 27   // Green LED → connected
#define PIN_LED_R 14   // Red   LED → not connected / link lost

// ── Battery voltage divider ────────────────────────────────
#define PIN_BAT   36   // ADC1_CH0 │ input-only │ reads divided LiPo voltage
                       // Wiring: LiPo+ → 100kΩ → GPIO36 → 100kΩ → GND

// I2C uses ESP32 hardware defaults: SDA=21, SCL=22
// Wire.begin() with no arguments picks these up automatically

// ╔══════════════════════════════════════════════════════════╗
// ║                     CONFIGURATION                       ║
// ╚══════════════════════════════════════════════════════════╝

// >>> STEP 1: Flash rx_robot.ino, open Serial Monitor,
//             copy the MAC it prints, paste it here <<<
uint8_t RX_MAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

#define DEAD_ZONE        150   // ADC counts (0–4095) around center to ignore
                               // ESP32 ADC is noisier than C3 — 150 is safe
#define ACK_TIMEOUT_MS   500   // ms without ACK → declare link lost
#define SEND_INTERVAL_MS  20   // send packet every 20ms (~50 Hz)
#define DIR_THRESH       600   // stick direction threshold (30% of ±2047)
#define BAT_READ_INTERVAL 5000 // read battery voltage every 5 seconds

// Serial print interval for live data — keeps Serial readable
// without flooding at 50Hz
#define SERIAL_PRINT_INTERVAL 500   // print packet data every 500ms

// ╔══════════════════════════════════════════════════════════╗
// ║                   CONTROLLER DATA STRUCT                ║
// ║   Must be byte-identical on TX and RX — do not change   ║
// ║   field order or types without updating both sides      ║
// ╚══════════════════════════════════════════════════════════╝
typedef struct {
  int  LX, LY;   // Left  stick axes — range: -2047 .. +2047
  int  RX, RY;   // Right stick axes — range: -2047 .. +2047
  bool LS, RS;   // Stick click (SW pin) — true = pressed
  bool LB, RB;   // Push buttons        — true = pressed
} ControllerData;

ControllerData data;

// ╔══════════════════════════════════════════════════════════╗
// ║              LCD CUSTOM CHARACTER BITMAPS               ║
// ║  LCD CGRAM has 8 slots (0–7).                           ║
// ║  Slots 0–4: static cardinal directions, loaded once.    ║
// ║  Slots 5–6: diagonal directions, reloaded on demand     ║
// ║             when stick crosses a diagonal zone.         ║
// ║  Slot 7:    free for future use.                        ║
// ╚══════════════════════════════════════════════════════════╝
byte C0[8] = {B00000,B00000,B00000,B00100,B01110,B00100,B00000,B00000}; // ● center
byte U0[8] = {B00100,B01110,B10101,B00100,B00100,B00100,B00100,B00100}; // ↑ up
byte D0[8] = {B00100,B00100,B00100,B00100,B00100,B10101,B01110,B00100}; // ↓ down
byte L0[8] = {B00000,B00010,B00100,B01000,B11111,B01000,B00100,B00010}; // ← left
byte R0[8] = {B00000,B01000,B00100,B00010,B11111,B00010,B00100,B01000}; // → right
byte DL[8] = {B00000,B00000,B00001,B10010,B10100,B11000,B11110,B00000}; // ↙ diag
byte DR[8] = {B00000,B00000,B10000,B01001,B00101,B00011,B01111,B00000}; // ↘ diag
byte UR[8] = {B00000,B00000,B01111,B00011,B00101,B01001,B10000,B00000}; // ↗ diag
byte UL[8] = {B00000,B00000,B11110,B11000,B10100,B10010,B00001,B00000}; // ↖ diag

// CGRAM slot assignments
#define SLOT_C0  0
#define SLOT_U0  1
#define SLOT_D0  2
#define SLOT_L0  3
#define SLOT_R0  4
#define SLOT_D5  5   // diagonal slot — left  stick (reloaded on direction change)
#define SLOT_D6  6   // diagonal slot — right stick (reloaded on direction change)
//      slot  7      // free

// Diagonal IDs — unique ints used to detect when a slot needs reloading
#define DIAG_UL 10
#define DIAG_UR 11
#define DIAG_DL 12
#define DIAG_DR 13

// ╔══════════════════════════════════════════════════════════╗
// ║                       GLOBALS                           ║
// ╚══════════════════════════════════════════════════════════╝
hd44780_I2Cexp lcd;              // auto-detects I2C address (0x27 / 0x3F)

bool          connected    = false;
unsigned long lastAck      = 0;
unsigned long lastSend     = 0;
unsigned long lastBatRead  = 0;
unsigned long lastSerialPrint = 0;
int           batPercent   = 0;

// Send fail counter — increments when ESP-NOW delivery fails
// Resets on each successful ACK — useful to spot range/interference
unsigned long sendFailCount = 0;
unsigned long sendTotalCount = 0;

// Track which diagonal is currently loaded in each dynamic slot.
// -1 = nothing loaded yet → forces a write on first use.
int loadedD5 = -1;
int loadedD6 = -1;

// ╔══════════════════════════════════════════════════════════╗
// ║           ESP-NOW SEND CALLBACK  (Core 3.x)             ║
// ║                                                         ║
// ║  Core 2.x signature:  const uint8_t *mac_addr          ║
// ║  Core 3.x signature:  const wifi_tx_info_t *info  ←    ║
// ║                                                         ║
// ║  Using the old signature causes a compile error on      ║
// ║  Core 3.x — this is the correct version.               ║
// ╚══════════════════════════════════════════════════════════╝
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  sendTotalCount++;
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastAck = millis();   // reset watchdog timer on every confirmed delivery
    // Serial feedback for send success is intentionally omitted here
    // — it fires at 50Hz and would flood the monitor.
    // Use the 500ms print in loop() to track totals instead.
  } else {
    sendFailCount++;
    // Print immediately on failure — failures are rare and worth seeing instantly
    Serial.print("[ESP-NOW] Send FAILED. Total fails: ");
    Serial.print(sendFailCount);
    Serial.print(" / Total sent: ");
    Serial.println(sendTotalCount);
  }
}

// ╔══════════════════════════════════════════════════════════╗
// ║                     HELPER FUNCTIONS                    ║
// ╚══════════════════════════════════════════════════════════╝

/*
 * mapStick()
 * Converts raw ADC reading (0–4095) to signed value (-2047..+2047)
 * with a dead-zone applied around center (≈2048 counts).
 * Dead-zone prevents stick drift when physically at rest.
 */
int mapStick(int raw) {
  int val = raw - 2048;
  if (abs(val) < DEAD_ZONE) val = 0;
  return val;
}

/*
 * loadBaseChars()
 * Writes the 5 static direction bitmaps into CGRAM slots 0–4.
 * Called once in setup(). Diagonal slots 5 & 6 are lazy-loaded
 * by dirChar() only when a diagonal direction is first detected.
 */
void loadBaseChars() {
  lcd.createChar(SLOT_C0, C0);
  lcd.createChar(SLOT_U0, U0);
  lcd.createChar(SLOT_D0, D0);
  lcd.createChar(SLOT_L0, L0);
  lcd.createChar(SLOT_R0, R0);
}

/*
 * dirChar()
 * Given a stick's x/y in -2047..+2047 space, returns the CGRAM
 * slot number of the correct directional glyph to display.
 *
 * For cardinal directions, returns a fixed slot (no CGRAM write).
 * For diagonals, lazy-loads the bitmap into slot 5 (left stick)
 * or slot 6 (right stick) only when the direction has changed —
 * avoids unnecessary CGRAM writes every frame (~50Hz).
 */
uint8_t dirChar(int x, int y, bool isLeft) {
  bool up    = (y < -DIR_THRESH);
  bool down  = (y >  DIR_THRESH);
  bool left  = (x < -DIR_THRESH);
  bool right = (x >  DIR_THRESH);

  // Cardinal directions — fixed slots, never need reloading
  if (!up && !down && !left && !right) return SLOT_C0;
  if ( up   && !left && !right)        return SLOT_U0;
  if ( down && !left && !right)        return SLOT_D0;
  if ( left && !up   && !down)         return SLOT_L0;
  if ( right && !up  && !down)         return SLOT_R0;

  // Diagonal — pick correct bitmap and assign a unique ID
  byte* bmap   = nullptr;
  int   diagId = -1;
  if ( up   &&  left)  { bmap = UL; diagId = DIAG_UL; }
  if ( up   && right)  { bmap = UR; diagId = DIAG_UR; }
  if ( down &&  left)  { bmap = DL; diagId = DIAG_DL; }
  if ( down && right)  { bmap = DR; diagId = DIAG_DR; }

  // Left stick → slot 5 │ Right stick → slot 6
  int  slot   = isLeft ? SLOT_D5 : SLOT_D6;
  int& loaded = isLeft ? loadedD5 : loadedD6;

  // Only write to CGRAM if direction actually changed — saves bus time
  if (loaded != diagId) {
    lcd.createChar(slot, bmap);
    loaded = diagId;
  }
  return slot;
}

/*
 * readBattery()
 * Reads the voltage divider on GPIO 36 and converts to a 0–99%
 * charge estimate using the LiPo discharge curve.
 *
 * Voltage divider (equal resistors) halves the battery voltage:
 *   4.2V battery → 2.1V at ADC pin  ✓  (within 3.3V range)
 *   3.2V battery → 1.6V at ADC pin  ✓
 *
 * 16-sample average reduces ESP32 ADC noise.
 * +150mV software correction compensates for typical ESP32 ADC
 * non-linearity in the mid-range. Calibrate against a multimeter
 * if you need better accuracy — adjust the offset value.
 *
 * LiPo usable range mapped: 3200mV (0%) → 4200mV (99%)
 */
int readBattery() {
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(PIN_BAT);
    delay(1);
  }
  int raw = (int)(sum / 16);

  // ADC counts → millivolts at the pin, with non-linearity correction
  float adcMv  = (raw / 4095.0f) * 3300.0f + 150.0f;

  // Reconstruct actual battery voltage (×2 for equal-resistor divider)
  float batMv  = adcMv * 2.0f;

  // Map 3200–4200 mV → 0–99%
  int pct = (int)((batMv - 3200.0f) / (4200.0f - 3200.0f) * 99.0f);
  pct = constrain(pct, 0, 99);

  // Serial: print raw ADC, reconstructed voltage and percentage
  Serial.println("----------------------------------------");
  Serial.print("[BAT] Raw ADC avg : "); Serial.println(raw);
  Serial.print("[BAT] ADC pin mV  : "); Serial.println(adcMv, 1);
  Serial.print("[BAT] Battery mV  : "); Serial.println(batMv, 1);
  Serial.print("[BAT] Charge      : "); Serial.print(pct); Serial.println("%");
  Serial.println("----------------------------------------");

  return pct;
}

/*
 * updateLCD()
 *
 * Row 0:  L[LX][LY] R[RX][RY]
 *         "L-99+99 R+99-99"     (16 chars)
 *
 * Row 1:  LB  Ldir  BAT%  Rdir  RB
 *         "L:0 [d] 99% [d] R:0" (16 chars)
 *
 *  Col:  0123456789012345
 *  Row0: L-99+99 R+99-99
 *  Row1: L:0 [d] 99% [d] R:0
 *        └─┘  │   └─┘  │  └─┘
 *        LB   │   bat   │  RB
 *           Ldir      Rdir
 *
 * LB / RB = 0 (released) or 1 (pressed)
 * [d] = direction glyph from CGRAM (C0/U0/D0/L0/R0/diagonals)
 * Axis values: zero shows "  0" (no sign), nonzero shows "+NN"/"-NN"
 */
void updateLCD() {
  auto scl = [](int v) -> int { return v * 99 / 2047; };

  int lx = scl(data.LX), ly = scl(data.LY);
  int rx = scl(data.RX), ry = scl(data.RY);

  uint8_t lDir = dirChar(data.LX, data.LY, true);
  uint8_t rDir = dirChar(data.RX, data.RY, false);

  char buf[6];

  // ── Row 0: LX LY | RX RY ─────────────────────────────────
  // fmtAxis: zero → "  0" (no sign), nonzero → "+99" or "-99"
  // Writes exactly 3 chars into dst (plus null terminator)
  auto fmtAxis = [](char* dst, int v) {
    if (v == 0) snprintf(dst, 4, "  0");
    else        snprintf(dst, 4, "%+3d", v);
  };

  char sLX[4], sLY[4], sRX[4], sRY[4];
  fmtAxis(sLX, lx);
  fmtAxis(sLY, ly);
  fmtAxis(sRX, rx);
  fmtAxis(sRY, ry);

  // "L-99+99 R+99-99" — 16 chars exactly
  char row0[17];
  snprintf(row0, sizeof(row0), "L%s%s R%s%s", sLX, sLY, sRX, sRY);
  lcd.setCursor(0, 0);
  lcd.print(row0);

  // ── Row 1: LB | Ldir | BAT | Rdir | RB ──────────────────
  // Col 0-3:  "L:0 " — left button state + space
  // Col 4:    Ldir glyph
  // Col 5-8:  "  99%" — battery (space + 2 digits + %)
  // Col 9:    space
  // Col 10:   Rdir glyph
  // Col 11-15: " R:0" — space + right button state
  lcd.setCursor(0, 1);

  snprintf(buf, sizeof(buf), "L:%d ", data.LB ? 1 : 0);
  lcd.print(buf);                        // cols 0-3

  lcd.write(lDir);                       // col  4

  snprintf(buf, sizeof(buf), " %2d%% ", batPercent);
  lcd.print(buf);                        // cols 5-9

  lcd.write(rDir);                       // col  10

  snprintf(buf, sizeof(buf), " R:%d", data.RB ? 1 : 0);
  lcd.print(buf);                        // cols 11-15
}

/*
 * printSerialData()
 * Prints a formatted snapshot of all controller values to Serial.
 * Called every SERIAL_PRINT_INTERVAL ms — not every loop tick.
 * Keeping it periodic avoids flooding at 50Hz send rate.
 */
/*
 * printSerialData()
 * Single-line serial output — easy to read in Serial Monitor / Plotter
 *
 * Format:
 * [TX→OUT] L( LX, LY) R( RX, RY) | LA:0 RA:0 LB:0 RB:0 | BAT: 4.20V 99% | Link: OK
 *           └─ scaled ±99 ──────┘   └─ stick clicks ─┘ └─ push buttons ─┘
 *
 * LA = Left  stick click (LS)    LB = Left  push button
 * RA = Right stick click (RS)    RB = Right push button
 * Link: OK = connected           Link: Dead = no ACK for 500ms
 */
void printSerialData() {
  auto scl = [](int v) -> int { return v * 99 / 2047; };

  // Reconstruct battery voltage from last raw read for display
  // Same math as readBattery() — recalculated here for the serial line
  long sum = 0;
  for (int i = 0; i < 4; i++) {   // quick 4-sample read for serial only
    sum += analogRead(PIN_BAT);
  }
  float adcMv = (sum / 4.0f / 4095.0f) * 3300.0f + 150.0f;
  float batV  = (adcMv * 2.0f) / 1000.0f;   // convert mV → V

  Serial.printf("[TX→OUT] L(%+3d,%+3d) R(%+3d,%+3d) | LA:%d RA:%d | LB:%d RB:%d | BAT:%5.2fV %2d%% | Link: %s\n",
    scl(data.LX), scl(data.LY),
    scl(data.RX), scl(data.RY),
    data.LS ? 1 : 0,
    data.RS ? 1 : 0,
    data.LB ? 1 : 0,
    data.RB ? 1 : 0,
    batV,
    batPercent,
    connected ? "OK" : "Dead"
  );
}

// ╔══════════════════════════════════════════════════════════╗
// ║                         SETUP                           ║
// ╚══════════════════════════════════════════════════════════╝
void setup() {
  Serial.begin(115200);
  delay(500);   // let Serial settle before printing

  Serial.println("\n\n========================================");
  Serial.println("   URC TX v0.6 — Combat Robot Controller");
  Serial.println("========================================");

  // ── GPIO setup ────────────────────────────────────────────
  Serial.println("[INIT] Setting up GPIO...");
  pinMode(PIN_LS,    INPUT_PULLUP);   // left  stick click
  pinMode(PIN_RS,    INPUT_PULLUP);   // right stick click
  pinMode(PIN_LB,    INPUT_PULLUP);   // left  push button
  pinMode(PIN_RB,    INPUT_PULLUP);   // right push button
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);      // red on  = not connected (startup default)
  digitalWrite(PIN_LED_G, LOW);
  Serial.println("[INIT] GPIO OK");

  // ── LCD init ──────────────────────────────────────────────
  Serial.println("[INIT] Starting LCD (I2C SDA=21 SCL=22)...");
  Wire.begin();
  int lcdStatus = lcd.begin(16, 2);
  if (lcdStatus) {
    // lcd.begin() returns non-zero on any failure
    Serial.print("[ERROR] LCD init FAILED — hd44780 status code: ");
    Serial.println(lcdStatus);
    Serial.println("[ERROR] Check wiring: SDA→GPIO21, SCL→GPIO22, VCC→5V");
    Serial.println("[ERROR] Run I2C scanner to confirm address (0x27 or 0x3F)");
    while (1) {
      digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R));
      delay(150);   // fast blink = hardware fault
    }
  }
  Serial.println("[INIT] LCD OK — auto-detected I2C address");
  lcd.backlight();
  loadBaseChars();
  lcd.setCursor(0, 0); lcd.print("  Combat Robot  ");
  lcd.setCursor(0, 1); lcd.print("  Connecting... ");

  // ── ESP-NOW init ──────────────────────────────────────────
  Serial.println("[INIT] Starting WiFi in STA mode...");
  WiFi.mode(WIFI_STA);

  // Print this board's MAC — useful for the RX to know TX identity
  Serial.print("[INIT] TX MAC Address : ");
  Serial.println(WiFi.macAddress());

  // Print the target RX MAC so you can verify it matches
  Serial.print("[INIT] Target RX MAC  : ");
  for (int i = 0; i < 6; i++) {
    if (RX_MAC[i] < 0x10) Serial.print("0");
    Serial.print(RX_MAC[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  Serial.println("[INIT] Initialising ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAILED");
    lcd.setCursor(0, 1); lcd.print(" ESP-NOW  FAIL! ");
    while (1);
  }
  Serial.println("[INIT] ESP-NOW OK");

  // Register send callback — Core 3.x signature (wifi_tx_info_t)
  esp_now_register_send_cb(onDataSent);
  Serial.println("[INIT] Send callback registered (Core 3.x signature)");

  // Register the robot receiver as a peer
  Serial.println("[INIT] Adding RX as ESP-NOW peer...");
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RX_MAC, 6);
  peer.channel = 0;       // 0 = match current channel automatically
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ERROR] Failed to add ESP-NOW peer");
    Serial.println("[ERROR] Double-check RX_MAC matches RX Serial output");
    lcd.setCursor(0, 1); lcd.print("  PEER FAIL!    ");
    while (1);
  }
  Serial.println("[INIT] Peer added OK");

  // Initial battery read so display shows a real value immediately
  Serial.println("[INIT] Reading battery...");
  batPercent  = readBattery();
  lastBatRead = millis();

  Serial.println("[INIT] ── Setup complete. Entering loop. ──");
  Serial.println("  Waiting for first ACK from RX...");
  Serial.println("  Serial snapshot prints every 500ms.");
  Serial.println("========================================\n");

  delay(1200);
  lcd.clear();
}

// ╔══════════════════════════════════════════════════════════╗
// ║                          LOOP                           ║
// ╚══════════════════════════════════════════════════════════╝
void loop() {
  unsigned long now = millis();

  // ── Read joystick axes (ADC1 pins only) ─────────────────
  data.LX = mapStick(analogRead(PIN_LX));
  data.LY = mapStick(analogRead(PIN_LY));
  data.RX = mapStick(analogRead(PIN_RX_A));
  data.RY = mapStick(analogRead(PIN_RY));

  // ── Read stick clicks and push buttons (active LOW) ──────
  data.LS = !digitalRead(PIN_LS);
  data.RS = !digitalRead(PIN_RS);
  data.LB = !digitalRead(PIN_LB);
  data.RB = !digitalRead(PIN_RB);

  // ── Send packet at ~50 Hz ────────────────────────────────
  if (now - lastSend >= SEND_INTERVAL_MS) {
    esp_now_send(RX_MAC, (uint8_t*)&data, sizeof(data));
    lastSend = now;
  }

  // ── Battery read every 5 seconds ─────────────────────────
  if (now - lastBatRead >= BAT_READ_INTERVAL) {
    batPercent  = readBattery();
    lastBatRead = now;
  }

  // ── Connection watchdog ──────────────────────────────────
  // ACK arrives via onDataSent callback → updates lastAck.
  // If no ACK for ACK_TIMEOUT_MS ms, link is considered lost.
  bool wasConnected = connected;
  connected = (now - lastAck < ACK_TIMEOUT_MS);

  if (connected != wasConnected) {
    // State change — print immediately, don't wait for 500ms interval
    if (connected) {
      Serial.println("\n[LINK] ✓ CONNECTED to RX robot!");
      Serial.print("[LINK]   RX MAC: ");
      for (int i = 0; i < 6; i++) {
        if (RX_MAC[i] < 0x10) Serial.print("0");
        Serial.print(RX_MAC[i], HEX);
        if (i < 5) Serial.print(":");
      }
      Serial.println();
    } else {
      Serial.println("\n[LINK] ✗ LINK LOST — no ACK for 500ms");
      Serial.print("[LINK]   Last ACK was "); Serial.print(now - lastAck);
      Serial.println("ms ago");
    }
    digitalWrite(PIN_LED_G, connected ? HIGH : LOW);
    digitalWrite(PIN_LED_R, connected ? LOW  : HIGH);
  }

  // ── Periodic serial data snapshot ────────────────────────
  if (now - lastSerialPrint >= SERIAL_PRINT_INTERVAL) {
    printSerialData();
    lastSerialPrint = now;
  }

  // ── Update display ───────────────────────────────────────
  updateLCD();
}
