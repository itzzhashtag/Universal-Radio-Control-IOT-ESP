/*
 * ============================================================
 *  URC TX v0.4.2 — Combat Robot Controller
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
 *  │  Left Stick VRx  → GPIO 34  (ADC1_CH6, input-only)     │
 *  │  Left Stick VRy  → GPIO 35  (ADC1_CH7, input-only)     │
 *  │  Right Stick VRx → GPIO 32  (ADC1_CH4)                 │
 *  │  Right Stick VRy → GPIO 33  (ADC1_CH5)                 │
 *  │  All  stick VCC  → 3.3V                                 │
 *  │  All  stick GND  → GND                                  │
 *  │  Stick SW pins   → not connected (no click switch used) │
 *  │                                                         │
 *  │  ── BUTTONS ───────────────────────────────────────     │
 *  │  Left  Button    → GPIO 25  (one leg) → GND (other)    │
 *  │  Right Button    → GPIO 26  (one leg) → GND (other)    │
 *  │  (uses INPUT_PULLUP — no external resistor needed)      │
 *  │                                                         │
 *  │  ── STATUS LEDs ───────────────────────────────────     │
 *  │  Green LED anode → GPIO 27 → 220Ω resistor → GND      │
 *  │  Red   LED anode → GPIO 14 → 220Ω resistor → GND      │
 *  │                                                         │
 *  │  ── 16x2 I2C LCD ──────────────────────────────────     │
 *  │  LCD VCC → 5V  (or 3.3V if your module supports it)    │
 *  │  LCD GND → GND                                          │
 *  │  LCD SDA → GPIO 21  (ESP32 default I2C SDA)            │
 *  │  LCD SCL → GPIO 22  (ESP32 default I2C SCL)            │
 *  │  I2C address: 0x27 (most common) — check with          │
 *  │               I2C scanner if LCD stays blank            │
 *  │                                                         │
 *  │  ── POWER ─────────────────────────────────────────     │
 *  │  ESP32 powered via USB or 5V pin from a regulator       │
 *  │  Do NOT power ESP32 3.3V pin directly from LiPo         │
 *  └─────────────────────────────────────────────────────────┘
 *
 *  ── LIBRARIES NEEDED (install via Library Manager) ────────
 *  • hd44780  by Bill Perry   (replaces LiquidCrystal_I2C)
 *
 *  ── LCD DISPLAY LAYOUT ────────────────────────────────────
 *  Row 0:  L[dir]+NNN  R[dir]+NNN  [C/X]
 *          └─left──┘  └─right──┘   └connection
 *  Row 1:  Y+NNN [B]  Y+NNN [B]
 *          └─LY──┘LB  └─RY──┘RB
 *
 *  dir chars: C0=center  U0=up  D0=down  L0=left  R0=right
 *             UL UR DL DR = diagonals (loaded on demand)
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

// ADC1 pins only — ADC2 is unusable while ESP-NOW/WiFi is active
#define PIN_LX    34   // Left  stick X  │ ADC1_CH6 │ input-only pin
#define PIN_LY    35   // Left  stick Y  │ ADC1_CH7 │ input-only pin
#define PIN_RX_A  32   // Right stick X  │ ADC1_CH4
#define PIN_RY    33   // Right stick Y  │ ADC1_CH5

#define PIN_LB    25   // Left  button   │ INPUT_PULLUP, active LOW
#define PIN_RB    26   // Right button   │ INPUT_PULLUP, active LOW

#define PIN_LED_G 27   // Green LED  → connected
#define PIN_LED_R 14   // Red   LED  → not connected / lost link
// ── ADD to pin definitions ──────────────────────────────────
#define PIN_LS    18   // Left  stick click (SW pin) → INPUT_PULLUP
#define PIN_RS    19   // Right stick click (SW pin) → INPUT_PULLUP
// I2C uses ESP32 hardware defaults: SDA=21, SCL=22
// Wire.begin() with no args picks these up automatically

// ╔══════════════════════════════════════════════════════════╗
// ║                     CONFIGURATION                       ║
// ╚══════════════════════════════════════════════════════════╝

// >>> STEP 1: Flash rx_robot.ino, open Serial Monitor,
//             copy the MAC it prints, paste it here <
uint8_t RX_MAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

#define DEAD_ZONE       150    // ADC units (0–4095) around center to ignore
                               // ESP32 ADC is noisier than C3 — 150 is safe
#define ACK_TIMEOUT_MS  500    // ms without ACK → declare link lost
#define SEND_INTERVAL_MS 20    // send packet every 20ms (~50 Hz)

// Joystick direction threshold — 30% of half-range (2047)
#define DIR_THRESH      600

// ╔══════════════════════════════════════════════════════════╗
// ║                   CONTROLLER DATA STRUCT                ║
// ║   Must be byte-identical on TX and RX                   ║
// ╚══════════════════════════════════════════════════════════╝
typedef struct {
  int  LX, LY;   // Left  stick  — range: -2047 .. +2047
  int  RX, RY;   // Right stick  — range: -2047 .. +2047
  bool LS, RS;   // Stick click  — not wired, always false
  bool LB, RB;   // Buttons      — true = pressed
} ControllerData;

ControllerData data;

// ╔══════════════════════════════════════════════════════════╗
// ║              LCD CUSTOM CHARACTER BITMAPS               ║
// ║  8 slots available (0–7). Slots 5 & 6 are reused for    ║
// ║  diagonals and reloaded on demand when direction changes ║
// ╚══════════════════════════════════════════════════════════╝
byte C0[8] = {B00000,B00000,B00000,B00100,B01110,B00100,B00000,B00000}; // center dot
byte U0[8] = {B00100,B01110,B10101,B00100,B00100,B00100,B00100,B00100}; // up arrow
byte D0[8] = {B00100,B00100,B00100,B00100,B00100,B10101,B01110,B00100}; // down arrow
byte L0[8] = {B00000,B00010,B00100,B01000,B11111,B01000,B00100,B00010}; // left arrow
byte R0[8] = {B00000,B01000,B00100,B00010,B11111,B00010,B00100,B01000}; // right arrow
byte DL[8] = {B00000,B00000,B00001,B10010,B10100,B11000,B11110,B00000}; // diagonal ↙
byte DR[8] = {B00000,B00000,B10000,B01001,B00101,B00011,B01111,B00000}; // diagonal ↘
byte UR[8] = {B00000,B00000,B01111,B00011,B00101,B01001,B10000,B00000}; // diagonal ↗
byte UL[8] = {B00000,B00000,B11110,B11000,B10100,B10010,B00001,B00000}; // diagonal ↖

// Custom char slot assignments
#define SLOT_C0  0
#define SLOT_U0  1
#define SLOT_D0  2
#define SLOT_L0  3
#define SLOT_R0  4
#define SLOT_D5  5   // diagonal for LEFT  stick (reloaded on change)
#define SLOT_D6  6   // diagonal for RIGHT stick (reloaded on change)
// slot 7 = free

// ╔══════════════════════════════════════════════════════════╗
// ║                       GLOBALS                           ║
// ╚══════════════════════════════════════════════════════════╝
hd44780_I2Cexp lcd;              // auto-detects I2C address (0x27 / 0x3F)

bool          connected   = false;
unsigned long lastAck     = 0;
unsigned long lastSend    = 0;

// Track which diagonal bitmap is loaded in each slot
// -1 means "nothing loaded yet" — forces first load
int loadedD5 = -1;
int loadedD6 = -1;

// Diagonal IDs — arbitrary unique ints to detect change
#define DIAG_UL 10
#define DIAG_UR 11
#define DIAG_DL 12
#define DIAG_DR 13
// ── ADD to pin definitions ──────────────────────────────────
#define PIN_BAT   36   // Battery voltage divider output
                       // Wiring: LiPo+ → 100kΩ → GPIO36 → 100kΩ → GND

// ── ADD to globals ──────────────────────────────────────────
int  batPercent   = 0;
unsigned long lastBatRead = 0;
#define BAT_READ_INTERVAL 5000   // read battery every 5 seconds
                                 // ADC is noisy — no need to read every loop

// ── ADD this function ───────────────────────────────────────
/*
 * readBattery()
 * Reads voltage divider on GPIO 36, reconstructs actual LiPo voltage,
 * maps to 0–99% using a simplified LiPo discharge curve.
 *
 * Voltage divider: equal resistors → multiply ADC voltage × 2
 * ADC reference: 3.3V at 4095 counts
 *
 * LiPo usable range: 3.2V (0%) → 4.2V (100%)
 * Clamped to 0–99 for 2-digit display fit.
 */
int readBattery() {
  // Average 16 samples to reduce ADC noise
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(PIN_BAT);
    delay(1);
  }
  int raw = sum / 16;

  // Convert raw → millivolts at the ADC pin
  // ESP32 ADC is non-linear — add ~150mV correction for typical error
  float adcVoltage = (raw / 4095.0f) * 3300.0f + 150.0f;  // mV

  // Reconstruct actual battery voltage (×2 for divider)
  float batVoltage = adcVoltage * 2.0f;  // mV

  // Map 3200mV–4200mV → 0–99%
  int pct = (int)((batVoltage - 3200.0f) / (4200.0f - 3200.0f) * 99.0f);
  return constrain(pct, 0, 99);
}
// ╔══════════════════════════════════════════════════════════╗
// ║               ESP-NOW SEND CALLBACK (Core 3.x)          ║
// ║  Old Core 2.x used:  const uint8_t *mac_addr            ║
// ║  Core 3.x requires:  const wifi_tx_info_t *info         ║
// ╚══════════════════════════════════════════════════════════╝
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastAck = millis();   // reset watchdog on every successful ACK
  }
}

// ╔══════════════════════════════════════════════════════════╗
// ║                     HELPER FUNCTIONS                    ║
// ╚══════════════════════════════════════════════════════════╝

/*
 * mapStick()
 * Converts raw ADC reading (0–4095) to signed value (-2047..+2047)
 * with dead-zone applied around center.
 * Center of joystick pot ≈ 2048 ADC counts.
 */
int mapStick(int raw) {
  int val = raw - 2048;
  if (abs(val) < DEAD_ZONE) val = 0;
  return val;
}

/*
 * loadBaseChars()
 * Writes the 5 static direction chars into LCD CGRAM slots 0–4.
 * Called once in setup(). Diagonal slots (5,6) are lazy-loaded.
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
 * Given a stick's (x, y) in -2047..+2047 space, returns the
 * LCD slot number of the correct direction glyph to display.
 *
 * For diagonals, it lazy-loads the bitmap into slot 5 (left stick)
 * or slot 6 (right stick) only when the direction has changed —
 * avoids unnecessary CGRAM writes every frame.
 *
 * isLeft = true  → uses SLOT_D5
 * isLeft = false → uses SLOT_D6
 */
uint8_t dirChar(int x, int y, bool isLeft) {
  bool up    = (y < -DIR_THRESH);
  bool down  = (y >  DIR_THRESH);
  bool left  = (x < -DIR_THRESH);
  bool right = (x >  DIR_THRESH);

  // Cardinal directions — static slots, no reload needed
  if (!up && !down && !left && !right) return SLOT_C0;
  if ( up   && !left && !right)        return SLOT_U0;
  if ( down && !left && !right)        return SLOT_D0;
  if ( left && !up   && !down)         return SLOT_L0;
  if ( right && !up  && !down)         return SLOT_R0;

  // Diagonals — determine which bitmap + assign a unique ID
  byte* bmap  = nullptr;
  int   diagId = -1;

  if ( up   &&  left)  { bmap = UL; diagId = DIAG_UL; }
  if ( up   && right)  { bmap = UR; diagId = DIAG_UR; }
  if ( down &&  left)  { bmap = DL; diagId = DIAG_DL; }
  if ( down && right)  { bmap = DR; diagId = DIAG_DR; }

  int  slot   = isLeft ? SLOT_D5 : SLOT_D6;
  int& loaded = isLeft ? loadedD5 : loadedD6;

  // Only write to CGRAM if the direction actually changed
  if (loaded != diagId) {
    lcd.createChar(slot, bmap);
    loaded = diagId;
  }
  return slot;
}

/*
 * updateLCD()
 * Redraws both rows of the display every loop iteration.
 *
 * Row 0:  L[dir]+NNN  R[dir]+NNN  [C/X]
 * Row 1:  Y+NNN [B]   Y+NNN [B]
 *
 * Values are scaled from ±2047 → ±99 for compact 3-digit display.
 * 'B' appears at button positions when pressed, space when not.
 * 'C' (connected) or 'X' (no link) shown at col 15, row 0.
 */
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
 *        L:0 [d] 99% [d] R:0
 *        └─┘  │   └─┘  │  └─┘
 *        LB   │   bat   │  RB
 *           Ldir      Rdir
 *
 * LB / RB = 0 (released) or 1 (pressed)
 * [d] = direction glyph from CGRAM (C0/U0/D0/L0/R0/diagonals)
 * Battery centered, 3 chars wide "99%" or " 5%"
 */
void updateLCD() {
  auto scl = [](int v) -> int { return v * 99 / 2047; };

  int lx = scl(data.LX), ly = scl(data.LY);
  int rx = scl(data.RX), ry = scl(data.RY);

  uint8_t lDir = dirChar(data.LX, data.LY, true);
  uint8_t rDir = dirChar(data.RX, data.RY, false);

  char buf[6];

  // ── Row 0: LX LY | RX RY ─────────────────────────────────
  // "L-99+99 R+99-99"  — 16 chars exactly
  // snprintf into a 17-byte buffer (16 chars + null)
  char row0[17];
  snprintf(row0, sizeof(row0), "L%+3d%+3d R%+3d%+3d",
           lx, ly, rx, ry);
  lcd.setCursor(0, 0);
  lcd.print(row0);

  // ── Row 1: LB | Ldir | BAT | Rdir | RB ──────────────────
  // Col 0-2:  "L:0" or "L:1"  — left button state
  // Col 3:    space
  // Col 4:    Ldir glyph       — left stick direction char
  // Col 5:    space
  // Col 6-8:  "99%" or " 5%"  — battery, right-aligned 2 digits
  // Col 9:    space
  // Col 10:   Rdir glyph       — right stick direction char
  // Col 11:   space
  // Col 12-15:"R:0" or "R:1"  — right button state

  lcd.setCursor(0, 1);

  // LB state
  snprintf(buf, sizeof(buf), "L:%d ", data.LB ? 1 : 0);
  lcd.print(buf);                        // cols 0-3

  // Left direction glyph
  lcd.write(lDir);                       // col  4

  // Battery
  snprintf(buf, sizeof(buf), " %2d%%", batPercent);
  lcd.print(buf);                        // cols 5-8  " 99%" or "  5%"

  // Right direction glyph
  lcd.print(' ');                        // col  9
  lcd.write(rDir);                       // col  10

  // RB state
  snprintf(buf, sizeof(buf), " R:%d", data.RB ? 1 : 0);
  lcd.print(buf);                        // cols 11-15
}

// ╔══════════════════════════════════════════════════════════╗
// ║                         SETUP                           ║
// ╚══════════════════════════════════════════════════════════╝
void setup() {
  Serial.begin(115200);

  // ── GPIO ──────────────────────────────────────────────────
  pinMode(PIN_LB,    INPUT_PULLUP);
  pinMode(PIN_RB,    INPUT_PULLUP);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);   // red on  = not connected (default)
  digitalWrite(PIN_LED_G, LOW);

  // ── LCD ───────────────────────────────────────────────────
  // Wire.begin() uses ESP32 defaults: SDA=21, SCL=22
  Wire.begin();
  int lcdStatus = lcd.begin(16, 2);
  if (lcdStatus) {
    // begin() returns non-zero on failure
    // Can't use LCD to report — fall back to Serial
    Serial.print("LCD init failed, status=");
    Serial.println(lcdStatus);
    // Blink red LED rapidly to signal hardware fault
    while (1) {
      digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R));
      delay(200);
    }
  }
  lcd.backlight();
  loadBaseChars();
  lcd.setCursor(0, 0); lcd.print("  Combat Robot  ");
  lcd.setCursor(0, 1); lcd.print("  Connecting... ");

  // ── ESP-NOW ───────────────────────────────────────────────
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    lcd.setCursor(0, 1); lcd.print(" ESP-NOW  FAIL! ");
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  // Core 3.x send callback signature
  esp_now_register_send_cb(onDataSent);

  // Register the robot receiver as a peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, RX_MAC, 6);
  peer.channel = 0;       // 0 = auto match current channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    lcd.setCursor(0, 1); lcd.print("  PEER FAIL!    ");
    Serial.println("Failed to add peer");
    while (1);
  }

  Serial.print("TX MAC: "); Serial.println(WiFi.macAddress());
  delay(1200);
  lcd.clear();
}

// ╔══════════════════════════════════════════════════════════╗
// ║                          LOOP                           ║
// ╚══════════════════════════════════════════════════════════╝
void loop() {
  unsigned long now = millis();

  // ── Read joystick axes (ADC1 only) ──────────────────────
  data.LX = mapStick(analogRead(PIN_LX));
  data.LY = mapStick(analogRead(PIN_LY));
  data.RX = mapStick(analogRead(PIN_RX_A));
  data.RY = mapStick(analogRead(PIN_RY));

  // ── Read buttons (active LOW, pulled up internally) ──────
  data.LB = !digitalRead(PIN_LB);
  data.RB = !digitalRead(PIN_RB);
  data.LS = false;   // no stick-click on basic joystick modules
  data.RS = false;

  // ── Send packet at ~50 Hz ────────────────────────────────
  if (now - lastSend >= SEND_INTERVAL_MS) {
    esp_now_send(RX_MAC, (uint8_t*)&data, sizeof(data));
    lastSend = now;
  }

  // ── Connection watchdog ──────────────────────────────────
  // If ACK hasn't arrived within ACK_TIMEOUT_MS → link lost
  bool wasConnected = connected;
  connected = (now - lastAck < ACK_TIMEOUT_MS);

  if (connected != wasConnected) {
    // Link state changed — update LEDs immediately
    digitalWrite(PIN_LED_G, connected ? HIGH : LOW);
    digitalWrite(PIN_LED_R, connected ? LOW  : HIGH);
  }
  // ── Read battery every 5 seconds ────────────────────────
  //if (now - lastBatRead >= BAT_READ_INTERVAL) {
  //  batPercent = readBattery();
  //  lastBatRead = now;
  //}
  readBattery();
  // ── Update display ───────────────────────────────────────
  updateLCD();
}