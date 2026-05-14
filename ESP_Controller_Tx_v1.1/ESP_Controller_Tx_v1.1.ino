// ================================================================
// ESP-NOW RC Controller  v0.5
// Role  : Master / Transmitter
// Board : ESP32 (38-pin devkit)
// Hardware:
//   2x Analog Joystick  (LX/LY, RX/RY)
//   2x Joystick Click   (LS, RS)
//   2x Push Button      (LB, RB)
//   2x Toggle Switch    (SW1, SW2)   ← NEW
//   2x Potentiometer    (PotM1/2)    ← NEW
//   20x4 I2C LCD        (0x27)
//   2x LED              (RED=13, GREEN=12)
// Protocol : ESP-NOW (one-way to receiver)
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================================================================
//  LCD
// ================================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);     // I2C addr, 20 cols, 4 rows

// ================================================================
//  LED PINS
//  GREEN = send success  |  RED = send fail / not connected
// ================================================================
#define RED_LED   13
#define GREEN_LED 12

// ================================================================
//  JOYSTICK AXIS PINS  (ADC – do not use as OUTPUT)
// ================================================================
#define LX_PIN  34    // Left  stick X
#define LY_PIN  35    // Left  stick Y
#define RX_PIN  32    // Right stick X
#define RY_PIN  33    // Right stick Y

// ================================================================
//  DIGITAL INPUT PINS  (all use INPUT_PULLUP, active LOW)
// ================================================================
#define L_STICK_PIN  26   // Left  joystick click  (LS)
#define R_STICK_PIN  27   // Right joystick click  (RS)
#define L_BTN_PIN    25   // Left  push button     (LB)
#define R_BTN_PIN    14   // Right push button     (RB)
#define SW1_PIN      16   // Toggle switch 1       (SW1) ← NEW
#define SW2_PIN      17   // Toggle switch 2       (SW2) ← NEW
#define BUZZ_PIN     18

// ================================================================
//  POTENTIOMETER PINS  (ADC input-only pins, no internal pullup)
//  GPIO 36 (VP) and 39 (VN) – these are input-only on ESP32
// ================================================================
#define POTM1_PIN  36    // Potentiometer 1  (PotM1) ← NEW
#define POTM2_PIN  4    // Potentiometer 2  (PotM2) ← NEW

// ================================================================
//  TUNING CONSTANTS  — edit here to tune behaviour
// ================================================================
#define DEADZONE          11   // Mapped joystick value below which = center
#define CHANGE_THRESHOLD   2   // Min axis change to register (kills jitter)
#define POT_ZERO_THRESH   80   // Raw ADC ≤ this = "pot is at zero" for startup

// ================================================================
//  BATTERY PLACEHOLDER
//  Wire an ADC + voltage-divider later and replace this int
//  with a real read. Displayed on LCD row 3.
//  (Add to ControllerData struct if receiver also needs it.)
// ================================================================
int BaT = 100;    // Battery %, default 100
#define BAT_PIN 39

#define FULL_BAT 8.4
#define EMPTY_BAT 6.6
#define LOW_BAT_THRESHOLD 6.8
// ================================================================
//  CUSTOM LCD CHARACTERS  (HD44780 CGRAM slots 0–7)
//
//  Slot  Char    getDirIcon() return value
//   0     UR     0
//   1     U      1
//   2     D      2
//   3     L      3
//   4     R      4
//   5     DL     5
//   6     DR     6
//   7     UL     7
//   --    center 255  (printed as '+' in code)
// ================================================================
byte UR_c[8] = {B00000,B00000,B01111,B00011,B00101,B01001,B10000,B00000}; // Up-Right   (slot 0)
byte U_c[8]  = {B00100,B01110,B10101,B00100,B00100,B00100,B00100,B00100}; // Up         (slot 1)
byte D_c[8]  = {B00100,B00100,B00100,B00100,B00100,B10101,B01110,B00100}; // Down       (slot 2)
byte L_c[8]  = {B00000,B00010,B00100,B01000,B11111,B01000,B00100,B00010}; // Left       (slot 3)
byte R_c[8]  = {B00000,B01000,B00100,B00010,B11111,B00010,B00100,B01000}; // Right      (slot 4)
byte DL_c[8] = {B00000,B00000,B00001,B10010,B10100,B11000,B11110,B00000}; // Down-Left  (slot 5)
byte DR_c[8] = {B00000,B00000,B10000,B01001,B00101,B00011,B01111,B00000}; // Down-Right (slot 6)
byte UL_c[8] = {B00000,B00000,B11110,B11000,B10100,B10010,B00001,B00000}; // Up-Left    (slot 7)

// ================================================================
//  DATA STRUCT  — sent over ESP-NOW to the receiver every loop
//  Add BaT here if the receiver also needs battery level.
// ================================================================
typedef struct {
  int  LX, LY;        // Left  stick: −100 … +100
  int  RX, RY;        // Right stick: −100 … +100
  bool LS, RS;         // Joystick clicks
  bool LB, RB;         // Push buttons
  bool SW1, SW2;       // Toggle switches  ← NEW
  int  PotM1, PotM2;  // Potentiometers 0…100 ← NEW
} ControllerData;

ControllerData data;        // Live state sent each loop
ControllerData prev = {0};  // Previous state (kept for reference; extend if needed)

// ================================================================
//  LCD LINE CACHE  — only re-draws a row when its string changes
// ================================================================
String line0 = "", line1 = "", line2 = "", line3 = "";

// ================================================================
//  RECEIVER MAC  — change to your robot ESP32's MAC address
// ================================================================
uint8_t receiverMAC[] = {0xA4, 0xF0, 0x0F, 0x5F, 0xE0, 0x68};


// ================================================================
//  MAP: JOYSTICK AXIS
//  Raw ADC 0–4095  →  −100…+100 with deadzone
//  Flip the map(…, 100, -100) to (…, -100, 100) if axis is inverted
// ================================================================
int mapAxis(int val) {
  int mapped = map(val, 0, 4095, 99, -99);  // Inverted: tweak if needed
  if (abs(mapped) < DEADZONE) return 0;        // Zero the dead-centre zone
  return mapped;
}

// ================================================================
//  MAP: POTENTIOMETER
//  Raw ADC 0–4095  →  0…100
// ================================================================
int mapPot(int val) {
  return map(val, 0, 4095, 0, 100);
}

// ================================================================
//  SMART LCD PRINT  — writes only if content changed (no flicker)
// ================================================================
void printIfChanged(int col, int row, String text, String &cache) {
  if (text != cache) {
    lcd.setCursor(col, row);
    lcd.print(text);
    cache = text;
  }
}

// ================================================================
//  FORMAT AXIS VALUE  → always 4 chars with sign
//   0  →  "   0"    +99  →  " +99"    -100  →  "-100"
// ================================================================
String fmt(int v) {
  char buf[6];
  if (v == 0) sprintf(buf, "%3d",  v);   // No sign for zero
  else        sprintf(buf, "%+3d", v);   // +/- for nonzero
  return String(buf);
}

// ================================================================
//  DIRECTION ICON  — maps (x,y) to a CGRAM slot (0–7) or 255
//  255 = dead-centre, printed as '+' in updateDisplay()
// ================================================================
byte getDirIcon(int x, int y) {
  if (abs(x) < DEADZONE && abs(y) < DEADZONE) return 255; // Centre
  if (y >  DEADZONE && x >  DEADZONE) return 0;   // Up-Right
  if (y >  DEADZONE && x < -DEADZONE) return 7;   // Up-Left
  if (y < -DEADZONE && x >  DEADZONE) return 6;   // Down-Right
  if (y < -DEADZONE && x < -DEADZONE) return 5;   // Down-Left
  if (y >  DEADZONE)  return 1;   // Up
  if (y < -DEADZONE)  return 2;   // Down
  if (x < -DEADZONE)  return 3;   // Left
  if (x >  DEADZONE)  return 4;   // Right
  return 255;                      // Fallback centre
}

// ================================================================
//  UPDATE DISPLAY
//
//  LCD layout  (20 cols × 4 rows):
//  ┌────────────────────┐
//  │LX:+100 LY:+100 L:0►│  row 0 — left stick, LS, direction icon
//  │RX:+100 RY:+100 R:0▲│  row 1 — right stick, RS, direction icon
//  │LB:0 RB:0 S1:0 S2:0 │  row 2 — buttons + toggle switches
//  │P1:100 P2:100 B:100% │  row 3 — pots + battery %
//  └────────────────────┘
//
//  Cols 0–18 filled by sprintf (19 chars).
//  Col 19 = direction icon character (written separately).
//
//  HOW TO EDIT:
//   • Change a label?  Edit the format string in the sprintf below.
//   • Change units?    Edit the format string and matching %d / %s.
//   • Add a field?     Shorten another label to make room (20 col limit).
// ================================================================
void updateDisplay() {
  char l0[21], l1[21], l2[21], l3[21];

  // --- Row 0: Left stick axes (fmt = 4 chars each) + LS state ---
  // "LX:+100 LY:+100 L:0"  →  19 chars, icon written at col 19
  
  sprintf(l0, "L x:%s y:%s ", fmt(data.LX).c_str(), fmt(data.LY).c_str());  // 14 chars → cols 0-13
  // --- Row 1: Right stick axes + RS state ---
  // "RX:+100 RY:+100 R:0"  →  19 chars, icon at col 19
   
  
  sprintf(l1, "R x:%s y:%s ", fmt(data.RX).c_str(), fmt(data.RY).c_str());  // 14 chars → cols 0-13
 
  // --- Row 2: Buttons + Switches ---
  // "LB:0 RB:0 S1:0 S2:0 "  →  20 chars exactly
  sprintf(l2, "LB:%d RB:%d  S1:%d S2:%d",
    (int)data.LB,
    (int)data.RB,
    (int)data.SW1,
    (int)data.SW2
  );

  // --- Row 3: Potentiometers + Battery ---
  // "P1:100 P2:100 B:100%"  →  20 chars exactly
  // %3d = right-justified 3-digit field (space-padded); %% = literal '%'
  sprintf(l3, "P1:%3d P2:%3d B:%3d%%",
    data.PotM1,
    data.PotM2,
    BaT
  );

  // --- Smart redraw (only rows that changed are written) ---
  printIfChanged(0, 0, l0, line0);
  printIfChanged(0, 1, l1, line1);
  printIfChanged(0, 2, l2, line2);
  printIfChanged(0, 3, l3, line3);

  // --- Direction icons at col 19 (always written, 1 char each) ---
  byte iconL = getDirIcon(data.LX, data.LY);
  byte iconR = getDirIcon(data.RX, data.RY);

  lcd.setCursor(14, 0);                                    // Icon at col 14
  if (iconL == 255) lcd.print("+"); else lcd.write(iconL); // direction char
  lcd.print("  L:");                                       // cols 15-18
  lcd.print((int)data.LS);                                 // col 19
  
  lcd.setCursor(14, 1);
  if (iconR == 255) lcd.print("+"); else lcd.write(iconR);
  lcd.print("  R:");
  lcd.print((int)data.RS);
}

// ================================================================
//  ESP-NOW SEND CALLBACK  — fires after every esp_now_send()
//  Status drives LEDs only; LCD status line removed (you have LEDs).
// ================================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    digitalWrite(GREEN_LED, HIGH);  // Green ON  = receiver acknowledged
    digitalWrite(RED_LED,   LOW);
  } else {
    digitalWrite(RED_LED,   HIGH);  // Red ON    = no receiver / send failed
    digitalWrite(GREEN_LED, LOW);
  }
}

// ================================================================
//  READ & SEND DATA
//  Reads every input, applies filtering, packs into struct, sends.
// ================================================================
void DataWork() {
  // -- Joystick axes: map raw ADC, apply change-threshold filter --
  int newLX = mapAxis(analogRead(LX_PIN));
  int newLY = mapAxis(analogRead(LY_PIN));
  int newRX = mapAxis(analogRead(RX_PIN));
  int newRY = mapAxis(analogRead(RY_PIN));

  if (abs(newLX - data.LX) > CHANGE_THRESHOLD) data.LX = newLX; // Ignore tiny jitter
  if (abs(newLY - data.LY) > CHANGE_THRESHOLD) data.LY = newLY;
  if (abs(newRX - data.RX) > CHANGE_THRESHOLD) data.RX = newRX;
  if (abs(newRY - data.RY) > CHANGE_THRESHOLD) data.RY = newRY;

  // -- Digital: joystick clicks + push buttons (active LOW = pressed) --
  data.LS = !digitalRead(L_STICK_PIN);
  data.RS = !digitalRead(R_STICK_PIN);
  data.LB = !digitalRead(L_BTN_PIN);
  data.RB = !digitalRead(R_BTN_PIN);

  // -- Toggle switches (active LOW with pullup = ON when LOW) --
  data.SW1 = !digitalRead(SW1_PIN);   // true = switch ON
  data.SW2 = !digitalRead(SW2_PIN);

  // -- Potentiometers: map raw ADC to 0–100 --
  data.PotM1 = mapPot(analogRead(POTM1_PIN));
  int raw = analogRead(POTM2_PIN);
  int mapped = mapPot(raw);
  data.PotM2 = (data.PotM2 * 3 + mapped) / 4;  // smoothing

  // -- Send entire struct over ESP-NOW --
  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

  // -- Serial debug (open Serial Monitor at 115200) --
  Serial.printf(
    "L(%d,%d) R(%d,%d) | LB:%d LS:%d RS:%d RB:%d | SW1:%d SW2:%d | P1:%d P2:%d | BAT:%d%%\n",
    data.LX, data.LY, data.RX, data.RY,
    data.LB, data.LS, data.RS, data.RB,
    data.SW1, data.SW2,
    data.PotM1, data.PotM2,
    BaT
  );
}

// ================================================================
//  STARTUP SAFETY CHECK  (blocking — loop() does NOT run until clear)
//
//  Rule: controller must not be armed at power-on.
//  Passes only when:
//    SW1  = OFF  (HIGH on pullup → switch open)
//    SW2  = OFF
//    PotM1 raw ADC ≤ POT_ZERO_THRESH  (knob at minimum)
//    PotM2 raw ADC ≤ POT_ZERO_THRESH
//
//  LCD during check:
//  ┌────────────────────┐
//  │== STARTUP  CHECK ==│  row 0 — fixed header
//  │SW1:OK   SW2:FAIL   │  row 1 — switch status
//  │P1:OK    P2:FAIL    │  row 2 — pot status
//  │Set to 0/OFF & wait │  row 3 — user hint
//  └────────────────────┘
//
//  Passes → "ALL CLEAR" for 800 ms → returns and boots normally.
// ================================================================
void startup() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("===== Caution! ====="); // Row 0: fixed — 20 chars

  while (true) {
    // -- Read raw safety conditions --
    bool sw1Safe  = digitalRead(SW1_PIN);          // HIGH = OFF = safe
    bool sw2Safe  = digitalRead(SW2_PIN);
    int  rawP1    = analogRead(POTM1_PIN);
    int  rawP2    = analogRead(POTM2_PIN);
    bool pot1Safe = (rawP1 <= POT_ZERO_THRESH);    // Near zero = safe
    bool pot2Safe = (rawP2 <= POT_ZERO_THRESH);

    // -- All conditions satisfied? --
    if (sw1Safe && sw2Safe && pot1Safe && pot2Safe) {
      lcd.setCursor(0, 0); lcd.print("===================="); // Clear rows 1-3
      lcd.setCursor(0, 1); lcd.print("       Caution!     "); // Clear rows 1-3
      lcd.setCursor(0, 2); lcd.print("   ** ALL CLEAR **  "); // 20 chars
      lcd.setCursor(0, 3); lcd.print("====================");
      delay(800);   // Show confirmation briefly
      lcd.clear();
      return;       // Exit: proceed to normal boot
    }

    //====================
    //LX:-99 Ly:+99 +  L:1
    //Rx:  0 Ry:  0 +  R:0
    //
    //


    // -- Build status strings (each exactly 20 chars) --
    // %-4s = left-justify in 4-char field: "OK  " or "FAIL"
    // %-5s = left-justify in 5-char field: "OK   " or "FAIL "

    char rowSW[21];   // Switch row
    char rowPT[21];   // Pot row
    // "SW1:OK   SW2:OK   " or "SW1:FAIL SW2:FAIL   " → 20 chars
    sprintf(rowSW, " SW1:%-4s  SW2:%-4s ",
      sw1Safe  ? "^_^" : "FAIL",
      sw2Safe  ? "^_^" : "FAIL"
    );
    // "P1:OK    P2:OK    " → 20 chars
    sprintf(rowPT, " P1 :%-5s P2 :%-5s",
      pot1Safe ? "^_^" : "FAIL",
      pot2Safe ? "^_^" : "FAIL"
    );

    lcd.setCursor(0, 1); lcd.print(rowSW);
    lcd.setCursor(0, 2); lcd.print(rowPT);
    lcd.setCursor(0, 3); lcd.print("===================="); // 20 chars

    delay(150);   // Re-check rate
  }
}



bool BatteryManager() 
{
  static unsigned long lastRead = 0;
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  static bool killSent = false;   // ensure we send neutral only once

  static float batteryVoltage = 8.4;

  // --- Read battery every 200ms ---
  if (millis() - lastRead > 200) {
    lastRead = millis();

    int raw = analogRead(BAT_PIN);
    float v_adc = raw * (3.3 / 4095.0);
    batteryVoltage = v_adc * ((100.0 + 47.0) / 47.0);

    // Convert to %
    if (batteryVoltage >= FULL_BAT) BaT = 100;
    else if (batteryVoltage <= EMPTY_BAT) BaT = 0;
    else BaT = (batteryVoltage - EMPTY_BAT) * 100 / (FULL_BAT - EMPTY_BAT);
  }

  // ================================
  // 🚨 LOW BATTERY MODE
  // ================================
  if (batteryVoltage <= LOW_BAT_THRESHOLD) {

    // --- SEND SAFE ZERO STATE ONCE ---
    if (!killSent) {
      data.LX = 0;
      data.LY = 0;
      data.RX = 0;
      data.RY = 0;

      data.LS = 0;
      data.RS = 0;
      data.LB = 0;
      data.RB = 0;

      data.SW1 = 0;
      data.SW2 = 0;

      data.PotM1 = 0;
      data.PotM2 = 0;
      Serial.printf(
      "SAFE MODE -> L(%d,%d) R(%d,%d) | LB:%d LS:%d RS:%d RB:%d | SW1:%d SW2:%d | P1:%d P2:%d | BAT:%d%%\n",
      data.LX, data.LY, data.RX, data.RY,
      data.LB, data.LS, data.RS, data.RB,
      data.SW1, data.SW2,
      data.PotM1, data.PotM2,
      BaT
      );
      // send neutral packet
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

      killSent = true;
    }

    // --- Blink RED LED ---
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(RED_LED, ledState);
    }

    digitalWrite(GREEN_LED, LOW);

    //Override LCD (center message)
    lcd.setCursor(0, 0); lcd.print("    |=========|     ");
    lcd.setCursor(0, 1); lcd.print("    | LOW BAT |     ");
    lcd.setCursor(0, 2); lcd.print("    |Radio-OFF|     ");
    lcd.setCursor(0, 3); lcd.print("    |=========|  ");

    return true;  // ⛔ STOP loop
  }

  // --- NORMAL MODE ---
  killSent = false;  // reset when battery OK again
  digitalWrite(RED_LED, LOW);

  return false;
}

 
// ================================================================
//  SETUP
// ================================================================
void setup() 
{
  Serial.begin(115200);

  // -- LED outputs (start with RED = not connected) --
  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED,   HIGH);
  digitalWrite(GREEN_LED, LOW);

  // -- Digital inputs with internal pull-up --
  //    Switch/button is pressed (active) when pin reads LOW
  pinMode(L_STICK_PIN, INPUT_PULLUP);
  pinMode(R_STICK_PIN, INPUT_PULLUP);
  pinMode(L_BTN_PIN,   INPUT_PULLUP);
  pinMode(R_BTN_PIN,   INPUT_PULLUP);
  pinMode(SW1_PIN,     INPUT_PULLUP);   // New toggle switch 1
  pinMode(SW2_PIN,     INPUT_PULLUP);   // New toggle switch 2

  // -- ADC pins for potentiometers --
  //    GPIO 36 and 39 are input-only; INPUT_PULLUP has no effect on them
  pinMode(POTM1_PIN, INPUT);   // Pot 1
  pinMode(POTM2_PIN, INPUT);   // Pot 2

  analogSetAttenuation(ADC_11db);

  // -- LCD init --
  lcd.init();
  lcd.backlight();

  // -- Load custom direction characters into CGRAM slots 0–7 --
  //    Must match slot numbers returned by getDirIcon()
  //    NOTE: HD44780 only has 8 slots (0–7); slot 8 wraps = bug in older code, fixed here
  lcd.createChar(0, UR_c);    // Up-Right
  lcd.createChar(1, U_c);     // Up
  lcd.createChar(2, D_c);     // Down
  lcd.createChar(3, L_c);     // Left
  lcd.createChar(4, R_c);     // Right
  lcd.createChar(5, DL_c);    // Down-Left
  lcd.createChar(6, DR_c);    // Down-Right
  lcd.createChar(7, UL_c);    // Up-Left
  lcd.home();                  // Return cursor after CGRAM writes

  // -- Startup safety check (BLOCKS until all inputs are neutral) --
  startup();

  // -- Boot splash (shown after startup check passes) --
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  Radio Booting...  ");
  lcd.setCursor(0, 2); lcd.print("     ESP-NOW v5     ");
  lcd.setCursor(0, 3); lcd.print("====================");
  delay(350);

  // -- WiFi STA mode (required by ESP-NOW) --
  WiFi.mode(WIFI_STA);
  delay(150);

  // -- ESP-NOW init + peer registration --
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  esp_now_add_peer(&peerInfo);

  // -- Clear LCD, ready for main display --
  lcd.clear();
}

// ================================================================
//  LOOP  — ~20 Hz
// ================================================================
void loop() 
{
  if (BatteryManager()) return;  // 🚨 stops everything if low battery
  DataWork();       // Read all inputs → pack struct → send ESP-NOW
  updateDisplay();  // Refresh LCD (smart: only changed rows redrawn)
  delay(50);        // 50 ms = 20 Hz
}