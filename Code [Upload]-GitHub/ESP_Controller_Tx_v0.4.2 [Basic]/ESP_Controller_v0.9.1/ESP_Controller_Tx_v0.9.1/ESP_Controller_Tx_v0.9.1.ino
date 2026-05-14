// ================================================================
//  URC v0.9.1 — ESP-NOW RC Controller
//  Role    : Master / Transmitter
//  Board   : ESP32 38-pin DevKit
//  MAC     : 20:e7:c8:9f:47:f8
//
//  Hardware:
//    2x Analog Joystick     (LX/LY, RX/RY)
//    2x Joystick Click      (LABt, RABt)
//    2x Push Button         (LBt, RBt)
//    1x Potentiometer       (GPIO 39 — reserved, unused in 0.9.1)
//    1x Buzzer              (GPIO 18)
//    20x4 I2C LCD           (0x27)
//    2x LED                 (RED=13, GREEN=12)
//    Battery divider        (GPIO 36)
//
//  Struct (v0.9.1 — trimmed):
//    Lx, Ly     — left  stick  (-99 to +99)
//    Rx, Ry     — right stick  (-99 to +99)
//    LBt, RBt   — push buttons (true = held)
//    LABt, RABt — stick clicks (true = pressed)
//
//  Controls:
//    Ly   → forward / backward  (sent to robot drive)
//    Rx   → left / right turn   (sent to robot steer)
//    LBt  → arm/disarm STBY on robot TB6612
//    RBt  → reserved (empty on robot side)
//    LABt → stub — add light/horn/servo later
//    RABt → stub — add light/horn/servo later
//
//  Beep policy:
//    LBt / RBt  : beep on PRESS and on RELEASE (toggle feel)
//    LABt/RABt  : beep on PRESS only (one-shot click feel)
//
//  Startup safety:
//    Blocks until LBt = OFF and RBt = OFF
//    Battery ≥ 10% (LOW_BAT_PERCENT)
//
//  Changelog v0.9.1:
//    - Struct trimmed to 8 fields (removed TSW1/TSW2/Pot1/Pot2/BAT)
//    - BAT still sent as a separate field for display parity
//    - Startup caution: only LBt + RBt + battery checked
//    - Beep policy split: buttons beep both edges, clicks beep press only
//    - LCD row 2 simplified (no switches)
//    - LCD row 3 shows potBar stub + battery
// ================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================================================================
//  LCD
// ================================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ================================================================
//  PIN DEFINITIONS
// ================================================================
#define RED_LED      13
#define GREEN_LED    12
#define BUZZ_PIN     18

#define LX_PIN       34   // Left  stick X  (ADC1 only — safe with WiFi)
#define LY_PIN       35   // Left  stick Y
#define RX_PIN       32   // Right stick X
#define RY_PIN       33   // Right stick Y

#define L_STICK_PIN  26   // Left  stick click (LABt)  INPUT_PULLUP, active LOW
#define R_STICK_PIN  27   // Right stick click (RABt)  INPUT_PULLUP, active LOW
#define L_BTN_PIN    25   // Left  push button (LBt)   INPUT_PULLUP, active LOW
#define R_BTN_PIN    14   // Right push button (RBt)   INPUT_PULLUP, active LOW

#define BAT_PIN      36   // Battery voltage divider   ADC1, input-only
                          // Wiring: Lipo+ → 100kΩ → GPIO36 → 47kΩ → GND
                          // Divider ratio: 147/47 → calibrate BAT_CAL_FACTOR

// ================================================================
//  BATTERY CALIBRATION
//  HOW TO CALIBRATE:
//    1. Measure real battery voltage with a multimeter → e.g. 8.2V
//    2. Read what the code calculates → e.g. 7.8V  (check Serial)
//    3. Set BAT_CAL_FACTOR = (measured_real / code_calculated)
//       e.g. 8.2 / 7.8 ≈ 1.051
// ================================================================
#define FULL_BAT          8.4f   // Volts at 100% (2S LiPo fully charged)
#define EMPTY_BAT         6.6f   // Volts at   0% (2S LiPo cutoff)
#define LOW_BAT_THRESHOLD 6.8f   // Volts → triggers safe-mode / kill radio
#define LOW_BAT_PERCENT   10     // Startup blocks if battery below this %
#define BAT_CAL_FACTOR    (8.2f / 7.8f)  // Adjust with real multimeter reading

// ================================================================
//  JOYSTICK TUNING
//  DEADZONE        — raw % units to ignore around center (prevents drift)
//                    Range 0–99.  Increase if stick drifts when untouched.
//  CHANGE_THRESHOLD — minimum axis delta to register a new value.
//                    Prevents jitter noise clogging the radio.
// ================================================================
#define DEADZONE          15    // units (0-99 scale).  Try 10–20.
#define CHANGE_THRESHOLD   2    // axis must move this much to update

// ================================================================
//  BUZZER
// ================================================================
#define BEEP_FREQ   1200   // Hz  — change pitch here
#define BEEP_DUR      40   // ms  — duration of one beep tap

// ================================================================
//  RECEIVER MAC  — paste your robot's MAC here
//  Leave as broadcast (0xFF×6) during bench testing
// ================================================================
uint8_t receiverMAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// ================================================================
//  DATA STRUCT — MUST be byte-identical on TX and RX
//  Change field order or types → update BOTH sides
// ================================================================
typedef struct {
  int  Lx, Ly;       // Left  stick axes   -99 … +99
  int  Rx, Ry;       // Right stick axes   -99 … +99
  bool LBt, RBt;     // Push buttons       true = held down
  bool LABt, RABt;   // Stick clicks       true = pressed
  int  BAT;          // Battery %          0–100  (displayed on robot side too)
} ControllerData;

typedef struct {
  bool alive;        // Robot sends this back as heartbeat ACK
} AckData;

ControllerData data;
ControllerData prev;

// ================================================================
//  RUNTIME STATE
// ================================================================
int           BaT         = 100;
float         batteryVolt = 8.4f;
bool          Stat        = false;     // true = robot ACK received recently
unsigned long lastAckTime = 0;

// ================================================================
//  LCD LINE CACHE — only redraws lines that changed
// ================================================================
String line0 = "", line1 = "", line2 = "", line3 = "";

// ================================================================
//  CUSTOM LCD CHARACTERS
//  Slot 0 = ↗UR  1=↑U  2=↓D  3=←L  4=→R  5=↙DL  6=↘DR  7=↖UL
// ================================================================
byte UR_c[8] = { B00000, B00000, B01111, B00011, B00101, B01001, B10000, B00000 };
byte U_c[8]  = { B00100, B01110, B10101, B00100, B00100, B00100, B00100, B00100 };
byte D_c[8]  = { B00100, B00100, B00100, B00100, B00100, B10101, B01110, B00100 };
byte L_c[8]  = { B00000, B00010, B00100, B01000, B11111, B01000, B00100, B00010 };
byte R_c[8]  = { B00000, B01000, B00100, B00010, B11111, B00010, B00100, B01000 };
byte DL_c[8] = { B00000, B00000, B00001, B10010, B10100, B11000, B11110, B00000 };
byte DR_c[8] = { B00000, B00000, B10000, B01001, B00101, B00011, B01111, B00000 };
byte UL_c[8] = { B00000, B00000, B11110, B11000, B10100, B10010, B00001, B00000 };

// ================================================================
//  HELPERS
// ================================================================
void beep() {
  tone(BUZZ_PIN, BEEP_FREQ, BEEP_DUR);
}

// Map raw ADC (0-4095) → -99…+99 with deadzone applied
int mapAxis(int val) {
  int mapped = map(val, 0, 4095, 99, -99);
  if (abs(mapped) < DEADZONE) return 0;
  return mapped;
}

// Format axis value: 0 → " 0", positive → "+NN", negative → "-NN"
String fmt(int v) {
  char buf[6];
  if (v == 0) sprintf(buf, "%3d", v);
  else        sprintf(buf, "%+3d", v);
  return String(buf);
}

// Only write to LCD if content changed (saves flicker and I2C bus time)
void printIfChanged(int col, int row, String text, String &cache) {
  if (text != cache) {
    lcd.setCursor(col, row);
    lcd.print(text);
    cache = text;
  }
}

// Return CGRAM slot index for joystick direction arrow, 255 = center
byte getDirIcon(int x, int y) {
  if (abs(x) < DEADZONE && abs(y) < DEADZONE) return 255;
  if (y > DEADZONE  && x > DEADZONE)  return 0;  // ↗
  if (y > DEADZONE  && x < -DEADZONE) return 7;  // ↖
  if (y < -DEADZONE && x > DEADZONE)  return 6;  // ↘
  if (y < -DEADZONE && x < -DEADZONE) return 5;  // ↙
  if (y > DEADZONE)                   return 1;  // ↑
  if (y < -DEADZONE)                  return 2;  // ↓
  if (x < -DEADZONE)                  return 3;  // ←
  if (x > DEADZONE)                   return 4;  // →
  return 255;
}

// Simple ASCII bar for axis value display  e.g. "[-===  ]"
String axisBar(int val, int width = 6) {
  // val: -99…+99 → map center to mid of bar
  int filled = ((val + 99) * width) / 198;
  filled = constrain(filled, 0, width - 1);
  String bar = "[";
  for (int i = 0; i < width; i++) bar += (i == filled) ? '=' : '-';
  bar += "]";
  return bar;
}

// ================================================================
//  BEEP POLICY
//  LBt / RBt  : beep on PRESS (LOW→HIGH) AND on RELEASE (HIGH→LOW)
//  LABt/RABt  : beep on PRESS only
// ================================================================
void checkBeeps() {
  static bool pLBt  = false, pRBt  = false;
  static bool pLABt = false, pRABt = false;

  // Push buttons — beep both edges
  if ( data.LBt  && !pLBt)  beep();   // LBt pressed
  if (!data.LBt  &&  pLBt)  beep();   // LBt released
  if ( data.RBt  && !pRBt)  beep();   // RBt pressed
  if (!data.RBt  &&  pRBt)  beep();   // RBt released

  // Stick clicks — beep press only
  if ( data.LABt && !pLABt) beep();
  if ( data.RABt && !pRABt) beep();

  pLBt  = data.LBt;
  pRBt  = data.RBt;
  pLABt = data.LABt;
  pRABt = data.RABt;
}

// ================================================================
//  ESP-NOW RECEIVE CALLBACK  (ACK from robot)
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
//  READ INPUTS, BUILD PACKET, SEND
//  Called every loop tick — no delays inside
// ================================================================
void DataWork() {
  // --- Joystick axes ---
  int newLx = mapAxis(analogRead(LX_PIN));
  int newLy = mapAxis(analogRead(LY_PIN));
  int newRx = mapAxis(analogRead(RX_PIN));
  int newRy = mapAxis(analogRead(RY_PIN));

  // Only update if change exceeds threshold (anti-jitter)
  if (abs(newLx - data.Lx) > CHANGE_THRESHOLD) data.Lx = newLx;
  if (abs(newLy - data.Ly) > CHANGE_THRESHOLD) data.Ly = newLy;
  if (abs(newRx - data.Rx) > CHANGE_THRESHOLD) data.Rx = newRx;
  if (abs(newRy - data.Ry) > CHANGE_THRESHOLD) data.Ry = newRy;

  // --- Buttons & stick clicks (active LOW → invert) ---
  data.LABt = !digitalRead(L_STICK_PIN);
  data.RABt = !digitalRead(R_STICK_PIN);
  data.LBt  = !digitalRead(L_BTN_PIN);
  data.RBt  = !digitalRead(R_BTN_PIN);
  data.BAT  = BaT;

  // --- Beep check (edge detection, must run after data updated) ---
  checkBeeps();

  // --- Send over ESP-NOW ---
  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

  // --- Serial debug (every packet) ---
  Serial.printf("L(%+3d,%+3d) R(%+3d,%+3d) | LBt:%d RBt:%d | LABt:%d RABt:%d | BAT:%.2fV %3d%% | %s\n",
    data.Lx, data.Ly,
    data.Rx, data.Ry,
    (int)data.LBt, (int)data.RBt,
    (int)data.LABt, (int)data.RABt,
    batteryVolt, BaT,
    Stat ? "CONNECTED" : "NO LINK");
}

// ================================================================
//  UPDATE LCD
//
//  ┌────────────────────┐
//  │L x:+99 y:+99 [↑] L│  row 0 — left stick + direction
//  │R x:+99 y:+99 [↑] R│  row 1 — right stick + direction
//  │LB:0 RB:0 LA:0 RA:0│  row 2 — buttons and clicks
//  │Drive[======] BAT99%│  row 3 — drive bar + battery
//  └────────────────────┘
// ================================================================
void updateDisplay() {
  char l0[21], l1[21], l2[21], l3[21];

  snprintf(l0, 21, "L x:%s y:%s     ", fmt(data.Lx).c_str(), fmt(data.Ly).c_str());
  snprintf(l1, 21, "R x:%s y:%s     ", fmt(data.Rx).c_str(), fmt(data.Ry).c_str());
  snprintf(l2, 21, "LB:%d RB:%d LA:%d RA:%d",
           (int)data.LBt, (int)data.RBt, (int)data.LABt, (int)data.RABt);

  // Row 3: drive bar (Ly) + battery
  snprintf(l3, 21, "Drv%s BAT%3d%%", axisBar(data.Ly).c_str(), BaT);

  printIfChanged(0, 0, l0, line0);
  printIfChanged(0, 1, l1, line1);
  printIfChanged(0, 2, l2, line2);
  printIfChanged(0, 3, l3, line3);

  // Stick direction icons (cols 14-19 of rows 0 & 1)
  byte iconL = getDirIcon(data.Lx, data.Ly);
  byte iconR = getDirIcon(data.Rx, data.Ry);

  lcd.setCursor(14, 0);
  if (iconL == 255) lcd.print("+");
  else              lcd.write(iconL);
  lcd.print(" L:");
  lcd.print((int)data.LABt);

  lcd.setCursor(14, 1);
  if (iconR == 255) lcd.print("+");
  else              lcd.write(iconR);
  lcd.print(" R:");
  lcd.print((int)data.RABt);
}

// ================================================================
//  CONNECTION LED
//  GREEN = robot ACK received within last 500ms
//  RED   = no recent ACK
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
//  BATTERY MANAGER
//  - Reads battery every 5 seconds (non-blocking)
//  - If battery drops below LOW_BAT_THRESHOLD:
//      → sends zero packet (kills robot)
//      → blinks red LED
//      → shows LOW BAT screen
//      → blocks until battery recovers
//  Returns true if in low-battery safe mode
// ================================================================
bool BatteryManager() {
  static unsigned long lastRead  = 0;
  static unsigned long lastBlink = 0;
  static bool ledState  = false;
  static bool killSent  = false;

  // Read battery every 5 seconds
  if (millis() - lastRead > 5000) {
    lastRead = millis();
    int raw = analogRead(BAT_PIN);
    float v_adc = raw * (3.3f / 4095.0f);
    batteryVolt = v_adc * ((100.0f + 47.0f) / 47.0f) * BAT_CAL_FACTOR;

    if      (batteryVolt >= FULL_BAT)  BaT = 100;
    else if (batteryVolt <= EMPTY_BAT) BaT = 0;
    else BaT = (int)((batteryVolt - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));

    Serial.printf("[BAT] %.2fV  %d%%\n", batteryVolt, BaT);
  }

  // Low battery safe mode
  if (batteryVolt <= LOW_BAT_THRESHOLD) {
    if (!killSent) {
      memset(&data, 0, sizeof(data));
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
      Serial.printf("[BAT] !! SAFE MODE !! %.2fV %d%% — radio KILLED\n", batteryVolt, BaT);
      killSent = true;
    }

    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState  = !ledState;
      digitalWrite(RED_LED, ledState);
    }
    digitalWrite(GREEN_LED, LOW);

    lcd.setCursor(0, 0); lcd.print("    |=========|     ");
    lcd.setCursor(0, 1); lcd.print("    | LOW BAT |     ");
    lcd.setCursor(0, 2); lcd.print("    |Radio-OFF|     ");
    lcd.setCursor(0, 3); lcd.print("    |=========|     ");

    return true;
  }

  killSent = false;
  return false;
}

// ================================================================
//  SYSTEM CONTROL
//  Wraps battery manager + restart logic
// ================================================================
void SystemControl() {
  static bool wasLow = false;
  bool isLow = BatteryManager();

  if (isLow) {
    wasLow = true;
    while (BatteryManager()) { delay(50); }
  }

  if (wasLow) {
    wasLow = false;
    startup();  // re-run safety check after battery recovery
    line0 = line1 = line2 = line3 = "";
  }
}

// ================================================================
//  STARTUP SAFETY CHECK  (v0.9.1)
//  Blocks until:
//    LBt  = OFF (HIGH on pullup — button not pressed)
//    RBt  = OFF
//    Battery ≥ LOW_BAT_PERCENT
//
//  ┌────────────────────┐
//  │===== Caution! =====│  row 0
//  │ LBt:^_^  RBt:FAIL │  row 1
//  │  Battery:  85%     │  row 2
//  │====================│  row 3
//  └────────────────────┘
// ================================================================
void startup() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("===== Caution! =====");
  Serial.println("[STARTUP] Safety check — release LBt and RBt before starting");

  while (true) {
    // Read battery for startup check
    int rawBat  = analogRead(BAT_PIN);
    float v_adc = rawBat * (3.3f / 4095.0f);
    float batV  = v_adc * ((100.0f + 47.0f) / 47.0f);
    int batPct  = 0;
    if      (batV >= FULL_BAT)  batPct = 100;
    else if (batV <= EMPTY_BAT) batPct = 0;
    else    batPct = (int)((batV - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));

    // digitalRead HIGH = not pressed (pullup) = safe
    bool lbtSafe  = digitalRead(L_BTN_PIN);
    bool rbtSafe  = digitalRead(R_BTN_PIN);
    bool batSafe  = (batPct >= LOW_BAT_PERCENT);

    if (lbtSafe && rbtSafe && batSafe) {
      lcd.setCursor(0, 0); lcd.print("====================");
      lcd.setCursor(0, 1); lcd.print("      Caution!      ");
      lcd.setCursor(0, 2); lcd.print("   ** ALL CLEAR **  ");
      lcd.setCursor(0, 3); lcd.print("====================");
      Serial.println("[STARTUP] All clear — proceeding");
      delay(800);
      lcd.clear();
      return;
    }

    char rowBtn[21], rowBat[21];
    snprintf(rowBtn, 21, " LBt:%-4s  RBt:%-4s ",
             lbtSafe ? "^_^" : "FAIL",
             rbtSafe ? "^_^" : "FAIL");
    snprintf(rowBat, 21, "    Battery: %3d%%   ", batPct);

    lcd.setCursor(0, 1); lcd.print(rowBtn);
    lcd.setCursor(0, 2); lcd.print(rowBat);
    lcd.setCursor(0, 3); lcd.print("====================");

    delay(150);
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n==============================");
  Serial.println("  URC TX v0.9.1 — Booting...");
  Serial.println("==============================");

  // GPIO
  pinMode(RED_LED,      OUTPUT);
  pinMode(GREEN_LED,    OUTPUT);
  pinMode(BUZZ_PIN,     OUTPUT);
  digitalWrite(RED_LED,   HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BUZZ_PIN,  LOW);

  pinMode(L_STICK_PIN, INPUT_PULLUP);
  pinMode(R_STICK_PIN, INPUT_PULLUP);
  pinMode(L_BTN_PIN,   INPUT_PULLUP);
  pinMode(R_BTN_PIN,   INPUT_PULLUP);
  pinMode(BAT_PIN,     INPUT);

  analogSetAttenuation(ADC_11db);   // full 0–3.3V range for ADC

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, UR_c);
  lcd.createChar(1, U_c);
  lcd.createChar(2, D_c);
  lcd.createChar(3, L_c);
  lcd.createChar(4, R_c);
  lcd.createChar(5, DL_c);
  lcd.createChar(6, DR_c);
  lcd.createChar(7, UL_c);
  lcd.home();

  // Startup safety gate
  startup();

  // Boot splash
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  Radio Booting...  ");
  lcd.setCursor(0, 2); lcd.print("    ESP-NOW v0.9.1  ");
  lcd.setCursor(0, 3); lcd.print("====================");

  // Boot beep: two ascending tones
  tone(BUZZ_PIN, 1000, 80); delay(130);
  tone(BUZZ_PIN, 1400, 80); delay(350);

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.print("[INIT] TX MAC: ");
  Serial.println(WiFi.macAddress());
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAILED");
    while (1) { digitalWrite(RED_LED, !digitalRead(RED_LED)); delay(200); }
  }
  Serial.println("[INIT] ESP-NOW OK");

  esp_now_register_recv_cb(OnDataRecv);

  // Add receiver as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("[INIT] Peer added — entering main loop");
  lcd.clear();
}

// ================================================================
//  MAIN LOOP  — target ~50Hz, no blocking delays
//  DataWork() → updateDisplay() → updateConnectionLED() ~20ms/tick
// ================================================================
void loop() {
  SystemControl();       // battery watchdog (non-blocking unless fault)
  DataWork();            // read inputs, beep edges, send ESP-NOW packet
  updateDisplay();       // refresh LCD (cached, only changed lines drawn)
  updateConnectionLED(); // green/red LED based on ACK timeout
  delay(20);             // ~50Hz  — lower for faster response (min ~10ms)
}
