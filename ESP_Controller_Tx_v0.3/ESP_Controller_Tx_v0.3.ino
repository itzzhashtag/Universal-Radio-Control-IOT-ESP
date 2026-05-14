// ================================================================
// ESP-NOW RC Controller  v0.6
// Role  : Master / Transmitter
// Board : ESP32 (38-pin devkit)
// Hardware:
//   2x Analog Joystick  (LX/LY, RX/RY)
//   2x Joystick Click   (LS, RS)
//   2x Push Button      (LB, RB)
//   2x Toggle Switch    (SW1=16, SW2=17)
//   1x Potentiometer    (PotM1=36)
//   1x Buzzer           (BUZZ=18)
//   20x4 I2C LCD        (0x27)
//   2x LED              (RED=13, GREEN=12)
// Protocol : ESP-NOW (one-way to receiver)
// Changelog v0.6:
//   - Removed PotM2 entirely
//   - SW1 → GPIO 16, SW2 → GPIO 17
//   - Buzzer added on GPIO 18
//   - Startup check: SW1, SW2, PotM1, battery > 10%
//   - Startup LCD shows Battery% in place of P2
//   - Beep once on any button press or switch toggle
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
//  LED PINS
// ================================================================
#define RED_LED   13
#define GREEN_LED 12

// ================================================================
//  BUZZER
// ================================================================
#define BUZZ_PIN  18      // Passive or active buzzer
#define BEEP_FREQ 1200    // Hz (ignored for active buzzers)
#define BEEP_DUR  40      // ms — short tap feel

// ================================================================
//  JOYSTICK AXIS PINS
// ================================================================
#define LX_PIN  34
#define LY_PIN  35
#define RX_PIN  32
#define RY_PIN  33

// ================================================================
//  DIGITAL INPUT PINS  (INPUT_PULLUP, active LOW)
// ================================================================
#define L_STICK_PIN  26
#define R_STICK_PIN  27
#define L_BTN_PIN    25
#define R_BTN_PIN    14
#define SW1_PIN      16   // Toggle switch 1  ← moved from 18
#define SW2_PIN      17   // Toggle switch 2  ← moved from 19

// ================================================================
//  POTENTIOMETER PIN
// ================================================================
#define POTM1_PIN  39    // Only one pot now (GPIO 36, input-only)

// ================================================================
//  BATTERY
// ================================================================
#define BAT_PIN           36
#define FULL_BAT          8.4
#define EMPTY_BAT         6.6
#define LOW_BAT_THRESHOLD 6.8
#define LOW_BAT_PERCENT   10   // Startup blocks if battery below this %

// ================================================================
//  TUNING CONSTANTS
// ================================================================
#define DEADZONE          11
#define CHANGE_THRESHOLD   2
#define POT_ZERO_THRESH   80   // Raw ADC ≤ this = pot at minimum

// ================================================================
//  BATTERY STATE (global, updated by BatteryManager)
// ================================================================
int BaT = 100;

// ================================================================
//  CUSTOM LCD CHARACTERS
// ================================================================
byte UR_c[8] = {B00000,B00000,B01111,B00011,B00101,B01001,B10000,B00000};
byte U_c[8]  = {B00100,B01110,B10101,B00100,B00100,B00100,B00100,B00100};
byte D_c[8]  = {B00100,B00100,B00100,B00100,B00100,B10101,B01110,B00100};
byte L_c[8]  = {B00000,B00010,B00100,B01000,B11111,B01000,B00100,B00010};
byte R_c[8]  = {B00000,B01000,B00100,B00010,B11111,B00010,B00100,B01000};
byte DL_c[8] = {B00000,B00000,B00001,B10010,B10100,B11000,B11110,B00000};
byte DR_c[8] = {B00000,B00000,B10000,B01001,B00101,B00011,B01111,B00000};
byte UL_c[8] = {B00000,B00000,B11110,B11000,B10100,B10010,B00001,B00000};

// ================================================================
//  DATA STRUCT  — sent over ESP-NOW
// ================================================================
typedef struct 
{
  int  LX, LY;
  int  RX, RY;
  bool LS, RS;
  bool LB, RB;
  bool SW1, SW2;
  int  PotM1;       // Single pot, 0–100
} ControllerData;

ControllerData data;
ControllerData prev = {0};

// ================================================================
//  LCD LINE CACHE
// ================================================================
String line0 = "", line1 = "", line2 = "", line3 = "";

// ================================================================
//  RECEIVER MAC
// ================================================================
uint8_t receiverMAC[] = {0xA4, 0xF0, 0x0F, 0x5F, 0xE0, 0x68};

// ================================================================
//  BEEP  — fires tone() which is timer-driven (non-blocking)
//  Call this function once per edge; it returns immediately.
// ================================================================
void beep() 
{
  tone(BUZZ_PIN, BEEP_FREQ, BEEP_DUR);
}

// ================================================================
//  BEEP ON PRESS  — edge detection for all digital inputs
//  Call after data is freshly read in DataWork().
//  Triggers one beep per LOW→HIGH transition (press/toggle ON).
// ================================================================
void checkBeeps() 
{
  static bool pLB = 0, pRB = 0, pLS = 0, pRS = 0, pSW1 = 0, pSW2 = 0;

  if (data.LB  && !pLB)  beep();
  if (data.RB  && !pRB)  beep();
  if (data.LS  && !pLS)  beep();
  if (data.RS  && !pRS)  beep();
  if (data.SW1 && !pSW1) beep();
  if (data.SW2 && !pSW2) beep();

  pLB  = data.LB;
  pRB  = data.RB;
  pLS  = data.LS;
  pRS  = data.RS;
  pSW1 = data.SW1;
  pSW2 = data.SW2;
}

// ================================================================
//  HELPERS
// ================================================================
int mapAxis(int val) 
{
  int mapped = map(val, 0, 4095, 99, -99);
  if (abs(mapped) < DEADZONE) return 0;
  return mapped;
}

int mapPot(int val) 
{
  return map(val, 0, 4095, 100, 0);
}

void printIfChanged(int col, int row, String text, String &cache) 
{
  if (text != cache) 
  {
    lcd.setCursor(col, row);
    lcd.print(text);
    cache = text;
  }
}

String fmt(int v) 
{
  char buf[6];
  if (v == 0) sprintf(buf, "%3d",  v);
  else        sprintf(buf, "%+3d", v);
  return String(buf);
}

byte getDirIcon(int x, int y) 
{
  if (abs(x) < DEADZONE && abs(y) < DEADZONE) return 255;
  if (y >  DEADZONE && x >  DEADZONE) return 0;
  if (y >  DEADZONE && x < -DEADZONE) return 7;
  if (y < -DEADZONE && x >  DEADZONE) return 6;
  if (y < -DEADZONE && x < -DEADZONE) return 5;
  if (y >  DEADZONE)  return 1;
  if (y < -DEADZONE)  return 2;
  if (x < -DEADZONE)  return 3;
  if (x >  DEADZONE)  return 4;
  return 255;
}
String potBar(int pct, int width = 6) 
{
  int filled = (pct * width) / 100;
  String bar = "P:[";
  for (int i = 0; i < width; i++) {
    bar += (i < filled) ? '=' : '_';
  }
  bar += "]";
  return bar;  // e.g. "P:[===___]"
}
// ================================================================
//  UPDATE DISPLAY
//
//  LCD layout (20 cols × 4 rows):
//  ┌────────────────────┐
//  │L x:+100 y:+100 + L:0│  row 0 — left stick + direction icon
//  │R x:+100 y:+100 + R:0│  row 1 — right stick + direction icon
//  │LB:0 RB:0  S1:0 S2:0│  row 2 — buttons + switches
//  │P1: 100  BAT:  100% │  row 3 — pot + battery %
//  └────────────────────┘
// ================================================================
void updateDisplay() 
{
  char l0[21], l1[21], l2[21], l3[21];

  sprintf(l0, "L x:%s y:%s ", fmt(data.LX).c_str(), fmt(data.LY).c_str());
  sprintf(l1, "R x:%s y:%s ", fmt(data.RX).c_str(), fmt(data.RY).c_str());
  sprintf(l2, "LB:%d RB:%d  S1:%d S2:%d",(int)data.LB, (int)data.RB,(int)data.SW1, (int)data.SW2);
  sprintf(l3, "%s BAT:%3d%%", potBar(data.PotM1).c_str(), BaT);

  printIfChanged(0, 0, l0, line0);
  printIfChanged(0, 1, l1, line1);
  printIfChanged(0, 2, l2, line2);
  printIfChanged(0, 3, l3, line3);

  byte iconL = getDirIcon(data.LX, data.LY);
  byte iconR = getDirIcon(data.RX, data.RY);

  lcd.setCursor(14, 0);
  if (iconL == 255) lcd.print("+"); else lcd.write(iconL);
  lcd.print("  L:");
  lcd.print((int)data.LS);

  lcd.setCursor(14, 1);
  if (iconR == 255) lcd.print("+"); else lcd.write(iconR);
  lcd.print("  R:");
  lcd.print((int)data.RS);
}

// ================================================================
//  ESP-NOW SEND CALLBACK
// ================================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) 
{
  if (status == ESP_NOW_SEND_SUCCESS) 
  {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED,   LOW);
  } else {
    digitalWrite(RED_LED,   HIGH);
    digitalWrite(GREEN_LED, LOW);
  }
}

// ================================================================
//  READ & SEND DATA
// ================================================================
void DataWork() 
{
  int newLX = mapAxis(analogRead(LX_PIN));
  int newLY = mapAxis(analogRead(LY_PIN));
  int newRX = mapAxis(analogRead(RX_PIN));
  int newRY = mapAxis(analogRead(RY_PIN));

  if (abs(newLX - data.LX) > CHANGE_THRESHOLD) data.LX = newLX;
  if (abs(newLY - data.LY) > CHANGE_THRESHOLD) data.LY = newLY;
  if (abs(newRX - data.RX) > CHANGE_THRESHOLD) data.RX = newRX;
  if (abs(newRY - data.RY) > CHANGE_THRESHOLD) data.RY = newRY;

  data.LS  = !digitalRead(L_STICK_PIN);
  data.RS  = !digitalRead(R_STICK_PIN);
  data.LB  = !digitalRead(L_BTN_PIN);
  data.RB  = !digitalRead(R_BTN_PIN);
  data.SW1 = !digitalRead(SW1_PIN);
  data.SW2 = !digitalRead(SW2_PIN);

  data.PotM1 = mapPot(analogRead(POTM1_PIN));

  // Edge-detect beeps (must run after data is updated)
  checkBeeps();

  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

  Serial.printf(
    "L(%d,%d) R(%d,%d) | LB:%d LS:%d RS:%d RB:%d | SW1:%d SW2:%d | P1:%d | BAT:%d%%\n",
    data.LX, data.LY, data.RX, data.RY,
    data.LB, data.LS, data.RS, data.RB,
    data.SW1, data.SW2,
    data.PotM1,
    BaT
  );
}

// ================================================================
//  STARTUP SAFETY CHECK
//
//  Blocks until all of:
//    SW1  = OFF  (HIGH on pullup)
//    SW2  = OFF
//    PotM1 raw ADC ≤ POT_ZERO_THRESH
//    Battery ≥ LOW_BAT_PERCENT (10%)
//
//  LCD layout during check:
//  ┌────────────────────┐
//  │===== Caution! =====│  row 0 — fixed
//  │ SW1:^_^   SW2:FAIL │  row 1 — switch status
//  │ P1 :^_^   BAT: 85% │  row 2 — pot + battery %
//  │====================│  row 3 — fixed border
//  └────────────────────┘
// ================================================================
void startup() 
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("===== Caution! =====");

  while (true) {
    // Read raw battery for startup check
    int rawBat   = analogRead(BAT_PIN);
    float v_adc  = rawBat * (3.3f / 4095.0f);
    float batV   = v_adc * ((100.0f + 47.0f) / 47.0f);
    int   batPct = 0;
    if (batV >= FULL_BAT)       batPct = 100;
    else if (batV <= EMPTY_BAT) batPct = 0;
    else batPct = (int)((batV - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));

    bool sw1Safe  = digitalRead(SW1_PIN);           // HIGH = OFF = safe
    bool sw2Safe  = digitalRead(SW2_PIN);
    int  rawP1    = analogRead(POTM1_PIN);
    bool pot1Safe = (rawP1 >= (4095 - POT_ZERO_THRESH));
    bool batSafe  = (batPct >= LOW_BAT_PERCENT);

    if (sw1Safe && sw2Safe && pot1Safe && batSafe) 
    {
      lcd.setCursor(0, 0); lcd.print("====================");
      lcd.setCursor(0, 1); lcd.print("       Caution!     ");
      lcd.setCursor(0, 2); lcd.print("   ** ALL CLEAR **  ");
      lcd.setCursor(0, 3); lcd.print("====================");
      delay(800);
      lcd.clear();
      return;
    }

    char rowSW[21];
    char rowPT[21];

    // Row 1: switch status  "SW1:^_^  SW2:FAIL"  (20 chars)
    sprintf(rowSW, " SW1:%-4s  SW2:%-4s ",
      sw1Safe ? "^_^" : "FAIL",
      sw2Safe ? "^_^" : "FAIL"
    );

    // Row 2: pot status + battery %  "P1:^_^   BAT: 85%"  (20 chars)
    if (pot1Safe) {
      sprintf(rowPT, " P1 :^_^   BAT:%3d%% ", batPct);
    } else {
      sprintf(rowPT, " P1 :FAIL  BAT:%3d%% ", batPct);
    }

    lcd.setCursor(0, 1); lcd.print(rowSW);
    lcd.setCursor(0, 2); lcd.print(rowPT);
    lcd.setCursor(0, 3); lcd.print("====================");

    delay(150);
  }
}

// ================================================================
//  BATTERY MANAGER
// ================================================================
bool BatteryManager() 
{
  static unsigned long lastRead  = 0;
  static unsigned long lastBlink = 0;
  static bool ledState  = false;
  static bool killSent  = false;
  static float batteryVoltage = 8.4f;

  if (millis() - lastRead > 200) {
    lastRead = millis();
    int raw      = analogRead(BAT_PIN);
    float v_adc  = raw * (3.3f / 4095.0f);
    batteryVoltage = v_adc * ((100.0f + 47.0f) / 47.0f);

    if (batteryVoltage >= FULL_BAT)       BaT = 100;
    else if (batteryVoltage <= EMPTY_BAT) BaT = 0;
    else BaT = (int)((batteryVoltage - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));
  }

  if (batteryVoltage <= LOW_BAT_THRESHOLD) 
  {
    if (!killSent) 
    {
      memset(&data, 0, sizeof(data));
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
      Serial.printf("SAFE MODE — BAT:%d%%\n", BaT);
      killSent = true;
    }

    if (millis() - lastBlink > 300) 
    {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(RED_LED, ledState);
    }
    digitalWrite(GREEN_LED, LOW);

    lcd.setCursor(0, 0); lcd.print("    |=========|     ");
    lcd.setCursor(0, 1); lcd.print("    | LOW BAT |     ");
    lcd.setCursor(0, 2); lcd.print("    |Radio-OFF|     ");
    lcd.setCursor(0, 3); lcd.print("    |=========| ");

    return true;
  }

  killSent = false;
  digitalWrite(RED_LED, LOW);
  return false;
}

// ================================================================
//  SETUP
// ================================================================
void setup() 
{
  Serial.begin(115200);

  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZ_PIN,  OUTPUT);
  digitalWrite(RED_LED,   HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BUZZ_PIN,  LOW);

  pinMode(L_STICK_PIN, INPUT_PULLUP);
  pinMode(R_STICK_PIN, INPUT_PULLUP);
  pinMode(L_BTN_PIN,   INPUT_PULLUP);
  pinMode(R_BTN_PIN,   INPUT_PULLUP);
  pinMode(SW1_PIN,     INPUT_PULLUP);
  pinMode(SW2_PIN,     INPUT_PULLUP);
  pinMode(POTM1_PIN,   INPUT);

  analogSetAttenuation(ADC_11db);

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

  //startup();

  // Boot splash
  lcd.setCursor(0, 0); lcd.print("====================");
  lcd.setCursor(0, 1); lcd.print("  Radio Booting...  ");
  lcd.setCursor(0, 2); lcd.print("    ESP-NOW v0.3    ");
  lcd.setCursor(0, 3); lcd.print("====================");

  // Boot beep: two short tones
  tone(BUZZ_PIN, 1000, 80); delay(130);
  tone(BUZZ_PIN, 1400, 80);
  delay(350);

  WiFi.mode(WIFI_STA);
  delay(150);

  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  esp_now_add_peer(&peerInfo);

  lcd.clear();
}

// ================================================================
//  LOOP  — ~20 Hz
// ================================================================
void loop() 
{
  //if (BatteryManager()) 
    return;
  DataWork();       // reads inputs, calls checkBeeps(), sends ESP-NOW
  updateDisplay();  // smart LCD refresh
  delay(50);
}
