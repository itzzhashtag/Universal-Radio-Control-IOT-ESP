// ================================================================
//  ESP-NOW RC Controller  v1.2  — DEBUG EDITION
//  Role    : Master / Transmitter
//  Board   : ESP32 (38-pin devkit)
//
//  ── DEBUG FEATURES ADDED ────────────────────────────────────────
//  #define DEBUG          → enables all Serial.print debug output
//  #define DEBUG_TEST     → enables TEST MODE (synthetic data loop)
//                           automatically kicks in at boot when defined
//
//  TEST MODE behaviour (DEBUG_TEST):
//    • Bypasses real sensor reads in DataWork()
//    • runTestMode() generates a scripted sweep of all axes,
//      buttons, and sliders so you can verify the Nano display
//      without physically moving any hardware.
//    • Hold SW1 + SW2 simultaneously at any time to toggle
//      TEST MODE on / off at runtime (requires DEBUG defined).
//
//  Serial output format (DEBUG):
//    [PKT] L(lx,ly) R(rx,ry) | LB:x LS:x RS:x RB:x | SW1:x SW2:x
//          | P1:xx P2:xx | BAT:xx% | CONN:x | STATE:x
//    [BAT] raw voltage and percent on every battery read
//    [SAFE] low-battery kill event
//    [TEST] each synthetic frame description
//    [NANO] every packet sent to Nano (hex dump)
//    [CB]   ESP-NOW send callback result
// ================================================================

#include <esp_now.h>
#include <WiFi.h>

// ── DEBUG SWITCHES ───────────────────────────────────────────────
// Comment out either line to disable that feature

#define DEBUG          // master switch: enables all Serial debug prints
#define DEBUG_TEST     // enables the synthetic test-mode function

// ── Conditional print macros ─────────────────────────────────────
#ifdef DEBUG
  #define DBG(...)     Serial.print(__VA_ARGS__)
  #define DBGLN(...)   Serial.println(__VA_ARGS__)
  #define DBGF(...)    Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
  #define DBGF(...)
#endif

// ── PIN DEFINITIONS ──────────────────────────────────────────────
#define RED_LED      13
#define GREEN_LED    12
#define BUZZ_PIN     18

#define LX_PIN       34
#define LY_PIN       35
#define RX_PIN       32
#define RY_PIN       33

#define L_STICK_PIN  26
#define R_STICK_PIN  27
#define L_BTN_PIN    25
#define R_BTN_PIN    14
#define SW1_PIN      16
#define SW2_PIN      17

#define P1_PIN       39
#define P2_PIN       38
#define BAT_PIN      36

// ── NANO SERIAL ──────────────────────────────────────────────────
#define NANO_TX_PIN  22
#define NANO_BAUD    19200

// ── BUZZER ───────────────────────────────────────────────────────
#define BEEP_FREQ    1200
#define BEEP_DUR     40

// ── BATTERY ──────────────────────────────────────────────────────
#define FULL_BAT          8.4f
#define EMPTY_BAT         6.6f
#define LOW_BAT_THRESH    6.8f
#define LOW_BAT_PERCENT   10

// ── JOYSTICK TUNING ──────────────────────────────────────────────
#define DEADZONE          11
#define CHANGE_THRESH      2
#define POT_ZERO_THRESH   80   // raw ADC ≥ (4095 - this) = slider at minimum

// ── DISPLAY PACKET STATES ────────────────────────────────────────
#define STATE_NORMAL   0
#define STATE_CAUTION  1
#define STATE_LOWBAT   2
#define STATE_ALLCLEAR 3

// ── GLOBALS ──────────────────────────────────────────────────────
int  BaT          = 100;
bool espConnected = false;
static unsigned long lastSentOk = 0;

// testMode flag — toggled by SW1+SW2 hold, or forced on by DEBUG_TEST at boot
bool testMode = false;

// ── ESP-NOW PAYLOAD ──────────────────────────────────────────────
typedef struct {
  int  LX, LY, RX, RY;
  bool LS, RS, LB, RB;
  bool SW1, SW2;
  int  P1, P2;
} ControllerData;

ControllerData data;

// ── NANO DISPLAY PACKET (12 bytes, packed) ───────────────────────
typedef struct __attribute__((packed)) {
  uint8_t hdr;      // 0xAA  start marker
  uint8_t state;    // STATE_*
  int8_t  LX, LY;  // -99..99
  int8_t  RX, RY;
  uint8_t P1, P2;  // 0..100
  uint8_t BAT;     // 0..100
  uint8_t flags;   // b0=LB b1=RB b2=LS b3=RS b4=SW1 b5=SW2 b6=CONN
  uint8_t safety;  // b0=sw1OK b1=sw2OK b2=p1OK b3=p2OK b4=batOK
  uint8_t ftr;     // 0x55  end marker
} NanoPacket;

// ── RECEIVER MAC — update to match your hexapod ESP32 ────────────
uint8_t receiverMAC[] = {0xA4, 0xF0, 0x0F, 0x5F, 0xE0, 0x68};

// ================================================================
//  DEBUG HELPER — hex dump any buffer to Serial
// ================================================================
#ifdef DEBUG
void debugHexDump(const char *label, const uint8_t *buf, size_t len)
{
  DBG(label);
  DBG(" [ ");
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) DBG("0");
    DBG(buf[i], HEX);
    DBG(" ");
  }
  DBGLN("]");
}
#endif

// ================================================================
//  HELPERS
// ================================================================
int mapAxis(int val)
{
  int m = map(val, 0, 4095, 99, -99);
  return (abs(m) < DEADZONE) ? 0 : m;
}

int mapPot(int val)
{
  return map(val, 0, 4095, 100, 0);
}

void beep()
{
  tone(BUZZ_PIN, BEEP_FREQ, BEEP_DUR);
}

int readBatPct(float &batVout)
{
  int   raw  = analogRead(BAT_PIN);
  float vADC = raw * (3.3f / 4095.0f);
  batVout    = vADC * (147.0f / 47.0f);  // (100k+47k)/47k divider

  DBGF("[BAT] raw=%d  vADC=%.3fV  batV=%.3fV\n", raw, vADC, batVout);

  if (batVout >= FULL_BAT)  return 100;
  if (batVout <= EMPTY_BAT) return 0;
  return (int)((batVout - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));
}

// ================================================================
//  SEND DISPLAY PACKET TO NANO
//  Builds the 12-byte NanoPacket and writes it to Serial1.
//  In DEBUG mode also prints a hex dump.
// ================================================================
void sendNano(uint8_t state, uint8_t safety)
{
  NanoPacket pkt;
  pkt.hdr    = 0xAA;
  pkt.state  = state;
  pkt.LX     = (int8_t)constrain(data.LX, -99, 99);
  pkt.LY     = (int8_t)constrain(data.LY, -99, 99);
  pkt.RX     = (int8_t)constrain(data.RX, -99, 99);
  pkt.RY     = (int8_t)constrain(data.RY, -99, 99);
  pkt.P1     = (uint8_t)constrain(data.P1, 0, 100);
  pkt.P2     = (uint8_t)constrain(data.P2, 0, 100);
  pkt.BAT    = (uint8_t)BaT;
  pkt.flags  = ((uint8_t)data.LB      << 0)
             | ((uint8_t)data.RB      << 1)
             | ((uint8_t)data.LS      << 2)
             | ((uint8_t)data.RS      << 3)
             | ((uint8_t)data.SW1     << 4)
             | ((uint8_t)data.SW2     << 5)
             | ((uint8_t)espConnected << 6);
  pkt.safety = safety;
  pkt.ftr    = 0x55;

  Serial1.write((uint8_t *)&pkt, sizeof(pkt));

  // Print hex dump of every packet sent to Nano
  #ifdef DEBUG
    debugHexDump("[NANO]", (uint8_t *)&pkt, sizeof(pkt));
  #endif
}

// ================================================================
//  ESP-NOW SEND CALLBACK
// ================================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSentOk   = millis();
    espConnected = true;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED,   LOW);
    DBGLN("[CB] ESP-NOW → SUCCESS");
  } else {
    digitalWrite(RED_LED,   HIGH);
    digitalWrite(GREEN_LED, LOW);
    DBGLN("[CB] ESP-NOW → FAIL");
  }
}

// ================================================================
//  BEEP ON DIGITAL PRESS — edge detect only
// ================================================================
void checkBeeps()
{
  static bool pLB=0, pRB=0, pLS=0, pRS=0, pSW1=0, pSW2=0;

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
//  TEST MODE TOGGLE (runtime, via SW1+SW2 held simultaneously)
//  Only compiled when DEBUG is defined.
// ================================================================
#ifdef DEBUG
void checkTestModeToggle()
{
  static bool lastBothHeld = false;
  // Read the physical pins directly — bypass data struct
  bool sw1 = !digitalRead(SW1_PIN);
  bool sw2 = !digitalRead(SW2_PIN);
  bool bothHeld = sw1 && sw2;

  if (bothHeld && !lastBothHeld) {
    testMode = !testMode;
    DBGF("[TEST] Test mode is now %s\n", testMode ? "ON" : "OFF");
    beep(); delay(60); beep();   // double beep confirmation
  }
  lastBothHeld = bothHeld;
}
#endif

// ================================================================
//  RUN TEST MODE — synthetic data sweep
//  Fills the global `data` struct with scripted patterns instead
//  of reading real hardware.  One "scene" every 40 frames (~2 s).
//
//  Scene list:
//   0  — centre all axes, all buttons off, P1/P2=50, BAT=80
//   1  — LX/LY sweep +99, RX/RY sweep -99
//   2  — LX/LY sweep -99, RX/RY sweep +99
//   3  — all buttons ON, P1/P2=100
//   4  — all buttons OFF, P1=0 P2=100
//   5  — BAT=15 (triggers low-bat blink on display)
//   6  — BAT=100, back to normal
// ================================================================
#ifdef DEBUG_TEST
void runTestMode()
{
  static uint32_t lastScene = 0;
  static uint8_t  scene     = 0;
  static int      sweepVal  = 0;
  static int      sweepDir  = 1;

  // Scene advances every 2 seconds
  if (millis() - lastScene > 2000) {
    lastScene = millis();
    scene = (scene + 1) % 7;
    DBGF("[TEST] Scene %d\n", scene);
  }

  // Smooth sweep value (±99), steps by 3 per frame
  sweepVal += sweepDir * 3;
  if (sweepVal >=  99) { sweepVal =  99; sweepDir = -1; }
  if (sweepVal <= -99) { sweepVal = -99; sweepDir =  1; }

  // Zero everything first, then apply scene overrides
  memset(&data, 0, sizeof(data));

  switch (scene)
  {
    case 0:  // All centred, mid sliders
      data.LX = 0;  data.LY = 0;
      data.RX = 0;  data.RY = 0;
      data.P1 = 50; data.P2 = 50;
      BaT = 80;
      DBGLN("[TEST] Scene 0: CENTRE");
      break;

    case 1:  // L stick sweeps positive, R stick sweeps negative
      data.LX = sweepVal;  data.LY = sweepVal;
      data.RX = -sweepVal; data.RY = -sweepVal;
      data.P1 = (uint8_t)map(sweepVal, -99, 99, 0, 100);
      data.P2 = 50;
      BaT = 80;
      DBGF("[TEST] Scene 1: SWEEP L=%d R=%d\n", sweepVal, -sweepVal);
      break;

    case 2:  // Opposite sweep
      data.LX = -sweepVal; data.LY = -sweepVal;
      data.RX = sweepVal;  data.RY = sweepVal;
      data.P1 = 50;
      data.P2 = (uint8_t)map(sweepVal, -99, 99, 100, 0);
      BaT = 80;
      DBGF("[TEST] Scene 2: SWEEP L=%d R=%d\n", -sweepVal, sweepVal);
      break;

    case 3:  // All buttons ON
      data.LB = true; data.RB = true;
      data.LS = true; data.RS = true;
      data.SW1 = true; data.SW2 = true;
      data.P1 = 100; data.P2 = 100;
      BaT = 75;
      DBGLN("[TEST] Scene 3: ALL BUTTONS ON");
      break;

    case 4:  // Partial: P1 min, P2 max
      data.LB = false; data.RB = true;
      data.SW1 = false; data.SW2 = true;
      data.P1 = 0;  data.P2 = 100;
      BaT = 60;
      DBGLN("[TEST] Scene 4: P1=0 P2=100");
      break;

    case 5:  // Low battery warning scenario
      data.P1 = 50; data.P2 = 50;
      BaT = 15;   // below 20% → display bat icon blinks
      DBGLN("[TEST] Scene 5: LOW BATTERY");
      break;

    case 6:  // Back to normal full battery
      data.P1 = 50; data.P2 = 50;
      BaT = 100;
      DBGLN("[TEST] Scene 6: FULL BAT");
      break;
  }
}
#endif  // DEBUG_TEST

// ================================================================
//  PRINT CONTROLLER STATE — full readable dump for Serial monitor
// ================================================================
#ifdef DEBUG
void printControllerState(uint8_t nanoState)
{
  DBGF(
    "[PKT] L(%3d,%3d) R(%3d,%3d) | LB:%d LS:%d RS:%d RB:%d"
    " | SW1:%d SW2:%d | P1:%3d P2:%3d | BAT:%3d%% | CONN:%d | STATE:%d%s\n",
    data.LX, data.LY, data.RX, data.RY,
    data.LB, data.LS, data.RS, data.RB,
    data.SW1, data.SW2,
    data.P1, data.P2,
    BaT,
    (int)espConnected,
    nanoState,
    testMode ? " [TEST]" : ""
  );
}
#endif

// ================================================================
//  STARTUP SAFETY CHECK
// ================================================================
void startup()
{
  DBGLN("[STARTUP] Waiting for safe start conditions...");

  while (true)
  {
    float batV;
    BaT = readBatPct(batV);

    bool sw1OK = (bool)digitalRead(SW1_PIN);
    bool sw2OK = (bool)digitalRead(SW2_PIN);
    bool p1OK  = (analogRead(P1_PIN) >= (4095 - POT_ZERO_THRESH));
    bool p2OK  = (analogRead(P2_PIN) >= (4095 - POT_ZERO_THRESH));
    bool batOK = (BaT >= LOW_BAT_PERCENT);

    DBGF("[STARTUP] SW1:%d SW2:%d P1ok:%d P2ok:%d batOK:%d  BAT=%d%% batV=%.2fV\n",
         sw1OK, sw2OK, p1OK, p2OK, batOK, BaT, batV);

    uint8_t safety = ((uint8_t)sw1OK << 0)
                   | ((uint8_t)sw2OK << 1)
                   | ((uint8_t)p1OK  << 2)
                   | ((uint8_t)p2OK  << 3)
                   | ((uint8_t)batOK << 4);

    if (sw1OK && sw2OK && p1OK && p2OK && batOK)
    {
      DBGLN("[STARTUP] All clear — sending STATE_ALLCLEAR to Nano");
      sendNano(STATE_ALLCLEAR, 0xFF);
      delay(900);
      return;
    }

    sendNano(STATE_CAUTION, safety);
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
  static bool ledState = false;
  static bool killSent = false;
  static float batV    = 8.4f;

  if (millis() - lastRead > 200)
  {
    lastRead = millis();
    BaT = readBatPct(batV);
  }

  espConnected = (millis() - lastSentOk < 1000);

  if (batV <= LOW_BAT_THRESH)
  {
    if (!killSent)
    {
      memset(&data, 0, sizeof(data));
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
      DBGF("[SAFE] LOW BATTERY KILL — BAT: %d%%  V=%.2f\n", BaT, batV);
      killSent = true;
    }
    if (millis() - lastBlink > 300)
    {
      lastBlink = millis();
      ledState  = !ledState;
      digitalWrite(RED_LED, ledState);
    }
    digitalWrite(GREEN_LED, LOW);
    sendNano(STATE_LOWBAT, 0);
    return true;
  }

  killSent = false;
  digitalWrite(RED_LED, LOW);
  return false;
}

// ================================================================
//  DATA WORK — read all inputs, send ESP-NOW, check beeps
// ================================================================
void DataWork()
{
  #ifdef DEBUG_TEST
  // ── TEST MODE override: skip real hardware reads ──────────────
  if (testMode) {
    runTestMode();     // fills data + BaT with synthetic values
    // still send over ESP-NOW so receiver also gets test data
    esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
    return;            // skip the real sensor block below
  }
  #endif

  // ── REAL HARDWARE READS ───────────────────────────────────────

  // Joystick axes with deadzone and change threshold
  int nLX = mapAxis(analogRead(LX_PIN));
  int nLY = mapAxis(analogRead(LY_PIN));
  int nRX = mapAxis(analogRead(RX_PIN));
  int nRY = mapAxis(analogRead(RY_PIN));
  if (abs(nLX - data.LX) > CHANGE_THRESH) data.LX = nLX;
  if (abs(nLY - data.LY) > CHANGE_THRESH) data.LY = nLY;
  if (abs(nRX - data.RX) > CHANGE_THRESH) data.RX = nRX;
  if (abs(nRY - data.RY) > CHANGE_THRESH) data.RY = nRY;

  // Digital inputs — active LOW, inverted here
  data.LS  = !digitalRead(L_STICK_PIN);
  data.RS  = !digitalRead(R_STICK_PIN);
  data.LB  = !digitalRead(L_BTN_PIN);
  data.RB  = !digitalRead(R_BTN_PIN);
  data.SW1 = !digitalRead(SW1_PIN);
  data.SW2 = !digitalRead(SW2_PIN);

  // Slider pots
  data.P1 = mapPot(analogRead(P1_PIN));
  data.P2 = mapPot(analogRead(P2_PIN));

  checkBeeps();

  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);    // USB debug monitor
  DBGLN("=== ESP32 RC Controller v1.2 DEBUG EDITION ===");
  DBGF("DEBUG_TEST mode: %s\n", 
  #ifdef DEBUG_TEST
    "COMPILED IN"
  #else
    "disabled"
  #endif
  );

  // Serial1 → Nano, TX only on GPIO 22
  Serial1.begin(NANO_BAUD, SERIAL_8N1, -1, NANO_TX_PIN);

  // Output pins
  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZ_PIN,  OUTPUT);
  digitalWrite(RED_LED,   HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BUZZ_PIN,  LOW);

  // Input pins
  pinMode(L_STICK_PIN, INPUT_PULLUP);
  pinMode(R_STICK_PIN, INPUT_PULLUP);
  pinMode(L_BTN_PIN,   INPUT_PULLUP);
  pinMode(R_BTN_PIN,   INPUT_PULLUP);
  pinMode(SW1_PIN,     INPUT_PULLUP);
  pinMode(SW2_PIN,     INPUT_PULLUP);

  analogSetAttenuation(ADC_11db);

  // Power-on double beep
  tone(BUZZ_PIN, 900,  80); delay(130);
  tone(BUZZ_PIN, 1200, 80); delay(200);

  #ifdef DEBUG_TEST
    testMode = true;   // auto-enable test mode at boot when compiled with DEBUG_TEST
    DBGLN("[TEST] Test mode AUTO-ENABLED at boot (DEBUG_TEST defined)");
  #endif

  startup();

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  delay(150);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  esp_now_add_peer(&peerInfo);

  // Ready beeps
  tone(BUZZ_PIN, 1200, 60); delay(100);
  tone(BUZZ_PIN, 1600, 80); delay(200);

  DBGLN("[SETUP] Setup complete. Entering loop.");
}

// ================================================================
//  LOOP — ~20 Hz
// ================================================================
void loop()
{
  // Check for SW1+SW2 held to toggle test mode (DEBUG only)
  #ifdef DEBUG
    checkTestModeToggle();
  #endif

  if (BatteryManager()) return;

  DataWork();

  // Print full state to Serial monitor (throttled to ~4 Hz to avoid spam)
  #ifdef DEBUG
  {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 250) {
      lastPrint = millis();
      printControllerState(STATE_NORMAL);
    }
  }
  #endif

  sendNano(STATE_NORMAL, 0);

  delay(50);
}
