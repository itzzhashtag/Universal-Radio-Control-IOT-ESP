// ================================================================
// ESP-NOW RC Controller  v1.2
// Role  : Master / Transmitter
// Board : ESP32 (38-pin devkit)
// Hardware:
//   2x Analog Joystick  (LX/LY=34/35, RX/RY=32/33)
//   2x Joystick Click   (LS=26, RS=27)
//   2x Push Button      (LB=25, RB=14)
//   2x Toggle Switch    (SW1=16, SW2=17)
//   2x Slider Pot       (P1=39, P2=38)
//   1x Buzzer           (BUZZ=18)
//   1x Battery divider  (BAT=36)
//   2x LED              (RED=13, GREEN=12)
//   Arduino Nano        (TX→GPIO22, 19200 baud)
// Protocol : ESP-NOW (one-way to receiver)
// Changelog v1.2:
//   - Added P2 slider on GPIO 38
//   - LCD removed — display offloaded to Arduino Nano
//   - Nano receives NanoPacket via Serial1 (GPIO 22 TX)
//   - Startup caution blocks on SW1/SW2/P1/P2/BAT
// ================================================================

#include <esp_now.h>
#include <WiFi.h>

// ── PINS ─────────────────────────────────────────
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
#define P1_PIN       39   // Slider 1
#define P2_PIN       38   // Slider 2
#define BAT_PIN      36

// ── NANO SERIAL ───────────────────────────────────
#define NANO_TX_PIN  22
#define NANO_BAUD    19200

// ── BUZZER ────────────────────────────────────────
#define BEEP_FREQ    1200
#define BEEP_DUR     40

// ── BATTERY ──────────────────────────────────────
#define FULL_BAT          8.4f
#define EMPTY_BAT         6.6f
#define LOW_BAT_THRESH    6.8f
#define LOW_BAT_PERCENT   10

// ── TUNING ───────────────────────────────────────
#define DEADZONE          11
#define CHANGE_THRESH      2
#define POT_ZERO_THRESH   80   // raw ADC ≥ (4095-this) = slider at minimum

// ── NANO PACKET STATES ───────────────────────────
#define STATE_NORMAL   0
#define STATE_CAUTION  1
#define STATE_LOWBAT   2
#define STATE_ALLCLEAR 3

// ── GLOBALS ──────────────────────────────────────
int  BaT = 100;
bool espConnected = false;
static unsigned long lastSentOk = 0;

// ── ESP-NOW PAYLOAD ──────────────────────────────
typedef struct {
  int  LX, LY, RX, RY;
  bool LS, RS, LB, RB;
  bool SW1, SW2;
  int  P1, P2;
} ControllerData;

ControllerData data;

// ── NANO DISPLAY PACKET ──────────────────────────
// Sent via Serial1 to Arduino Nano at every loop tick
// 12 bytes total, all single-byte fields (no padding issues)
typedef struct __attribute__((packed)) {
  uint8_t hdr;      // 0xAA — start marker
  uint8_t state;    // STATE_* constants
  int8_t  LX, LY;  // -99..99
  int8_t  RX, RY;
  uint8_t P1, P2;  // 0..100
  uint8_t BAT;     // 0..100
  uint8_t flags;   // b0=LB b1=RB b2=LS b3=RS b4=SW1 b5=SW2 b6=CONN
  uint8_t safety;  // b0=sw1OK b1=sw2OK b2=p1OK b3=p2OK b4=batOK
  uint8_t ftr;     // 0x55 — end marker
} NanoPacket;      // = 12 bytes

uint8_t receiverMAC[] = {0xA4, 0xF0, 0x0F, 0x5F, 0xE0, 0x68};

// ================================================================
//  HELPERS
// ================================================================
int mapAxis(int val) {
  int m = map(val, 0, 4095, 99, -99);
  return (abs(m) < DEADZONE) ? 0 : m;
}

int mapPot(int val) {
  return map(val, 0, 4095, 100, 0);   // inverted: full throw = 0%
}

void beep() {
  tone(BUZZ_PIN, BEEP_FREQ, BEEP_DUR);
}

int readBatPct(float &batVout) {
  int   raw  = analogRead(BAT_PIN);
  float vADC = raw * (3.3f / 4095.0f);
  batVout    = vADC * (147.0f / 47.0f);   // (100+47)/47 voltage divider
  if (batVout >= FULL_BAT)        return 100;
  if (batVout <= EMPTY_BAT)       return 0;
  return (int)((batVout - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));
}

// ================================================================
//  SEND PACKET TO NANO
// ================================================================
void sendNano(uint8_t state, uint8_t safety) {
  NanoPacket pkt;
  pkt.hdr    = 0xAA;
  pkt.state  = state;
  pkt.LX     = (int8_t)constrain(data.LX, -99, 99);
  pkt.LY     = (int8_t)constrain(data.LY, -99, 99);
  pkt.RX     = (int8_t)constrain(data.RX, -99, 99);
  pkt.RY     = (int8_t)constrain(data.RY, -99, 99);
  pkt.P1     = (uint8_t)data.P1;
  pkt.P2     = (uint8_t)data.P2;
  pkt.BAT    = (uint8_t)BaT;
  pkt.flags  = (data.LB       ? 1 : 0)
             | (data.RB       ? 1 : 0) << 1
             | (data.LS       ? 1 : 0) << 2
             | (data.RS       ? 1 : 0) << 3
             | (data.SW1      ? 1 : 0) << 4
             | (data.SW2      ? 1 : 0) << 5
             | (espConnected  ? 1 : 0) << 6;
  pkt.safety = safety;
  pkt.ftr    = 0x55;
  Serial1.write((uint8_t *)&pkt, sizeof(pkt));
}

// ================================================================
//  ESP-NOW CALLBACK
// ================================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSentOk = millis();
    espConnected = true;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED,   LOW);
  } else {
    digitalWrite(RED_LED,   HIGH);
    digitalWrite(GREEN_LED, LOW);
  }
}

// ================================================================
//  BEEP ON DIGITAL PRESS — edge detect, skips analog
// ================================================================
void checkBeeps() {
  static bool pLB=0, pRB=0, pLS=0, pRS=0, pSW1=0, pSW2=0;
  if (data.LB  && !pLB)  beep();
  if (data.RB  && !pRB)  beep();
  if (data.LS  && !pLS)  beep();
  if (data.RS  && !pRS)  beep();
  if (data.SW1 && !pSW1) beep();
  if (data.SW2 && !pSW2) beep();
  pLB=data.LB; pRB=data.RB; pLS=data.LS;
  pRS=data.RS; pSW1=data.SW1; pSW2=data.SW2;
}

// ================================================================
//  STARTUP SAFETY CHECK
//  Blocks until: SW1 off, SW2 off, P1 min, P2 min, BAT ≥ 10%
//  Sends caution packets to Nano while waiting
// ================================================================
void startup() {
  while (true) {
    float batV;
    BaT = readBatPct(batV);

    bool sw1OK = (bool)digitalRead(SW1_PIN);           // HIGH = off = safe
    bool sw2OK = (bool)digitalRead(SW2_PIN);
    bool p1OK  = (analogRead(P1_PIN) >= (4095 - POT_ZERO_THRESH));
    bool p2OK  = (analogRead(P2_PIN) >= (4095 - POT_ZERO_THRESH));
    bool batOK = (BaT >= LOW_BAT_PERCENT);

    uint8_t safety = (sw1OK ? 1 : 0)
                   | (sw2OK ? 1 : 0) << 1
                   | (p1OK  ? 1 : 0) << 2
                   | (p2OK  ? 1 : 0) << 3
                   | (batOK ? 1 : 0) << 4;

    if (sw1OK && sw2OK && p1OK && p2OK && batOK) {
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
//  Returns true and shows low-bat screen if voltage critical
// ================================================================
bool BatteryManager() {
  static unsigned long lastRead  = 0;
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  static bool killSent = false;
  static float batV    = 8.4f;

  if (millis() - lastRead > 200) {
    lastRead = millis();
    BaT = readBatPct(batV);
  }

  espConnected = (millis() - lastSentOk < 1000);

  if (batV <= LOW_BAT_THRESH) {
    if (!killSent) {
      memset(&data, 0, sizeof(data));
      esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));
      Serial.printf("[SAFE MODE] BAT: %d%%\n", BaT);
      killSent = true;
    }
    if (millis() - lastBlink > 300) {
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
//  READ ALL INPUTS, SEND ESP-NOW
// ================================================================
void DataWork() {
  // ─ Joysticks ─
  int nLX = mapAxis(analogRead(LX_PIN));
  int nLY = mapAxis(analogRead(LY_PIN));
  int nRX = mapAxis(analogRead(RX_PIN));
  int nRY = mapAxis(analogRead(RY_PIN));
  if (abs(nLX - data.LX) > CHANGE_THRESH) data.LX = nLX;
  if (abs(nLY - data.LY) > CHANGE_THRESH) data.LY = nLY;
  if (abs(nRX - data.RX) > CHANGE_THRESH) data.RX = nRX;
  if (abs(nRY - data.RY) > CHANGE_THRESH) data.RY = nRY;

  // ─ Digital inputs ─
  data.LS  = !digitalRead(L_STICK_PIN);
  data.RS  = !digitalRead(R_STICK_PIN);
  data.LB  = !digitalRead(L_BTN_PIN);
  data.RB  = !digitalRead(R_BTN_PIN);
  data.SW1 = !digitalRead(SW1_PIN);
  data.SW2 = !digitalRead(SW2_PIN);

  // ─ Sliders ─
  data.P1 = mapPot(analogRead(P1_PIN));
  data.P2 = mapPot(analogRead(P2_PIN));

  checkBeeps();
  esp_now_send(receiverMAC, (uint8_t *)&data, sizeof(data));

  Serial.printf(
    "L(%d,%d) R(%d,%d) | LB:%d LS:%d RS:%d RB:%d | SW1:%d SW2:%d | P1:%d P2:%d | BAT:%d%%\n",
    data.LX, data.LY, data.RX, data.RY,
    data.LB, data.LS, data.RS, data.RB,
    data.SW1, data.SW2, data.P1, data.P2, BaT
  );
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // Serial1 → Nano (TX only, GPIO 22)
  Serial1.begin(NANO_BAUD, SERIAL_8N1, -1, NANO_TX_PIN);

  // ─ Output pins ─
  pinMode(RED_LED,   OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZ_PIN,  OUTPUT);
  digitalWrite(RED_LED,   HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BUZZ_PIN,  LOW);

  // ─ Input pins ─
  pinMode(L_STICK_PIN, INPUT_PULLUP);
  pinMode(R_STICK_PIN, INPUT_PULLUP);
  pinMode(L_BTN_PIN,   INPUT_PULLUP);
  pinMode(R_BTN_PIN,   INPUT_PULLUP);
  pinMode(SW1_PIN,     INPUT_PULLUP);
  pinMode(SW2_PIN,     INPUT_PULLUP);

  analogSetAttenuation(ADC_11db);

  // ─ Power-on beep ─
  tone(BUZZ_PIN, 900,  80); delay(130);
  tone(BUZZ_PIN, 1200, 80); delay(200);

  // ─ Startup safety gate ─
  startup();

  // ─ ESP-NOW init ─
  WiFi.mode(WIFI_STA);
  delay(150);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  esp_now_add_peer(&peerInfo);

  // ─ Ready beep ─
  tone(BUZZ_PIN, 1200, 60); delay(100);
  tone(BUZZ_PIN, 1600, 80); delay(200);
}

// ================================================================
//  LOOP — ~20 Hz
// ================================================================
void loop() {
  if (BatteryManager()) return;   // low bat: show warning, skip rest
  DataWork();                      // read inputs + send ESP-NOW
  sendNano(STATE_NORMAL, 0);       // send display packet to Nano
  delay(50);
}