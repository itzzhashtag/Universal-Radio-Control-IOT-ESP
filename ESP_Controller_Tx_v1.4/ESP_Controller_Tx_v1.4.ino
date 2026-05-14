// ================================================================
//  ESP-NOW RC Controller  v1.2
//  Role    : Master / Transmitter
//  Board   : ESP32 (38-pin devkit)
//
//  Hardware:
//    2x Analog Joystick  LX=34  LY=35  RX=32  RY=33
//    2x Joystick Click   LS=26  RS=27
//    2x Push Button      LB=25  RB=14
//    2x Toggle Switch    SW1=16 SW2=17
//    2x Slider Pot       P1=39  P2=38
//    1x Buzzer           BUZZ=18
//    1x Battery divider  BAT=36  (100kΩ + 47kΩ voltage divider)
//    2x LED              RED=13  GREEN=12
//    Arduino Nano        TX → GPIO22  (19200 baud, display link)
//
//  Protocol : ESP-NOW one-way → receiver hexapod
//  Display  : offloaded to Arduino Nano via Serial1 (GPIO 22 TX only)
// ================================================================

#include <esp_now.h>
#include <WiFi.h>

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
int  BaT         = 100;
bool espConnected = false;
static unsigned long lastSentOk = 0;

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
//  HELPERS
// ================================================================
int mapAxis(int val)
{
  int m = map(val, 0, 4095, 99, -99);
  return (abs(m) < DEADZONE) ? 0 : m;
}

int mapPot(int val)
{
  // Full throw (slider top) = 100, minimum (slider bottom) = 0
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
  batVout    = vADC * (147.0f / 47.0f);   // voltage divider: (100k+47k)/47k
  if (batVout >= FULL_BAT)  return 100;
  if (batVout <= EMPTY_BAT) return 0;
  return (int)((batVout - EMPTY_BAT) * 100.0f / (FULL_BAT - EMPTY_BAT));
}

// ================================================================
//  SEND DISPLAY PACKET TO NANO
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
  } else {
    digitalWrite(RED_LED,   HIGH);
    digitalWrite(GREEN_LED, LOW);
  }
}

// ================================================================
//  BEEP ON DIGITAL PRESS — edge detect only, no analog
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
//  STARTUP SAFETY CHECK
//  Blocks until: SW1 off, SW2 off, P1 at min, P2 at min, BAT ≥ 10%
//  Streams STATE_CAUTION packets to Nano while waiting.
// ================================================================
void startup()
{
  while (true)
  {
    float batV;
    BaT = readBatPct(batV);

    bool sw1OK = (bool)digitalRead(SW1_PIN);                       // HIGH = OFF = safe
    bool sw2OK = (bool)digitalRead(SW2_PIN);
    bool p1OK  = (analogRead(P1_PIN) >= (4095 - POT_ZERO_THRESH)); // slider at minimum
    bool p2OK  = (analogRead(P2_PIN) >= (4095 - POT_ZERO_THRESH));
    bool batOK = (BaT >= LOW_BAT_PERCENT);

    uint8_t safety = ((uint8_t)sw1OK << 0)
                   | ((uint8_t)sw2OK << 1)
                   | ((uint8_t)p1OK  << 2)
                   | ((uint8_t)p2OK  << 3)
                   | ((uint8_t)batOK << 4);

    if (sw1OK && sw2OK && p1OK && p2OK && batOK)
    {
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
//  Returns true (and streams STATE_LOWBAT) when voltage is critical.
//  Caller should skip DataWork / sendNano(NORMAL) when true.
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
      Serial.printf("[SAFE MODE] BAT: %d%%\n", BaT);
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

  Serial.printf(
    "L(%d,%d) R(%d,%d) | LB:%d LS:%d RS:%d RB:%d | SW1:%d SW2:%d | P1:%d P2:%d | BAT:%d%%\n",
    data.LX, data.LY, data.RX, data.RY,
    data.LB, data.LS, data.RS, data.RB,
    data.SW1, data.SW2,
    data.P1, data.P2,
    BaT
  );
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);

  // Serial1 → Nano, TX only on GPIO 22 (RX unused, set to -1)
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

  // Block here until all controls are in safe start position
  startup();

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  delay(150);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  esp_now_add_peer(&peerInfo);

  // Ready double beep (higher pitch)
  tone(BUZZ_PIN, 1200, 60); delay(100);
  tone(BUZZ_PIN, 1600, 80); delay(200);
}

// ================================================================
//  LOOP — ~20 Hz
// ================================================================
void loop()
{
  if (BatteryManager()) return;   // critical battery: stream warning, skip rest

  DataWork();                      // read inputs + send ESP-NOW
  sendNano(STATE_NORMAL, 0);       // stream display packet to Nano

  delay(50);
}
