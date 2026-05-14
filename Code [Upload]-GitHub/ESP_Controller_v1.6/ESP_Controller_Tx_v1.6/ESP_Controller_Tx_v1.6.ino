// ================================================================
//  ESP32 Universal RC Controller — TX v1.0
//  Role    : Transmitter / Master
//  Board   : ESP32 (38-pin devkit)
//
//  Hardware on ESP1:
//    2x Analog Joystick   LX=34 LY=35 / RX=32 RY=33
//    4x Push Buttons      LABt=26  RABt=27  LBt=25  RBt=14
//    2x Toggle Switch     TSW1=16  TSW2=17
//    2x Slider Pot        Pot1=39  Pot2=38
//    1x Battery Divider   BAT=36   (100kΩ + 47kΩ)
//    1x RED LED           GPIO 2
//    1x GREEN LED         GPIO 15
//    1x JHD12864E LCD     SPI mode → GPIO 5/18/23/4
//       ↳ No Nano needed — ESP32 has enough pins to drive LCD directly
//
//  Protocol : ESP-NOW → all receivers (broadcast) or specific MAC
//  Library  : U8g2 by olikraus (install via Library Manager)
//
//  ── WIRING — JHD12864E / ST7920 128×64 LCD ─────────────────────
//
//  LCD 20-pin connector:
//  ┌──────┬───────────────────────────────────────────────────────┐
//  │ Pin  │ Connect to                                            │
//  ├──────┼───────────────────────────────────────────────────────┤
//  │  1 VSS  │ GND                                               │
//  │  2 VDD  │ 5V  (or 3.3V if your module is 3.3V tolerant)    │
//  │  3 V0   │ Wiper of 10kΩ contrast pot                        │
//  │         │   Left leg → GND    Right leg → 5V               │
//  │  4 RS   │ ESP32 GPIO  5  (SPI CS)                           │
//  │  5 R/W  │ ESP32 GPIO 23  (SPI MOSI / SID)                  │
//  │  6 E    │ ESP32 GPIO 18  (SPI SCK / CLK)                   │
//  │ 7-14 DB0-DB7 → NC  (unused in SPI mode)                    │
//  │ 15 PSB  │ GND  ← MUST be GND to enable SPI mode !          │
//  │ 16 NC   │ NC                                                │
//  │ 17 RST  │ ESP32 GPIO  4  (or tie to VDD to skip reset)      │
//  │ 18 VOUT │ NC  (optionally 0.1µF cap to GND for booster)     │
//  │ 19 BLA  │ 5V via 10Ω resistor  (backlight anode)           │
//  │ 20 BLK  │ GND  (backlight cathode)                          │
//  └──────────┴───────────────────────────────────────────────────┘
//
//  ── ESP32 Full Pin Map ──────────────────────────────────────────
//  GPIO 34 → LX  joystick-L X-axis   (ADC1, input-only)
//  GPIO 35 → LY  joystick-L Y-axis   (ADC1, input-only)
//  GPIO 32 → RX  joystick-R X-axis   (ADC1)
//  GPIO 33 → RY  joystick-R Y-axis   (ADC1)
//  GPIO 39 → Pot1 slider              (ADC1, input-only)
//  GPIO 38 → Pot2 slider              (ADC1, input-only)
//  GPIO 36 → Battery sense            (ADC1, input-only)
//            Divider: VBAT ──100kΩ──┬── GPIO36
//                                   └── 47kΩ ── GND
//  GPIO 26 → LABt  (INPUT_PULLUP, button other end → GND)
//  GPIO 27 → RABt  (INPUT_PULLUP)
//  GPIO 25 → LBt   (INPUT_PULLUP)
//  GPIO 14 → RBt   (INPUT_PULLUP)
//  GPIO 16 → TSW1  (INPUT_PULLUP, toggle one end → GND)
//  GPIO 17 → TSW2  (INPUT_PULLUP)
//  GPIO  2 → RED LED  (100Ω → GND)
//  GPIO 15 → GREEN LED (100Ω → GND)
//  GPIO  5 → LCD RS/CS
//  GPIO 18 → LCD E/CLK    ← VSPI SCK  (fixed, don't move)
//  GPIO 23 → LCD R/W/SID  ← VSPI MOSI (fixed, don't move)
//  GPIO  4 → LCD RST
//
//  ── Joystick module (KY-023 style) ─────────────────────────────
//  GND → GND,  VCC → 3.3V,  VRx → LX_PIN,  VRy → LY_PIN
//  SW  → LABt/RABt pin (INPUT_PULLUP handles the pullup)
//  !! Use 3.3V not 5V — ESP32 ADC is 3.3V max !!
// ================================================================

#include <U8g2lib.h>
#include <esp_now.h>
#include <WiFi.h>

// ================================================================
//  PAYLOAD STRUCT  — must be identical in TX and all receivers
// ================================================================
typedef struct {
  int  Lx, Ly, Rx, Ry;   // Joystick axes        -99 .. 99
  bool LABt, RABt;         // A-buttons left/right
  bool LBt,  RBt;          // B-buttons left/right
  bool TSW1, TSW2;          // Toggle switches
  int  Pot1, Pot2;          // Potentiometers       0 .. 100
  int  BAT;                 // Battery percent      0 .. 100
} ControllerData;
 

ControllerData txData;      // live outgoing data

// ================================================================
//  BROADCAST / UNICAST SELECTION
//  BROADCAST 1 → FF:FF:FF:FF:FF:FF  — any receiver picks it up
//  BROADCAST 0 → send only to peerMAC below
// ================================================================
#define BROADCAST  1        // ← set to 0 for targeted send

#if BROADCAST
  uint8_t peerMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#else
  uint8_t peerMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};  // ← paste receiver MAC
#endif

// ================================================================
//  PINS
// ================================================================
#define PIN_LX    34
#define PIN_LY    35
#define PIN_RX    32
#define PIN_RY    33
#define PIN_LABt  26
#define PIN_RABt  27
#define PIN_LBt   25
#define PIN_RBt   14
#define PIN_TSW1  16
#define PIN_TSW2  17
#define PIN_POT1  39
#define PIN_POT2  38
#define PIN_BAT   36
#define PIN_LED_R  2
#define PIN_LED_G 15
#define PIN_LCD_CS   5    // LCD RS/CS
#define PIN_LCD_RST  4    // LCD RST

// ================================================================
//  TUNING
// ================================================================
#define DEADZONE         11   // joystick dead-band (mapped units)
#define CHANGE_THRESH     2   // min axis change to update
#define POT_ZERO_THRESH  80   // raw ADC ≥ (4095 - this) = slider at min

#define BAT_FULL     8.4f
#define BAT_EMPTY    6.6f
#define BAT_LOW_V    6.8f    // voltage → kill radio
#define BAT_LOW_PCT  10      // startup blocks below this %

// ================================================================
//  U8g2  — ST7920, full buffer, hardware SPI
//  VSPI bus: MOSI=GPIO23  SCK=GPIO18  CS=PIN_LCD_CS  RST=PIN_LCD_RST
// ================================================================
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, PIN_LCD_CS, PIN_LCD_RST);

// ================================================================
//  GLOBALS
// ================================================================
int  gBatPct    = 100;
bool gConnected = false;
static unsigned long lastSentOk = 0;

// Light IIR smooth values for display
static int sLx=0, sLy=0, sRx=0, sRy=0;
static int sP1=50, sP2=50, sBat=100;
static uint8_t animTick = 0;
static bool    batBlink = false;

// ================================================================
//  HELPERS
// ================================================================
int mapAxis(int raw)
{
  int v = map(raw, 0, 4095, 99, -99);
  return (abs(v) < DEADZONE) ? 0 : v;
}

int mapPot(int raw)
{
  // Full physical throw → 100, minimum → 0
  return map(raw, 0, 4095, 100, 0);
}

int iirSmooth(int cur, int tgt)
{
  int d = tgt - cur;
  return (d == 0 || abs(d) <= 1) ? tgt : cur + d / 4;
}

int readBatPct(float &vOut)
{
  float vADC = analogRead(PIN_BAT) * (3.3f / 4095.0f);
  vOut = vADC * (147.0f / 47.0f);   // voltage divider ratio: (100k+47k)/47k
  if (vOut >= BAT_FULL)  return 100;
  if (vOut <= BAT_EMPTY) return 0;
  return (int)((vOut - BAT_EMPTY) * 100.0f / (BAT_FULL - BAT_EMPTY));
}

// ================================================================
//  ESP-NOW SEND CALLBACK
//  NOTE: If you are on ESP32 Arduino Core 3.x and get a compile
//  error here, change the first argument to:
//  const wifi_tx_info_t *info   (and remove the mac_addr usage below)
// ================================================================
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
  bool ok = (status == ESP_NOW_SEND_SUCCESS);

  if (ok) {
    lastSentOk = millis();
    gConnected = true;
    digitalWrite(PIN_LED_G, HIGH);
    digitalWrite(PIN_LED_R, LOW);
  } else {
    gConnected = false;
    digitalWrite(PIN_LED_R, HIGH);
    digitalWrite(PIN_LED_G, LOW);
  }
  if (info) 
  {
    Serial.printf("To: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  info->des_addr[0], info->des_addr[1], info->des_addr[2],
                  info->des_addr[3], info->des_addr[4], info->des_addr[5]);
  }
}
 
// ================================================================
//  READ ALL INPUTS & SEND ESP-NOW
// ================================================================
void readAndSend()
{
  // ── Joystick axes — change threshold prevents jitter noise ───
  int nLx = mapAxis(analogRead(PIN_LX));
  int nLy = mapAxis(analogRead(PIN_LY));
  int nRx = mapAxis(analogRead(PIN_RX));
  int nRy = mapAxis(analogRead(PIN_RY));

  if (abs(nLx - txData.Lx) > CHANGE_THRESH) txData.Lx = nLx;
  if (abs(nLy - txData.Ly) > CHANGE_THRESH) txData.Ly = nLy;
  if (abs(nRx - txData.Rx) > CHANGE_THRESH) txData.Rx = nRx;
  if (abs(nRy - txData.Ry) > CHANGE_THRESH) txData.Ry = nRy;

  // ── Digital inputs — active LOW, inverted here ───────────────
  txData.LABt = !digitalRead(PIN_LABt);
  txData.RABt = !digitalRead(PIN_RABt);
  txData.LBt  = !digitalRead(PIN_LBt);
  txData.RBt  = !digitalRead(PIN_RBt);
  txData.TSW1 = !digitalRead(PIN_TSW1);
  txData.TSW2 = !digitalRead(PIN_TSW2);

  // ── Potentiometers ───────────────────────────────────────────
  txData.Pot1 = mapPot(analogRead(PIN_POT1));
  txData.Pot2 = mapPot(analogRead(PIN_POT2));

  // ── Battery % (computed by batteryManager) ───────────────────
  txData.BAT = gBatPct;

  // ── Fire ESP-NOW ─────────────────────────────────────────────
  esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));

  // ── Serial debug ─────────────────────────────────────────────
  Serial.printf("[TX→OUT] L(%+3d,%+3d) R(%+3d,%+3d) | LA:%d RA:%d LB:%d RB:%d | S1:%d S2:%d | P1:%3d P2:%3d | BAT:%3d%%\n",
    txData.Lx, txData.Ly, txData.Rx, txData.Ry,
    txData.LABt, txData.RABt, txData.LBt, txData.RBt,
    txData.TSW1, txData.TSW2,
    txData.Pot1, txData.Pot2,
    txData.BAT);
}

// ================================================================
//  BATTERY MANAGER
//  Reads voltage every 250 ms.
//  Returns true = low-bat safe mode active (caller skips normal loop)
// ================================================================
bool batteryManager()
{
  static unsigned long lastRead  = 0;
  static unsigned long lastBlink = 0;
  static bool  ledState = false;
  static bool  killSent = false;
  static float batV     = 8.4f;

  if (millis() - lastRead > 250) {
    lastRead = millis();
    gBatPct  = readBatPct(batV);
    Serial.printf("[TX→BAT] %.2fV  %d%%\n", batV, gBatPct);
  }

  // Update connection status (lost if no successful send for 1.5 s)
  gConnected = (millis() - lastSentOk < 1500);

  if (batV <= BAT_LOW_V) {
    if (!killSent) {
      memset(&txData, 0, sizeof(txData));
      esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));
      Serial.printf("[TX→BAT] !! SAFE MODE !! %.2fV  %d%%  — radio KILLED\n", batV, gBatPct);
      killSent = true;
    }
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState  = !ledState;
      digitalWrite(PIN_LED_R, ledState);   // fast blink red
    }
    digitalWrite(PIN_LED_G, LOW);
    return true;
  }

  killSent = false;
  digitalWrite(PIN_LED_R, LOW);
  return false;
}

// ================================================================
//  LCD DRAW HELPERS
// ================================================================

// Square joystick with dashed crosshair + moving 3×3 dot
// cx,cy = center   half = box half-size   lx,ly = -99..99
void drawJoystick(uint8_t cx, uint8_t cy, uint8_t half, int lx, int ly)
{
  uint8_t x0 = cx - half, y0 = cy - half, sz = half * 2 + 1;
  u8g2.drawFrame(x0, y0, sz, sz);                         // outer square
  for (uint8_t i = x0+2; i < x0+sz-2; i+=2) u8g2.drawPixel(i, cy);   // H dash
  for (uint8_t i = y0+2; i < y0+sz-2; i+=2) u8g2.drawPixel(cx, i);   // V dash
  u8g2.drawPixel(x0+1, y0+1); u8g2.drawPixel(x0+sz-2, y0+1);          // corner ticks
  u8g2.drawPixel(x0+1, y0+sz-2); u8g2.drawPixel(x0+sz-2, y0+sz-2);
  uint8_t inn = half - 3;
  int8_t  dx  = (int8_t)((lx * (int8_t)inn) / 99);
  int8_t  dy  = (int8_t)((ly * (int8_t)inn) / 99);
  u8g2.drawBox(cx + dx - 1, cy - dy - 1, 3, 3);           // 3×3 moving dot (+Y = up)
}

// Vertical fill bar — fill rises from bottom (100% = full)
void drawVSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct)
{
  u8g2.drawFrame(x, y, w, h);
  uint8_t fh = (uint16_t)pct * (h - 2) / 100;
  if (fh) u8g2.drawBox(x + 1, y + h - 1 - fh, w - 2, fh);
}

// Signal bars — 3 bars, bottom-aligned, top bar blinks when connected
void drawSignal(uint8_t cx, uint8_t bot, bool connected)
{
  const uint8_t BW = 4, GAP = 3, x0 = cx - 9;
  const uint8_t HTS[3] = {6, 11, 16};
  bool blink = (animTick / 5) % 2;
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t bx = x0 + i * (BW + GAP);
    uint8_t bh = HTS[i];
    uint8_t by = bot - bh;
    bool filled = connected && (i < 2 || blink);
    if (filled) u8g2.drawBox(bx, by, BW, bh);
    else        u8g2.drawFrame(bx, by, BW, bh);
  }
}

// Button box — inverted (filled+white text) when active/pressed
void drawBtn(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
             const char *label, bool active)
{
  if (active) {
    u8g2.drawBox(x, y, w, h);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x + 1, y + h - 1, label);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawFrame(x, y, w, h);
    u8g2.drawStr(x + 1, y + h - 1, label);
  }
}

// Vertical mobile-style battery — nub on top, fill from bottom
void drawVBattery(uint8_t bx, uint8_t by, uint8_t bw, uint8_t bh,
                  uint8_t pct, bool blink)
{
  uint8_t nw = bw / 2, nx = bx + (bw - nw) / 2;
  u8g2.drawBox(nx, by - 2, nw, 2);           // nub above body
  u8g2.drawFrame(bx, by, bw, bh);             // body outline
  if (!blink) {
    uint8_t fh = (uint16_t)pct * (bh - 2) / 100;
    if (fh) u8g2.drawBox(bx + 1, by + bh - 1 - fh, bw - 2, fh);
  }
}

// ================================================================
//  SCREEN: NORMAL OPERATION
//
//  ┌───────────────────────────────────────────────────────────────┐
//  │ [L stick] [P1▲] [▪▪▪ signal ▪▪▪] [P2▲] [R stick]          │ y 0-28
//  ├───────────────────────────────────────────────────────────────┤ y 29
//  │ [LA][LB][S1]              (gap)        [S2][RA][RB]          │ y 30-44
//  ├───────────────────────────────────────────────────────────────┤ y 45
//  │ ┌──────────────────────┐                   [▓bat▓]  82%      │ y 46-63
//  │ │ HASHTAG              │                                       │
//  │ │ URC v1.0             │                                       │
//  │ └──────────────────────┘                                       │
//  └───────────────────────────────────────────────────────────────┘
// ================================================================
void drawNormal()
{
  u8g2.setFont(u8g2_font_4x6_tf);

  // ── TOP: joysticks, vertical sliders, signal ─────────────────
  drawJoystick(13,  14, 12, sLx, sLy);   // L stick: cx=13 cy=14 half=12 → box x=1..25
  drawVSlider( 27,   2,  5, 25, (uint8_t)sP1);  // P1 vertical: x=27, right of L stick
  drawSignal(  64,  25, gConnected);              // signal bars: centered x=64
  drawVSlider( 96,   2,  5, 25, (uint8_t)sP2);  // P2 vertical: x=96, left of R stick
  drawJoystick(114, 14, 12, sRx, sRy);   // R stick: cx=114 cy=14 half=12 → box x=102..126

  u8g2.drawHLine(0, 29, 128);  // ── separator ──

  // ── MID: buttons only, symmetric around screen center ────────
  // Left group ends at x=36, right group starts at x=90, gap center = 63
  drawBtn( 2, 31, 10, 8, "LA", txData.LABt);
  drawBtn(14, 31, 10, 8, "LB", txData.LBt);
  drawBtn(26, 31, 10, 8, "S1", txData.TSW1);
  drawBtn(90, 31, 10, 8, "S2", txData.TSW2);
  drawBtn(102,31, 10, 8, "RA", txData.RABt);
  drawBtn(114,31, 10, 8, "RB", txData.RBt);

  u8g2.drawHLine(0, 45, 128);  // ── separator ──

  // ── BOTTOM: name box left, battery right ─────────────────────
  u8g2.drawFrame(0, 46, 58, 17);         // name outline box
  u8g2.drawStr(3, 54, "HASHTAG");
  u8g2.drawStr(3, 61, "URC v1.0");

  bool batLow = (sBat < 20) && batBlink;                      // blink if < 20%
  drawVBattery(96, 49, 8, 11, (uint8_t)sBat, batLow);        // bat body x=96 y=49
  char buf[5];
  sprintf(buf, sBat < 10 ? "  %d%%" : sBat < 100 ? " %d%%" : "%d%%", sBat);
  u8g2.drawStr(106, 61, buf);                                  // % to the right
}

// ================================================================
//  SCREEN: STARTUP CAUTION
// ================================================================
void drawCaution(bool sw1OK, bool sw2OK, bool p1OK, bool p2OK, bool batOK)
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(16, 10, "!! CAUTION !!");
  u8g2.drawHLine(0, 12, 128);
  u8g2.setFont(u8g2_font_4x6_tf);

  u8g2.drawStr( 2, 24, "SW1:"); u8g2.drawStr(22, 24, sw1OK ? " OK" : "OFF");
  u8g2.drawStr(66, 24, "SW2:"); u8g2.drawStr(86, 24, sw2OK ? " OK" : "OFF");
  u8g2.drawStr( 2, 34, "P1 :"); u8g2.drawStr(22, 34, p1OK  ? " OK" : "MIN");
  u8g2.drawStr(66, 34, "P2 :"); u8g2.drawStr(86, 34, p2OK  ? " OK" : "MIN");
  u8g2.drawStr( 2, 44, "BAT:");
  if (batOK) { char b[6]; sprintf(b, " %3d%%", gBatPct); u8g2.drawStr(22, 44, b); }
  else         u8g2.drawStr(22, 44, " LOW!");

  u8g2.drawHLine(0, 47, 128);

  // Animated waiting dots (0-3 dots cycling at ~2 Hz)
  char dots[14] = "Waiting ";
  uint8_t nd = (animTick / 8) % 4;
  for (uint8_t i = 0; i < nd; i++) dots[8 + i] = '.';
  dots[8 + nd] = '\0';
  u8g2.drawStr(10, 58, dots);
}

// ================================================================
//  SCREEN: LOW BATTERY WARNING
// ================================================================
void drawLowBat()
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 14, "LOW BATTERY!");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(24, 26, "Radio  OFF");
  u8g2.drawFrame(10, 32, 96, 20);   // large horizontal battery body
  u8g2.drawBox(106, 38,  5,  8);    // positive nub
  if (batBlink) {
    uint8_t fill = (uint16_t)gBatPct * 94 / 100;
    if (fill) u8g2.drawBox(11, 33, fill, 18);
  }
  char buf[5]; sprintf(buf, "%d%%", gBatPct);
  u8g2.drawStr(50, 45, buf);
}

// ================================================================
//  STARTUP SAFETY GATE
//  Blocks until: TSW1=off, TSW2=off, Pot1=min, Pot2=min, BAT≥10%
//  Renders CAUTION screen while waiting.
// ================================================================
void startup()
{
  Serial.println("[TX→BOOT] Startup safety check:");
  Serial.println("[TX→BOOT]   TSW1 and TSW2 must be OFF");
  Serial.println("[TX→BOOT]   Pot1 and Pot2 must be at minimum");
  Serial.println("[TX→BOOT]   Battery must be >= 10%");

  while (true)
  {
    float batV;
    gBatPct = readBatPct(batV);

    bool sw1OK = (bool)digitalRead(PIN_TSW1);                        // HIGH=OFF=safe
    bool sw2OK = (bool)digitalRead(PIN_TSW2);
    bool p1OK  = (analogRead(PIN_POT1) >= (4095 - POT_ZERO_THRESH)); // slider at min
    bool p2OK  = (analogRead(PIN_POT2) >= (4095 - POT_ZERO_THRESH));
    bool batOK = (gBatPct >= BAT_LOW_PCT);

    Serial.printf("[TX→CAUTION] SW1:%s SW2:%s P1:%s P2:%s BAT:%s(%d%%)\n",
      sw1OK?"OK":"WAIT", sw2OK?"OK":"WAIT",
      p1OK?"OK":"WAIT",  p2OK?"OK":"WAIT",
      batOK?"OK":"LOW",  gBatPct);

    if (sw1OK && sw2OK && p1OK && p2OK && batOK) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(4,  22, "** ALL CLEAR **");
      u8g2.setFont(u8g2_font_4x6_tf);
      u8g2.drawStr(22, 38, "System Ready");
      u8g2.drawStr(18, 50, "Starting up ...");
      u8g2.sendBuffer();
      Serial.println("[TX→BOOT] All clear — proceeding");
      delay(900);
      return;
    }

    animTick++;
    u8g2.clearBuffer();
    drawCaution(sw1OK, sw2OK, p1OK, p2OK, batOK);
    u8g2.sendBuffer();
    delay(150);
  }
}

// ================================================================
//  UPDATE DISPLAY  — called every loop tick
// ================================================================
void updateDisplay()
{
  // IIR smooth (joystick jitter prevention)
  sLx  = iirSmooth(sLx,  txData.Lx);
  sLy  = iirSmooth(sLy,  txData.Ly);
  sRx  = iirSmooth(sRx,  txData.Rx);
  sRy  = iirSmooth(sRy,  txData.Ry);
  sP1  = iirSmooth(sP1,  txData.Pot1);
  sP2  = iirSmooth(sP2,  txData.Pot2);
  sBat = iirSmooth(sBat, gBatPct);

  animTick++;
  if (animTick >= 200) animTick = 0;
  batBlink = ((animTick % 10) < 5);   // 2 Hz blink for low battery

  u8g2.clearBuffer();
  drawNormal();
  u8g2.sendBuffer();
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("\n================================================");
  Serial.println(" ESP32 Universal RC Controller — TX v1.0 boot");
  Serial.println("================================================");

  // ── Output pins ──────────────────────────────────────────────
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);   // red during boot
  digitalWrite(PIN_LED_G, LOW);

  // ── Input pins ───────────────────────────────────────────────
  pinMode(PIN_LABt, INPUT_PULLUP);
  pinMode(PIN_RABt, INPUT_PULLUP);
  pinMode(PIN_LBt,  INPUT_PULLUP);
  pinMode(PIN_RBt,  INPUT_PULLUP);
  pinMode(PIN_TSW1, INPUT_PULLUP);
  pinMode(PIN_TSW2, INPUT_PULLUP);

  // ADC 11dB attenuation → full 0-3.3V range on all ADC1 pins
  analogSetAttenuation(ADC_11db);
  Serial.println("[TX→BOOT] GPIO and ADC configured");

  // ── LCD init ─────────────────────────────────────────────────
  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  Serial.println("[TX→BOOT] LCD ST7920 init OK");

  // ── Safety gate (blocks here until controls are safe) ─────────
  //startup();

  // ── ESP-NOW ──────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  Serial.printf("[TX→BOOT] WiFi MAC (share with receivers): %s\n",
    WiFi.macAddress().c_str());
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[TX→ERROR] ESP-NOW init FAILED — halting");
    while (1) { digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R)); delay(200); }
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.printf("[TX→BOOT] ESP-NOW ready  mode:%s  peer:%02X:%02X:%02X:%02X:%02X:%02X\n",
    BROADCAST ? "BROADCAST" : "UNICAST",
    peerMAC[0],peerMAC[1],peerMAC[2],
    peerMAC[3],peerMAC[4],peerMAC[5]);

  digitalWrite(PIN_LED_R, LOW);
  Serial.println("[TX→BOOT] Boot complete — running at ~20 Hz\n");
}

// ================================================================
//  LOOP — ~20 Hz
// ================================================================
void loop()
{
  // Low battery: render warning screen, skip all normal operation
  //if (batteryManager()) {
    u8g2.clearBuffer();
    animTick++;
    if (animTick >= 200) animTick = 0;
    batBlink = ((animTick % 10) < 5);
    drawLowBat();
    u8g2.sendBuffer();
    delay(50);
    return;
  //}

  readAndSend();    // 1. read all inputs + fire ESP-NOW packet
  updateDisplay();  // 2. render LCD with smoothed values
  delay(50);        // 3. ~20 Hz loop rate
}
