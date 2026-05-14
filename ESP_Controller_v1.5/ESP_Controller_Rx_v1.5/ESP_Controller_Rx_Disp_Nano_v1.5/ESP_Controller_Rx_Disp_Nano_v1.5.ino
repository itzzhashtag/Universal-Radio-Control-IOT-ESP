// ================================================================
//  Nano Display  v1.2  — DEBUG EDITION
//  Board   : Arduino Nano (ATmega328P, 16 MHz)
//  Display : ST7920 128×64 LCD  (SPI / serial mode)
//  Input   : NanoPacket via SoftwareSerial D2 ← ESP32 GPIO 22
//
//  ── DEBUG FEATURES ADDED ────────────────────────────────────────
//  #define DEBUG        → enables all Serial.print debug output
//                         (uses the USB/UART0 Serial at 115200 baud
//                          — same connector as programming cable)
//
//  Serial output format (DEBUG):
//    [PKT]  every valid packet decoded: all fields, human readable
//    [HEX]  raw bytes of the received packet (hex dump)
//    [ERR]  bad footer — packet dropped
//    [LOST] connection-lost event (first frame where millis gap > 600ms)
//    [BACK] connection-restored event
//    [SCREEN] which screen is being rendered each frame
//    [SMOOTH] smoothed axis values every 10 frames
//
//  NOTE: SoftwareSerial (D2) RX conflicts are possible if the USB
//  serial and SoftwareSerial are both maxed out simultaneously.
//  At 19200 baud from ESP32 and 115200 baud USB monitor this is
//  not normally a problem at 20 Hz update rate.
//
//  ST7920 WIRING — PSB pin MUST be tied to GND for SPI mode
//  ┌─────────────┬──────────┬──────────────────────────────────┐
//  │  Nano pin   │ LCD pin  │ Function                         │
//  ├─────────────┼──────────┼──────────────────────────────────┤
//  │  D10        │  RS/CS   │ Chip select                      │
//  │  D11        │  R/W/SID │ MOSI data                        │
//  │  D13        │  E/CLK   │ SPI clock                        │
//  │  D8         │  RST     │ Reset  (or tie to 5 V)           │
//  │  D2         │  —       │ SoftwareSerial RX ← ESP32 GPIO22 │
//  │  5 V        │  VDD     │ Power                            │
//  │  GND        │  VSS     │ Ground                           │
//  │  GND        │  PSB     │ ← MUST be GND to enable SPI      │
//  └─────────────┴──────────┴──────────────────────────────────┘
// ================================================================

#include <U8g2lib.h>
#include <SoftwareSerial.h>

// ── DEBUG SWITCH ─────────────────────────────────────────────────
// Comment out to disable all debug serial prints
#define DEBUG

// ── Conditional print macros ─────────────────────────────────────
#ifdef DEBUG
  #define DBG(...)   Serial.print(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)

  // printf replacement for Arduino Nano
  #define DBGF(...) do { \
    char _buf[128]; \
    snprintf(_buf, sizeof(_buf), __VA_ARGS__); \
    Serial.print(_buf); \
  } while (0)

#else
  #define DBG(...)
  #define DBGLN(...)
  #define DBGF(...)
#endif

// ── HARDWARE ─────────────────────────────────────────────────────
U8G2_ST7920_128X64_1_HW_SPI u8g2(U8G2_R0, /*CS=*/10, /*reset=*/8);

// SoftwareSerial: RX=D2 (from ESP32 GPIO22), TX=D3 (unused)
SoftwareSerial espSerial(2, 3);

// ── PACKET DEFINITION ─────────────────────────────────────────────
#define PKT_SIZE   12
#define PKT_HDR    0xAA
#define PKT_FTR    0x55

#define STATE_NORMAL   0
#define STATE_CAUTION  1
#define STATE_LOWBAT   2
#define STATE_ALLCLEAR 3

typedef struct __attribute__((packed)) {
  uint8_t hdr;
  uint8_t state;
  int8_t  LX, LY, RX, RY;
  uint8_t P1, P2, BAT;
  uint8_t flags;   // b0=LB b1=RB b2=LS b3=RS b4=SW1 b5=SW2 b6=CONN
  uint8_t safety;  // b0=sw1OK b1=sw2OK b2=p1OK b3=p2OK b4=batOK
  uint8_t ftr;
} NanoPacket;

NanoPacket live;

// ── SMOOTHED VALUES ──────────────────────────────────────────────
int sLX=0, sLY=0, sRX=0, sRY=0;
int sP1=0, sP2=0, sBAT=0;

// ── ANIMATION / STATE ────────────────────────────────────────────
uint8_t  animTick   = 0;
bool     batBlink   = false;
uint32_t lastPktMs  = 0;

// Tracks previous connection state for [LOST]/[BACK] log events
bool prevConnLost = false;

// ── SERIAL STATE MACHINE ─────────────────────────────────────────
enum RxState { WAIT_HDR, READ_BODY };
RxState  rxState = WAIT_HDR;
uint8_t  rxBuf[PKT_SIZE];
uint8_t  rxIdx   = 0;

// ── PACKET COUNTER (debug stats) ─────────────────────────────────
#ifdef DEBUG
  uint32_t pktReceived = 0;   // valid packets accepted
  uint32_t pktDropped  = 0;   // bad footer, dropped
#endif

// ================================================================
//  DEBUG HELPER — hex dump
// ================================================================
#ifdef DEBUG
void debugHexDump(const char *label, const uint8_t *buf, uint8_t len)
{
  DBG(label);
  DBG(" [ ");
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) DBG("0");
    DBG(buf[i], HEX);
    DBG(" ");
  }
  DBGLN("]");
}
#endif

// ================================================================
//  DEBUG PACKET PRINT — human readable decode of a NanoPacket
// ================================================================
#ifdef DEBUG
void debugPrintPacket(const NanoPacket &p)
{
  // State name lookup
  const char *stateStr;
  switch (p.state) {
    case STATE_NORMAL:   stateStr = "NORMAL";   break;
    case STATE_CAUTION:  stateStr = "CAUTION";  break;
    case STATE_LOWBAT:   stateStr = "LOWBAT";   break;
    case STATE_ALLCLEAR: stateStr = "ALLCLEAR"; break;
    default:             stateStr = "???";       break;
  }

  bool LB   = p.flags & (1 << 0);
  bool RB   = p.flags & (1 << 1);
  bool LS   = p.flags & (1 << 2);
  bool RS   = p.flags & (1 << 3);
  bool SW1  = p.flags & (1 << 4);
  bool SW2  = p.flags & (1 << 5);
  bool CONN = p.flags & (1 << 6);

  bool sw1OK = p.safety & (1 << 0);
  bool sw2OK = p.safety & (1 << 1);
  bool p1OK  = p.safety & (1 << 2);
  bool p2OK  = p.safety & (1 << 3);
  bool batOK = p.safety & (1 << 4);

  DBGF("[PKT#%lu] STATE=%s | L(%d,%d) R(%d,%d) | P1=%d P2=%d BAT=%d%%\n",
       pktReceived, stateStr,
       p.LX, p.LY, p.RX, p.RY,
       p.P1, p.P2, p.BAT);
  DBGF("         flags: LB=%d RB=%d LS=%d RS=%d SW1=%d SW2=%d CONN=%d\n",
       LB, RB, LS, RS, SW1, SW2, CONN);
  DBGF("         safety: sw1OK=%d sw2OK=%d p1OK=%d p2OK=%d batOK=%d\n",
       sw1OK, sw2OK, p1OK, p2OK, batOK);
}
#endif

// ================================================================
//  receivePacket — drain serial buffer, parse on full valid frame
// ================================================================
void receivePacket()
{
  while (espSerial.available())
  {
    uint8_t b = espSerial.read();
    switch (rxState)
    {
      case WAIT_HDR:
        if (b == PKT_HDR) {
          rxBuf[0] = b;
          rxIdx    = 1;
          rxState  = READ_BODY;
        }
        break;

      case READ_BODY:
        rxBuf[rxIdx++] = b;
        if (rxIdx == PKT_SIZE)
        {
          rxState = WAIT_HDR;
          rxIdx   = 0;

          if (rxBuf[PKT_SIZE - 1] == PKT_FTR)
          {
            memcpy(&live, rxBuf, PKT_SIZE);
            lastPktMs = millis();

            #ifdef DEBUG
              pktReceived++;
              // Print hex dump of raw bytes
              debugHexDump("[HEX]", rxBuf, PKT_SIZE);
              // Print decoded packet every 10th packet to avoid serial flood
              if (pktReceived % 10 == 0) debugPrintPacket(live);
            #endif
          }
          else
          {
            #ifdef DEBUG
              pktDropped++;
              DBGF("[ERR] Bad footer: expected 0x55 got 0x%02X  (total dropped: %lu)\n",
                   rxBuf[PKT_SIZE - 1], pktDropped);
              debugHexDump("[ERR HEX]", rxBuf, PKT_SIZE);
            #endif
          }
        }
        break;
    }
  }
}

// ================================================================
//  updateSmooth — integer IIR low-pass
// ================================================================
int smoothStep(int cur, int tgt)
{
  int d = tgt - cur;
  if (d == 0 || abs(d) <= 1) return tgt;
  return cur + d / 4;
}

void updateSmooth()
{
  sLX  = smoothStep(sLX,  (int)live.LX);
  sLY  = smoothStep(sLY,  (int)live.LY);
  sRX  = smoothStep(sRX,  (int)live.RX);
  sRY  = smoothStep(sRY,  (int)live.RY);
  sP1  = smoothStep(sP1,  (int)live.P1);
  sP2  = smoothStep(sP2,  (int)live.P2);
  sBAT = smoothStep(sBAT, (int)live.BAT);
}

// ================================================================
//  CONNECTION LOST / BACK events — logged once each transition
// ================================================================
void checkConnEvents()
{
  #ifdef DEBUG
  bool connLost = (millis() - lastPktMs > 600);
  if (connLost && !prevConnLost) {
    DBGF("[LOST] No packet for >600ms — last seen %lums ago\n",
         millis() - lastPktMs);
  }
  if (!connLost && prevConnLost) {
    DBGLN("[BACK] Connection restored");
  }
  prevConnLost = connLost;
  #endif
}

// ================================================================
//  DRAW HELPERS (unchanged from original)
// ================================================================
void drawJoystick(uint8_t cx, uint8_t cy, uint8_t half, int lx, int ly)
{
  uint8_t x0 = cx - half;
  uint8_t y0 = cy - half;
  uint8_t sz = half * 2 + 1;
  u8g2.drawFrame(x0, y0, sz, sz);
  for (uint8_t i = x0 + 2; i < x0 + sz - 2; i += 2) u8g2.drawPixel(i, cy);
  for (uint8_t i = y0 + 2; i < y0 + sz - 2; i += 2) u8g2.drawPixel(cx, i);
  u8g2.drawPixel(x0 + 1, y0 + 1);
  u8g2.drawPixel(x0 + sz - 2, y0 + 1);
  u8g2.drawPixel(x0 + 1, y0 + sz - 2);
  u8g2.drawPixel(x0 + sz - 2, y0 + sz - 2);
  uint8_t inner = half - 3;
  int8_t  dx    = (int8_t)((lx * (int8_t)inner) / 99);
  int8_t  dy    = (int8_t)((ly * (int8_t)inner) / 99);
  u8g2.drawBox(cx + dx - 1, cy - dy - 1, 3, 3);
}

void drawVerticalSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct)
{
  u8g2.drawFrame(x, y, w, h);
  uint8_t fillH = (uint16_t)pct * (h - 2) / 100;
  if (fillH) u8g2.drawBox(x + 1, y + h - 1 - fillH, w - 2, fillH);
}

void drawVerticalBattery(uint8_t bx, uint8_t by,
                         uint8_t bw, uint8_t bh,
                         uint8_t pct, bool blink)
{
  uint8_t nw = bw / 2;
  uint8_t nx = bx + (bw - nw) / 2;
  u8g2.drawBox(nx, by - 2, nw, 2);
  u8g2.drawFrame(bx, by, bw, bh);
  if (!blink)
  {
    uint8_t fillH = (uint16_t)pct * (bh - 2) / 100;
    if (fillH) u8g2.drawBox(bx + 1, by + bh - 1 - fillH, bw - 2, fillH);
  }
}

void drawBtn(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
             const char *label, bool active)
{
  if (active)
  {
    u8g2.drawBox(x, y, w, h);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x + 1, y + h - 1, label);
    u8g2.setDrawColor(1);
  }
  else
  {
    u8g2.drawFrame(x, y, w, h);
    u8g2.drawStr(x + 1, y + h - 1, label);
  }
}

// ================================================================
//  SCREEN: NORMAL
// ================================================================
void drawNormalScreen()
{
  DBGLN("[SCREEN] NORMAL");

  bool connLost = (millis() - lastPktMs > 600);
  bool LB   = live.flags & (1 << 0);
  bool RB   = live.flags & (1 << 1);
  bool LS   = live.flags & (1 << 2);
  bool RS   = live.flags & (1 << 3);
  bool SW1  = live.flags & (1 << 4);
  bool SW2  = live.flags & (1 << 5);
  bool CONN = !connLost && (live.flags & (1 << 6));

  u8g2.setFont(u8g2_font_4x6_tf);

  drawJoystick(13, 14, 12, sLX, sLY);
  drawVerticalSlider(27, 2, 5, 25, sP1);

  {
    const uint8_t BW = 4, GAP = 3, BOT = 25;
    const uint8_t HTS[3] = {6, 11, 16};
    const uint8_t x0 = 64 - 9;
    bool blink = (animTick / 5) % 2;
    for (uint8_t i = 0; i < 3; i++)
    {
      uint8_t bx = x0 + i * (BW + GAP);
      uint8_t bh = HTS[i];
      uint8_t by = BOT - bh;
      bool filled = CONN && (i < 2 || blink);
      if (filled) u8g2.drawBox(bx, by, BW, bh);
      else        u8g2.drawFrame(bx, by, BW, bh);
    }
  }

  drawVerticalSlider(96, 2, 5, 25, sP2);
  drawJoystick(114, 14, 12, sRX, sRY);

  u8g2.drawHLine(0, 29, 128);

  drawBtn( 2,  31, 10, 8, "LB", LB);
  drawBtn(14,  31, 10, 8, "LS", LS);
  drawBtn(26,  31, 10, 8, "S1", SW1);
  drawBtn(90,  31, 10, 8, "S2", SW2);
  drawBtn(102, 31, 10, 8, "RS", RS);
  drawBtn(114, 31, 10, 8, "RB", RB);

  u8g2.drawHLine(0, 45, 128);

  u8g2.drawFrame(0, 46, 58, 17);
  u8g2.drawStr(3, 54, "HASHTAG");
  u8g2.drawStr(3, 61, "URC V1.2");

  bool batLow = (sBAT < 20) && batBlink;
  drawVerticalBattery(96, 49, 8, 11, sBAT, batLow);

  char batbuf[5];
  sprintf(batbuf, sBAT < 10 ? "  %d%%" : sBAT < 100 ? " %d%%" : "%d%%", sBAT);
  u8g2.drawStr(106, 61, batbuf);
}

// ================================================================
//  SCREEN: CAUTION
// ================================================================
void drawCautionScreen()
{
  //DBGLN("[SCREEN] CAUTION");

  bool sw1OK = live.safety & (1 << 0);
  bool sw2OK = live.safety & (1 << 1);
  bool p1OK  = live.safety & (1 << 2);
  bool p2OK  = live.safety & (1 << 3);
  bool batOK = live.safety & (1 << 4);

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(16, 10, "!! CAUTION !!");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr( 2, 24, "SW1:"); u8g2.drawStr(22, 24, sw1OK ? " OK" : "OFF");
  u8g2.drawStr(66, 24, "SW2:"); u8g2.drawStr(86, 24, sw2OK ? " OK" : "OFF");
  u8g2.drawStr( 2, 34, "P1 :"); u8g2.drawStr(22, 34, p1OK  ? " OK" : "MIN");
  u8g2.drawStr(66, 34, "P2 :"); u8g2.drawStr(86, 34, p2OK  ? " OK" : "MIN");

  u8g2.drawStr(2, 44, "BAT:");
  if (batOK)
  {
    char buf[6]; sprintf(buf, " %3d%%", live.BAT);
    u8g2.drawStr(22, 44, buf);
  }
  else
  {
    u8g2.drawStr(22, 44, " LOW!");
  }

  u8g2.drawHLine(0, 47, 128);

  char dots[14] = "Waiting ";
  uint8_t nd = (animTick / 8) % 4;
  for (uint8_t i = 0; i < nd; i++) dots[8 + i] = '.';
  dots[8 + nd] = '\0';
  u8g2.drawStr(10, 58, dots);
}

// ================================================================
//  SCREEN: ALL CLEAR
// ================================================================
void drawAllClearScreen()
{
  DBGLN("[SCREEN] ALL CLEAR");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(4,  22, "** ALL CLEAR **");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(22, 38, "System Ready");
  u8g2.drawStr(18, 50, "Starting up ...");
}

// ================================================================
//  SCREEN: LOW BATTERY
// ================================================================
void drawLowBatScreen()
{
  DBGLN("[SCREEN] LOW BATTERY");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 14, "LOW BATTERY!");

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(24, 26, "Radio  OFF");

  u8g2.drawFrame(10, 32, 96, 20);
  u8g2.drawBox(106, 38, 5, 8);

  if (batBlink)
  {
    uint8_t fill = (uint16_t)live.BAT * 94 / 100;
    if (fill) u8g2.drawBox(11, 33, fill, 18);
  }

  char buf[5]; sprintf(buf, "%d%%", live.BAT);
  u8g2.drawStr(52, 45, buf);
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  // USB serial — debug monitor
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) { /* wait for USB */ }
    DBGLN("=== Nano Display v1.2 DEBUG EDITION ===");
    DBGLN("[SETUP] Serial ready at 115200");
  #endif

  espSerial.begin(19200);
  DBGLN("[SETUP] SoftwareSerial RX on D2 at 19200 baud");

  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);
  DBGLN("[SETUP] U8g2 display initialised");

  memset(&live, 0, sizeof(live));
  live.BAT   = 100;
  live.state = STATE_CAUTION;
  lastPktMs  = millis();

  DBGLN("[SETUP] Defaults set. Waiting for ESP32 packets...");
}

// ================================================================
//  LOOP — capped at 20 FPS
// ================================================================
void loop()
{
  static uint32_t lastFrame = 0;

  receivePacket();   // always drain serial buffer first

  uint32_t now = millis();
  if (now - lastFrame < 50) return;
  lastFrame = now;

  animTick++;
  if (animTick >= 200) animTick = 0;
  batBlink = ((animTick % 10) < 5);

  checkConnEvents();  // log lost/back transitions
  updateSmooth();

  // Print smoothed values every 50 frames (~2.5 s) to monitor IIR
  #ifdef DEBUG
  if (animTick % 50 == 0) {
    DBGF("[SMOOTH] sLX=%d sLY=%d sRX=%d sRY=%d sP1=%d sP2=%d sBAT=%d\n",
         sLX, sLY, sRX, sRY, sP1, sP2, sBAT);
    DBGF("[STATS] pkts OK=%lu  dropped=%lu\n", pktReceived, pktDropped);
  }
  #endif

  u8g2.clearBuffer();

  switch (live.state)
  {
    case STATE_CAUTION:  drawCautionScreen();   break;
    case STATE_ALLCLEAR: drawAllClearScreen();  break;
    case STATE_LOWBAT:   drawLowBatScreen();    break;
    default:             drawNormalScreen();    break;
  }

  u8g2.sendBuffer();
}
