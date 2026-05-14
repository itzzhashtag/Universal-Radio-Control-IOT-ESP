// ================================================================
//  Nano Display  — URC v1.2
//  Board   : Arduino Nano (ATmega328P, 16 MHz)
//  Display : ST7920 128×64 LCD  (SPI / serial mode)
//  Input   : NanoPacket via SoftwareSerial D2 ← ESP32 GPIO 22
//
//  Library : U8g2  (install via Library Manager → "U8g2 by olikraus")
//
//  ST7920 WIRING  — PSB pin on LCD MUST be tied to GND for SPI mode
//  ┌─────────────┬──────────┬────────────────────────────────────┐
//  │  Nano pin   │ LCD pin  │ Function                           │
//  ├─────────────┼──────────┼────────────────────────────────────┤
//  │  D10        │  RS/CS   │ Chip select                        │
//  │  D11        │  R/W/SID │ MOSI data                          │
//  │  D13        │  E/CLK   │ SPI clock                          │
//  │  D8         │  RST     │ Reset  (or tie to 5 V)             │
//  │  D2         │  —       │ SoftwareSerial RX ← ESP32 GPIO 22  │
//  │  5 V        │  VDD     │ Power                              │
//  │  5 V→10 Ω→  │  BLA     │ Backlight anode                    │
//  │  GND        │  VSS     │ Ground                             │
//  │  GND        │  PSB     │ ← MUST be GND to enable SPI        │
//  │  GND        │  BLK     │ Backlight cathode                  │
//  │  pot wiper  │  V0      │ Contrast (10 kΩ pot, ends→5V/GND)  │
//  └─────────────┴──────────┴────────────────────────────────────┘
//  Nano D3 is SoftwareSerial TX — wired to nothing, just declared.
//  Common GND between ESP32 and Nano is required.
// ================================================================

#include <U8g2lib.h>
#include <SoftwareSerial.h>

// ── HARDWARE ─────────────────────────────────────────────────────
// ST7920 hardware SPI: CS=D10, RESET=D8
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /*CS=*/10, /*reset=*/8);

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

// ── SMOOTHED VALUES — integer IIR, no floats ─────────────────────
int sLX=0, sLY=0, sRX=0, sRY=0;
int sP1=0, sP2=0, sBAT=0;

// ── ANIMATION COUNTERS ────────────────────────────────────────────
uint8_t  animTick  = 0;       // increments every frame (~20 Hz)
bool     batBlink  = false;   // toggled at 2 Hz for low-battery flash
uint32_t lastPktMs = 0;       // timestamp of last valid packet

// ── SERIAL STATE MACHINE ─────────────────────────────────────────
enum RxState { WAIT_HDR, READ_BODY };
RxState  rxState = WAIT_HDR;
uint8_t  rxBuf[PKT_SIZE];
uint8_t  rxIdx   = 0;

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
        if (b == PKT_HDR) { rxBuf[0] = b; rxIdx = 1; rxState = READ_BODY; }
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
          }
        }
        break;
    }
  }
}

// ================================================================
//  updateSmooth — integer IIR low-pass, converges in ~8 frames
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
//  DRAW HELPERS
// ================================================================

// Square joystick with dashed crosshair and moving 3×3 dot
// cx,cy = center   half = half-width of square   lx,ly = -99..99
void drawJoystick(uint8_t cx, uint8_t cy, uint8_t half, int lx, int ly)
{
  uint8_t x0 = cx - half;
  uint8_t y0 = cy - half;
  uint8_t sz = half * 2 + 1;

  // Outer square border
  u8g2.drawFrame(x0, y0, sz, sz);

  // Dashed crosshair — every other pixel
  for (uint8_t i = x0 + 2; i < x0 + sz - 2; i += 2) u8g2.drawPixel(i, cy);
  for (uint8_t i = y0 + 2; i < y0 + sz - 2; i += 2) u8g2.drawPixel(cx, i);

  // Corner accent pixels (Radiomaster-style tick marks)
  u8g2.drawPixel(x0 + 1, y0 + 1);
  u8g2.drawPixel(x0 + sz - 2, y0 + 1);
  u8g2.drawPixel(x0 + 1, y0 + sz - 2);
  u8g2.drawPixel(x0 + sz - 2, y0 + sz - 2);

  // 3×3 dot — mapped to inner area (3px margin from border)
  uint8_t inner = half - 3;
  int8_t  dx    = (int8_t)((lx * (int8_t)inner) / 99);
  int8_t  dy    = (int8_t)((ly * (int8_t)inner) / 99);
  u8g2.drawBox(cx + dx - 1, cy - dy - 1, 3, 3);  // Y flipped: +ly = up
}

// Vertical slider bar — fill rises from bottom (high value = more fill)
// x,y = top-left corner   w,h = dimensions   pct = 0..100
void drawVerticalSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct)
{
  u8g2.drawFrame(x, y, w, h);
  uint8_t fillH = (uint16_t)pct * (h - 2) / 100;
  if (fillH) u8g2.drawBox(x + 1, y + h - 1 - fillH, w - 2, fillH);
}

// Vertical mobile-style battery — nub on top, fill from bottom, blinks when low
// bx,by = top-left of body   bw,bh = body size   pct = 0..100   blink = blank fill
void drawVerticalBattery(uint8_t bx, uint8_t by,
                         uint8_t bw, uint8_t bh,
                         uint8_t pct, bool blink)
{
  uint8_t nw = bw / 2;
  uint8_t nx = bx + (bw - nw) / 2;
  u8g2.drawBox(nx, by - 2, nw, 2);           // nub (2px above body)
  u8g2.drawFrame(bx, by, bw, bh);             // body outline
  if (!blink)
  {
    uint8_t fillH = (uint16_t)pct * (bh - 2) / 100;
    if (fillH) u8g2.drawBox(bx + 1, by + bh - 1 - fillH, bw - 2, fillH);
  }
}

// Button box — inverted (filled) when active
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
//
//  ┌──────────────────────────────────────────────────────────────┐
//  │ [L stick 25×25] [P1 5×25] [sig bars cx=64] [P2 5×25] [R]  │ y 0..28
//  ├──────────────────────────────────────────────────────────────┤ y 29
//  │ [LB][LS][S1]                          [S2][RS][RB]          │ y 30..44
//  ├──────────────────────────────────────────────────────────────┤ y 45
//  │ ┌─────────────────────┐          [bat body] [82%]           │ y 46..63
//  │ │ HASHTAG             │                                      │
//  │ │ URC V1.2            │                                      │
//  │ └─────────────────────┘                                      │
//  └──────────────────────────────────────────────────────────────┘
// ================================================================
void drawNormalScreen()
{
  bool connLost = (millis() - lastPktMs > 600);
  bool LB   = live.flags & (1 << 0);
  bool RB   = live.flags & (1 << 1);
  bool LS   = live.flags & (1 << 2);
  bool RS   = live.flags & (1 << 3);
  bool SW1  = live.flags & (1 << 4);
  bool SW2  = live.flags & (1 << 5);
  bool CONN = !connLost && (live.flags & (1 << 6));

  u8g2.setFont(u8g2_font_4x6_tf);

  // ── TOP ZONE  y=0..28 ─────────────────────────────────────────

  // Left joystick: square cx=13 cy=14 half=12  → box x=1..25
  drawJoystick(13, 14, 12, sLX, sLY);

  // P1 vertical slider: x=27 y=2 w=5 h=25  (right of L stick)
  drawVerticalSlider(27, 2, 5, 25, sP1);

  // Signal bars: 3 bars, BW=4 GAP=3, heights [6,11,16]
  // centered at x=64, bottom-aligned at y=25
  // top bar (tallest) blinks when connected
  {
    const uint8_t BW = 4, GAP = 3, BOT = 25;
    const uint8_t HTS[3] = {6, 11, 16};
    const uint8_t x0 = 64 - 9;   // 3 bars total width = 3*4+2*3=18, half=9
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

  // P2 vertical slider: x=96 y=2 w=5 h=25  (left of R stick)
  drawVerticalSlider(96, 2, 5, 25, sP2);

  // Right joystick: square cx=114 cy=14 half=12  → box x=102..126
  drawJoystick(114, 14, 12, sRX, sRY);

  // ── SEPARATOR  y=29 ───────────────────────────────────────────
  u8g2.drawHLine(0, 29, 128);

  // ── MID ZONE  y=30..44  — buttons only, symmetric ─────────────
  // Left group:  LB x=2  LS x=14  S1 x=26  → ends x=36
  // Right group: S2 x=90 RS x=102 RB x=114  → ends x=124
  // Center of gap = (36+90)/2 = 63 ≈ screen center
  drawBtn( 2,  31, 10, 8, "LB", LB);
  drawBtn(14,  31, 10, 8, "LS", LS);
  drawBtn(26,  31, 10, 8, "S1", SW1);
  drawBtn(90,  31, 10, 8, "S2", SW2);
  drawBtn(102, 31, 10, 8, "RS", RS);
  drawBtn(114, 31, 10, 8, "RB", RB);

  // ── SEPARATOR  y=45 ───────────────────────────────────────────
  u8g2.drawHLine(0, 45, 128);

  // ── BOTTOM ZONE  y=46..63 ─────────────────────────────────────

  // Left: name box  frame(0, 46, 58, 17)
  u8g2.drawFrame(0, 46, 58, 17);
  u8g2.drawStr(3, 54, "HASHTAG");
  u8g2.drawStr(3, 61, "URC V1.2");

  // Right: vertical battery body x=96 y=49 bw=8 bh=11
  //        nub drawn 2px above body = y=47
  //        % text to the right of battery at x=106
  bool batLow = (sBAT < 20) && batBlink;
  drawVerticalBattery(96, 49, 8, 11, sBAT, batLow);

  char batbuf[5];
  sprintf(batbuf, sBAT < 10 ? "  %d%%" : sBAT < 100 ? " %d%%" : "%d%%", sBAT);
  u8g2.drawStr(106, 61, batbuf);
}

// ================================================================
//  SCREEN: CAUTION
//  Shown during startup safety check — streams from ESP32.
//
//  ┌────────────────────────────────┐
//  │      !! CAUTION !!             │
//  │────────────────────────────────│
//  │  SW1: OK    SW2: OFF           │
//  │  P1 : OK    P2 : MIN           │
//  │  BAT: 82%                      │
//  │────────────────────────────────│
//  │  Waiting ...                   │
//  └────────────────────────────────┘
// ================================================================
void drawCautionScreen()
{
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

  // Animated dots: 0-3 dots cycling at ~2 Hz
  char dots[14] = "Waiting ";
  uint8_t nd = (animTick / 8) % 4;
  for (uint8_t i = 0; i < nd; i++) dots[8 + i] = '.';
  dots[8 + nd] = '\0';
  u8g2.drawStr(10, 58, dots);
}

// ================================================================
//  SCREEN: ALL CLEAR — shown briefly when startup check passes
// ================================================================
void drawAllClearScreen()
{
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
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 14, "LOW BATTERY!");

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(24, 26, "Radio  OFF");

  // Large horizontal battery icon for emphasis
  u8g2.drawFrame(10, 32, 96, 20);    // body
  u8g2.drawBox(106, 38, 5, 8);       // positive nub

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
  espSerial.begin(19200);

  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);

  // Safe defaults while waiting for first packet
  memset(&live, 0, sizeof(live));
  live.BAT   = 100;
  live.state = STATE_CAUTION;
  lastPktMs  = millis();
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

  // Animation counters
  animTick++;
  if (animTick >= 200) animTick = 0;
  batBlink = ((animTick % 10) < 5);   // 2 Hz blink

  updateSmooth();    // IIR smooth joystick + slider values

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
