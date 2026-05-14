// ================================================================
//  Arduino Nano — KS0108 Display Receiver  v1.0
//
//  Receives 16-byte binary packets from ESP32 over UART,
//  renders controller state on JHD12864E (KS0108 128×64).
//
//  ── Memory budget (Nano has 2KB SRAM) ───────────────────────────
//  U8g2 full-buffer: 128×64/8 = 1024 bytes
//  Stack + locals  : ~300 bytes
//  Packet buffer   : 16 bytes
//  Total           : ~1340 bytes  — safe margin ~660 bytes
//
//  ── Wiring (matches your working KS0108 base code) ──────────────
//  LCD D0-D7  → Nano pins 2-9
//  LCD EN     → Nano A4 (pin 18)
//  LCD RS/DC  → Nano A5 (pin 19)
//  LCD CS1    → Nano pin 10
//  LCD CS2    → Nano pin 11
//  LCD R/W    → GND   (write-only)
//  LCD RST    → not connected
//  LCD VCC    → 5V,  GND → GND
//
//  ESP32 → Nano UART:
//    ESP32 TX (GPIO1)  → Nano RX (pin 0)
//    GND               → GND
//
//  !! Disconnect Nano RX pin 0 when uploading sketches !!
//  !! (UART conflicts with USB upload)                  !!
//
//  ── Required Library ────────────────────────────────────────────
//  U8g2 by olikraus  (Library Manager → search "U8g2")
// ================================================================

#include <Arduino.h>
#include <U8g2lib.h>

// ── U8g2 constructor — matches your working base code ────────────
U8G2_KS0108_128X64_F u8g2(U8G2_R0,
  2, 3, 4, 5, 6, 7, 8, 9,   // D0–D7
  18,                         // EN  (A4)
  19,                         // DC  (A5)
  10,                         // CS1
  11,                         // CS2
  U8X8_PIN_NONE               // RST
);

// ── Packet ───────────────────────────────────────────────────────
#define PKT_LEN   16
#define PKT_START 0xAA

static uint8_t  rxBuf[PKT_LEN];
static uint8_t  rxPos   = 0;
static bool     synced  = false;

// ── Decoded display state ─────────────────────────────────────────
static int8_t  dLx=0, dLy=0, dRx=0, dRy=0;
static uint8_t dP1=50, dP2=50, dBat=100;
static uint8_t dBtns = 0;   // bitmask, same layout as pkt[8]
static bool    dConn = false;

// ── Animation tick (wraps 0-199) ─────────────────────────────────
static uint8_t animTick = 0;

// ================================================================
//  PACKET PARSER
//  Feed bytes one at a time. Returns true when a valid frame lands.
// ================================================================
bool parseByte(uint8_t b)
{
  // Hunt for start byte
  if (!synced) {
    if (b == PKT_START) { synced = true; rxPos = 0; rxBuf[rxPos++] = b; }
    return false;
  }

  rxBuf[rxPos++] = b;

  if (rxPos < PKT_LEN) return false;   // still collecting

  // Full packet — verify checksum
  uint8_t cs = 0;
  for (uint8_t i = 1; i < 15; i++) cs ^= rxBuf[i];
  synced = false;
  rxPos  = 0;

  if (cs != rxBuf[15]) return false;   // bad checksum, discard

  // Unpack
  dLx  = (int8_t)(rxBuf[1] - 99);
  dLy  = (int8_t)(rxBuf[2] - 99);
  dRx  = (int8_t)(rxBuf[3] - 99);
  dRy  = (int8_t)(rxBuf[4] - 99);
  dP1  = rxBuf[5];
  dP2  = rxBuf[6];
  dBat = rxBuf[7];
  dBtns= rxBuf[8];
  dConn= (rxBuf[8] & 0x40) != 0;

  return true;
}

// ================================================================
//  DRAW HELPERS  — mirror of the HTML visualiser logic
// ================================================================

// Box outline
static void fr(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  u8g2.drawFrame(x, y, w, h);
}
// Filled box
static void fb(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  u8g2.drawBox(x, y, w, h);
}

// Joystick widget
void drawJoy(uint8_t cx, uint8_t cy, uint8_t half, int8_t lx, int8_t ly)
{
  uint8_t x0 = cx - half, y0 = cy - half, sz = half * 2 + 1;
  fr(x0, y0, sz, sz);
  // Dashed crosshairs
  for (uint8_t i = x0+2; i < x0+sz-2; i+=2) u8g2.drawPixel(i, cy);
  for (uint8_t i = y0+2; i < y0+sz-2; i+=2) u8g2.drawPixel(cx, i);
  // Corner ticks
  u8g2.drawPixel(x0+1, y0+1);     u8g2.drawPixel(x0+sz-2, y0+1);
  u8g2.drawPixel(x0+1, y0+sz-2); u8g2.drawPixel(x0+sz-2, y0+sz-2);
  // Moving dot
  uint8_t inn = half - 3;
  int8_t  dx  = (int8_t)((lx * (int8_t)inn) / 99);
  int8_t  dy  = (int8_t)((ly * (int8_t)inn) / 99);
  fb(cx + dx - 1, cy - dy - 1, 3, 3);
}

// Vertical fill bar (pot / slider)
void drawVSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct)
{
  fr(x, y, w, h);
  uint8_t fh = (uint16_t)pct * (h - 2) / 100;
  if (fh) fb(x+1, y+h-1-fh, w-2, fh);
}

// Signal bars (3 bars, top bar blinks when connected)
void drawSignal(uint8_t cx, uint8_t bot)
{
  const uint8_t BW=4, GAP=3;
  const uint8_t x0 = cx - 9;
  const uint8_t HTS[3] = {6, 11, 16};
  bool bl = (animTick % 10) < 5;

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t bx = x0 + i * (BW + GAP);
    uint8_t bh = HTS[i];
    uint8_t by = bot - bh;
    bool filled = dConn && (i < 2 || bl);
    if (filled) fb(bx, by, BW, bh);
    else        fr(bx, by, BW, bh);
  }
}

// Button: inverted when active
void drawBtn(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
             const char *lbl, bool act)
{
  if (act) {
    fb(x, y, w, h);
    u8g2.setDrawColor(0);
    u8g2.drawStr(x+1, y+h-1, lbl);
    u8g2.setDrawColor(1);
  } else {
    fr(x, y, w, h);
    u8g2.drawStr(x+1, y+h-1, lbl);
  }
}

// Vertical battery icon (nub on top, fill from bottom)
void drawVBattery(uint8_t bx, uint8_t by, uint8_t bw, uint8_t bh, uint8_t pct)
{
  uint8_t nw = bw / 2, nx = bx + (bw - nw) / 2;
  fb(nx, by-2, nw, 2);          // nub
  fr(bx, by, bw, bh);           // body
  bool blink = (pct < 20) && ((animTick % 10) >= 5);
  if (!blink) {
    uint8_t fh = (uint16_t)pct * (bh - 2) / 100;
    if (fh) fb(bx+1, by+bh-1-fh, bw-2, fh);
  }
}

// ================================================================
//  MAIN DRAW — mirrors HTML visualiser layout exactly
//
//  TOP   y=0..28  : joy-L  P1  signal  P2  joy-R
//  SEP   y=29
//  MID   y=30..44 : LB LS S1  (gap)  S2 RS RB
//  SEP   y=45
//  BOT   y=46..63 : [HASHTAG / URC v1.0]   [bat] [%]
// ================================================================
void drawNormal()
{
  u8g2.setFont(u8g2_font_4x6_tf);

  // ── TOP ──────────────────────────────────────────────────────
  drawJoy(13, 14, 12, dLx, dLy);
  drawVSlider(27, 2, 5, 25, dP1);
  drawSignal(64, 25);
  drawVSlider(96, 2, 5, 25, dP2);
  drawJoy(114, 14, 12, dRx, dRy);

  u8g2.drawHLine(0, 29, 128);

  // ── MID buttons ──────────────────────────────────────────────
  drawBtn(  2, 31, 10, 8, "LB", dBtns & 0x04);   // LBt
  drawBtn( 14, 31, 10, 8, "LS", dBtns & 0x01);   // LABt
  drawBtn( 26, 31, 10, 8, "S1", dBtns & 0x10);   // TSW1
  drawBtn( 90, 31, 10, 8, "S2", dBtns & 0x20);   // TSW2
  drawBtn(102, 31, 10, 8, "RS", dBtns & 0x02);   // RABt
  drawBtn(114, 31, 10, 8, "RB", dBtns & 0x08);   // RBt

  u8g2.drawHLine(0, 45, 128);

  // ── BOTTOM ───────────────────────────────────────────────────
  fr(0, 46, 58, 17);
  u8g2.drawStr(3, 54, "HASHTAG");
  u8g2.drawStr(3, 61, "URC v1.0");

  drawVBattery(96, 49, 8, 11, dBat);

  char buf[5];
  if      (dBat < 10)  sprintf_P(buf, PSTR("  %d%%"), dBat);
  else if (dBat < 100) sprintf_P(buf, PSTR(" %d%%"),  dBat);
  else                 sprintf_P(buf, PSTR("%d%%"),   dBat);
  u8g2.drawStr(106, 61, buf);
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);   // must match ESP32 baud
  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);

  // Splash
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(4, 22, "HASHTAG URC");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(22, 38, "Display v1.0");
  u8g2.drawStr(14, 50, "Waiting for ESP...");
  u8g2.sendBuffer();
  delay(1500);
}

// ================================================================
//  LOOP
// ================================================================
void loop()
{
  // Drain all available bytes into the parser
  while (Serial.available()) {
    parseByte((uint8_t)Serial.read());
  }

  // Advance animation tick (drives signal bar blink + bat blink)
  animTick++;
  if (animTick >= 200) animTick = 0;

  // Redraw at ~20 Hz regardless of new packet (smooth animation)
  u8g2.clearBuffer();
  drawNormal();
  u8g2.sendBuffer();

  delay(50);
}
