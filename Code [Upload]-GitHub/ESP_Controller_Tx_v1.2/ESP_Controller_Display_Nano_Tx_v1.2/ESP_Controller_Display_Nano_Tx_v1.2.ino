// ================================================================
// Nano Display  — for ESP-NOW RC Controller v1.2
// Board   : Arduino Nano (ATmega328P, 16MHz)
// Display : ST7920 128x64 LCD (SPI/serial mode, PSB=GND)
// Input   : NanoPacket via SoftwareSerial D2 (from ESP32 GPIO22)
// Library : U8g2 (Install via Library Manager)
//
// ST7920 Wiring (SPI mode — PSB tied to GND):
//   Nano D10 → RS  (CS)        Nano D11 → R/W (SID/MOSI)
//   Nano D13 → E   (CLK)       Nano D8  → RST
//   5V → VDD, BLA(via 10Ω)    GND → VSS, PSB, BLK
//   10kΩ pot wiper → V0  (contrast, ends to 5V/GND)
//
// Serial from ESP32:
//   ESP32 GPIO22 → Nano D2  (direct, 3.3V logic readable by 5V Nano)
//   Common GND required
// ================================================================

#include <U8g2lib.h>
#include <SoftwareSerial.h>

// ── HARDWARE ─────────────────────────────────────
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /*CS=*/10, /*reset=*/8);
SoftwareSerial espSerial(2, 3);  // RX=D2 (from ESP32), TX=D3 unused

// ── PACKET DEFINITION ─────────────────────────────
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
  uint8_t flags;    // b0=LB b1=RB b2=LS b3=RS b4=SW1 b5=SW2 b6=CONN
  uint8_t safety;   // b0=sw1OK b1=sw2OK b2=p1OK b3=p2OK b4=batOK
  uint8_t ftr;
} NanoPacket;

NanoPacket live;

// ── SMOOTHED DISPLAY VALUES ───────────────────────
// Integer IIR — no floats needed
int sLX=0, sLY=0, sRX=0, sRY=0;   // -99..99
int sP1=0,  sP2=0,  sBAT=0;        // 0..100

// ── ANIMATION ─────────────────────────────────────
uint8_t  animTick  = 0;       // increments every frame (~20Hz)
bool     batBlink  = false;   // 2Hz blink flag for low battery
uint32_t lastPktMs = 0;       // last valid packet timestamp

// ── RX STATE MACHINE ─────────────────────────────
enum RxState { WAIT_HDR, READ_BODY };
RxState  rxState = WAIT_HDR;
uint8_t  rxBuf[PKT_SIZE];
uint8_t  rxIdx   = 0;

// ================================================================
//  RECEIVE & PARSE PACKET
// ================================================================
void receivePacket() {
  while (espSerial.available()) {
    uint8_t b = espSerial.read();
    switch (rxState) {
      case WAIT_HDR:
        if (b == PKT_HDR) { rxBuf[0] = b; rxIdx = 1; rxState = READ_BODY; }
        break;
      case READ_BODY:
        rxBuf[rxIdx++] = b;
        if (rxIdx == PKT_SIZE) {
          rxState = WAIT_HDR;
          rxIdx   = 0;
          if (rxBuf[PKT_SIZE - 1] == PKT_FTR) {
            memcpy(&live, rxBuf, PKT_SIZE);
            lastPktMs = millis();
          }
        }
        break;
    }
  }
}

// ================================================================
//  SMOOTH STEP — integer IIR, converges without float
// ================================================================
int smoothStep(int cur, int tgt) {
  int d = tgt - cur;
  if (d == 0 || abs(d) <= 1) return tgt;
  return cur + d / 4;
}

void updateSmooth() {
  sLX  = smoothStep(sLX,  live.LX);
  sLY  = smoothStep(sLY,  live.LY);
  sRX  = smoothStep(sRX,  live.RX);
  sRY  = smoothStep(sRY,  live.RY);
  sP1  = smoothStep(sP1,  live.P1);
  sP2  = smoothStep(sP2,  live.P2);
  sBAT = smoothStep(sBAT, live.BAT);
}

// ================================================================
//  DRAW HELPERS
// ================================================================

// Analog joystick circle with moving dot
// cx,cy = center  r = circle radius  lx,ly = stick value -99..99
 // REPLACE the existing drawJoystick() with this:

void drawJoystick(uint8_t cx, uint8_t cy, uint8_t half, int lx, int ly) {
  uint8_t x0 = cx - half;
  uint8_t y0 = cy - half;
  uint8_t sz = half * 2 + 1;

  // ── Outer square border ──────────────────────────
  u8g2.drawFrame(x0, y0, sz, sz);

  // ── Crosshair (dashed via every-other pixel) ─────
  for (uint8_t i = x0 + 2; i < x0 + sz - 2; i += 2)
    u8g2.drawPixel(i, cy);           // horizontal dash
  for (uint8_t i = y0 + 2; i < y0 + sz - 2; i += 2)
    u8g2.drawPixel(cx, i);           // vertical dash

  // ── Corner tick marks ────────────────────────────
  // tiny L-shaped marks at each corner for Radiomaster feel
  u8g2.drawPixel(x0 + 1, y0 + 1);
  u8g2.drawPixel(x0 + sz - 2, y0 + 1);
  u8g2.drawPixel(x0 + 1, y0 + sz - 2);
  u8g2.drawPixel(x0 + sz - 2, y0 + sz - 2);

  // ── Moving dot ────────────────────────────────────
  // map -99..99 → pixel offset, leave 3px margin inside border
  uint8_t inner = half - 3;
  int8_t  dx    = (int8_t)((lx * (int8_t)inner) / 99);
  int8_t  dy    = (int8_t)((ly * (int8_t)inner) / 99);

  // 3×3 solid square dot (cleaner than disc on small pixel grids)
  uint8_t dotX = (uint8_t)(cx + dx - 1);
  uint8_t dotY = (uint8_t)(cy - dy - 1);   // Y flipped: +ly = up
  u8g2.drawBox(dotX, dotY, 3, 3);
}

// Filled button indicator box — inverts text color when active
void drawBtn(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
             const char *label, bool active) {
  if (active) {
    u8g2.drawBox(x, y, w, h);
    u8g2.setDrawColor(0);        // white text on black fill
    u8g2.drawStr(x + 1, y + h - 1, label);
    u8g2.setDrawColor(1);        // restore
  } else {
    u8g2.drawFrame(x, y, w, h);
    u8g2.drawStr(x + 1, y + h - 1, label);
  }
}

// Horizontal slider bar with label and percentage
// x,y = top-left of bar  w = total row width  h = bar height  pct = 0..100
void drawSlider(uint8_t x, uint8_t y, uint8_t totalW, uint8_t h,
                uint8_t pct, const char *label) {
  const uint8_t LBL_W  = 14;   // reserved for label (e.g. "P1")
  const uint8_t PCT_W  = 20;   // reserved for "100%" text
  uint8_t       bx     = x + LBL_W;
  uint8_t       bw     = totalW - LBL_W - PCT_W;

  // Label (baseline at bottom of bar)
  u8g2.drawStr(x, y + h, label);

  // Bar outline
  u8g2.drawFrame(bx, y, bw, h);

  // Fill
  uint8_t fill = (uint16_t)pct * (bw - 2) / 100;
  if (fill) u8g2.drawBox(bx + 1, y + 1, fill, h - 2);

  // Percentage text
  char buf[6]; sprintf(buf, "%3d%%", pct);
  u8g2.drawStr(bx + bw + 2, y + h, buf);
}

// Battery shaped icon with proportional fill and blinking when low
// x,y = top-left of body  bw = body width  h = body height
void drawBattery(uint8_t x, uint8_t y, uint8_t bw, uint8_t h,
                 uint8_t pct, bool blink) {
  // Body
  u8g2.drawFrame(x, y, bw, h);
  // Positive nub (right side, centered vertically)
  u8g2.drawBox(x + bw, y + h / 2 - 1, 3, 3);
  // Fill (blinks when low)
  if (!blink || batBlink) {
    uint8_t fill = (uint16_t)pct * (bw - 2) / 100;
    if (fill) u8g2.drawBox(x + 1, y + 1, fill, h - 2);
  }
}

// Signal bars — 3 bars, fills animate on connection activity
void drawSignal(uint8_t x, uint8_t y, bool connected) {
  // Bar heights: 3, 5, 8 (px), all bottom-aligned
  const uint8_t heights[3] = { 3, 5, 8 };
  const uint8_t BW = 4, GAP = 2;
  const uint8_t BASE = y + 8;  // bottom edge

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t bx = x + i * (BW + GAP);
    uint8_t bh = heights[i];
    uint8_t by = BASE - bh;
    bool fillBar = connected && (i < 2 || ((animTick / 5) % 2 == 0));
    if (fillBar) u8g2.drawBox(bx, by, BW, bh);
    else         u8g2.drawFrame(bx, by, BW, bh);
  }
}

// ================================================================
//  SCREEN: NORMAL OPERATION
//
//  Layout (128×64):
//  ┌──────────────────────────────────────────────────────────────┐
//  │ [L Joystick]  LB  LS  [SIG] RS  RB  [R Joystick]          │ y 0..28
//  │    28×28      [S1 toggle]   [S2 toggle]    28×28           │
//  │──────────────────────────────────────────────────────────────│ y 30
//  │ P1 [█████████░░░░░░] 100%                                   │ y 32..40
//  │ P2 [████░░░░░░░░░░░]  35%                                   │ y 43..51
//  │ BAT [==] [▓▓▓▓░░] 85%                                      │ y 54..62
//  └──────────────────────────────────────────────────────────────┘
// ================================================================
void drawNormalScreen() {
  bool connLost = (millis() - lastPktMs > 600);
  bool LB   = live.flags & (1 << 0);
  bool RB   = live.flags & (1 << 1);
  bool LS   = live.flags & (1 << 2);
  bool RS   = live.flags & (1 << 3);
  bool SW1  = live.flags & (1 << 4);
  bool SW2  = live.flags & (1 << 5);
  bool CONN = !connLost && (live.flags & (1 << 6));

  u8g2.setFont(u8g2_font_4x6_tf);

  // ── Joysticks ─────────────────────────────────
  drawJoystick(14, 14, 13, sLX, sLY);
  drawJoystick(113, 14, 13, sRX, sRY);

  // ── Button row (y=1..8) ───────────────────────
  // 9×8 boxes: LB LS (left of center) RS RB (right of center)
  drawBtn(29,  1, 9, 8, "LB", LB);
  drawBtn(42,  1, 9, 8, "LS", LS);
  drawBtn(69,  1, 9, 8, "RS", RS);
  drawBtn(82,  1, 9, 8, "RB", RB);

  // ── Toggle + signal row (y=15..23) ────────────
  // S1 / S2 are wider boxes (14×8) for two-char label
  drawBtn(29, 15, 14, 8, "S1", SW1);
  drawBtn(84, 15, 14, 8, "S2", SW2);
  drawSignal(54, 15, CONN);    // 16px wide signal icon

  // ── Separator line ────────────────────────────
  u8g2.drawHLine(0, 30, 128);

  // ── Sliders (use font 4x6) ────────────────────
  // P1: bar y=32, h=8, label+pct at baseline y=40
  drawSlider(0, 32, 128, 8, sP1, "P1");
  // P2: bar y=43, h=8
  drawSlider(0, 43, 128, 8, sP2, "P2");

  // ── Battery ───────────────────────────────────
  // "BAT" label, battery body, percentage
  u8g2.drawStr(0, 62, "BAT");      // baseline y=62
  drawBattery(16, 54, 74, 8, sBAT, sBAT < 20);
  char batbuf[6]; sprintf(batbuf, "%3d%%", sBAT);
  u8g2.drawStr(93, 62, batbuf);
}

// ================================================================
//  SCREEN: CAUTION (startup safety check)
//
//  ┌──────────────────────────────┐
//  │    !! CAUTION !!             │
//  │──────────────────────────────│
//  │ SW1: OK    SW2: OK           │
//  │ P1 : MIN   P2 : OK           │
//  │ BAT: 82%                     │
//  │──────────────────────────────│
//  │ Waiting .....                │
//  └──────────────────────────────┘
// ================================================================
// ── NEW HELPER: vertical slider (fill rises from bottom = high value) ──
void drawVerticalSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct) {
  u8g2.drawFrame(x, y, w, h);
  uint8_t fillH = (uint16_t)pct * (h - 2) / 100;
  if (fillH)
    u8g2.drawBox(x + 1, y + h - 1 - fillH, w - 2, fillH);
}

// ── NEW HELPER: vertical mobile battery (nub on top, fill from bottom) ──
void drawVerticalBattery(uint8_t bx, uint8_t by, uint8_t bw, uint8_t bh,
                         uint8_t pct, bool blink) {
  uint8_t nw = bw / 2, nx = bx + (bw - nw) / 2;
  u8g2.drawBox(nx, by - 2, nw, 2);          // nub above body
  u8g2.drawFrame(bx, by, bw, bh);           // body outline
  if (!blink) {
    uint8_t fh = (uint16_t)pct * (bh - 2) / 100;
    if (fh) u8g2.drawBox(bx + 1, by + bh - 1 - fh, bw - 2, fh);
  }
}

// ================================================================
//  UPDATED drawNormalScreen()
// ================================================================
void drawNormalScreen() {
  bool connLost = (millis() - lastPktMs > 600);
  bool LB   = live.flags & (1 << 0);
  bool RB   = live.flags & (1 << 1);
  bool LS   = live.flags & (1 << 2);
  bool RS   = live.flags & (1 << 3);
  bool SW1  = live.flags & (1 << 4);
  bool SW2  = live.flags & (1 << 5);
  bool CONN = !connLost && (live.flags & (1 << 6));

  u8g2.setFont(u8g2_font_4x6_tf);

  // ── TOP ZONE  y=0..28 ────────────────────────────────────────
  // L joystick: square box, cx=13, cy=14, half=12
  drawJoystick(13, 14, 12, sLX, sLY);

  // P1 vertical slider: right of L stick
  // frame x=27..31 (5px wide), y=2..26 (25px tall)
  drawVerticalSlider(27, 2, 5, 25, sP1);

  // Signal bars: 3 bars, BW=4, GAP=3, centered at x=64, bot=25
  // heights: [6, 11, 16] px — top bar blinks when connected
  {
    const uint8_t BW = 4, GAP = 3, BOT = 25;
    const uint8_t HTS[3] = {6, 11, 16};
    const uint8_t x0 = 64 - 9;   // center 64, total width 18, half=9
    bool blink = (animTick / 5) % 2;
    for (uint8_t i = 0; i < 3; i++) {
      uint8_t bx = x0 + i * (BW + GAP);
      uint8_t bh = HTS[i];
      uint8_t by = BOT - bh;
      bool filled = CONN && (i < 2 || blink);
      if (filled) u8g2.drawBox(bx, by, BW, bh);
      else        u8g2.drawFrame(bx, by, BW, bh);
    }
  }

  // P2 vertical slider: left of R stick
  // frame x=96..100 (5px wide), y=2..26 (25px tall)
  drawVerticalSlider(96, 2, 5, 25, sP2);

  // R joystick: square box, cx=114, cy=14, half=12
  drawJoystick(114, 14, 12, sRX, sRY);

  // ── SEPARATOR  y=29 ──────────────────────────────────────────
  u8g2.drawHLine(0, 29, 128);

  // ── MID ZONE  y=30..45 ───────────────────────────────────────
  // Buttons: 10×8 each, y=31
  drawBtn(1,  31, 10, 8, "LB", LB);
  drawBtn(13, 31, 10, 8, "LS", LS);
  drawBtn(25, 31, 10, 8, "S1", SW1);
  drawBtn(66, 31, 10, 8, "S2", SW2);
  drawBtn(78, 31, 10, 8, "RS", RS);
  drawBtn(90, 31, 10, 8, "RB", RB);

  // Vertical battery: bw=8, bh=12, body y=33..44, nub at y=31
  bool batLow = (sBAT < 20) && batBlink;
  drawVerticalBattery(115, 33, 8, 12, sBAT, batLow);

  // ── SEPARATOR  y=46 ──────────────────────────────────────────
  u8g2.drawHLine(0, 46, 128);

  // ── BOTTOM ZONE  y=47..63 ────────────────────────────────────
  // "P1" label — column-aligned under P1 slider (x=27 area)
  u8g2.drawStr(24, 59, "P1");

  // "URC V1.2" outlined box — centered on screen
  u8g2.drawFrame(40, 51, 48, 10);
  u8g2.drawStr(42, 59, "URC V1.2");

  // "P2" label — column-aligned under P2 slider (x=96 area)
  u8g2.drawStr(90, 59, "P2");

  // Battery percentage — right-aligned under battery icon (cx=119)
  char batbuf[5];
  sprintf(batbuf, sBAT < 10 ? "  %d%%" : sBAT < 100 ? " %d%%" : "%d%%", sBAT);
  u8g2.drawStr(108, 59, batbuf);
}

// ================================================================
//  SCREEN: ALL CLEAR (shown briefly after caution passes)
// ================================================================
void drawAllClearScreen() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(4,  22, "** ALL CLEAR **");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(22, 38, "System Ready");
  u8g2.drawStr(18, 50, "Starting up ...");
}

// ================================================================
//  SCREEN: LOW BATTERY
// ================================================================
void drawLowBatScreen() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 14, "LOW BATTERY!");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(24, 26, "Radio  OFF");

  // Large battery icon
  u8g2.drawFrame(10, 32, 96, 20);      // body
  u8g2.drawBox(106, 38, 5, 8);         // nub
  if (batBlink) {
    uint8_t fill = (uint16_t)live.BAT * 94 / 100;
    if (fill) u8g2.drawBox(11, 33, fill, 18);
  }
  // Percentage centered inside
  char buf[6]; sprintf(buf, "%d%%", live.BAT);
  u8g2.drawStr(52, 45, buf);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  espSerial.begin(19200);

  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);

  // Default live data while waiting for first packet
  memset(&live, 0, sizeof(live));
  live.BAT   = 100;
  live.state = STATE_CAUTION;

  lastPktMs = millis();
}

// ================================================================
//  LOOP — render at 20 FPS
// ================================================================
void loop() {
  static uint32_t lastFrame = 0;

  receivePacket();   // always drain serial buffer first

  uint32_t now = millis();
  if (now - lastFrame < 50) return;   // cap at 20 FPS
  lastFrame = now;

  animTick++;
  if (animTick >= 200) animTick = 0;
  batBlink = ((animTick % 10) < 5);   // 2Hz blink

  updateSmooth();

  u8g2.clearBuffer();

  switch (live.state) {
    case STATE_CAUTION:  drawCautionScreen();   break;
    case STATE_ALLCLEAR: drawAllClearScreen();  break;
    case STATE_LOWBAT:   drawLowBatScreen();    break;
    default:             drawNormalScreen();    break;
  }

  u8g2.sendBuffer();
}