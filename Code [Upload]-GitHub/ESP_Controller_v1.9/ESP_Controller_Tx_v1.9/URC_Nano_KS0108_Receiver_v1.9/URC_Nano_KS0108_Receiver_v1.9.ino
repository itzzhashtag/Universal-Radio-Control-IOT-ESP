// ================================================================
//  Arduino Nano — KS0108 Display Receiver  v2.0
//
//  Role  : Slave display — receives 18-byte UART packets from ESP32
//          and renders the appropriate screen on JHD12864E (KS0108)
//
//  ── What changed from v1.9 ──────────────────────────────────────
//  • Packet is now 18 bytes (was 16) — added screenID + cautionFlags
//  • Handles all four screens:
//      SCREEN_BOOT    → "URC Booting / Please Wait" + bouncing box
//      SCREEN_CAUTION → safety gate with live SW/POT/BAT status
//      SCREEN_LOWBAT  → low battery warning with animated icon
//      SCREEN_NORMAL  → live controller state (joysticks, pots, etc.)
//  • LCD_FPS is tunable at the top of the file
//  • Improved comments throughout
//
//  ── Memory budget (Nano has 2 KB SRAM) ──────────────────────────
//  U8g2 full-buffer  : 128×64/8 = 1024 bytes
//  Stack + locals    : ~300 bytes
//  Packet buffer     : 18 bytes
//  Total             : ~1342 bytes  |  free margin ~680 bytes
//
//  ── Wiring (KS0108 parallel interface) ──────────────────────────
//  LCD D0–D7  → Nano pins 2–9
//  LCD EN     → Nano A4  (pin 18)
//  LCD RS/DC  → Nano A5  (pin 19)
//  LCD CS1    → Nano pin 10
//  LCD CS2    → Nano pin 11
//  LCD R/W    → GND  (write-only)
//  LCD RST    → not connected  (U8X8_PIN_NONE)
//  LCD VCC    → 5 V     GND → GND
//
//  ── ESP32 → Nano UART ───────────────────────────────────────────
//  ESP32 TX (GPIO1)  → Nano RX (pin 0)
//  GND               → GND
//  !! Disconnect pin 0 before uploading sketches — it conflicts !!
//
//  ── Required Library ────────────────────────────────────────────
//  U8g2 by olikraus  (Library Manager → search "U8g2")
// ================================================================

#include <Arduino.h>
#include <U8g2lib.h>

// ================================================================
//  ★  USER-TUNABLE CONSTANTS  ★
// ================================================================

// Target display refresh rate in Hz.
// Higher = snappier joystick response on the LCD.
// Nano max practical rate with KS0108 full-buffer is ~30 Hz.
// Keep at or below 30 Hz to avoid rendering artefacts.
#define LCD_FPS 60  // Hz — drives loop() delay

// ================================================================
//  SCREEN ID CONSTANTS  (must match ESP32 firmware)
// ================================================================
#define SCREEN_BOOT 0x01     // Boot animation (capacitor settle)
#define SCREEN_CAUTION 0x02  // Startup safety gate
#define SCREEN_LOWBAT 0x03   // Low battery safe mode
#define SCREEN_NORMAL 0x04   // Normal operation

// ================================================================
//  U8g2 CONSTRUCTOR
//  KS0108 128×64 full-buffer, parallel 8-bit interface.
//  U8G2_R0 = no rotation.
// ================================================================
U8G2_KS0108_128X64_F u8g2(U8G2_R0,
                          2, 3, 4, 5, 6, 7, 8, 9,  // D0–D7 data bus
                          18,                      // EN  (A4)
                          19,                      // DC  (A5)
                          10,                      // CS1 — left  half (pixels 0–63)
                          11,                      // CS2 — right half (pixels 64–127)
                          U8X8_PIN_NONE            // RST — not used
);

// ================================================================
//  UART PACKET FORMAT  (18 bytes, same as ESP32 TX firmware)
//
//  [0]  0xAA  start marker
//  [1]  screenID     — SCREEN_* constant
//  [2]  cautionFlags — bit0=sw1OK  bit1=sw2OK  bit2=p1OK  bit3=batOK
//  [3]  Lx + 99  (0..198)
//  [4]  Ly + 99
//  [5]  Rx + 99
//  [6]  Ry + 99
//  [7]  Pot1  (0..100)
//  [8]  Pot2  (0..100)
//  [9]  BAT % (0..100)
//  [10] button bitmask:
//         bit0=LABt  bit1=RABt  bit2=LBt  bit3=RBt
//         bit4=TSW1  bit5=TSW2  bit6=gConnected
//  [11-16] reserved
//  [17] XOR checksum of bytes [1]..[16]
// ================================================================
#define PKT_LEN 18
#define PKT_START 0xAA

static uint8_t rxBuf[PKT_LEN];  // raw incoming bytes
static uint8_t rxPos = 0;       // write position in rxBuf
static bool synced = false;     // true = found start byte, collecting

// ================================================================
//  DECODED DISPLAY STATE
//  Populated from the last valid packet received.
//  Kept as module-level vars so all draw functions can access them.
// ================================================================
static uint8_t dScreen = SCREEN_BOOT;  // current screen to render
static uint8_t dCaution = 0;           // caution flag bitmask
static int8_t dLx = 0, dLy = 0;        // left  joystick (−99..+99)
static int8_t dRx = 0, dRy = 0;        // right joystick
static uint8_t dP1 = 50, dP2 = 50;     // potentiometers (0..100)
static uint8_t dBat = 100;             // battery % (0..100)
static uint8_t dBtns = 0;              // button bitmask (see packet format)
static bool dConn = false;             // ESP-NOW link alive

// ================================================================
//  ANIMATION STATE
//  animTick cycles 0–199 and is incremented each loop iteration.
//  Used by all animated elements (signal bars, bat blink, boot box).
// ================================================================
static uint8_t animTick = 0;

// Boot screen bouncing box state (persists between frames)
static int8_t bootBoxX = 0;   // current X position of box
static int8_t bootBoxDX = 1;  // direction: +1 = right, -1 = left

// ================================================================
//  LOOP DELAY  (derived from LCD_FPS)
// ================================================================
static const uint16_t LOOP_MS = 1000 / LCD_FPS;

// ================================================================
//  PACKET PARSER
//  Feed bytes one at a time from the UART receive buffer.
//  Returns true when a complete, valid packet has been parsed and
//  dScreen / all display state variables have been updated.
// ================================================================
bool parseByte(uint8_t b) {
  // ── Phase 1: hunt for start marker ──────────────────────────
  if (!synced) {
    if (b == PKT_START) {
      synced = true;
      rxPos = 0;
      rxBuf[rxPos++] = b;  // store start byte at [0]
    }
    return false;
  }

  // ── Phase 2: collect remaining bytes ─────────────────────────
  rxBuf[rxPos++] = b;

  if (rxPos < PKT_LEN) return false;  // not done yet

  // ── Phase 3: verify XOR checksum ─────────────────────────────
  // Checksum is XOR of bytes [1]..[16], stored in [17]
  uint8_t cs = 0;
  for (uint8_t i = 1; i < 17; i++) cs ^= rxBuf[i];

  synced = false;  // ready to hunt for next packet regardless
  rxPos = 0;

  if (cs != rxBuf[17]) return false;  // bad checksum — discard silently

  // ── Phase 4: unpack fields into display state ─────────────────
  dScreen = rxBuf[1];
  dCaution = rxBuf[2];

  // Joystick axes: subtract 99 to recover signed value
  dLx = (int8_t)(rxBuf[3] - 99);
  dLy = (int8_t)(rxBuf[4] - 99);
  dRx = (int8_t)(rxBuf[5] - 99);
  dRy = (int8_t)(rxBuf[6] - 99);

  dP1 = rxBuf[7];
  dP2 = rxBuf[8];
  dBat = rxBuf[9];
  dBtns = rxBuf[10];
  dConn = (rxBuf[10] & 0x40) != 0;  // bit 6 = gConnected

  return true;
}

// ================================================================
//  DRAW HELPERS — shorthand wrappers + shared widgets
// ================================================================

// Outline rectangle
inline void fr(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  u8g2.drawFrame(x, y, w, h);
}

// Filled rectangle
inline void fb(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  u8g2.drawBox(x, y, w, h);
}

// ─────────────────────────────────────────────────────────────────
//  JOYSTICK WIDGET
//  Draws a square box with a dashed crosshair and a 3×3 filled dot
//  that tracks the joystick position.
//
//  cx, cy  = centre pixel of the widget
//  half    = half-size of the square (box goes cx±half, cy±half)
//  lx, ly  = axis values −99..+99 (lx +99 = full right, ly +99 = up)
// ─────────────────────────────────────────────────────────────────
void drawJoy(uint8_t cx, uint8_t cy, uint8_t half, int8_t lx, int8_t ly) {
  uint8_t x0 = cx - half;
  uint8_t y0 = cy - half;
  uint8_t sz = half * 2 + 1;

  fr(x0, y0, sz, sz);  // outer box

  // Dashed horizontal crosshair across the centre row
  for (uint8_t i = x0 + 2; i < x0 + sz - 2; i += 2)
    u8g2.drawPixel(i, cy);

  // Dashed vertical crosshair down the centre column
  for (uint8_t i = y0 + 2; i < y0 + sz - 2; i += 2)
    u8g2.drawPixel(cx, i);

  // Corner tick marks for visual alignment
  u8g2.drawPixel(x0 + 1, y0 + 1);
  u8g2.drawPixel(x0 + sz - 2, y0 + 1);
  u8g2.drawPixel(x0 + 1, y0 + sz - 2);
  u8g2.drawPixel(x0 + sz - 2, y0 + sz - 2);

  // 3×3 moving dot: maps lx/ly to pixel offset inside the box.
  // inn = inner travel range (box half minus dot margin)
  uint8_t inn = half - 3;
  int8_t dx = (int8_t)((lx * (int8_t)inn) / 99);
  int8_t dy = (int8_t)((ly * (int8_t)inn) / 99);
  // Y axis is inverted: positive ly → dot moves up (smaller pixel y)
  fb(cx + dx - 1, cy - dy - 1, 3, 3);
}

// ─────────────────────────────────────────────────────────────────
//  VERTICAL SLIDER  (potentiometer indicator)
//  Fills from the bottom up. pct=100 → full bar, pct=0 → empty.
// ─────────────────────────────────────────────────────────────────
void drawVSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct) {
  fr(x, y, w, h);  // outline
  uint8_t fh = (uint16_t)pct * (h - 2) / 100;
  if (fh) fb(x + 1, y + h - 1 - fh, w - 2, fh);  // fill from bottom
}

// ─────────────────────────────────────────────────────────────────
//  SIGNAL STRENGTH BARS  (3 bars, bottom-aligned)
//  When connected: all bars solid, tallest bar blinks at ~3 Hz.
//  When disconnected: all bars are outlines only.
// ─────────────────────────────────────────────────────────────────
void drawSignal(uint8_t cx, uint8_t bot) {
  const uint8_t BW = 4;                  // bar width in pixels
  const uint8_t GAP = 3;                 // gap between bars
  const uint8_t x0 = cx - 9;             // leftmost bar X start
  const uint8_t HTS[3] = { 6, 11, 16 };  // heights: short, mid, tall

  // Tallest (rightmost) bar blinks at ~3 Hz when connected
  bool blink = (animTick % 10) < 5;

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t bx = x0 + i * (BW + GAP);
    uint8_t bh = HTS[i];
    uint8_t by = bot - bh;
    // Lower two bars are always solid when connected; top bar blinks
    bool filled = dConn && (i < 2 || blink);
    if (filled) fb(bx, by, BW, bh);
    else fr(bx, by, BW, bh);
  }
}

// ─────────────────────────────────────────────────────────────────
//  BUTTON WIDGET
//  Outline-only when inactive; inverted (white text on black box)
//  when active (pressed).
// ─────────────────────────────────────────────────────────────────
void drawBtn(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
             const char *lbl, bool act) {
  if (act) {
    fb(x, y, w, h);        // black fill
    u8g2.setDrawColor(0);  // switch to white ink
    u8g2.drawStr(x + 1, y + h - 1, lbl);
    u8g2.setDrawColor(1);  // restore black ink
  } else {
    fr(x, y, w, h);  // outline only
    u8g2.drawStr(x + 1, y + h - 1, lbl);
  }
}

// ─────────────────────────────────────────────────────────────────
//  VERTICAL BATTERY ICON
//  Positive nub on top. Fill rises from bottom.
//  When pct < 20 % the icon blinks at ~1 Hz (low battery warning).
// ─────────────────────────────────────────────────────────────────
void drawVBattery(uint8_t bx, uint8_t by, uint8_t bw, uint8_t bh, uint8_t pct) {
  uint8_t nw = bw / 2;
  uint8_t nx = bx + (bw - nw) / 2;
  fb(nx, by - 2, nw, 2);  // positive nub above body
  fr(bx, by, bw, bh);     // body outline

  // Blink the fill at ~1 Hz when battery is low
  bool lowBlink = (pct < 20) && ((animTick % 20) >= 10);
  if (!lowBlink) {
    uint8_t fh = (uint16_t)pct * (bh - 2) / 100;
    if (fh) fb(bx + 1, by + bh - 1 - fh, bw - 2, fh);
  }
}

// ================================================================
//  SCREEN: NORMAL OPERATION
//
//  Layout (128×64 pixels):
//  y 0–28  : L-joy | P1 bar | signal | P2 bar | R-joy
//  y 29    : horizontal divider
//  y 30–44 : button row  [LB][LS][S1]    [S2][RS][RB]
//  y 45    : horizontal divider
//  y 46–63 : name plate (left) | battery icon + % (right)
// ================================================================
void drawNormal() {
  u8g2.setFont(u8g2_font_4x6_tf);

  // ── TOP ROW ──────────────────────────────────────────────────
  drawJoy(13, 14, 12, dLx, dLy);   // Left  joystick: centred at (13,14)
  drawVSlider(27, 2, 5, 25, dP1);  // Pot1 slider: just right of L-stick
  drawSignal(64, 25);              // Signal bars: centred at x=64
  drawVSlider(96, 2, 5, 25, dP2);  // Pot2 slider: just left of R-stick
  drawJoy(114, 14, 12, dRx, dRy);  // Right joystick: centred at (114,14)

  u8g2.drawHLine(0, 29, 128);  // ── divider ──

  // ── MID ROW: 6 buttons ───────────────────────────────────────
  // Left cluster
  drawBtn(2, 31, 10, 8, "LB", dBtns & 0x04);   // LBt  (bit 2)
  drawBtn(14, 31, 10, 8, "LS", dBtns & 0x01);  // LABt (bit 0) — joystick click
  drawBtn(26, 31, 10, 8, "S1", dBtns & 0x10);  // TSW1 (bit 4)
  // Right cluster
  drawBtn(90, 31, 10, 8, "S2", dBtns & 0x20);   // TSW2 (bit 5)
  drawBtn(102, 31, 10, 8, "RS", dBtns & 0x02);  // RABt (bit 1) — joystick click
  drawBtn(114, 31, 10, 8, "RB", dBtns & 0x08);  // RBt  (bit 3)

  u8g2.drawHLine(0, 45, 128);  // ── divider ──

  // ── BOTTOM ROW ───────────────────────────────────────────────
  fr(0, 46, 58, 17);  // name box outline
  u8g2.drawStr(3, 54, " HASHTAG");
  u8g2.drawStr(3, 61, "URC v2.0");

  drawVBattery(96, 49, 8, 11, dBat);  // battery icon at right side

  // Right-align battery percentage text (1, 2, or 3 digits)
  char buf[5];
  if (dBat < 10) sprintf_P(buf, PSTR("  %d%%"), dBat);
  else if (dBat < 100) sprintf_P(buf, PSTR(" %d%%"), dBat);
  else sprintf_P(buf, PSTR("%d%%"), dBat);
  u8g2.drawStr(106, 61, buf);
}

// ================================================================
//  SCREEN: BOOT ANIMATION
//
//  Shown during ESP32's capacitor-charge settle period (BOOT_SETTLE_MS).
//  A box bounces left↔right across y=40 while "URC Booting" and
//  "Please Wait" are displayed above.
// ================================================================
void drawBoot() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(1, 14, "Universal Radio Comm!");

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(15, 28, "Booting Please wait...");

  // Bouncing 8×8 box: moves across y=40, x=0..120
  fb(bootBoxX, 40, 8, 8);

  // Advance the box position each frame
  bootBoxX += bootBoxDX;
  if (bootBoxX >= 120 || bootBoxX <= 0)
    bootBoxDX = -bootBoxDX;  // reverse direction at edges

  // Progress dots that cycle 0→3 at ~2 Hz (visual "alive" indicator)
  char dots[14] = "URC v2.0";
  uint8_t nd = (animTick / 8) % 4;
  for (uint8_t i = 0; i < nd; i++) dots[8 + i] = '.';
  dots[8 + nd] = '\0';
  u8g2.drawStr(30, 58, dots);
}

// ================================================================
//  SCREEN: STARTUP CAUTION GATE
//
//  Displays which safety checks are passing (OK) and which are
//  still waiting. Flag bits come from dCaution in the packet:
//    bit0 = SW1 OK    bit1 = SW2 OK
//    bit2 = P1  OK    bit3 = BAT OK
// ================================================================
void drawCaution() {
  // Extract individual flags from the bitmask
  bool sw1OK = !(dCaution & 0x01);
  bool sw2OK = !(dCaution & 0x02);
  bool p1OK = !(dCaution & 0x04);
  bool p2OK = !(dCaution & 0x08);
  bool batOK = (dCaution & 0x10);


  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(20, 10, "!! CAUTION !!");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_4x6_tf);

  // Left column: SW1 and P1 status
  u8g2.drawStr(2, 24, "SW1:");
  u8g2.drawStr(22, 24, sw1OK ? "HIGH!" : " OK ");
  u8g2.drawStr(2, 34, "P1 :");
  u8g2.drawStr(22, 34, p1OK ? "HIGH!" : " OK ");

  // Right column: SW2 status
  u8g2.drawStr(66, 24, "SW2:");
  u8g2.drawStr(88, 24, sw2OK ? "HIGH!" : " OK ");
  u8g2.drawStr(66, 34, "P2 :");
  u8g2.drawStr(88, 34, p2OK ? "HIGH!" : " OK ");

  // Battery row spanning full width
  // Battery row spanning full width
  char batBuf[22];

  if (batOK) {
    sprintf_P(batBuf, PSTR("BAT: OK    %3d%%"), dBat);
  } else {
    sprintf_P(batBuf, PSTR("BAT: LOW!  %3d%%"), dBat);
  }

  u8g2.drawStr(2, 44, batBuf);

  u8g2.drawHLine(0, 47, 128);

  // Animated waiting dots (~2 Hz) so display looks alive
  char dots[14] = "Waiting ";
  uint8_t nd = (animTick / 8) % 4;
  for (uint8_t i = 0; i < nd; i++) dots[8 + i] = '.';
  dots[8 + nd] = '\0';
  u8g2.drawStr(10, 58, dots);
}

// ================================================================
//  SCREEN: LOW BATTERY WARNING
//
//  Shown when ESP32 enters safe mode (batV <= BAT_LOW_V).
//  Radio is off on the ESP32 side; robot has been zeroed.
//  Large horizontal battery bar blinks to draw attention.
// ================================================================
void drawLowBat() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 14, "LOW BATTERY!");

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(24, 26, "Radio  OFF");

  // Large horizontal battery body (96×20 pixels)
  fr(10, 32, 96, 20);
  fb(106, 38, 5, 8);  // positive nub on right side

  // Fill blinks at ~1 Hz to signal danger
  bool blink = (animTick % 20) < 10;
  if (blink) {
    uint8_t fill = (uint16_t)dBat * 94 / 100;
    if (fill) fb(11, 33, fill, 18);
  }

  char buf[5];
  sprintf_P(buf, PSTR("%d%%"), dBat);
  u8g2.drawStr(50, 45, buf);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  // UART from ESP32 TX → Nano RX pin 0.
  // Baud must match UART_BAUD in the ESP32 firmware.
  Serial.begin(115200);

  // Initialise the KS0108 display
  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);

  // ── Splash screen — shown once on Nano power-up ───────────────
  // The ESP32 may not be ready yet; this gives the user visual
  // feedback that the Nano is alive and waiting.
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 22, "HASHTAG URC v1.9");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(38, 38, "KS0108 v1.9");
  u8g2.drawStr(24, 50, "Waiting for CPU...");
  u8g2.sendBuffer();
  delay(3000);  // show splash for 1.5 s before entering main loop
}

// ================================================================
//  LOOP  — runs at LCD_FPS Hz
// ================================================================
void loop() {
  // ── 1. Drain all available UART bytes into the packet parser ──
  // parseByte() updates dScreen and all display state on a valid packet.
  while (Serial.available()) {
    parseByte((uint8_t)Serial.read());
  }

  // ── 2. Advance animation counter ─────────────────────────────
  // Wraps 0–199. Shared by all animated elements.
  animTick++;
  if (animTick >= 200) animTick = 0;

  // ── 3. Render the current screen ─────────────────────────────
  u8g2.clearBuffer();

  switch (dScreen) {
    case SCREEN_BOOT:
      drawBoot();
      break;

    case SCREEN_CAUTION:
      drawCaution();
      break;

    case SCREEN_LOWBAT:
      drawLowBat();
      break;

    case SCREEN_NORMAL:
    default:
      drawNormal();
      break;
  }

  u8g2.sendBuffer();  // push frame buffer to LCD

  // ── 4. Wait to hit target frame rate ─────────────────────────
  delay(LOOP_MS);
}
