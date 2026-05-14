// ============================================================
//  KS0108 128x64 Parallel LCD — Pin Configuration (U8g2)
//  Controller: KS0108 (JHS12864 or compatible)
//  Library:    U8g2  (Full framebuffer mode "_F")
//  MCU target: Arduino Mega / boards with enough pins
// ============================================================
//
//  KS0108 DATA BUS (D0–D7) — 8-bit parallel data lines
//    D0 → Arduino pin 2
//    D1 → Arduino pin 3
//    D2 → Arduino pin 4
//    D3 → Arduino pin 5
//    D4 → Arduino pin 6
//    D5 → Arduino pin 7
//    D6 → Arduino pin 8
//    D7 → Arduino pin 9
//
//  KS0108 CONTROL LINES
//    EN  (Enable / clock strobe)  → Arduino pin 16
//    DC  (Data/Instruction select, aka D/I or RS) → Arduino pin 14
//    CS1 (Chip Select, left  half) → Arduino pin 10
//    CS2 (Chip Select, right half) → Arduino pin 11
//    RST (Reset) → U8X8_PIN_NONE  (tied HIGH or unused)
//
//  POWER
//    VDD → 5 V
//    GND → GND
//    V0  → Contrast pot wiper (10 kΩ between VDD & GND)
//    BLA → Backlight anode  (via 33–100 Ω resistor to 5 V) or 3.3v
//    BLK → Backlight cathode → GND
//
//  R/W pin of the KS0108 MUST be tied LOW (Write-only mode).
//  U8g2 never reads from the display.
// ============================================================
//  JHS12864 (KS0108) LCD — 20-Pin Wiring Reference
// ============================================================
//
//  LCD Pin │ Label │ Connects To
//  ────────┼───────┼──────────────────────────────────────────
//    1     │ GND   │ GND
//    2     │ VCC   │ 5V
//    3     │ V0    │ NC (contrast fixed on module)
//    4     │ RS    │ Arduino pin 19  (D/I — Data or Instruction) / A5
//    5     │ R/W   │ GND             (always LOW = write only)
//    6     │ EN    │ Arduino pin 18  (Enable / clock strobe) / A4
//    7     │ D0    │ Arduino pin 2
//    8     │ D1    │ Arduino pin 3
//    9     │ D2    │ Arduino pin 4
//   10     │ D3    │ Arduino pin 5
//   11     │ D4    │ Arduino pin 6
//   12     │ D5    │ Arduino pin 7
//   13     │ D6    │ Arduino pin 8
//   14     │ D7    │ Arduino pin 9
//   15     │ CS1   │ Arduino pin 10  (Chip Select — left  half)
//   16     │ CS2   │ Arduino pin 11  (Chip Select — right half)
//   17     │ RST   │ NC -- Blank(Floating) 
//   18     │ VEE   │ NC -- Blank(Floating) 
//   19     │ BLA   │ 5V via 33Ω resistor (Backlight anode)/3.3v
//   20     │ BLK   │ GND             (Backlight cathode)
//
//  R/W (pin 5) MUST be tied to GND — U8g2 is write-only.
// ============================================================

#include <Arduino.h>
#include <U8g2lib.h>

//            Data pins ────────────────────┐
//            (D0 … D7 in order)            │
//                                          ▼
U8G2_KS0108_128X64_F u8g2(U8G2_R0,   // Rotation: 0°
  2, 3, 4, 5, 6, 7, 8, 9,            // D0–D7
  /*enable=*/ 18,                     // EN
  /*dc=*/     19,                     // D/I (Data / Instruction)
  /*cs1=*/    10,                     // CS1 – left  64-px half
  /*cs2=*/    11,                     // CS2 – right 64-px half
  U8X8_PIN_NONE                       // RST – not connected
);

// ── Helpers ──────────────────────────────────────────────────

// Print a header banner at the top of the screen
void pre(const char* title = nullptr) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);

  // Inverted top bar
  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 0, 128, 13);
  u8g2.setDrawColor(0);                       // White text on black box
  u8g2.drawStr(2, 10, title ? title : " U8g2 Library ");
  u8g2.setDrawColor(1);                       // Back to normal

  u8g2.setFont(u8g2_font_6x10_tr);
}

// Draw a vertical bar (column) in normal or inverse colour
void draw_bar(uint8_t col_px, bool is_inverse) {
  // Each "column" in the old U8x8 was 8 px wide (tile width)
  uint8_t x = col_px * 8;
  if (is_inverse) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(x, 0, 8, 64);
  } else {
    u8g2.setDrawColor(0);
    u8g2.drawBox(x, 0, 8, 64);
  }
  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

// Draw one row of ASCII characters starting at 'start'
void draw_ascii_row(uint8_t row_tile, int start) {
  // U8x8 tiles are 8×8 px; map to pixel coordinates
  uint8_t y = row_tile * 8 + 7;   // baseline for u8g2 drawStr
  u8g2.setFont(u8g2_font_6x10_tr);
  for (uint8_t c = 0; c < 16; c++) {    // 128/8 = 16 tile columns
    int a = start + c;
    if (a >= 32 && a <= 255) {
      char buf[2] = { (char)a, '\0' };
      u8g2.drawStr(c * 8, y, buf);
    }
  }
}

// ── Setup ────────────────────────────────────────────────────

void setup() {
  delay(1000);
  u8g2.begin();
  u8g2.setContrast(255);
}

// ── Main loop ────────────────────────────────────────────────

void loop() {
  // --- Screen 1: Library credit ---
  pre(" U8g2 Library ");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 26, "github.com/");
  u8g2.drawStr(0, 38, "olikraus/u8g2");
  u8g2.sendBuffer();
  delay(2000);

  // Tile size info (U8g2 full buffer = 128×64)
  u8g2.drawStr(0, 50, "Display: 128x64 px");
  u8g2.sendBuffer();
  delay(2000);

  // --- Screen 2: Countdown ---
  pre("Countdown");
  for (int i = 19; i > 0; i--) {
    u8g2.setDrawColor(0);
    u8g2.drawBox(20, 20, 40, 14);   // Erase previous number
    u8g2.setDrawColor(1);
    char buf[4];
    snprintf(buf, sizeof(buf), "%d", i);
    u8g2.setFont(u8g2_font_9x15_tr);
    u8g2.drawStr(24, 32, buf);
    u8g2.sendBuffer();
    delay(150);
  }

  // --- Screen 3: Sweeping bar (left → right) ---
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  for (uint8_t c = 0; c < 16; c++) {
    draw_bar(c, true);
    if (c > 0) draw_bar(c - 1, false);
    delay(50);
  }
  draw_bar(15, false);

  // --- Screen 4: ASCII table scroll ---
  for (uint8_t d = 0; d < 8; d++) {
    u8g2.clearBuffer();
    pre("ASCII Table");
    for (uint8_t r = 1; r < 7; r++) {        // 6 tile rows below header
      draw_ascii_row(r, (r - 1 + d) * 16 + 32);
    }
    u8g2.sendBuffer();
    delay(400);
  }

  // --- Screen 5: Sweeping bar (right → left) ---
  draw_bar(15, true);
  for (uint8_t c = 15; c > 0; c--) {
    draw_bar(c - 1, true);
    draw_bar(c, false);
    delay(50);
  }
  draw_bar(0, false);

  // --- Screen 6: Scale-up demo ---
  pre("Scale Up");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 26, "Small");
  // Simulate 2x2 by drawing larger font
  u8g2.setFont(u8g2_font_9x15B_tr);
  u8g2.drawStr(0, 55, "Scale Up");
  u8g2.sendBuffer();
  delay(3000);

  // --- Screen 7: Big numeric font ---
  pre("3x6 Font");
  u8g2.sendBuffer();
  u8g2.setFont(u8g2_font_inb16_mr);   // Large number font
  for (int i = 0; i < 100; i++) {
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 20, 80, 25);
    u8g2.setDrawColor(1);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", i);
    u8g2.drawStr(0, 45, buf);
    u8g2.sendBuffer();
    delay(10);
  }

  // --- Screen 8: Weather icons ---
  pre("Weather Icons");
  u8g2.setFont(u8g2_font_open_iconic_weather_4x_t);
  for (uint8_t c = 0; c < 6; c++) {
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 14, 36, 36);
    u8g2.setDrawColor(1);
    u8g2.drawGlyph(0, 50, '@' + c);   // Glyph codes match U8x8
    u8g2.sendBuffer();
    delay(300);
  }

  // --- Screen 9: println demo ---
  pre("Print demo");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 26, "print \\n");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr(0, 38, "println");
  u8g2.sendBuffer();
  delay(500);
  u8g2.drawStr(0, 50, "done");
  u8g2.sendBuffer();
  delay(1500);

  // --- Screen 10: Fill then wipe line by line ---
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 0, 128, 64);   // fillDisplay equivalent
  u8g2.sendBuffer();
  for (uint8_t r = 0; r < 8; r++) {
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, r * 8, 128, 8);   // clearLine equivalent
    u8g2.setDrawColor(1);
    u8g2.sendBuffer();
    delay(100);
  }
  delay(1000);
}