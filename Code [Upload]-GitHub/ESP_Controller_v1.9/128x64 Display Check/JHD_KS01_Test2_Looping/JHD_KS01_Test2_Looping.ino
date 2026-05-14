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

void setup() {
  Serial.begin(9600);
  Serial.println("KS0108 Init...");

  u8g2.begin();
  Serial.println("begin() done");

  // --- Test 1: Full pixel fill (all pixels ON) ---
  // If you see a fully lit screen, display is working
  // Adjust V0 pot until pixels are visible
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 0, 128, 64); // fill entire screen
  u8g2.sendBuffer();
  Serial.println("Full fill sent - adjust pot now");
  delay(3000);

  // --- Test 2: Text ---
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, "JHD12864E OK");
  u8g2.drawStr(0, 28, "KS0108 Nano");
  u8g2.drawStr(0, 44, "U8g2 Working!");
  u8g2.sendBuffer();
  Serial.println("Text sent");
  delay(3000);

  // --- Test 3: Graphics ---
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);       // border
  u8g2.drawLine(0, 0, 128, 64);        // diagonal
  u8g2.drawLine(128, 0, 0, 64);        // other diagonal
  u8g2.drawCircle(64, 32, 20);         // center circle
  u8g2.sendBuffer();
  Serial.println("Graphics sent");
}

void loop() {
  // Animate a bouncing pixel to confirm loop is running
  static int x = 0;
  static int dx = 1;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, "LOOP RUNNING");
  u8g2.drawBox(x, 40, 8, 8);  // bouncing box
  u8g2.sendBuffer();

  x += dx;
  if (x >= 120 || x <= 0) dx = -dx;
  delay(30);
}