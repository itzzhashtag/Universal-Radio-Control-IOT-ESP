/*
  JHD12864E KS0108 - Nano Test
  Display Pin Reference:
    Pin4  RS  -> A1
    Pin5  RW  -> A0  (this is Enable in U8g2)
    Pin6  E   -> A2  (this is DC/RS in U8g2)
    Pin7-14 DB0-DB7 -> D2-D9
    Pin15 CS1 -> A3
    Pin16 CS2 -> A4
    Pin17 RST -> A5
*/

#include <Arduino.h>
#include <U8g2lib.h>
U8G2_KS0108_128X64_F u8g2(U8G2_R0, 2, 3, 4, 5, 6, 7, 8, 9, /*enable=*/16, /*dc=*/14, /*cs1=*/10, /*cs2=*/11, U8X8_PIN_NONE /*reset=12*/);
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