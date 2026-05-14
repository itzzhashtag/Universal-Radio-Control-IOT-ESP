#include <U8g2lib.h>
//U8G2_KS0108_128X64_1 u8g2(U8G2_R0, 2, 3, 4, 5, 6, 7, 8, 9, /*enable=*/16, /*dc=*/14, /*cs1=*/10, /*cs2=*/11, /*reset=*/12);
  U8G2_KS0108_128X64_F u8g2(U8G2_R0, 2, 3, 4, 5, 6, 7, 8, 9, /*enable=*/16, /*dc=*/14, /*cs1=*/10, /*cs2=*/11, U8X8_PIN_NONE /*reset=12*/);

void setup() {
  delay(1000);
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(10, 20, "WORKING");

  u8g2.drawFrame(0, 0, 128, 64);

  u8g2.sendBuffer();

  delay(100);
}