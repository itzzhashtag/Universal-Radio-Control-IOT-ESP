#include "Receiver.h"

Receiver rx;

// =====================================================
void setup()
{
  Serial.begin(115200);
  rx.begin();
}

// =====================================================
void loop()
{
  if (rx.newData)
  {
    rx.newData = false;

    // ===============================
    // PRINT EVERYTHING IN ONE LINE
    // ===============================
    Serial.printf(
      "L(%d,%d) R(%d,%d) | LB:%d LS:%d RS:%d RB:%d | SW1:%d SW2:%d | P1:%d\n",
      rx.data.LX, rx.data.LY,
      rx.data.RX, rx.data.RY,
      rx.data.LB, rx.data.LS,
      rx.data.RS, rx.data.RB,
      rx.data.SW1, rx.data.SW2,
      rx.data.PotM1
    );

    // ===============================
    // HERE YOU CONTROL MOTORS / SERVO
    // ===============================
    // Example:
    // if (rx.data.LY > 20) forward();
    // if (rx.data.LY < -20) backward();
  }
}