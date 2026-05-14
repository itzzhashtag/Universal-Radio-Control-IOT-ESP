#include "ReceiverModule.h"
 
int leftX, leftY, rightX, rightY, pot1, pot2;  // Joystick axes  -99 .. 99  //Potentiometers  0 .. 100
bool btnLA, btnRA, btnLB, btnRB, sw1, sw2;     // A-buttons (left/right)    // B-buttons (left/right)    // Toggle switches
int battery;                                   // Battery percent 0 .. 100
 
// ── Hexapod / servo / motor driver includes go here ─────────────
// #include <Adafruit_PWMServoDriver.h>
// etc.
void receivedata()
{
  if (receiver.available())
  {
    leftX   = receiver.data.Lx;
    leftY   = receiver.data.Ly;
    rightX  = receiver.data.Rx;
    rightY  = receiver.data.Ry;
    btnLA   = receiver.data.LABt;
    btnRA   = receiver.data.RABt;
    btnLB   = receiver.data.LBt;
    btnRB   = receiver.data.RBt;
    sw1     = receiver.data.TSW1;
    sw2     = receiver.data.TSW2;
    pot1    = receiver.data.Pot1;
    pot2    = receiver.data.Pot2;
    battery = receiver.data.BAT;

    printReceiverData();  // 👈 add this line

    if (!receiver.connected(1000)) 
    {
      Serial.println("[RX] WARNING: no packet for 1 second — TX disconnected?");
    }
  }

  delay(5);
}
void setup()
{
  Serial.begin(115200);
  // ── Open to all (matches TX BROADCAST mode): ─────────────────
  receiver.begin();
  // ── OR: accept only from specific TX (paste TX MAC below): ───
  // receiver.begin("A4:F0:0F:5F:E0:68");   // TX prints its MAC at boot

  // ── Your hardware init here ───────────────────────────────────
  // pwm.begin(); etc.

}

void loop()
{
  receivedata();
}
void printReceiverData()
{
  Serial.print("LX:"); Serial.print(leftX);
  Serial.print(" LY:"); Serial.print(leftY);
  Serial.print(" RX:"); Serial.print(rightX);
  Serial.print(" RY:"); Serial.print(rightY);

  Serial.print(" | LA:"); Serial.print(btnLA);
  Serial.print(" RA:"); Serial.print(btnRA);
  Serial.print(" LB:"); Serial.print(btnLB);
  Serial.print(" RB:"); Serial.print(btnRB);

  Serial.print(" | SW1:"); Serial.print(sw1);
  Serial.print(" SW2:"); Serial.print(sw2);

  Serial.print(" | P1:"); Serial.print(pot1);
  Serial.print(" P2:"); Serial.print(pot2);

  Serial.print(" | BAT:"); Serial.print(battery);

  Serial.println(); // end line
}