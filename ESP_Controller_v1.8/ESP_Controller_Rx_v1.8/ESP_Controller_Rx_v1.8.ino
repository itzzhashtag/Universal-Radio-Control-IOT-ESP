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
 
    // ── Example: two-motor differential drive ─────────────────
    // int spd = map(leftY, -99, 99, -255, 255);
    // analogWrite(MOTOR_A_PIN, abs(spd));
    // digitalWrite(MOTOR_DIR_PIN, spd > 0 ? HIGH : LOW);

    // ── Example: arm servo from right Y ───────────────────────
    // int angle = map(rightY, -99, 99, 0, 180);
    // myServo.write(angle);
  }

  // ── Handle signal loss (after receiver.update() zeroes data) ──
  if (!receiver.connected())
  {
    //Serial.println("[RX] WARNING: no packet for 1 second — TX disconnected?");
    // Data is already zeroed by update() — just add your safe-state
    // logic here (e.g. hold position, stop motors, raise flag):
    // Serial.println("TX disconnected — motors stopped");
  }

  // ── Read connection flag at any time ─────────────────────────
  // bool linkOK = receiver.connected_flag;

  // Non-blocking logic here — avoid long delay() calls
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
  receiver.update();
  receivedata();
}
 