// ================================================================
//  USAGE EXAMPLE
//  Copy this block into a new .ino file.
//  Do NOT modify ReceiverModule.h directly for your project logic.
// ================================================================

#include "ReceiverModule.h"

// -- Include your robot / servo / motor libraries here ----------
// #include <Adafruit_PWMServoDriver.h>

void setup()
{
  Serial.begin(115200);

  // ── Option A: Accept packets from ANY transmitter (broadcast) ─
  receiver.begin("20:E7:C8:9F:47:F8", true);

  // ── Option B: Accept ONLY from one specific TX (unicast / safe)
  // Paste your TX MAC below — TX prints it on Serial at boot.
  // receiver.begin("20:E7:C8:9F:47:F8");

  // ── Option C: No ACK reply (lighter link, TX won't get feedback)
  // receiver.begin("", false);
  // receiver.begin("20:E7:C8:9F:47:F8", false);

  // ── Failsafe timeout (optional, default 1000 ms) ─────────────
  // receiver.timeoutMs = 500; // set before begin() if you want <1 s

  // -- Your hardware init goes here ----------------------------
  // pwm.begin(); etc.
}

void loop()
{
  // ── MUST call update() every loop for failsafe + status flag ──
  receiver.update();

  // ── Check for new data ────────────────────────────────────────
  if (receiver.available())
  {
    // ── All available fields from receiver.data: ──────────────

    int  leftX   = receiver.data.Lx;    // Left  joystick X : -99..99
    int  leftY   = receiver.data.Ly;    // Left  joystick Y : -99..99
    int  rightX  = receiver.data.Rx;    // Right joystick X : -99..99
    int  rightY  = receiver.data.Ry;    // Right joystick Y : -99..99
    bool btnLA   = receiver.data.LABt;  // Left  A-button
    bool btnRA   = receiver.data.RABt;  // Right A-button
    bool btnLB   = receiver.data.LBt;   // Left  B-button
    bool btnRB   = receiver.data.RBt;   // Right B-button
    bool sw1     = receiver.data.TSW1;  // Toggle switch 1
    bool sw2     = receiver.data.TSW2;  // Toggle switch 2
    int  pot1    = receiver.data.Pot1;  // Potentiometer 1  :  0..100
    //int  pot2    = receiver.data.Pot2;  // Potentiometer 2  :  0..100
    //int  battery = receiver.data.BAT;   // TX battery %     :  0..100

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
    // Data is already zeroed by update() — just add your safe-state
    // logic here (e.g. hold position, stop motors, raise flag):
    // Serial.println("TX disconnected — motors stopped");
  }

  // ── Read connection flag at any time ─────────────────────────
  // bool linkOK = receiver.connected_flag;

  // Non-blocking logic here — avoid long delay() calls
  delay(5);
}