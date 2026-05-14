// ================================================================
//  ESP-NOW Receiver — Universal Test Template  v1.0
//  Role  : Receiver / Slave
//  Board : Any ESP32 (devkit, custom, hexapod main board, etc.)
//
//  ── HOW TO USE THIS FILE ─────────────────────────────────────────
//  This is a drop-in receiver template.  To adapt it to any robot:
//
//    1. Paste this file into your project (or add as a tab).
//    2. In your setup() call:   RC_Setup();
//    3. In your loop() call:    RC_Update();
//    4. After RC_Update(), read the global `rc` struct for live data.
//       All fields are described below.
//
//  The RC_Handler class wraps all ESP-NOW boilerplate so your main
//  code only sees clean rc.LX, rc.P1, rc.SW1, etc.
//
//  ── WIRING ───────────────────────────────────────────────────────
//  No extra wiring needed — this is pure Wi-Fi (ESP-NOW).
//  Common GND with transmitter is NOT required for ESP-NOW.
//
//  ── GETTING YOUR MAC ADDRESS ─────────────────────────────────────
//  Flash this sketch once with DEBUG defined and check Serial at
//  115200 baud.  The [MAC] line prints this board's MAC address.
//  Copy that address into the transmitter's receiverMAC[] array.
//
//  ── DATA FIELDS ──────────────────────────────────────────────────
//  rc.LX, rc.LY       Left stick X / Y       -99 .. +99
//  rc.RX, rc.RY       Right stick X / Y      -99 .. +99
//  rc.P1, rc.P2       Slider pots             0 .. 100
//  rc.LS, rc.RS       Joystick click buttons  true/false
//  rc.LB, rc.RB       Shoulder buttons        true/false
//  rc.SW1, rc.SW2     Toggle switches         true/false
//  rc.fresh           true if a new packet arrived since last RC_Update()
//  rc.lastMs          millis() of last received packet
//  rc.rssi            RSSI of last received packet (dBm, negative)
//  rc.packetCount     total packets received since boot
//  rc.lostCount       total packets lost (connection timeout events)
//
//  ── CONNECTION WATCHDOG ──────────────────────────────────────────
//  If no packet arrives for CONN_TIMEOUT_MS milliseconds,
//  RC_Update() sets all axes to 0, all buttons to false, and
//  increments rc.lostCount.  You can check RC_IsConnected() for
//  a clean boolean.
// ================================================================

#include <esp_now.h>
#include <WiFi.h>

// ── DEBUG SWITCH ─────────────────────────────────────────────────
// Comment out to silence all Serial output
#define DEBUG

#ifdef DEBUG
  #define DBG(...)   Serial.print(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)
  #define DBGF(...)  Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
  #define DBGF(...)
#endif

// ── TUNING ───────────────────────────────────────────────────────
#define CONN_TIMEOUT_MS  600   // ms without a packet = connection lost
#define PRINT_INTERVAL   500   // ms between debug prints in loop

// ================================================================
//  CONTROLLER DATA STRUCT
//  Must exactly match the transmitter's ControllerData typedef.
// ================================================================
typedef struct {
  int  LX, LY;    // Left joystick  X / Y   (-99 .. +99, 0 = center)
  int  RX, RY;    // Right joystick X / Y   (-99 .. +99, 0 = center)
  bool LS, RS;    // Joystick click (left stick press / right stick press)
  bool LB, RB;    // Shoulder buttons (left bumper / right bumper)
  bool SW1, SW2;  // Toggle switches
  int  P1, P2;    // Slider potentiometers  (0 = minimum, 100 = maximum)
} ControllerData;

// ================================================================
//  RC STATE STRUCT
//  Extends ControllerData with receiver-side metadata.
//  Access all fields via the global `rc` variable.
// ================================================================
struct RCState {
  // ── Joystick axes ─────────────────────────────────────────────
  int  LX = 0, LY = 0;   // Left stick  X / Y
  int  RX = 0, RY = 0;   // Right stick X / Y

  // ── Buttons / switches ────────────────────────────────────────
  bool LS  = false;   // Left  stick click
  bool RS  = false;   // Right stick click
  bool LB  = false;   // Left  shoulder button
  bool RB  = false;   // Right shoulder button
  bool SW1 = false;   // Toggle switch 1
  bool SW2 = false;   // Toggle switch 2

  // ── Sliders ───────────────────────────────────────────────────
  int P1 = 0;   // Slider pot 1  (0..100)
  int P2 = 0;   // Slider pot 2  (0..100)

  // ── Metadata (set by RC_Update) ───────────────────────────────
  bool     fresh       = false;  // true for one RC_Update() cycle after new data
  uint32_t lastMs      = 0;      // millis() of last received packet
  int      rssi        = 0;      // RSSI in dBm (0 = never received)
  uint32_t packetCount = 0;      // total valid packets received
  uint32_t lostCount   = 0;      // number of connection-lost events

  // ── Connection state ──────────────────────────────────────────
  bool connected = false;        // updated each RC_Update() call
};

// ── GLOBAL ACCESSIBLE FROM ANYWHERE IN YOUR SKETCH ───────────────
RCState rc;

// ── INTERNAL: raw buffer filled by ESP-NOW callback ──────────────
static ControllerData _rawData;
static volatile bool  _newDataFlag = false;
static volatile int   _lastRssi    = 0;

// ================================================================
//  ESP-NOW RECEIVE CALLBACK
//  Called from a different task context — keep short, no Serial here.
//  Sets a flag; RC_Update() in loop() does the actual work.
// ================================================================
void _onDataRecv(const esp_now_recv_info_t *info,
                 const uint8_t *incomingData, int len)
{
  if (len != sizeof(ControllerData)) return;  // wrong size = ignore
  memcpy(&_rawData, incomingData, sizeof(ControllerData));
  _lastRssi    = info->rx_ctrl->rssi;
  _newDataFlag = true;
}

// ================================================================
//  RC_Handler — bundles all ESP-NOW setup and update logic.
//  You don't call methods on this class directly;
//  use the free functions RC_Setup(), RC_Update(), RC_IsConnected().
// ================================================================
class RC_Handler
{
public:

  // ── RC_Setup ─────────────────────────────────────────────────
  // Call once in your sketch's setup().
  // Initialises WiFi (STA mode) and registers the ESP-NOW callback.
  void begin()
  {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    #ifdef DEBUG
      // Print this board's own MAC so you can copy it to the transmitter
      DBGF("[MAC] This receiver MAC: %s\n",
           WiFi.macAddress().c_str());
      DBGLN("[RC] Initialising ESP-NOW...");
    #endif

    if (esp_now_init() != ESP_OK) {
      DBGLN("[RC] ERROR: esp_now_init() failed!");
      return;
    }
    esp_now_register_recv_cb(_onDataRecv);
    DBGLN("[RC] ESP-NOW ready — waiting for transmitter.");
  }

  // ── RC_Update ────────────────────────────────────────────────
  // Call every iteration of your loop().
  // Copies raw data into rc struct, applies watchdog, updates metadata.
  void update()
  {
    rc.fresh = false;   // clear the "new data" flag from last cycle

    if (_newDataFlag)
    {
      _newDataFlag = false;  // clear flag atomically (single-core safe here)

      // Copy fields from raw struct into the clean rc struct
      rc.LX  = _rawData.LX;   rc.LY  = _rawData.LY;
      rc.RX  = _rawData.RX;   rc.RY  = _rawData.RY;
      rc.LS  = _rawData.LS;   rc.RS  = _rawData.RS;
      rc.LB  = _rawData.LB;   rc.RB  = _rawData.RB;
      rc.SW1 = _rawData.SW1;  rc.SW2 = _rawData.SW2;
      rc.P1  = _rawData.P1;   rc.P2  = _rawData.P2;

      rc.lastMs = millis();
      rc.rssi   = _lastRssi;
      rc.packetCount++;
      rc.fresh  = true;   // signal that fresh data is available this cycle

      #ifdef DEBUG
        _printPacket();
      #endif
    }

    // ── Connection watchdog ──────────────────────────────────────
    bool nowConnected = (millis() - rc.lastMs < CONN_TIMEOUT_MS);

    if (!nowConnected && rc.connected) {
      // Transition: connected → lost
      rc.lostCount++;
      DBGF("[RC] CONNECTION LOST  (lost#%lu, last pkt %lums ago)\n",
           rc.lostCount, millis() - rc.lastMs);
      _safeZero();
    }
    if (nowConnected && !rc.connected) {
      DBGLN("[RC] CONNECTION RESTORED");
    }
    rc.connected = nowConnected;
  }

  // ── RC_IsConnected ───────────────────────────────────────────
  // Returns true if a packet arrived within CONN_TIMEOUT_MS.
  bool isConnected() { return rc.connected; }

  private:

  // Zero all control values (called on connection loss)
  void _safeZero()
  {
    rc.LX = rc.LY = rc.RX = rc.RY = 0;
    rc.P1 = rc.P2 = 0;
    rc.LS = rc.RS = rc.LB = rc.RB = false;
    rc.SW1 = rc.SW2 = false;
  }

  #ifdef DEBUG
  // Throttled print — one line per PRINT_INTERVAL ms
  void _printPacket()
  {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < PRINT_INTERVAL) return;
    lastPrint = millis();

    DBGF("[RC#%lu] L(%3d,%3d) R(%3d,%3d) | LB:%d LS:%d RS:%d RB:%d"
         " | SW1:%d SW2:%d | P1:%3d P2:%3d | RSSI:%ddBm\n",
         rc.packetCount,
         rc.LX, rc.LY, rc.RX, rc.RY,
         rc.LB, rc.LS, rc.RS, rc.RB,
         rc.SW1, rc.SW2,
         rc.P1, rc.P2,
         rc.rssi);
  }
  #endif
};

// ── Singleton instance ───────────────────────────────────────────
RC_Handler _rcHandler;

// ── Free function wrappers (call these from your sketch) ─────────
void     RC_Setup()        { _rcHandler.begin(); }
void     RC_Update()       { _rcHandler.update(); }
bool     RC_IsConnected()  { return _rcHandler.isConnected(); }

// ================================================================
// ================================================================
//
//  EXAMPLE ROBOT USAGE BELOW
//  Replace with your own motor / servo / actuator code.
//  Everything between the dashed lines is example code.
//
// ================================================================
// ================================================================

// ── Example robot pins (replace with your own) ───────────────────
// #define MOTOR_L_PIN  25
// #define MOTOR_R_PIN  26
// etc.

// ── Example: convert joystick to motor speed ─────────────────────
//  Returns a value in -255..255 ready for analogWrite / PWM
int joystickToMotor(int axisVal)
{
  // rc axes are -99..+99, map to -255..+255 for PWM
  return map(axisVal, -99, 99, -255, 255);
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  #ifdef DEBUG
    Serial.begin(115200);
    delay(200);
    DBGLN("=== ESP-NOW Receiver — Universal Test Template ===");
  #endif

  // ── Initialise ESP-NOW receiver ──────────────────────────────
  RC_Setup();

  // ── Your hardware init goes here ─────────────────────────────
  // pinMode(MOTOR_L_PIN, OUTPUT);
  // ledcSetup(0, 5000, 8);
  // etc.

  DBGLN("[SETUP] Ready.");
}

// ================================================================
//  LOOP
// ================================================================
void loop()
{
  // ── 1. Always call RC_Update() first ─────────────────────────
  //      This processes incoming packets and updates rc.* fields.
  RC_Update();

  // ── 2. Check connection before using data ────────────────────
  if (!RC_IsConnected()) {
    // Robot is not receiving data — apply failsafe behaviour here.
    // rc.* fields are already zeroed by the watchdog inside RC_Update().
    DBGLN("[LOOP] Failsafe: no signal");

    // Example failsafe: stop all motors
    // analogWrite(MOTOR_L_PIN, 0);
    // analogWrite(MOTOR_R_PIN, 0);
    return;
  }

  // ── 3. Use rc.* fields to drive your robot ───────────────────
  //      rc.fresh == true means a new packet arrived this cycle.
  //      You can check rc.fresh if you only want to act on new data.

  // ── Joystick examples ────────────────────────────────────────
  int leftPower  = joystickToMotor(rc.LY);   // forward/back via left Y
  int rightPower = joystickToMotor(rc.RY);   // turn via right Y
  int steer      = joystickToMotor(rc.LX);   // strafe via left X

  // Example: differential drive
  // int leftMotor  = constrain(leftPower + steer, -255, 255);
  // int rightMotor = constrain(leftPower - steer, -255, 255);
  // driveMotors(leftMotor, rightMotor);

  // ── Button / switch examples ──────────────────────────────────
  if (rc.SW1) {
    // SW1 toggled ON — e.g. enable arm, activate a mode, etc.
    // armEnabled = true;
  }

  if (rc.LB) {
    // Left bumper held — e.g. speed boost, special move, claw grip
  }

  if (rc.LS) {
    // Left stick click — e.g. horn, light toggle
  }

  // ── Slider pot examples ───────────────────────────────────────
  // rc.P1  0..100 — e.g. map to servo angle: map(rc.P1, 0,100, 0,180)
  // rc.P2  0..100 — e.g. speed limit: maxSpeed = map(rc.P2, 0,100, 50,255)

  // int servoAngle = map(rc.P1, 0, 100, 0, 180);
  // myServo.write(servoAngle);

  // ── Hexapod-specific example (URC hexapod) ───────────────────
  // rc.LX  = lateral strafe  (-99..+99)
  // rc.LY  = forward / back  (-99..+99)
  // rc.RX  = body yaw        (-99..+99)
  // rc.RY  = body pitch      (-99..+99, if used)
  // rc.P1  = gait speed      (0..100)
  // rc.P2  = body height     (0..100)
  // rc.SW1 = standing mode ON/OFF toggle
  // rc.SW2 = wave/pose mode  toggle
  // rc.LB  = sit down
  // rc.RB  = stand up

  // Example gait call (pseudocode):
  // hexapod.setGait(rc.LX, rc.LY, rc.RX, rc.P1, rc.P2);

  // ── RC-car / rover example ───────────────────────────────────
  // throttle = rc.LY           (forward = +99)
  // steering = rc.RX           (right   = +99)
  // int thr = map(rc.LY, -99, 99, -255, 255);
  // int str = map(rc.RX, -99, 99, -255, 255);

  // ── Drone / flight controller example ────────────────────────
  // throttle = rc.LY           (up   = +99)
  // yaw      = rc.LX           (right = +99)
  // pitch    = rc.RY           (fwd  = +99)
  // roll     = rc.RX           (right = +99)

  // ── Full periodic status dump (DEBUG only) ───────────────────
  #ifdef DEBUG
  {
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus > 2000) {
      lastStatus = millis();
      DBGF("[STATUS] pkts=%lu  lost=%lu  RSSI=%ddBm  connected=%d\n",
           rc.packetCount, rc.lostCount, rc.rssi, rc.connected);
    }
  }
  #endif
}
