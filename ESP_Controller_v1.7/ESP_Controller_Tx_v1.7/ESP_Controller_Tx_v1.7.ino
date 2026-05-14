// ================================================================
//  ESP32 Universal RC Controller — TX v1.7
//
//  This file merges:
//    v0.7 hardware layout  (LED 13/12, buzzer, single Pot1, ACK-based link)
//    v1.0 display style    (U8g2 SPI ST7920 128×64 LCD, graphical widgets)
//
//  Role    : Transmitter / Master
//  Board   : ESP32 (38-pin devkit)
//  Protocol: ESP-NOW  (broadcast by default)
//
//  ── Hardware ────────────────────────────────────────────────────
//    2x Analog Joystick   LX=34  LY=35  / RX=32  RY=33
//    4x Push Buttons      LABt=26  RABt=27  LBt=25  RBt=14
//    2x Toggle Switch     TSW1=16  TSW2=17
//    1x Slider Pot        Pot1=39   (ADC1, input-only)
//    1x Battery Divider   BAT=36    (100kΩ + 47kΩ, input-only)
//    1x RED LED           GPIO 13   (100Ω → GND)
//    1x GREEN LED         GPIO 12   (100Ω → GND)
//    1x Passive Buzzer    GPIO 19   ← MOVED from 18 (18 = SPI LCD CLK)
//    1x JHD12864E LCD     SPI mode  (ST7920 128×64)
//
//  ── SPI LCD Wiring (JHD12864E / ST7920) ─────────────────────────
//  Pin  1 VSS  → GND
//  Pin  2 VDD  → 5V
//  Pin  3 V0   → Wiper of 10kΩ contrast pot  (Left leg→GND, Right→5V)
//  Pin  4 RS   → ESP32 GPIO  5   (CS / chip-select)
//  Pin  5 R/W  → ESP32 GPIO 23   (SID / SPI MOSI)  ← VSPI MOSI, fixed
//  Pin  6 E    → ESP32 GPIO 18   (CLK / SPI SCK)   ← VSPI SCK,  fixed
//  Pin 7-14 DB0-DB7 → NC  (unused in SPI mode)
//  Pin 15 PSB  → GND  ← MUST be LOW to select SPI mode
//  Pin 16 NC   → NC
//  Pin 17 RST  → ESP32 GPIO  4   (or tie HIGH to skip HW reset)
//  Pin 18 VOUT → NC  (optionally 0.1µF cap to GND for booster stability)
//  Pin 19 BLA  → 5V via 10Ω resistor  (backlight anode)
//  Pin 20 BLK  → GND  (backlight cathode)
//
//  ── Connection Detection ─────────────────────────────────────────
//  ACK-based (from v0.7):
//    Receiver sends a small AckData packet back to TX after each packet.
//    OnDataRecv() logs lastAckTime = millis() on each ACK.
//    updateConnectionLED() marks gConnected = true for ACK_TIMEOUT ms.
//    GREEN LED = link alive;  RED LED = no ACK in last 500 ms.
//
//  ── MAC Addresses ────────────────────────────────────────────────
//  TX sends to BROADCAST (FF:FF:FF:FF:FF:FF) by default.
//  Any receiver running ESP-NOW on the same channel will accept it.
//  To send to a specific robot only, set BROADCAST 0 and paste its MAC.
//  Robot's confirmed MAC: 20:E7:C8:9F:47:F8
//
//  ── Required Libraries ───────────────────────────────────────────
//  U8g2 by olikraus  (install via Library Manager → search "U8g2")
// ================================================================

#include <U8g2lib.h>
#include <esp_now.h>
#include <WiFi.h>

// ================================================================
//  PAYLOAD STRUCT  — must be IDENTICAL on TX and every receiver.
//  Field order matters: changing it breaks compatibility silently.
// ================================================================
typedef struct {
  int  Lx, Ly;       // Left  joystick X/Y  → -99 .. +99
  int  Rx, Ry;       // Right joystick X/Y  → -99 .. +99
  bool LABt, RABt;   // Joystick click  (left A / right A)
  bool LBt,  RBt;    // Shoulder buttons (left B / right B)
  bool TSW1, TSW2;   // Toggle switches
  int  Pot1, Pot2;         // Potentiometer    → 0 .. 100
  int  BAT;          // Battery %        → 0 .. 100  (informational for receiver)
} ControllerData;

ControllerData txData;   // live outgoing packet (zero-initialized at boot)

// ACK struct — receiver sends this back every time it gets a packet
typedef struct {
  bool alive;            // always set to true by receiver
} AckData;
static float batV     = 8.4f;
// ================================================================
//  MAC / PEER SELECTION
//  ─────────────────────────────────────────────────────────────────
//  BROADCAST 1 (default)
//    TX sends to FF:FF:FF:FF:FF:FF.
//    Any receiver on the same ESP-NOW channel picks it up.
//    Receiver must still know TX's MAC to send ACKs back —
//    it reads info->src_addr from the incoming packet and calls
//    esp_now_add_peer() + esp_now_send() dynamically.
//
//  BROADCAST 0 (unicast / targeted)
//    TX sends only to the MAC defined in ROBOT_MAC below.
//    Use this when you have one specific robot and want to
//    avoid other nearby receivers picking up your packets.
//    Paste the robot's MAC into ROBOT_MAC (lower-case OK).
// ================================================================
#define BROADCAST  1       // ← 0 = unicast, 1 = broadcast

// Robot's confirmed MAC address (from v0.7 working setup)
// Used directly when BROADCAST 0; kept as reference when BROADCAST 1
#define ROBOT_MAC_BYTES  0x20, 0xE7, 0xC8, 0x9F, 0x47, 0xF8

#if BROADCAST
  uint8_t peerMAC[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#else
  uint8_t peerMAC[] = { ROBOT_MAC_BYTES };   // ← unicast target
#endif

// ================================================================
//  PIN DEFINITIONS
//  ── Joystick axes  (ADC1, input-only on 34/35/39) ───────────────
// ================================================================
#define PIN_RX    33    // Right joystick X-axis
#define PIN_RY    34    // Right joystick Y-axis
#define PIN_LX    35    // Left joystick X-axis
#define PIN_LY    32    // Left joystick Y-axis

//  ── Digital inputs  (INPUT_PULLUP, button/switch active LOW) ───
#define PIN_LABt  26    // Left  joystick click (A-button left)
#define PIN_RABt  27    // Right joystick click (A-button right)
#define PIN_LBt   25    // Left  shoulder button
#define PIN_RBt   14    // Right shoulder button
#define PIN_TSW1  17    // Toggle switch 1
#define PIN_TSW2  16    // Toggle switch 2

//  ── Analog inputs ──────────────────────────────────────────────
#define PIN_POT1  39    // Potentiometer  (ADC1, input-only)
#define PIN_POT2  36
#define PIN_BAT   40    // Battery voltage divider (ADC1, input-only)
                        // Divider: VBAT ─100kΩ─ GPIO36 ─47kΩ─ GND

//  ── Output pins ────────────────────────────────────────────────
#define PIN_LED_R  13   // Red   LED  (HIGH = ON)  — from v0.7
#define PIN_LED_G  12   // Green LED  (HIGH = ON)  — from v0.7
#define PIN_BUZZ   19   // Passive buzzer           — from v0.7 (was GPIO 18)
                        //   !! GPIO 18 moved to LCD CLK in v1.x !!
                        //   Rewire buzzer from GPIO 18 → GPIO 19

//  ── SPI LCD pins ───────────────────────────────────────────────
#define PIN_LCD_CS   5  // ST7920 RS / CS
#define PIN_LCD_RST  4  // ST7920 RST  (or tie to VDD to skip HW reset)
//  LCD CLK  → GPIO 18  (VSPI SCK  — hardware-fixed, cannot remap)
//  LCD MOSI → GPIO 23  (VSPI MOSI — hardware-fixed, cannot remap)

// ================================================================
//  TUNING CONSTANTS
// ================================================================
#define DEADZONE         11   // Joystick dead-band in mapped units (-99..99)
                              // Values inside ±DEADZONE are reported as 0
#define CHANGE_THRESH     2   // Minimum axis delta to update txData (jitter filter)
#define POT_ZERO_THRESH  80   // raw ADC ≥ (4095-this) → pot considered at minimum
                              // Startup blocks until pot is physically at minimum

//  Battery voltage thresholds  (2S LiPo: 8.4 V full, 6.6 V empty)
#define BAT_FULL     8.4f     // V → mapped to 100%
#define BAT_EMPTY    6.6f     // V → mapped to   0%
#define BAT_LOW_V    6.8f     // V → triggers safe mode (radio off)
#define BAT_LOW_PCT  10       // % → startup gate blocks below this

//  Buzzer
#define BEEP_FREQ   1200      // Hz  (active buzzers ignore this — they self-oscillate)
#define BEEP_DUR      40      // ms  — short tap feel; tone() is non-blocking

//  Connection watchdog
#define ACK_TIMEOUT  500      // ms  — no ACK within this window → RED LED / disconnected

// ================================================================
//  U8g2  — ST7920 128×64, full-buffer, hardware SPI
//  U8G2_R0 = no rotation
//  VSPI bus (hardware): MOSI=GPIO23  SCK=GPIO18
//  CS and RST are software-selectable (defined above)
// ================================================================
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, PIN_LCD_CS, PIN_LCD_RST);

// ================================================================
//  GLOBALS
// ================================================================
int  gBatPct          = 100;    // Current battery %, updated by batteryManager()
bool gConnected       = false;  // true = ACK received within ACK_TIMEOUT ms

static unsigned long lastAckTime = 0;  // millis() when last ACK was received

// IIR-smoothed display values (prevents joystick/pot jitter on LCD)
static int sLx=0, sLy=0, sRx=0, sRy=0, sP1=50, sP2=50, sBat=100;

// Animation helpers
static uint8_t animTick = 0;    // 0..199 counter, wraps
static bool    batBlink = false; // 2 Hz blink flag for low-battery icon

// ================================================================
//  HELPERS
// ================================================================

// Map raw ADC 0-4095 → -99..+99 with centre dead-band
int mapAxis(int raw)
{
  int v = map(raw, 0, 4095, 99, -99);
  return (abs(v) < DEADZONE) ? 0 : v;
}

// Map raw ADC 0-4095 → 0..100  (max physical throw → 100)
int mapPot(int raw)
{
  return map(raw, 0, 4095, 0, 100);
}

// IIR low-pass smoother: steps 1/4 of remaining gap each call
int iirSmooth(int cur, int tgt)
{
  int d = tgt - cur;
  return (d == 0 || abs(d) <= 1) ? tgt : cur + d / 4;
}

// Read battery pack voltage and return integer % (0-100)
// vOut receives the computed pack voltage for threshold comparisons
int readBatPct(float &vOut)
{
  // Divider: VBAT ─100kΩ─ GPIO36 ─47kΩ─ GND
  // v_adc = vOut × 47/147  →  vOut = v_adc × 147/47
  float vADC = analogRead(PIN_BAT) * (3.3f / 4095.0f);
  vOut = vADC * (147.0f / 47.0f);
  if (vOut >= BAT_FULL)  return 100;
  if (vOut <= BAT_EMPTY) return 0;
  return (int)((vOut - BAT_EMPTY) * 100.0f / (BAT_FULL - BAT_EMPTY));
}

// ================================================================
//  BUZZER  — single beep, non-blocking (ESP32 timer-driven tone())
//  Call once per event; returns immediately.
// ================================================================
void beep()
{
  tone(PIN_BUZZ, BEEP_FREQ, BEEP_DUR);
}

// ================================================================
//  BEEP ON EDGE  — fires one beep per LOW→HIGH transition per input
//  Must be called AFTER txData has been freshly updated.
// ================================================================
void checkBeeps()
{
  // Shadow registers hold previous state for edge detection
  static bool pLABt=0, pRABt=0, pLBt=0, pRBt=0, pTSW1=0, pTSW2=0;

  if (txData.LABt && !pLABt) beep();   // joystick L click  → press edge
  if (txData.RABt && !pRABt) beep();   // joystick R click  → press edge
  if (txData.LBt  && !pLBt)  beep();   // shoulder L button → press edge
  if (txData.RBt  && !pRBt)  beep();   // shoulder R button → press edge
  if (txData.TSW1 && !pTSW1) beep();   // toggle SW1        → flipped ON
  if (txData.TSW2 && !pTSW2) beep();   // toggle SW2        → flipped ON

  pLABt = txData.LABt;  pRABt = txData.RABt;
  pLBt  = txData.LBt;   pRBt  = txData.RBt;
  pTSW1 = txData.TSW1;  pTSW2 = txData.TSW2;
}

// ================================================================
//  ESP-NOW CALLBACKS
// ================================================================

// OnDataSent — fires after every esp_now_send() attempt.
// In this version connection status is driven by ACK reception, NOT
// by send success, so we use this only for verbose serial debug.
// A successful send just means the packet left our antenna —
// it does NOT confirm the receiver got and processed it.
//
// NOTE: If you're on ESP32 Arduino Core < 3.x and get a compile
// error here, change the signature to:
//   void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
  // Uncomment for verbose send-side debug:
  // Serial.printf("[TX→SENT] %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
  (void)info; (void)status;   // suppress unused-variable warnings when commented out
}

// OnDataRecv — fires when receiver sends back an AckData heartbeat.
// Resets lastAckTime; updateConnectionLED() uses this to drive the LEDs.
//
// NOTE: Same Core version caveat as OnDataSent.
// For Core < 3.x change signature to:
//   void OnDataRecv(const uint8_t *mac_addr, const uint8_t *buf, int len)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *buf, int len)
{
  if (len == sizeof(AckData))
  {
    AckData ack;
    memcpy(&ack, buf, sizeof(ack));
    if (ack.alive)
    {
      lastAckTime = millis();   // watchdog reset
    }
  }
  (void)info;   // suppress unused-variable warning if src_addr not needed
}

// ================================================================
//  CONNECTION LED  — call every loop tick
//  Checks whether an ACK arrived within ACK_TIMEOUT ms.
//  GREEN LED = receiver alive;  RED LED = receiver missing.
// ================================================================
void updateConnectionLED()
{
  gConnected = (millis() - lastAckTime < ACK_TIMEOUT);
  digitalWrite(PIN_LED_G, gConnected ? HIGH : LOW);
  digitalWrite(PIN_LED_R, gConnected ? LOW  : HIGH);
}

// ================================================================
//  READ ALL INPUTS & SEND ESP-NOW PACKET
// ================================================================
void readAndSend()
{
  // ── Joystick axes: change-threshold prevents ADC noise spamming ──
  int nLx = mapAxis(analogRead(PIN_LX));
  int nLy = mapAxis(analogRead(PIN_LY));
  int nRx = mapAxis(analogRead(PIN_RX));
  int nRy = mapAxis(analogRead(PIN_RY));

  if (abs(nLx - txData.Lx) > CHANGE_THRESH) txData.Lx = nLx;
  if (abs(nLy - txData.Ly) > CHANGE_THRESH) txData.Ly = nLy;
  if (abs(nRx - txData.Rx) > CHANGE_THRESH) txData.Rx = nRx;
  if (abs(nRy - txData.Ry) > CHANGE_THRESH) txData.Ry = nRy;

  // ── Digital inputs: active LOW, inverted here to active HIGH ──
  txData.LABt = !digitalRead(PIN_LABt);
  txData.RABt = !digitalRead(PIN_RABt);
  txData.LBt  = !digitalRead(PIN_LBt);
  txData.RBt  = !digitalRead(PIN_RBt);
  txData.TSW1 = !digitalRead(PIN_TSW1);
  txData.TSW2 = !digitalRead(PIN_TSW2);

  // ── Potentiometer ────────────────────────────────────────────
  txData.Pot1 = mapPot(analogRead(PIN_POT1));
  txData.Pot2 = mapPot(analogRead(PIN_POT2));
  // ── Attach current battery % so receiver can display/log it ─
  txData.BAT = gBatPct;

  // ── Beep on press/toggle edges (must be after txData update) ─
  checkBeeps();

  // ── Fire ESP-NOW packet to peerMAC ──────────────────────────
  esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));

  // ── Serial debug line ────────────────────────────────────────
  Serial.printf("[TX→OUT] L(%+3d,%+3d) R(%+3d,%+3d) | LA:%d RA:%d LB:%d RB:%d | S1:%d S2:%d | P1:%3d P2:%3d | BAT: %.2fV %3d%% | Link: %s\n",
    txData.Lx,   txData.Ly, txData.Rx,   txData.Ry,
    txData.LABt, txData.RABt, txData.LBt,  txData.RBt,
    txData.TSW1, txData.TSW2, txData.Pot1, txData.Pot2,batV, txData.BAT, gConnected ? "Connected" : "Dead");
}

// ================================================================
//  BATTERY MANAGER
//  Reads voltage every 250 ms (non-blocking, static timer).
//  Returns true  → low-bat safe mode active; caller skips normal ops
//  Returns false → battery OK; proceed normally
// ================================================================
bool batteryManager()
{
  static unsigned long lastRead  = 0;
  static unsigned long lastBlink = 0;
  static bool  ledState = false;
  static bool  killSent = false;
  //static float batV     = 8.4f;   // optimistic default until first read
  
  // Read ADC every 250 ms
  if (millis() - lastRead > 250) {
    lastRead = millis();
    gBatPct  = readBatPct(batV);
    Serial.printf("[TX→BAT] %.2fV  %d%%\n", batV, gBatPct);
  }
  //batV = batV * 1.06;   // adjust this (start with 1.05–1.10)
  if (batV <= BAT_LOW_V)
  {
    // First time crossing threshold: zero all outputs, kill radio
    if (!killSent) {
      memset(&txData, 0, sizeof(txData));                            // zero packet
      esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));    // tell robot to stop
      Serial.printf("[TX→BAT] !! SAFE MODE !! %.2fV  %d%%  — radio KILLED\n", batV, gBatPct);
      killSent = true;
    }

    // Fast-blink red LED (300 ms period) while in safe mode
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState  = !ledState;
      digitalWrite(PIN_LED_R, ledState);
    }
    digitalWrite(PIN_LED_G, LOW);
    return true;   // caller should render LOW_BAT screen
  }

  // Voltage recovered
  killSent = false;
  return false;
}

// ================================================================
//  SYSTEM CONTROL  — call every loop tick
//  Wraps batteryManager() with blocking recovery loop and re-startup.
//  If battery dips low and then recovers, runs startup() again so
//  the operator must confirm safe state before normal ops resume.
// ================================================================
void systemControl()
{
  static bool wasLow = false;

  bool isLow = batteryManager();

  if (isLow) {
    wasLow = true;

    // Block here, rendering LOW_BAT screen, until voltage recovers
    while (batteryManager()) {
      u8g2.clearBuffer();
      animTick++;
      if (animTick >= 200) animTick = 0;
      batBlink = ((animTick % 10) < 5);
      drawLowBat();     // defined below
      u8g2.sendBuffer();
      delay(50);
    }
  }

  if (wasLow) {
    wasLow = false;
    // Reset smoothed values so display updates immediately on recovery
    sLx = sLy = sRx = sRy = sP1 = 0;
    startup();          // require operator to confirm safe state again
  }
}

// ================================================================
//  LCD DRAW HELPERS
// ================================================================

// Square joystick widget with dashed crosshair + 3×3 moving dot
// cx,cy = centre pixel   half = box half-size   lx,ly = -99..99
void drawJoystick(uint8_t cx, uint8_t cy, uint8_t half, int lx, int ly)
{
  uint8_t x0 = cx - half, y0 = cy - half, sz = half * 2 + 1;
  u8g2.drawFrame(x0, y0, sz, sz);                                     // outer box
  for (uint8_t i = x0+2; i < x0+sz-2; i+=2) u8g2.drawPixel(i, cy);  // H dashed crosshair
  for (uint8_t i = y0+2; i < y0+sz-2; i+=2) u8g2.drawPixel(cx, i);  // V dashed crosshair
  // Small tick marks at corners
  u8g2.drawPixel(x0+1, y0+1);     u8g2.drawPixel(x0+sz-2, y0+1);
  u8g2.drawPixel(x0+1, y0+sz-2); u8g2.drawPixel(x0+sz-2, y0+sz-2);
  // Moving 3×3 dot: lx=+99→right, ly=+99→up (Y inverted on pixel coords)
  uint8_t inn = half - 3;
  int8_t  dx  = (int8_t)((lx * (int8_t)inn) / 99);
  int8_t  dy  = (int8_t)((ly * (int8_t)inn) / 99);
  u8g2.drawBox(cx + dx - 1, cy - dy - 1, 3, 3);
}

// Vertical fill bar rising from bottom (pct 100 = full bar)
// Used to display Pot1 value
void drawVSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct)
{
  u8g2.drawFrame(x, y, w, h);
  uint8_t fh = (uint16_t)pct * (h - 2) / 100;
  if (fh) u8g2.drawBox(x + 1, y + h - 1 - fh, w - 2, fh);
}

// Signal strength bars  — 3 bars, bottom-aligned, tallest blinks when linked
void drawSignal(uint8_t cx, uint8_t bot, bool connected)
{
  const uint8_t BW=4, GAP=3;
  const uint8_t x0 = cx - 9;             // leftmost bar X
  const uint8_t HTS[3] = {6, 11, 16};    // bar heights: short → tall
  bool blink = (animTick / 5) % 2;       // ~3 Hz blink on top bar

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t bx = x0 + i * (BW + GAP);
    uint8_t bh = HTS[i];
    uint8_t by = bot - bh;
    bool filled = connected && (i < 2 || blink);   // top bar blinks, lower solid
    if (filled) u8g2.drawBox(bx, by, BW, bh);
    else        u8g2.drawFrame(bx, by, BW, bh);
  }
}

// Button widget — inverted (white label on black box) when active
void drawBtn(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
             const char *label, bool active)
{
  if (active) {
    u8g2.drawBox(x, y, w, h);            // filled rectangle
    u8g2.setDrawColor(0);                 // draw white
    u8g2.drawStr(x + 1, y + h - 1, label);
    u8g2.setDrawColor(1);                 // restore black
  } else {
    u8g2.drawFrame(x, y, w, h);          // outline only
    u8g2.drawStr(x + 1, y + h - 1, label);
  }
}

// Vertical mobile-style battery icon: nub on top, fill rises from bottom
void drawVBattery(uint8_t bx, uint8_t by, uint8_t bw, uint8_t bh,
                  uint8_t pct, bool blink)
{
  uint8_t nw = bw / 2, nx = bx + (bw - nw) / 2;
  u8g2.drawBox(nx, by - 2, nw, 2);           // positive nub above body
  u8g2.drawFrame(bx, by, bw, bh);            // body outline
  if (!blink) {                               // when blinking: show hollow
    uint8_t fh = (uint16_t)pct * (bh - 2) / 100;
    if (fh) u8g2.drawBox(bx + 1, by + bh - 1 - fh, bw - 2, fh);
  }
}

// ================================================================
//  SCREEN: NORMAL OPERATION
//
//  128×64 pixel layout:
//  ┌──────────────────────────────────────────────────────┐ y 0-28
//  │  [L stick 25×25]  [P1 ▲]  [▪▪▪ signal ▪▪▪]  [R stick 25×25]  │
//  ├──────────────────────────────────────────────────────┤ y 29
//  │  [LA] [LB] [S1]          (gap)          [S2] [RA] [RB]  │ y 30-44
//  ├──────────────────────────────────────────────────────┤ y 45
//  │  ┌── HASHTAG ──┐                   [▓bat▓]  82%    │ y 46-63
//  │  │  URC v1.1   │                                    │
//  │  └─────────────┘                                    │
//  └──────────────────────────────────────────────────────┘
// ================================================================

// ================================================================
//  SCREEN: NORMAL OPERATION
//
//  ┌───────────────────────────────────────────────────────────────┐
//  │ [L stick] [P1▲] [▪▪▪ signal ▪▪▪] [P2▲] [R stick]          │ y 0-28
//  ├───────────────────────────────────────────────────────────────┤ y 29
//  │ [LA][LB][S1]              (gap)        [S2][RA][RB]          │ y 30-44
//  ├───────────────────────────────────────────────────────────────┤ y 45
//  │ ┌──────────────────────┐                   [▓bat▓]  82%      │ y 46-63
//  │ │ HASHTAG              │                                       │
//  │ │ URC v1.0             │                                       │
//  │ └──────────────────────┘                                       │
//  └───────────────────────────────────────────────────────────────┘
// ================================================================
void drawNormal()
{
  u8g2.setFont(u8g2_font_4x6_tf);

  // ── TOP ROW ──────────────────────────────────────────────────

  // Left joystick: centre (13,14), half=12 → box x=1..25 y=2..26
  drawJoystick(13, 14, 12, sLx, sLy);

  // Pot1 vertical slider: x=28, just right of L stick
  // Width=5, height=25; value rises from bottom
  drawVSlider( 27,   2,  5, 25, (uint8_t)sP1);

  // Signal bars: centred at x=64, bottom at y=25
  drawSignal(64, 25, gConnected);

  // Pot2 vertical slide : x=96, left of R stick
  drawVSlider( 96,   2,  5, 25, (uint8_t)sP2);  
  // Right joystick: centre (114,14), half=12 → box x=102..126 y=2..26
  drawJoystick(114, 14, 12, sRx, sRy);

  u8g2.drawHLine(0, 29, 128);   // ── divider ──

  // ── MID ROW: 6 buttons, symmetric about x=63 ─────────────────
  // Each button: 10px wide, 8px tall, label 4×6 font
  drawBtn(  2, 31, 10, 8, "LA", txData.LABt);   // left  joystick click
  drawBtn( 14, 31, 10, 8, "LB", txData.LBt);    // left  shoulder btn
  drawBtn( 26, 31, 10, 8, "S1", txData.TSW1);   // toggle switch 1
  drawBtn( 90, 31, 10, 8, "S2", txData.TSW2);   // toggle switch 2
  drawBtn(102, 31, 10, 8, "RA", txData.RABt);   // right joystick click
  drawBtn(114, 31, 10, 8, "RB", txData.RBt);    // right shoulder btn

  u8g2.drawHLine(0, 45, 128);   // ── divider ──

  // ── BOTTOM ROW: name plate left, battery icon+% right ─────────
  u8g2.drawFrame(0, 46, 58, 17);    // name box outline
  u8g2.drawStr( 3, 54, "HASHTAG");
  u8g2.drawStr( 3, 61, "URC v1.7");

  // Battery blinks when < 20% (2 Hz)
  bool batLow = (sBat < 20) && batBlink;
  drawVBattery(96, 49, 8, 11, (uint8_t)sBat, batLow);

  // Right-align % text: single digit gets two leading spaces
  char buf[5];
  sprintf(buf, sBat < 10 ? "  %d%%" : sBat < 100 ? " %d%%" : "%d%%", sBat);
  u8g2.drawStr(106, 61, buf);
}

// ================================================================
//  SCREEN: STARTUP CAUTION GATE
//  sw1OK/sw2OK — toggles OFF (HIGH on pullup)
//  p1OK        — pot at minimum position
//  batOK       — battery ≥ 10%
// ================================================================
void drawCaution(bool sw1OK, bool sw2OK, bool p1OK, bool batOK)
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(16, 10, "!! CAUTION !!");
  u8g2.drawHLine(0, 12, 128);

  u8g2.setFont(u8g2_font_4x6_tf);

  // Left column: SW1, P1
  u8g2.drawStr( 2, 24, "SW1:"); u8g2.drawStr(22, 24, sw1OK ? " OK " : "OFF!");
  u8g2.drawStr( 2, 34, "P1 :"); u8g2.drawStr(22, 34, p1OK  ? " OK " : "MIN!");

  // Right column: SW2
  u8g2.drawStr(66, 24, "SW2:"); u8g2.drawStr(86, 24, sw2OK ? " OK " : "OFF!");

  // Battery row spans full width
  u8g2.drawStr( 2, 44, "BAT:");
  if (batOK) {
    char b[6]; sprintf(b, " %3d%%", gBatPct);
    u8g2.drawStr(22, 44, b);
  } else {
    u8g2.drawStr(22, 44, " LOW!");
  }

  u8g2.drawHLine(0, 47, 128);

  // Animated waiting dots cycling 0→3 at ~2 Hz
  char dots[14] = "Waiting ";
  uint8_t nd = (animTick / 8) % 4;
  for (uint8_t i = 0; i < nd; i++) dots[8 + i] = '.';
  dots[8 + nd] = '\0';
  u8g2.drawStr(10, 58, dots);
}

// ================================================================
//  SCREEN: LOW BATTERY WARNING
//  Shown while batteryManager() returns true (safe mode active).
// ================================================================
void drawLowBat()
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 14, "LOW BATTERY!");

  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(24, 26, "Radio  OFF");

  // Large horizontal battery body  (96px wide × 20px tall)
  u8g2.drawFrame(10, 32, 96, 20);
  u8g2.drawBox(106, 38,  5,  8);   // positive nub (right side)

  // Fill blinks in sync with batBlink
  if (batBlink) {
    uint8_t fill = (uint16_t)gBatPct * 94 / 100;
    if (fill) u8g2.drawBox(11, 33, fill, 18);
  }
  char buf[5]; sprintf(buf, "%d%%", gBatPct);
  u8g2.drawStr(50, 45, buf);
}

// ================================================================
//  STARTUP SAFETY GATE
//  Blocks until ALL conditions are met simultaneously:
//    TSW1 = OFF  (digitalRead HIGH → switch open → pullup HIGH)
//    TSW2 = OFF
//    Pot1 at physical minimum  (raw ADC ≥ 4095 - POT_ZERO_THRESH)
//    Battery ≥ BAT_LOW_PCT (10%)
//
//  This prevents the robot from suddenly moving when powered on
//  with sticks/pot in unknown positions.
// ================================================================
void startup()
{
  Serial.println("[TX→BOOT] Startup safety check:");
  Serial.println("[TX→BOOT]   TSW1 and TSW2 must be OFF (switch open)");
  Serial.println("[TX→BOOT]   Pot1 must be at minimum (raw ADC near 4095)");
  Serial.println("[TX→BOOT]   Battery must be >= 10%");

  while (true)
  {
    float batV;
    gBatPct = readBatPct(batV);

    bool sw1OK = (bool)digitalRead(PIN_TSW1);                         // HIGH = OFF = safe
    bool sw2OK = (bool)digitalRead(PIN_TSW2);
    bool p1OK  = (analogRead(PIN_POT1) >= (4095 - POT_ZERO_THRESH)); // slider at minimum
    bool batOK = (gBatPct >= BAT_LOW_PCT);

    Serial.printf("[TX→CAUTION] SW1:%s SW2:%s P1:%s BAT:%s(%d%%)\n",
      sw1OK?"OK":"WAIT", sw2OK?"OK":"WAIT",
      p1OK ?"OK":"WAIT", batOK?"OK":"LOW", gBatPct);

    if (sw1OK && sw2OK && p1OK && batOK)
    {
      // All conditions satisfied — show brief ALL CLEAR screen
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(4,  22, "** ALL CLEAR **");
      u8g2.setFont(u8g2_font_4x6_tf);
      u8g2.drawStr(22, 38, "System Ready");
      u8g2.drawStr(18, 50, "Starting up ...");
      u8g2.sendBuffer();
      Serial.println("[TX→BOOT] All clear — proceeding");

      // Boot beep: ascending two-tone
      tone(PIN_BUZZ, 1000, 80); delay(130);
      tone(PIN_BUZZ, 1400, 80);
      delay(900);   // let "ALL CLEAR" screen linger
      return;
    }

    // Not yet safe — render caution screen with live status
    animTick++;
    u8g2.clearBuffer();
    drawCaution(sw1OK, sw2OK, p1OK, batOK);
    u8g2.sendBuffer();
    delay(150);
  }
}

// ================================================================
//  UPDATE DISPLAY  — called every loop tick during normal ops
// ================================================================
void updateDisplay()
{
  // Apply IIR smoothing to all display values
  // This filters joystick jitter and gives the LCD a fluid feel
  sLx  = iirSmooth(sLx,  txData.Lx);
  sLy  = iirSmooth(sLy,  txData.Ly);
  sRx  = iirSmooth(sRx,  txData.Rx);
  sRy  = iirSmooth(sRy,  txData.Ry);
  sP1  = iirSmooth(sP1,  txData.Pot1);
  sP2  = iirSmooth(sP2,  txData.Pot2);
  sBat = iirSmooth(sBat, gBatPct);

  // Advance animation tick (0-199, wraps)
  animTick++;
  if (animTick >= 200) animTick = 0;
  batBlink = ((animTick % 10) < 5);   // 2 Hz blink for low-battery icon

  u8g2.clearBuffer();
  drawNormal();
  u8g2.sendBuffer();
}

// ================================================================
//  SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("\n================================================");
  Serial.println(" ESP32 Universal RC Controller — TX v1.7");
  Serial.println(" Hardware: v0.7   Display: v1.0 U8g2");
  Serial.println("================================================");

  // ── Output pins ──────────────────────────────────────────────
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_BUZZ,  OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);   // red LED on during boot
  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_BUZZ,  LOW);

  // ── Digital input pins (active LOW with internal pullup) ─────
  pinMode(PIN_LABt, INPUT_PULLUP);
  pinMode(PIN_RABt, INPUT_PULLUP);
  pinMode(PIN_LBt,  INPUT_PULLUP);
  pinMode(PIN_RBt,  INPUT_PULLUP);
  pinMode(PIN_TSW1, INPUT_PULLUP);
  pinMode(PIN_TSW2, INPUT_PULLUP);
  // Analog pins (34,35,32,33,39,36) default to input — no pinMode needed

  // ── ADC: 11dB attenuation → full 0-3.3V range on ADC1 ───────
  // Without this, ADC only reads up to ~1.1V
  analogSetAttenuation(ADC_11db);
  Serial.println("[TX→BOOT] GPIO and ADC configured");

  // ── LCD init ─────────────────────────────────────────────────
  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  Serial.println("[TX→BOOT] LCD ST7920 128×64 init OK");

  // ── Startup safety gate (blocks until controls are safe) ─────
  //startup();

  // ── WiFi mode ────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  Serial.printf("[TX→BOOT] TX MAC address: %s\n", WiFi.macAddress().c_str());
  Serial.println("[TX→BOOT]   ↑ Share this with the receiver firmware so it can send ACKs back");
  delay(100);

  // ── ESP-NOW init ─────────────────────────────────────────────
  if (esp_now_init() != ESP_OK) {
    Serial.println("[TX→ERROR] ESP-NOW init FAILED — halting");
    // Blink red indefinitely to signal failure
    while (1) { digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R)); delay(200); }
  }

  // Register both callbacks:
  //   OnDataSent → fires after each send (debug only in this version)
  //   OnDataRecv → fires when receiver sends back AckData heartbeat
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add the peer (broadcast or unicast, controlled by BROADCAST define)
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;       // 0 = follow current WiFi channel automatically
  peer.encrypt = false;   // no AES encryption (add key here if needed)
  esp_now_add_peer(&peer);

  Serial.printf("[TX→BOOT] ESP-NOW ready  mode:%s  peer:%02X:%02X:%02X:%02X:%02X:%02X\n",
    BROADCAST ? "BROADCAST" : "UNICAST",
    peerMAC[0], peerMAC[1], peerMAC[2],
    peerMAC[3], peerMAC[4], peerMAC[5]);

  digitalWrite(PIN_LED_R, LOW);   // turn off red now that boot succeeded
  Serial.println("[TX→BOOT] Boot complete — running at ~20 Hz\n");
}

// ================================================================
//  LOOP  — ~20 Hz  (50 ms per tick)
// ================================================================
void loop()
{
  //systemControl();        // 1. battery check; blocks + renders LOW_BAT if needed
  //batteryManager(); 
  readAndSend();          // 2. sample all inputs → send ESP-NOW packet + beep on edges
  updateDisplay();        // 3. IIR smooth → render LCD frame
  updateConnectionLED();  // 4. ACK watchdog → update GREEN/RED LED
  delay(50);             //    ~20 Hz
}
