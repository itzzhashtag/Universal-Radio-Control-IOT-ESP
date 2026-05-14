// ================================================================
//  ESP32 Universal RC Controller — TX v1.1
//  Role    : Transmitter / Master
//  Board   : ESP32 (38-pin devkit, e.g. AZ-Delivery ESP32 DevKitC)
//
//  ── PROTOCOL ────────────────────────────────────────────────────
//
//  TX  →  broadcasts ControllerData to FF:FF:FF:FF:FF:FF
//          (every RX on the same WiFi channel picks it up)
//
//  RX  →  filters incoming packets by TX MAC (only accepts from us)
//          responds immediately with AckData to FF:FF:FF:FF:FF:FF
//          (broadcast ACK so TX doesn't need to register each RX)
//
//  TX  →  listens for AckData; sets gConnected = true if an ACK
//          arrived within the last CONN_TIMEOUT_MS milliseconds.
//          GREEN LED = connected, RED LED = no ACK (link lost).
//
//  WHY BROADCAST FOR ACK?
//    If RX sent ACK directly to TX MAC, TX would need to register
//    the RX as a peer, requiring TX to know every receiver's MAC.
//    By broadcasting the ACK, any TX on the channel hears it and
//    no pre-registration is needed on either side.
//
//  WHY NOT USE OnDataSent FOR CONNECTION STATUS?
//    OnDataSent fires for every outgoing packet regardless of
//    whether any receiver is alive — it only confirms the PHY
//    layer accepted the frame.  ACK-based detection (a real
//    response from the receiver) is a true application-level
//    heartbeat and gives accurate link status.
//
//  ── HARDWARE ON TX ESP32 ────────────────────────────────────────
//    2× Analog Joystick   LX=34  LY=35  /  RX=32  RY=33
//    4× Push Buttons      LABt=26  RABt=27  LBt=25  RBt=14
//    2× Toggle Switch     TSW1=16  TSW2=17
//    2× Slider Pot        Pot1=39  Pot2=38
//    1× Battery Divider   BAT=36   (100 kΩ + 47 kΩ voltage divider)
//    1× RED LED           GPIO 2   (100 Ω → GND)
//    1× GREEN LED         GPIO 15  (100 Ω → GND)
//    1× JHD12864E LCD     ST7920 128×64, SPI mode
//         CS  → GPIO 5
//         CLK → GPIO 18   (VSPI SCK  — fixed by ESP32 silicon)
//         SID → GPIO 23   (VSPI MOSI — fixed by ESP32 silicon)
//         RST → GPIO 4
//
//  ── WIRING — JHD12864E / ST7920 128×64 LCD (SPI mode) ──────────
//
//  Pin  Signal  Connect to
//  ───  ──────  ──────────────────────────────────────────────────
//   1   VSS     GND
//   2   VDD     5 V  (or 3.3 V if your module is 3.3 V tolerant)
//   3   V0      Wiper of 10 kΩ contrast pot
//               left leg → GND, right leg → 5 V / 3.3 V
//   4   RS      ESP32 GPIO 5    (acts as SPI CS in SPI mode)
//   5   R/W     ESP32 GPIO 23   (SPI MOSI / SID data line)
//   6   E       ESP32 GPIO 18   (SPI SCK / CLK)
//  7-14 DB0-DB7 NC  (not used in SPI mode)
//  15   PSB     GND  ← MUST be tied to GND to enable SPI mode!
//  16   NC      NC
//  17   RST     ESP32 GPIO 4    (or tie to VDD to skip reset)
//  18   VOUT    NC  (optionally add 0.1 µF cap to GND for charge pump)
//  19   BLA     5 V via 10 Ω   (backlight anode)
//  20   BLK     GND             (backlight cathode)
//
//  ── ESP32 Pin Map ───────────────────────────────────────────────
//  GPIO 34 → LX   Joystick-L X-axis   (ADC1, input-only, no pullup)
//  GPIO 35 → LY   Joystick-L Y-axis   (ADC1, input-only)
//  GPIO 32 → RX   Joystick-R X-axis   (ADC1)
//  GPIO 33 → RY   Joystick-R Y-axis   (ADC1)
//  GPIO 39 → Pot1 Slider 1            (ADC1, input-only, no pullup)
//  GPIO 38 → Pot2 Slider 2            (ADC1, input-only)
//  GPIO 36 → BAT  Battery sense       (ADC1, input-only)
//               Divider: VBAT ──100 kΩ──┬── GPIO 36
//                                       └── 47 kΩ ── GND
//               Ratio = 147/47 ≈ 3.128×
//               Max safe input = 3.3 V → max readable VBAT ≈ 10.3 V
//               Fully-charged 2S LiPo = 8.4 V → GPIO 36 sees ≈ 2.69 V ✓
//  GPIO 26 → LABt  Left  A-button  (INPUT_PULLUP, button → GND)
//  GPIO 27 → RABt  Right A-button  (INPUT_PULLUP)
//  GPIO 25 → LBt   Left  B-button  (INPUT_PULLUP)
//  GPIO 14 → RBt   Right B-button  (INPUT_PULLUP)
//  GPIO 16 → TSW1  Toggle switch 1 (INPUT_PULLUP, switch → GND)
//  GPIO 17 → TSW2  Toggle switch 2 (INPUT_PULLUP)
//  GPIO  2 → RED LED   (100 Ω → GND; HIGH = LED on)
//  GPIO 15 → GREEN LED (100 Ω → GND; HIGH = LED on)
//  GPIO  5 → LCD CS / RS
//  GPIO 18 → LCD CLK / E    (VSPI SCK  — do NOT reassign)
//  GPIO 23 → LCD SID / R/W  (VSPI MOSI — do NOT reassign)
//  GPIO  4 → LCD RST
//
//  ── Joystick module wiring (KY-023 style) ───────────────────────
//  GND → GND,  VCC → 3.3 V (NOT 5 V — ESP32 ADC max is 3.3 V!)
//  VRx → LX_PIN / RX_PIN
//  VRy → LY_PIN / RY_PIN
//  SW  → LABt_PIN / RABt_PIN  (INPUT_PULLUP handles the pull-up)
//
//  ── Library required ────────────────────────────────────────────
//  U8g2 by olikraus  — install via Arduino Library Manager
// ================================================================

#include <U8g2lib.h>
#include <esp_now.h>
#include <WiFi.h>


// ================================================================
//  PAYLOAD STRUCTS
//  Both must be byte-for-byte identical on TX and every RX.
// ================================================================

// ── Outgoing: controller state sent from TX → RX ─────────────────
typedef struct {
  int  Lx, Ly, Rx, Ry;   // Joystick axes         range −99 … +99
  bool LABt, RABt;         // Joystick A-buttons    (press-to-click)
  bool LBt,  RBt;          // Extra B-buttons
  bool TSW1, TSW2;          // Toggle switches
  int  Pot1, Pot2;          // Potentiometers        range 0 … 100
  int  BAT;                 // Battery percentage    range 0 … 100
} ControllerData;


// ── Incoming: acknowledgement sent from RX → TX ──────────────────
// RX sends this immediately after receiving a valid ControllerData
// packet.  TX uses the arrival time to determine link health.
typedef struct {
  bool alive;   // always true; presence of packet is what matters
} AckData;


ControllerData txData;           // current outgoing data, updated each loop
static unsigned long lastAckMs = 0;   // millis() of the last received ACK
bool gConnected = false;              // true if ACK arrived within CONN_TIMEOUT_MS


// ================================================================
//  BROADCAST MAC
//  FF:FF:FF:FF:FF:FF makes every ESP32 on the same channel receive
//  the packet without needing TX to know receiver MACs in advance.
//
//  If you want to target a specific receiver only, set BROADCAST 0
//  and paste its MAC into peerMAC[] below.
// ================================================================
#define BROADCAST  1          // 1 = broadcast to all, 0 = unicast

#if BROADCAST
  // Broadcast address — every ESP-NOW node on the channel receives this
  uint8_t peerMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#else
  // Unicast — replace with the actual MAC of your receiver ESP32.
  // Read the receiver's MAC by running  Serial.println(WiFi.macAddress())
  // in its setup(), then paste the 6 bytes here.
  uint8_t peerMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
#endif


// ================================================================
//  PINS
// ================================================================

// ── Joystick axes (ADC1, 12-bit, 0–4095) ─────────────────────────
#define PIN_LX    34   // Left  joystick X
#define PIN_LY    35   // Left  joystick Y
#define PIN_RX    32   // Right joystick X
#define PIN_RY    33   // Right joystick Y

// ── Digital inputs (INPUT_PULLUP; active LOW, inverted in firmware) ─
#define PIN_LABt  26   // Left  joystick click / A-button
#define PIN_RABt  27   // Right joystick click / A-button
#define PIN_LBt   25   // Left  extra B-button
#define PIN_RBt   14   // Right extra B-button
#define PIN_TSW1  16   // Toggle switch 1
#define PIN_TSW2  17   // Toggle switch 2

// ── Analog inputs (ADC1, input-only) ─────────────────────────────
#define PIN_POT1  39   // Slider potentiometer 1
#define PIN_POT2  38   // Slider potentiometer 2
#define PIN_BAT   36   // Battery voltage divider sense pin

// ── Status LEDs ───────────────────────────────────────────────────
#define PIN_LED_R  2   // RED   LED — no ACK / error / low battery
#define PIN_LED_G 15   // GREEN LED — ACK received / link healthy

// ── LCD (ST7920 SPI) ──────────────────────────────────────────────
#define PIN_LCD_CS   5   // SPI chip-select (LCD RS pin)
#define PIN_LCD_RST  4   // LCD reset


// ================================================================
//  TUNING
// ================================================================
#define DEADZONE         11   // Joystick dead-band in mapped (−99..+99) units
#define CHANGE_THRESH     2   // Minimum axis movement before updating txData
#define POT_ZERO_THRESH  80   // Startup gate: raw ADC ≥ (4095 − this) = slider at min

// Battery thresholds for a 2S LiPo (7.4 V nominal)
#define BAT_FULL       8.4f   // 100% — fully charged 2S LiPo
#define BAT_EMPTY      6.6f   //   0% — storage / cutoff voltage
#define BAT_LOW_V      6.8f   // Below this → safe-mode: kill radio
#define BAT_LOW_PCT    10     // Startup blocks until battery is above this %

// Link timeout — if no ACK arrives for this long, mark as disconnected
#define CONN_TIMEOUT_MS  1500   // 1.5 s at 20 Hz = ~30 missed packets


// ================================================================
//  U8g2 — ST7920 128×64, full-frame buffer, hardware SPI
//
//  Hardware SPI bus used: VSPI
//    MOSI = GPIO 23  (fixed by ESP32 silicon, drives LCD SID)
//    SCK  = GPIO 18  (fixed by ESP32 silicon, drives LCD CLK / E)
//    CS   = GPIO  5  (user-defined, drives LCD RS)
//    RST  = GPIO  4  (user-defined, drives LCD RST)
//
//  PSB pin on LCD MUST be tied to GND to enable SPI mode.
// ================================================================
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, PIN_LCD_CS, PIN_LCD_RST);


// ================================================================
//  GLOBALS
// ================================================================
int  gBatPct  = 100;   // Battery percentage, updated by batteryManager()

// IIR-smoothed display values (prevent jitter flickering on LCD)
static int sLx=0, sLy=0, sRx=0, sRy=0;
static int sP1=50, sP2=50, sBat=100;

// Animation helpers
static uint8_t animTick = 0;   // 0–199 counter, incremented each frame
static bool    batBlink = false;  // true on alternating ticks for low-bat blink


// ================================================================
//  HELPERS
// ================================================================

// Map raw 12-bit ADC (0–4095) to axis units (−99 … +99) with dead-band.
// Axis direction: high ADC → negative (matches typical KY-023 orientation).
int mapAxis(int raw)
{
  int v = map(raw, 0, 4095, 99, -99);
  return (abs(v) < DEADZONE) ? 0 : v;
}

// Map raw 12-bit ADC (0–4095) to percentage (0–100).
// High ADC = slider at mechanical minimum → 0 %
int mapPot(int raw)
{
  return map(raw, 0, 4095, 100, 0);
}

// Single-pole IIR low-pass filter for smooth LCD display.
// α = 0.25 → ~75% of previous value retained each tick.
// Snaps directly to target when the difference is ≤ 1.
int iirSmooth(int cur, int tgt)
{
  int d = tgt - cur;
  return (d == 0 || abs(d) <= 1) ? tgt : cur + d / 4;
}

// Read battery voltage through the 100 kΩ + 47 kΩ voltage divider.
// Returns percentage; writes actual voltage to vOut.
int readBatPct(float &vOut)
{
  // ADC reads divider mid-point; scale back up to VBAT
  float vADC = analogRead(PIN_BAT) * (3.3f / 4095.0f);
  vOut = vADC * (147.0f / 47.0f);   // (100k + 47k) / 47k ≈ 3.128

  if (vOut >= BAT_FULL)  return 100;
  if (vOut <= BAT_EMPTY) return 0;
  return (int)((vOut - BAT_EMPTY) * 100.0f / (BAT_FULL - BAT_EMPTY));
}


// ================================================================
//  ESP-NOW CALLBACKS
// ================================================================

// ── OnDataSent — fires after the radio driver attempts a TX ───────
//
//  NOTE: This callback reports whether the 802.11 PHY layer
//  accepted the frame — NOT whether any receiver is alive.
//  ESP_NOW_SEND_SUCCESS means the channel was clear and the packet
//  was transmitted; it does NOT confirm reception.
//
//  Connection status (gConnected) is driven by ACK reception in
//  OnDataRecv below — that is a true application-level heartbeat.
//  This callback is used for debug Serial output only.
//
//  ── Core version note ───────────────────────────────────────────
//  Arduino Core 2.x signature:
//    void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
//  Arduino Core 3.x renamed the first argument to wifi_tx_info_t*.
//  If you get a compile error on this function, comment out the
//  mac_addr Serial.printf below and change the signature to match
//  your core version.
// ────────────────────────────────────────────────────────────────
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Debug only — connection LED is updated in OnDataRecv
  Serial.printf("[TX→SEND] PHY status: %s  to: %02X:%02X:%02X:%02X:%02X:%02X\n",
    status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL",
    mac_addr[0], mac_addr[1], mac_addr[2],
    mac_addr[3], mac_addr[4], mac_addr[5]);
}

// ── OnDataRecv — fires when TX receives any ESP-NOW packet ────────
//
//  RX sends an AckData struct immediately after it processes each
//  ControllerData frame.  TX updates lastAckMs here; the main loop
//  checks how long ago the last ACK arrived to set gConnected.
//
//  WHY THE MAC FILTER CHECK?
//  In broadcast mode TX might receive stray ESP-NOW packets from
//  other devices.  We only count ACKs from known receivers.
//  If you have multiple RX units you can remove the MAC check and
//  treat any valid AckData from any sender as proof of link life.
// ────────────────────────────────────────────────────────────────
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
{
  // Verify payload size matches our AckData struct
  if (len != sizeof(AckData)) return;

  AckData ack;
  memcpy(&ack, incomingData, sizeof(ack));

  if (!ack.alive) return;   // malformed or stale packet

  // Record arrival time — used by batteryManager() to set gConnected
  lastAckMs = millis();

  Serial.printf("[TX→ACK ] Received from %02X:%02X:%02X:%02X:%02X:%02X\n",
    info->src_addr[0], info->src_addr[1], info->src_addr[2],
    info->src_addr[3], info->src_addr[4], info->src_addr[5]);
}


// ================================================================
//  READ ALL INPUTS & SEND ESP-NOW PACKET
// ================================================================
void readAndSend()
{
  // ── Joystick axes ─────────────────────────────────────────────
  // CHANGE_THRESH prevents micro-jitter from updating the struct
  // (and saturating the ESP-NOW channel) on every tick.
  int nLx = mapAxis(analogRead(PIN_LX));
  int nLy = mapAxis(analogRead(PIN_LY));
  int nRx = mapAxis(analogRead(PIN_RX));
  int nRy = mapAxis(analogRead(PIN_RY));

  if (abs(nLx - txData.Lx) > CHANGE_THRESH) txData.Lx = nLx;
  if (abs(nLy - txData.Ly) > CHANGE_THRESH) txData.Ly = nLy;
  if (abs(nRx - txData.Rx) > CHANGE_THRESH) txData.Rx = nRx;
  if (abs(nRy - txData.Ry) > CHANGE_THRESH) txData.Ry = nRy;

  // ── Digital inputs — active LOW (INPUT_PULLUP), inverted here ─
  txData.LABt = !digitalRead(PIN_LABt);
  txData.RABt = !digitalRead(PIN_RABt);
  txData.LBt  = !digitalRead(PIN_LBt);
  txData.RBt  = !digitalRead(PIN_RBt);
  txData.TSW1 = !digitalRead(PIN_TSW1);
  txData.TSW2 = !digitalRead(PIN_TSW2);

  // ── Potentiometers ────────────────────────────────────────────
  txData.Pot1 = mapPot(analogRead(PIN_POT1));
  txData.Pot2 = mapPot(analogRead(PIN_POT2));

  // ── Battery percentage (kept up to date by batteryManager) ───
  txData.BAT = gBatPct;

  // ── Fire ESP-NOW broadcast ────────────────────────────────────
  // peerMAC is FF:FF:FF:FF:FF:FF (broadcast) or a specific MAC.
  // All registered peers on the channel receive this frame.
  esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));

  // ── Serial debug ──────────────────────────────────────────────
  Serial.printf("[TX→OUT ] L(%3d,%3d) R(%3d,%3d) | LA:%d RA:%d LB:%d RB:%d"
                " | S1:%d S2:%d | P1:%3d P2:%3d | BAT:%3d%%\n",
    txData.Lx, txData.Ly, txData.Rx, txData.Ry,
    txData.LABt, txData.RABt, txData.LBt, txData.RBt,
    txData.TSW1, txData.TSW2,
    txData.Pot1, txData.Pot2, txData.BAT);
    
}


// ================================================================
//  BATTERY MANAGER
//
//  Call every loop tick.
//  Returns true  → low-battery safe-mode active; caller must skip
//                  normal operation and show the warning screen.
//  Returns false → battery OK; proceed normally.
//
//  Also updates gConnected based on ACK heartbeat timing.
//  Doing this here (rather than in OnDataRecv) keeps the logic in
//  one place and avoids race conditions with the callback context.
// ================================================================
bool batteryManager()
{
  static unsigned long lastRead  = 0;
  static unsigned long lastBlink = 0;
  static bool  ledState = false;
  static bool  killSent = false;
  static float batV     = 8.4f;

  // Re-read voltage every 250 ms (ADC is noisy; no need to read faster)
  if (millis() - lastRead > 250) {
    lastRead = millis();
    gBatPct  = readBatPct(batV);
    Serial.printf("[TX→BAT ] %.2fV  %d%%\n", batV, gBatPct);
  }

  // ── Connection status — derived from ACK heartbeat ────────────
  // If the last ACK arrived more than CONN_TIMEOUT_MS ago, the
  // link is considered lost.  At 20 Hz transmit rate this equals
  // approximately 30 consecutive missed ACKs.
  gConnected = (millis() - lastAckMs < CONN_TIMEOUT_MS);

  // Update connection LEDs (GREEN = linked, RED = no link)
  // Note: battery safe-mode below will override these with a blink.
  if (gConnected) {
    digitalWrite(PIN_LED_G, HIGH);
    digitalWrite(PIN_LED_R, LOW);
  } else {
    digitalWrite(PIN_LED_G, LOW);
    digitalWrite(PIN_LED_R, HIGH);
  }

  // ── Low-battery safe-mode ─────────────────────────────────────
  if (batV <= BAT_LOW_V) {
    if (!killSent) {
      // Send one zero-state packet so the receiver knows to stop
      memset(&txData, 0, sizeof(txData));
      esp_now_send(peerMAC, (uint8_t *)&txData, sizeof(txData));
      Serial.printf("[TX→BAT ] !! SAFE MODE !! %.2fV %d%% — radio KILLED\n",
                    batV, gBatPct);
      killSent = true;
    }
    // Override LEDs: fast-blink red to signal low battery
    if (millis() - lastBlink > 300) {
      lastBlink = millis();
      ledState  = !ledState;
      digitalWrite(PIN_LED_R, ledState);
    }
    digitalWrite(PIN_LED_G, LOW);
    return true;   // tell caller to show warning screen and skip TX
  }

  killSent = false;
  return false;
}


// ================================================================
//  LCD DRAW HELPERS
// ================================================================

// Square joystick widget with dashed crosshair and moving 3×3 dot.
//   cx, cy = centre pixel    half = box half-size in pixels
//   lx, ly = axis values in −99 … +99 (positive Y = stick up)
void drawJoystick(uint8_t cx, uint8_t cy, uint8_t half, int lx, int ly)
{
  uint8_t x0 = cx - half, y0 = cy - half, sz = half * 2 + 1;
  u8g2.drawFrame(x0, y0, sz, sz);                              // outer box
  for (uint8_t i = x0+2; i < x0+sz-2; i+=2)
    u8g2.drawPixel(i, cy);                                     // horizontal dashes
  for (uint8_t i = y0+2; i < y0+sz-2; i+=2)
    u8g2.drawPixel(cx, i);                                     // vertical dashes
  // corner tick marks (visual reference)
  u8g2.drawPixel(x0+1, y0+1);     u8g2.drawPixel(x0+sz-2, y0+1);
  u8g2.drawPixel(x0+1, y0+sz-2); u8g2.drawPixel(x0+sz-2, y0+sz-2);
  // moving dot: +Y axis maps upward on screen (dy negated)
  uint8_t inn = half - 3;
  int8_t  dx  = (int8_t)((lx * (int8_t)inn) / 99);
  int8_t  dy  = (int8_t)((ly * (int8_t)inn) / 99);
  u8g2.drawBox(cx + dx - 1, cy - dy - 1, 3, 3);              // 3×3 dot
}

// Vertical fill bar — fill rises from bottom (100 % = completely full).
//   x, y = top-left corner    w, h = dimensions    pct = 0–100
void drawVSlider(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t pct)
{
  u8g2.drawFrame(x, y, w, h);
  uint8_t fh = (uint16_t)pct * (h - 2) / 100;
  if (fh) u8g2.drawBox(x + 1, y + h - 1 - fh, w - 2, fh);
}

// Signal-strength widget — 3 bars, bottom-aligned.
// Tallest bar blinks at ~2 Hz when connected (animTick drives blink).
// All bars show as outlines only when disconnected.
void drawSignal(uint8_t cx, uint8_t bot, bool connected)
{
  const uint8_t BW = 4, GAP = 3, x0 = cx - 9;
  const uint8_t HEIGHTS[3] = {6, 11, 16};
  bool blink = (animTick / 5) % 2;   // ~2 Hz blink period
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t bx = x0 + i * (BW + GAP);
    uint8_t bh = HEIGHTS[i];
    uint8_t by = bot - bh;
    // Top bar blinks when connected; all bars filled when connected, outlines when not
    bool filled = connected && (i < 2 || blink);
    if (filled) u8g2.drawBox  (bx, by, BW, bh);
    else        u8g2.drawFrame(bx, by, BW, bh);
  }
}

// Button box — filled (inverted) with white label when active/pressed,
// outline with dark label when inactive.
void drawBtn(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
             const char *label, bool active)
{
  if (active) {
    u8g2.drawBox(x, y, w, h);          // fill box
    u8g2.setDrawColor(0);               // switch to "erase" colour for text
    u8g2.drawStr(x + 1, y + h - 1, label);
    u8g2.setDrawColor(1);               // restore normal draw colour
  } else {
    u8g2.drawFrame(x, y, w, h);
    u8g2.drawStr(x + 1, y + h - 1, label);
  }
}

// Vertical mobile-style battery icon — nub on top, fill from bottom.
// When blink=true the fill is suppressed (produces a flash-empty look).
void drawVBattery(uint8_t bx, uint8_t by, uint8_t bw, uint8_t bh,
                  uint8_t pct, bool blink)
{
  uint8_t nw = bw / 2, nx = bx + (bw - nw) / 2;
  u8g2.drawBox  (nx, by - 2, nw, 2);          // positive nub above body
  u8g2.drawFrame(bx, by, bw, bh);              // body outline
  if (!blink) {
    uint8_t fh = (uint16_t)pct * (bh - 2) / 100;
    if (fh) u8g2.drawBox(bx + 1, by + bh - 1 - fh, bw - 2, fh);
  }
}


// ================================================================
//  SCREEN: NORMAL OPERATION
//
//  Layout (128×64 pixels):
//  ┌───────────────────────────────────────────────────────────────┐
//  │ [L joy] [P1↕] [▪▪▪ signal ▪▪▪] [P2↕] [R joy]              │ y 0-28
//  ├───────────────────────────────────────────────────────────────┤ y 29
//  │ [LA][LB][S1]          (gap)           [S2][RA][RB]           │ y 30-44
//  ├───────────────────────────────────────────────────────────────┤ y 45
//  │ ┌──────────────────┐                        [▓bat] 82%       │ y 46-63
//  │ │ HASHTAG          │                                          │
//  │ │ URC v1.1         │                                          │
//  │ └──────────────────┘                                          │
//  └───────────────────────────────────────────────────────────────┘
// ================================================================
void drawNormal()
{
  u8g2.setFont(u8g2_font_4x6_tf);

  // ── Top row: joysticks (left & right), vertical pot sliders, signal ──
  drawJoystick( 13,  14, 12, sLx, sLy);          // L stick, centre x=13 y=14
  drawVSlider ( 27,   2,  5, 25, (uint8_t)sP1);  // Pot1 bar, to the right of L stick
  drawSignal  ( 64,  25, gConnected);             // signal bars, screen centre
  drawVSlider ( 96,   2,  5, 25, (uint8_t)sP2);  // Pot2 bar, to the left of R stick
  drawJoystick(114,  14, 12, sRx, sRy);          // R stick, centre x=114 y=14

  u8g2.drawHLine(0, 29, 128);   // separator line

  // ── Middle row: buttons, symmetric around x=63 ───────────────
  drawBtn( 2, 31, 10, 8, "LA", txData.LABt);
  drawBtn(14, 31, 10, 8, "LB", txData.LBt);
  drawBtn(26, 31, 10, 8, "S1", txData.TSW1);
  drawBtn(90, 31, 10, 8, "S2", txData.TSW2);
  drawBtn(102,31, 10, 8, "RA", txData.RABt);
  drawBtn(114,31, 10, 8, "RB", txData.RBt);

  u8g2.drawHLine(0, 45, 128);   // separator line

  // ── Bottom row: name box (left) and battery indicator (right) ─
  u8g2.drawFrame(0, 46, 58, 17);   // name box outline
  u8g2.drawStr(3, 54, "HASHTAG");
  u8g2.drawStr(3, 61, "URC v1.1");

  // Battery icon blinks when below 20 %
  bool batLow = (sBat < 20) && batBlink;
  drawVBattery(96, 49, 8, 11, (uint8_t)sBat, batLow);

  char buf[5];
  sprintf(buf, sBat < 10 ? "  %d%%" : sBat < 100 ? " %d%%" : "%d%%", sBat);
  u8g2.drawStr(106, 61, buf);
}


// ================================================================
//  SCREEN: STARTUP SAFETY CAUTION
// ================================================================
void drawCaution(bool sw1OK, bool sw2OK, bool p1OK, bool p2OK, bool batOK)
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(16, 10, "!! CAUTION !!");
  u8g2.drawHLine(0, 12, 128);
  u8g2.setFont(u8g2_font_4x6_tf);

  u8g2.drawStr( 2, 24, "SW1:"); u8g2.drawStr(22, 24, sw1OK ? " OK" : "OFF");
  u8g2.drawStr(66, 24, "SW2:"); u8g2.drawStr(86, 24, sw2OK ? " OK" : "OFF");
  u8g2.drawStr( 2, 34, "P1 :"); u8g2.drawStr(22, 34, p1OK  ? " OK" : "MIN");
  u8g2.drawStr(66, 34, "P2 :"); u8g2.drawStr(86, 34, p2OK  ? " OK" : "MIN");
  u8g2.drawStr( 2, 44, "BAT:");
  if (batOK) { char b[6]; sprintf(b, " %3d%%", gBatPct); u8g2.drawStr(22, 44, b); }
  else         u8g2.drawStr(22, 44, " LOW!");

  u8g2.drawHLine(0, 47, 128);

  // Animated waiting dots (0–3 dots cycling at ~2 Hz)
  char dots[14] = "Waiting ";
  uint8_t nd = (animTick / 8) % 4;
  for (uint8_t i = 0; i < nd; i++) dots[8 + i] = '.';
  dots[8 + nd] = '\0';
  u8g2.drawStr(10, 58, dots);
}


// ================================================================
//  SCREEN: LOW BATTERY WARNING
// ================================================================
void drawLowBat()
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(14, 14, "LOW BATTERY!");
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.drawStr(24, 26, "Radio  OFF");
  // Horizontal battery bar (large, centre screen)
  u8g2.drawFrame(10, 32, 96, 20);   // body
  u8g2.drawBox (106, 38,  5,  8);   // positive nub
  if (batBlink) {
    uint8_t fill = (uint16_t)gBatPct * 94 / 100;
    if (fill) u8g2.drawBox(11, 33, fill, 18);
  }
  char buf[5]; sprintf(buf, "%d%%", gBatPct);
  u8g2.drawStr(50, 45, buf);
}


// ================================================================
//  STARTUP SAFETY GATE
//
//  Blocks in a loop until ALL of these conditions are true:
//    TSW1 = OFF  (HIGH on INPUT_PULLUP)
//    TSW2 = OFF
//    Pot1 raw ADC ≥ (4095 − POT_ZERO_THRESH)  = slider at minimum
//    Pot2 raw ADC ≥ (4095 − POT_ZERO_THRESH)
//    Battery ≥ BAT_LOW_PCT (10 %)
//
//  This prevents the receiver from receiving unexpected high pot /
//  switch states on power-up, which could cause sudden motor jumps.
// ================================================================
void startup()
{
  Serial.println("[TX→BOOT] Startup safety check:");
  Serial.println("[TX→BOOT]   TSW1 and TSW2 must be OFF");
  Serial.println("[TX→BOOT]   Pot1 and Pot2 must be at minimum");
  Serial.println("[TX→BOOT]   Battery must be >= 10%");

  while (true)
  {
    float batV;
    gBatPct = readBatPct(batV);

    bool sw1OK = (bool)digitalRead(PIN_TSW1);                         // HIGH = OFF = safe
    bool sw2OK = (bool)digitalRead(PIN_TSW2);
    bool p1OK  = (analogRead(PIN_POT1) >= (4095 - POT_ZERO_THRESH));  // near raw max = min throw
    bool p2OK  = (analogRead(PIN_POT2) >= (4095 - POT_ZERO_THRESH));
    bool batOK = (gBatPct >= BAT_LOW_PCT);

    Serial.printf("[TX→CAUTION] SW1:%s SW2:%s P1:%s P2:%s BAT:%s(%d%%)\n",
      sw1OK?"OK":"WAIT", sw2OK?"OK":"WAIT",
      p1OK ?"OK":"WAIT", p2OK ?"OK":"WAIT",
      batOK?"OK":"LOW",  gBatPct);

    if (sw1OK && sw2OK && p1OK && p2OK && batOK) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(4,  22, "** ALL CLEAR **");
      u8g2.setFont(u8g2_font_4x6_tf);
      u8g2.drawStr(22, 38, "System Ready");
      u8g2.drawStr(18, 50, "Starting up ...");
      u8g2.sendBuffer();
      Serial.println("[TX→BOOT] All clear — proceeding");
      delay(900);
      return;
    }

    animTick++;
    u8g2.clearBuffer();
    drawCaution(sw1OK, sw2OK, p1OK, p2OK, batOK);
    u8g2.sendBuffer();
    delay(150);
  }
}


// ================================================================
//  UPDATE DISPLAY — called each loop tick
// ================================================================
void updateDisplay()
{
  // Apply IIR smoothing to suppress joystick ADC jitter on screen
  sLx  = iirSmooth(sLx,  txData.Lx);
  sLy  = iirSmooth(sLy,  txData.Ly);
  sRx  = iirSmooth(sRx,  txData.Rx);
  sRy  = iirSmooth(sRy,  txData.Ry);
  sP1  = iirSmooth(sP1,  txData.Pot1);
  sP2  = iirSmooth(sP2,  txData.Pot2);
  sBat = iirSmooth(sBat, gBatPct);

  animTick++;
  if (animTick >= 200) animTick = 0;
  batBlink = ((animTick % 10) < 5);   // 2 Hz blink for low-battery warning

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
  Serial.println(" ESP32 Universal RC Controller — TX v1.1 boot");
  Serial.println("================================================");

  // ── Output pins ──────────────────────────────────────────────
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  digitalWrite(PIN_LED_R, HIGH);   // red during boot to signal initialising
  digitalWrite(PIN_LED_G, LOW);

  // ── Digital input pins (active LOW buttons/switches) ─────────
  // INPUT_PULLUP enables the internal ~45 kΩ pull-up.
  // Button/switch connects the pin to GND when active.
  pinMode(PIN_LABt, INPUT_PULLUP);
  pinMode(PIN_RABt, INPUT_PULLUP);
  pinMode(PIN_LBt,  INPUT_PULLUP);
  pinMode(PIN_RBt,  INPUT_PULLUP);
  pinMode(PIN_TSW1, INPUT_PULLUP);
  pinMode(PIN_TSW2, INPUT_PULLUP);

  // ── ADC configuration ─────────────────────────────────────────
  // ADC_11db attenuation extends the measurable range to ~3.3 V
  // (from the default ~1.1 V), needed for full joystick/pot swing.
  // This applies to all ADC1 pins (GPIO 32-39).
  analogSetAttenuation(ADC_11db);
  Serial.println("[TX→BOOT] GPIO and ADC configured");

  // ── LCD init ─────────────────────────────────────────────────
  // u8g2.begin() initialises the ST7920 over hardware SPI,
  // sends the init sequence, and clears the display RAM.
  u8g2.begin();
  u8g2.setFont(u8g2_font_4x6_tf);
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  Serial.println("[TX→BOOT] LCD ST7920 init OK");

  // ── Startup safety gate ───────────────────────────────────────
  // Uncomment to enable the safety interlock on power-up.
  // startup();

  // ── WiFi / ESP-NOW init ───────────────────────────────────────
  // ESP-NOW requires WiFi in station mode.
  // The MAC address shown here is what receivers must know to
  // filter incoming ControllerData packets (see RX firmware).
  WiFi.mode(WIFI_STA);
  Serial.printf("[TX→BOOT] TX MAC (give this to each RX): %s\n",
    WiFi.macAddress().c_str());
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[TX→ERROR] ESP-NOW init FAILED — halting");
    // Halt with fast blink so the problem is obvious on the bench
    while (1) { digitalWrite(PIN_LED_R, !digitalRead(PIN_LED_R)); delay(200); }
  }

  // Register send callback (debug Serial only — not used for link status)
  esp_now_register_send_cb(OnDataSent);

  // Register receive callback — this is where ACK packets are caught
  // and lastAckMs is updated to track connection health.
  esp_now_register_recv_cb(OnDataRecv);

  // ── Register broadcast peer ───────────────────────────────────
  // ESP-NOW requires every destination MAC to be registered as a
  // peer before esp_now_send() will accept it.
  // Broadcast (FF:FF:FF:FF:FF:FF) is a valid peer address.
  // channel = 0 → use the current WiFi channel automatically.
  // encrypt = false → no CCMP encryption (both sides must match).
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.printf("[TX→BOOT] ESP-NOW ready  mode:%s  peer:%02X:%02X:%02X:%02X:%02X:%02X\n",
    BROADCAST ? "BROADCAST" : "UNICAST",
    peerMAC[0], peerMAC[1], peerMAC[2],
    peerMAC[3], peerMAC[4], peerMAC[5]);

  digitalWrite(PIN_LED_R, LOW);
  Serial.println("[TX→BOOT] Boot complete — running at ~20 Hz\n");
}


// ================================================================
//  MAIN LOOP  — ~20 Hz (50 ms period)
// ================================================================
void loop()
{
  // batteryManager():
  //   1. Reads VBAT every 250 ms and updates gBatPct
  //   2. Updates gConnected based on lastAckMs heartbeat
  //   3. Updates connection LEDs (GREEN = ACK within timeout)
  //   4. Returns true if VBAT ≤ BAT_LOW_V (safe-mode)
  if (batteryManager()) {
    // Safe-mode: show low-battery screen and do NOT transmit
    u8g2.clearBuffer();
    animTick++;
    if (animTick >= 200) animTick = 0;
    batBlink = ((animTick % 10) < 5);
    drawLowBat();
    u8g2.sendBuffer();
    delay(50);
    return;   // skip readAndSend() and updateDisplay()
  }

  readAndSend();    // 1. sample all inputs → update txData → esp_now_send()
  updateDisplay();  // 2. IIR smooth → render LCD frame
  delay(50);        // 3. pace loop to ~20 Hz
}
