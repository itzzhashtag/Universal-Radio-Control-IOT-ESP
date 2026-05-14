// ================================================================
//  ReceiverModule.h — ESP-NOW Drop-in Receiver
//  URC v1.4 — Universal RC Receiver Module
//
//  HOW TO USE:
//  ─────────────────────────────────────────────────────────────
//  1. Copy this file into your sketch folder (same folder as .ino)
//  2. At the very top of your .ino add:
//       #include "ReceiverModule.h"
//  3. Inside setup() add:
//       receiver.begin();
//  4. Inside loop() use:
//       if (receiver.available()) 
//       {
//         // use receiver.data.Lx, receiver.data.Ly, receiver.data.LABt, etc.
//       }
//
//  OPTIONAL — accept only from one specific transmitter MAC:
//       receiver.begin("AA:BB:CC:DD:EE:FF");   // TX prints its MAC at boot
//  Leave empty to accept from ANY sender (matches BROADCAST mode).
//
//  ─────────────────────────────────────────────────────────────
//  COMPLETE EXAMPLE at bottom of this file ↓
// ================================================================

#pragma once
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

// ================================================================
//  PAYLOAD STRUCT — must be byte-for-byte identical to TX
// ================================================================
typedef struct {
  int Lx, Ly, Rx, Ry;  // Joystick axes        -99 .. 99
  bool LABt, RABt;     // A-buttons (left/right)
  bool LBt, RBt;       // B-buttons (left/right)
  bool TSW1, TSW2;     // Toggle switches
  int Pot1, Pot2;      // Potentiometers       0 .. 100
  int BAT;             // Battery percent      0 .. 100
} ControllerData;

// ================================================================
//  RECEIVER CLASS
// ================================================================
class ESPNowReceiver 
{
  public:
  // ── Public data ────────────────────────────────────────────────
  ControllerData data = {};  // latest packet — read this freely
  uint32_t lastRecvMs = 0;   // millis() of last valid packet

  // ── begin(allowedMAC) ──────────────────────────────────────────
  // Call once in setup().
  // Pass transmitter MAC string "AA:BB:CC:DD:EE:FF" to filter,
  // or leave empty / pass "" to accept from any sender.
  void begin(const char *allowedMAC = "") 
  {
    _instance = this;  // set singleton pointer for static callback
    _filterMAC = (allowedMAC && strlen(allowedMAC) == 17);  // Parse MAC filter if provided
    if (_filterMAC) 
    {
      Serial.println("[RX→INIT] ───────────────────────────────────────");
      sscanf(allowedMAC, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",&_mac[0], &_mac[1], &_mac[2],&_mac[3], &_mac[4], &_mac[5]);
      Serial.printf("[RX→INIT] MAC filter ON — only accepting from %s\n", allowedMAC);
    } 
    else 
    {
      Serial.println("[RX→INIT] ───────────────────────────────────────");
      Serial.println("[RX→INIT] Open mode — accepting packets from any transmitter");
    }

    WiFi.mode(WIFI_STA);
    Serial.printf("[RX→INIT] This receiver MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("[RX→INIT] Give the above MAC to your TX if using UNICAST mode");

    if (esp_now_init() != ESP_OK) 
    {
      Serial.println("[RX→ERROR] ESP-NOW init FAILED — halting");
      while (1)
        ;
    }

    // ── Core version compatibility ──────────────────────────────
    // Default: ESP32 Arduino Core 2.x callback signature.
    // If compile error: comment out the line below and uncomment the v3.x line.
    //esp_now_register_recv_cb(_onRecv_v2);          // Core 2.x
    esp_now_register_recv_cb((esp_now_recv_cb_t)_onRecv_v3);  // Core 3.x
    Serial.println("[RX→INIT] ESP-NOW ready — waiting for packets");
    Serial.println("[RX→INIT] ───────────────────────────────────────");
  }

  // ── available() ────────────────────────────────────────────────
  // Returns true exactly once per new packet, then clears the flag.
  bool available() 
  {
    if (_newData) 
    {
      _newData = false;
      return true;
    }
    return false;
  }

  // ── connected() ────────────────────────────────────────────────
  // Returns true if a packet arrived within the last `ms` milliseconds.
  // Default 1000 ms — adjust to taste.
  bool connected(uint32_t ms = 1000) 
  {
    return (millis() - lastRecvMs < ms);
  }

  private:
  bool _filterMAC = false;
  uint8_t _mac[6] = {};
  volatile bool _newData = false;

  // Singleton pointer — allows static callback to reach instance members
  static ESPNowReceiver *_instance;

  // ── Receive callback — Core 2.x ──────────────────────────────
  //static void _onRecv_v2(const uint8_t *mac,const uint8_t *buf, int len)
  //{
  //  _handlePacket(mac, buf, len);
  //}

  // ── Receive callback — Core 3.x (uncomment if needed) ─────────
  static void _onRecv_v3(const esp_now_recv_info_t *info,const uint8_t *buf, int len)
  {
    _handlePacket(info->src_addr, buf, len);
  }

  // ── Shared packet handler ─────────────────────────────────────
  static void _handlePacket(const uint8_t *mac, const uint8_t *buf, int len) 
  {
    if (!_instance) return;

    // Size check — wrong packet = not from our TX
    if (len != (int)sizeof(ControllerData)) 
    {
      Serial.printf("[RX→PKT] Ignored: size mismatch (got %d, expected %d)\n", len, (int)sizeof(ControllerData));
      return;
    }

    // MAC filter check
    if (_instance->_filterMAC && memcmp(mac, _instance->_mac, 6) != 0) 
    {
      Serial.printf("[RX→PKT] Ignored: unexpected sender %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      return;
    }

    // Copy payload into public data struct
    memcpy(&_instance->data, buf, sizeof(ControllerData));
    _instance->lastRecvMs = millis();
    _instance->_newData = true;

    // ── Serial debug — every received packet ─────────────────────
    Serial.printf("[RX←PKT] from %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("[RX←PKT] L(%+3d,%+3d) R(%+3d,%+3d) | LA:%d RA:%d LB:%d RB:%d | S1:%d S2:%d | P1:%3d P2:%3d | BAT:%3d%%\n",
                  _instance->data.Lx, _instance->data.Ly,
                  _instance->data.Rx, _instance->data.Ry,
                  _instance->data.LABt, _instance->data.RABt,
                  _instance->data.LBt, _instance->data.RBt,
                  _instance->data.TSW1, _instance->data.TSW2,
                  _instance->data.Pot1, _instance->data.Pot2,
                  _instance->data.BAT);
  }
};

// ── Singleton definitions ─────────────────────────────────────────
// One global RX object — use receiver.begin(), receiver.available(), receiver.data
ESPNowReceiver receiver;
ESPNowReceiver *ESPNowReceiver::_instance = nullptr;

// ================================================================
//  USAGE EXAMPLE
//  (Copy this into a new .ino, don't modify ReceiverModule.h)
// ================================================================
/*

#include "ReceiverModule.h"

// ── Hexapod / servo / motor driver includes go here ─────────────
// #include <Adafruit_PWMServoDriver.h>
// etc.

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
  if (receiver.available())
  {
    // Use any field from receiver.data:
    int  leftX   = receiver.data.Lx;     // -99..99
    int  leftY   = receiver.data.Ly;
    int  rightX  = receiver.data.Rx;
    int  rightY  = receiver.data.Ry;
    bool btnLA   = receiver.data.LABt;
    bool btnRA   = receiver.data.RABt;
    bool btnLB   = receiver.data.LBt;
    bool btnRB   = receiver.data.RBt;
    bool sw1     = receiver.data.TSW1;
    bool sw2     = receiver.data.TSW2;
    int  pot1    = receiver.data.Pot1;   // 0..100
    int  pot2    = receiver.data.Pot2;
    int  battery = receiver.data.BAT;    // 0..100

    // ── Example: drive two motors with left joystick ──────────
    // int speed = map(leftY, -99, 99, -255, 255);
    // analogWrite(MOTOR_A, abs(speed));
    // digitalWrite(MOTOR_DIR, speed > 0 ? HIGH : LOW);

    // ── Example: check connection timeout ─────────────────────
    if (!receiver.connected(1000)) {
      Serial.println("[RX] WARNING: no packet for 1 second — TX disconnected?");
      // stop motors / safe position etc.
    }
  }

  // Your non-blocking logic here (avoid long delays)
  delay(5);
}


*/
// ================================================================
