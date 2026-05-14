#ifndef RECEIVER_H
#define RECEIVER_H

#include <esp_now.h>
#include <WiFi.h>

// =====================================================
// DATA STRUCT (same as TX)
// =====================================================
typedef struct 
{
  int  LX, LY;
  int  RX, RY;
  bool LS, RS;
  bool LB, RB;
  bool SW1, SW2;
  int  PotM1;
} ControllerData;

// =====================================================
// RECEIVER CLASS
// =====================================================
class Receiver
{
  public:
    ControllerData data;
    bool newData = false;

    void begin()
    {
      instance = this;

      WiFi.mode(WIFI_STA);

      if (esp_now_init() != ESP_OK)
      {
        Serial.println("ESP-NOW Init Failed");
        return;
      }

      esp_now_register_recv_cb(onReceive);

      Serial.println("Receiver Ready...");
    }

  private:
    static Receiver *instance;
    static bool deviceConnected;
    static uint8_t lastMAC[6];

    // ===============================
    // PRINT MAC
    // ===============================
    static void printMAC(const uint8_t *mac)
    {
      char macStr[18];
      sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
              mac[0], mac[1], mac[2],
              mac[3], mac[4], mac[5]);

      Serial.print(macStr);
    }

    // ===============================
    // RECEIVE CALLBACK
    // ===============================
    static void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
    {
      if (instance == nullptr) return;

      memcpy(&instance->data, incomingData, sizeof(instance->data));
      instance->newData = true;

      // 🔌 First connection detect
      if (!deviceConnected || memcmp(lastMAC, info->src_addr, 6) != 0)
      {
        memcpy(lastMAC, info->src_addr, 6);
        deviceConnected = true;

        Serial.print("Connected to: ");
        printMAC(info->src_addr);
        Serial.println();
      }

      // 📡 Print received data
      Serial.printf(
        "RX -> L(%d,%d) R(%d,%d) | LB:%d LS:%d RS:%d RB:%d | SW1:%d SW2:%d | P1:%d\n",
        instance->data.LX, instance->data.LY,
        instance->data.RX, instance->data.RY,
        instance->data.LB, instance->data.LS,
        instance->data.RS, instance->data.RB,
        instance->data.SW1, instance->data.SW2,
        instance->data.PotM1
      );
    }
};

// =====================================================
// STATIC VARIABLES (IMPORTANT)
// =====================================================
Receiver* Receiver::instance = nullptr;
bool Receiver::deviceConnected = false;
uint8_t Receiver::lastMAC[6] = {0};

#endif