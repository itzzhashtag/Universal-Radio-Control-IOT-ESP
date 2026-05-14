// ================================================================
// ESP-NOW RECEIVER (Broadcast Mode + TX Filter)
// ================================================================

#include <esp_now.h>
#include <WiFi.h>

// YOUR TRANSMITTER MAC
uint8_t allowedTX[] = {0x20,0xE7,0xC8,0x9F,0x47,0xF8};

// ================================================================
// DATA STRUCT (same as TX)
// ================================================================
typedef struct 
{
  int  LX, LY;
  int  RX, RY;
  bool LS, RS;
  bool LB, RB;
  bool SW1, SW2;
  int  PotM1;
} ControllerData;

ControllerData data;
typedef struct {
  bool alive;
} AckData;

AckData ack = {true};
bool Stat=false;
uint8_t txMAC[] = {0x20,0xE7,0xC8,0x9F,0x47,0xF8};

// ================================================================
unsigned long lastPacketTime = 0;
#define TIMEOUT 1000

// ================================================================
// CHECK MAC MATCH
// ================================================================
bool isAllowed(uint8_t *mac)
{
  for (int i = 0; i < 6; i++)
    if (mac[i] != allowedTX[i]) return false;
  return true;
}
void sendAck()
{
  esp_now_send(txMAC, (uint8_t*)&ack, sizeof(ack));
}
// ================================================================
// RECEIVE CALLBACK
// ================================================================

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
{
  // 🔒 FILTER BY TX MAC
  if (!isAllowed((uint8_t*)info->src_addr))
  {
    Serial.println("Ignored unknown device");
    return;
  }

  // ✅ COPY DATA
  memcpy(&data, incomingData, sizeof(data));
  lastPacketTime = millis();

  // ✅ PRINT DATA
  Serial.printf(
    "L(%3d,%3d) R(%3d,%3d) | LB:%d RB:%d | SW1:%d SW2:%d | P1:%3d | Status:%s\n",
    data.LX, data.LY,
    data.RX, data.RY,
    data.LB, data.RB,
    data.SW1, data.SW2,
    data.PotM1,(Stat ? "true" : "false"));

  // ============================================================
  // 🔥 SEND ACK BACK TO THE SAME TX (FIXED PROPERLY)
  // ============================================================

  uint8_t *senderMAC = (uint8_t*)info->src_addr;
  // ✅ Ensure TX is added as peer (IMPORTANT!)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, senderMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(senderMAC))
  {
    esp_now_add_peer(&peerInfo);
  }
  // ✅ Send structured ACK
  AckData ack;
  ack.alive = true;
  esp_now_send(senderMAC, (uint8_t*)&ack, sizeof(ack));
  //Serial.println("ACK sent to TX");
  Stat=true;
  // mark alive + timestamp
  Stat = true;
  lastPacketTime = millis();
}
void checkConnection()
{
  if (millis() - lastPacketTime > 1000)  // 1 sec timeout
  {
    Stat = false;
  }
}

// ================================================================
// FAILSAFE
// ================================================================
void checkSignal()
{
  if (millis() - lastPacketTime > TIMEOUT)
  {
    Serial.println("⚠ SIGNAL LOST");
    memset(&data, 0, sizeof(data));
    // stop motors here if needed
  }
}
// ================================================================
// SETUP
// ================================================================
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Receiver Ready (Broadcast Mode)");
}

// ================================================================
// LOOP
// ================================================================
void loop()
{
  checkConnection();
  checkSignal();
  sendAck();
  delay(100);
}