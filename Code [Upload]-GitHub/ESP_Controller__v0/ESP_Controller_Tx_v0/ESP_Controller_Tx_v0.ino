#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct { int ping; } Packet;
Packet tx;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
    Serial.println("TX: ✓ Connected — ping sent OK");
  else
    Serial.println("TX: ✗ Send FAILED");
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("TX MAC: ");
  Serial.println(WiFi.macAddress());

  esp_now_init();
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMAC, 6);
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("TX ready — sending pings...");
}

void loop()
{
  tx.ping++;
  esp_now_send(broadcastMAC, (uint8_t *)&tx, sizeof(tx));
  Serial.printf("TX: Sent ping #%d\n", tx.ping);
  delay(1000);
}