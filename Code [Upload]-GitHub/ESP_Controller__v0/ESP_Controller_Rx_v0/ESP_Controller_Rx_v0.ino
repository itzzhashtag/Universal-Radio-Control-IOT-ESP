#include <esp_now.h>
#include <WiFi.h>

typedef struct { int ping; } Packet;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *buf, int len)
{
  Packet rx;
  memcpy(&rx, buf, sizeof(rx));

  Serial.printf("RX: ✓ Connected — ping #%d from %02X:%02X:%02X:%02X:%02X:%02X\n",
    rx.ping,
    info->src_addr[0], info->src_addr[1], info->src_addr[2],
    info->src_addr[3], info->src_addr[4], info->src_addr[5]);
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("RX MAC: ");
  Serial.println(WiFi.macAddress());

  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("RX ready — waiting for pings...");
}

void loop()
{
  delay(10);
}