#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static const uint8_t WIFI_CHANNEL = 1;

typedef struct __attribute__((packed)) {
  uint32_t seq;
  int32_t  speedL;   // xung / 50 ms (có dấu theo chiều quay)
  int32_t  speedR;   // xung / 50 ms (có dấu theo chiều quay)
  int32_t  encL;     // tổng xung tích lũy trái
  int32_t  encR;     // tổng xung tích lũy phải
  float    err;      // sai số dò line hiện tại
  float    distCm;   // khoảng cách siêu âm (cm)
} TelemetryPacket;

volatile bool hasNewPacket = false;
TelemetryPacket latestPkt;
uint8_t lastSenderMac[6] = {0};

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == (int)sizeof(TelemetryPacket) && recv_info != nullptr) {
    memcpy((void *)&latestPkt, incomingData, sizeof(TelemetryPacket));
    if (recv_info->src_addr != nullptr) {
      memcpy(lastSenderMac, recv_info->src_addr, 6);
    }
    hasNewPacket = true;
  }
}
#else
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == (int)sizeof(TelemetryPacket)) {
    memcpy((void *)&latestPkt, incomingData, sizeof(TelemetryPacket));
    if (mac_addr != nullptr) {
      memcpy(lastSenderMac, mac_addr, 6);
    }
    hasNewPacket = true;
  }
}
#endif

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("PC ESP32 STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Dang lang nghe ESP-NOW tren channel %u...\n", WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Loi khoi tao ESP-NOW!");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  if (hasNewPacket) {
    noInterrupts();
    TelemetryPacket pkt = latestPkt;
    uint8_t mac[6];
    memcpy(mac, lastSenderMac, 6);
    hasNewPacket = false;
    interrupts();

    // Tốc độ tương đương xung/giây (vì chu kỳ đo của xe là 50 ms => x20)
    long ppsL = (long)pkt.speedL * 20L;
    long ppsR = (long)pkt.speedR * 20L;

    Serial.printf(
      "[#%lu | %02X:%02X:%02X:%02X:%02X:%02X] spdL=%ld/50ms (%ld/s) | spdR=%ld/50ms (%ld/s) | encL=%ld | encR=%ld | err=%.0f | dist=%.0fcm\n",
      (unsigned long)pkt.seq,
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
      (long)pkt.speedL, ppsL,
      (long)pkt.speedR, ppsR,
      (long)pkt.encL, (long)pkt.encR,
      pkt.err, pkt.distCm
    );
  }
}