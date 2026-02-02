#include <ESP8266WiFi.h>
#include <espnow.h>

unsigned long lastPacket = 0;
unsigned long lastSend = 0;

void SendDefault() {
  Serial.print("128,128,128,128,128,128,128,128,255,0,0,0,0,0,0,0,0,255,0\n");
  lastSend = millis();
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  Serial.write(incomingData, len);  // Assumes data includes \n
  lastPacket = millis();
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    while (1);
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  unsigned long now = millis();
  if (now - lastPacket > 500 && now - lastSend > 500) {
    SendDefault();
  }
}
