#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t receiverMAC[] = {0x40, 0x91, 0x51, 0x44, 0xE5, 0x5F};  // Receiver MAC

void OnDataSent(uint8_t *mac_addr, uint8_t status) {
  // Optional: Serial.print("Send status: "); Serial.println(status == 0 ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);  // From Mega

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s += "\n";  // Add back \n
    esp_now_send(receiverMAC, (uint8_t*)s.c_str(), s.length());
  }
  yield();  // Prevent WDT reset
}
