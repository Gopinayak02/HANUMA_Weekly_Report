// ESP-NOW Transmitter (basic test)
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct struct_message {
  int command;
} struct_message;

struct_message testData;

void setup() {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);
}

void loop() {
  testData.command = 1; // forward
  esp_now_send(broadcastAddress, (uint8_t *) &testData, sizeof(testData));
  delay(500);
}
