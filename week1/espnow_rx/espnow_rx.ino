// ESP-NOW Receiver (basic test)
#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  int command;
} struct_message;

struct_message incomingData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataBuffer, int len) {
  memcpy(&incomingData, incomingDataBuffer, sizeof(incomingData));
  Serial.print("Received Command: ");
  Serial.println(incomingData.command);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {}
