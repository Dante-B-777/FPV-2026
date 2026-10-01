// ESP32 ESP-NOW Receiver
// Laptop ESP32: receives normalized joystick data and prints it to Serial Monitor.

#include <WiFi.h>
#include <esp_now.h>

typedef struct JoystickData {
  float xValue;   // -1.0 to +1.0
  float yValue;   // -1.0 to +1.0
  char button;    // 'L', 'R', or 'N'
} JoystickData;

JoystickData incomingData;

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));

  Serial.print(incomingData.xValue, 3);
  Serial.print(",");
  Serial.print(incomingData.yValue, 3);
  Serial.print(",");
  Serial.println(incomingData.button);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("ESP-NOW Receiver Started");
  Serial.println("Receiver MAC Address: 5C:01:3B:BA:CA:EC");
  Serial.println("Waiting for joystick data...");
  Serial.println("X,Y,Button");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataReceive);
}

void loop() {
  // Nothing needed here.
  // Incoming packets are handled by onDataReceive().
}