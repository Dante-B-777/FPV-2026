// ESP32 ESP-NOW Transmitter
// Joystick ESP32: reads joystick values, normalizes them to -1.0 to +1.0,
// reads buttons, and sends data wirelessly to receiver ESP32.

#include <WiFi.h>
#include <esp_now.h>

#define JOYSTICK_X_PIN 34
#define JOYSTICK_Y_PIN 35

#define LEFT_BUTTON_PIN 26   // D26 sends L
#define RIGHT_BUTTON_PIN 27  // D27 sends R

// Measured idle centers from your data
#define X_CENTER 1924
#define Y_CENTER 1906

// Removes small noise around idle
#define DEADZONE 0.04

// Receiver MAC Address: 5C:01:3B:BA:CA:EC
uint8_t receiverMacAddress[] = {0x70, 0x4B, 0xCA, 0x6E, 0xAF, 0x98};

typedef struct JoystickData {
  float xValue;   // -1.0 to +1.0
  float yValue;   // -1.0 to +1.0
  char button;    // 'L', 'R', or 'N'
} JoystickData;

JoystickData joystickData;
esp_now_peer_info_t peerInfo;

float normalizeJoystick(int raw, int center) {
  float value;

  if (raw >= center) {
    value = (float)(raw - center) / (4095 - center);
  } else {
    value = (float)(raw - center) / center;
  }

  // Clamp to exact range
  if (value > 1.0) value = 1.0;
  if (value < -1.0) value = -1.0;

  // Deadzone around idle
  if (abs(value) < DEADZONE) {
    value = 0.0;
  }

  return value;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ADC setup
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Button setup
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);

  // ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("ESP-NOW Transmitter Started");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add receiver peer");
    return;
  }

  Serial.println("Receiver peer added");
}

void loop() {
  int xRaw = analogRead(JOYSTICK_X_PIN);
  int yRaw = analogRead(JOYSTICK_Y_PIN);

  joystickData.xValue = normalizeJoystick(xRaw, X_CENTER);
  joystickData.yValue = normalizeJoystick(yRaw, Y_CENTER);

  bool leftPressed = digitalRead(LEFT_BUTTON_PIN) == LOW;
  bool rightPressed = digitalRead(RIGHT_BUTTON_PIN) == LOW;

  if (leftPressed) {
    joystickData.button = 'L';
  } 
  else if (rightPressed) {
    joystickData.button = 'R';
  } 
  else {
    joystickData.button = 'N';
  }

  esp_err_t result = esp_now_send(
    receiverMacAddress,
    (uint8_t *) &joystickData,
    sizeof(joystickData)
  );

  // Optional local debug print on transmitter
  Serial.print("Sent: ");
  Serial.print(joystickData.xValue, 3);
  Serial.print(",");
  Serial.print(joystickData.yValue, 3);
  Serial.print(",");
  Serial.print(joystickData.button);
  Serial.print(" | ");

  if (result == ESP_OK) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  delay(50);
}