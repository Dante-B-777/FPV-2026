/*
  ESP32 ESP-NOW Receiver + 4-Motor DShot Throttle Control

  Motor pins:
    Motor 1 -> GPIO25
    Motor 2 -> GPIO26
    Motor 3 -> GPIO27
    Motor 4 -> GPIO14

  Joystick mapping:
    yValue <= 0.0  -> stop
    yValue  0..1   -> DShot throttle 48..500

  REMOVE PROPS BEFORE TESTING.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <DShotRMT.h>

// -------------------- DShot Motor Setup --------------------

static constexpr gpio_num_t MOTOR1_PIN = GPIO_NUM_25;
static constexpr gpio_num_t MOTOR2_PIN = GPIO_NUM_26;
static constexpr gpio_num_t MOTOR3_PIN = GPIO_NUM_27;
static constexpr gpio_num_t MOTOR4_PIN = GPIO_NUM_14;

static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;
static constexpr bool IS_BIDIRECTIONAL = false;

DShotRMT motor1(MOTOR1_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor2(MOTOR2_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor3(MOTOR3_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);
DShotRMT motor4(MOTOR4_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

// DShot throttle values
static constexpr uint16_t DSHOT_STOP = DSHOT_CMD_MOTOR_STOP;
static constexpr uint16_t DSHOT_MIN_THROTTLE = 48;
static constexpr uint16_t DSHOT_MAX_THROTTLE = 500;

// Current throttle command sent to all motors
volatile uint16_t targetThrottle = DSHOT_STOP;

// Failsafe
volatile bool packetReceived = false;
volatile unsigned long lastPacketTimeMs = 0;
static constexpr unsigned long FAILSAFE_TIMEOUT_MS = 500;

// -------------------- ESP-NOW Data Struct --------------------

typedef struct JoystickData {
  float xValue;   // -1.0 to +1.0
  float yValue;   // -1.0 to +1.0
  char button;    // 'L', 'R', or 'N'
} JoystickData;

JoystickData incomingData;

// -------------------- Helper Functions --------------------

float clampFloat(float value, float minVal, float maxVal)
{
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

uint16_t joystickToDshot(float y)
{
  if (y <= 0.0f)
  {
    return DSHOT_STOP;
  }

  y = clampFloat(y, 0.0f, 1.0f);

  float mapped = DSHOT_MIN_THROTTLE +
                 y * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE);

  return (uint16_t)(mapped + 0.5f);
}

void sendAllMotors(uint16_t value)
{
  motor1.sendThrottle(value);
  motor2.sendThrottle(value);
  motor3.sendThrottle(value);
  motor4.sendThrottle(value);
}

void sendAllStopForMs(uint32_t ms)
{
  uint32_t start = millis();

  while (millis() - start < ms)
  {
    sendAllMotors(DSHOT_STOP);
    delay(2);
  }
}

// -------------------- ESP-NOW Callback --------------------

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (len != sizeof(JoystickData))
  {
    Serial.print("Bad packet length: ");
    Serial.println(len);
    return;
  }

  memcpy(&incomingData, data, sizeof(incomingData));

  uint16_t newThrottle = joystickToDshot(incomingData.yValue);

  targetThrottle = newThrottle;
  packetReceived = true;
  lastPacketTimeMs = millis();

  Serial.print("X=");
  Serial.print(incomingData.xValue, 3);

  Serial.print(" Y=");
  Serial.print(incomingData.yValue, 3);

  Serial.print(" Button=");
  Serial.print(incomingData.button);

  Serial.print(" DShot throttle=");
  Serial.println(newThrottle);
}

// -------------------- Setup --------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP-NOW Receiver + 4-Motor DShot Throttle Control");
  Serial.println("REMOVE PROPS.");
  Serial.println();

  Serial.println("Motor mapping:");
  Serial.println("  Motor 1 -> GPIO25");
  Serial.println("  Motor 2 -> GPIO26");
  Serial.println("  Motor 3 -> GPIO27");
  Serial.println("  Motor 4 -> GPIO14");
  Serial.println();

  Serial.println("Throttle mapping:");
  Serial.println("  yValue <= 0.0 -> stop");
  Serial.println("  yValue 0..1   -> DShot 48..500");
  Serial.println();

  motor1.begin();
  motor2.begin();
  motor3.begin();
  motor4.begin();

  Serial.println("Sending DShot stop for 5 seconds.");
  Serial.println("Power ESC now if not already powered.");
  sendAllStopForMs(5000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("Receiver MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");

    while (true)
    {
      sendAllMotors(DSHOT_STOP);
      delay(2);
    }
  }

  esp_now_register_recv_cb(onDataReceive);

  Serial.println("ESP-NOW Receiver Started");
  Serial.println("Waiting for joystick throttle data...");
}

// -------------------- Main Loop --------------------

void loop()
{
  unsigned long now = millis();

  if (!packetReceived || (now - lastPacketTimeMs > FAILSAFE_TIMEOUT_MS))
  {
    targetThrottle = DSHOT_STOP;
  }

  sendAllMotors(targetThrottle);

  delay(2);
}