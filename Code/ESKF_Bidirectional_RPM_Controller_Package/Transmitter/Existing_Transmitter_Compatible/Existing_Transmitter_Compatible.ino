#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

// ---------------- PIN SETUP ----------------

#define THROTTLE_PIN        35
#define THROTTLE_SW_PIN     27

#define ROLL_PIN            34
#define PITCH_PIN           32

#define KILL_BUTTON_PIN     13
#define ONOFF_BUTTON_PIN    14

#define ON_LED_PIN          22
#define OFF_LED_PIN         21

// ---------------- JOYSTICK CALIBRATION ----------------
//
// Throttle is centered:
//   -1.0 = low throttle / descend / motor-stop side
//    0.0 = hover command
//   +1.0 = above-hover / climb side
//
// With THROTTLE_SIGN = -1.0:
//   raw near 0    -> CMD T:+1.000
//   raw near 1902 -> CMD T: 0.000
//   raw near 4095 -> CMD T:-1.000

int throttleMin    = 0;
int throttleCenter = 1936;
int throttleMax    = 4095;

int rollMin        = 0;
int rollCenter     = 1884;
int rollMax        = 4095;

int pitchMin       = 0;
int pitchCenter    = 1919;
int pitchMax       = 4095;

#define DEADZONE 0.10f
#define RUN_FULL_STICK_CALIBRATION 1

// ---------------- AXIS DIRECTIONS ----------------

#define THROTTLE_SIGN  -1.0f
#define PITCH_SIGN     -1.0f
#define ROLL_SIGN      -1.0f

#define YAW_COMMAND     0.0f

// ---------------- ESP-NOW RECEIVER MAC ----------------

uint8_t receiverMacAddress[] = {0x5C, 0x01, 0x3B, 0xBA, 0xCA, 0xEC};

// ---------------- DATA PACKET ----------------
//
// IMPORTANT:
// Receiver must use this exact same struct.

#define PACKET_MAGIC 0x44524F4EUL  // "DRON"

typedef struct ControlData {
  uint32_t magic;
  uint32_t seq;
  float throttle;      // -1.0 to +1.0, where 0.0 is hover command
  float pitch;         // -1.0 to +1.0
  float roll;          // -1.0 to +1.0
  float yaw;           // -1.0 to +1.0
  bool motorsEnabled;
  bool kill;
} ControlData;

ControlData controlData;
esp_now_peer_info_t peerInfo;

uint32_t packetSeq = 0;

// ---------------- STATE ----------------

bool motorsEnabled = false;
bool killActive = false;

bool prevOnOffPressed = false;
bool prevKillPressed = false;

volatile esp_now_send_status_t lastSendStatus = ESP_NOW_SEND_FAIL;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_PERIOD_MS = 50;

unsigned long lastSendMs = 0;
const unsigned long SEND_PERIOD_MS = 20;

unsigned long lastBlinkMs = 0;
const unsigned long KILL_BLINK_PERIOD_MS = 250;
bool offLedBlinkState = false;

// ---------------- HELPER FUNCTIONS ----------------

float clampFloat(float value, float low, float high)
{
  if (value > high) return high;
  if (value < low) return low;
  return value;
}

float applyDeadzoneAndRescale(float value)
{
  if (fabsf(value) < DEADZONE)
  {
    return 0.0f;
  }

  if (value > 0.0f)
  {
    value = (value - DEADZONE) / (1.0f - DEADZONE);
  }
  else
  {
    value = (value + DEADZONE) / (1.0f - DEADZONE);
  }

  return clampFloat(value, -1.0f, 1.0f);
}

float normalizeCalibratedJoystick(int raw, int minVal, int centerVal, int maxVal)
{
  float value;

  if (centerVal <= minVal + 10 || centerVal >= maxVal - 10)
  {
    return 0.0f;
  }

  if (raw >= centerVal)
  {
    value = (float)(raw - centerVal) / (float)(maxVal - centerVal);
  }
  else
  {
    value = (float)(raw - centerVal) / (float)(centerVal - minVal);
  }

  value = clampFloat(value, -1.0f, 1.0f);
  value = applyDeadzoneAndRescale(value);

  return value;
}

float normalizeCalibratedThrottle(int raw, int minVal, int centerVal, int maxVal)
{
  float value = normalizeCalibratedJoystick(raw, minVal, centerVal, maxVal);

  value *= THROTTLE_SIGN;

  return clampFloat(value, -1.0f, 1.0f);
}

void safeOffCommands()
{
  // Important:
  // throttle 0.0 now means hover command.
  // OFF/KILL sends -1.0 so receiver sees low throttle.
  controlData.throttle = -1.0f;
  controlData.pitch = 0.0f;
  controlData.roll = 0.0f;
  controlData.yaw = 0.0f;
}

// ---------------- JOYSTICK CALIBRATION ----------------

void calibrateSticksFullRange()
{
  Serial.println();
  Serial.println("FULL JOYSTICK CALIBRATION");
  Serial.println();
  Serial.println("Step 1:");
  Serial.println("Leave throttle, pitch, and roll sticks centered.");
  Serial.println("Do not touch the sticks.");
  Serial.println("Center calibration begins in 2 seconds...");
  delay(2000);

  const int centerSamples = 800;

  long throttleSum = 0;
  long rollSum = 0;
  long pitchSum = 0;

  int throttleCenterMin = 4095;
  int throttleCenterMax = 0;

  int rollCenterMin = 4095;
  int rollCenterMax = 0;

  int pitchCenterMin = 4095;
  int pitchCenterMax = 0;

  for (int i = 0; i < centerSamples; i++)
  {
    int t = analogRead(THROTTLE_PIN);
    int r = analogRead(ROLL_PIN);
    int p = analogRead(PITCH_PIN);

    throttleSum += t;
    rollSum += r;
    pitchSum += p;

    if (t < throttleCenterMin) throttleCenterMin = t;
    if (t > throttleCenterMax) throttleCenterMax = t;

    if (r < rollCenterMin) rollCenterMin = r;
    if (r > rollCenterMax) rollCenterMax = r;

    if (p < pitchCenterMin) pitchCenterMin = p;
    if (p > pitchCenterMax) pitchCenterMax = p;

    delay(2);
  }

  throttleCenter = throttleSum / centerSamples;
  rollCenter     = rollSum / centerSamples;
  pitchCenter    = pitchSum / centerSamples;

  throttleMin = throttleCenter;
  throttleMax = throttleCenter;

  rollMin = rollCenter;
  rollMax = rollCenter;

  pitchMin = pitchCenter;
  pitchMax = pitchCenter;

  Serial.println();
  Serial.println("Center captured.");
  Serial.print("Throttle center: "); Serial.print(throttleCenter);
  Serial.print("  center noise: "); Serial.print(throttleCenterMin);
  Serial.print(" to "); Serial.println(throttleCenterMax);

  Serial.print("Roll center:     "); Serial.print(rollCenter);
  Serial.print("  center noise: "); Serial.print(rollCenterMin);
  Serial.print(" to "); Serial.println(rollCenterMax);

  Serial.print("Pitch center:    "); Serial.print(pitchCenter);
  Serial.print("  center noise: "); Serial.print(pitchCenterMin);
  Serial.print(" to "); Serial.println(pitchCenterMax);

  Serial.println();
  Serial.println("Step 2:");
  Serial.println("Move all sticks through their FULL ranges.");
  Serial.println("Move throttle full down and full up.");
  Serial.println("Move roll full left and full right.");
  Serial.println("Move pitch full forward and full back.");
  Serial.println("Keep moving for 6 seconds...");
  delay(1000);

  unsigned long startMs = millis();

  while (millis() - startMs < 6000)
  {
    int t = analogRead(THROTTLE_PIN);
    int r = analogRead(ROLL_PIN);
    int p = analogRead(PITCH_PIN);

    if (t < throttleMin) throttleMin = t;
    if (t > throttleMax) throttleMax = t;

    if (r < rollMin) rollMin = r;
    if (r > rollMax) rollMax = r;

    if (p < pitchMin) pitchMin = p;
    if (p > pitchMax) pitchMax = p;

    delay(2);
  }

  Serial.println();
  Serial.println("Full calibration complete.");
  Serial.println();
  Serial.println("COPY THESE VALUES INTO THE TOP OF THE CODE:");
  Serial.println();

  Serial.print("int throttleMin    = "); Serial.print(throttleMin); Serial.println(";");
  Serial.print("int throttleCenter = "); Serial.print(throttleCenter); Serial.println(";");
  Serial.print("int throttleMax    = "); Serial.print(throttleMax); Serial.println(";");

  Serial.println();

  Serial.print("int rollMin        = "); Serial.print(rollMin); Serial.println(";");
  Serial.print("int rollCenter     = "); Serial.print(rollCenter); Serial.println(";");
  Serial.print("int rollMax        = "); Serial.print(rollMax); Serial.println(";");

  Serial.println();

  Serial.print("int pitchMin       = "); Serial.print(pitchMin); Serial.println(";");
  Serial.print("int pitchCenter    = "); Serial.print(pitchCenter); Serial.println(";");
  Serial.print("int pitchMax       = "); Serial.print(pitchMax); Serial.println(";");

  Serial.println();
  Serial.println("Then set:");
  Serial.println("#define RUN_FULL_STICK_CALIBRATION 0");
  Serial.println();
}

void printSavedCalibration()
{
  Serial.println();
  Serial.println("Using saved joystick calibration values.");

  Serial.print("Throttle min/center/max: ");
  Serial.print(throttleMin);
  Serial.print(" / ");
  Serial.print(throttleCenter);
  Serial.print(" / ");
  Serial.println(throttleMax);

  Serial.print("Roll min/center/max: ");
  Serial.print(rollMin);
  Serial.print(" / ");
  Serial.print(rollCenter);
  Serial.print(" / ");
  Serial.println(rollMax);

  Serial.print("Pitch min/center/max: ");
  Serial.print(pitchMin);
  Serial.print(" / ");
  Serial.print(pitchCenter);
  Serial.print(" / ");
  Serial.println(pitchMax);

  Serial.println();
}

// ---------------- BUTTON AND LED LOGIC ----------------

void updateButtons()
{
  bool onOffPressed = digitalRead(ONOFF_BUTTON_PIN) == LOW;
  bool killPressed  = digitalRead(KILL_BUTTON_PIN) == LOW;

  bool onOffJustPressed = onOffPressed && !prevOnOffPressed;
  bool killJustPressed  = killPressed && !prevKillPressed;

  if (killJustPressed)
  {
    killActive = true;
    motorsEnabled = false;
  }

  if (onOffJustPressed)
  {
    if (killActive)
    {
      killActive = false;
      motorsEnabled = true;
    }
    else
    {
      motorsEnabled = !motorsEnabled;
    }
  }

  prevOnOffPressed = onOffPressed;
  prevKillPressed = killPressed;
}

void updateLEDs()
{
  unsigned long nowMs = millis();

  if (killActive)
  {
    digitalWrite(ON_LED_PIN, LOW);

    if (nowMs - lastBlinkMs >= KILL_BLINK_PERIOD_MS)
    {
      lastBlinkMs = nowMs;
      offLedBlinkState = !offLedBlinkState;
    }

    digitalWrite(OFF_LED_PIN, offLedBlinkState ? HIGH : LOW);
  }
  else if (motorsEnabled)
  {
    digitalWrite(ON_LED_PIN, HIGH);
    digitalWrite(OFF_LED_PIN, LOW);
  }
  else
  {
    digitalWrite(ON_LED_PIN, LOW);
    digitalWrite(OFF_LED_PIN, HIGH);
  }
}

// ---------------- DEBUG PRINT ----------------

void printDebug(
  int throttleRaw,
  int pitchRaw,
  int rollRaw,
  bool killPressed,
  bool onOffPressed,
  esp_err_t sendResult,
  esp_now_send_status_t lastStatus
)
{
  Serial.print("RAW ");
  Serial.print("T:");
  Serial.print(throttleRaw);
  Serial.print(",P:");
  Serial.print(pitchRaw);
  Serial.print(",R:");
  Serial.print(rollRaw);

  Serial.print(" | CTR ");
  Serial.print("T:");
  Serial.print(throttleCenter);
  Serial.print(",P:");
  Serial.print(pitchCenter);
  Serial.print(",R:");
  Serial.print(rollCenter);

  Serial.print(" | MINMAX ");
  Serial.print("T:");
  Serial.print(throttleMin);
  Serial.print("/");
  Serial.print(throttleMax);
  Serial.print(",P:");
  Serial.print(pitchMin);
  Serial.print("/");
  Serial.print(pitchMax);
  Serial.print(",R:");
  Serial.print(rollMin);
  Serial.print("/");
  Serial.print(rollMax);

  Serial.print(" | BTN ");
  Serial.print("KILL:");
  Serial.print(killPressed ? "PRESSED" : "off");
  Serial.print(",ONOFF:");
  Serial.print(onOffPressed ? "PRESSED" : "off");

  Serial.print(" | STATE ");
  Serial.print("EN:");
  Serial.print(controlData.motorsEnabled ? 1 : 0);
  Serial.print(",KILL:");
  Serial.print(controlData.kill ? 1 : 0);

  Serial.print(" | SEQ:");
  Serial.print(controlData.seq);

  Serial.print(" | CMD ");
  Serial.print("T:");
  Serial.print(controlData.throttle, 3);
  Serial.print(",P:");
  Serial.print(controlData.pitch, 3);
  Serial.print(",R:");
  Serial.print(controlData.roll, 3);
  Serial.print(",Y:");
  Serial.print(controlData.yaw, 3);

  Serial.print(" | SEND_CALL:");
  Serial.print(sendResult == ESP_OK ? "OK" : "FAIL");

  Serial.print(" | DELIVERY:");
  Serial.println(lastStatus == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// ---------------- ESP-NOW SEND CALLBACK ----------------

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
  lastSendStatus = status;
}

// ---------------- SETUP ----------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(THROTTLE_SW_PIN, INPUT_PULLUP);
  pinMode(KILL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ONOFF_BUTTON_PIN, INPUT_PULLUP);

  pinMode(ON_LED_PIN, OUTPUT);
  pinMode(OFF_LED_PIN, OUTPUT);

  digitalWrite(ON_LED_PIN, LOW);
  digitalWrite(OFF_LED_PIN, HIGH);

  if (RUN_FULL_STICK_CALIBRATION)
  {
    calibrateSticksFullRange();
  }
  else
  {
    printSavedCalibration();
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(300);

  Serial.println();
  Serial.println("ESP-NOW Transmitter Started");
  Serial.print("Transmitter MAC Address: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Receiver MAC Address: ");
  Serial.println("5C:01:3B:BA:CA:EC");

  Serial.print("Packet size = ");
  Serial.println(sizeof(ControlData));

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");

    while (true)
    {
      digitalWrite(ON_LED_PIN, LOW);
      digitalWrite(OFF_LED_PIN, HIGH);
      delay(100);
    }
  }

  esp_now_register_send_cb(onDataSent);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add receiver peer");

    while (true)
    {
      digitalWrite(ON_LED_PIN, LOW);
      digitalWrite(OFF_LED_PIN, HIGH);
      delay(100);
    }
  }

  motorsEnabled = false;
  killActive = false;

  safeOffCommands();
  controlData.magic = PACKET_MAGIC;
  controlData.seq = 0;
  controlData.motorsEnabled = false;
  controlData.kill = false;

  Serial.println("Receiver peer added");
  Serial.println("Starting normal OFF.");
  Serial.println();
}

// ---------------- LOOP ----------------

void loop()
{
  unsigned long nowMs = millis();

  updateButtons();
  updateLEDs();

  if (nowMs - lastSendMs >= SEND_PERIOD_MS)
  {
    lastSendMs = nowMs;

    int throttleRaw = analogRead(THROTTLE_PIN);
    int rollRaw     = analogRead(ROLL_PIN);
    int pitchRaw    = analogRead(PITCH_PIN);

    float throttleCmd = normalizeCalibratedThrottle(
      throttleRaw,
      throttleMin,
      throttleCenter,
      throttleMax
    );

    float rollCmd = normalizeCalibratedJoystick(
      rollRaw,
      rollMin,
      rollCenter,
      rollMax
    );
    rollCmd *= ROLL_SIGN;
    rollCmd = clampFloat(rollCmd, -1.0f, 1.0f);

    float pitchCmd = normalizeCalibratedJoystick(
      pitchRaw,
      pitchMin,
      pitchCenter,
      pitchMax
    );
    pitchCmd *= PITCH_SIGN;
    pitchCmd = clampFloat(pitchCmd, -1.0f, 1.0f);

    controlData.magic = PACKET_MAGIC;
    controlData.seq = packetSeq++;

    controlData.motorsEnabled = motorsEnabled;
    controlData.kill = killActive;

    if (motorsEnabled && !killActive)
    {
      controlData.throttle = throttleCmd;
      controlData.pitch = pitchCmd;
      controlData.roll = rollCmd;
      controlData.yaw = YAW_COMMAND;
    }
    else
    {
      safeOffCommands();
    }

    esp_err_t result = esp_now_send(
      receiverMacAddress,
      (uint8_t *)&controlData,
      sizeof(controlData)
    );

    if (nowMs - lastPrintMs >= PRINT_PERIOD_MS)
    {
      lastPrintMs = nowMs;

      bool killPressed = digitalRead(KILL_BUTTON_PIN) == LOW;
      bool onOffPressed = digitalRead(ONOFF_BUTTON_PIN) == LOW;

      esp_now_send_status_t statusCopy = lastSendStatus;

      printDebug(
        throttleRaw,
        pitchRaw,
        rollRaw,
        killPressed,
        onOffPressed,
        result,
        statusCopy
      );
    }
  }
}