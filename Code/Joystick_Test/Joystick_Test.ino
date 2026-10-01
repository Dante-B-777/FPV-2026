// ESP32 Joystick Reader
// Reads X and Y analog joystick values and two joystick buttons over USB Serial

#define JOYSTICK_X_PIN 34
#define JOYSTICK_Y_PIN 35

#define LEFT_BUTTON_PIN 26   // D26
#define RIGHT_BUTTON_PIN 27  // D27

void setup() {
  Serial.begin(115200);

  // ESP32 ADC setup
  analogReadResolution(12);
  // 12-bit ADC gives values from 0 to 4095

  analogSetAttenuation(ADC_11db);
  // Allows reading close to full 0V to 3.3V range

  // Button setup
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  // With INPUT_PULLUP:
  // not pressed = HIGH
  // pressed = LOW

  delay(1000);
  Serial.println("Joystick reader started");
  Serial.println("X,Y,Button");
}

void loop() {
  int xValue = analogRead(JOYSTICK_X_PIN);
  int yValue = analogRead(JOYSTICK_Y_PIN);

  bool leftPressed = digitalRead(LEFT_BUTTON_PIN) == LOW;
  bool rightPressed = digitalRead(RIGHT_BUTTON_PIN) == LOW;

  Serial.print(xValue);
  Serial.print(",");
  Serial.print(yValue);
  Serial.print(",");

  if (leftPressed) {
    Serial.println("L");
  } 
  else if (rightPressed) {
    Serial.println("R");
  } 
  else {
    Serial.println("N"); // N = no button pressed
  }

  delay(50);
  // 50 ms = about 20 readings per second
}