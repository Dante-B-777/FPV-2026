#include <Arduino.h>
#include <DShotRMT.h>

static constexpr auto &USB_SERIAL = Serial;
static constexpr auto USB_SERIAL_BAUD = 115200;

static constexpr gpio_num_t ESC_PIN = GPIO_NUM_27;

// Try DSHOT300 first. If nothing, try DSHOT150.
static constexpr dshot_mode_t DSHOT_MODE = DSHOT300;

// IMPORTANT: keep this false for first spin/receive tests.
static constexpr bool IS_BIDIRECTIONAL = false;

DShotRMT esc(ESC_PIN, DSHOT_MODE, IS_BIDIRECTIONAL);

uint16_t throttle = DSHOT_CMD_MOTOR_STOP;

void sendStopForMs(uint32_t ms)
{
  uint32_t start = millis();
  while (millis() - start < ms)
  {
    esc.sendThrottle(DSHOT_CMD_MOTOR_STOP);
    delay(2);
  }
}

void sendThrottleForMs(uint16_t value, uint32_t ms)
{
  uint32_t start = millis();

  while (millis() - start < ms)
  {
    esc.sendThrottle(value);
    delay(2);
  }
}

void sendBeaconCommand1()
{
  USB_SERIAL.println("Sending Bluejay beacon command 1...");

  // Send command 1 multiple times.
  // Bluejay command 1 should trigger a beacon beep.
  for (int i = 0; i < 50; i++)
  {
    dshot_result_t result = esc.sendCommand(1);
    printDShotResult(result);
    delay(20);
  }

  USB_SERIAL.println("Done sending command 1.");
}

void printMenu()
{
  USB_SERIAL.println();
  USB_SERIAL.println("Bluejay / DShotRMT ESC Test");
  USB_SERIAL.println("REMOVE PROPS.");
  USB_SERIAL.println("--------------------------------");
  USB_SERIAL.println("b     = send Bluejay beacon command 1");
  USB_SERIAL.println("0     = stop");
  USB_SERIAL.println("100   = throttle 100");
  USB_SERIAL.println("300   = throttle 300");
  USB_SERIAL.println("600   = throttle 600");
  USB_SERIAL.println("1000  = throttle 1000");
  USB_SERIAL.println("r     = ramp 48 to 1000");
  USB_SERIAL.println("--------------------------------");
  USB_SERIAL.println("Use Serial Monitor set to 115200 baud and Newline.");
  USB_SERIAL.println();
}

void setup()
{
  USB_SERIAL.begin(USB_SERIAL_BAUD);
  delay(1000);

  esc.begin();

  printMenu();

  USB_SERIAL.println("Sending stop for 5 seconds.");
  USB_SERIAL.println("Power ESC now if not already powered.");
  sendStopForMs(5000);

  USB_SERIAL.println("Ready.");
}

void loop()
{
  // Always keep sending current throttle continuously.
  esc.sendThrottle(throttle);
  delay(2);

  if (USB_SERIAL.available())
  {
    String input = USB_SERIAL.readStringUntil('\n');
    input.trim();

    if (input == "b")
    {
      throttle = DSHOT_CMD_MOTOR_STOP;
      sendStopForMs(500);
      sendBeaconCommand1();
      sendStopForMs(1000);
    }
    else if (input == "0")
    {
      throttle = DSHOT_CMD_MOTOR_STOP;
      USB_SERIAL.println("Stop.");
    }
    else if (input == "100")
    {
      throttle = 100;
      USB_SERIAL.println("Throttle 100.");
    }
    else if (input == "300")
    {
      throttle = 300;
      USB_SERIAL.println("Throttle 300.");
    }
    else if (input == "600")
    {
      throttle = 600;
      USB_SERIAL.println("Throttle 600.");
    }
    else if (input == "1000")
    {
      throttle = 1000;
      USB_SERIAL.println("Throttle 1000.");
    }
    else if (input == "r")
    {
      USB_SERIAL.println("Ramping 48 to 1000.");

      for (uint16_t value = 48; value <= 200; value += 20)
      {
        throttle = value;
        USB_SERIAL.print("Throttle ");
        USB_SERIAL.println(throttle);
        sendThrottleForMs(throttle, 250);
      }

      USB_SERIAL.println("Ramp done. Stopping.");
      throttle = DSHOT_CMD_MOTOR_STOP;
      sendStopForMs(1000);
    }
    else if (input == "h" || input == "help")
    {
      printMenu();
    }
    else
    {
      USB_SERIAL.println("Unknown command.");
      printMenu();
    }
  }
}