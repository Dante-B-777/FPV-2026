# Open-Source Reference Implementations Used

These are mature implementations to compare against, not blind drop-in files.
The included robust controller ports the relevant architecture to the supplied
ESP32 pins, estimator, mixer, DShot service, and telemetry format.

## Bitcraze Crazyflie firmware

Repository: `bitcraze/crazyflie-firmware`

Relevant areas:

- Flow-deck PMW3901 driver, commonly named `flowdeck_v1v2.c`.
  - reads a full motion burst;
  - checks the PMW motion marker;
  - maps the mounted sensor axes into the vehicle frame;
  - derives measurement uncertainty from shutter/exposure.
- Kalman optical-flow measurement model, commonly named
  `kalman_core_mm_flow.c`.
  - predicts raw pixel motion from body velocity, height, angular rate, and
    camera offset instead of treating raw counts as perfect velocity.

## Espressif ESP-Drone

Repository: `espressif/esp-drone`

Relevant areas:

- Flow-deck/PMW3901 driver under the deck-driver components.
  - useful because it is an ESP32 port of the Crazyflie flow-deck stack;
  - reads the 12-byte PMW motion burst from register `0x16`;
  - uses the same quality-aware sensor path.
- Cascaded position/velocity PID controller, commonly named
  `controller_pid.c` or an equivalent controller module.
  - separates position-to-velocity and velocity-to-attitude stages;
  - applies limits at each stage.

ESP-Drone is the closest already-coded full reference for this processor, but
it assumes the ESP-Drone state estimator, motor abstraction, deck framework,
and configuration system. Replacing only one file in the current project with
ESP-Drone code would not compile or preserve the present motor mapping.

## PX4 Autopilot

Repository: `PX4/PX4-Autopilot`

Relevant areas:

- PMW3901 optical-flow driver.
  - rejects poor-quality samples;
  - uses a calibrated count scale near `1/385 rad/count`;
  - rotates sensor flow using a configured mounting orientation.
- Optical-flow sensor configuration.
  - requires the sensor rotation and physical offset to be configured rather
    than guessed.

## Why the basic Arduino PMW3901 library is not enough by itself

The Bitcraze Arduino PMW3901 API used by the old sketch exposes motion counts
but does not, through the basic `readMotionCount()` call, supply the complete
quality path needed by a flight-critical hold. The diagnostic and robust
controller therefore perform a direct full-burst SPI read after the library has
initialized the sensor.
