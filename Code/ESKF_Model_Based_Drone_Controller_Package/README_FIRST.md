# ESP32 ESKF + Model-Based Quadrotor Controller

## Read this before powering the motors

**Remove all propellers for the first tests.** Verify sensor signs, estimator angles, motor numbering, motor direction, and mixer response with no propellers. Then use a restrained/tethered test stand. Do not begin with a free-flight test.

This package is a receiver-side replacement for the existing controller. The existing transmitter packet, command signs, enable switch, kill behavior, and receiver MAC remain unchanged.

## Package layout

```text
Receiver/
  ESKF_Model_Based_Drone_Controller/
    ESKF_Model_Based_Drone_Controller.ino
    Eskf9.h
    MotionModelFF.h
Transmitter/
  Existing_Transmitter_Compatible/
    Existing_Transmitter_Compatible.ino
Validation/
  run_host_validation.sh
  VALIDATION_REPORT.txt
  VALIDATION_OUTPUT.txt
  test_eskf.cpp
  test_motion_model.cpp
  test_signs.cpp
  Host_Stubs/
```

Open `Receiver/ESKF_Model_Based_Drone_Controller/ESKF_Model_Based_Drone_Controller.ino` in Arduino IDE. Keep the two header files in the same sketch folder.

The transmitter copy is included only to make the matching packet explicit. It is the supplied transmitter code, unchanged. Do not place the transmitter sketch in the receiver sketch folder because Arduino IDE compiles every `.ino` file in one folder together.

## Arduino IDE target and libraries

Board target: **ESP32 Dev Module / ESP32 Dev Kit 1**.

Required libraries are the same ones used by the supplied working receiver:

- `DShotRMT`
- Bitcraze `PMW3901`
- Pololu `VL53L0X`

The estimator uses fixed-size arrays and therefore does **not** require Eigen.

## Preserved hardware and protocol

### Motors and DShot

Viewed from above:

```text
                    FRONT

              M1             M2
             GPIO26         GPIO14
               CCW            CW

              M4             M3
             GPIO25         GPIO27
                CW           CCW
```

- DShot300
- Non-bidirectional DShot
- Startup ramp: DShot 200 to 505 over 2 seconds
- Normal manual range: DShot 350 to 600
- Shutdown ramp: current base to DShot 200 over 2 seconds, then motor stop

The validated feedback mixer is preserved:

```text
M1 = base - pitch - roll + yaw
M2 = base - pitch + roll - yaw
M3 = base + pitch + roll + yaw
M4 = base + pitch - roll - yaw
```

The model feed-forward is added after this feedback calculation as a small, bounded per-motor term.

### Sensors

- MPU6050: SDA GPIO22, SCL GPIO21, address `0x68`, mounted upside down
- VL53L0X: shared I2C bus, address `0x29`
- PMW3901: SCK 18, MISO 19, MOSI 23, CS 13, reset 33

Preserved installed-axis signs:

```text
ROLL_GYRO_SIGN  = -1
PITCH_GYRO_SIGN = +1
YAW_GYRO_SIGN   = +1   // current controller convention: positive clockwise
FLOW_X_DIRECTION = +1
FLOW_Y_DIRECTION = -1
```

The ESKF itself uses a right-handed body frame (`+X` right, `+Y` forward,
`+Z` up). The proven controller/mixer convention is mapped explicitly as:

```text
controller pitch = - right-handed rotation about body +X
controller roll  = - right-handed rotation about body +Y
clockwise yaw    = - right-handed rotation about body +Z
```

This conversion is isolated in constants near the top of the receiver sketch.
It does not change the transmitter signs, the displayed controller angles, or
the established feedback mixer.

### ESP-NOW packet

The receiver uses the exact supplied transmitter packet:

```cpp
struct ControlData {
  uint32_t magic;
  uint32_t seq;
  float throttle;
  float pitch;
  float roll;
  float yaw;
  bool motorsEnabled;
  bool kill;
};
```

`PACKET_MAGIC` remains `0x44524F4EUL`, and packet timeout remains 300 ms.

## Controller architecture

```text
MPU6050, 250 Hz
  -> calibrated specific force and body rates
  -> ESKF prediction
  -> ESKF quaternion / position / velocity

PMW3901, 50 Hz
  -> existing count signs, scale, gyro compensation, and lever-arm correction
  -> body-velocity Kalman update

VL53L0X, approximately 50 Hz
  -> existing non-blocking data-ready polling and range filtering
  -> altitude and filtered vertical-velocity Kalman updates

Pilot commands
  -> existing X/Y position-velocity outer loop and Z position-velocity loop
  -> desired roll, pitch, yaw rate, and vertical velocity
  -> supplied Python inverse-dynamics model
  -> bounded rate feed-forward plus bounded motor feed-forward
  -> preserved angle P loop
  -> preserved roll/pitch/yaw rate PID
  -> preserved motor mixer
  -> DShot300
```

## How each reference was used

### 1. `koledickarlo/ESKF-ESP32`

The repository provides the estimator architecture, not a complete flight controller. This implementation adopts its core sequence:

1. Propagate nominal position, velocity, and quaternion from IMU data.
2. Propagate a 9-by-9 error covariance.
3. Apply asynchronous optical-flow and ToF updates when those measurements arrive.
4. Inject the position, velocity, and small-angle error correction into the nominal state.
5. Reset the attitude error and rotate/symmetrize the covariance.

`Eskf9.h` implements that pattern with fixed-size C++ arrays instead of Eigen. It adds innovation gating, a Joseph-form covariance update, a gravity-direction attitude correction, and a complementary-filter cross-check. The existing complementary angle estimate remains an independent fallback; X/Y hold is disabled when the ESKF or flow update is not trustworthy.

### 2. Supplied Python motion model

`MotionModelFF.h` ports the supplied operation loop and physical constants:

- Mass: 1 lb = 0.45359237 kg
- Frame dimensions: 0.1645 m front-to-back and 0.1630 m side-to-side
- Propeller radius: 2.5 in = 0.0635 m
- Air density: 1.225 kg/m^3
- Thrust coefficient input: `CT = 0.18`
- Hover rotor speed: 874.8454591 rad/s
- Rotor inertia expression and `c = 4/pi^2 * CT * rho * r^4`
- The four-element `C` thrust allocation vector
- The scalar `lambda` solution used to redistribute counter-rotating rotor speeds
- `omega_i = sqrt(thrust_i / c)`

Embedded changes are required because the original script used a one-second time step and SciPy:

- Actual measured control-loop `dt` is used.
- Stick/angle commands are low-pass filtered and slew limited.
- Angular and vertical accelerations are bounded.
- A bounded bisection solve replaces `scipy.optimize.brentq`.
- Pitch/roll signs are mapped to the already-validated motor mixer.
- Predicted rotor speed is converted to DShot with a local hover approximation.

The model is used as **feed-forward**, not as the only controller. Default blends are intentionally low:

```text
motionModelMotorBlend = 0.12
motionModelRateBlend  = 0.20
```

For commissioning safety, both new actuator-facing paths start disabled after
every boot:

```text
ESKF attitude replacement: OFF
motion-model feed-forward: OFF
```

The ESKF still runs continuously and supplies X/Y/Z position and velocity.
After the propeller-off sign checks, use `eatt on` to let the ESKF quaternion
replace the complementary roll/pitch estimate, and use `model on` to apply the
bounded model feed-forward. Both ON commands are rejected while the motors are
enabled, preventing an in-flight step change.

### 3. Supplied current receiver and transmitter

The current scripts supplied all airframe-specific information that should not be guessed:

- Motor pins, order, rotation directions, DShot mode, and mixer signs
- MPU6050 mounting and roll/pitch/yaw polarities
- PMW3901 SPI pins, signs, custom count scale, rotational compensation, and sensor offset
- VL53L0X I2C behavior, activation limits, short-loss freeze, long-loss descent, and recovery logic
- Roll/pitch angle gains and rate PID gains
- Yaw-rate damping gains
- Enable/disable ramps, throttle mapping, kill, signal-loss, and tilt safety
- Exact ESP-NOW packet layout and centered-throttle enable requirement

Those behaviors remain the control and safety baseline around the new ESKF and model feed-forward.

## Control and estimator rates

- Attitude/ESKF prediction and feedback controller: 250 Hz
- PMW3901 update: 50 Hz
- VL53L0X data-ready polling: 250 Hz maximum; measurements approximately every 20 ms
- Transmitter packet: 50 Hz
- Debug line: 1 Hz

## Serial commands

Existing gain commands remain available. Added commands are:

```text
eskf                 print ESKF state and innovation statistics
eatt                 print ESKF-attitude commissioning state
eatt on              use ESKF roll/pitch/yaw for attitude feedback
eatt off             retain ESKF navigation but use complementary attitude
model                print model status and current applied feed-forward
model on             enable model feed-forward
model off            disable model feed-forward; feedback PID remains active
ffblend <0..0.50>     set motor feed-forward blend
rateff <0..0.80>      set desired-rate feed-forward blend
```

Useful existing commands:

```text
status
level
navreset
pid
outer
pidreset
kill / stop / disarm
```

## Required test sequence

### Stage 1: compile and stationary sensor test, no propellers

1. Upload the receiver sketch.
2. Keep the airframe level and motionless during calibration.
3. Run `eskf` and `status`.
4. Confirm `ESKF,HEALTHY:1` and that roll/pitch are near zero.
5. Tilt the right side and nose by hand. Confirm the `eskf` P/R values have the
   same signs as the old displayed controller angles. ESKF attitude feedback is
   still OFF at this point.
6. Move the drone right and forward over a textured surface. Confirm PMW velocity signs.
7. Raise and lower the drone. Confirm `Z` and `VZ` signs.

Do not continue if any sign is wrong.

### Stage 2: motor and mixer test, no propellers

1. Leave model feed-forward off: `model off`.
2. Verify each numbered motor pin and spin direction individually.
3. With `eatt off`, verify roll, pitch, and yaw feedback corrections match the
   old controller.
4. Disable the motors, issue `eatt on`, re-enable, and repeat the hand-tilt
   mixer test. The correction pattern must remain identical.
5. Disable the motors again, set `ffblend 0.05`, `rateff 0.10`, then issue
   `model on`.
6. Re-enable and verify the model terms are only a small addition to the same
   motor pattern.

### Stage 3: restrained/tethered test

1. Use the already-working PID values unchanged.
2. Test with `eatt off`, `model off` first to establish the estimator-navigation
   baseline without replacing the old attitude feedback.
3. Test `eatt on`, `model off` next.
4. Enable the model with `ffblend 0.05`, `rateff 0.10` while motors are off.
5. Increase one blend at a time in small steps while checking the 1 Hz debug output.
6. Stop increasing if motor commands chatter, reverse sharply, saturate, or produce a delayed oscillation.

### Stage 4: free flight

Only after all previous stages pass. Begin low and over a clear area with immediate kill access.

## Important limitations

1. **No measured RPM:** non-bidirectional DShot gives no rotor-speed feedback. The Python model predicts rad/s, but the conversion to DShot is only a local linear approximation around hover. Battery voltage, ESC behavior, motor loading, and propeller aerodynamics will change it.
2. **`CT = 0.18` is not an identified value for this exact motor/propeller system:** model feed-forward must remain conservative until a thrust/RPM test identifies the real coefficient.
3. **No online IMU bias states:** gyro and accelerometer biases are calibrated at startup. Long-term temperature drift is not estimated.
4. **No absolute yaw reference:** yaw remains gyro-dead-reckoned and will drift.
5. **Optical flow and ToF are local sensors:** floor texture, lighting, vibration, tilt, ground effect, and sensor outages still matter.
6. **Raw ToF convention is preserved:** the altitude loop uses sensor-to-floor range, not a tilt-corrected center-of-mass altitude and not the 0.03 m landing-gear offset.
7. **The model is near-hover inverse dynamics:** it does not include motor electrical dynamics, battery sag, drag, blade flapping, vortex-ring behavior, or a calibrated yaw drag coefficient.

## Validation performed for this generated package

- The receiver sketch was compiled as standard C++ against local Arduino/ESP32 API stubs with warnings enabled.
- The complete setup/loop path was executed under address and undefined-behavior sanitizers using those stubs.
- ESKF host tests covered stationary propagation, gravity-direction tilt recovery, optical-flow velocity correction, and position integration.
- Motion-model host tests checked hover validity, pitch motor polarity, roll motor polarity, and settled tilt collective compensation.
- A dedicated convention test checked the negative right-handed pitch/roll mapping used by the established mixer.

The actual ESP32 board core and the three external Arduino libraries were not installed in the generation environment, so the final hardware-library compile and all physical sign tests must be performed in your Arduino IDE and on your airframe.
