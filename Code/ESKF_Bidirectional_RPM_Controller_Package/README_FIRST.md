# ESP32 ESKF Controller with Bluejay Bidirectional DShot RPM Feedback

## Safety first

**Remove all propellers before the first upload, RPM test, motor-number test, mixer-sign test, and kill test.** After the propeller-free checks, use a restrained/tethered test stand. Do not begin with free flight.

This receiver is built for the **original ESP32 / ESP32-WROOM / ESP32-D0WD family** using **Arduino-ESP32 3.3.10**. It uses all eight original ESP32 RMT channels: four TX and four RX. Do not use `DShotRMT`, NeoPixel/WS2812 RMT, IR RMT, or another RMT consumer in the same firmware.

## What to upload

Open this sketch in Arduino IDE:

```text
Receiver/
  ESKF_Model_Based_Drone_Controller/
    ESKF_Model_Based_Drone_Controller.ino
```

Keep every file below in that same sketch folder. Arduino compiles the `.cpp` and `.h` files automatically.

```text
ESKF_Model_Based_Drone_Controller/
  ESKF_Model_Based_Drone_Controller.ino
  BidirectionalDShot.h
  BidirectionalDShot.cpp
  RotorFeedbackModel.h
  MotionModelFF.h
  Eskf9.h
```

The transmitter sketch is unchanged and remains packet-compatible:

```text
Transmitter/
  Existing_Transmitter_Compatible/
    Existing_Transmitter_Compatible.ino
```

## Arduino IDE target and libraries

Board:

```text
ESP32 Dev Module
```

or the equivalent ESP32 Dev Kit 1 entry.

Required external libraries:

- Bitcraze PMW3901
- VL53L0X by Pololu

**Do not install or include `DShotRMT` for this receiver.** The local `BidirectionalDShot` module directly uses the ESP-IDF 5.5 RMT API that ships with Arduino-ESP32 3.3.10.

## Preserved motor map

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

The intentional M3/M4 pin arrangement is preserved:

```text
M1 = GPIO26
M2 = GPIO14
M3 = GPIO27
M4 = GPIO25
```

Motor pole counts are:

```text
14, 14, 14, 14
```

Fourteen magnetic poles means seven pole pairs. Mechanical RPM is electrical RPM divided by seven.

The existing feedback mixer remains unchanged:

```text
M1 = base - pitch - roll + yaw
M2 = base - pitch + roll - yaw
M3 = base + pitch + roll + yaw
M4 = base + pitch - roll - yaw
```

## Architecture

```text
ESP-NOW command packet, 50 Hz
        |
        v
Existing enable/kill/ramp logic
        |
        v
Existing X/Y/Z outer loops
        |
        +-------------------------------+
        |                               |
        v                               v
Existing angle/rate PID       Python inverse-dynamics model
        |                     + measured rotor-state input
        +---------------+---------------+
                        |
                        v
              Preserved motor mixer, 250 Hz
                        |
                        v
        coherent four-command DShot buffer
                        |
                        v
 Dedicated bidirectional DShot task, 500 Hz, core 0
        |                               |
        v                               v
  four Bluejay commands          four RPM replies
                                        |
                                        v
                           RotorFeedbackModel
                         measured RPM when fresh
                         motor-lag fallback if stale
                                        |
                  +---------------------+------------------+
                  |                                        |
                  v                                        v
      MotionModelFF known input                 ESKF uncertainty scale
  actual rotor momentum + learned map        no duplicate thrust injection
```

## How RPM is used

RPM is implemented as a **known motion-model input**. It is not added as four new ESKF states and it is not treated as a direct Kalman measurement of position or attitude.

For each motor, the RPM path stores:

- raw mechanical RPM;
- filtered mechanical RPM;
- angular velocity in rad/s;
- last-valid transaction timestamp;
- age;
- last-packet validity;
- stale/lost state;
- consecutive good and bad counts;
- rolling 64-packet quality;
- decoder status.

The conversion is:

```text
omega [rad/s] = RPM * 2*pi / 60
```

`RotorFeedbackModel.h` then does three things:

1. Uses fresh filtered RPM directly as the rotor-speed input.
2. Maintains a first-order command-to-speed state for continuity and fallback.
3. Slowly learns a local `omega / DShot` scale from valid telemetry.

`MotionModelFF.h` now uses this input in two places:

- the current signed rotor angular momentum in the Python model's `L` calculation;
- the measured, locally learned `omega / DShot` scale when converting desired rotor angular velocity to bounded DShot feed-forward.

This removes the previous assumption that current rotor speed can be inferred only from the DShot command.

The measured rotor speeds also produce diagnostic model values:

```text
Fi = kF * omega_i^2
```

and body roll/pitch torque from the actual motor positions. The `kF` value still comes from the supplied Python model (`CT = 0.18`); RPM alone does not make thrust exact.

The yaw reaction-torque coefficient `kM` is initialized to zero because no measured value was supplied. Rotor angular momentum is still calculated with the correct M1/M3 positive and M2/M4 negative spin signs. Do not assign negative mechanical RPM to clockwise motors.

## Why RPM force is not directly added to the ESKF acceleration

The ESKF nominal state is already propagated from measured MPU6050 specific force. Adding `kF*omega^2/m` to that same nominal acceleration would count propulsion twice.

Instead:

- measured RPM drives the separate rotor/motion model;
- fresh/stale RPM health scales ESKF process uncertainty;
- the accelerometer remains the ESKF mean-propagation input.

With all four RPM channels fresh, the process-noise scale is `1.0`. It increases gradually to `2.0` as channels fall back.

## Timing

| Function | Rate / bound |
|---|---:|
| Bidirectional DShot command + reply service | 500 Hz |
| DShot frame period | 2,000 us |
| RMT transaction deadline inside DShot task | 1,200 us |
| Attitude controller and ESKF prediction | 250 Hz |
| PMW3901 update | 50 Hz |
| VL53L0X measurement | about 50 Hz |
| ESP-NOW transmitter packet | 50 Hz |
| Debug output | 1 Hz |

The DShot service runs in a dedicated FreeRTOS task pinned to core 0. The flight-control loop never calls `rmt_receive`, waits on an RX semaphore, or decodes telemetry. It only:

1. writes a coherent four-motor command snapshot; and
2. copies the latest completed RPM snapshot.

A 250 Hz control output is normally transmitted twice by the 500 Hz DShot task.

Typical RPM age at the controller should be about 1-4 ms. Policies are:

```text
5 consecutive failed packets -> telemetry lost for that motor
20 ms without a valid sample  -> stale
isolated failed packet         -> last valid RPM is held briefly
stale/lost telemetry           -> smooth first-order motor-state fallback
```

Telemetry loss does not block a DShot command and does not prevent an immediate stop command.

## Preserved Bluejay implementation details

`BidirectionalDShot.cpp` preserves the working standalone protocol details:

```text
RMT resolution             40 MHz
DShot rate                 300 kbit/s
Telemetry rate             375 kbit/s
DShot bit ticks            133
DShot-one low ticks        100
DShot-zero low ticks       50
End HIGH hold              133 ticks
RX idle stop               300 us
Frame period               2 ms
```

It also preserves:

- inverted bidirectional checksum;
- telemetry bit left at zero;
- push-pull hardware TX;
- same-pin RMT RX;
- one-cell HIGH hold after the 16-bit command;
- weak internal pull-up during the released reply window;
- GCR, NRZI, checksum, exponent/mantissa, and pole-pair decoding;
- manual GPIO-matrix output-enable control.

The critical output-enable behavior remains:

```cpp
GPIO.func_out_sel_cfg[gpio].oen_sel = 1;
GPIO.func_out_sel_cfg[gpio].oen_inv_sel = 0;
```

TX completion releases the ESC line with `GPIO.enable_w1tc`; the next command reconnects it with `GPIO.enable_w1ts`.

## Safety and fallback behavior

Preserved safety behavior includes:

- centered-throttle remote enable;
- startup ramp from DShot 200 to 505 over two seconds;
- remote disable ramp;
- immediate kill, serial stop, IMU failure, signal-loss, and excessive-tilt stop logic;
- 300 ms ESP-NOW timeout;
- range-loss controlled descent;
- optical-flow loss and recovery logic.

RPM telemetry is **not** allowed to delay or veto motor stop. `stopAll()` writes zero to the command buffer and wakes the DShot task for an additional prompt zero frame.

The ESKF attitude replacement and model feed-forward still start disabled after every reboot:

```text
eatt off
model off
```

RPM acquisition itself starts immediately and runs regardless of those commissioning interlocks.

## Serial diagnostics

Serial Monitor baud:

```text
115200
```

Use:

```text
rpm
```

to print the detailed RPM line. It includes, for every motor:

```text
c      DShot command used for the transaction
r      raw RPM
f      filtered RPM
w      angular speed in rad/s
v      last valid sample is within the age limit
lp     last packet passed decoding
loss   telemetry-loss state
q      rolling packet quality percentage
a      sample age in microseconds
s      decoder status
src    M = measured model input, F = fallback model input
k      learned omega-per-DShot scale
```

Example shape:

```text
RPM,SEQ:...,MASK:F,ALL:1,QPROC:1.00,FZ_N:...
 | M1:c500 r9360 f9400 w984.4 v1 lp1 loss0 q99.6 a1200 sOK srcM k1.91
 ...
```

The ordinary 1 Hz controller debug line also includes:

```text
RPMM     fresh-motor bit mask
RPMOK    all four fresh
RPMFZ    RPM-model total thrust estimate
QPROC    ESKF process-noise scale
```

## Propeller-free validation plan

1. Remove all propellers and disconnect the LiPo while uploading.
2. Upload the receiver sketch and open Serial Monitor at 115200.
3. Keep the frame level and still during IMU calibration.
4. Confirm startup reports `BIDIRECTIONAL DSHOT300 + BLUEJAY RPM FEEDBACK`.
5. Power the ESC and leave the remote disabled. Type `rpm`.
6. Confirm all four channels return valid stopped telemetry or begin returning valid telemetry once the motors are enabled.
7. Confirm the motor map remains M1=26, M2=14, M3=27, M4=25.
8. Center throttle and enable from the existing transmitter.
9. During the two-second ramp, verify each motor's command and RPM rise together.
10. Confirm `srcM` appears for all four motors and the mask is `F`.
11. At a stable command near 500, compare packet quality with the standalone result. Aim for approximately 99% under the same wiring and power conditions.
12. Verify each motor individually by identifying the physical motor while watching the matching M1-M4 RPM field.
13. Move the frame by hand with props removed and verify the established mixer response remains unchanged.
14. Test the remote kill. All commands must become zero immediately, independently of RPM validity.
15. Leave `eatt off` and `model off` until all signs and motor numbers are confirmed.

Useful decoder statuses are:

```text
OK, TIMEOUT, EMPTY, NO_GAP, NO_REPLY, SHORT, START, GCR, CRC, PERIOD
```

A persistent `NO_REPLY` or captures showing only the outgoing command usually points to a half-duplex/OEN or wiring problem. This package retains the manual-OEN fix from the verified implementation.

## Restrained/tethered test plan

1. Fit verified propellers and restrain the frame on a test stand.
2. First run with `eatt off` and `model off`; RPM acquisition still runs.
3. Confirm packet quality remains stable under motor electrical noise and loaded RPM.
4. Confirm no control-loop overruns increase abnormally.
5. Confirm RPM follows each mixer command during small roll and pitch disturbances.
6. Verify the model input source remains measured (`srcM`) and does not chatter between measured and fallback.
7. Disable motors completely, then enable model feed-forward with:

```text
ffblend 0.05
rateff 0.10
model on
```

8. Repeat small restrained roll/pitch tests. The existing PID remains authoritative.
9. Test remote disable ramp and kill again.
10. Only after the restrained tests pass should you consider a low hover test.

## Changed and added files

### Added

- `BidirectionalDShot.h`
- `BidirectionalDShot.cpp`
- `RotorFeedbackModel.h`

### Modified

- `ESKF_Model_Based_Drone_Controller.ino`
- `MotionModelFF.h`
- `Eskf9.h`

### Unchanged

- transmitter packet and transmitter sketch;
- motor pins, numbering, spin directions, mixer signs;
- roll/pitch/yaw gains;
- enable/disable ramps and kill logic;
- MPU6050, PMW3901, and VL53L0X wiring and signs.

## Configuration locations

| Parameter | File |
|---|---|
| Motor pins and pole counts | `ESKF_Model_Based_Drone_Controller.ino` |
| RPM filter alpha, stale timeout, loss count, DShot timing | `BidirectionalDShot.h` |
| Motor time constant, fallback correction, spin signs | `RotorFeedbackModel.h` |
| `kF`, mass, frame dimensions, prop radius, CT | `MotionModelFF.h` |
| `kM` default and setter | `RotorFeedbackModel.h` |

## Validation performed in the generated package

The `Validation` folder contains host tests for:

- the ESKF stationary, gravity, and optical-flow behavior;
- existing pitch/roll model signs;
- controller-to-ESKF sign conversion;
- measured RPM selection;
- learned omega/DShot scaling;
- stale-RPM motor-model fallback;
- RPM-derived force and roll/pitch torque signs;
- full receiver syntax with RMT/FreeRTOS API stubs;
- setup/loop execution under AddressSanitizer and UndefinedBehaviorSanitizer.

This does not replace the final Arduino IDE compile against your installed ESP32 3.3.10 board package or physical Bluejay bench testing.
