# FPV 2026 — Quadrotor Flight Controller

A 232 mm quadrotor and its flight controller, both built from scratch
over the summer of 2026 on a single ESP32. Control runs on **per-rotor angular
velocity**, closed on RPM fed back over bidirectional DShot, with state supplied
by a 9-state error-state Kalman filter.

Build photos, wiring sheets, and flight footage:
**[dantebenedetti.com/projects/quadrotor.html](https://dantebenedetti.com/projects/quadrotor.html)**

Sole designer and developer. Independent project, June – August 2026.

## What makes it different

**Every sign convention is derived from one declaration.** `FrameConventions.h`
defines the body frame (+X right, +Y forward, +Z up), declares all rotations
right-handed, and derives the IMU mounting transform from it. An earlier build
stored a counter-clockwise yaw rate in a clockwise-named variable; three
consumers each assumed clockwise and each broke differently — positive feedback
in the mixer, a sign-flipped magnetic heading measurement into the filter, and
body/navigation transforms written as each other's transpose. No gain change
could have fixed it. `Code/Rotor_Rate_Controller_Package/README.md` is the full
post-mortem.

**The mixer is derived, not transcribed.** `RotorAllocator` builds the control
effectiveness matrix by summing `r × F` over the measured rotor positions and
spin directions, so a sign error would have to be a geometry error.

**Rotors are closed-loop actuators.** Because the ESCs report measured eRPM on
the same wire that carries the command, each rotor is commanded in rad/s rather
than open-loop throttle, which removes battery sag, prop loading, and
temperature from the control path.

**The drag-torque coefficient is identified in flight.** `YawTorqueIdentifier`
runs normalised LMS on the yaw channel to estimate `k_M`, which was never
characterised for this airframe.

## Control cascade

```
position → velocity → acceleration → tilt
tilt → body rate → angular acceleration → torque
(thrust, torque) → RotorAllocator → per-rotor ω setpoint
ω setpoint → RotorSpeedLoop (closed on DShot RPM telemetry) → DShot command
```

## Hardware

| | |
|---|---|
| Frame | 232 mm diagonal (163 mm side-to-side), 5-inch tri-blade props (R = 63.5 mm), 2 CW / 2 CCW |
| Power | 1300 mAh 4S LiPo → XT60 → distribution → 4× ESC, buck to 5 V for the ESP32 |
| Compute | ESP32 devkit, flight controller and estimator on-board |
| ESCs | Bidirectional DShot with RPM telemetry (Bluejay) |
| Sensing | MPU6050 IMU, magnetometer, PMW3901 optical flow, VL53L0X rangefinder |
| Link | ESP-NOW at 50 Hz to a hand-built ESP32 transmitter, telemetry downlink to a laptop |

Two wiring gotchas, both easy to discover expensively in the air: the I²C bus is
swapped from the ESP32 default (`Wire.begin(22, 21)`), and the gimbal
potentiometers must run from 3.3 V — GPIO34/35 are 3.6 V absolute maximum.

## Layout

```
Code/Rotor_Rate_Controller_Package/   current flight firmware — receiver and
                                      transmitter sketches, ESKF, rotor
                                      allocator, speed loop, hold manager,
                                      yaw-torque identifier, host validation
                                      suite, flight logs and analysis scripts
Code/ESKF_Bidirectional_RPM_...       prior iteration: ESKF + bidirectional DShot
Code/ESKF_Model_Based_...             prior iteration: model-based controller
Code/Robust_XY_Repair_Package         XY/flow repair attempt and its diagnosis
Code/Bidirectional_Dshot              DShot telemetry bring-up sketch
Code/MPU6050, Range_Finder,           per-sensor bring-up and calibration
     Optical_Flow, Motor_Test,        sketches, plus a 3-D IMU visualiser
     Joystick_Test, Remote_Control
Code/Receiver_PID                     earlier PID-on-attitude controller
Schematics/                           Multisim source and generated wiring PDFs
                                      (drawn from the firmware pin map)
Simulation/                           MATLAB: rigid-body rotation, LQR, NMPC,
                                      motor and manual-flight models
Report/                               IEEEtran paper — main.tex, refs.bib, and
                                      figures generated from flight logs
```

## Building and flying

Firmware is Arduino-framework ESP32; open the `.ino` in Arduino IDE or
`arduino-cli` and flash the receiver to the airframe, the transmitter to the
hand controller. Flight procedure in
`Code/Rotor_Rate_Controller_Package/VIDEO_FLIGHT_PROCEDURE.md`.

The estimator, motion model, allocator, and every sign convention are covered by
a host-side test suite that compiles with stubbed Arduino/ESP-IDF headers — no
hardware required:

```bash
cd Code/ESKF_Bidirectional_RPM_Controller_Package/Validation && ./run_host_validation.sh
```

Figures and the paper regenerate from the logs:

```bash
cd Report && python make_figures.py   # then compile main.tex with pdfLaTeX
```

Vendor datasheets and reference papers used during the build are intentionally
not redistributed here.
