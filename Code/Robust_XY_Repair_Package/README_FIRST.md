# Robust XY Repair Package for the Current ESP32 Drone

## Read this before flashing

The old XY controller is not safe for another unrestricted hover. It activates
while the vehicle is still climbing, has almost no position-return authority,
accepts PMW3901 data without real quality checks, allows a one-axis ESKF update
to count as a valid 2-D measurement, and is being used while yaw is oscillating
near 120 deg/s.

This package is deliberately split into a diagnostic phase and a flight-code
phase. XY hold is **disabled by an interlock after every boot** until the
physical sensor mapping is proven and `xycommission on` is issued while the
motors are stopped.

## Package contents

```text
README_FIRST.md
DIAGNOSIS.md
GITHUB_REFERENCE_MAP.md
latest_hover_log.txt
analyze_telemetry.py

PMW3901_Flow_Bench_Diagnostic/
  PMW3901_Flow_Bench_Diagnostic.ino

Receiver_Robust_XY/
  Receiver_Robust_XY.ino
  Eskf9_Robust_XY.h
  OpticalFlowAsync_Robust.h
  AsyncDebugTelemetry.h/.cpp
  BidirectionalDShot.h/.cpp
  MotionModelFF.h
  RotorFeedbackModel.h

Main_Controller_Robust_XY.patch
Eskf9_Robust_XY.patch
```

The receiver folder is assembled as a complete Arduino sketch from the files
supplied in this conversation plus the two robust replacements. Required
external Arduino libraries remain:

- Bitcraze PMW3901
- Pololu VL53L0X
- Arduino-ESP32 version compatible with the existing IDF 5 RMT code

## Phase 0: reproduce the log diagnosis

On a computer with Python 3:

```bash
python analyze_telemetry.py latest_hover_log.txt
```

Expected key results are approximately:

```text
first XY activation: Z=0.272 m, VZ=+0.427 m/s, ALT=0
max |X-XT|: 1.162 m
max |Y-YT|: 1.516 m
max |YR|: 119.73 deg/s
max |YC|: 12.0
```

## Phase 1: prove the PMW3901 before using it for control

1. Remove all propellers.
2. Put the aircraft 0.4-0.7 m above a matte, textured, stationary floor.
3. Flash `PMW3901_Flow_Bench_Diagnostic.ino`.
4. Open Serial Monitor at 460800 baud.
5. Start with compensation disabled:

```text
comp 0
zero
```

6. Move the complete aircraft physically to its right without yawing it. Try
   mapping modes until reported body X is positive and body Y is near zero:

```text
map 0
map 1
...
map 7
```

7. Move it physically forward. The same mapping must report positive body Y
   and near-zero body X.
8. Record the selected mode. Do not infer it from PCB labels.
9. With the aircraft held over one point, rotate only in roll and then only in
   pitch. Compare `comp 0` with `comp 1`. Correct compensation should reduce,
   not enlarge, the integrated false translation.
10. Run the motors with propellers still removed and inspect motion marker,
    SQUAL, shutter, stationary velocity, and spikes.

Diagnostic mapping modes:

```text
0: body X=+raw X, body Y=+raw Y
1: body X=+raw X, body Y=-raw Y   old controller default
2: body X=-raw X, body Y=+raw Y
3: body X=-raw X, body Y=-raw Y
4: body X=+raw Y, body Y=+raw X
5: body X=+raw Y, body Y=-raw X
6: body X=-raw Y, body Y=+raw X
7: body X=-raw Y, body Y=-raw X   common flow-deck mapping
```

Engineering pass criteria for commissioning:

- right movement and forward movement have the correct independent signs;
- stationary output does not show recurring large velocity spikes;
- the PMW motion marker is normally `0xB0`;
- SQUAL is nonzero over the test surface;
- gyro compensation reduces rotation-induced integrated translation by at
  least several times instead of increasing it;
- motor vibration does not destroy quality.

## Phase 2: flash the safe receiver

1. Open `Receiver_Robust_XY/Receiver_Robust_XY.ino` in Arduino IDE.
2. Confirm the board and Arduino-ESP32 version match the working DShot build.
3. Flash with propellers removed first.
4. On boot, verify the startup text says:

```text
Commissioning mode: magnetic yaw fusion starts OFF.
Yaw heading hold starts OFF.
XY hold: SAFE INTERLOCK OFF ...
```

5. Set the measured PMW mapping while motors are stopped:

```text
flowmap N
flowstatus
```

`flowmap` intentionally turns commissioning off again, forcing a fresh review.
6. Verify `FLOWSTAT` shows coherent motion, nonzero SQUAL, plausible shutter,
   both-axis acceptance mask `3`, and no rapidly increasing rejection counter.
7. Only then enter, while stopped:

```text
xycommission on
```

## Phase 3: fix yaw before world-frame XY hold

Do not enable heading hold or free-flight XY while the yaw-rate loop is
reversing at tens to hundreds of degrees per second.

Minimum yaw pass conditions on a safe stand/tether:

- clockwise hand disturbance produces the expected signed yaw rate;
- yaw correction opposes the disturbance;
- motor pair response matches the physical torque direction;
- no repeated `YC=+limit/-limit` reversal;
- centered-stick yaw rate settles below roughly 8-10 deg/s before XY arming.

The robust build starts compass fusion and heading hold off specifically so
flow mapping can be tested without that unstable loop contaminating it.

## Phase 4: tethered XY commissioning

Use a restrained rig that allows small horizontal displacement but prevents
contact or flyaway.

### Stage A: velocity damping only

```text
xyposkp 0
xyvelkp 1.0
```

Enable `xycommission on` while stopped, arm, let altitude hold settle, and wait
for automatic XY activation. Apply small right/left and forward/back
disturbances separately. Pass only when measured velocity returns toward zero
without growing oscillation and without repeated attitude-limit reversal.

### Stage B: weak position return

```text
xyposkp 0.30
xyvelkp 1.0
```

The position error should decrease after a small displacement. Test one axis at
a time by physically constraining the other direction.

### Stage C: nominal robust values

```text
xyposkp 0.60
xyvelkp 1.50
```

The robust controller interprets these in physical units:

```text
position error * xyPositionKp -> desired velocity
velocity error * xyVelocityKp -> desired acceleration
angle = atan2(horizontal acceleration, gravity)
```

It limits velocity to 0.35 m/s, horizontal acceleration to 0.8 m/s^2, and XY
attitude demand to 5 degrees, with a 0.12 s filter and 30 deg/s slew limit.

## Automatic activation requirements in the robust build

XY hold will not activate until all conditions remain true for one second:

- `xycommission on` was issued;
- startup ramp is complete;
- altitude hold is active;
- at least 50 consecutive valid 100 Hz flow samples were accepted;
- both ESKF velocity axes accept each sample;
- height is at least 0.30 m;
- vertical speed is at most 0.12 m/s;
- horizontal speed is at most 0.20 m/s;
- yaw rate is at most 8 deg/s;
- roll and pitch are each within 6 degrees.

Hold aborts on stale flow, bad estimator state, excessive vertical speed,
excessive yaw rate, or excessive tilt.

## Important limitations

This package was statically checked and its log analysis was executed. It was
not compiled against the exact local Arduino toolchain and was not flown on the
physical drone. The sensor-axis map, gyro-compensation signs, optical scale,
yaw torque direction, and final gains must be commissioned on the actual
hardware in the order above.
