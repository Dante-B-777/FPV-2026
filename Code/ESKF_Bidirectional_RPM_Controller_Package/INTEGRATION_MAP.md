# Bidirectional DShot RPM Integration Map

## Classification of RPM in this controller

RPM is a **known motion-model input**.

It is not:

- a new ESKF state;
- an ESKF measurement update;
- a direct standalone RPM PID loop.

## Exact motor-output integration point

`updateFlightController()` preserves the existing mixer and writes:

```text
motor1Command
motor2Command
motor3Command
motor4Command
```

`sendMotorCommands()` copies those four values in one call to:

```cpp
bidirectionalDshot.setAllMotorCommands(commands);
```

The 500 Hz DShot task snapshots all four values atomically before launching a transaction.

## Exact RPM/model integration point

At the start of each 250 Hz control cycle:

```cpp
updateRotorFeedback(dt);
```

This function:

1. copies the latest completed four-motor RPM snapshot;
2. passes RPM and the transaction's DShot commands to `RotorFeedbackModel`;
3. selects measured RPM when fresh and a first-order estimate otherwise;
4. populates `motionModelRotorInput`;
5. sets the ESKF process-noise scale from telemetry health.

`updateModelFeedforward()` then calls:

```cpp
motionModelFeedforward.update(..., &motionModelRotorInput);
```

`MotionModelFF.h` uses the input for current rotor angular momentum and the learned omega/DShot conversion.

## ESKF integration point

`updateAttitude()` calls:

```cpp
stateEstimator.predict(
    eskfSpecificForceBody,
    gyroBodyRadPerSecond,
    dt,
    eskfActuatorProcessNoiseScale);
```

The nominal ESKF acceleration remains IMU-based. RPM health affects uncertainty only, preventing propulsion force from being counted twice.

## Resource audit

The previous `DShotRMT` objects and include were removed. No other receiver feature uses RMT.

The custom module allocates:

```text
4 RMT RX channels
4 RMT TX channels
```

PMW3901 uses SPI. VL53L0X and MPU6050 use I2C. ESP-NOW uses Wi-Fi. No additional GPIO or RMT conflict was found in the supplied receiver.
