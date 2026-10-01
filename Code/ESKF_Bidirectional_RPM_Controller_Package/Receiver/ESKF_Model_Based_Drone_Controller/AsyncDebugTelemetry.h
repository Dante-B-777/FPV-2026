#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// All periodic telemetry formatting and UART writes are performed by the
// low-priority task created by this class. The flight loop only copies a fixed
// binary snapshot into a one-element queue and never waits for the UART.

struct AsyncRpmMotorSnapshot
{
  uint16_t dshotCommand = 0;
  float rpmRaw = 0.0f;
  float rpmFiltered = 0.0f;
  float omegaRadPerSec = 0.0f;
  bool valid = false;
  bool lastPacketValid = false;
  bool lost = false;
  float packetQuality = 0.0f;
  uint32_t ageUs = UINT32_MAX;
  char statusName[20] = {};
  bool measuredSource = false;
  float omegaPerDshot = 0.0f;
};

struct AsyncFlightTelemetrySnapshot
{
  uint32_t timestampMs = 0;

  bool signalLost = true;
  bool imuValid = false;
  bool safetyStop = true;
  bool tiltStop = false;
  bool flightEnabled = false;
  bool eskfStateHealthy = false;
  bool eskfAttitudeUsed = false;
  bool motionModelValid = false;
  bool motionModelSaturated = false;
  uint8_t rpmFreshMask = 0;
  bool rpmTelemetryHealthy = false;
  float rotorForceZ_N = 0.0f;
  float eskfProcessNoiseScale = 1.0f;
  bool xyHoldActive = false;
  bool altitudeHoldActive = false;
  bool rangeControlUsable = false;
  uint32_t rangeAgeMs = UINT32_MAX;
  bool flowReady = false;
  int32_t baseThrottleDshot = 0;

  float rangeM = 0.0f;
  float altitudeM = 0.0f;
  float altitudeTargetM = 0.0f;
  float verticalVelocityMps = 0.0f;
  float desiredVerticalVelocityMps = 0.0f;
  float altitudeCorrectionDshot = 0.0f;

  float positionX_M = 0.0f;
  float targetX_M = 0.0f;
  float positionY_M = 0.0f;
  float targetY_M = 0.0f;
  float measuredVelocityX_Mps = 0.0f;
  float targetVelocityX_Mps = 0.0f;
  float measuredVelocityY_Mps = 0.0f;
  float targetVelocityY_Mps = 0.0f;
  float fusedVelocityX_Mps = 0.0f;
  float fusedVelocityY_Mps = 0.0f;
  float linearAccelX_Mps2 = 0.0f;
  float linearAccelY_Mps2 = 0.0f;

  float rollAngleDeg = 0.0f;
  float desiredRollAngleDeg = 0.0f;
  float pitchAngleDeg = 0.0f;
  float desiredPitchAngleDeg = 0.0f;
  float yawRateDps = 0.0f;
  float yawCorrection = 0.0f;
  float innovationRatio = 0.0f;
  float modelCollectiveDshot = 0.0f;
  float modelMotorDeltaDshot[4] = {};

  uint16_t motorCommand[4] = {};
  uint32_t controlOverrunCount = 0;
  uint32_t maximumControlDtUs = 0;
  uint32_t minimumControlDtUs = UINT32_MAX;

  // One-second profiler window. All execution times are microseconds.
  uint32_t timingCommandAvgUs = 0;
  uint32_t timingCommandMaxUs = 0;
  uint32_t timingRxCopyAvgUs = 0;
  uint32_t timingRxCopyMaxUs = 0;
  uint32_t timingControlAvgUs = 0;
  uint32_t timingControlMaxUs = 0;
  uint32_t timingRotorAvgUs = 0;
  uint32_t timingRotorMaxUs = 0;
  uint32_t timingAttitudeAvgUs = 0;
  uint32_t timingAttitudeMaxUs = 0;
  uint32_t timingImuAvgUs = 0;
  uint32_t timingImuMaxUs = 0;
  uint32_t timingEskfAvgUs = 0;
  uint32_t timingEskfMaxUs = 0;
  uint32_t timingEnableAvgUs = 0;
  uint32_t timingEnableMaxUs = 0;
  uint32_t timingFlightControllerAvgUs = 0;
  uint32_t timingFlightControllerMaxUs = 0;
  uint32_t timingMotorPublishAvgUs = 0;
  uint32_t timingMotorPublishMaxUs = 0;
  uint32_t timingRangeAvgUs = 0;
  uint32_t timingRangeMaxUs = 0;
  // Core-1 flow processing is separate from opposite-core PMW SPI I/O.
  uint32_t timingFlowAvgUs = 0;
  uint32_t timingFlowMaxUs = 0;
  uint32_t timingFlowIoAvgUs = 0;
  uint32_t timingFlowIoMaxUs = 0;
  float flowTaskHz = 0.0f;
  uint32_t flowSkippedSamples = 0;
  int8_t flowTaskCore = -1;
  uint32_t timingLoopAvgUs = 0;
  uint32_t timingLoopMaxUs = 0;
  uint32_t timingAsyncPublishAvgUs = 0;
  uint32_t timingAsyncPublishMaxUs = 0;

  uint32_t espNowCallbackAvgUs = 0;
  uint32_t espNowCallbackMaxUs = 0;
  uint32_t espNowIntervalUs = 0;
  uint32_t espNowIntervalMinUs = 0;
  uint32_t espNowIntervalMaxUs = 0;
  float dshotServiceHz = 0.0f;

  uint32_t rpmSequence = 0;
  float rotorTorqueX_Nm = 0.0f;
  float rotorTorqueY_Nm = 0.0f;
  AsyncRpmMotorSnapshot rpmMotor[4];
};

enum class AsyncFlightEventType : uint8_t
{
  XyHoldDisabledFlowTimeout,
  XyHoldActivated,
  AltitudeHoldActivated,
  RangeRecovered,
  DisableRampComplete,
  EnableRampComplete,
  RemoteEnableOff,
  ImmediateFlightStop,
  EnableIgnoredDuringRampDown,
  RemoteEnableAccepted,
  RemoteEnableRejected
};

enum class AsyncStopReason : int32_t
{
  RemoteKill = 0,
  SignalLost = 1,
  ImuFailure = 2,
  ExcessiveTilt = 3,
  SerialStop = 4
};

struct AsyncFlightEvent
{
  AsyncFlightEventType type = AsyncFlightEventType::XyHoldDisabledFlowTimeout;
  uint32_t timestampMs = 0;
  float value0 = 0.0f;
  float value1 = 0.0f;
  int32_t integer0 = 0;
  int32_t integer1 = 0;
  uint32_t unsigned0 = 0;
};

class AsyncDebugTelemetry
{
public:
  static constexpr size_t TEXT_LINE_CAPACITY = 768;
  static constexpr size_t COMMAND_LINE_CAPACITY = 96;

  bool begin(
    HardwareSerial &serial,
    BaseType_t coreId,
    UBaseType_t priority = 1,
    uint32_t stackBytes = 12288
  );

  // Nonblocking. The newest snapshot replaces an older unprinted snapshot.
  bool publish(const AsyncFlightTelemetrySnapshot &snapshot);

  // Nonblocking. Events are discarded rather than delaying flight control if
  // the small event queue is full.
  bool postEvent(const AsyncFlightEvent &event);

  // Intended for low-frequency serial-command responses, not periodic flight
  // telemetry. The caller supplies a completed line without a trailing '\n'.
  bool postText(const char *line);

  // Nonblocking consumer API for the flight core. UART RX itself is owned by
  // the debug task; complete lines are transferred through a FreeRTOS queue.
  bool popCommand(char *destination, size_t capacity);

  bool started() const { return taskHandle_ != nullptr; }
  BaseType_t coreId() const { return coreId_; }
  uint32_t droppedSnapshots() const { return droppedSnapshots_; }
  uint32_t droppedEvents() const { return droppedEvents_; }
  uint32_t droppedTextLines() const { return droppedTextLines_; }
  uint32_t droppedCommands() const { return droppedCommands_; }

private:
  struct TextLine
  {
    char text[TEXT_LINE_CAPACITY];
  };

  struct CommandLine
  {
    char text[COMMAND_LINE_CAPACITY];
  };

  static void taskEntry(void *argument);
  void run();
  void printTelemetry(const AsyncFlightTelemetrySnapshot &snapshot);
  void printEvent(const AsyncFlightEvent &event);
  void pollSerialRx();

  HardwareSerial *serial_ = nullptr;
  QueueHandle_t telemetryQueue_ = nullptr;
  QueueHandle_t eventQueue_ = nullptr;
  QueueHandle_t textQueue_ = nullptr;
  QueueHandle_t commandQueue_ = nullptr;
  TaskHandle_t taskHandle_ = nullptr;
  BaseType_t coreId_ = tskNO_AFFINITY;

  volatile uint32_t droppedSnapshots_ = 0;
  volatile uint32_t droppedEvents_ = 0;
  volatile uint32_t droppedTextLines_ = 0;
  volatile uint32_t droppedCommands_ = 0;

  char rxBuffer_[COMMAND_LINE_CAPACITY] = {};
  size_t rxLength_ = 0;
};