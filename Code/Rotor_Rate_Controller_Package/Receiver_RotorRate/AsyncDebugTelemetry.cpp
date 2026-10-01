#include "AsyncDebugTelemetry.h"

#include <cstring>

bool AsyncDebugTelemetry::begin(
  HardwareSerial &serial,
  BaseType_t coreId,
  UBaseType_t priority,
  uint32_t stackBytes
)
{
  if (taskHandle_ != nullptr)
  {
    return true;
  }

  serial_ = &serial;
  coreId_ = coreId;

  telemetryQueue_ = xQueueCreate(1, sizeof(AsyncFlightTelemetrySnapshot));
  eventQueue_ = xQueueCreate(16, sizeof(AsyncFlightEvent));
  textQueue_ = xQueueCreate(8, sizeof(TextLine));
  commandQueue_ = xQueueCreate(8, sizeof(CommandLine));
  trackQueue_ = xQueueCreate(TRACK_QUEUE_DEPTH, sizeof(AsyncTrackSample));

  if (
    telemetryQueue_ == nullptr ||
    eventQueue_ == nullptr ||
    textQueue_ == nullptr ||
    commandQueue_ == nullptr ||
    trackQueue_ == nullptr
  )
  {
    if (telemetryQueue_ != nullptr) vQueueDelete(telemetryQueue_);
    if (eventQueue_ != nullptr) vQueueDelete(eventQueue_);
    if (textQueue_ != nullptr) vQueueDelete(textQueue_);
    if (commandQueue_ != nullptr) vQueueDelete(commandQueue_);
    if (trackQueue_ != nullptr) vQueueDelete(trackQueue_);
    telemetryQueue_ = nullptr;
    eventQueue_ = nullptr;
    textQueue_ = nullptr;
    commandQueue_ = nullptr;
    trackQueue_ = nullptr;
    return false;
  }

  const BaseType_t result = xTaskCreatePinnedToCore(
    taskEntry,
    "AsyncDebug",
    stackBytes,
    this,
    priority,
    &taskHandle_,
    coreId
  );

  if (result != pdPASS)
  {
    vQueueDelete(telemetryQueue_);
    vQueueDelete(eventQueue_);
    vQueueDelete(textQueue_);
    vQueueDelete(commandQueue_);
    telemetryQueue_ = nullptr;
    eventQueue_ = nullptr;
    textQueue_ = nullptr;
    commandQueue_ = nullptr;
    taskHandle_ = nullptr;
    return false;
  }

  return true;
}

bool AsyncDebugTelemetry::publish(
  const AsyncFlightTelemetrySnapshot &snapshot
)
{
  if (telemetryQueue_ == nullptr || taskHandle_ == nullptr)
  {
    droppedSnapshots_++;
    return false;
  }

  // A one-element queue plus overwrite guarantees that flight control never
  // waits for the UART task. The newest complete sample always wins.
  if (xQueueOverwrite(telemetryQueue_, &snapshot) != pdPASS)
  {
    droppedSnapshots_++;
    return false;
  }

  xTaskNotifyGive(taskHandle_);
  return true;
}

bool AsyncDebugTelemetry::postEvent(const AsyncFlightEvent &event)
{
  if (eventQueue_ == nullptr || taskHandle_ == nullptr)
  {
    droppedEvents_++;
    return false;
  }

  if (xQueueSend(eventQueue_, &event, 0) != pdPASS)
  {
    droppedEvents_++;
    return false;
  }

  xTaskNotifyGive(taskHandle_);
  return true;
}

bool AsyncDebugTelemetry::publishTrack(const AsyncTrackSample &sample)
{
  if (trackQueue_ == nullptr || taskHandle_ == nullptr)
  {
    droppedTrackSamples_++;
    return false;
  }

  // Deliberately xQueueSend, not xQueueOverwrite. A dropped sample leaves a
  // gap in the sequence number, which the analysis can see; a silently
  // replaced sample would look like clean data with a wrong time base.
  if (xQueueSend(trackQueue_, &sample, 0) != pdPASS)
  {
    droppedTrackSamples_++;
    return false;
  }

  xTaskNotifyGive(taskHandle_);
  return true;
}

bool AsyncDebugTelemetry::postText(const char *line)
{
  if (line == nullptr || textQueue_ == nullptr || taskHandle_ == nullptr)
  {
    droppedTextLines_++;
    return false;
  }

  TextLine item = {};
  strncpy(item.text, line, sizeof(item.text) - 1);
  item.text[sizeof(item.text) - 1] = '\0';

  if (xQueueSend(textQueue_, &item, 0) != pdPASS)
  {
    droppedTextLines_++;
    return false;
  }

  xTaskNotifyGive(taskHandle_);
  return true;
}


bool AsyncDebugTelemetry::popCommand(char *destination, size_t capacity)
{
  if (destination == nullptr || capacity == 0 || commandQueue_ == nullptr)
  {
    return false;
  }

  CommandLine command = {};
  if (xQueueReceive(commandQueue_, &command, 0) != pdTRUE)
  {
    return false;
  }

  strncpy(destination, command.text, capacity - 1);
  destination[capacity - 1] = '\0';
  return true;
}

void AsyncDebugTelemetry::pollSerialRx()
{
  if (serial_ == nullptr || commandQueue_ == nullptr) return;

  while (serial_->available() > 0)
  {
    const char character = static_cast<char>(serial_->read());

    if (character == '\n')
    {
      rxBuffer_[rxLength_] = '\0';

      if (rxLength_ > 0)
      {
        CommandLine command = {};
        strncpy(command.text, rxBuffer_, sizeof(command.text) - 1);
        command.text[sizeof(command.text) - 1] = '\0';

        if (xQueueSend(commandQueue_, &command, 0) != pdPASS)
        {
          droppedCommands_++;
          serial_->println("SERIAL COMMAND DROPPED: command queue full.");
        }
      }

      rxLength_ = 0;
    }
    else if (character != '\r')
    {
      if (rxLength_ < sizeof(rxBuffer_) - 1)
      {
        rxBuffer_[rxLength_++] = character;
      }
      else
      {
        rxLength_ = 0;
        droppedCommands_++;
        serial_->println("Serial command too long.");
      }
    }
  }
}

void AsyncDebugTelemetry::taskEntry(void *argument)
{
  static_cast<AsyncDebugTelemetry *>(argument)->run();
}

void AsyncDebugTelemetry::run()
{
  AsyncFlightEvent event;
  TextLine textLine;
  AsyncFlightTelemetrySnapshot telemetry;
  AsyncTrackSample track;

  while (true)
  {
    // Core 0 owns both UART TX and RX. Wake periodically so serial commands
    // are collected without any Serial access from the 500 Hz flight core.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));

    pollSerialRx();

    // Events and command responses are printed before periodic telemetry.
    while (xQueueReceive(eventQueue_, &event, 0) == pdTRUE)
    {
      printEvent(event);
    }

    while (xQueueReceive(textQueue_, &textLine, 0) == pdTRUE)
    {
      serial_->println(textLine.text);
    }

    /*
      Track samples. Drained in order and never coalesced, because the whole
      point of the log is a uniformly sampled record. The RX poll inside the
      loop keeps serial commands responsive if a backlog has built up.
    */
    size_t trackDrained = 0;
    while (xQueueReceive(trackQueue_, &track, 0) == pdTRUE)
    {
      // Downlink first. It is the path that has a deadline: the transmitter
      // reconstructs sample timing from t_ms, but ESP-NOW queues are shallow
      // and a UART stall ahead of the send would show up as a downlink drop.
      if (trackSink_ != nullptr) trackSink_(track);
      if (trackSerialEnabled_) printTrack(track);

      if ((++trackDrained & 0x07u) == 0)
      {
        pollSerialRx();
      }
    }

    // Queue length is one, so this is always the most recent snapshot.
    if (xQueueReceive(telemetryQueue_, &telemetry, 0) == pdTRUE)
    {
      printTelemetry(telemetry);
    }

    pollSerialRx();

    // Let higher-priority Wi-Fi, ESP-NOW, and motor-service work run first.
    taskYIELD();
  }
}

/*
  One CSV row of the command-to-response chain.

  Every row is prefixed "TRK," so it can be extracted from a capture file that
  also holds ordinary telemetry, events and command output. The column names
  are emitted separately as a "#TRK" header line by the sketch when logging
  starts, so the analysis never hard-codes a column order.

  Formatting runs here, on core 0, for the reason given in the header.
*/
void AsyncDebugTelemetry::printTrack(const AsyncTrackSample &s)
{
  // The formatter lives in TrackSample.h so the transmitter emits byte-identical
  // rows when the log arrives over the downlink instead of the UART.
  char line[640];
  formatTrackRow(line, sizeof(line), s);
  serial_->println(line);
}

void AsyncDebugTelemetry::printTelemetry(
  const AsyncFlightTelemetrySnapshot &s
)
{
  serial_->print("LOST:"); serial_->print(s.signalLost ? 1 : 0);
  serial_->print(",IMU:"); serial_->print(s.imuValid ? 1 : 0);
  serial_->print(",STOP:"); serial_->print(s.safetyStop ? 1 : 0);
  serial_->print(",TILT:"); serial_->print(s.tiltStop ? 1 : 0);
  serial_->print(",EN:"); serial_->print(s.flightEnabled ? 1 : 0);
  serial_->print(",EOK:"); serial_->print(s.eskfStateHealthy ? 1 : 0);
  serial_->print(",EATT:"); serial_->print(s.eskfAttitudeUsed ? 1 : 0);
  serial_->print(",MFF:"); serial_->print(s.motionModelValid ? 1 : 0);
  serial_->print(",MSAT:"); serial_->print(s.motionModelSaturated ? 1 : 0);
  serial_->print(",RPMM:"); serial_->print(static_cast<unsigned>(s.rpmFreshMask), HEX);
  serial_->print(",RPMOK:"); serial_->print(s.rpmTelemetryHealthy ? 1 : 0);
  serial_->print(",RPMFZ:"); serial_->print(s.rotorForceZ_N, 2);
  serial_->print(",QPROC:"); serial_->print(s.eskfProcessNoiseScale, 2);
  serial_->print(",XY:"); serial_->print(s.xyHoldActive ? 1 : 0);
  serial_->print(",ALT:"); serial_->print(s.altitudeHoldActive ? 1 : 0);
  serial_->print(",RNGOK:"); serial_->print(s.rangeControlUsable ? 1 : 0);
  serial_->print(",RNGA:"); serial_->print(s.rangeAgeMs);
  serial_->print(",FLW:"); serial_->print(s.flowReady ? 1 : 0);
  serial_->print(",BASE:"); serial_->print(s.baseThrottleDshot);

  serial_->print(",RNG:"); serial_->print(s.rangeM, 3);
  serial_->print(",Z:"); serial_->print(s.altitudeM, 3);
  serial_->print(",ZT:"); serial_->print(s.altitudeTargetM, 3);
  serial_->print(",VZ:"); serial_->print(s.verticalVelocityMps, 3);
  serial_->print(",VZT:"); serial_->print(s.desiredVerticalVelocityMps, 3);
  serial_->print(",ZC:"); serial_->print(s.altitudeCorrectionDshot, 1);

  serial_->print(",X:"); serial_->print(s.positionX_M, 3);
  serial_->print(",XT:"); serial_->print(s.targetX_M, 3);
  serial_->print(",Y:"); serial_->print(s.positionY_M, 3);
  serial_->print(",YT:"); serial_->print(s.targetY_M, 3);
  serial_->print(",VX:"); serial_->print(s.measuredVelocityX_Mps, 3);
  serial_->print(",VXT:"); serial_->print(s.targetVelocityX_Mps, 3);
  serial_->print(",VY:"); serial_->print(s.measuredVelocityY_Mps, 3);
  serial_->print(",VYT:"); serial_->print(s.targetVelocityY_Mps, 3);
  serial_->print(",FVX:"); serial_->print(s.fusedVelocityX_Mps, 3);
  serial_->print(",FVY:"); serial_->print(s.fusedVelocityY_Mps, 3);
  serial_->print(",AX:"); serial_->print(s.linearAccelX_Mps2, 2);
  serial_->print(",AY:"); serial_->print(s.linearAccelY_Mps2, 2);

  serial_->print(",RA:"); serial_->print(s.rollAngleDeg, 2);
  serial_->print(",RAT:"); serial_->print(s.desiredRollAngleDeg, 2);
  serial_->print(",PA:"); serial_->print(s.pitchAngleDeg, 2);
  serial_->print(",PAT:"); serial_->print(s.desiredPitchAngleDeg, 2);
  serial_->print(",YR:"); serial_->print(s.yawRateDps, 2);
  serial_->print(",YC:"); serial_->print(s.yawCorrection, 1);
  serial_->print(",EIR:"); serial_->print(s.innovationRatio, 2);
  serial_->print(",FFC:"); serial_->print(s.modelCollectiveDshot, 1);
  serial_->print(",FFD1:"); serial_->print(s.modelMotorDeltaDshot[0], 1);
  serial_->print(",FFD2:"); serial_->print(s.modelMotorDeltaDshot[1], 1);
  serial_->print(",FFD3:"); serial_->print(s.modelMotorDeltaDshot[2], 1);
  serial_->print(",FFD4:"); serial_->print(s.modelMotorDeltaDshot[3], 1);

  serial_->print(",M1:"); serial_->print(s.motorCommand[0]);
  serial_->print(",M2:"); serial_->print(s.motorCommand[1]);
  serial_->print(",M3:"); serial_->print(s.motorCommand[2]);
  serial_->print(",M4:"); serial_->print(s.motorCommand[3]);
  serial_->print(",OVR:"); serial_->print(s.controlOverrunCount);
  serial_->print(",MAXDTUS:"); serial_->print(s.maximumControlDtUs);
  serial_->print(",MINDTUS:"); serial_->print(s.minimumControlDtUs);
  serial_->print(",DBGCORE:"); serial_->print(coreId_);
  serial_->print(",DBGDS:"); serial_->print(droppedSnapshots_);
  serial_->print(",DBGDE:"); serial_->print(droppedEvents_);
  serial_->print(",DBGDT:"); serial_->print(droppedTextLines_);
  serial_->print(",DBGDC:"); serial_->println(droppedCommands_);

  // Timing line is formatted entirely on the debug/UART core. Averages and
  // maxima cover the preceding one-second profiler window. Units are us.
  serial_->print("TIME,CTRLAVG:"); serial_->print(s.timingControlAvgUs);
  serial_->print(",CTRLMAX:"); serial_->print(s.timingControlMaxUs);
  serial_->print(",ROTAVG:"); serial_->print(s.timingRotorAvgUs);
  serial_->print(",ROTMAX:"); serial_->print(s.timingRotorMaxUs);
  serial_->print(",ATTAVG:"); serial_->print(s.timingAttitudeAvgUs);
  serial_->print(",ATTMAX:"); serial_->print(s.timingAttitudeMaxUs);
  serial_->print(",IMUAVG:"); serial_->print(s.timingImuAvgUs);
  serial_->print(",IMUMAX:"); serial_->print(s.timingImuMaxUs);
  serial_->print(",ESKFAVG:"); serial_->print(s.timingEskfAvgUs);
  serial_->print(",ESKFMAX:"); serial_->print(s.timingEskfMaxUs);
  serial_->print(",ENAVG:"); serial_->print(s.timingEnableAvgUs);
  serial_->print(",ENMAX:"); serial_->print(s.timingEnableMaxUs);
  serial_->print(",FCAVG:"); serial_->print(s.timingFlightControllerAvgUs);
  serial_->print(",FCMAX:"); serial_->print(s.timingFlightControllerMaxUs);
  serial_->print(",MPAVG:"); serial_->print(s.timingMotorPublishAvgUs);
  serial_->print(",MPMAX:"); serial_->print(s.timingMotorPublishMaxUs);
  serial_->print(",RNGAVG:"); serial_->print(s.timingRangeAvgUs);
  serial_->print(",RNGMAX:"); serial_->print(s.timingRangeMaxUs);
  serial_->print(",FLOWAVG:"); serial_->print(s.timingFlowAvgUs);
  serial_->print(",FLOWMAX:"); serial_->print(s.timingFlowMaxUs);
  serial_->print(",FLOWIOAVG:"); serial_->print(s.timingFlowIoAvgUs);
  serial_->print(",FLOWIOMAX:"); serial_->print(s.timingFlowIoMaxUs);
  serial_->print(",FLOWHZ:"); serial_->print(s.flowTaskHz, 1);
  serial_->print(",FLOWSKIP:"); serial_->print(s.flowSkippedSamples);
  serial_->print(",FLOWCORE:"); serial_->print(static_cast<int>(s.flowTaskCore));
  serial_->print(",CMDAVG:"); serial_->print(s.timingCommandAvgUs);
  serial_->print(",CMDMAX:"); serial_->print(s.timingCommandMaxUs);
  serial_->print(",RXCAVG:"); serial_->print(s.timingRxCopyAvgUs);
  serial_->print(",RXCMAX:"); serial_->print(s.timingRxCopyMaxUs);
  serial_->print(",LOOPAVG:"); serial_->print(s.timingLoopAvgUs);
  serial_->print(",LOOPMAX:"); serial_->print(s.timingLoopMaxUs);
  serial_->print(",PUBAVG:"); serial_->print(s.timingAsyncPublishAvgUs);
  serial_->print(",PUBMAX:"); serial_->print(s.timingAsyncPublishMaxUs);
  serial_->print(",ESPCBAVG:"); serial_->print(s.espNowCallbackAvgUs);
  serial_->print(",ESPCBMAX:"); serial_->print(s.espNowCallbackMaxUs);
  serial_->print(",ESPDT:"); serial_->print(s.espNowIntervalUs);
  serial_->print(",ESPDTMIN:"); serial_->print(s.espNowIntervalMinUs);
  serial_->print(",ESPDTMAX:"); serial_->print(s.espNowIntervalMaxUs);
  serial_->print(",ESPHZ:");
  if (s.espNowIntervalUs > 0)
  {
    serial_->print(1000000.0f / static_cast<float>(s.espNowIntervalUs), 1);
  }
  else
  {
    serial_->print(0.0f, 1);
  }
  serial_->print(",DSHZ:"); serial_->println(s.dshotServiceHz, 1);

  serial_->print("RPM,SEQ:"); serial_->print(s.rpmSequence);
  serial_->print(",MASK:"); serial_->print(static_cast<unsigned>(s.rpmFreshMask), HEX);
  serial_->print(",ALL:"); serial_->print(s.rpmTelemetryHealthy ? 1 : 0);
  serial_->print(",QPROC:"); serial_->print(s.eskfProcessNoiseScale, 2);
  serial_->print(",FZ_N:"); serial_->print(s.rotorForceZ_N, 3);
  serial_->print(",TX_NM:"); serial_->print(s.rotorTorqueX_Nm, 5);
  serial_->print(",TY_NM:"); serial_->print(s.rotorTorqueY_Nm, 5);

  for (int motor = 0; motor < 4; ++motor)
  {
    const AsyncRpmMotorSnapshot &m = s.rpmMotor[motor];
    serial_->print(" | M"); serial_->print(motor + 1);
    serial_->print(":c"); serial_->print(m.dshotCommand);
    serial_->print(" r"); serial_->print(m.rpmRaw, 0);
    serial_->print(" f"); serial_->print(m.rpmFiltered, 0);
    serial_->print(" w"); serial_->print(m.omegaRadPerSec, 1);
    serial_->print(" v"); serial_->print(m.valid ? 1 : 0);
    serial_->print(" lp"); serial_->print(m.lastPacketValid ? 1 : 0);
    serial_->print(" loss"); serial_->print(m.lost ? 1 : 0);
    serial_->print(" q"); serial_->print(m.packetQuality, 1);
    serial_->print(" a");
    if (m.ageUs == UINT32_MAX) serial_->print(-1);
    else serial_->print(m.ageUs);
    serial_->print(" s"); serial_->print(m.statusName);
    serial_->print(" src"); serial_->print(m.measuredSource ? 'M' : 'F');
    serial_->print(" k"); serial_->print(m.omegaPerDshot, 3);
  }
  serial_->println();
}

void AsyncDebugTelemetry::printEvent(const AsyncFlightEvent &event)
{
  switch (event.type)
  {
    case AsyncFlightEventType::XyHoldDisabledFlowTimeout:
      serial_->println("XY HOLD DISABLED: optical-flow timeout.");
      break;

    case AsyncFlightEventType::XyHoldActivated:
      serial_->print("XY HOLD ACTIVE at X=");
      serial_->print(event.value0, 3);
      serial_->print(", Y=");
      serial_->println(event.value1, 3);
      break;

    case AsyncFlightEventType::AltitudeHoldActivated:
      serial_->print("ALTITUDE HOLD ACTIVE at ");
      serial_->print(event.value0, 3);
      serial_->print(" m, hover DShot estimate ");
      serial_->println(event.integer0);
      break;

    case AsyncFlightEventType::RangeRecovered:
      serial_->print("RANGE RECOVERED: altitude target recaptured at ");
      serial_->print(event.value0, 3);
      serial_->println(" m.");
      break;

    case AsyncFlightEventType::DisableRampComplete:
      serial_->println("DISABLE RAMP COMPLETE: motors stopped.");
      break;

    case AsyncFlightEventType::EnableRampComplete:
      serial_->println("ENABLE RAMP COMPLETE: live throttle/altitude logic active.");
      break;

    case AsyncFlightEventType::RemoteEnableOff:
      serial_->print("REMOTE ENABLE OFF: ramping base DShot ");
      serial_->print(event.integer0);
      serial_->print(" -> ");
      serial_->print(event.integer1);
      serial_->print(" over ");
      serial_->print(event.unsigned0);
      serial_->println(" ms, then motor stop.");
      break;

    case AsyncFlightEventType::ImmediateFlightStop:
      serial_->print("IMMEDIATE FLIGHT STOP:");
      switch (static_cast<AsyncStopReason>(event.integer0))
      {
        case AsyncStopReason::RemoteKill: serial_->println("REMOTE_KILL"); break;
        case AsyncStopReason::SignalLost: serial_->println("SIGNAL_LOST"); break;
        case AsyncStopReason::ImuFailure: serial_->println("IMU_FAILURE"); break;
        case AsyncStopReason::ExcessiveTilt: serial_->println("EXCESSIVE_TILT"); break;
        default: serial_->println("SERIAL_STOP"); break;
      }
      break;

    case AsyncFlightEventType::EnableIgnoredDuringRampDown:
      serial_->println(
        "ENABLE IGNORED DURING RAMP DOWN: leave switch OFF until motors stop, then enable again."
      );
      break;

    case AsyncFlightEventType::RemoteEnableAccepted:
      serial_->print("REMOTE ENABLE ACCEPTED: ramping base DShot ");
      serial_->print(event.integer0);
      serial_->print(" -> ");
      serial_->print(event.integer1);
      serial_->print(" over ");
      serial_->print(event.unsigned0);
      serial_->println(" ms.");
      break;

    case AsyncFlightEventType::RemoteEnableRejected:
      serial_->print("REMOTE ENABLE REJECTED: center throttle first. THR:");
      serial_->println(event.value0, 3);
      break;
  }
}