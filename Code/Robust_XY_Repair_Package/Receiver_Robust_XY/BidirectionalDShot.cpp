#include "BidirectionalDShot.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "soc/gpio_struct.h"

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "The four-motor bidirectional DShot module requires the original ESP32 with eight RMT channels."
#endif

namespace
{
  const int8_t GCR_DECODE[32] = {
    -1, -1, -1, -1, -1, -1, -1, -1,
    -1,  9, 10, 11, -1, 13, 14, 15,
    -1, -1,  2,  3, -1,  5,  6,  7,
    -1,  0,  8,  1, -1,  4, 12, -1,
  };
}

BidirectionalDShot::BidirectionalDShot(
  const gpio_num_t motorPins[MOTOR_COUNT],
  const uint8_t motorPoleCounts[MOTOR_COUNT]
)
{
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    motorPins_[i] = motorPins[i];
    motorPoleCounts_[i] = motorPoleCounts[i];
    pendingCommands_[i] = 0;
  }
}

const char *BidirectionalDShot::statusName(TelemetryStatus status)
{
  switch (status)
  {
    case TelemetryStatus::OK: return "OK";
    case TelemetryStatus::RX_START_ERROR: return "RX_START";
    case TelemetryStatus::TX_ERROR: return "TX";
    case TelemetryStatus::TX_RELEASE_ERROR: return "RELEASE";
    case TelemetryStatus::RX_TIMEOUT: return "TIMEOUT";
    case TelemetryStatus::RX_EMPTY: return "EMPTY";
    case TelemetryStatus::NO_TURNAROUND_GAP: return "NO_GAP";
    case TelemetryStatus::NO_RESPONSE_EDGE: return "NO_REPLY";
    case TelemetryStatus::TOO_FEW_CELLS: return "SHORT";
    case TelemetryStatus::BAD_START_BIT: return "START";
    case TelemetryStatus::BAD_GCR_CODE: return "GCR";
    case TelemetryStatus::BAD_CRC: return "CRC";
    case TelemetryStatus::BAD_PERIOD: return "PERIOD";
  }
  return "UNKNOWN";
}

bool BidirectionalDShot::setError(const char *operation, esp_err_t error)
{
  snprintf(
    lastError_,
    sizeof(lastError_),
    "%s failed: %s (0x%X)",
    operation,
    esp_err_to_name(error),
    static_cast<unsigned>(error)
  );
  return false;
}

bool BidirectionalDShot::setErrorText(const char *text)
{
  snprintf(lastError_, sizeof(lastError_), "%s", text);
  return false;
}

uint16_t BidirectionalDShot::sanitizeCommand(uint16_t command)
{
  if (command == 0)
  {
    return 0;
  }

  // Values 1..47 are DShot special commands and are never emitted by this
  // flight-control motor-output path.
  if (command < 48)
  {
    return 0;
  }

  return command > 2047 ? 2047 : command;
}

bool BidirectionalDShot::begin()
{
  if (started_)
  {
    return true;
  }

  lastError_[0] = '\0';

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    const int pin = static_cast<int>(motorPins_[i]);
    if (pin < 0 || pin >= 32)
    {
      return setErrorText(
        "All four verified ESC pins must be original-ESP32 GPIOs below 32."
      );
    }

    if (motorPoleCounts_[i] < 2 || (motorPoleCounts_[i] & 1U) != 0U)
    {
      return setErrorText("Each motor pole count must be a positive even number.");
    }

    // Match the verified standalone startup: force every ESC signal LOW before
    // the RMT matrix owns the pad.
    gpio_reset_pin(motorPins_[i]);
    gpio_set_direction(motorPins_[i], GPIO_MODE_OUTPUT);
    gpio_set_level(motorPins_[i], 0);
  }

  if (!initializeRmt())
  {
    return false;
  }

  const BaseType_t taskResult = xTaskCreatePinnedToCore(
    serviceTaskThunk,
    "bdshot500",
    8192,
    this,
    4,
    &serviceTaskHandle_,
    0
  );

  if (taskResult != pdPASS || serviceTaskHandle_ == nullptr)
  {
    return setErrorText("Could not create the bidirectional DShot service task.");
  }

  esp_timer_create_args_t timerArguments = {};
  timerArguments.callback = frameTimerThunk;
  timerArguments.arg = this;
  timerArguments.dispatch_method = ESP_TIMER_TASK;
  timerArguments.name = "bdshot_tick";
  timerArguments.skip_unhandled_events = true;

  esp_err_t error = esp_timer_create(&timerArguments, &frameTimer_);
  if (error != ESP_OK)
  {
    return setError("esp_timer_create", error);
  }

  error = esp_timer_start_periodic(frameTimer_, FRAME_PERIOD_US);
  if (error != ESP_OK)
  {
    return setError("esp_timer_start_periodic", error);
  }

  started_ = true;

  // Send command zero immediately rather than waiting for the first timer tick.
  xTaskNotifyGive(serviceTaskHandle_);
  return true;
}

void BidirectionalDShot::setAllMotorCommands(
  const uint16_t commands[MOTOR_COUNT]
)
{
  portENTER_CRITICAL(&commandMux_);
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    pendingCommands_[i] = sanitizeCommand(commands[i]);
  }
  portEXIT_CRITICAL(&commandMux_);
}

void BidirectionalDShot::setMotorCommand(
  size_t motorIndex,
  uint16_t dshotCommand
)
{
  if (motorIndex >= MOTOR_COUNT)
  {
    return;
  }

  portENTER_CRITICAL(&commandMux_);
  pendingCommands_[motorIndex] = sanitizeCommand(dshotCommand);
  portEXIT_CRITICAL(&commandMux_);
}

void BidirectionalDShot::stopAll()
{
  const uint16_t stopCommands[MOTOR_COUNT] = {0, 0, 0, 0};
  setAllMotorCommands(stopCommands);

  if (serviceTaskHandle_ != nullptr)
  {
    // An extra zero frame is safe and reduces serial/local-kill latency.
    xTaskNotifyGive(serviceTaskHandle_);
  }
}

bool BidirectionalDShot::getSnapshot(RpmSnapshot &snapshot) const
{
  if (!started_)
  {
    snapshot = RpmSnapshot{};
    return false;
  }

  portENTER_CRITICAL(&snapshotMux_);
  memcpy(&snapshot, &publishedSnapshot_, sizeof(snapshot));
  portEXIT_CRITICAL(&snapshotMux_);

  const uint32_t nowUs = micros();
  snapshot.freshMask = 0;

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    MotorRpmSample &sample = snapshot.motor[i];

    if (sample.timestampUs == 0)
    {
      sample.ageUs = UINT32_MAX;
      sample.valid = false;
      sample.stale = true;
      sample.lost = true;
      continue;
    }

    sample.ageUs = static_cast<uint32_t>(nowUs - sample.timestampUs);
    sample.valid = sample.ageUs <= RPM_STALE_TIMEOUT_US;
    sample.stale = !sample.valid;
    sample.lost = sample.stale ||
      sample.consecutiveBad >= RPM_LOSS_COUNT;

    if (sample.valid && !sample.lost)
    {
      snapshot.freshMask |= static_cast<uint8_t>(1U << i);
    }
  }

  snapshot.allFresh = snapshot.freshMask == 0x0FU;
  return true;
}

bool BidirectionalDShot::getMotorSample(
  size_t motorIndex,
  MotorRpmSample &sample
) const
{
  if (motorIndex >= MOTOR_COUNT)
  {
    sample = MotorRpmSample{};
    return false;
  }

  RpmSnapshot snapshot;
  if (!getSnapshot(snapshot))
  {
    sample = MotorRpmSample{};
    return false;
  }

  sample = snapshot.motor[motorIndex];
  return true;
}

void BidirectionalDShot::getAllMotorSamples(
  MotorRpmSample samples[MOTOR_COUNT]
) const
{
  RpmSnapshot snapshot;
  getSnapshot(snapshot);
  memcpy(samples, snapshot.motor, sizeof(snapshot.motor));
}

void BidirectionalDShot::serviceTaskThunk(void *argument)
{
  static_cast<BidirectionalDShot *>(argument)->serviceTask();
}

void BidirectionalDShot::serviceTask()
{
  while (true)
  {
    // The periodic esp_timer and emergency stop use the same notification.
    // pdTRUE collapses accumulated ticks so a delayed task never emits a burst
    // of old motor frames.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    runCycle();
  }
}

void BidirectionalDShot::frameTimerThunk(void *argument)
{
  BidirectionalDShot *instance =
    static_cast<BidirectionalDShot *>(argument);

  if (instance != nullptr && instance->serviceTaskHandle_ != nullptr)
  {
    xTaskNotifyGive(instance->serviceTaskHandle_);
  }
}

bool IRAM_ATTR BidirectionalDShot::onRmtRxDone(
  rmt_channel_handle_t,
  const rmt_rx_done_event_data_t *eventData,
  void *userData
)
{
  MotorChannel *motor = static_cast<MotorChannel *>(userData);
  motor->rxCount = eventData->num_symbols;

  BaseType_t higherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(motor->rxDone, &higherPriorityTaskWoken);
  return higherPriorityTaskWoken == pdTRUE;
}

bool IRAM_ATTR BidirectionalDShot::onRmtTxDone(
  rmt_channel_handle_t,
  const rmt_tx_done_event_data_t *,
  void *userData
)
{
  MotorChannel *motor = static_cast<MotorChannel *>(userData);
  releaseEscSignal(*motor);
  return false;
}

inline void IRAM_ATTR BidirectionalDShot::releaseEscSignal(
  MotorChannel &motor
)
{
  const uint32_t gpio = static_cast<uint32_t>(motor.pin);
  GPIO.enable_w1tc = 1UL << gpio;
  motor.txOutputReleased = true;
}

inline void BidirectionalDShot::enableEscSignalOutput(
  MotorChannel &motor
)
{
  const uint32_t gpio = static_cast<uint32_t>(motor.pin);
  motor.txOutputReleased = false;
  GPIO.enable_w1ts = 1UL << gpio;
}

bool BidirectionalDShot::initializeRmt()
{
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    motors_[i].owner = this;
    motors_[i].index = static_cast<uint8_t>(i);
    motors_[i].pin = motorPins_[i];
    motors_[i].poles = motorPoleCounts_[i];
    motors_[i].rxDone = xSemaphoreCreateBinary();

    if (motors_[i].rxDone == nullptr)
    {
      return setErrorText("Could not allocate an RMT RX completion semaphore.");
    }
  }

  // Allocate all four RX channels first, exactly as in the verified sketch.
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    rmt_rx_channel_config_t rxConfig = {};
    rxConfig.gpio_num = motors_[i].pin;
    rxConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    rxConfig.resolution_hz = RMT_RESOLUTION_HZ;
    rxConfig.mem_block_symbols = RX_SYMBOL_CAPACITY;
    rxConfig.intr_priority = 0;
    rxConfig.flags.invert_in = 0;
    rxConfig.flags.with_dma = 0;
    rxConfig.flags.io_loop_back = 0;
    rxConfig.flags.allow_pd = 0;

    esp_err_t error = rmt_new_rx_channel(&rxConfig, &motors_[i].rx);
    if (error != ESP_OK)
    {
      return setError("rmt_new_rx_channel", error);
    }

    rmt_rx_event_callbacks_t callbacks = {};
    callbacks.on_recv_done = onRmtRxDone;
    error = rmt_rx_register_event_callbacks(
      motors_[i].rx,
      &callbacks,
      &motors_[i]
    );
    if (error != ESP_OK)
    {
      return setError("rmt_rx_register_event_callbacks", error);
    }
  }

  // Allocate all four TX channels on the same four pins.
  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    rmt_tx_channel_config_t txConfig = {};
    txConfig.gpio_num = motors_[i].pin;
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.resolution_hz = RMT_RESOLUTION_HZ;
    txConfig.mem_block_symbols = RX_SYMBOL_CAPACITY;
    txConfig.trans_queue_depth = 1;
    txConfig.intr_priority = 0;
    txConfig.flags.invert_out = 0;
    txConfig.flags.with_dma = 0;
    txConfig.flags.io_loop_back = 0;
    txConfig.flags.io_od_mode = 0;
    txConfig.flags.allow_pd = 0;
    txConfig.flags.init_level = 1;

    esp_err_t error = rmt_new_tx_channel(&txConfig, &motors_[i].tx);
    if (error != ESP_OK)
    {
      return setError("rmt_new_tx_channel", error);
    }

    // Critical Bluejay half-duplex fix: GPIO ENABLE, not the peripheral,
    // controls pad output-enable. RMT still supplies the waveform data.
    const uint32_t gpio = static_cast<uint32_t>(motors_[i].pin);
    GPIO.func_out_sel_cfg[gpio].oen_sel = 1;
    GPIO.func_out_sel_cfg[gpio].oen_inv_sel = 0;

    rmt_tx_event_callbacks_t txCallbacks = {};
    txCallbacks.on_trans_done = onRmtTxDone;
    error = rmt_tx_register_event_callbacks(
      motors_[i].tx,
      &txCallbacks,
      &motors_[i]
    );
    if (error != ESP_OK)
    {
      return setError("rmt_tx_register_event_callbacks", error);
    }

    releaseEscSignal(motors_[i]);

    rmt_copy_encoder_config_t copyConfig = {};
    error = rmt_new_copy_encoder(&copyConfig, &motors_[i].copyEncoder);
    if (error != ESP_OK)
    {
      return setError("rmt_new_copy_encoder", error);
    }
  }

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    esp_err_t error = rmt_enable(motors_[i].rx);
    if (error != ESP_OK)
    {
      return setError("rmt_enable RX", error);
    }

    error = rmt_enable(motors_[i].tx);
    if (error != ESP_OK)
    {
      return setError("rmt_enable TX", error);
    }

    error = gpio_input_enable(motors_[i].pin);
    if (error != ESP_OK)
    {
      return setError("gpio_input_enable", error);
    }

    gpio_pulldown_dis(motors_[i].pin);
    gpio_pullup_en(motors_[i].pin);
    enableEscSignalOutput(motors_[i]);

    const uint32_t gpio = static_cast<uint32_t>(motors_[i].pin);
    if (GPIO.func_out_sel_cfg[gpio].oen_sel != 1)
    {
      return setErrorText("GPIO matrix OEN_SEL did not switch to GPIO control.");
    }
  }

  return true;
}

uint16_t BidirectionalDShot::makeBidirectionalDshotFrame(uint16_t command)
{
  const uint16_t payload12 =
    static_cast<uint16_t>((command & 0x07FFU) << 1U);

  const uint8_t checksum = static_cast<uint8_t>(
    (~(payload12 ^ (payload12 >> 4U) ^ (payload12 >> 8U))) & 0x0FU
  );

  return static_cast<uint16_t>((payload12 << 4U) | checksum);
}

void BidirectionalDShot::buildDshotSymbols(MotorChannel &motor)
{
  const uint16_t frame = makeBidirectionalDshotFrame(motor.command);

  for (size_t bit = 0; bit < 16; ++bit)
  {
    const bool logicalOne = (frame & (0x8000U >> bit)) != 0;
    const uint16_t lowTicks = logicalOne
      ? DSHOT_ONE_LOW_TICKS
      : DSHOT_ZERO_LOW_TICKS;

    motor.txSymbols[bit].level0 = 0;
    motor.txSymbols[bit].duration0 = lowTicks;
    motor.txSymbols[bit].level1 = 1;
    motor.txSymbols[bit].duration1 = DSHOT_BIT_TICKS - lowTicks;
  }

  motor.txSymbols[16].level0 = 1;
  motor.txSymbols[16].duration0 = DSHOT_END_HOLD_TICKS - 1U;
  motor.txSymbols[16].level1 = 1;
  motor.txSymbols[16].duration1 = 1U;
}

bool BidirectionalDShot::runCycle()
{
  uint16_t commands[MOTOR_COUNT] = {};
  portENTER_CRITICAL(&commandMux_);
  memcpy(commands, pendingCommands_, sizeof(commands));
  portEXIT_CRITICAL(&commandMux_);

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    motors_[i].command = commands[i];
    buildDshotSymbols(motors_[i]);
  }

  rmt_receive_config_t receiveConfig = {};
  receiveConfig.signal_range_min_ns = RX_MIN_PULSE_NS;
  receiveConfig.signal_range_max_ns = RX_IDLE_STOP_NS;
  receiveConfig.flags.en_partial_rx = 0;

  rmt_transmit_config_t transmitConfig = {};
  transmitConfig.loop_count = 0;
  transmitConfig.flags.eot_level = 1;
  transmitConfig.flags.queue_nonblocking = 0;

  bool rxArmed[MOTOR_COUNT] = {};
  bool txQueued[MOTOR_COUNT] = {};
  bool rxCompleted[MOTOR_COUNT] = {};

  const uint32_t transactionStartUs = micros();

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    while (xSemaphoreTake(motors_[i].rxDone, 0) == pdTRUE)
    {
      // Drain a stale token.
    }

    motors_[i].rxCount = 0;
    esp_err_t error = rmt_receive(
      motors_[i].rx,
      motors_[i].rxSymbols,
      sizeof(motors_[i].rxSymbols),
      &receiveConfig
    );

    rxArmed[i] = error == ESP_OK;
    if (!rxArmed[i])
    {
      registerPacketResult(
        motors_[i],
        TelemetryStatus::RX_START_ERROR,
        transactionStartUs
      );
      continue;
    }

    enableEscSignalOutput(motors_[i]);

    error = rmt_transmit(
      motors_[i].tx,
      motors_[i].copyEncoder,
      motors_[i].txSymbols,
      sizeof(motors_[i].txSymbols),
      &transmitConfig
    );

    txQueued[i] = error == ESP_OK;
    if (!txQueued[i])
    {
      releaseEscSignal(motors_[i]);
      registerPacketResult(
        motors_[i],
        TelemetryStatus::TX_ERROR,
        transactionStartUs
      );
    }
  }

  // Hardware generates the whole command. Poll only the TX-complete flags;
  // this task is isolated from the 250 Hz controller on the other core.
  const uint32_t txDeadlineUs = transactionStartUs + 350U;
  bool allTxReleased = false;
  while (!allTxReleased &&
         static_cast<int32_t>(micros() - txDeadlineUs) < 0)
  {
    allTxReleased = true;
    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
      if (txQueued[i] && !motors_[i].txOutputReleased)
      {
        allTxReleased = false;
      }
    }
    if (!allTxReleased)
    {
      taskYIELD();
    }
  }

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    if (!txQueued[i])
    {
      continue;
    }

    const esp_err_t waitError = rmt_tx_wait_all_done(motors_[i].tx, 1);
    if (waitError != ESP_OK)
    {
      releaseEscSignal(motors_[i]);
      txQueued[i] = false;
      registerPacketResult(
        motors_[i],
        TelemetryStatus::TX_ERROR,
        transactionStartUs
      );
    }
    else if (!motors_[i].txOutputReleased)
    {
      releaseEscSignal(motors_[i]);
      txQueued[i] = false;
      registerPacketResult(
        motors_[i],
        TelemetryStatus::TX_RELEASE_ERROR,
        transactionStartUs
      );
    }
  }

  const uint32_t transactionDeadlineUs =
    transactionStartUs + TRANSACTION_DEADLINE_US;

  bool allExpectedRxCompleted = false;
  while (!allExpectedRxCompleted &&
         static_cast<int32_t>(micros() - transactionDeadlineUs) < 0)
  {
    allExpectedRxCompleted = true;

    for (size_t i = 0; i < MOTOR_COUNT; ++i)
    {
      if (!rxArmed[i] || !txQueued[i] || rxCompleted[i])
      {
        continue;
      }

      if (xSemaphoreTake(motors_[i].rxDone, 0) == pdTRUE)
      {
        rxCompleted[i] = true;
      }
      else
      {
        allExpectedRxCompleted = false;
      }
    }

    if (!allExpectedRxCompleted)
    {
      taskYIELD();
    }
  }

  const uint32_t transactionEndUs = micros();
  const uint32_t transactionTimestampUs = transactionStartUs +
    static_cast<uint32_t>(transactionEndUs - transactionStartUs) / 2U;

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    if (!rxArmed[i] || !txQueued[i])
    {
      if (rxArmed[i] && !txQueued[i])
      {
        recoverRxChannel(motors_[i]);
      }
      enableEscSignalOutput(motors_[i]);
      continue;
    }

    if (!rxCompleted[i])
    {
      motors_[i].timeouts++;
      registerPacketResult(
        motors_[i],
        TelemetryStatus::RX_TIMEOUT,
        transactionTimestampUs
      );
      recoverRxChannel(motors_[i]);
      enableEscSignalOutput(motors_[i]);
      continue;
    }

    enableEscSignalOutput(motors_[i]);

    const TelemetryStatus status = decodeTelemetry(motors_[i]);
    registerPacketResult(
      motors_[i],
      status,
      transactionTimestampUs
    );
  }

  publishSnapshot(commands, transactionTimestampUs);
  return true;
}

void BidirectionalDShot::recoverRxChannel(MotorChannel &motor)
{
  // A timeout is not expected with the 300 us idle threshold, but explicitly
  // restart the channel so one anomalous transaction cannot poison later ones.
  rmt_disable(motor.rx);
  rmt_enable(motor.rx);

  while (xSemaphoreTake(motor.rxDone, 0) == pdTRUE)
  {
  }
  motor.rxCount = 0;
}

void BidirectionalDShot::appendRun(
  Run *runs,
  size_t &runCount,
  uint8_t level,
  uint32_t ticks
)
{
  if (ticks == 0 || runCount >= MAX_RUNS)
  {
    return;
  }

  if (runCount > 0 && runs[runCount - 1].level == level)
  {
    runs[runCount - 1].ticks += ticks;
  }
  else
  {
    runs[runCount].level = level;
    runs[runCount].ticks = ticks;
    runCount++;
  }
}

BidirectionalDShot::TelemetryStatus
BidirectionalDShot::decodePhysicalPacket(
  MotorChannel &motor,
  uint32_t physical21
)
{
  if ((physical21 & (1UL << 20U)) != 0)
  {
    return TelemetryStatus::BAD_START_BIT;
  }

  const uint32_t gcr20 =
    (physical21 ^ (physical21 >> 1U)) & 0x000FFFFFUL;

  uint32_t decoded16 = 0;
  for (size_t group = 0; group < 4; ++group)
  {
    const uint8_t code = static_cast<uint8_t>(
      (gcr20 >> (group * 5U)) & 0x1FU
    );

    const int8_t nibble = GCR_DECODE[code];
    if (nibble < 0)
    {
      return TelemetryStatus::BAD_GCR_CODE;
    }

    decoded16 |= static_cast<uint32_t>(nibble) << (group * 4U);
  }

  uint32_t checksumFold = decoded16;
  checksumFold ^= checksumFold >> 8U;
  checksumFold ^= checksumFold >> 4U;
  if ((checksumFold & 0x0FU) != 0x0FU)
  {
    return TelemetryStatus::BAD_CRC;
  }

  const uint16_t raw12 = static_cast<uint16_t>(decoded16 >> 4U);
  motor.rawTelemetry = raw12;

  if (raw12 == 0x0FFFU)
  {
    motor.electricalPeriodUs = 0;
    motor.rpmRaw = 0.0f;
    motor.rpmFiltered *= (1.0f - RPM_FILTER_ALPHA);
    if (motor.rpmFiltered < 1.0f)
    {
      motor.rpmFiltered = 0.0f;
    }
    return TelemetryStatus::OK;
  }

  const uint8_t exponent = static_cast<uint8_t>((raw12 >> 9U) & 0x07U);
  const uint16_t mantissa = static_cast<uint16_t>(raw12 & 0x01FFU);
  const uint32_t electricalPeriodUs =
    static_cast<uint32_t>(mantissa) << exponent;

  if (electricalPeriodUs == 0 || motor.poles < 2 ||
      (motor.poles & 1U) != 0U)
  {
    return TelemetryStatus::BAD_PERIOD;
  }

  motor.electricalPeriodUs = electricalPeriodUs;

  const float electricalRpm =
    60000000.0f / static_cast<float>(electricalPeriodUs);
  const float polePairs = static_cast<float>(motor.poles) * 0.5f;
  motor.rpmRaw = electricalRpm / polePairs;

  if (!motor.hasValidSample)
  {
    motor.rpmFiltered = motor.rpmRaw;
  }
  else
  {
    motor.rpmFiltered +=
      RPM_FILTER_ALPHA * (motor.rpmRaw - motor.rpmFiltered);
  }

  return TelemetryStatus::OK;
}

BidirectionalDShot::TelemetryStatus
BidirectionalDShot::decodeRunsFromIndex(
  MotorChannel &motor,
  const Run *runs,
  size_t runCount,
  size_t startIndex
)
{
  uint32_t physical21 = 0;
  size_t physicalBitCount = 0;

  for (size_t i = startIndex;
       i < runCount && physicalBitCount < TELEMETRY_PHYSICAL_BITS;
       ++i)
  {
    uint32_t cells = static_cast<uint32_t>(
      (static_cast<uint64_t>(runs[i].ticks) * TELEMETRY_BITRATE_HZ +
       (RMT_RESOLUTION_HZ / 2U)) /
      RMT_RESOLUTION_HZ
    );

    if (cells == 0)
    {
      continue;
    }

    const size_t remaining =
      TELEMETRY_PHYSICAL_BITS - physicalBitCount;
    if (cells > remaining)
    {
      cells = static_cast<uint32_t>(remaining);
    }

    for (uint32_t cell = 0; cell < cells; ++cell)
    {
      physical21 =
        (physical21 << 1U) | (runs[i].level ? 1U : 0U);
      physicalBitCount++;
    }
  }

  if (physicalBitCount < MIN_INFERABLE_TELEMETRY_BITS)
  {
    return TelemetryStatus::TOO_FEW_CELLS;
  }

  while (physicalBitCount < TELEMETRY_PHYSICAL_BITS)
  {
    physical21 = (physical21 << 1U) | 1U;
    physicalBitCount++;
  }

  return decodePhysicalPacket(motor, physical21);
}

BidirectionalDShot::TelemetryStatus
BidirectionalDShot::decodeTelemetry(MotorChannel &motor)
{
  Run runs[MAX_RUNS] = {};
  size_t runCount = 0;

  motor.lastRunCount = 0;
  motor.lastLowRuns = 0;
  motor.lastCaptureTicks = 0;
  motor.lastReplyRun = -1;
  motor.decodeMode = '-';

  const size_t symbolCount = motor.rxCount < RX_SYMBOL_CAPACITY
    ? motor.rxCount
    : RX_SYMBOL_CAPACITY;

  for (size_t i = 0; i < symbolCount; ++i)
  {
    const rmt_symbol_word_t &symbol = motor.rxSymbols[i];
    appendRun(runs, runCount, symbol.level0, symbol.duration0);
    appendRun(runs, runCount, symbol.level1, symbol.duration1);
  }

  if (runCount == 0)
  {
    return TelemetryStatus::RX_EMPTY;
  }

  size_t lowRunsSeen = 0;
  for (size_t i = 0; i < runCount; ++i)
  {
    motor.lastCaptureTicks += runs[i].ticks;
    if (runs[i].level == 0)
    {
      lowRunsSeen++;
    }
  }

  motor.lastRunCount = static_cast<uint16_t>(runCount);
  motor.lastLowRuns = static_cast<uint16_t>(lowRunsSeen);

  bool foundGap = false;
  TelemetryStatus firstGapError = TelemetryStatus::NO_RESPONSE_EDGE;

  for (size_t i = 0; i + 1 < runCount; ++i)
  {
    if (runs[i].level == 1 &&
        runs[i].ticks >= RESPONSE_GAP_MIN_TICKS &&
        runs[i + 1].level == 0)
    {
      foundGap = true;
      const TelemetryStatus status =
        decodeRunsFromIndex(motor, runs, runCount, i + 1U);

      if (status == TelemetryStatus::OK)
      {
        motor.lastReplyRun = static_cast<int16_t>(i + 1U);
        motor.decodeMode = 'G';
        return status;
      }

      if (firstGapError == TelemetryStatus::NO_RESPONSE_EDGE)
      {
        firstGapError = status;
      }
    }
  }

  TelemetryStatus firstCandidateError =
    TelemetryStatus::NO_TURNAROUND_GAP;
  bool testedCandidate = false;

  for (size_t i = 0; i < runCount; ++i)
  {
    if (runs[i].level != 0)
    {
      continue;
    }

    testedCandidate = true;
    const TelemetryStatus status =
      decodeRunsFromIndex(motor, runs, runCount, i);

    if (status == TelemetryStatus::OK)
    {
      motor.lastReplyRun = static_cast<int16_t>(i);
      motor.decodeMode = 'R';
      return status;
    }

    if (firstCandidateError == TelemetryStatus::NO_TURNAROUND_GAP &&
        status != TelemetryStatus::TOO_FEW_CELLS)
    {
      firstCandidateError = status;
    }
  }

  if (foundGap)
  {
    return firstGapError;
  }

  if (lowRunsSeen >= 16U)
  {
    return TelemetryStatus::NO_RESPONSE_EDGE;
  }

  if (testedCandidate &&
      firstCandidateError != TelemetryStatus::NO_TURNAROUND_GAP)
  {
    return firstCandidateError;
  }

  return TelemetryStatus::NO_TURNAROUND_GAP;
}

void BidirectionalDShot::registerPacketResult(
  MotorChannel &motor,
  TelemetryStatus status,
  uint32_t sampleTimestampUs
)
{
  motor.status = status;
  motor.lastPacketValid = status == TelemetryStatus::OK;

  const bool good = motor.lastPacketValid;
  motor.qualityHistory = (motor.qualityHistory << 1U) | (good ? 1ULL : 0ULL);
  if (motor.qualitySampleCount < 64U)
  {
    motor.qualitySampleCount++;
  }

  if (good)
  {
    motor.goodPackets++;
    motor.consecutiveGood = motor.consecutiveGood < UINT16_MAX
      ? static_cast<uint16_t>(motor.consecutiveGood + 1U)
      : UINT16_MAX;
    motor.consecutiveBad = 0;
    motor.hasValidSample = true;
    motor.lastValidTimestampUs = sampleTimestampUs;
  }
  else
  {
    motor.badPackets++;
    motor.consecutiveBad = motor.consecutiveBad < UINT16_MAX
      ? static_cast<uint16_t>(motor.consecutiveBad + 1U)
      : UINT16_MAX;
    motor.consecutiveGood = 0;
  }
}

void BidirectionalDShot::publishSnapshot(
  const uint16_t commands[MOTOR_COUNT],
  uint32_t transactionTimestampUs
)
{
  RpmSnapshot snapshot;
  snapshot.snapshotTimestampUs = transactionTimestampUs;
  snapshot.sequence = ++snapshotSequence_;
  snapshot.freshMask = 0;

  for (size_t i = 0; i < MOTOR_COUNT; ++i)
  {
    const MotorChannel &motor = motors_[i];
    MotorRpmSample &sample = snapshot.motor[i];

    sample.dshotCommand = commands[i];
    sample.rawTelemetry = motor.rawTelemetry;
    sample.electricalPeriodUs = motor.electricalPeriodUs;
    sample.rpmRaw = motor.rpmRaw;
    sample.rpmFiltered = motor.rpmFiltered;
    sample.omegaRadPerSec = motor.rpmFiltered * TWO_PI_OVER_60;
    sample.timestampUs = motor.lastValidTimestampUs;

    if (motor.hasValidSample)
    {
      sample.ageUs = static_cast<uint32_t>(
        transactionTimestampUs - motor.lastValidTimestampUs
      );
      sample.valid = sample.ageUs <= RPM_STALE_TIMEOUT_US;
      sample.stale = !sample.valid;
    }
    else
    {
      sample.ageUs = UINT32_MAX;
      sample.valid = false;
      sample.stale = true;
    }

    sample.lost = sample.stale ||
      motor.consecutiveBad >= RPM_LOSS_COUNT;

    if (sample.valid && !sample.lost)
    {
      snapshot.freshMask |= static_cast<uint8_t>(1U << i);
    }

    sample.lastPacketValid = motor.lastPacketValid;
    sample.goodPackets = motor.goodPackets;
    sample.badPackets = motor.badPackets;
    sample.timeouts = motor.timeouts;
    sample.consecutiveGood = motor.consecutiveGood;
    sample.consecutiveBad = motor.consecutiveBad;
    sample.packetQuality = rollingQualityPercent(motor);
    sample.status = motor.status;
    sample.capturedSymbols = static_cast<uint16_t>(motor.rxCount);
    sample.capturedRuns = motor.lastRunCount;
    sample.decodeMode = motor.decodeMode;
  }

  snapshot.allFresh = snapshot.freshMask == 0x0FU;

  portENTER_CRITICAL(&snapshotMux_);
  memcpy(&publishedSnapshot_, &snapshot, sizeof(snapshot));
  portEXIT_CRITICAL(&snapshotMux_);
}

uint8_t BidirectionalDShot::popcount64(uint64_t value)
{
  uint8_t count = 0;
  while (value != 0)
  {
    value &= value - 1ULL;
    count++;
  }
  return count;
}

float BidirectionalDShot::rollingQualityPercent(const MotorChannel &motor)
{
  if (motor.qualitySampleCount == 0)
  {
    return 0.0f;
  }

  uint64_t mask = UINT64_MAX;
  if (motor.qualitySampleCount < 64U)
  {
    mask = (1ULL << motor.qualitySampleCount) - 1ULL;
  }

  const uint8_t good = popcount64(motor.qualityHistory & mask);
  return 100.0f * static_cast<float>(good) /
    static_cast<float>(motor.qualitySampleCount);
}
