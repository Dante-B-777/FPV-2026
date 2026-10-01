#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/*
  Four-channel Bluejay bidirectional DShot300 service for the original ESP32.

  This module preserves the verified half-duplex implementation:
    - four RMT TX + four RMT RX channels;
    - 40 MHz RMT clock;
    - inverted bidirectional DShot checksum/waveform;
    - same-pin receive;
    - one-cell HIGH hold after bit 16;
    - manual GPIO-matrix OEN control;
    - TX-complete tri-state;
    - GCR/NRZI/checksum/period decoding.

  A dedicated FreeRTOS task performs the 500 Hz DShot transaction. The flight
  controller only writes a coherent four-command buffer and reads the latest
  completed RPM snapshot; it never waits for an ESC response.
*/
class BidirectionalDShot
{
public:
  static constexpr size_t MOTOR_COUNT = 4;
  static constexpr uint32_t FRAME_PERIOD_US = 2000;       // 500 Hz
  static constexpr uint32_t RPM_STALE_TIMEOUT_US = 20000; // 10 missed frames
  static constexpr uint16_t RPM_LOSS_COUNT = 5;           // consecutive failures

  enum class TelemetryStatus : uint8_t
  {
    OK,
    RX_START_ERROR,
    TX_ERROR,
    TX_RELEASE_ERROR,
    RX_TIMEOUT,
    RX_EMPTY,
    NO_TURNAROUND_GAP,
    NO_RESPONSE_EDGE,
    TOO_FEW_CELLS,
    BAD_START_BIT,
    BAD_GCR_CODE,
    BAD_CRC,
    BAD_PERIOD,
  };

  struct MotorRpmSample
  {
    uint16_t dshotCommand = 0;
    uint16_t rawTelemetry = 0x0FFF;
    uint32_t electricalPeriodUs = 0;

    float rpmRaw = 0.0f;
    float rpmFiltered = 0.0f;
    float omegaRadPerSec = 0.0f;

    uint32_t timestampUs = 0; // midpoint of the most recent valid transaction
    uint32_t ageUs = UINT32_MAX;

    bool lastPacketValid = false;
    bool valid = false; // last valid sample is not stale
    bool stale = true;
    bool lost = true;

    uint32_t goodPackets = 0;
    uint32_t badPackets = 0;
    uint32_t timeouts = 0;
    uint16_t consecutiveGood = 0;
    uint16_t consecutiveBad = 0;
    float packetQuality = 0.0f; // rolling 64-packet percentage

    TelemetryStatus status = TelemetryStatus::RX_TIMEOUT;
    uint16_t capturedSymbols = 0;
    uint16_t capturedRuns = 0;
    char decodeMode = '-';
  };

  struct RpmSnapshot
  {
    MotorRpmSample motor[MOTOR_COUNT] = {};
    uint32_t snapshotTimestampUs = 0;
    uint32_t sequence = 0;
    uint8_t freshMask = 0;
    bool allFresh = false;
  };

  BidirectionalDShot(
    const gpio_num_t motorPins[MOTOR_COUNT],
    const uint8_t motorPoleCounts[MOTOR_COUNT]
  );

  // Call once from setup(). It begins transmitting command 0 immediately.
  bool begin();

  // Commands are copied atomically as a coherent four-motor set.
  void setAllMotorCommands(const uint16_t commands[MOTOR_COUNT]);
  void setMotorCommand(size_t motorIndex, uint16_t dshotCommand);

  // Sets all commands to zero and wakes the service task for a prompt frame.
  void stopAll();

  bool getMotorSample(size_t motorIndex, MotorRpmSample &sample) const;
  void getAllMotorSamples(MotorRpmSample samples[MOTOR_COUNT]) const;
  bool getSnapshot(RpmSnapshot &snapshot) const;

  bool started() const { return started_; }
  const char *lastError() const { return lastError_; }

  static const char *statusName(TelemetryStatus status);

private:
  static constexpr uint32_t RMT_RESOLUTION_HZ = 40000000UL;
  static constexpr uint32_t DSHOT_BITRATE_HZ = 300000UL;
  static constexpr uint32_t TELEMETRY_BITRATE_HZ =
    (DSHOT_BITRATE_HZ * 5UL) / 4UL;

  static constexpr uint16_t DSHOT_BIT_TICKS = 133;
  static constexpr uint16_t DSHOT_ONE_LOW_TICKS = 100;
  static constexpr uint16_t DSHOT_ZERO_LOW_TICKS = 50;
  static constexpr uint16_t DSHOT_END_HOLD_TICKS = DSHOT_BIT_TICKS;
  static constexpr size_t DSHOT_TX_SYMBOL_COUNT = 17;

  static constexpr uint32_t RESPONSE_GAP_MIN_TICKS =
    (RMT_RESOLUTION_HZ / 1000000UL) * 10UL;

  static constexpr uint32_t RX_MIN_PULSE_NS = 150;
  static constexpr uint32_t RX_IDLE_STOP_NS = 300000;
  static constexpr uint32_t TRANSACTION_DEADLINE_US = 1200;

  static constexpr size_t RX_SYMBOL_CAPACITY = 64;
  static constexpr size_t MAX_RUNS = RX_SYMBOL_CAPACITY * 2 + 2;
  static constexpr size_t TELEMETRY_PHYSICAL_BITS = 21;
  static constexpr size_t MIN_INFERABLE_TELEMETRY_BITS = 18;

  static constexpr float RPM_FILTER_ALPHA = 0.25f;
  static constexpr float TWO_PI_OVER_60 =
    0.10471975511965977462f;

  struct Run
  {
    uint8_t level = 0;
    uint32_t ticks = 0;
  };

  struct MotorChannel
  {
    BidirectionalDShot *owner = nullptr;
    uint8_t index = 0;
    gpio_num_t pin = GPIO_NUM_NC;
    uint8_t poles = 14;

    rmt_channel_handle_t rx = nullptr;
    rmt_channel_handle_t tx = nullptr;
    rmt_encoder_handle_t copyEncoder = nullptr;
    SemaphoreHandle_t rxDone = nullptr;

    volatile size_t rxCount = 0;
    volatile bool txOutputReleased = true;

    uint16_t lastRunCount = 0;
    uint16_t lastLowRuns = 0;
    uint32_t lastCaptureTicks = 0;
    int16_t lastReplyRun = -1;
    char decodeMode = '-';

    rmt_symbol_word_t rxSymbols[RX_SYMBOL_CAPACITY] = {};
    rmt_symbol_word_t txSymbols[DSHOT_TX_SYMBOL_COUNT] = {};

    uint16_t command = 0;
    uint16_t rawTelemetry = 0x0FFF;
    uint32_t electricalPeriodUs = 0;
    float rpmRaw = 0.0f;
    float rpmFiltered = 0.0f;

    bool hasValidSample = false;
    bool lastPacketValid = false;
    uint32_t lastValidTimestampUs = 0;

    uint32_t goodPackets = 0;
    uint32_t badPackets = 0;
    uint32_t timeouts = 0;
    uint16_t consecutiveGood = 0;
    uint16_t consecutiveBad = 0;

    uint64_t qualityHistory = 0;
    uint8_t qualitySampleCount = 0;

    TelemetryStatus status = TelemetryStatus::RX_TIMEOUT;
  };

  gpio_num_t motorPins_[MOTOR_COUNT] = {};
  uint8_t motorPoleCounts_[MOTOR_COUNT] = {};
  MotorChannel motors_[MOTOR_COUNT] = {};

  mutable portMUX_TYPE commandMux_ = portMUX_INITIALIZER_UNLOCKED;
  mutable portMUX_TYPE snapshotMux_ = portMUX_INITIALIZER_UNLOCKED;

  uint16_t pendingCommands_[MOTOR_COUNT] = {};
  RpmSnapshot publishedSnapshot_ = {};
  uint32_t snapshotSequence_ = 0;

  TaskHandle_t serviceTaskHandle_ = nullptr;
  esp_timer_handle_t frameTimer_ = nullptr;
  bool started_ = false;
  char lastError_[160] = {};

  static void serviceTaskThunk(void *argument);
  void serviceTask();

  static void frameTimerThunk(void *argument);

  static bool IRAM_ATTR onRmtRxDone(
    rmt_channel_handle_t channel,
    const rmt_rx_done_event_data_t *eventData,
    void *userData
  );

  static bool IRAM_ATTR onRmtTxDone(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t *eventData,
    void *userData
  );

  static inline void IRAM_ATTR releaseEscSignal(MotorChannel &motor);
  static inline void enableEscSignalOutput(MotorChannel &motor);

  bool initializeRmt();
  bool runCycle();
  void recoverRxChannel(MotorChannel &motor);

  static uint16_t sanitizeCommand(uint16_t command);
  static uint16_t makeBidirectionalDshotFrame(uint16_t command);
  static void buildDshotSymbols(MotorChannel &motor);

  static void appendRun(
    Run *runs,
    size_t &runCount,
    uint8_t level,
    uint32_t ticks
  );

  TelemetryStatus decodePhysicalPacket(
    MotorChannel &motor,
    uint32_t physical21
  );

  TelemetryStatus decodeRunsFromIndex(
    MotorChannel &motor,
    const Run *runs,
    size_t runCount,
    size_t startIndex
  );

  TelemetryStatus decodeTelemetry(MotorChannel &motor);

  void registerPacketResult(
    MotorChannel &motor,
    TelemetryStatus status,
    uint32_t sampleTimestampUs
  );

  void publishSnapshot(
    const uint16_t commands[MOTOR_COUNT],
    uint32_t transactionTimestampUs
  );

  static float rollingQualityPercent(const MotorChannel &motor);
  static uint8_t popcount64(uint64_t value);

  bool setError(const char *operation, esp_err_t error);
  bool setErrorText(const char *text);
};
