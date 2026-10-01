/*
  ESP32 + Bluejay bidirectional DShot300 RPM reader
  -------------------------------------------------
  Target:
    - Original ESP32 (ESP32-WROOM / ESP32-D0WD family)
    - Arduino-ESP32 3.3.10 (ESP-IDF 5.5.4 RMT API)
    - Four Bluejay / BLHeli_S ESC signal lines

  IMPORTANT - REMOVE ALL PROPELLERS BEFORE TESTING.

  Wiring for EACH ESC channel on the 4-in-1 ESC:
    ESP32 GPIO ------------------------------------ ESC signal pad/wire
    ESP32 GND  ------------------------------------ ESC GND

  This revision does NOT use open-drain transmission and does NOT require an
  external pull-up. RMT drives each command with normal push-pull timing.

  IMPORTANT ESP32 FIX IN THIS REVISION:
  The ESP-IDF RMT driver routes the RMT output through the GPIO matrix with the
  pad output-enable controlled by the peripheral. In that mode, writing
  GPIO.enable_w1tc does NOT necessarily tri-state the pad. The prior version
  therefore kept driving HIGH after TX, which blocked every Bluejay reply and
  produced exactly 16 captured command symbols with NO_REPLY.

  This version changes GPIO_FUNCx_OEN_SEL so the GPIO ENABLE register controls
  the pad driver while the RMT peripheral still controls the HIGH/LOW data.
  The TX-complete ISR can then genuinely disconnect the output driver. A weak
  internal pull-up holds the released line HIGH until Bluejay actively drives
  its response. After RX, the ESP32 reconnects push-pull HIGH idle.

  A 100 to 330 ohm series resistor is optional for protection/ringing control,
  but the signal can remain directly connected as in your working motor test.

  Recommended bench sequence:
    1. REMOVE ALL PROPELLERS.
    2. Power/program the ESP32 and open Serial Monitor.
    3. Power the 4-in-1 ESC.
    4. Type: arm
    5. Wait for the 2-second zero-throttle interval to complete.

  The three thick motor wires are the motor phase wires. Never connect those
  wires to the ESP32. Bidirectional DShot telemetry returns on the thin ESC
  signal wire.

  Motor signal pins (M3/M4 swap preserved):
    M1 = GPIO26
    M2 = GPIO14
    M3 = GPIO27
    M4 = GPIO25

  Serial Monitor commands (921600 baud, newline ending):
    stop       -> command 0 to every motor
    arm        -> command 0 and restart the 2-second arming interval
    all 100    -> command DShot throttle 100 to all motors
    m1 100     -> command motor 1 only
    m2 100
    m3 100
    m4 100
    help

  Legal throttle commands in this test sketch are 0 or 48..2047.
  Values 1..47 are DShot special commands and are deliberately blocked.

  RPM conversion:
    The ESC returns electrical RPM (eRPM) timing. Mechanical RPM is
    eRPM / pole_pairs. Most 5-inch FPV outrunners have 14 magnet poles
    (7 pole pairs), but verify your motor and edit MOTOR_POLES below.

  Resource note:
    The original ESP32 has eight RMT channels. This four-motor implementation
    uses all eight: four RX + four TX. Do not use another RMT-based library
    (DShotRMT, NeoPixel RMT, IR RMT, etc.) in this standalone test.
*/

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/gpio_struct.h"

#if !defined(CONFIG_IDF_TARGET_ESP32)
#error "This four-motor sketch is for the original ESP32, which has eight RMT channels."
#endif

namespace {

constexpr size_t MOTOR_COUNT = 4;

constexpr gpio_num_t ESC_PINS[MOTOR_COUNT] = {
  GPIO_NUM_26,
  GPIO_NUM_14,
  GPIO_NUM_27,
  GPIO_NUM_25,
};

// Edit these if your motors do not have 14 magnet poles.
const uint8_t MOTOR_POLES[MOTOR_COUNT] = {14, 14, 14, 14};

// RMT timing: 40 MHz gives one tick = 25 ns.
constexpr uint32_t RMT_RESOLUTION_HZ = 40000000UL;
constexpr uint32_t DSHOT_BITRATE_HZ = 300000UL;
constexpr uint32_t TELEMETRY_BITRATE_HZ = (DSHOT_BITRATE_HZ * 5UL) / 4UL; // 375 kbit/s

// DShot300 bit cell = 3.333 us. We use 133 ticks = 3.325 us.
constexpr uint16_t DSHOT_BIT_TICKS = 133;
constexpr uint16_t DSHOT_ONE_LOW_TICKS = 100;  // 2.500 us low, then 0.825 us driven high
constexpr uint16_t DSHOT_ZERO_LOW_TICKS = 50;  // 1.250 us low, then 2.075 us driven high
// Betaflight holds the idle level for one additional DShot cell before
// switching the pin to input. This allows the ESC to sample the final bit
// cleanly and still leaves about 26 us before the nominal reply starts.
constexpr uint16_t DSHOT_END_HOLD_TICKS = DSHOT_BIT_TICKS;
constexpr size_t DSHOT_TX_SYMBOL_COUNT = 17;

// The command-to-response break is about 30 us. Search for a high run longer
// than 10 us only after all 16 outgoing low pulses have been observed.
constexpr uint32_t RESPONSE_GAP_MIN_TICKS =
    (RMT_RESOLUTION_HZ / 1000000UL) * 10UL;

// The RX idle/stop threshold must be longer than the 30 us turnaround break.
constexpr uint32_t RX_MIN_PULSE_NS = 150;
// 300 us is still short relative to the 2 ms frame period, but long enough
// that arming the RX channel and queuing TX cannot accidentally terminate RX
// before the first DShot edge. It is also safely longer than the 30 us break.
constexpr uint32_t RX_IDLE_STOP_NS = 300000; // 300 us

constexpr size_t RX_SYMBOL_CAPACITY = 64;
constexpr size_t MAX_RUNS = RX_SYMBOL_CAPACITY * 2 + 2;
constexpr size_t TELEMETRY_PHYSICAL_BITS = 21;
constexpr size_t MIN_INFERABLE_TELEMETRY_BITS = 18;

// 500 Hz is intentionally conservative for a first bench test.
constexpr uint32_t FRAME_PERIOD_US = 2000;
constexpr uint32_t PRINT_PERIOD_MS = 250;
constexpr uint32_t ARM_ZERO_TIME_MS = 2000;
constexpr uint32_t SERIAL_FAILSAFE_MS = 30000;
constexpr float RPM_FILTER_ALPHA = 0.25f;

enum class TelemetryStatus : uint8_t {
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

struct MotorChannel {
  gpio_num_t pin = GPIO_NUM_NC;
  uint8_t poles = 14;

  rmt_channel_handle_t rx = nullptr;
  rmt_channel_handle_t tx = nullptr;
  rmt_encoder_handle_t copyEncoder = nullptr;
  SemaphoreHandle_t rxDone = nullptr;

  volatile size_t rxCount = 0;
  volatile bool txOutputReleased = true;

  // Last-capture diagnostics. decodeMode is:
  //   'G' = reply found after a measured turnaround gap
  //   'R' = reply-only capture decoded without seeing the outgoing command
  //   '-' = no valid reply decoded
  uint16_t lastRunCount = 0;
  uint16_t lastLowRuns = 0;
  uint32_t lastCaptureTicks = 0;
  int16_t lastReplyRun = -1;
  char decodeMode = '-';

  rmt_symbol_word_t rxSymbols[RX_SYMBOL_CAPACITY] = {};
  rmt_symbol_word_t txSymbols[DSHOT_TX_SYMBOL_COUNT] = {};

  uint16_t throttle = 0;
  uint16_t rawTelemetry = 0x0FFF;
  uint32_t electricalPeriodUs = 0;
  float rpm = 0.0f;
  float rpmFiltered = 0.0f;

  uint32_t goodPackets = 0;
  uint32_t badPackets = 0;
  uint32_t timeouts = 0;
  TelemetryStatus status = TelemetryStatus::RX_TIMEOUT;
};

struct Run {
  uint8_t level;
  uint32_t ticks;
};

MotorChannel motors[MOTOR_COUNT];
uint32_t nextFrameUs = 0;
uint32_t nextPrintMs = 0;
uint32_t lastSerialCommandMs = 0;
uint32_t zeroStartMs = 0;
bool zeroArmingComplete = false;
char serialLine[64] = {};
size_t serialLineLength = 0;

// Inverse of the standard DShot 4b/5b GCR table.
const int8_t GCR_DECODE[32] = {
  -1, -1, -1, -1, -1, -1, -1, -1,
  -1,  9, 10, 11, -1, 13, 14, 15,
  -1, -1,  2,  3, -1,  5,  6,  7,
  -1,  0,  8,  1, -1,  4, 12, -1,
};

const char *statusName(TelemetryStatus status) {
  switch (status) {
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

void fatalEspError(const char *operation, esp_err_t error) {
  Serial.printf("\nFATAL: %s failed: %s (0x%X)\n",
                operation, esp_err_to_name(error), static_cast<unsigned>(error));
  Serial.println("The four-motor version needs all eight RMT channels on an original ESP32.");
  Serial.println("Remove other RMT libraries/peripherals and reset the board.");
  while (true) {
    delay(1000);
  }
}

void requireEspOk(const char *operation, esp_err_t error) {
  if (error != ESP_OK) {
    fatalEspError(operation, error);
  }
}

bool IRAM_ATTR onRmtRxDone(rmt_channel_handle_t,
                           const rmt_rx_done_event_data_t *eventData,
                           void *userData) {
  MotorChannel *motor = static_cast<MotorChannel *>(userData);
  motor->rxCount = eventData->num_symbols;

  BaseType_t higherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(motor->rxDone, &higherPriorityTaskWoken);
  return higherPriorityTaskWoken == pdTRUE;
}

// All selected ESC pins are below GPIO32, so the original ESP32 can switch
// output-enable with a single IRAM-safe register write. The RMT signal remains
// routed to the pad; only the pad driver is disconnected/reconnected.
static_assert(ESC_PINS[0] < GPIO_NUM_32 && ESC_PINS[1] < GPIO_NUM_32 &&
              ESC_PINS[2] < GPIO_NUM_32 && ESC_PINS[3] < GPIO_NUM_32,
              "This fast half-duplex helper expects ESC GPIOs below 32.");

inline void IRAM_ATTR releaseEscSignal(MotorChannel &motor) {
  GPIO.enable_w1tc = (1UL << static_cast<uint32_t>(motor.pin));
  motor.txOutputReleased = true;
}

inline void enableEscSignalOutput(MotorChannel &motor) {
  // The RMT channel is configured with init_level=1 and every transaction ends
  // at eot_level=1. Re-enabling the pad therefore actively holds the one-wire
  // bus HIGH between packets. Bluejay samples the idle level before a packet to
  // distinguish normal DShot (idle LOW) from bidirectional DShot (idle HIGH).
  motor.txOutputReleased = false;
  GPIO.enable_w1ts = (1UL << static_cast<uint32_t>(motor.pin));
}

bool IRAM_ATTR onRmtTxDone(rmt_channel_handle_t,
                           const rmt_tx_done_event_data_t *,
                           void *userData) {
  MotorChannel *motor = static_cast<MotorChannel *>(userData);

  // The final RMT level is HIGH. Immediately remove the ESP32 output driver so
  // Bluejay can drive the same wire after its turnaround delay. No per-bit CPU
  // timing is used; RMT generated the complete 16-bit command in hardware.
  releaseEscSignal(*motor);
  return false;
}

uint16_t makeBidirectionalDshotFrame(uint16_t throttle) {
  // In bidirectional DShot the ordinary telemetry-request bit remains zero;
  // the inverted waveform itself tells Bluejay to return eRPM on this line.
  const uint16_t payload12 = static_cast<uint16_t>((throttle & 0x07FFU) << 1U);
  const uint8_t checksum = static_cast<uint8_t>(
      (~(payload12 ^ (payload12 >> 4U) ^ (payload12 >> 8U))) & 0x0FU);
  return static_cast<uint16_t>((payload12 << 4U) | checksum);
}

void buildDshotSymbols(MotorChannel &motor) {
  const uint16_t frame = makeBidirectionalDshotFrame(motor.throttle);

  for (size_t bit = 0; bit < 16; ++bit) {
    const bool logicalOne = (frame & (0x8000U >> bit)) != 0;
    const uint16_t lowTicks = logicalOne
                                ? DSHOT_ONE_LOW_TICKS
                                : DSHOT_ZERO_LOW_TICKS;

    // Inverted/bidirectional DShot: drive LOW first, then drive HIGH.
    motor.txSymbols[bit].level0 = 0;
    motor.txSymbols[bit].duration0 = lowTicks;
    motor.txSymbols[bit].level1 = 1;
    motor.txSymbols[bit].duration1 = DSHOT_BIT_TICKS - lowTicks;
  }

  // Keep the line actively HIGH for one complete DShot cell after bit 16.
  // Two same-level durations are used so the copy encoder does not interpret a
  // zero duration as an early end marker.
  motor.txSymbols[16].level0 = 1;
  motor.txSymbols[16].duration0 = DSHOT_END_HOLD_TICKS - 1U;
  motor.txSymbols[16].level1 = 1;
  motor.txSymbols[16].duration1 = 1U;
}

void appendRun(Run *runs, size_t &runCount, uint8_t level, uint32_t ticks) {
  if (ticks == 0 || runCount >= MAX_RUNS) {
    return;
  }

  if (runCount > 0 && runs[runCount - 1].level == level) {
    runs[runCount - 1].ticks += ticks;
  } else {
    runs[runCount].level = level;
    runs[runCount].ticks = ticks;
    ++runCount;
  }
}

TelemetryStatus decodePhysicalPacket(MotorChannel &motor,
                                      uint32_t physical21) {
  // The 21-bit NRZI stream has an explicit leading zero.
  if ((physical21 & (1UL << 20U)) != 0) {
    return TelemetryStatus::BAD_START_BIT;
  }

  // Differential/NRZI decode: adjacent physical levels XOR to the 20 GCR bits.
  const uint32_t gcr20 = (physical21 ^ (physical21 >> 1U)) & 0x000FFFFFUL;

  uint32_t decoded16 = 0;
  for (size_t group = 0; group < 4; ++group) {
    const uint8_t code = static_cast<uint8_t>((gcr20 >> (group * 5U)) & 0x1FU);
    const int8_t nibble = GCR_DECODE[code];
    if (nibble < 0) {
      return TelemetryStatus::BAD_GCR_CODE;
    }
    decoded16 |= static_cast<uint32_t>(nibble) << (group * 4U);
  }

  // Bluejay's returned packet is valid when XOR of all four nibbles is 0xF.
  uint32_t checksumFold = decoded16;
  checksumFold ^= checksumFold >> 8U;
  checksumFold ^= checksumFold >> 4U;
  if ((checksumFold & 0x0FU) != 0x0FU) {
    return TelemetryStatus::BAD_CRC;
  }

  const uint16_t raw12 = static_cast<uint16_t>(decoded16 >> 4U);
  motor.rawTelemetry = raw12;

  // 0xFFF is Bluejay's stopped/no-period value.
  if (raw12 == 0x0FFFU) {
    motor.electricalPeriodUs = 0;
    motor.rpm = 0.0f;
    motor.rpmFiltered *= (1.0f - RPM_FILTER_ALPHA);
    if (motor.rpmFiltered < 1.0f) {
      motor.rpmFiltered = 0.0f;
    }
    return TelemetryStatus::OK;
  }

  const uint8_t exponent = static_cast<uint8_t>((raw12 >> 9U) & 0x07U);
  const uint16_t mantissa = static_cast<uint16_t>(raw12 & 0x01FFU);
  const uint32_t electricalPeriodUs = static_cast<uint32_t>(mantissa) << exponent;

  if (electricalPeriodUs == 0 || motor.poles < 2 || (motor.poles & 1U) != 0) {
    return TelemetryStatus::BAD_PERIOD;
  }

  motor.electricalPeriodUs = electricalPeriodUs;

  const float electricalRpm = 60000000.0f / static_cast<float>(electricalPeriodUs);
  const float polePairs = static_cast<float>(motor.poles) * 0.5f;
  motor.rpm = electricalRpm / polePairs;

  if (motor.goodPackets == 0) {
    motor.rpmFiltered = motor.rpm;
  } else {
    motor.rpmFiltered += RPM_FILTER_ALPHA * (motor.rpm - motor.rpmFiltered);
  }

  return TelemetryStatus::OK;
}

TelemetryStatus decodeRunsFromIndex(MotorChannel &motor,
                                    const Run *runs,
                                    size_t runCount,
                                    size_t startIndex) {
  uint32_t physical21 = 0;
  size_t physicalBitCount = 0;

  for (size_t i = startIndex;
       i < runCount && physicalBitCount < TELEMETRY_PHYSICAL_BITS;
       ++i) {
    // Round each measured run to the nearest number of 375 kbit/s cells.
    uint32_t cells = static_cast<uint32_t>(
        (static_cast<uint64_t>(runs[i].ticks) * TELEMETRY_BITRATE_HZ +
         (RMT_RESOLUTION_HZ / 2U)) /
        RMT_RESOLUTION_HZ);

    if (cells == 0) {
      continue;
    }

    const size_t remaining = TELEMETRY_PHYSICAL_BITS - physicalBitCount;
    if (cells > remaining) {
      // The last HIGH run is commonly merged with the idle HIGH period.
      cells = static_cast<uint32_t>(remaining);
    }

    for (uint32_t cell = 0; cell < cells; ++cell) {
      physical21 = (physical21 << 1U) | (runs[i].level ? 1U : 0U);
      ++physicalBitCount;
    }
  }

  if (physicalBitCount < MIN_INFERABLE_TELEMETRY_BITS) {
    return TelemetryStatus::TOO_FEW_CELLS;
  }

  // The RMT idle detector can swallow the response tail. Bluejay returns to
  // HIGH idle, so missing final cells can be inferred as HIGH.
  while (physicalBitCount < TELEMETRY_PHYSICAL_BITS) {
    physical21 = (physical21 << 1U) | 1U;
    ++physicalBitCount;
  }

  return decodePhysicalPacket(motor, physical21);
}

TelemetryStatus decodeTelemetry(MotorChannel &motor) {
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

  for (size_t i = 0; i < symbolCount; ++i) {
    const rmt_symbol_word_t &symbol = motor.rxSymbols[i];
    appendRun(runs, runCount, symbol.level0, symbol.duration0);
    appendRun(runs, runCount, symbol.level1, symbol.duration1);
  }

  if (runCount == 0) {
    return TelemetryStatus::RX_EMPTY;
  }

  size_t lowRunsSeen = 0;
  for (size_t i = 0; i < runCount; ++i) {
    motor.lastCaptureTicks += runs[i].ticks;
    if (runs[i].level == 0) {
      ++lowRunsSeen;
    }
  }
  motor.lastRunCount = static_cast<uint16_t>(runCount);
  motor.lastLowRuns = static_cast<uint16_t>(lowRunsSeen);

  // First try the normal capture layout: outgoing DShot command, a long HIGH
  // turnaround interval, and then a LOW first telemetry cell. Do not require
  // that the RX channel saw all 16 outgoing LOW pulses. On the original ESP32,
  // a separately allocated same-pad RX channel can begin recording only after
  // the TX output driver is released, even though the ESC received the command.
  bool foundGap = false;
  TelemetryStatus firstGapError = TelemetryStatus::NO_RESPONSE_EDGE;
  for (size_t i = 0; i + 1 < runCount; ++i) {
    if (runs[i].level == 1 &&
        runs[i].ticks >= RESPONSE_GAP_MIN_TICKS &&
        runs[i + 1].level == 0) {
      foundGap = true;
      const TelemetryStatus status =
          decodeRunsFromIndex(motor, runs, runCount, i + 1U);
      if (status == TelemetryStatus::OK) {
        motor.lastReplyRun = static_cast<int16_t>(i + 1U);
        motor.decodeMode = 'G';
        return status;
      }
      if (firstGapError == TelemetryStatus::NO_RESPONSE_EDGE) {
        firstGapError = status;
      }
    }
  }

  // Fallback for reply-only captures. Try every LOW run as the first physical
  // telemetry cell and accept only a packet that passes GCR and CRC checks.
  // This cannot manufacture a plausible RPM from random edges without also
  // satisfying the protocol's code and checksum constraints.
  TelemetryStatus firstCandidateError = TelemetryStatus::NO_TURNAROUND_GAP;
  bool testedCandidate = false;
  for (size_t i = 0; i < runCount; ++i) {
    if (runs[i].level != 0) {
      continue;
    }

    testedCandidate = true;
    const TelemetryStatus status =
        decodeRunsFromIndex(motor, runs, runCount, i);
    if (status == TelemetryStatus::OK) {
      motor.lastReplyRun = static_cast<int16_t>(i);
      motor.decodeMode = 'R';
      return status;
    }

    if (firstCandidateError == TelemetryStatus::NO_TURNAROUND_GAP &&
        status != TelemetryStatus::TOO_FEW_CELLS) {
      firstCandidateError = status;
    }
  }

  if (foundGap) {
    return firstGapError;
  }

  if (lowRunsSeen >= 16U) {
    return TelemetryStatus::NO_RESPONSE_EDGE;
  }

  if (testedCandidate &&
      firstCandidateError != TelemetryStatus::NO_TURNAROUND_GAP) {
    return firstCandidateError;
  }

  return TelemetryStatus::NO_TURNAROUND_GAP;
}

void initializeRmt() {
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    motors[i].pin = ESC_PINS[i];
    motors[i].poles = MOTOR_POLES[i];
    motors[i].rxDone = xSemaphoreCreateBinary();
    if (motors[i].rxDone == nullptr) {
      Serial.println("FATAL: could not allocate RX semaphore");
      while (true) {
        delay(1000);
      }
    }
  }

  // Allocate all four RX channels first.
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    rmt_rx_channel_config_t rxConfig = {};
    rxConfig.gpio_num = motors[i].pin;
    rxConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    rxConfig.resolution_hz = RMT_RESOLUTION_HZ;
    rxConfig.mem_block_symbols = RX_SYMBOL_CAPACITY;
    rxConfig.intr_priority = 0;
    rxConfig.flags.invert_in = 0;
    rxConfig.flags.with_dma = 0;
    rxConfig.flags.io_loop_back = 0;
    rxConfig.flags.allow_pd = 0;

    requireEspOk("rmt_new_rx_channel",
                 rmt_new_rx_channel(&rxConfig, &motors[i].rx));

    rmt_rx_event_callbacks_t callbacks = {};
    callbacks.on_recv_done = onRmtRxDone;
    requireEspOk("rmt_rx_register_event_callbacks",
                 rmt_rx_register_event_callbacks(
                     motors[i].rx, &callbacks, &motors[i]));
  }

  // Allocate all four TX channels on the same four pins. RMT generates the
  // outgoing waveform with push-pull drive. The TX-done callback disables the
  // pad output-enable bit immediately after the 16th bit so Bluejay can reply.
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    rmt_tx_channel_config_t txConfig = {};
    txConfig.gpio_num = motors[i].pin;
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.resolution_hz = RMT_RESOLUTION_HZ;
    txConfig.mem_block_symbols = RX_SYMBOL_CAPACITY;
    txConfig.trans_queue_depth = 1;
    txConfig.intr_priority = 0;
    txConfig.flags.invert_out = 0;
    txConfig.flags.with_dma = 0;
    // A separate RMT RX channel is already attached to this pad. Enabling
    // the TX channel's test loopback can interfere with that RX route on ESP32.
    txConfig.flags.io_loop_back = 0;
    txConfig.flags.io_od_mode = 0;  // normal push-pull while transmitting
    txConfig.flags.allow_pd = 0;
    txConfig.flags.init_level = 1;

    requireEspOk("rmt_new_tx_channel",
                 rmt_new_tx_channel(&txConfig, &motors[i].tx));

    // On the original ESP32, esp_rom_gpio_connect_out_signal() leaves OEN_SEL
    // at 0, so the peripheral controls pad output-enable. GPIO.enable_w1tc
    // cannot tri-state the line in that state. Force OEN_SEL=1 so the GPIO
    // ENABLE register controls only the pad driver while RMT still supplies
    // the output data signal. This is the key fix for the n16/NO_REPLY result.
    GPIO.func_out_sel_cfg[static_cast<uint32_t>(motors[i].pin)].oen_sel = 1;
    GPIO.func_out_sel_cfg[static_cast<uint32_t>(motors[i].pin)].oen_inv_sel = 0;

    rmt_tx_event_callbacks_t txCallbacks = {};
    txCallbacks.on_trans_done = onRmtTxDone;
    requireEspOk("rmt_tx_register_event_callbacks",
                 rmt_tx_register_event_callbacks(
                     motors[i].tx, &txCallbacks, &motors[i]));

    // rmt_new_tx_channel() enables the GPIO output path. Keep it disconnected
    // until a frame is actually about to start.
    releaseEscSignal(motors[i]);

    rmt_copy_encoder_config_t copyConfig = {};
    requireEspOk("rmt_new_copy_encoder",
                 rmt_new_copy_encoder(&copyConfig, &motors[i].copyEncoder));
  }

  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    requireEspOk("rmt_enable RX", rmt_enable(motors[i].rx));
    requireEspOk("rmt_enable TX", rmt_enable(motors[i].tx));

    // Keep the input path active for RMT RX. The weak internal pull-up only
    // holds the bus HIGH while the ESP32 driver is disconnected; it does not
    // generate the outgoing DShot edges. Bluejay actively drives its reply.
    requireEspOk("gpio_input_enable", gpio_input_enable(motors[i].pin));
    gpio_pulldown_dis(motors[i].pin);
    gpio_pullup_en(motors[i].pin);
    enableEscSignalOutput(motors[i]);

    if (GPIO.func_out_sel_cfg[static_cast<uint32_t>(motors[i].pin)].oen_sel != 1) {
      Serial.printf("FATAL: GPIO%u OEN_SEL did not switch to GPIO control.\n",
                    static_cast<unsigned>(motors[i].pin));
      while (true) {
        delay(1000);
      }
    }
  }
}

void setAllThrottle(uint16_t throttle) {
  for (MotorChannel &motor : motors) {
    motor.throttle = throttle;
  }
}

void restartZeroArming() {
  setAllThrottle(0);
  for (MotorChannel &motor : motors) {
    motor.rawTelemetry = 0x0FFF;
    motor.electricalPeriodUs = 0;
    motor.rpm = 0.0f;
    motor.rpmFiltered = 0.0f;
    motor.goodPackets = 0;
    motor.badPackets = 0;
    motor.timeouts = 0;
    motor.status = TelemetryStatus::RX_TIMEOUT;
    motor.lastRunCount = 0;
    motor.lastLowRuns = 0;
    motor.lastCaptureTicks = 0;
    motor.lastReplyRun = -1;
    motor.decodeMode = '-';
  }
  zeroArmingComplete = false;
  zeroStartMs = millis();
  lastSerialCommandMs = zeroStartMs;
  Serial.println("Arming restarted: counters cleared; sending 2 seconds of command 0.");
}

void dumpCapture(size_t motorIndex) {
  if (motorIndex >= MOTOR_COUNT) {
    return;
  }

  const MotorChannel &motor = motors[motorIndex];
  const size_t count = motor.rxCount < RX_SYMBOL_CAPACITY
                         ? motor.rxCount
                         : RX_SYMBOL_CAPACITY;

  Serial.printf("\nCAPTURE M%u GPIO%u: symbols=%u runs=%u lowRuns=%u ticks=%lu (%.2f us) mode=%c replyRun=%d status=%s\n",
                static_cast<unsigned>(motorIndex + 1U),
                static_cast<unsigned>(motor.pin),
                static_cast<unsigned>(count),
                static_cast<unsigned>(motor.lastRunCount),
                static_cast<unsigned>(motor.lastLowRuns),
                static_cast<unsigned long>(motor.lastCaptureTicks),
                static_cast<double>(motor.lastCaptureTicks) * 1000000.0 /
                    static_cast<double>(RMT_RESOLUTION_HZ),
                motor.decodeMode,
                static_cast<int>(motor.lastReplyRun),
                statusName(motor.status));

  for (size_t i = 0; i < count; ++i) {
    const rmt_symbol_word_t &s = motor.rxSymbols[i];
    Serial.printf("  %02u: L%u=%u (%.3fus), L%u=%u (%.3fus)\n",
                  static_cast<unsigned>(i),
                  static_cast<unsigned>(s.level0),
                  static_cast<unsigned>(s.duration0),
                  static_cast<double>(s.duration0) * 1000000.0 /
                      static_cast<double>(RMT_RESOLUTION_HZ),
                  static_cast<unsigned>(s.level1),
                  static_cast<unsigned>(s.duration1),
                  static_cast<double>(s.duration1) * 1000000.0 /
                      static_cast<double>(RMT_RESOLUTION_HZ));
  }
  Serial.println("END CAPTURE\n");
}

bool validThrottle(long value) {
  return value == 0 || (value >= 48 && value <= 2047);
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  stop       : stop all motors");
  Serial.println("  arm        : stop all motors and restart 2-second zero arming");
  Serial.println("  all N      : set all motors to DShot N (N=48..2047)");
  Serial.println("  m1 N       : set motor 1 only");
  Serial.println("  m2 N       : set motor 2 only");
  Serial.println("  m3 N       : set motor 3 only");
  Serial.println("  m4 N       : set motor 4 only");
  Serial.println("  dump1      : print the last raw RMT capture for motor 1");
  Serial.println("  dump2/dump3/dump4 work the same way");
  Serial.println("  help");
  Serial.println("Values 1..47 are blocked because they are DShot special commands.");
  Serial.println();
}

void processSerialCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    ++line;
  }

  for (char *p = line; *p != '\0'; ++p) {
    if (*p >= 'A' && *p <= 'Z') {
      *p = static_cast<char>(*p - 'A' + 'a');
    }
  }

  // Ignore accidental spaces/tabs at the end of a Serial Monitor command.
  char *end = line + strlen(line);
  while (end > line && (end[-1] == ' ' || end[-1] == '\t')) {
    *--end = '\0';
  }

  if (strcmp(line, "stop") == 0 || strcmp(line, "off") == 0) {
    setAllThrottle(0);
    lastSerialCommandMs = millis();
    Serial.println("All motors set to 0.");
    return;
  }

  if (strcmp(line, "arm") == 0 || strcmp(line, "rearm") == 0) {
    restartZeroArming();
    return;
  }

  if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
    printHelp();
    return;
  }

  if (strncmp(line, "dump", 4) == 0 &&
      line[4] >= '1' && line[4] <= '4' && line[5] == '\0') {
    dumpCapture(static_cast<size_t>(line[4] - '1'));
    return;
  }

  char target[8] = {};
  long value = -1;
  if (sscanf(line, "%7s %ld", target, &value) != 2) {
    Serial.println("Bad command. Type help.");
    return;
  }

  if (!validThrottle(value)) {
    Serial.println("Rejected: use 0 or a throttle from 48 through 2047.");
    return;
  }

  if (value > 0 && !zeroArmingComplete) {
    Serial.println("Rejected: startup zero-throttle arming interval is not complete.");
    return;
  }

  if (strcmp(target, "all") == 0) {
    setAllThrottle(static_cast<uint16_t>(value));
  } else if (target[0] == 'm' && target[2] == '\0' &&
             target[1] >= '1' && target[1] <= '4') {
    const size_t index = static_cast<size_t>(target[1] - '1');
    motors[index].throttle = static_cast<uint16_t>(value);
  } else {
    Serial.println("Unknown target. Use all, m1, m2, m3, or m4.");
    return;
  }

  lastSerialCommandMs = millis();
  Serial.printf("Accepted: %s = %ld\n", target, value);
}

void serviceSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r' || c == '\n') {
      if (serialLineLength > 0) {
        serialLine[serialLineLength] = '\0';
        processSerialCommand(serialLine);
        serialLineLength = 0;
      }
    } else if (c == '\b' || c == 127) {
      if (serialLineLength > 0) {
        --serialLineLength;
      }
    } else if (serialLineLength + 1 < sizeof(serialLine)) {
      serialLine[serialLineLength++] = c;
    }
  }
}

void runBidirectionalDshotCycle() {
  rmt_receive_config_t receiveConfig = {};
  receiveConfig.signal_range_min_ns = RX_MIN_PULSE_NS;
  receiveConfig.signal_range_max_ns = RX_IDLE_STOP_NS;
  receiveConfig.flags.en_partial_rx = 0;

  rmt_transmit_config_t transmitConfig = {};
  transmitConfig.loop_count = 0;
  transmitConfig.flags.eot_level = 1;       // finish HIGH, then ISR tri-states pad
  transmitConfig.flags.queue_nonblocking = 0;

  bool rxArmed[MOTOR_COUNT] = {};
  bool txQueued[MOTOR_COUNT] = {};

  // Build all payloads before arming RX so no receiver sits idle while the
  // CPU is doing frame construction.
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    buildDshotSymbols(motors[i]);
  }

  // For each motor, arm its hardware receiver and immediately queue its
  // hardware transmitter. RX therefore sees our outgoing frame, the ~30 us
  // high break, and the ESC reply on the same wire. The small launch skew
  // between motors is irrelevant because every motor is decoded independently.
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    while (xSemaphoreTake(motors[i].rxDone, 0) == pdTRUE) {
      // Drain a stale completion token, if any.
    }

    motors[i].rxCount = 0;
    esp_err_t error = rmt_receive(
        motors[i].rx,
        motors[i].rxSymbols,
        sizeof(motors[i].rxSymbols),
        &receiveConfig);

    rxArmed[i] = error == ESP_OK;
    if (!rxArmed[i]) {
      motors[i].status = TelemetryStatus::RX_START_ERROR;
      ++motors[i].badPackets;
      continue;
    }

    // Reconnect push-pull output only for the outgoing hardware-timed frame.
    // The onRmtTxDone ISR disconnects it again at the exact frame boundary.
    enableEscSignalOutput(motors[i]);

    error = rmt_transmit(
        motors[i].tx,
        motors[i].copyEncoder,
        motors[i].txSymbols,
        sizeof(motors[i].txSymbols),
        &transmitConfig);

    txQueued[i] = error == ESP_OK;
    if (!txQueued[i]) {
      releaseEscSignal(motors[i]);
      motors[i].status = TelemetryStatus::TX_ERROR;
      ++motors[i].badPackets;
    }
  }

  // The command itself lasts about 53.3 us. A 2 ms software timeout leaves a
  // large margin for the 30 us turnaround and roughly 56 us reply.
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (txQueued[i]) {
      const esp_err_t error = rmt_tx_wait_all_done(motors[i].tx, 2);
      if (error != ESP_OK) {
        releaseEscSignal(motors[i]);
        motors[i].status = TelemetryStatus::TX_ERROR;
        ++motors[i].badPackets;
        txQueued[i] = false;
      } else if (!motors[i].txOutputReleased) {
        // The callback should have executed before the waiting task resumes.
        // Force a safe high-impedance state and expose the fault in telemetry.
        releaseEscSignal(motors[i]);
        motors[i].status = TelemetryStatus::TX_RELEASE_ERROR;
        ++motors[i].badPackets;
        txQueued[i] = false;
      }
    }
  }

  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    if (!rxArmed[i] || !txQueued[i]) {
      // Preserve a defined bidirectional-DShot idle state even after an error.
      enableEscSignalOutput(motors[i]);
      continue;
    }

    if (xSemaphoreTake(motors[i].rxDone, pdMS_TO_TICKS(2)) != pdTRUE) {
      motors[i].status = TelemetryStatus::RX_TIMEOUT;
      ++motors[i].timeouts;
      ++motors[i].badPackets;

      // The telemetry window is over. Restore a solid HIGH idle immediately.
      enableEscSignalOutput(motors[i]);
      continue;
    }

    // RX has stopped after the configured idle timeout, so the ESC has finished
    // its reply (or did not reply). Reconnect push-pull HIGH before doing the
    // more expensive software decode. This gives Bluejay far more than its
    // required stable-idle detection time before the next packet.
    enableEscSignalOutput(motors[i]);

    motors[i].status = decodeTelemetry(motors[i]);
    if (motors[i].status == TelemetryStatus::OK) {
      ++motors[i].goodPackets;
    } else {
      ++motors[i].badPackets;
    }
  }
}

void printTelemetry() {
  // Keep each report below a few hundred bytes so debug output does not create
  // long holes in the 500 Hz DShot schedule. At 921600 baud this is a small,
  // infrequent UART transfer rather than four long blocking print lines.
  char output[384] = {};
  size_t used = 0;

  int written = snprintf(
      output,
      sizeof(output),
      "t=%lu arm=%u",
      static_cast<unsigned long>(millis()),
      zeroArmingComplete ? 1U : 0U);
  if (written < 0) {
    return;
  }
  used = static_cast<size_t>(written);
  if (used >= sizeof(output)) {
    used = sizeof(output) - 1U;
  }

  for (size_t i = 0; i < MOTOR_COUNT && used + 1U < sizeof(output); ++i) {
    const MotorChannel &motor = motors[i];
    const uint32_t total = motor.goodPackets + motor.badPackets;
    const uint32_t qualityTenths = total > 0
        ? static_cast<uint32_t>(
              (static_cast<uint64_t>(motor.goodPackets) * 1000ULL) / total)
        : 0U;

    written = snprintf(
        output + used,
        sizeof(output) - used,
        " | M%u:g%u c%u r%lu f%lu %s q%lu.%lu n%u d%c",
        static_cast<unsigned>(i + 1U),
        static_cast<unsigned>(motor.pin),
        static_cast<unsigned>(motor.throttle),
        static_cast<unsigned long>(motor.rpm + 0.5f),
        static_cast<unsigned long>(motor.rpmFiltered + 0.5f),
        statusName(motor.status),
        static_cast<unsigned long>(qualityTenths / 10U),
        static_cast<unsigned long>(qualityTenths % 10U),
        static_cast<unsigned>(motor.rxCount),
        motor.decodeMode);

    if (written < 0) {
      break;
    }
    const size_t appended = static_cast<size_t>(written);
    if (appended >= sizeof(output) - used) {
      used = sizeof(output) - 1U;
      break;
    }
    used += appended;
  }

  if (used + 1U < sizeof(output)) {
    output[used++] = '\n';
    output[used] = '\0';
  }
  Serial.write(reinterpret_cast<const uint8_t *>(output), used);
}

} // namespace

void setup() {
  // Get all ESC signal pins LOW as soon as Arduino setup() begins. The RMT
  // channels later own the signal routing; output is enabled only during frames.
  for (size_t i = 0; i < MOTOR_COUNT; ++i) {
    gpio_reset_pin(ESC_PINS[i]);
    gpio_set_direction(ESC_PINS[i], GPIO_MODE_OUTPUT);
    gpio_set_level(ESC_PINS[i], 0);
  }

  Serial.begin(921600);

  Serial.println();
  Serial.println("ESP32 / Bluejay bidirectional DShot300 RPM test - manual-OEN v4");
  Serial.println("REMOVE ALL PROPELLERS.");
  Serial.println("RMT push-pull TX + manual GPIO-matrix OEN gating + same-pin RMT RX.");
  Serial.println("Adds one-cell end hold and a weak pull-up during the reply window.");
  Serial.println("Motor map: M1=26 M2=14 M3=27 M4=25.");

  initializeRmt();
  setAllThrottle(0);

  // Begin sending zero frames immediately; do not insert a startup delay here.
  zeroStartMs = millis();
  lastSerialCommandMs = zeroStartMs;
  nextFrameUs = micros();
  nextPrintMs = zeroStartMs + PRINT_PERIOD_MS;

  printHelp();
  Serial.println("After connecting ESC power, type 'arm' and wait 2 seconds.");
}

void loop() {
  serviceSerial();

  const uint32_t nowMs = millis();
  if (!zeroArmingComplete && (nowMs - zeroStartMs) >= ARM_ZERO_TIME_MS) {
    zeroArmingComplete = true;
    Serial.println("Zero-throttle arming interval complete. Throttle commands are now accepted.");
  }

  if ((nowMs - lastSerialCommandMs) > SERIAL_FAILSAFE_MS) {
    bool anyRunning = false;
    for (const MotorChannel &motor : motors) {
      anyRunning |= motor.throttle != 0;
    }
    if (anyRunning) {
      setAllThrottle(0);
      Serial.println("Serial failsafe: no command for 30 seconds; all motors set to 0.");
    }
    lastSerialCommandMs = nowMs;
  }

  const uint32_t nowUs = micros();
  if (static_cast<int32_t>(nowUs - nextFrameUs) >= 0) {
    runBidirectionalDshotCycle();
    nextFrameUs += FRAME_PERIOD_US;

    // Check the clock after the complete TX/turnaround/RX transaction. If
    // printing or another task delayed us, restart the phase instead of
    // emitting a burst of stale frames.
    const uint32_t afterCycleUs = micros();
    if (static_cast<int32_t>(afterCycleUs - nextFrameUs) >
        static_cast<int32_t>(FRAME_PERIOD_US * 2U)) {
      nextFrameUs = afterCycleUs + FRAME_PERIOD_US;
    }
  }

  if (static_cast<int32_t>(nowMs - nextPrintMs) >= 0) {
    printTelemetry();
    nextPrintMs += PRINT_PERIOD_MS;
  }

  delay(0); // feed the Arduino/FreeRTOS scheduler without adding a millisecond delay
}