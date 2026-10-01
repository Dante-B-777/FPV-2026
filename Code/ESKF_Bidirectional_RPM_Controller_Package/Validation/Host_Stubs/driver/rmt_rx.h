#pragma once
#include <cstddef>
#include <cstdint>
#include "driver/gpio.h"
#include "esp_err.h"
struct StubRmtChannel {};
using rmt_channel_handle_t = StubRmtChannel*;
static constexpr int RMT_CLK_SRC_DEFAULT = 0;
struct rmt_symbol_word_t {
  uint16_t duration0=0;
  uint16_t level0=0;
  uint16_t duration1=0;
  uint16_t level1=0;
};
struct rmt_rx_channel_config_t {
  gpio_num_t gpio_num=GPIO_NUM_NC;
  int clk_src=0;
  uint32_t resolution_hz=0;
  size_t mem_block_symbols=0;
  int intr_priority=0;
  struct { unsigned invert_in:1; unsigned with_dma:1; unsigned io_loop_back:1; unsigned allow_pd:1; } flags{};
};
struct rmt_rx_done_event_data_t { size_t num_symbols=0; };
using rmt_rx_done_callback_t = bool (*)(rmt_channel_handle_t, const rmt_rx_done_event_data_t*, void*);
struct rmt_rx_event_callbacks_t { rmt_rx_done_callback_t on_recv_done=nullptr; };
struct rmt_receive_config_t {
  uint32_t signal_range_min_ns=0;
  uint32_t signal_range_max_ns=0;
  struct { unsigned en_partial_rx:1; } flags{};
};
inline esp_err_t rmt_new_rx_channel(const rmt_rx_channel_config_t*, rmt_channel_handle_t *out) { if(out) *out=new StubRmtChannel{}; return ESP_OK; }
inline esp_err_t rmt_rx_register_event_callbacks(rmt_channel_handle_t, const rmt_rx_event_callbacks_t*, void*) { return ESP_OK; }
inline esp_err_t rmt_receive(rmt_channel_handle_t, void*, size_t, const rmt_receive_config_t*) { return ESP_OK; }
inline esp_err_t rmt_enable(rmt_channel_handle_t) { return ESP_OK; }
inline esp_err_t rmt_disable(rmt_channel_handle_t) { return ESP_OK; }
