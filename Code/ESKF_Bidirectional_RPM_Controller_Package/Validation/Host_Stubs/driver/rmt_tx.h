#pragma once
#include <cstddef>
#include <cstdint>
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
struct rmt_tx_channel_config_t {
  gpio_num_t gpio_num=GPIO_NUM_NC;
  int clk_src=0;
  uint32_t resolution_hz=0;
  size_t mem_block_symbols=0;
  size_t trans_queue_depth=0;
  int intr_priority=0;
  struct {
    unsigned invert_out:1;
    unsigned with_dma:1;
    unsigned io_loop_back:1;
    unsigned io_od_mode:1;
    unsigned allow_pd:1;
    unsigned init_level:1;
  } flags{};
};
struct rmt_tx_done_event_data_t {};
using rmt_tx_done_callback_t = bool (*)(rmt_channel_handle_t, const rmt_tx_done_event_data_t*, void*);
struct rmt_tx_event_callbacks_t { rmt_tx_done_callback_t on_trans_done=nullptr; };
struct rmt_transmit_config_t {
  int loop_count=0;
  struct { unsigned eot_level:1; unsigned queue_nonblocking:1; } flags{};
};
inline esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t*, rmt_channel_handle_t *out) { if(out) *out=new StubRmtChannel{}; return ESP_OK; }
inline esp_err_t rmt_tx_register_event_callbacks(rmt_channel_handle_t, const rmt_tx_event_callbacks_t*, void*) { return ESP_OK; }
inline esp_err_t rmt_transmit(rmt_channel_handle_t, rmt_encoder_handle_t, const void*, size_t, const rmt_transmit_config_t*) { return ESP_OK; }
inline esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t, int) { return ESP_OK; }
