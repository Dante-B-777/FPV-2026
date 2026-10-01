#pragma once
#include <cstdint>
struct StubFuncOutSelCfg { uint32_t oen_sel=0; uint32_t oen_inv_sel=0; };
struct gpio_dev_t {
  uint32_t enable_w1tc=0;
  uint32_t enable_w1ts=0;
  StubFuncOutSelCfg func_out_sel_cfg[40]{};
};
extern gpio_dev_t GPIO;
