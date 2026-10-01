#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <strings.h>

using std::size_t;
#ifndef GPIO_NUM_T_DEFINED
#define GPIO_NUM_T_DEFINED
using gpio_num_t = int;
#endif
static constexpr gpio_num_t GPIO_NUM_26=26, GPIO_NUM_14=14, GPIO_NUM_27=27, GPIO_NUM_25=25;
static constexpr int HIGH=1, LOW=0, OUTPUT=1, INPUT_PULLUP=2;
static constexpr int HEX=16;
#ifndef CONFIG_IDF_TARGET_ESP32
#define CONFIG_IDF_TARGET_ESP32 1
#endif
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

class HardwareSerial {
public:
  void setTxBufferSize(size_t) {}
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  template<typename T> void print(const T&) {}
  template<typename T> void print(const T&, int) {}
  template<typename T> void println(const T&) {}
  template<typename T> void println(const T&, int) {}
  void println() {}
};
extern HardwareSerial Serial;
inline uint32_t millis(){ static uint32_t x=0; return x+=1; }
inline uint32_t micros(){ static uint32_t x=0; return x+=1000; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(int,int) {}
inline void digitalWrite(int,int) {}
inline int digitalRead(int){return HIGH;}

using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) do { (void)(x); } while(0)
#define portEXIT_CRITICAL(x) do { (void)(x); } while(0)
