#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ROOT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
RECEIVER_DIR="$ROOT_DIR/Receiver/ESKF_Model_Based_Drone_Controller"
BUILD_DIR="$SCRIPT_DIR/build"

mkdir -p "$BUILD_DIR"

WARNINGS="-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wdouble-promotion -Wformat=2 -Wundef -Wcast-qual -Wcast-align -Wnull-dereference -Wold-style-cast -Woverloaded-virtual -Wnon-virtual-dtor"
CXXFLAGS="-std=c++17 $WARNINGS -I$RECEIVER_DIR"

c++ $CXXFLAGS "$SCRIPT_DIR/test_eskf.cpp" -o "$BUILD_DIR/test_eskf"
c++ $CXXFLAGS "$SCRIPT_DIR/test_motion_model.cpp" -o "$BUILD_DIR/test_motion_model"
c++ $CXXFLAGS "$SCRIPT_DIR/test_signs.cpp" -o "$BUILD_DIR/test_signs"
c++ $CXXFLAGS "$SCRIPT_DIR/test_rotor_feedback.cpp" -o "$BUILD_DIR/test_rotor_feedback"

"$BUILD_DIR/test_eskf"
"$BUILD_DIR/test_motion_model"
"$BUILD_DIR/test_signs"
"$BUILD_DIR/test_rotor_feedback"

FIRMWARE_FLAGS="-std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Wformat=2 -Wundef -DCONFIG_IDF_TARGET_ESP32=1 -I$SCRIPT_DIR/Host_Stubs -I$RECEIVER_DIR"

c++ $FIRMWARE_FLAGS -x c++ -fsyntax-only \
  "$RECEIVER_DIR/ESKF_Model_Based_Drone_Controller.ino" \
  "$RECEIVER_DIR/BidirectionalDShot.cpp"

# Arduino-ESP32 releases have used GNU C++11-compatible builds; keep a second
# syntax pass so the local headers do not accidentally rely on C++17-only code.
FIRMWARE_FLAGS_CXX11="-std=gnu++11 -Wall -Wextra -Wpedantic -Wshadow -Wformat=2 -Wundef -DCONFIG_IDF_TARGET_ESP32=1 -I$SCRIPT_DIR/Host_Stubs -I$RECEIVER_DIR"
c++ $FIRMWARE_FLAGS_CXX11 -x c++ -fsyntax-only \
  "$RECEIVER_DIR/ESKF_Model_Based_Drone_Controller.ino" \
  "$RECEIVER_DIR/BidirectionalDShot.cpp"

c++ $FIRMWARE_FLAGS -fsanitize=address,undefined -fno-omit-frame-pointer \
  -x c++ "$RECEIVER_DIR/ESKF_Model_Based_Drone_Controller.ino" \
  "$RECEIVER_DIR/BidirectionalDShot.cpp" \
  "$SCRIPT_DIR/test_firmware_main.cpp" \
  "$SCRIPT_DIR/Host_Stubs/Arduino.cpp" \
  "$SCRIPT_DIR/Host_Stubs/SPI.cpp" \
  "$SCRIPT_DIR/Host_Stubs/WiFi.cpp" \
  "$SCRIPT_DIR/Host_Stubs/Wire.cpp" \
  "$SCRIPT_DIR/Host_Stubs/RmtStubs.cpp" \
  -o "$BUILD_DIR/test_firmware"

ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
  "$BUILD_DIR/test_firmware"

echo "All host validation checks passed."
