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

"$BUILD_DIR/test_eskf"
"$BUILD_DIR/test_motion_model"
"$BUILD_DIR/test_signs"

FIRMWARE_FLAGS="-std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Wformat=2 -Wundef -I$SCRIPT_DIR/Host_Stubs -I$RECEIVER_DIR"

c++ $FIRMWARE_FLAGS -x c++ -fsyntax-only \
  "$RECEIVER_DIR/ESKF_Model_Based_Drone_Controller.ino"

c++ $FIRMWARE_FLAGS -fsanitize=address,undefined -fno-omit-frame-pointer \
  -x c++ "$RECEIVER_DIR/ESKF_Model_Based_Drone_Controller.ino" \
  "$SCRIPT_DIR/test_firmware_main.cpp" \
  "$SCRIPT_DIR/Host_Stubs/Arduino.cpp" \
  "$SCRIPT_DIR/Host_Stubs/SPI.cpp" \
  "$SCRIPT_DIR/Host_Stubs/WiFi.cpp" \
  "$SCRIPT_DIR/Host_Stubs/Wire.cpp" \
  -o "$BUILD_DIR/test_firmware"

ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
  "$BUILD_DIR/test_firmware"

echo "All host validation checks passed."
