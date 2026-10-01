#pragma once
#include <string>
static constexpr int WIFI_STA=1;
class WiFiClass { public: void mode(int){} void disconnect(){} const char* macAddress(){return "00:00:00:00:00:00";} };
extern WiFiClass WiFi;
