#pragma once
#include "Wire.h"
#define APDS9960_ADDRESS 0x39
enum apds9960AGain_t { APDS9960_AGAIN_1X, APDS9960_AGAIN_4X, APDS9960_AGAIN_16X, APDS9960_AGAIN_64X };
struct Adafruit_APDS9960 {
  bool begin(uint16_t, apds9960AGain_t, uint8_t, TwoWire*){ return true; }
  void enableColor(bool){}
  bool colorDataReady(){ return true; }
  void getColorData(uint16_t*,uint16_t*,uint16_t*,uint16_t*){}
};
