#include "Arduino.h"
#include "Wire.h"
#include "Arduino_RouterBridge.h"
uint32_t g_millis = 0; int g_pinLevel[64]; int g_pinMode[64];
TwoWire Wire; BridgeT Bridge; Print Monitor;
#include "sketch.ino"
int main(){ setup(); loop(); return 0; }
