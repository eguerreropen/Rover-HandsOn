#include "Arduino.h"
#include "Wire.h"
#include "Arduino_RouterBridge.h"
TwoWire Wire; BridgeT Bridge; Print Monitor;
#include "sketch.ino"
int main(){ setup(); loop(); return 0; }
