#pragma once
#include "Arduino.h"
struct BridgeT { void begin(){} template<class F> void provide_safe(const char*, F){} template<class F> void provide(const char*, F){} };
extern BridgeT Bridge;
extern Print Monitor;
