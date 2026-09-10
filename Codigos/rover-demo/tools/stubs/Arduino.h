#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
using std::min; using std::max;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define A0 14
#define A1 15
#define A2 16
#define A3 17
template<class T, class L, class H> T constrain(T v, L lo, H hi){ return v<lo?(T)lo:(v>hi?(T)hi:v); }
inline uint32_t millis(){ return 0; }
inline void pinMode(uint8_t,int){}
inline void digitalWrite(uint8_t,int){}
inline void analogWrite(uint8_t,int){}
inline int analogRead(uint8_t){ return 0; }
inline void analogReadResolution(int){}
struct Print { void print(const char*){} void print(int){} void print(unsigned){} void print(long){} void print(unsigned long){} void print(uint8_t){} void println(const char*){} void println(int){} void println(unsigned){} void println(unsigned long){} void println(uint8_t){} void begin(){} };
