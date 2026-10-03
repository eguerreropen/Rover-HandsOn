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
#define INPUT_PULLUP 2
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
template<class T, class L, class H> T constrain(T v, L lo, H hi){ return v<lo?(T)lo:(v>hi?(T)hi:v); }
// Reloj CONTROLABLE desde la prueba. Sin esto no se puede comprobar que un
// parpadeo parpadea -- ni que un servo que gira por tiempo se para: millis()
// clavado en 0 deja todo en el mismo instante y la prueba pasaria sin mirar.
extern uint32_t g_millis;
inline uint32_t millis(){ return g_millis; }
inline void setMillis(uint32_t t){ g_millis = t; }
// Los pines RECUERDAN su ultimo estado. Sin esto no se puede comprobar que
// el LED RGB enciende el canal correcto ni que la polaridad de comun anodo /
// comun catodo esta bien: dos cosas que se escriben al reves con facilidad y
// que en la placa solo se ven con el LED en la mano.
extern int g_pinLevel[64];
extern int g_pinMode[64];
// INPUT_PULLUP deja el pin en HIGH, como en la placa con el boton suelto (o
// sin boton). Sin esto el pin empezaria en 0 = "pulsado" y la prueba mentiria.
// Pero si la prueba ya lo "pulso" (setPin), el pull-up no gana: un boton
// cerrado a GND manda sobre una resistencia de pull-up.
inline bool g_pinDriven[64];
inline void pinMode(uint8_t p,int m){ if(p<64){ g_pinMode[p]=m; if(m==INPUT_PULLUP && !g_pinDriven[p]) g_pinLevel[p]=HIGH; } }
inline int  digitalRead(uint8_t p){ return p<64 ? g_pinLevel[p] : HIGH; }
inline void setPin(uint8_t p,int v){ if(p<64){ g_pinLevel[p]=v; g_pinDriven[p]=true; } }   // la prueba "pulsa"
inline void digitalWrite(uint8_t p,int v){ if(p<64) g_pinLevel[p]=v; }
inline int  pinLevel(uint8_t p){ return p<64 ? g_pinLevel[p] : -1; }
inline void analogWrite(uint8_t,int){}
inline int analogRead(uint8_t){ return 0; }
inline void analogReadResolution(int){}
struct Print { void print(const char*){} void print(int){} void print(unsigned){} void print(long){} void print(unsigned long){} void print(uint8_t){} void println(const char*){} void println(int){} void println(unsigned){} void println(unsigned long){} void println(uint8_t){} void begin(){} };
