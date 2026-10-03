#pragma once
#include <Arduino.h>
#include "config.h"

/*
 * Cuatro TCRT-5000 en las esquinas, leidos por ADC.
 * Mascara: bit0 FL, bit1 FR, bit2 RL, bit3 RR.
 *
 * El umbral y la histeresis son VARIABLES: el MPU los manda al arrancar
 * (cfg_lines). Los #define son solo el valor de arranque.
 */
class Lines {
public:
  enum { FL = 0, FR = 1, RL = 2, RR = 3, COUNT = 4 };

  void begin() {
    analogReadResolution(LINE_ADC_BITS);       // fijar la escala: 0..4095
    for (int i = 0; i < COUNT; i++) pinMode(_pin[i], INPUT);
  }

  void configure(uint16_t blackBelow, uint16_t hysteresis) {
    _blackBelow = blackBelow;
    _hyst = hysteresis;
  }
  uint16_t blackBelow() const { return _blackBelow; }

  void tick() {
    for (int i = 0; i < COUNT; i++) {
      if (!present(i)) {                       // pin al aire: no miente
        _raw[i] = LINE_ADC_MAX;
        _black[i] = false;
        continue;
      }
      _raw[i] = readAvg(_pin[i]);
      _black[i] = _black[i] ? (_raw[i] < _blackBelow + _hyst)
                            : (_raw[i] < _blackBelow);
    }
  }

  static bool present(int i) { return (LINE_PRESENT_MASK >> i) & 1; }
  static bool frontPresent() { return present(FL) && present(FR); }
  static bool rearPresent()  { return present(RL) && present(RR); }

  bool frontBlack() const { return _black[FL] || _black[FR]; }
  bool rearBlack()  const { return _black[RL] || _black[RR]; }

  uint8_t blackMask() const {
    uint8_t m = 0;
    for (int i = 0; i < COUNT; i++) if (_black[i]) m |= (1 << i);
    return m;
  }
  uint16_t raw(int i) const { return _raw[constrain(i, 0, COUNT - 1)]; }

private:
  const uint8_t _pin[COUNT] = {PIN_LINE_FL, PIN_LINE_FR, PIN_LINE_RL, PIN_LINE_RR};
  uint16_t _raw[COUNT]   = {LINE_ADC_MAX, LINE_ADC_MAX, LINE_ADC_MAX, LINE_ADC_MAX};
  bool     _black[COUNT] = {false, false, false, false};
  uint16_t _blackBelow = LINE_BLACK_BELOW_DEFAULT;
  uint16_t _hyst = LINE_HYST_DEFAULT;

  // Unico punto que conoce la polaridad del modulo. Desde aqui, todo habla
  // en "menos = mas oscuro".
  static uint16_t readAvg(uint8_t pin) {
    uint32_t acc = 0;
    for (int i = 0; i < LINE_SAMPLES; i++) acc += analogRead(pin);
    uint16_t v = (uint16_t)(acc / LINE_SAMPLES);
#if LINE_INVERT
    v = (uint16_t)(LINE_ADC_MAX - v);
#endif
    return v;
  }
};
