#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_APDS9960.h>
#include "config.h"

/*
 * APDS-9960 del piso, en Wire (D20/D21). Solo LEE r,g,b,c crudos: la
 * clasificacion vive en el MPU (python/colors.py), donde calibrar es editar
 * un archivo y reiniciar la App.
 */
class ColorSense {
public:
  void begin() {
    Wire.begin();
    _ok = _apds.begin(APDS_INTEGRATION_MS, APDS_GAIN, APDS9960_ADDRESS, &Wire);
    if (_ok) _apds.enableColor(true);
  }

  bool ok() const { return _ok; }

  // Devuelve true si hizo una transaccion I2C en esta pasada (para medirla).
  bool tick() {
    uint32_t now = millis();
    if (!_ok || now - _lastMs < APDS_POLL_MS) return false;
    _lastMs = now;
    if (_apds.colorDataReady()) {
      _apds.getColorData(&_r, &_g, &_b, &_c);
      _fresh++;
    }
    return true;
  }

  uint16_t r() const { return _r; }
  uint16_t g() const { return _g; }
  uint16_t b() const { return _b; }
  uint16_t c() const { return _c; }
  uint32_t fresh() const { return _fresh; }   // lecturas nuevas acumuladas

private:
  Adafruit_APDS9960 _apds;
  bool _ok = false;
  uint16_t _r = 0, _g = 0, _b = 0, _c = 0;
  uint32_t _fresh = 0;
  uint32_t _lastMs = 0;
};
