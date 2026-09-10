#pragma once
#include <Arduino.h>
#include "config.h"

/*
 * LED de bandera (D13): senaliza la deteccion de la bandera rival.
 *   0 = apagado   1 = parpadeo (bandera detectada)   2 = fijo (bandera en control)
 *
 * PARTE 3: solo este LED. Los de equipo (D11 rojo / D12 azul, requisito de
 * reglamento) se anaden cuando toque, con la misma clase.
 *
 * El parpadeo se genera aqui, en el MCU, y no desde el MPU: un LED que
 * parpadea con RPCs a 5 Hz por el Bridge es trafico inutil en el mismo canal
 * que las ordenes de traccion.
 */
class Signals {
public:
  void begin() {
    pinMode(PIN_LED_FLAG, OUTPUT);
    setFlag(0);
  }
  void setFlag(uint8_t mode) { _mode = mode; }
  uint8_t flag() const { return _mode; }

  void tick() {
    switch (_mode) {
      case 2:  digitalWrite(PIN_LED_FLAG, HIGH); break;
      case 1:  digitalWrite(PIN_LED_FLAG, (millis() / FLAG_BLINK_MS) % 2 ? HIGH : LOW); break;
      default: digitalWrite(PIN_LED_FLAG, LOW);
    }
  }

private:
  uint8_t _mode = 0;
};
