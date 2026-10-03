#pragma once
#include <Arduino.h>
#include "config.h"

/*
 * Pulsador de EQUIPO (A4 a GND, pull-up interno). Cada pulsacion la cuenta
 * el MCU; el MPU alterna rojo <-> azul.
 *
 * ANTIRREBOTE: un contacto mecanico rebota varios ms al cerrarse y al
 * abrirse. Un cambio de nivel solo se acepta cuando el pin lleva
 * TEAM_BTN_DEBOUNCE_MS sin moverse; cada rebote reinicia la cuenta.
 *
 * SE CUENTA LA BAJADA (suelto -> pulsado), no el nivel. Consecuencias:
 *   - mantenerlo pulsado cuenta UNA vez, no una por cada lectura;
 *   - si arranca ya pulsado (atascado, cortocircuito, dedo encima) NO cuenta:
 *     hay que soltarlo y volver a pulsar. Un boton roto no cambia el equipo;
 *   - sin boton conectado el pull-up deja el pin en HIGH y nunca cuenta.
 */
class TeamButton {
public:
  void begin() {
    pinMode(PIN_TEAM_BTN, INPUT_PULLUP);
    _raw = _stable = readPressed();
    _changedAt = millis();
    _presses = 0;
  }

  void tick() {
    bool r = readPressed();
    uint32_t now = millis();
    if (r != _raw) {                 // se movio: rebote o cambio, se espera
      _raw = r;
      _changedAt = now;
      return;
    }
    if (r != _stable && (int32_t)(now - _changedAt) >= TEAM_BTN_DEBOUNCE_MS) {
      _stable = r;
      if (r) _presses++;             // solo la BAJADA cuenta
    }
  }

  bool pressed() const { return _stable; }
  uint16_t presses() const { return _presses; }

private:
  static bool readPressed() { return digitalRead(PIN_TEAM_BTN) == LOW; }

  bool _raw = false, _stable = false;
  uint32_t _changedAt = 0;
  uint16_t _presses = 0;
};
