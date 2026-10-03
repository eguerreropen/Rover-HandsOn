#pragma once
#include <Arduino.h>
#include "config.h"

/*
 * LED RGB (R=D13, G=D12, B=D11): senaliza EQUIPO y BANDERA a la vez.
 *
 *     EL TONO  dice de que equipo es el rover (rojo o azul). No cambia nunca.
 *     EL RITMO dice que pasa con la bandera:
 *         0 fijo             normal
 *         1 parpadeo lento   bandera rival a la vista
 *         2 parpadeo rapido  bandera rival en su poder
 *
 * POR QUE EL TONO NO SENALIZA LA BANDERA. Seria lo facil -verde cuando la ve,
 * por ejemplo- pero entonces el rover dejaria de mostrar su equipo justo en el
 * momento mas visible de la ronda, y el equipo es un requisito del reglamento.
 * El ritmo es un canal libre que no pisa al otro.
 *
 * EL PARPADEO SE GENERA AQUI, en el MCU, y no mandando RPCs desde el MPU: un
 * LED que parpadea a 5 Hz por el Bridge es trafico inutil en el mismo canal
 * que las ordenes de traccion.
 *
 * Solo D11 es PWM de los tres, asi que cada canal esta encendido o apagado.
 * Bastan dos colores, no hacen falta intensidades.
 */
class Signals {
public:
  // Equipo: 0 = rojo, 1 = azul. Lo manda el MPU desde config.TEAM, para que
  // no haya dos sitios donde decir de que equipo somos.
  enum Team { RED = 0, BLUE = 1 };

  void begin() {
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    write(false, false, false);
  }

  void setTeam(uint8_t t) { _team = (t == BLUE) ? BLUE : RED; }
  void setFlag(uint8_t mode) { _mode = mode > 2 ? 2 : mode; }

  // Prueba manual desde la pagina: enciende un color crudo durante un rato.
  // Sirve para averiguar si el modulo es comun anodo o comun catodo sin
  // recompilar nada -- si al pedir ROJO se enciende todo menos el rojo, hay
  // que cambiar RGB_COMMON_ANODE.
  void testColor(bool r, bool g, bool b, uint32_t ms) {
    _testR = r; _testG = g; _testB = b;
    _testUntil = millis() + ms;
  }

  uint8_t flag() const { return _mode; }
  uint8_t team() const { return _team; }

  void tick() {
    if (_testUntil && (int32_t)(millis() - _testUntil) < 0) {
      write(_testR, _testG, _testB);
      return;
    }
    _testUntil = 0;

    bool on = true;
    if (_mode == 1) on = (millis() / FLAG_BLINK_MS) % 2;
    else if (_mode == 2) on = (millis() / FLAG_BLINK_FAST_MS) % 2;

    if (!on) { write(false, false, false); return; }
    write(_team == RED, false, _team == BLUE);
  }

private:
  uint8_t _team = RED;
  uint8_t _mode = 0;
  bool _testR = false, _testG = false, _testB = false;
  uint32_t _testUntil = 0;

  static void one(uint8_t pin, bool encendido) {
    // La polaridad vive en UN solo sitio. Con comun anodo la pata comun va a
    // 5 V y el canal se enciende poniendo el pin a LOW: invertir esto en tres
    // llamadas distintas es como se acaba con un canal al reves.
#if RGB_COMMON_ANODE
    digitalWrite(pin, encendido ? LOW : HIGH);
#else
    digitalWrite(pin, encendido ? HIGH : LOW);
#endif
  }

  static void write(bool r, bool g, bool b) {
    one(PIN_LED_R, r);
    one(PIN_LED_G, g);
    one(PIN_LED_B, b);
  }
};
