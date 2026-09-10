#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "timing.h"

/*
 * Servos. PARTE 4: los tres (elevacion D6, gripper D9, llave D10).
 *
 * El MCU NO decide angulos: los manda el MPU (config.py tiene HOLD y
 * RELEASE). Aqui solo hay un valor de ARRANQUE, porque el servo tiene que ir
 * a la posicion de retencion en cuanto hay energia -antes de que Linux
 * termine de arrancar- para que la llave se pueda cargar a mano y quede
 * sujeta.
 *
 * Dos cosas que no son cosmeticas:
 *
 * 1. Limites por software (0..180). Un servo de 20 kg que se pasa de rango
 *    choca contra el chasis y entra en corriente de bloqueo: hunde la
 *    tension y reinicia placas.
 *
 * 2. Bloqueo de motores mientras un servo se mueve (MOVE_LOCK_MS). Motores
 *    arrancando + un servo de 20 kg arrancando a la vez es el pico de
 *    corriente mas alto del rover. Se serializa a proposito.
 */
class Actuators {
public:
  enum Id { LIFT = 0, GRIP = 1, KEY = 2, COUNT = 3 };

  void begin() {
    // SOLO la llave se engancha al arrancar, y a proposito. La llave se carga
    // A MANO antes de la ronda y tiene que quedar sujeta desde que hay
    // energia, antes incluso de que Linux termine de arrancar. La elevacion y
    // el gripper es al reves: el brazo se coloca a mano, y un servo de 20 kg
    // empujando mientras lo colocas es una forma de romper el mecanismo o el
    // servo. Esos dos se enganchan solos con la primera orden del MPU.
    move(KEY, SERVO_KEY_HOLD_DEFAULT);
    _lastMoveMs = 0;                     // el movimiento de arranque no bloquea
  }

  bool setRaw(int id, int deg) {
    if (id < 0 || id >= COUNT) return false;
    move(id, constrain(deg, 0, 180));
    return true;
  }

  /*
   * APAGAR el servo: detach() corta el tren de pulsos y el servo deja de
   * empujar. Sirve para que, con la llave ya en el suelo, un servo de 20 kg
   * no se quede horas en corriente de bloqueo contra su tope (zumbido,
   * calor, bateria, y el rail de 5 V del UBEC hundido).
   *
   * OJO: write() es asincrono. detach() ANTES de que el servo llegue
   * fisicamente lo deja a medio camino. El MPU espera SERVO_SETTLE_S desde
   * la ultima orden antes de mandar esto; aqui no se comprueba a proposito,
   * porque el MCU no sabe donde esta el eje de verdad.
   *
   * Sin pulsos el eje queda libre: el reductor de un servo de 20 kg lo
   * retiene bajo carga ligera, pero no es una posicion garantizada. Por eso
   * al volver a mover (setRaw) se re-engancha solo.
   */
  bool off(int id) {
    if (id < 0 || id >= COUNT) return false;
    if (_on[id]) { _servo[id].detach(); _on[id] = false; }
    return true;
  }

  // Estado propio y no Servo::attached(): esa funcion no es const en la
  // libreria, y ademas asi el estado que se reporta es el que ESTA clase
  // decidio, sin depender de detalles internos de la implementacion.
  bool attached(int id) const { return (id >= 0 && id < COUNT) && _on[id]; }
  int  angle(int id)    const { return (id >= 0 && id < COUNT) ? _angle[id] : -1; }

  // true mientras haya que mantener los motores quietos por el pico.
  bool motorsLocked() const {
    return _lastMoveMs && elapsedSince(_lastMoveMs, millis()) < MOVE_LOCK_MS;
  }
  bool settling() const {
    return _lastMoveMs && elapsedSince(_lastMoveMs, millis()) < SERVO_MOVE_MS;
  }

private:
  // Un array indexado por Id, no tres miembros sueltos: asi move() y off()
  // no pueden olvidarse de un servo al anadirlo, que es exactamente el tipo
  // de olvido que dejo el gripper sin enganchar en la primera version.
  Servo _servo[COUNT];
  bool _on[COUNT] = {false, false, false};
  static const uint8_t _pin[COUNT];
  int _angle[COUNT] = {-1, -1, -1};
  uint32_t _lastMoveMs = 0;

  void move(int id, int deg) {
    {
      /*
       * ATTACH SIEMPRE ANTES QUE WRITE. NO LO INVIERTAS.
       *
       * Aqui hubo una version que escribia ANTES de enganchar, con la idea
       * de evitar un supuesto salto a 90 grados al hacer attach(). Esa idea
       * venia de la implementacion AVR de Servo y en zephyr es FALSA y
       * ADEMAS CORRUPTA MEMORIA:
       *
       *   Servo::Servo()  deja servoIndex = 255
       *   Servo::write()  ->  servo_handle.getMin(servoIndex)
       *                   ->  servos[255], sobre un array de 16
       *
       * La implementacion zephyr NO comprueba el indice (la de AVR si: usa
       * "if (channel < MAX_SERVOS)"). Un write() antes del attach() lee 239
       * posiciones mas alla del array y deja de rebote un puntero basura;
       * si no es nulo, lo desreferencia y escribe en el. Resultado: el MCU
       * se cae DENTRO de setup(), no llega a registrar los RPC, y desde el
       * MPU se ve como "method not available" en todas las llamadas.
       *
       * Y el miedo al salto a 90 no existia: attach() hace
       * "new servoTimer_t()", que inicializa position_tick a 0, y con eso el
       * pin se queda en LOW sin emitir pulso hasta el primer write().
       */
      if (!_on[id]) {
        _servo[id].attach(_pin[id]);
        _on[id] = true;
      }
      _servo[id].write(deg);
    }
    _angle[id] = deg;
    _lastMoveMs = millis();
  }
};

// Fuera de la clase: el orden TIENE que seguir a enum Id { LIFT, GRIP, KEY }.
const uint8_t Actuators::_pin[Actuators::COUNT] = {
    PIN_SERVO_LIFT, PIN_SERVO_GRIP, PIN_SERVO_KEY};
