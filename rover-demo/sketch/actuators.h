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
    // NINGUN servo se engancha al arrancar.
    //
    // La elevacion y el gripper, porque el brazo se coloca a mano y un servo
    // de 20 kg empujando mientras lo colocas rompe el mecanismo.
    //
    // Y LA CAJA, porque desde el 28/09 es un servo de ROTACION CONTINUA. Antes
    // aqui habia un move(KEY, 180) para retener la llave desde que hay
    // corriente; en un servo continuo eso es "maxima velocidad, sin fin",
    // antes incluso de que arranque Linux. Sin pulsos, un servo continuo
    // esta parado: es el estado seguro.
    _lastMoveMs = 0;
  }

  /*
   * Servo de POSICION a un angulo. Para elevacion y gripper.
   *
   * LA CAJA LO RECHAZA. Es un servo continuo: setRaw(KEY, x) lo pondria a
   * girar SIN LIMITE DE TIEMPO hasta que alguien se acordara de apagarlo. El
   * unico camino para mover la caja es spin(), que siempre se para solo. Asi
   * no existe ninguna orden -ni de la FSM, ni de un boton de la pagina, ni de
   * un curl a mano- que pueda dejarla girando.
   */
  bool setRaw(int id, int deg) {
    if (id < 0 || id >= COUNT) return false;
    if (id == KEY) return false;
    move(id, constrain(deg, 0, 180));
    return true;
  }

  /*
   * GIRAR la caja 'ms' milisegundos a 'deg' (90 ~ parado) y PARAR SOLO.
   *
   * El tiempo lo cuenta el MCU en tick(), no el MPU. Tres razones:
   *   - es como se calibro en la App de prueba: los 0.3 s medidos alli son
   *     exactamente los de aqui. Temporizarlo desde Python (a 20 Hz, mas la
   *     latencia del Bridge) daria +-50 ms: un 17 % de un giro de 0.3 s.
   *   - si la App de Python se cae a mitad del giro, el servo se para igual.
   *   - se para quitando los pulsos (detach), no con write(90): el neutro de
   *     un servo continuo casi nunca es 90 exacto y suele arrastrar un poco.
   */
  bool spin(int id, int deg, int ms) {
    if (id != KEY) return false;               // solo la caja es continua
    deg = constrain(deg, 0, 180);
    ms  = constrain(ms, 0, KEY_SPIN_MAX_MS);
    move(id, deg);                             // attach ANTES que write
    _spinStart = millis();
    _spinDur   = (uint32_t)ms;
    _spinning  = true;
    return true;
  }

  bool spinning() const { return _spinning; }

  // Llamar en cada loop(): para el giro cuando se acaba su tiempo. Resta CON
  // SIGNO (la leccion del watchdog fantasma): sobrevive al desbordamiento de
  // millis() sin quedarse girando.
  void tick() {
    if (_spinning && (int32_t)(millis() - _spinStart) >= (int32_t)_spinDur) {
      _spinning = false;
      off(KEY);
    }
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
    if (id == KEY) _spinning = false;          // parar a mano cancela el giro
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
  bool     _spinning  = false;
  uint32_t _spinStart = 0;
  uint32_t _spinDur   = 0;

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
