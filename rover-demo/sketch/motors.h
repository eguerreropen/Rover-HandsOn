#pragma once
#include <Arduino.h>
#include "config.h"

/*
 * Traccion diferencial sobre L298N. API en velocidades normalizadas
 * [-1000, 1000]; el PWM y la zona muerta son asunto exclusivo de esta clase.
 */
class Motors {
public:
  void begin() {
    pinMode(PIN_MOT_L_EN, OUTPUT);  pinMode(PIN_MOT_L_IN1, OUTPUT);  pinMode(PIN_MOT_L_IN2, OUTPUT);
    pinMode(PIN_MOT_R_EN, OUTPUT);  pinMode(PIN_MOT_R_IN1, OUTPUT);  pinMode(PIN_MOT_R_IN2, OUTPUT);
    stop();
  }

  void set(int left, int right) {
    _tgtL = constrain(left,  -1000, 1000);
    _tgtR = constrain(right, -1000, 1000);
  }

  void stop() {
    _tgtL = _tgtR = _curL = _curR = 0;
    apply(0, 0);
  }

  // Bloqueo por sentido: frente en negro -> no avanzar; atras -> no
  // retroceder; los dos -> solo giro. El giro NUNCA se bloquea: el rover
  // tiene que poder salir del borde.
  void setBlocked(bool forward, bool reverse) { _fwdBlocked = forward; _revBlocked = reverse; }

  void tick() {
    int tl = _tgtL, tr = _tgtR;
    if (_fwdBlocked && tl > 0 && tr > 0) { tl = 0; tr = 0; }
    if (_revBlocked && tl < 0 && tr < 0) { tl = 0; tr = 0; }
    _curL = slew(_curL, tl);
    _curR = slew(_curR, tr);
    apply(_curL, _curR);
  }

  int left()  const { return _curL; }
  int right() const { return _curR; }

  // Lo que el MPU esta PIDIENDO, antes de bloqueos y rampa: el enclavamiento
  // lo usa para distinguir "insiste contra la linea" de "esta maniobrando".
  bool pushingForward() const { return _tgtL > 0 && _tgtR > 0; }
  bool pushingReverse() const { return _tgtL < 0 && _tgtR < 0; }

private:
  int  _tgtL = 0, _tgtR = 0, _curL = 0, _curR = 0;
  bool _fwdBlocked = false, _revBlocked = false;

  static int slew(int cur, int tgt) {
    const int step = (MOTOR_SLEW_PER_TICK * 1000) / MOTOR_MAX_PWM;
    if (tgt > cur) return min(tgt, cur + step);
    if (tgt < cur) return max(tgt, cur - step);
    return cur;
  }

  static uint8_t toPwm(int norm) {
    int mag = abs(norm);
    if (mag == 0) return 0;
    long pwm = MOTOR_DEADZONE_PWM +
               ((long)(MOTOR_MAX_PWM - MOTOR_DEADZONE_PWM) * mag) / 1000L;
    return (uint8_t)constrain(pwm, 0, MOTOR_MAX_PWM);
  }

  static void one(int norm, bool inv, uint8_t en, uint8_t in1, uint8_t in2) {
    if (inv) norm = -norm;
    if (norm == 0) {                       // freno activo
      digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); analogWrite(en, 0);
      return;
    }
    digitalWrite(in1, norm > 0 ? HIGH : LOW);
    digitalWrite(in2, norm > 0 ? LOW  : HIGH);
    analogWrite(en, toPwm(norm));
  }

  static void apply(int l, int r) {
    one(l, MOTOR_L_INVERT, PIN_MOT_L_EN, PIN_MOT_L_IN1, PIN_MOT_L_IN2);
    one(r, MOTOR_R_INVERT, PIN_MOT_R_EN, PIN_MOT_R_IN1, PIN_MOT_R_IN2);
  }
};
