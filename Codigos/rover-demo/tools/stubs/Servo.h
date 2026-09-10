#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
/*
 * Stub de Servo que EMULA LA SEMANTICA DE ZEPHYR, no la de AVR.
 *
 * La implementacion zephyr deja servoIndex = 255 en el constructor y NO
 * comprueba el indice en write(): indexa servos[255] sobre un array de 16.
 * O sea que write() antes de attach() corrompe memoria y tumba el MCU.
 * La de AVR, en cambio, lo ignora silenciosamente. Este stub aborta para que
 * ese error se cace aqui y no en la placa.
 */
struct Servo {
  int _idx = 255;
  int _deg = -1;
  uint8_t attach(int){ _idx = 0; return 0; }
  void detach(){ _idx = 255; }
  bool attached(){ return _idx != 255; }
  void write(int v){
    if (_idx == 255) {
      std::fprintf(stderr,
        "\n*** Servo::write() ANTES de attach(): en zephyr esto indexa "
        "servos[255] sobre un array de 16 y tumba el MCU ***\n");
      std::abort();
    }
    _deg = v;
  }
  int read(){ return _deg; }
};
