#pragma once
#include <Arduino.h>

/*
 * Comparacion de marcas de millis() A PRUEBA DE MARCAS "DEL FUTURO".
 *
 * Es la leccion mas cara de la parte 1. loop() toma now = millis() al
 * principio y lo usa en todo el cuerpo; los handlers del Bridge escriben sus
 * marcas con un millis() propio. Si un handler se despacha DESPUES de que
 * loop() tomara su now, la marca es mas nueva que now, y entonces
 * (now - marca) sin signo se desborda a ~4.29e9: "han pasado 49 dias". Asi
 * saltaba el watchdog con las ordenes llegando perfectamente.
 *
 * Regla: TODA resta de marcas de tiempo pasa por aqui. La resta con signo
 * trata una marca del futuro como "ahora mismo" (0 ms), y ademas sigue
 * siendo correcta cuando millis() da la vuelta a los 49 dias.
 *
 * g_futureMarks cuenta cuantas veces ocurrio: es el contador que demostro la
 * causa, y sigue reportandose en la telemetria por si vuelve a subir.
 */
static uint16_t g_futureMarks = 0;

static inline uint32_t elapsedSince(uint32_t mark, uint32_t nowMs) {
  int32_t d = (int32_t)(nowMs - mark);
  if (d < 0) { g_futureMarks++; return 0; }
  return (uint32_t)d;
}
