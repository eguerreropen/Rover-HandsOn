/*
 * Pruebas del codigo del MCU que se pueden EJECUTAR en un PC.
 *
 *     tools/test_sketch.sh
 *
 * POR QUE EXISTE
 * --------------
 * Hasta ahora del sketch solo se comprobaba que COMPILARA. Y compilar no
 * basta: el fallo que dejo el rover mudo durante dos sesiones compilaba
 * perfectamente. Era una llamada a Servo::write() antes de attach(), que en
 * la implementacion zephyr indexa servos[255] sobre un array de 16, corrompe
 * memoria y tumba el MCU dentro de setup() -- con lo que no se registraba
 * ningun RPC y desde el MPU todo respondia "method not available".
 *
 * Los stubs de /tmp/stubq emulan la semantica de ZEPHYR (no la de AVR, que
 * ignora ese error en silencio): Servo::write() sin attach() aborta.
 */
#include <cassert>
#include <cstdio>
#include "Arduino.h"
#include "Wire.h"
#include "Arduino_RouterBridge.h"

TwoWire Wire;
BridgeT Bridge;
Print Monitor;

#include "actuators.h"
#include "motors.h"
#include "signals.h"

static int fallos = 0;
static void ok(const char *m) { std::printf("  ok   %s\n", m); }

int main() {
  std::printf("ROVER H07 - pruebas del codigo del MCU\n\n");

  // --- Actuators: el orden attach/write es lo que tumbaba la placa ---
  {
    Actuators act;
    act.begin();                       // si escribe antes de enganchar, aborta
    assert(act.attached(Actuators::KEY));
    assert(act.angle(Actuators::KEY) == SERVO_KEY_HOLD_DEFAULT);
    ok("begin(): engancha el servo ANTES de escribirle, y queda en retencion");
  }
  {
    Actuators act;
    act.begin();
    assert(act.setRaw(Actuators::KEY, 180));
    assert(act.angle(Actuators::KEY) == 180);
    assert(!act.setRaw(99, 90));                // id fuera de rango
    assert(!act.setRaw(-1, 90));
    assert(act.setRaw(Actuators::KEY, 999));    // se recorta a 0..180
    assert(act.angle(Actuators::KEY) == 180);
    assert(act.setRaw(Actuators::KEY, -50));
    assert(act.angle(Actuators::KEY) == 0);
    ok("setRaw(): id validado, y el angulo se recorta a 0..180");
  }
  // --- PARTE 4: elevacion y gripper. El mismo fallo que tumbo la placa
  //     con la llave se repetiria identico aqui si move() se olvidara de
  //     enganchar uno de los tres. El stub aborta si eso pasa.
  {
    Actuators act;
    act.begin();
    // begin() NO engancha elevacion ni gripper: el brazo se coloca a mano.
    assert(!act.attached(Actuators::LIFT));
    assert(!act.attached(Actuators::GRIP));
    assert(act.angle(Actuators::LIFT) == -1);

    assert(act.setRaw(Actuators::LIFT, 40));    // primera orden -> engancha
    assert(act.attached(Actuators::LIFT));
    assert(act.angle(Actuators::LIFT) == 40);
    assert(act.setRaw(Actuators::GRIP, 90));
    assert(act.attached(Actuators::GRIP));
    assert(act.angle(Actuators::GRIP) == 90);
    // Los tres son independientes: apagar uno no toca a los otros.
    assert(act.off(Actuators::GRIP));
    assert(!act.attached(Actuators::GRIP));
    assert(act.attached(Actuators::LIFT) && act.attached(Actuators::KEY));
    assert(act.setRaw(Actuators::GRIP, 0));     // re-enganche
    assert(act.attached(Actuators::GRIP));
    ok("los TRES servos: enganchan antes de escribir y son independientes");
  }
  {
    Actuators act;
    act.begin();
    assert(act.off(Actuators::KEY));
    assert(!act.attached(Actuators::KEY));       // sin pulsos
    assert(act.off(Actuators::KEY));             // apagar dos veces no rompe
    assert(act.setRaw(Actuators::KEY, 135));     // RE-ENGANCHE: no debe abortar
    assert(act.attached(Actuators::KEY));
    assert(act.angle(Actuators::KEY) == 135);
    ok("off() + re-enganche: vuelve a enganchar antes de escribir");
  }

  // --- Motors: el bloqueo direccional no debe impedir el giro ---
  {
    Motors m;
    m.begin();
    m.setBlocked(true, false);                   // frente bloqueado
    m.set(500, 500);
    for (int i = 0; i < 200; i++) m.tick();
    assert(m.left() == 0 && m.right() == 0);
    m.set(500, -500);                            // giro sobre el eje
    for (int i = 0; i < 200; i++) m.tick();
    assert(m.left() != 0 || m.right() != 0);
    ok("bloqueo de borde: corta el avance pero NUNCA el giro de escape");
  }
  {
    Motors m;
    m.begin();
    m.set(1000, 1000);
    m.tick();
    assert(m.left() > 0 && m.left() < 1000);     // rampa, no salto
    ok("rampa: no salta de 0 al maximo en un tick");
  }

  // --- Signals: modos del LED de bandera ---
  {
    Signals s;
    s.begin();
    assert(s.flag() == 0);
    s.setFlag(2); assert(s.flag() == 2);
    s.setFlag(9); assert(s.flag() == 9);          // el sketch ya lo recorta
    s.tick();                                     // no debe romperse
    ok("LED de bandera: modos y tick()");
  }

  std::printf("\n%s\n", fallos ? "HAY FALLOS" : "todas OK");
  return fallos;
}
