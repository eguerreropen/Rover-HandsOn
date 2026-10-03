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

uint32_t g_millis = 0;
int g_pinLevel[64];
int g_pinMode[64];
TwoWire Wire;
BridgeT Bridge;
Print Monitor;

#include "actuators.h"
#include "motors.h"
#include "signals.h"
#include "button.h"

static int fallos = 0;
static void ok(const char *m) { std::printf("  ok   %s\n", m); }

int main() {
  std::printf("ROVER H07 - pruebas del codigo del MCU\n\n");

  // --- Actuators: NADA se engancha al arrancar ---
  //
  // Desde el 28/09 la caja es un servo CONTINUO: un write() al arrancar la
  // pondria a girar sin fin antes de que Linux arrancara. Sin pulsos esta
  // parada, que es el estado seguro.
  {
    setMillis(0);
    Actuators act;
    act.begin();
    assert(!act.attached(Actuators::KEY));
    assert(!act.attached(Actuators::LIFT));
    assert(!act.attached(Actuators::GRIP));
    assert(!act.spinning());
    act.tick();
    assert(!act.attached(Actuators::KEY));
    ok("begin(): NINGUN servo con pulsos (la caja continua arranca parada)");
  }
  // --- setRaw(): posicion, solo para elevacion y gripper ---
  {
    Actuators act;
    act.begin();
    assert(act.setRaw(Actuators::LIFT, 40));
    assert(act.angle(Actuators::LIFT) == 40);
    assert(!act.setRaw(99, 90));                // id fuera de rango
    assert(!act.setRaw(-1, 90));
    assert(act.setRaw(Actuators::GRIP, 999));   // se recorta a 0..180
    assert(act.angle(Actuators::GRIP) == 180);
    assert(act.setRaw(Actuators::GRIP, -50));
    assert(act.angle(Actuators::GRIP) == 0);
    // LA CAJA NO ACEPTA setRaw: la dejaria girando sin limite de tiempo.
    assert(!act.setRaw(Actuators::KEY, 180));
    assert(!act.attached(Actuators::KEY));
    ok("setRaw(): recorta a 0..180, y RECHAZA la caja (la dejaria girando)");
  }
  // --- elevacion y gripper: enganchan antes de escribir, independientes ---
  {
    Actuators act;
    act.begin();
    assert(act.angle(Actuators::LIFT) == -1);
    assert(act.setRaw(Actuators::LIFT, 40));    // primera orden -> engancha
    assert(act.attached(Actuators::LIFT));
    assert(act.setRaw(Actuators::GRIP, 90));
    assert(act.attached(Actuators::GRIP));
    assert(act.off(Actuators::GRIP));
    assert(!act.attached(Actuators::GRIP));
    assert(act.attached(Actuators::LIFT));      // apagar uno no toca al otro
    assert(act.off(Actuators::GRIP));           // dos veces no rompe
    assert(act.setRaw(Actuators::GRIP, 0));     // re-enganche: no aborta
    assert(act.attached(Actuators::GRIP));
    ok("elevacion y gripper: enganchan antes de escribir y son independientes");
  }
  // --- LA CAJA: gira un tiempo y SE PARA SOLA ---
  //
  // Lo que importa de un servo continuo no es "va a tal angulo" -- no va a
  // ninguno --, sino que SE PARA: cuando se acaba el tiempo, cuando se le
  // pide, y aunque millis() desborde. Un fallo en cualquiera es la caja
  // girando sin fin.
  {
    Actuators act;
    act.begin();

    setMillis(1000);
    assert(act.spin(Actuators::KEY, 70, 300)); // attach ANTES que write
    assert(act.attached(Actuators::KEY) && act.spinning());
    assert(act.angle(Actuators::KEY) == 70);
    setMillis(1299); act.tick();
    assert(act.attached(Actuators::KEY));      // aun dentro del tiempo
    setMillis(1300); act.tick();
    assert(!act.attached(Actuators::KEY) && !act.spinning());   // parada sola

    // Recoger: sentido contrario, igual.
    setMillis(2000);
    assert(act.spin(Actuators::KEY, 115, 300));
    assert(act.angle(Actuators::KEY) == 115);
    setMillis(2300); act.tick();
    assert(!act.attached(Actuators::KEY));
    ok("caja: gira 0.3 s y se para SOLA, en los dos sentidos");

    // off() para en el acto y cancela el giro pendiente.
    setMillis(3000);
    act.spin(Actuators::KEY, 85, 2000);
    act.off(Actuators::KEY);
    assert(!act.attached(Actuators::KEY) && !act.spinning());
    ok("caja: off() la para en el acto");

    // Tope de seguridad: ni pidiendo un tiempo absurdo sigue girando.
    setMillis(4000);
    act.spin(Actuators::KEY, 0, 999999);
    setMillis(4000 + KEY_SPIN_MAX_MS); act.tick();
    assert(!act.attached(Actuators::KEY));
    ok("caja: tope de KEY_SPIN_MAX_MS aunque el MPU pida mas");

    // Solo la caja es continua: spin() en otro servo se rechaza.
    assert(!act.spin(Actuators::GRIP, 85, 300));
    assert(!act.spin(Actuators::LIFT, 85, 300));
    assert(!act.attached(Actuators::GRIP) && !act.attached(Actuators::LIFT));

    // Desbordamiento de millis() (~49 dias): resta con signo.
    //
    // El fallo de la version SIN signo no aparece despues de que millis() de
    // la vuelta, sino ANTES: "inicio + duracion" ya se ha desbordado a un
    // numero pequeno, y "millis() >= inicio + duracion" es cierto de
    // inmediato -- la caja se para a los 16 ms en vez de a los 300. Por eso
    // hay que mirar un instante ANTES de la vuelta, no solo despues.
    setMillis(0xFFFFFF00u);
    act.spin(Actuators::KEY, 85, 300);
    setMillis(0xFFFFFF10u); act.tick();        // 16 ms despues, sin desbordar
    assert(act.attached(Actuators::KEY));
    setMillis(0x00000010u); act.tick();        // 272 ms despues
    assert(act.attached(Actuators::KEY));
    setMillis(0x00000040u); act.tick();        // 320 ms despues
    assert(!act.attached(Actuators::KEY));
    ok("caja: solo spin() la mueve, y el desbordamiento de millis() no la deja girando");
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

  // --- Signals: el LED RGB senaliza EQUIPO y BANDERA a la vez ---
  //
  // Lo que se comprueba aqui no se puede comprobar de otra forma que con el
  // LED en la mano: que el canal encendido sea el del equipo correcto, y que
  // la polaridad (comun anodo / comun catodo) no este invertida. Las dos se
  // escriben al reves con una facilidad pasmosa.
  {
    const int ON  = RGB_COMMON_ANODE ? LOW  : HIGH;
    const int OFF = RGB_COMMON_ANODE ? HIGH : LOW;

    Signals s;
    s.begin();
    assert(g_pinMode[PIN_LED_R] == OUTPUT);
    assert(g_pinMode[PIN_LED_G] == OUTPUT);
    assert(g_pinMode[PIN_LED_B] == OUTPUT);
    assert(s.flag() == 0 && s.team() == Signals::RED);

    // EQUIPO ROJO: solo el canal rojo.
    s.setTeam(Signals::RED);
    s.setFlag(0);
    s.tick();
    assert(pinLevel(PIN_LED_R) == ON);
    assert(pinLevel(PIN_LED_G) == OFF);
    assert(pinLevel(PIN_LED_B) == OFF);

    // EQUIPO AZUL: solo el canal azul. Si esto falla, el rover se presenta
    // como del equipo contrario.
    s.setTeam(Signals::BLUE);
    s.tick();
    assert(pinLevel(PIN_LED_B) == ON);
    assert(pinLevel(PIN_LED_R) == OFF);
    assert(pinLevel(PIN_LED_G) == OFF);

    // El modo se recorta a 0..2.
    s.setFlag(9); assert(s.flag() == 2);

    // EL TONO NO CAMBIA CON EL RITMO. Se recorre un periodo entero del
    // parpadeo: en los instantes encendidos tiene que seguir siendo AZUL.
    s.setFlag(1);
    int vecesEncendido = 0, vecesApagado = 0;
    for (uint32_t t = 0; t < FLAG_BLINK_MS * 4u; t += 10) {
      setMillis(t);
      s.tick();
      if (pinLevel(PIN_LED_B) == ON) {
        vecesEncendido++;
        assert(pinLevel(PIN_LED_R) == OFF);   // nunca rojo siendo azul
      } else {
        vecesApagado++;
        assert(pinLevel(PIN_LED_R) == OFF && pinLevel(PIN_LED_G) == OFF);
      }
    }
    assert(vecesEncendido > 0 && vecesApagado > 0);   // parpadea de verdad

    // El parpadeo RAPIDO cambia mas veces que el lento en la misma ventana.
    auto cambios = [&](uint8_t modo) {
      s.setFlag(modo);
      int n = 0, prev = -1;
      for (uint32_t t = 0; t < 2000; t += 5) {
        setMillis(t); s.tick();
        int v = pinLevel(PIN_LED_B);
        if (prev != -1 && v != prev) n++;
        prev = v;
      }
      return n;
    };
    int lento = cambios(1), rapido = cambios(2);
    assert(rapido > lento);

    // La prueba de color cruda manda sobre todo, para poder averiguar la
    // polaridad del modulo desde la pagina sin recompilar.
    setMillis(0);
    s.setFlag(0);
    s.testColor(true, true, true, 500);
    s.tick();
    assert(pinLevel(PIN_LED_R) == ON && pinLevel(PIN_LED_G) == ON
           && pinLevel(PIN_LED_B) == ON);
    setMillis(600);                            // caducada
    s.tick();
    assert(pinLevel(PIN_LED_G) == OFF);        // vuelve al color del equipo
    ok("LED RGB: tono = equipo, ritmo = bandera, polaridad y prueba de color");
  }

  // --- Boton de equipo (A4): cuenta PULSACIONES limpias, nada mas --------
  //
  // Lo que importa: un rebote no es una pulsacion, mantenerlo no son muchas,
  // y un boton que ya estaba pulsado al arrancar no cambia el equipo.
  {
    setMillis(0);
    TeamButton b;
    b.begin();
    assert(g_pinMode[PIN_TEAM_BTN] == INPUT_PULLUP);
    assert(!b.pressed() && b.presses() == 0);          // suelto (pull-up)
    for (uint32_t t = 0; t <= 200; t += 10) { setMillis(t); b.tick(); }
    assert(b.presses() == 0);
    ok("boton: sin pulsar (o sin boton) no cuenta nada");

    // Rebotes al cerrar: el nivel salta cada 5 ms durante 20 ms.
    uint32_t t = 1000;
    for (int i = 0; i < 4; i++, t += 5) {
      setPin(PIN_TEAM_BTN, i % 2 ? HIGH : LOW); setMillis(t); b.tick();
    }
    assert(b.presses() == 0);                  // aun rebotando: nada
    setPin(PIN_TEAM_BTN, LOW);
    for (; t < 1000 + 20 + TEAM_BTN_DEBOUNCE_MS + 10; t += 5) { setMillis(t); b.tick(); }
    assert(b.pressed() && b.presses() == 1);   // UNA pulsacion
    // Mantenido 2 s: sigue siendo una.
    for (uint32_t e = t + 2000; t < e; t += 10) { setMillis(t); b.tick(); }
    assert(b.presses() == 1);
    ok("boton: rebotes = una pulsacion, mantenerlo no cuenta mas");

    // Un pulso de 10 ms (menor que el antirrebote) es ruido, no pulsacion.
    setPin(PIN_TEAM_BTN, HIGH);
    for (uint32_t e = t + 100; t < e; t += 5) { setMillis(t); b.tick(); }
    assert(!b.pressed());
    // Pico de 15 ms leido en TRES pasadas seguidas (cada 5 ms): el nivel si
    // se repite entre lecturas, pero no aguanta TEAM_BTN_DEBOUNCE_MS.
    setPin(PIN_TEAM_BTN, LOW);
    for (uint32_t e = t + 15; t < e; t += 5) { setMillis(t); b.tick(); }
    setPin(PIN_TEAM_BTN, HIGH); setMillis(t); b.tick();
    for (uint32_t e = t + 100; t < e; t += 5) { setMillis(t); b.tick(); }
    assert(b.presses() == 1);
    // Y una segunda pulsacion real, si cuenta.
    setPin(PIN_TEAM_BTN, LOW);
    for (uint32_t e = t + 100; t < e; t += 5) { setMillis(t); b.tick(); }
    assert(b.presses() == 2);
    ok("boton: un pico de ruido no cuenta; la segunda pulsacion si");

    // Atascado/pulsado AL ARRANCAR: no cuenta hasta soltar y volver a pulsar.
    TeamButton c;
    setMillis(0);
    setPin(PIN_TEAM_BTN, LOW);
    c.begin();                                 // arranca ya pulsado
    for (uint32_t u = 0; u < 3000; u += 10) { setMillis(u); c.tick(); }
    assert(c.pressed() && c.presses() == 0);
    setPin(PIN_TEAM_BTN, HIGH);
    for (uint32_t u = 3000; u < 3100; u += 10) { setMillis(u); c.tick(); }
    setPin(PIN_TEAM_BTN, LOW);
    for (uint32_t u = 3100; u < 3200; u += 10) { setMillis(u); c.tick(); }
    assert(c.presses() == 1);
    ok("boton: pulsado al arrancar NO cuenta (hay que soltar y pulsar)");
  }

  std::printf("\n%s\n", fallos ? "HAY FALLOS" : "todas OK");
  return fallos;
}
