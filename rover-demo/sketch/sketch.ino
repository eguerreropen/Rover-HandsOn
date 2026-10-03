/*
 * ROVER H07 - DEMO PARTE 1 - Sketch del MCU (STM32U585, core arduino:zephyr)
 * ===========================================================================
 *
 * Esta parte hace SOLO dos cosas: leer los sensores de linea y de color, y
 * no dejar que el rover se salga de la pista. La logica (estados, decidir
 * hacia donde escapar, clasificar el color) esta en la App Python del MPU.
 *
 * Lo que vive aqui, porque el plazo importa:
 *   - Reflejo de borde negro con ENCLAVAMIENTO. A 100 Hz corta en <10 ms;
 *     por el Bridge serian 50-150 ms, que a 30 cm/s son 1,5-4,5 cm de mas.
 *   - Watchdog: si el MPU calla 400 ms, se para.
 *
 * CONTRATO CON EL MPU (debe coincidir con python/protocol.py):
 *   drive(l, r)  -> bool   velocidades [-1000,1000]; se manda por notify
 *   stop()       -> bool
 *   cfg_lines(black_below, hysteresis) -> bool   umbral del TCRT desde config.py
 *   spin(id,deg,ms)-> bool   servo CONTINUO de la caja: gira y se para solo.
 *   servo(id, deg) -> bool   PARTE 2: id 2 = llave; deg 0..180. Bloquea los
 *                            motores MOVE_LOCK_MS. No alimenta el watchdog.
 *                            Si el servo estaba apagado, lo re-engancha.
 *   servo_off(id)  -> bool   apaga el servo (detach): sin pulsos, sin
 *                            corriente de bloqueo. Esperar SERVO_SETTLE_S
 *                            desde la ultima orden antes de llamarlo.
 *   flagled(mode)  -> bool   LED RGB: ritmo (0 fijo, 1 lento, 2 rapido).
 *   team(t)        -> bool   LED RGB: tono (0 rojo, 1 azul).
 *   ledtest(r,g,b,ms) -> bool  enciende un color crudo (comun anodo/catodo).
 *                            0 apagado, 1 parpadeo, 2 fijo.
 *   sense()      -> CSV de 20 campos:
 *        vl,vr,mask,FL,FR,RL,RR,r,g,b,c,flags,ms,drives,loopmax,slow,dgap,fut,key,btn
 *        key     = angulo actual del servo de la llave (0..180)
 *        btn     = pulsaciones del boton de equipo (A4) desde el arranque;
 *                  el bit 0x8000 de flags dice si esta pulsado AHORA
 *        loopmax = pasada mas lenta DESDE LA ULTIMA CONSULTA (se consume)
 *        slow    = cuantas pasadas han superado LOOP_SLOW_MS (acumulado)
 *        dgap    = hueco mas largo entre dos drive() RECIBIDOS (acumulado)
 *        fut     = veces que una marca de tiempo quedo "en el futuro"
 *                  respecto al now de loop(). Ver elapsedSince().
 *   ping()       -> int
 *   alive()      -> int   etapa de arranque alcanzada (0..7). Es el UNICO
 *                         RPC no-safe: lo atiende el hilo del Bridge, asi
 *                         que contesta aunque setup() o loop() esten
 *                         colgados. Ver rpc_alive().
 *
 * FLAGS: 0x001 watchdog  0x002 black_lock  0x020 servo llave encendido
 *        0x040 servo asentandose
 *        0x080 APDS ok  0x400 encl. frente  0x800 encl. atras
 *        0x1000 TCRT frontales presentes  0x2000 traseros
 *        0x4000 APDS desconectado por lento
 */

#include <Arduino_RouterBridge.h>
#include "config.h"
#include "timing.h"
#include "motors.h"
#include "lines.h"
#include "colorsense.h"
#include "actuators.h"
#include "signals.h"
#include "button.h"

static Motors     motors;
static Lines      lines;
static ColorSense colorSense;
static Actuators  act;
static Signals    signals;
static TeamButton teamBtn;            // A4: elegir equipo

static uint32_t g_lastCmdMs   = 0;
static bool     g_watchdogHit = false;
static bool     g_blackLock   = false;

static bool     g_frontLatch   = false;
static bool     g_rearLatch    = false;
static uint32_t g_frontClearMs = 0;
static uint32_t g_rearClearMs  = 0;

static bool     g_colorDisabled = false;
static uint8_t  g_colorSlow     = 0;
static uint32_t g_lastWdPrintMs = 0;

/*
 * INSTRUMENTACION. En la version anterior el watchdog saltaba de forma
 * intermitente y nunca se cerro la causa; estos contadores separan las
 * causas posibles sin adivinar:
 *   g_driveCount  drive() RECIBIDOS (contra los enviados por el MPU)
 *   g_loopMaxMs   pasada de loop() mas lenta desde la ultima consulta
 */
static uint32_t g_driveCount  = 0;
static uint16_t g_loopMaxMs   = 0;
static uint16_t g_loopMaxEver = 0;
static uint32_t g_lastDriveMs = 0;
static uint16_t g_slowPasses  = 0;   // pasadas que superaron LOOP_SLOW_MS
/*
 * HUECO MAS LARGO ENTRE DOS drive() RECIBIDOS.
 *
 * La medida simetrica de la que ya hace el MPU con los que ENVIA. Es la que
 * distingue el ultimo caso que quedaba abierto:
 *
 *   MPU manda cada 50 ms  y  MCU recibe cada 50 ms   -> todo normal
 *   MPU manda cada 50 ms  y  MCU recibe cada 400 ms  -> las ordenes viajan
 *       AGRUPADAS: el transporte las acumula y las suelta a rafagas. El
 *       contador total cuadra (no se pierde casi nada) y ningun lazo se
 *       atasca, pero entre rafaga y rafaga el MCU se queda sin ordenes y su
 *       watchdog salta. Sin esta medida ese caso es indistinguible de
 *       "no pasa nada", que es justo lo que parecia.
 */
static uint32_t g_maxDriveGap = 0;

// elapsedSince() y g_futureMarks: ver timing.h (compartidos con actuators.h).
static char     g_buf[160];   // 20 campos: peor caso ~130 caracteres

// ------------------------------------------------------------ RPC handlers
// provide_safe() => se ejecutan en el hilo de loop(), sin carreras.
static void touch() { g_lastCmdMs = millis(); if (g_watchdogHit) g_watchdogHit = false; }

bool rpc_drive(int left, int right) {
  touch();
  g_driveCount++;
  uint32_t now = millis();
  if (g_lastDriveMs) {
    uint32_t gap = elapsedSince(g_lastDriveMs, now);
    if (gap > g_maxDriveGap && gap < 60000) g_maxDriveGap = gap;
  }
  g_lastDriveMs = now;
  motors.set(left, right);
  return !g_blackLock;
}

bool rpc_stop() { touch(); motors.stop(); return true; }

bool rpc_cfg_lines(int blackBelow, int hysteresis) {
  // No hace touch(): configurar no es mantener la mision.
  lines.configure((uint16_t)constrain(blackBelow, 0, LINE_ADC_MAX),
                  (uint16_t)constrain(hysteresis, 0, 1000));
  Monitor.print("[MCU] umbral de negro = ");
  Monitor.print(lines.blackBelow());
  Monitor.print("  histeresis = ");
  Monitor.println(hysteresis);
  return true;
}

bool rpc_servo(int id, int deg) {
  // No hace touch(): mover un servo no es mantener la mision.
  return act.setRaw(id, deg);
}

bool rpc_spin(int id, int deg, int ms) {
  // Servo CONTINUO de la caja: gira 'ms' a 'deg' y se para solo (tick()).
  return act.spin(id, deg, ms);
}

bool rpc_servo_off(int id) {
  // Apaga el servo (detach). Tampoco alimenta el watchdog, ni bloquea los
  // motores: cortar pulsos no tira corriente.
  return act.off(id);
}

bool rpc_team(int t) {
  // El equipo lo decide config.TEAM en el MPU: aqui solo se obedece, para que
  // no haya dos sitios donde declarar de que equipo somos.
  signals.setTeam((uint8_t)(t ? 1 : 0));
  return true;
}

bool rpc_ledtest(int r, int g, int b, int ms) {
  signals.testColor(r != 0, g != 0, b != 0,
                    (uint32_t)constrain(ms, 0, 10000));
  return true;
}

bool rpc_flagled(int mode) {
  // LED de bandera: 0 apagado, 1 parpadeo, 2 fijo. No alimenta el watchdog.
  signals.setFlag((uint8_t)constrain(mode, 0, 2));
  return true;
}

int rpc_ping() { return (int)millis(); }

/*
 * SONDEO DE VIDA. Este RPC es el unico que se registra con provide() y NO con
 * provide_safe(), y esa diferencia es todo el truco:
 *
 *   provide_safe  -> lo despacha loop(), en el hilo del sketch
 *   provide       -> lo despacha el HILO PROPIO del Bridge
 *
 * O sea que alive() contesta aunque setup() se haya quedado colgado en un
 * begin() (un I2C atascado bloquea para siempre) o aunque loop() no llegue a
 * correr. Devuelve la ultima etapa de arranque completada:
 *
 *   0 = ni siquiera empezo        4 = APDS
 *   1 = Bridge                    5 = servo de la llave
 *   2 = motores                   6 = LED  (setup completo)
 *   3 = TCRT                      7 = loop() esta corriendo
 *
 * Con esto se localiza el fallo SIN depender del monitor serie, que en App
 * Lab sobre Windows tiene un bug conocido y puede no mostrar nada.
 *
 * Solo lee un entero: no toca motores, sensores ni nada compartido, asi que
 * es seguro atenderlo desde otro hilo.
 */
static volatile int g_bootStage = 0;

int rpc_alive() { return g_bootStage; }

const char* rpc_sense() {
  uint16_t flags = 0;
  if (g_watchdogHit)        flags |= 0x001;
  if (g_blackLock)          flags |= 0x002;
  // Un bit por servo: cual esta ALIMENTADO ahora mismo. Con el apagado
  // automatico, saber si un servo tiene pulsos deja de ser un detalle y pasa
  // a ser lo que explica que el brazo ceda o que la bandera se caiga.
  if (act.attached(Actuators::KEY))  flags |= 0x020;
  if (act.attached(Actuators::LIFT)) flags |= 0x100;
  if (act.attached(Actuators::GRIP)) flags |= 0x200;
  if (act.settling())       flags |= 0x040;
  if (colorSense.ok())      flags |= 0x080;
  if (g_frontLatch)         flags |= 0x400;   // enclavamientos, no lectura
  if (g_rearLatch)          flags |= 0x800;
  if (Lines::frontPresent()) flags |= 0x1000;
  if (Lines::rearPresent())  flags |= 0x2000;
  if (g_colorDisabled)      flags |= 0x4000;
  if (teamBtn.pressed())   flags |= 0x8000;   // boton de equipo PULSADO ahora

  uint16_t loopMax = g_loopMaxMs;
  g_loopMaxMs = 0;                         // maximo por VENTANA (se consume)
  snprintf(g_buf, sizeof(g_buf), "%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u,%u,%lu,%u,%u,%u",
           motors.left(), motors.right(),
           (unsigned)lines.blackMask(),
           (unsigned)lines.raw(Lines::FL), (unsigned)lines.raw(Lines::FR),
           (unsigned)lines.raw(Lines::RL), (unsigned)lines.raw(Lines::RR),
           (unsigned)colorSense.r(), (unsigned)colorSense.g(),
           (unsigned)colorSense.b(), (unsigned)colorSense.c(),
           (unsigned)flags,
           (unsigned long)millis(),
           (unsigned long)g_driveCount,
           (unsigned)loopMax,
           (unsigned)g_slowPasses,
           (unsigned long)g_maxDriveGap,
           (unsigned)g_futureMarks,
           (unsigned)act.angle(Actuators::KEY),
           (unsigned)teamBtn.presses());    // campo 20: pulsaciones del boton de equipo (A4)
  return g_buf;
}

// -------------------------------------------------------------------- setup
void setup() {
  Bridge.begin();
  Monitor.begin();

  /*
   * ARRANQUE POR ETAPAS, CON TRAZA. Si el MCU se queda callado, esta traza
   * dice en que begin() se atasco: el ultimo mensaje que se vea es el paso
   * que termino; el siguiente es el culpable. Si no aparece NI el 1/6, el
   * sketch no esta corriendo (mira la compilacion en App Lab) o el monitor
   * no llega (bug conocido del Serial Monitor de App Lab en Windows).
   */
  g_bootStage = 1;
  Monitor.println("[MCU] boot 1/7 bridge OK");

  /*
   * LOS RPC SE REGISTRAN ANTES QUE EL HARDWARE, Y ES DELIBERADO.
   *
   * Antes iban al final, despues de los cinco begin(). Con ese orden, un
   * begin() que se cuelgue -el I2C del APDS es el sospechoso clasico- deja
   * el sketch SIN NINGUN metodo registrado, y desde el MPU eso se ve
   * exactamente igual que "el sketch no esta cargado": todas las llamadas
   * responden "method not available". Imposible de distinguir.
   *
   * Registrar primero no tiene ningun riesgo: los handlers de provide_safe()
   * los despacha loop(), que no arranca hasta que setup() termina. Lo unico
   * que puede pasar es que una orden llegue antes de que su hardware este
   * listo, y para eso los objetos ya estan construidos con valores seguros
   * (motores parados, servo sin enganchar).
   */
  Bridge.provide("alive",          rpc_alive);      // NO-safe: ver rpc_alive
  Bridge.provide_safe("drive",     rpc_drive);
  Bridge.provide_safe("stop",      rpc_stop);
  Bridge.provide_safe("cfg_lines", rpc_cfg_lines);
  Bridge.provide_safe("servo",     rpc_servo);
  Bridge.provide_safe("servo_off", rpc_servo_off);
  Bridge.provide_safe("spin",      rpc_spin);
  Bridge.provide_safe("flagled",   rpc_flagled);
  Bridge.provide_safe("team",      rpc_team);
  Bridge.provide_safe("ledtest",   rpc_ledtest);
  Bridge.provide_safe("sense",     rpc_sense);
  Bridge.provide_safe("ping",      rpc_ping);
  Monitor.println("[MCU] RPC registrados");

  motors.begin();
  g_bootStage = 2;
  Monitor.println("[MCU] boot 2/7 motores");
  lines.begin();
  g_bootStage = 3;
  Monitor.println("[MCU] boot 3/7 TCRT");
  colorSense.begin();
  g_bootStage = 4;
  Monitor.println("[MCU] boot 4/7 APDS");
  act.begin();                          // servo de la llave a RETENCION
  g_bootStage = 5;
  Monitor.println("[MCU] boot 5/7 servo llave");
  signals.begin();
  teamBtn.begin();                     // A4, pull-up interno: no se cuelga
  g_bootStage = 6;
  Monitor.println("[MCU] boot 6/7 LED bandera");

  g_lastCmdMs = millis();

  Monitor.println("[MCU] Rover H07 demo parte 3 listo (servo llave D10, LED bandera D13).");
  if (!Lines::frontPresent())
    Monitor.println("[MCU] *** SIN TCRT FRONTALES: NO hay proteccion de borde por delante ***");
  if (!Lines::rearPresent())
    Monitor.println("[MCU] *** SIN TCRT TRASEROS: NO hay proteccion de borde por detras ***");
  Monitor.print("[MCU] APDS piso (Wire D20/D21): ");
  Monitor.println(colorSense.ok() ? "OK" : "NO DETECTADO - revisa cableado y 3V3");
}

// --------------------------------------------------------------------- loop
void loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();

  /*
   * La primera pasada NO se mide.
   *
   * Con lastTick a 0, la primera vuelta calculaba millis() - 0, es decir
   * TODO lo que tardo setup() (Bridge.begin, I2C, los Monitor.println que
   * viajan por el enlace): cientos de ms. Eso entraba como "la pasada mas
   * lenta del lazo" y, al guardarse como maximo historico, se quedaba fijo
   * para siempre y TAPABA cualquier bloqueo real posterior.
   *
   * Un instrumento que miente una vez y luego oculta lo que viene es peor
   * que no tener instrumento: manda a buscar la causa donde no esta.
   */
  if (lastTick == 0) { lastTick = now; g_bootStage = 7; return; }

  if (now - lastTick < CONTROL_TICK_MS) return;

  uint32_t sinceLast = now - lastTick;
  if (sinceLast < 60000) {
    if (sinceLast > g_loopMaxMs)   g_loopMaxMs   = (uint16_t)sinceLast;
    if (sinceLast > g_loopMaxEver) g_loopMaxEver = (uint16_t)sinceLast;
    // Cuantas pasadas se han ido de tiempo. Un maximo NO distingue "paso una
    // vez" de "pasa cada dos por tres", y esa diferencia es justo la que
    // decide si hay que buscar una causa o no.
    if (sinceLast > LOOP_SLOW_MS) g_slowPasses++;
  }
  lastTick = now;

  lines.tick();

  /* ---- 1. REFLEJO DE BORDE NEGRO, ENCLAVADO -----------------------------
   * El bloqueo NO sigue al sensor: a 60 cm/s el sensor cruza la cinta en
   * 33 ms y un bloqueo que la siguiera duraria 3 ciclos (bug real, cazado en
   * pista). Se suelta solo cuando (1) llevan EDGE_LATCH_CLEAR_MS sin ver
   * negro y (2) el MPU ya no empuja hacia el lado prohibido.
   */
  bool front = lines.frontBlack();
  bool rear  = lines.rearBlack();

  if (front) {
    g_frontClearMs = 0;
    if (!g_frontLatch) {
      g_frontLatch = true;
      motors.stop();
      Monitor.print("[MCU] BORDE NEGRO delante  mascara=");
      Monitor.println(lines.blackMask());
    }
  } else if (g_frontLatch) {
    if (g_frontClearMs == 0) g_frontClearMs = now;
    if (!motors.pushingForward() &&
        elapsedSince(g_frontClearMs, now) >= EDGE_LATCH_CLEAR_MS) {
      g_frontLatch = false;
      Monitor.println("[MCU] borde delantero liberado");
    }
  }

  if (rear) {
    g_rearClearMs = 0;
    if (!g_rearLatch) {
      g_rearLatch = true;
      motors.stop();
      Monitor.print("[MCU] BORDE NEGRO detras  mascara=");
      Monitor.println(lines.blackMask());
    }
  } else if (g_rearLatch) {
    if (g_rearClearMs == 0) g_rearClearMs = now;
    if (!motors.pushingReverse() &&
        elapsedSince(g_rearClearMs, now) >= EDGE_LATCH_CLEAR_MS) {
      g_rearLatch = false;
      Monitor.println("[MCU] borde trasero liberado");
    }
  }

  g_blackLock = g_frontLatch || g_rearLatch;
  motors.setBlocked(g_frontLatch, g_rearLatch);

  // ---- 2. Watchdog del enlace con el MPU --------------------------------
  uint32_t sinceCmd = elapsedSince(g_lastCmdMs, now);
  if (sinceCmd > CMD_TIMEOUT_MS) {
    if (!g_watchdogHit) {
      g_watchdogHit = true;
      if (now - g_lastWdPrintMs > 1000) {      // 1/s: no congestionar el enlace
        g_lastWdPrintMs = now;
        Monitor.print("[MCU] WATCHDOG sin ordenes: gap=");
        Monitor.print((unsigned long)sinceCmd);
        Monitor.print(" ms  ultimo_drive_hace=");
        Monitor.print(g_lastDriveMs ? (unsigned long)(now - g_lastDriveMs) : 0UL);
        Monitor.print(" ms  drives=");
        Monitor.print((unsigned long)g_driveCount);
        Monitor.print("  loop_mas_lento=");
        Monitor.print((unsigned)g_loopMaxEver);
        Monitor.println(" ms");
      }
    }
    motors.stop();
  }

  // ---- 3. Pico de corriente del servo: motores quietos mientras se mueve --
  if (act.motorsLocked()) motors.set(0, 0);

  // ---- 4. Rampa y PWM ---------------------------------------------------
  motors.tick();

  signals.tick();
  teamBtn.tick();      // antirrebote del boton de equipo (A4)
  act.tick();           // para el servo continuo de la caja cuando toca

  // ---- 5. APDS, AL FINAL y con presupuesto de tiempo --------------------
  // Un I2C colgado bloquea loop(), y loop() es quien corre el reflejo y
  // recibe las ordenes. Si tarda demasiado varias veces, se desconecta.
  if (!g_colorDisabled) {
    uint32_t t0 = millis();
    bool did = colorSense.tick();
    uint32_t dur = millis() - t0;
    if (did && dur > COLOR_MAX_MS) {
      if (++g_colorSlow >= COLOR_SLOW_LIMIT) {
        g_colorDisabled = true;
        Monitor.print("[MCU] APDS DESCONECTADO: sus lecturas bloquean el lazo (");
        Monitor.print(dur);
        Monitor.println(" ms).");
      }
    } else if (did && g_colorSlow) {
      g_colorSlow = 0;
    }
  }
}
