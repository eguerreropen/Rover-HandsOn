#!/usr/bin/env python3
"""
Pruebas de la parte 1 SIN robot ni App Lab:  python3 tools/test_part1.py

Cubre el clasificador de color, el filtro de estabilidad, el parser de la
telemetria, la FSM (patrulla/borde/estop/watchdog) y un arranque completo de
main.py con el Bridge simulado (caza errores de cableado entre modulos).
"""
import os
import sys
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "python"))

# --- stub del runtime de App Lab -------------------------------------------
SENSE = "0,0,0,3900,3910,3890,3920,60,55,40,180,12416,{ms},{drives},3,0,55,0"


class StubBridge:
    def __init__(self):
        self.log, self.drives, self.t0 = [], 0, time.monotonic()

    def notify(self, name, *a):
        self.log.append((name, a))
        if name == "drive":
            self.drives += 1

    def call(self, name, *a):
        self.log.append((name, a))
        if name == "sense":
            ms = int((time.monotonic() - self.t0) * 1000) + 5000
            return SENSE.format(ms=ms, drives=self.drives)
        return {"ping": 1, "alive": 7}.get(name, True)


_ard = types.ModuleType("arduino")
_utils = types.ModuleType("arduino.app_utils")
_bricks = types.ModuleType("arduino.app_bricks")
_webui = types.ModuleType("arduino.app_bricks.web_ui")


class WebUI:
    def expose_api(self, *a, **k): pass
    def send_message(self, *a, **k): pass


_webui.WebUI = WebUI
_utils.Bridge = StubBridge()
_utils.App = type("App", (), {"run": staticmethod(lambda *a, **k: None)})
_ard.app_utils, _ard.app_bricks, _bricks.web_ui = _utils, _bricks, _webui
sys.modules.update({"arduino": _ard, "arduino.app_utils": _utils,
                    "arduino.app_bricks": _bricks, "arduino.app_bricks.web_ui": _webui})

import config                                       # noqa: E402
from colors import classify, Stable, ColorTracker   # noqa: E402
from mission import Mission, S, escape_from_edge    # noqa: E402
from protocol import Sense, _parse_sense            # noqa: E402

PASSED, FAILED = [], []


def ok(msg):
    PASSED.append(msg)
    print(f"  ok   {msg}")


def sense(**kw):
    s = Sense()
    s.t = time.monotonic()
    s.raw_fl = s.raw_fr = s.raw_rl = s.raw_rr = 3900
    s.flags = 0x1000 | 0x2000 | 0x080       # 4 TCRT montados, APDS ok
    for k, v in kw.items():
        setattr(s, k, v)
    return s


# ============================================================== COLOR
def test_classify():
    # Los casos se construyen RELATIVOS a los umbrales de config.py: asi la
    # prueba sigue comprobando el comportamiento aunque se recalibre, en vez
    # de romperse cada vez que se mide de nuevo.
    oscuro = config.COLOR_DARK_CLEAR - 5
    claro = config.COLOR_BRIGHT_CLEAR + 30
    medio = (config.COLOR_DARK_CLEAR + config.COLOR_BRIGHT_CLEAR) // 2
    assert classify(50, 50, 50, oscuro) == config.C_BLACK, "clear bajo = negro"
    assert classify(80, 80, 80, claro) == config.C_WHITE, "equilibrado y claro = blanco"
    assert classify(30, 30, 30, medio) == config.C_UNKNOWN, "equilibrado y oscuro = ?"
    assert classify(120, 100, 25, 120) == config.C_YELLOW, "rojo+verde altos, azul hundido"
    assert classify(150, 50, 40, 110) == config.C_RED
    assert classify(40, 60, 150, 110) == config.C_BLUE
    assert classify(0, 0, 0, 100) == config.C_UNKNOWN, "suma cero no revienta"
    ok("clasificador: negro, blanco, amarillo, rojo, azul, desconocido")


def test_umbral_de_negro_igual_en_los_dos_lados():
    """
    LINE_BLACK_BELOW (config.py, lo que el MPU manda con cfg_lines) y
    LINE_BLACK_BELOW_DEFAULT (config.h, lo que rige antes de que la App
    hable) tienen que ser el mismo numero. Si divergen, el rover frena en un
    sitio distinto durante los primeros segundos que despues.
    """
    import re
    ruta = os.path.join(ROOT, "sketch", "config.h")
    texto = open(ruta, encoding="utf-8", errors="replace").read()
    m = re.search(r"#define\s+LINE_BLACK_BELOW_DEFAULT\s+(\d+)", texto)
    assert m, "no se encontro LINE_BLACK_BELOW_DEFAULT en sketch/config.h"
    assert int(m.group(1)) == config.LINE_BLACK_BELOW, (
        f"config.h dice {m.group(1)} y config.py dice "
        f"{config.LINE_BLACK_BELOW}")
    ok(f"umbral de negro coherente entre sketch y App ({config.LINE_BLACK_BELOW})")


def test_umbrales_coherentes():
    """Los dos umbrales de clear tienen que dejar una banda intermedia real.
    Si alguien los cruza al recalibrar, 'blanco' y 'negro' se solapan y la
    clasificacion deja de significar nada."""
    assert 0 < config.COLOR_DARK_CLEAR < config.COLOR_BRIGHT_CLEAR

    # El minimo de 5 que habia aqui era MIO, no medido, y en pista resulto
    # estorbar: COLOR_DARK_CLEAR acabo en 2 (22/09) porque con mas alto el
    # AZUL -la cinta que menos luz devuelve- se clasificaba como negro y el
    # rover no reconocia su propia zona.
    #
    # Con 2 el "negro por color" casi no dispara, y eso esta BIEN: del borde
    # negro se encargan los TCRT. El APDS solo tiene que separar amarillo,
    # rojo, azul y blanco, y cualquier umbral de oscuridad que se coma una de
    # esas cintas hace mas dano que bien.
    assert config.COLOR_DARK_CLEAR >= 1, "0 dejaria la clase negro inalcanzable"
    assert 0 < config.COLOR_NEUTRAL_SPREAD < 1
    assert 1/3 < config.COLOR_DOMINANT < 1, "por debajo de 1/3 no es dominante"
    ok("umbrales de color coherentes entre si")


def test_yellow_before_red():
    """El amarillo tiene el rojo alto: sin la regla previa se leeria rojo."""
    r, g, b, c = 130, 110, 20, 130
    assert classify(r, g, b, c) == config.C_YELLOW
    assert r / (r + g + b) > 0.42, "el caso de prueba debe tener rojo dominante"
    ok("amarillo se comprueba antes que rojo")


def test_stable_filter():
    st = Stable(n=3)
    assert st.update(config.C_RED) == (config.C_UNKNOWN, False)
    assert st.update(config.C_RED) == (config.C_UNKNOWN, False)
    assert st.update(config.C_RED) == (config.C_RED, True), "3 seguidas -> cambio"
    assert st.update(config.C_BLUE) == (config.C_RED, False), "1 lectura suelta no cambia"
    assert st.update(config.C_RED) == (config.C_RED, False)
    ok("filtro de estabilidad: 3 lecturas seguidas, ruido ignorado")


def test_latencia_de_deteccion_vs_cinta():
    """
    El filtro de estabilidad cuesta CENTIMETROS, no solo parpadeo.

    Exigir N lecturas seguidas retrasa la deteccion N * periodo_del_APDS. Si
    ese retraso supera lo que tarda el sensor en cruzar la cinta, el rover
    pasa por encima de una linea de color sin declararla NUNCA -- y el fallo
    solo aparece en movimiento, no probando a mano.

    Esta prueba fija el margen para que nadie suba COLOR_STABLE_N sin ver el
    coste.
    """
    APDS_MS = 25.0                      # = APDS_POLL_MS del sketch
    ANCHO_CINTA_CM = 2.0                # cinta tipica
    VEL_CM_S = 25.0                     # velocidad de patrulla estimada

    retardo_s = config.COLOR_STABLE_N * APDS_MS / 1000.0
    recorrido_cm = retardo_s * VEL_CM_S
    assert recorrido_cm < ANCHO_CINTA_CM, (
        f"con COLOR_STABLE_N={config.COLOR_STABLE_N} el rover recorre "
        f"{recorrido_cm:.1f} cm antes de declarar el color, y la cinta mide "
        f"{ANCHO_CINTA_CM} cm: se la pasaria de largo")
    ok(f"filtro de color vs anchura de cinta: {recorrido_cm:.1f} cm de "
       f"{ANCHO_CINTA_CM} cm disponibles")


def test_tracker_events():
    tr = ColorTracker()
    s = sense(r=150, g=50, b=40, c=110)
    for i in range(3):
        tr.apply_to(s, i * 0.05)
    assert s.color_stable == config.C_RED
    assert tr.events and tr.events[-1][1] == "rojo"
    s2 = sense(flags=0x1000 | 0x2000)          # APDS ausente
    tr.apply_to(s2, 1.0)
    assert s2.color_stable == config.C_UNKNOWN, "sin APDS no se inventa color"
    ok("tracker: registra el cambio y no clasifica sin sensor")


# ============================================================ PROTOCOLO
def test_parser():
    # flags 15489 = 0x3C81: watchdog | APDS ok | encl. frente+atras | TCRT presentes
    s = _parse_sense("450,-320,9,120,3000,3100,150,70,60,40,180,15489,99123,7412,8,4,410,7")
    assert (s.left, s.right, s.black_mask) == (450, -320, 9)
    assert [s.raw_fl, s.raw_fr, s.raw_rl, s.raw_rr] == [120, 3000, 3100, 150]
    assert (s.r, s.g, s.b, s.c) == (70, 60, 40, 180)
    assert s.watchdog and s.front_sensors_ok and s.rear_sensors_ok
    assert s.front_latched and s.rear_latched and s.floor_sensor_ok
    assert s.drives == 7412 and s.loop_max_ms == 8 and s.mcu_ms == 99123
    assert s.slow_passes == 4 and s.drive_gap_ms == 410
    assert s.future_marks == 7
    assert s.front_black and s.rear_black
    assert _parse_sense("1,2,3") is None and _parse_sense(None) is None
    # Trama de 15 campos: sketch anterior todavia en la placa. Debe parsearse
    # y dejar el contador nuevo a cero, no tumbar la App.
    viejo = _parse_sense("0,0,0,900,900,900,900,1,1,1,50,0,10,5,2")
    assert viejo is not None and viejo.slow_passes == 0
    ok("parser de sense(): 16 campos, bits de estado y compatibilidad")


def test_hueco_entre_drive():
    """El hueco mas largo entre drive() es el discriminador principal cuando
    salta el watchdog: dice si el que se callo fue el MPU."""
    from protocol import RoverLink
    lk = RoverLink()
    lk.drive(0, 0)
    assert lk.max_drive_gap_ms == 0.0, "la primera orden no tiene hueco previo"
    lk._last_drive_t -= 0.5                     # simula 500 ms sin mandar
    lk.drive(0, 0)
    assert lk.max_drive_gap_ms >= 500, lk.max_drive_gap_ms
    lk.drive(0, 0)                              # una rapida no baja el maximo
    assert lk.max_drive_gap_ms >= 500
    ok("hueco maximo entre drive(): detecta que el MPU se callo")


def test_resta_de_marcas_con_signo():
    """
    El fallo que costo cuatro hipotesis: restar marcas de millis() sin signo.

    Si la marca es MAS NUEVA que el 'now' con el que se compara -porque un
    handler del Bridge la escribio despues-, la resta sin signo se desborda
    por abajo y da ~4.29e9, que supera cualquier timeout. El watchdog salta
    con las ordenes llegando perfectamente.

    Esta prueba fija la aritmetica correcta (la misma que hace elapsedSince()
    en el sketch) para que nadie la "simplifique" mas adelante.
    """
    M = 1 << 32

    def sin_signo(mark, now):
        return (now - mark) % M

    def con_signo(mark, now):                 # equivalente a elapsedSince()
        d = (now - mark) % M
        if d >= (1 << 31):
            return 0
        return d

    now, marca_futura = 1000, 1003
    assert sin_signo(marca_futura, now) > 400, "asi es como saltaba el watchdog"
    assert con_signo(marca_futura, now) == 0, "una marca del futuro es 'ahora'"
    # el caso normal no cambia
    assert con_signo(600, 1000) == 400
    # y sigue siendo correcto cuando millis() da la vuelta a los 49 dias
    assert con_signo(M - 100, 50) == 150
    ok("resta de marcas de tiempo: con signo, no sin signo")


def test_rafagas_detectadas():
    """El caso que ningun contador anterior distinguia: el MPU manda a ritmo,
    el MCU recibe todo, ningun lazo se atasca, pero las ordenes llegan
    agrupadas y entre rafaga y rafaga el watchdog salta."""
    from protocol import RoverLink
    lk = RoverLink()
    # 17 campos: el MCU reporta que entre dos drive() pasaron 410 ms
    lk._track(_parse_sense("0,0,0,900,900,900,900,1,1,1,50,0,1000,500,12,0,410,0"))
    assert lk.mcu_max_drive_gap_ms == 410
    # y el maximo no baja si luego llega una ventana buena
    lk._track(_parse_sense("0,0,0,900,900,900,900,1,1,1,50,0,2000,520,11,0,50,0"))
    assert lk.mcu_max_drive_gap_ms == 410, "el maximo historico no debe bajar"
    ok("rafagas: hueco de recepcion medido en el MCU")


def test_effective_mask():
    s = sense(black_mask=0, flags=0x400 | 0x1000 | 0x2000)
    assert s.effective_black_mask == 0b0011, "enclavado delante sin lectura -> frente"
    s = sense(black_mask=0b0100, flags=0x400)
    assert s.effective_black_mask == 0b0100, "la lectura instantanea manda si existe"
    ok("mascara efectiva desde los enclavamientos")


# ================================================================ FSM
def test_escape():
    assert escape_from_edge(0b0001) == (-1, 1), "FL -> retroceder, girar a la derecha"
    assert escape_from_edge(0b0010) == (-1, -1), "FR -> retroceder, girar a la izquierda"
    assert escape_from_edge(0b1100) == (1, 1), "atras -> avanzar"
    assert escape_from_edge(0b0101) == (0, 1), "cruzado -> solo pivotar"
    ok("maniobra de escape por esquinas")


def test_patrol_edge_cycle():
    m = Mission()
    assert m.start_patrol(sense())
    assert m.state == S.PATROL
    a = m.step(sense(), 0.05)
    assert a.throttle == config.PATROL_SPEED and a.turn == 0

    a = m.step(sense(black_mask=0b0001), 0.05)      # FL pisa negro
    assert m.state == S.EDGE
    assert a.throttle < 0, "primero retroceder"
    m._edge_until = time.monotonic() - 0.01         # acabo el retroceso
    a = m.step(sense(), 0.05)
    assert a.turn > 0, "girar hacia la derecha (borde a la izquierda)"
    m._edge_until = time.monotonic() - 0.01         # acabo el giro
    m.step(sense(), 0.05)
    assert m.state == S.PATROL, "borde superado -> vuelve a patrullar"
    ok("ciclo patrulla -> borde -> escape -> patrulla")


def test_patrol_refuses_without_front_sensors():
    m = Mission()
    assert not m.start_patrol(sense(flags=0x2000 | 0x080))
    assert m.state == S.IDLE
    ok("la patrulla se niega sin TCRT frontales")


def test_watchdog_and_estop():
    m = Mission()
    m.start_patrol(sense())
    m.step(sense(flags=0x001 | 0x1000 | 0x2000), 0.05)
    assert m.state == S.IDLE, "watchdog -> IDLE"
    m.estop("prueba")
    assert not m.start_patrol(sense()), "en ESTOP no arranca"
    m.request(S.PATROL, "no deberia colar")
    assert m.state == S.ESTOP, "ESTOP pegajoso"
    m.clear_estop()
    assert m.state == S.IDLE
    ok("watchdog -> IDLE; ESTOP pegajoso hasta rearme")


# ============================================================ ARRANQUE
def test_main_startup():
    """Importar main EJECUTA startup(): caza errores de cableado entre modulos."""
    import json
    import main
    m = main
    m.config.CONTROL_DT = 0.0
    m.config.DIAG_MCU_S = 0
    time.sleep(0.15)                       # que el hilo de sense() traiga datos
    for _ in range(5):
        m.control_loop()
    assert m.link.drive_calls >= 5
    json.dumps(m.snapshot())
    assert any(n == "cfg_lines" for n, _ in _utils.Bridge.log), \
        "el umbral de negro debe mandarse al MCU al arrancar"
    cfg = [a for n, a in _utils.Bridge.log if n == "cfg_lines"][0]
    assert cfg == (config.LINE_BLACK_BELOW, config.LINE_HYSTERESIS)
    # patrulla desde la "pagina"
    m.ui._start()
    assert m.mission.state == S.PATROL
    for _ in range(3):
        m.control_loop()
    assert m.link.last_command[0] > 0, "en patrulla se manda avance"
    m.ui._estop()
    assert m.mission.state == S.ESTOP
    ok("arranque completo de main.py con Bridge simulado")


if __name__ == "__main__":
    print("ROVER H07 - demo parte 1 - pruebas\n")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
            except AssertionError as e:
                FAILED.append(name)
                print(f"  FALLO  {name}: {e}")
            except Exception as e:                 # noqa: BLE001
                FAILED.append(name)
                print(f"  ERROR  {name}: {type(e).__name__}: {e}")
                import traceback
                traceback.print_exc()
    print(f"\n{len(PASSED)} OK, {len(FAILED)} fallidas")
    sys.exit(1 if FAILED else 0)
