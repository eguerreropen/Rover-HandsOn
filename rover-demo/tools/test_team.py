#!/usr/bin/env python3
"""
Pruebas del BOTON DE EQUIPO (A4) SIN robot:  python3 tools/test_team.py

El boton alterna rojo <-> azul fuera de ronda. Lo que tiene que cumplirse:

  - el cambio llega a TODO lo que depende del equipo: bandera a perseguir
    (camara), cinta de casa (FSM) y tono del LED (MCU);
  - en plena ronda se ignora, y lo pulsado en ronda NO se aplica despues;
  - lo pulsado antes de arrancar la App, o un reinicio del MCU, no cambia nada.

(El antirrebote y el conteo en el MCU se prueban en tools/test_actuators.cpp.)
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_part3 as t3                                  # noqa: E402  (stubs)
from test_part3 import CV, BLUE, RED, cv2, np            # noqa: E402
import test_part1 as t1                                  # noqa: E402
from test_part1 import sense, ok, PASSED, FAILED         # noqa: E402

import config                                            # noqa: E402
from mission import Mission, S                           # noqa: E402
from protocol import PressCounter, _parse_sense          # noqa: E402

EQUIPO_INICIAL = config.TEAM


def _restaura():
    config.set_team(EQUIPO_INICIAL)


# ============================================================ TELEMETRIA
def test_parser_boton():
    base = "0,0,0,900,900,900,900,1,1,1,50,{flags},10,5,2,0,50,0,90"
    s = _parse_sense(base.format(flags=0x8000) + ",7")
    assert s.btn_presses == 7 and s.btn_pressed
    s = _parse_sense(base.format(flags=0) + ",0")
    assert s.btn_presses == 0 and not s.btn_pressed
    viejo = _parse_sense(base.format(flags=0))
    assert viejo.btn_presses == -1, "sketch sin boton: -1, no 0"
    assert "btn_presses" in viejo.as_dict() and "btn_pressed" in viejo.as_dict()
    ok("parser: campo 20 = pulsaciones, bit 0x8000 = pulsado; sketch viejo = -1")


def test_contador_de_pulsaciones():
    c = PressCounter()
    assert c.new_presses(5) == 0, "la PRIMERA lectura solo fija la referencia"
    assert c.new_presses(5) == 0, "el mismo valor leido otra vez no es pulsacion"
    assert c.new_presses(6) == 1
    assert c.new_presses(9) == 3, "tres entre dos lecturas: se cuentan las tres"
    assert c.new_presses(0) == 0, "baja = el MCU se reinicio: referencia nueva"
    assert c.new_presses(1) == 1
    assert c.new_presses(-1) == 0 and c.new_presses(None) == 0
    ok("contador: referencia al arrancar, nuevas pulsaciones, reinicio del MCU")


# ================================================================ CONFIG
def test_set_team_recalcula_todo():
    try:
        config.set_team("red")
        assert (config.TEAM, config.ENEMY, config.TEAM_LED,
                config.TEAM_COLOR) == ("red", "blue", 0, config.C_RED)
        config.set_team("blue")
        assert (config.TEAM, config.ENEMY, config.TEAM_LED,
                config.TEAM_COLOR) == ("blue", "red", 1, config.C_BLUE)
        try:
            config.set_team("verde")
            assert False, "un equipo desconocido tiene que fallar"
        except ValueError:
            pass
        assert config.TEAM == "blue", "un fallo no deja el equipo a medias"
    finally:
        _restaura()
    ok("config.set_team: ENEMY, TEAM_LED y TEAM_COLOR salen del mismo sitio")


def test_la_fsm_usa_la_casa_del_equipo_vigente():
    """La FSM tiene que leer config.TEAM_COLOR al usarlo. Si lo copiara al
    arrancar, tras pulsar el boton entregaria la bandera en la zona RIVAL."""
    try:
        for equipo, casa, rival in (("red", config.C_RED, config.C_BLUE),
                                    ("blue", config.C_BLUE, config.C_RED)):
            config.set_team(equipo)
            m = Mission()
            m.key_deposited = True
            m.force_state(S.CARRY, sense())
            m.step(sense(color=rival, color_stable=rival), config.CONTROL_DT)
            assert m.state == S.CARRY, f"{equipo}: la cinta rival NO es casa"
            m.step(sense(color=casa, color_stable=casa), config.CONTROL_DT)
            assert m.state == S.DELIVER, f"{equipo}: su cinta SI es casa"
            assert m.as_dict()["team_color"] == config.COLOR_NAMES[casa]
    finally:
        _restaura()
    ok("FSM: tras cambiar de equipo, casa es la cinta nueva (y la otra no)")


def test_solo_se_cambia_fuera_de_ronda():
    m = Mission()
    for st in (S.IDLE, S.DONE, S.ESTOP):
        m.state = st
        assert m.can_change_team, st
    for st in (S.PATROL, S.EDGE, S.DEPOSIT_KEY, S.BACKOFF_KEY, S.APPROACH,
               S.GRAB, S.HOLDING, S.CARRY, S.DELIVER):
        m.state = st
        assert not m.can_change_team, st
    ok("solo se cambia de equipo en IDLE, DONE o ESTOP")


def test_el_sketch_lee_el_boton():
    """Que el boton compile no basta: sin teamBtn.tick() en loop() nunca
    cuenta, y sin teamBtn.begin() el pin queda sin pull-up y flota."""
    import re
    sk = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "sketch")
    ino = open(os.path.join(sk, "sketch.ino"), encoding="utf-8", errors="replace").read()
    cfg = open(os.path.join(sk, "config.h"), encoding="utf-8", errors="replace").read()
    sin = re.sub(r"/\*.*?\*/", "", ino, flags=re.S)
    sin = "\n".join(l.split("//")[0] for l in sin.splitlines())
    loop, setup = sin[sin.index("void loop()"):], sin[sin.index("void setup()"):sin.index("void loop()")]
    assert re.search(r"\bteamBtn\.tick\(\)\s*;", loop), "sin tick() en loop() no cuenta"
    assert re.search(r"\bteamBtn\.begin\(\)\s*;", setup), "sin begin() no hay pull-up"
    assert re.search(r"teamBtn\.presses\(\)", sin), "el contador tiene que ir en sense()"
    assert re.search(r"#define\s+PIN_TEAM_BTN\s+A4\b", cfg)
    ok("sketch: el boton se inicializa, se lee en loop() y viaja en sense()")


# ================================================================ CAMARA
def _frame(bgr):
    f = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    cv2.rectangle(f, (300, 200), (340, 320), bgr, -1)
    return f


def test_la_camara_cambia_de_bandera():
    if not CV:
        print("       (sin cv2 en este PC: se omite)")
        return
    from vision import Vision
    v = Vision("red")
    assert v.process(_frame(RED)) is not None
    assert v.process(_frame(BLUE)) is None, "buscando roja, la azul no cuenta"
    assert v.latest_flag() is None or v.latest_flag().t

    v.process(_frame(RED))
    assert v.latest_flag() is not None
    v.set_enemy("blue")
    assert v.latest_flag() is None, "la deteccion del color viejo se borra"
    assert v.process(_frame(BLUE)) is not None
    assert v.process(_frame(RED)) is None

    # Carrera: el cambio llega MIENTRAS se procesa un frame del color viejo.
    orig = v.detect

    def detect_y_cambian(frame):
        r = orig(frame)
        v.set_enemy("red")          # pulsan justo ahora
        return r
    v.detect = detect_y_cambian
    v.process(_frame(BLUE))         # detecta azul con los rangos viejos...
    v.detect = orig
    assert v.latest_flag() is None, "...pero no se publica: ya somos otro equipo"
    ok("camara: set_enemy cambia la bandera buscada y no deja pasar la vieja")


# ================================================================== MAIN
BTN = [0]


def _instala_boton_en_el_bridge():
    br = t1._utils.Bridge
    orig = br.call

    def call(name, *a):
        r = orig(name, *a)
        if name == "sense":         # SENSE del stub trae 18 campos: + key, + btn
            return f"{r},90,{BTN[0]}"
        return r
    br.call = call


def _ciclo(m):
    m.link.poll()
    m.control_loop()


def _ultimo_team(log):
    t = [a for n, a in log if n == "team"]
    return t[-1][0] if t else None


def test_main_boton_alterna_equipo():
    _instala_boton_en_el_bridge()
    import main
    m = main
    m.config.CONTROL_DT = 0.0
    m.config.DIAG_MCU_S = 0
    log = t1._utils.Bridge.log
    try:
        inicial = config.TEAM
        otro = "red" if inicial == "blue" else "blue"

        BTN[0] = 3                    # pulsado antes de que arrancara la App
        _ciclo(m)
        assert config.TEAM == inicial, "lo pulsado ANTES de arrancar no cuenta"

        BTN[0] = 4                    # una pulsacion
        _ciclo(m)
        assert config.TEAM == otro
        assert m.vision.enemy == config.ENEMY, "la camara busca la bandera nueva"
        assert _ultimo_team(log) == config.TEAM_LED, "el LED cambia de tono"
        assert m.snapshot()["team"]["team"] == otro

        BTN[0] = 5                    # otra: vuelve
        _ciclo(m)
        assert config.TEAM == inicial and m.vision.enemy == config.ENEMY

        # En ronda: se ignora. Y lo pulsado en ronda NO se aplica despues.
        time.sleep(0.05)
        assert m.ui._start()["ok"]
        assert m.mission.state == S.PATROL
        BTN[0] = 6
        _ciclo(m)
        assert config.TEAM == inicial, "en ronda el boton no hace nada"
        m.ui._stop()
        assert m.mission.state == S.IDLE
        _ciclo(m)
        assert config.TEAM == inicial, "la pulsacion de la ronda no se guarda"

        # Dos pulsaciones entre dos lecturas: mismo equipo (paridad).
        BTN[0] = 8
        _ciclo(m)
        assert config.TEAM == inicial

        # Reinicio del MCU: el contador vuelve a 0. Ni cambia el equipo...
        BTN[0] = 0
        _ciclo(m)
        assert config.TEAM == inicial
        # ...y el LED se reenvia (el MCU perdio el tono).
        log.clear()
        m.link.mcu_reboots += 1
        m.control_loop()
        assert _ultimo_team(log) == config.TEAM_LED, "tras reinicio se reenvia el tono"

        # ESTOP no es ronda: se puede cambiar.
        m.ui._estop()
        BTN[0] = 1
        _ciclo(m)
        assert config.TEAM == otro
        m.ui._clear()
    finally:
        _restaura()
        m.vision.set_enemy(config.ENEMY)
        m.ui._estop()
        m.vision.stop()
    ok("main.py: A4 alterna el equipo en IDLE/ESTOP, lo ignora en ronda, "
       "LED y camara siguen al cambio")


if __name__ == "__main__":
    print("ROVER H07 - boton de equipo (A4) - pruebas\n")
    for name, fn in sorted(globals().items(),
                           key=lambda kv: (kv[0].startswith("test_main"), kv[0])):
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
