#!/usr/bin/env python3
"""
Pruebas de la parte 2 (llave) SIN robot:  python3 tools/test_part2.py

Reutiliza el stub de App Lab de test_part1 y cubre la secuencia completa de
depositar la llave, el rebote en amarillo despues, las interrupciones por
borde negro y el rearme para repetir pruebas.
"""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "python"))

import test_part1 as t1                       # noqa: E402  (instala los stubs)
from test_part1 import sense, ok, PASSED, FAILED   # noqa: E402

import config                                 # noqa: E402
from mission import Mission, S                # noqa: E402
from protocol import _parse_sense             # noqa: E402


def amarillo(**kw):
    return sense(color_stable=config.C_YELLOW, r=120, g=100, b=25, c=120, **kw)


def cmds(action):
    return dict(action.commands)


def rewind(m, seconds):
    """Hace 'pasar' el tiempo de la fase actual."""
    m._phase_until -= seconds
    m._edge_until -= seconds


# ============================================================ SECUENCIA
def test_secuencia_completa():
    m = Mission()
    assert m.start_patrol(sense())
    a = m.step(sense(), 0.05)
    assert a.throttle == config.PATROL_SPEED

    # pisa amarillo con la llave a bordo -> entra en la zona a velocidad lenta
    a = m.step(amarillo(), 0.05)
    assert m.state == S.DEPOSIT_KEY and m._key_phase == "enter"
    assert a.throttle == config.SPEED_CREEP and not a.commands
    assert not m.key_deposited, "aun no se ha soltado"

    # termina el avance -> parado y servo a RELEASE
    rewind(m, config.ZONE_ENTER_ADVANCE_S + 0.01)
    a = m.step(amarillo(), 0.05)
    assert a.throttle == 0 and cmds(a).get("key") == config.SERVO_KEY_RELEASE
    assert m.key_deposited and m.key_drops == 1, "depositada en cuanto gira el servo"
    assert m._key_phase == "drop"

    # mientras cae: quieto, sin ordenes
    a = m.step(amarillo(), 0.05)
    assert a.throttle == 0 and a.turn == 0 and not a.commands

    # cayo -> retroceder (la caja esta DELANTE)
    rewind(m, config.KEY_RELEASE_S + 0.01)
    a = m.step(amarillo(), 0.05)
    assert m.state == S.BACKOFF_KEY and a.throttle < 0 and a.turn == 0

    # retroceso hecho -> media vuelta, y ya lejos de la caja se rearma el
    # servo (tiene todo el giro para llegar)
    rewind(m, config.KEY_BACKOFF_S + 0.01)
    a = m.step(sense(), 0.05)
    assert a.throttle == 0 and a.turn != 0, "pivote sin traslacion"
    assert cmds(a).get("key") == config.SERVO_KEY_HOLD, "servo rearmado a HOLD al empezar el giro"
    assert "key_off" not in cmds(a), "NO se apaga en la misma orden: el eje aun no llego"

    # todavia dentro del tiempo de asentamiento: sigue sin apagarse
    a = m.step(sense(), 0.05)
    assert "key_off" not in cmds(a)

    # pasado SERVO_SETTLE_S: se apaga, aunque el giro siga
    m._detach_at -= config.SERVO_SETTLE_S + 0.01
    a = m.step(sense(), 0.05)
    assert m.state == S.BACKOFF_KEY and "key_off" in cmds(a), \
        "apagado del servo en cuanto el eje ha llegado, sin esperar a la maniobra"

    # media vuelta hecha -> patrulla, sin mas ordenes al servo
    rewind(m, config.KEY_TURN_S + 0.01)
    a = m.step(sense(), 0.05)
    assert m.state == S.PATROL and not a.commands
    ok("secuencia completa: amarillo -> entrar -> soltar -> retroceder -> girar+rearmar -> apagar -> patrulla")


def test_apagado_respeta_el_asentamiento():
    """detach() antes de que el eje llegue lo deja a medio recorrido: el
    apagado tiene que ir SIEMPRE por detras de SERVO_SETTLE_S."""
    assert config.SERVO_SETTLE_S >= 0.6, \
        "SERVO_SETTLE_S debe cubrir los ~600 ms de recorrido del servo (SERVO_MOVE_MS)"
    m = Mission()
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    rewind(m, config.ZONE_ENTER_ADVANCE_S + 0.01); m.step(amarillo(), 0.05)
    rewind(m, config.KEY_RELEASE_S + 0.01);        m.step(amarillo(), 0.05)
    rewind(m, config.KEY_BACKOFF_S + 0.01);        m.step(sense(), 0.05)
    programado = m._detach_at - time.monotonic()
    assert config.SERVO_SETTLE_S - 0.1 <= programado <= config.SERVO_SETTLE_S, \
        f"el apagado se programo a {programado:.2f} s de la ultima orden"
    ok("el apagado se programa SERVO_SETTLE_S despues de la ultima orden al servo")


def test_rearme_cancela_apagado_pendiente():
    """Si se rearma la llave con un apagado programado, ese apagado no debe
    ejecutarse despues y soltar la llave recien cargada."""
    m = Mission()
    m._detach_at = time.monotonic() + 0.5
    m.reset_key()
    assert m._detach_at is None
    a = m.step(sense(), 0.05)
    assert "key_off" not in cmds(a)
    ok("rearmar la llave cancela un apagado pendiente")


def test_amarillo_prohibido_tras_depositar():
    m = Mission()
    m.start_patrol(sense())
    m.key_deposited = True
    a = m.step(amarillo(), 0.05)
    assert m.state == S.AVOID_ZONE, "con la llave depositada el amarillo se esquiva"
    assert not a.commands, "no se vuelve a mover el servo"
    a = m.step(amarillo(), 0.05)
    assert a.throttle < 0, "primero retroceder (la caja esta dentro)"
    rewind(m, config.AVOID_BACK_S + 0.01)
    a = m.step(sense(), 0.05)
    assert a.turn != 0
    rewind(m, config.AVOID_TURN_S + 0.01)
    m.step(sense(), 0.05)
    assert m.state == S.PATROL
    assert m.key_drops == 0, "esquivar no cuenta como soltar"
    ok("amarillo = zona prohibida tras depositar: rebota sin tocar el servo")


def test_avoid_alterna_el_lado():
    m = Mission()
    m.start_patrol(sense())
    m.key_deposited = True
    m.step(amarillo(), 0.05)
    rewind(m, config.AVOID_BACK_S + 0.01)
    a1 = m.step(sense(), 0.05)
    rewind(m, config.AVOID_TURN_S + 0.01)
    m.step(sense(), 0.05)
    m.step(amarillo(), 0.05)
    rewind(m, config.AVOID_BACK_S + 0.01)
    a2 = m.step(sense(), 0.05)
    assert (a1.turn > 0) != (a2.turn > 0), "dos rebotes seguidos giran a lados distintos"
    ok("el rebote en amarillo alterna el lado: no se queda en bucle")


def test_borde_interrumpe_la_entrada_pero_no_la_suelta():
    m = Mission()
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)                       # entrando
    assert m.state == S.DEPOSIT_KEY
    m.step(amarillo(black_mask=0b0001), 0.05)      # negro delante
    assert m.state == S.EDGE, "el borde manda sobre la entrada"
    assert not m.key_deposited, "no se solto: la llave sigue a bordo"

    # ahora la fase de SOLTAR no se interrumpe: el servo ya esta girando
    m2 = Mission()
    m2.start_patrol(sense())
    m2.step(amarillo(), 0.05)
    rewind(m2, config.ZONE_ENTER_ADVANCE_S + 0.01)
    m2.step(amarillo(), 0.05)                      # servo a RELEASE
    assert m2._key_phase == "drop"
    m2.step(amarillo(black_mask=0b0001), 0.05)
    assert m2.state == S.DEPOSIT_KEY, "soltando no se abandona a medias"
    rewind(m2, config.KEY_RELEASE_S + 0.01)
    m2.step(amarillo(black_mask=0b0001), 0.05)
    assert m2.state == S.EDGE, "y justo despues, el borde se atiende"
    ok("borde negro: interrumpe la entrada, respeta la suelta, y luego manda")


def test_angulos_de_la_llave_coherentes():
    """SERVO_KEY_HOLD (config.py) y SERVO_KEY_HOLD_DEFAULT (config.h) tienen
    que ser el mismo angulo: si no, el servo se coloca en una posicion al
    arrancar el sketch y salta a otra cuando la App manda la suya, con la
    llave ya cargada encima."""
    import re
    ruta = os.path.join(ROOT, "sketch", "config.h")
    texto = open(ruta, encoding="utf-8", errors="replace").read()
    m = re.search(r"#define\s+SERVO_KEY_HOLD_DEFAULT\s+(\d+)", texto)
    assert m, "no se encontro SERVO_KEY_HOLD_DEFAULT en sketch/config.h"
    assert int(m.group(1)) == config.SERVO_KEY_HOLD, (
        f"config.h dice {m.group(1)} y config.py dice {config.SERVO_KEY_HOLD}")
    assert config.SERVO_KEY_HOLD != config.SERVO_KEY_RELEASE
    ok(f"angulos de la llave coherentes: retener {config.SERVO_KEY_HOLD}, "
       f"soltar {config.SERVO_KEY_RELEASE}")


def test_rearme_para_repetir():
    m = Mission()
    m.key_deposited = True
    m.reset_key()
    assert not m.key_deposited
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    assert m.state == S.DEPOSIT_KEY, "rearmada, vuelve a depositar"
    ok("rearmar la llave permite repetir la prueba sin reiniciar la App")


def test_amarillo_instantaneo_no_dispara():
    """Solo el color ESTABLE dispara la secuencia: una lectura suelta no."""
    m = Mission()
    m.start_patrol(sense())
    m.step(sense(color=config.C_YELLOW, color_stable=config.C_WHITE), 0.05)
    assert m.state == S.PATROL
    ok("una lectura amarilla suelta no dispara la llave")


def test_parser_campo_llave():
    s = _parse_sense("0,0,0,900,900,900,900,1,1,1,50,64,10,5,2,0,50,0,180")
    assert s.key_angle == 180 and s.servo_moving
    viejo = _parse_sense("0,0,0,900,900,900,900,1,1,1,50,0,10,5,2,0,50,0")
    assert viejo.key_angle == -1, "sin el campo, angulo desconocido"
    ok("parser: angulo del servo de la llave (campo 19) y bit de asentamiento")


def test_arranque_manda_hold_y_ordenes_llegan():
    import main
    m = main
    m.config.CONTROL_DT = 0.0
    m.config.DIAG_MCU_S = 0
    log = t1._utils.Bridge.log
    servos = [a for n, a in log if n == "servo"]
    assert (config.SERVO_KEY, config.SERVO_KEY_HOLD) in servos, \
        "al arrancar el servo debe ir a HOLD con el valor de config.py"
    time.sleep(0.15)
    m.ui._start()
    m.mission.key_deposited = False
    m.mission.state = S.DEPOSIT_KEY
    m.mission._key_phase = "enter"
    m.mission._phase_until = time.monotonic() - 1
    m.link._sense.color_stable = config.C_YELLOW
    m.control_loop()
    servos = [a for n, a in log if n == "servo"]
    assert (config.SERVO_KEY, config.SERVO_KEY_RELEASE) in servos, \
        "la orden de soltar tiene que llegar al MCU como servo(2, 180)"
    m.ui._key_reset()
    assert not m.mission.key_deposited
    ok("main.py: HOLD al arrancar, RELEASE llega al MCU, rearme por la pagina")


if __name__ == "__main__":
    print("ROVER H07 - demo parte 2 (llave) - pruebas\n")
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
