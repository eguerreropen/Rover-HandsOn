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


def spins(action):
    """Los giros del servo continuo de la caja: [(angulo, segundos), ...]."""
    return [v for n, v in action.commands if n == "key_spin"]


def ordenes_caja(action):
    """Cualquier orden que toque la caja, sea del tipo que sea."""
    return [n for n, _ in action.commands if n.startswith("key")]


def hasta_fin_de_fase(m, s=None):
    """Consume la fase actual del todo y da un paso mas."""
    rewind(m, 60.0)
    return m.step(s or sense(), 0.05)


# ============================================================ SECUENCIA
def test_secuencia_completa():
    """SERVO CONTINUO (28/09): la caja se tira GIRANDO un tiempo, no yendo a
    un angulo. Secuencia medida en la App de prueba:

        ve amarillo -> PARA -> gira 70 durante 0.3 s (tira la caja)
        -> quieto 1 s (la caja se asienta) -> gira 115 durante 0.3 s (recoge
        el brazo) -> retrocede -> media vuelta -> patrulla
    """
    m = Mission()
    assert m.start_patrol(sense())
    a = m.step(sense(), 0.05)
    assert a.throttle == config.PATROL_SPEED

    # pisa amarillo -> se para en seco
    a = m.step(amarillo(), 0.05)
    assert m.state == S.DEPOSIT_KEY and m._key_phase == "enter"
    assert a.throttle == 0 and a.turn == 0
    assert not ordenes_caja(a) and not m.key_deposited

    # TIRAR: el giro medido, y la llave cuenta como depositada desde ya
    a = hasta_fin_de_fase(m, amarillo())
    assert spins(a) == [(config.KEY_DROP_ANGLE, config.KEY_DROP_S)], a.commands
    assert (config.KEY_DROP_ANGLE, config.KEY_DROP_S) == (70, 0.3), \
        "los valores medidos en la App de prueba"
    assert m._key_phase == "drop" and a.throttle == 0
    assert m.key_deposited and m.key_drops == 1

    # mientras gira: quieto y sin ordenes
    a = m.step(amarillo(), 0.05)
    assert a.throttle == 0 and a.turn == 0 and not a.commands

    # termina el giro -> ESPERA quieto mientras la caja se asienta
    a = hasta_fin_de_fase(m, amarillo())
    assert m._key_phase == "wait" and a.throttle == 0 and not ordenes_caja(a)

    # termina la espera -> RECOGER el brazo, TODAVIA PARADO
    a = hasta_fin_de_fase(m, amarillo())
    assert spins(a) == [(config.KEY_RETRACT_ANGLE, config.KEY_RETRACT_S)]
    assert (config.KEY_RETRACT_ANGLE, config.KEY_RETRACT_S) == (115, 0.3)
    assert m._key_phase == "retract" and a.throttle == 0 and a.turn == 0
    assert m.state == S.DEPOSIT_KEY, "aun no se mueve: el brazo esta volviendo"

    # brazo recogido -> AHORA si retrocede (la caja esta DELANTE)
    a = hasta_fin_de_fase(m, amarillo())
    assert m.state == S.BACKOFF_KEY and a.throttle < 0 and a.turn == 0

    # retroceso hecho -> media vuelta, SIN tocar el servo (ya se recogio)
    a = hasta_fin_de_fase(m)
    assert a.throttle == 0 and a.turn != 0, "pivote sin traslacion"
    assert not ordenes_caja(a), "el brazo ya se recogio antes de retroceder"

    rewind(m, config.KEY_TURN_S + 0.01)
    a = m.step(sense(), 0.05)
    assert m.state == S.PATROL and not a.commands
    ok("secuencia: para -> gira 70/0.3 -> espera -> gira 115/0.3 -> retrocede -> patrulla")


def test_el_brazo_se_recoge_ANTES_de_retroceder():
    """En pista, con el servo de posicion, el brazo extendido se enganchaba al
    dar marcha atras: se rearmaba DESPUES del retroceso. Ahora el giro de
    recoger sale antes de la primera orden de marcha atras."""
    m = Mission()
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    eventos = []
    for _ in range(12):
        a = hasta_fin_de_fase(m, amarillo())
        for g in spins(a):
            eventos.append(("recoger" if g[0] == config.KEY_RETRACT_ANGLE
                            else "tirar"))
        if a.throttle < 0:
            eventos.append("atras")
        if m.state == S.PATROL:
            break
    assert "recoger" in eventos and "atras" in eventos, eventos
    assert eventos.index("recoger") < eventos.index("atras"), (
        f"el brazo tiene que estar recogido antes de dar marcha atras: {eventos}")
    ok("el brazo se recoge ANTES de dar marcha atras (ya no se engancha)")


def test_la_fsm_espera_a_que_el_mcu_termine_cada_giro():
    """El giro lo para el MCU. La FSM no puede mandar la siguiente orden (o
    arrancar las ruedas) con el servo todavia girando: cada fase dura al menos
    el giro mas un margen."""
    m = Mission()
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    hasta_fin_de_fase(m, amarillo())                  # -> drop
    dur_drop = m._phase_until - time.monotonic()
    assert dur_drop >= config.KEY_DROP_S, (dur_drop, config.KEY_DROP_S)
    hasta_fin_de_fase(m, amarillo())                  # -> wait
    hasta_fin_de_fase(m, amarillo())                  # -> retract
    dur_ret = m._phase_until - time.monotonic()
    assert dur_ret >= config.KEY_RETRACT_S, (dur_ret, config.KEY_RETRACT_S)
    assert config.KEY_SPIN_MARGIN_S > 0
    ok("cada fase de giro dura al menos lo que el giro, mas un margen")


def test_nunca_se_manda_la_caja_a_un_angulo():
    """El peligro de verdad del servo continuo. ("key", angulo) lo pondria a
    girar SIN LIMITE de tiempo. En toda la mision no puede aparecer: solo
    ("key_spin", (angulo, segundos)), que el MCU para solo."""
    m = Mission()
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    vistos = []
    for _ in range(12):
        a = hasta_fin_de_fase(m, amarillo())
        vistos += ordenes_caja(a)
        if m.state == S.PATROL:
            break
    assert vistos and set(vistos) == {"key_spin"}, vistos
    ok("la mision solo manda giros con tiempo a la caja, nunca un angulo")


def test_el_amarillo_ya_no_se_esquiva_tras_depositar():
    """DESACTIVADO en pista (18/09). La garantia geometrica era buena sobre el
    papel -no entrar en el amarillo = no pisar la caja- pero la zona esta en
    el centro y rebotar en ella cortaba la patrulla constantemente, justo
    cuando el rover deberia ir a por la bandera.

    Coste asumido: el rover PUEDE empujar la caja que acaba de depositar.
    """
    assert config.AVOID_YELLOW_AFTER_DROP is False
    m = Mission()
    m.start_patrol(sense())
    m.key_deposited = True
    a = m.step(amarillo(), 0.05)
    assert m.state == S.PATROL, "ya no rebota: sigue patrullando"
    assert a.throttle == config.PATROL_SPEED, "y sigue avanzando"

    # El estado AVOID_ZONE sigue implementado: solo esta desconectado.
    prev = config.AVOID_YELLOW_AFTER_DROP
    try:
        config.AVOID_YELLOW_AFTER_DROP = True
        m2 = Mission()
        m2.start_patrol(sense())
        m2.key_deposited = True
        a = m2.step(amarillo(), 0.05)
        assert m2.state == S.AVOID_ZONE, "con True vuelve a esquivar"
        assert not a.commands, "no se vuelve a mover el servo"
        a = m2.step(amarillo(), 0.05)
        assert a.throttle < 0, "primero retroceder (la caja esta dentro)"
    finally:
        config.AVOID_YELLOW_AFTER_DROP = prev
    ok("el amarillo ya no se esquiva tras depositar (el interruptor lo revierte)")


def test_avoid_alterna_el_lado():
    prev = config.AVOID_YELLOW_AFTER_DROP
    config.AVOID_YELLOW_AFTER_DROP = True      # el estado sigue existiendo
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
    config.AVOID_YELLOW_AFTER_DROP = prev
    assert (a1.turn > 0) != (a2.turn > 0), "dos rebotes seguidos giran a lados distintos"
    ok("el rebote en amarillo alterna el lado: no se queda en bucle")


def test_el_borde_no_interrumpe_ni_los_giros_ni_la_espera():
    """Con el rover PARADO, un borde negro no interrumpe nada: la caja esta
    cayendo o el brazo volviendo, y cortar a medias los deja a mitad. El borde
    se atiende justo despues, y el escape ya retrocede si el negro esta
    delante -- que es donde esta la caja."""
    m = Mission()
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    hasta_fin_de_fase(m, amarillo())
    for fase in ("drop", "wait", "retract"):
        assert m._key_phase == fase, (m._key_phase, fase)
        a = m.step(amarillo(black_mask=0b0001), 0.05)
        assert m.state == S.DEPOSIT_KEY and a.throttle == 0, \
            f"un borde no interrumpe la fase {fase}"
        if fase != "retract":
            hasta_fin_de_fase(m, amarillo(black_mask=0b0001))
    rewind(m, 60.0)
    m.step(amarillo(black_mask=0b0001), 0.05)
    assert m.state == S.EDGE, "y en cuanto el brazo esta recogido, el borde manda"
    ok("el borde respeta tirar, esperar y recoger; y manda en cuanto terminan")


def test_espera_parado_tras_tirar():
    """Ni un tick de movimiento desde que ve el amarillo hasta que el brazo
    esta recogido: la caja cae delante, en el punto ciego del rover."""
    assert config.KEY_WAIT_AFTER_DROP_S >= 1.0
    m = Mission()
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    hasta_fin_de_fase(m, amarillo())
    while m.state == S.DEPOSIT_KEY:
        for _ in range(3):
            a = m.step(amarillo(), 0.05)
            assert a.throttle == 0 and a.turn == 0, m._key_phase
        a = hasta_fin_de_fase(m, amarillo())
    assert m.state == S.BACKOFF_KEY and a.throttle < 0
    total = (config.KEY_DROP_S + config.KEY_WAIT_AFTER_DROP_S
             + config.KEY_RETRACT_S + 2 * config.KEY_SPIN_MARGIN_S)
    ok(f"parado ~{total:.1f} s entre ver el amarillo y volver a moverse")


def test_el_sketch_no_puede_dejar_la_caja_girando():
    """Comprobaciones sobre el CODIGO del MCU, porque ahi es donde estaba el
    peligro: con el servo de posicion, begin() hacia move(KEY, 180) al dar
    corriente -- en un servo continuo, maxima velocidad sin fin."""
    import re
    leer = lambda n: open(os.path.join(ROOT, "sketch", n), encoding="utf-8",
                          errors="replace").read()
    act, cfg, ino = leer("actuators.h"), leer("config.h"), leer("sketch.ino")

    begin = re.search(r"void begin\(\)\s*\{(.*?)\n  \}", act, re.S).group(1)
    begin_sin_coment = "\n".join(l for l in begin.splitlines()
                                   if not l.strip().startswith("//"))
    assert "move(" not in begin_sin_coment and "write(" not in begin_sin_coment, \
        "begin() no puede mover NINGUN servo: la caja arrancaria girando"
    assert "SERVO_KEY_HOLD_DEFAULT" not in cfg, \
        "el angulo de 'retener' al arrancar no tiene sentido en un servo continuo"
    # Sin comentarios: "// act.tick();" contiene el texto pero no hace nada, y
    # una busqueda ingenua lo daria por bueno.
    loop = ino[ino.index("void loop()"):]
    loop = re.sub(r"/\*.*?\*/", "", loop, flags=re.S)
    loop = "\n".join(l.split("//")[0] for l in loop.splitlines())
    assert re.search(r"\bact\.tick\(\)\s*;", loop), \
        "sin act.tick() en loop(), el giro no se para nunca"
    assert '"spin"' in ino, "el MCU tiene que exponer el RPC spin"
    assert re.search(r"#define\s+KEY_SPIN_MAX_MS\s+\d+", cfg), "tope de seguridad"
    ok("sketch: nada gira al arrancar, loop() para el giro, y hay tope de tiempo")


def test_rearme_para_repetir():
    m = Mission()
    m.key_deposited = True
    m.reset_key()
    assert not m.key_deposited
    m.start_patrol(sense())
    m.step(amarillo(), 0.05)
    assert m.state == S.DEPOSIT_KEY, "rearmada, vuelve a depositar"
    ok("rearmar la llave permite repetir la prueba sin reiniciar la App")


def test_una_sola_lectura_de_amarillo_dispara_el_deposito():
    """EL FALLO QUE ARREGLA (visto en pista 18/09): la pagina mostraba el
    amarillo apareciendo un instante y el rover no depositaba nada.

    La causa era este filtro: el deposito miraba el color ESTABLE, que exige N
    lecturas seguidas, y cruzando la cinta solo daba tiempo a una. El color
    instantaneo cambiaba -por eso se veia en la pagina- pero el estable no
    llegaba a cuajar nunca.

    El intercambio NO es simetrico: un amarillo perdido significa que la llave
    no se deposita jamas; un amarillo falso, que cae donde no debia. Lo
    primero es seguro, lo segundo improbable y una sola vez por ronda.
    """
    m = Mission()
    m.start_patrol(sense())
    a = m.step(sense(color=config.C_YELLOW, color_stable=config.C_WHITE), 0.05)
    assert m.state == S.DEPOSIT_KEY, (
        "una sola lectura de amarillo tiene que bastar "
        f"(ZONE_TRIGGER_INSTANT = {config.ZONE_TRIGGER_INSTANT})")
    assert a.throttle == 0, "y se para en seco (ZONE_ENTER_ADVANCE_S = 0)"

    # Y el interruptor devuelve el comportamiento antiguo.
    prev = config.ZONE_TRIGGER_INSTANT
    try:
        config.ZONE_TRIGGER_INSTANT = False
        m2 = Mission()
        m2.start_patrol(sense())
        m2.step(sense(color=config.C_YELLOW, color_stable=config.C_WHITE), 0.05)
        assert m2.state == S.PATROL, "con False vuelve a exigir el color estable"
    finally:
        config.ZONE_TRIGGER_INSTANT = prev
    ok("una sola lectura de amarillo dispara el deposito (y el interruptor lo revierte)")


def test_parser_campo_llave():
    s = _parse_sense("0,0,0,900,900,900,900,1,1,1,50,64,10,5,2,0,50,0,180")
    assert s.key_angle == 180 and s.servo_moving
    viejo = _parse_sense("0,0,0,900,900,900,900,1,1,1,50,0,10,5,2,0,50,0")
    assert viejo.key_angle == -1, "sin el campo, angulo desconocido"
    ok("parser: angulo del servo de la llave (campo 19) y bit de asentamiento")


def test_arranque_no_mueve_la_caja_y_el_giro_llega_al_mcu():
    import main
    m = main
    m.config.CONTROL_DT = 0.0
    m.config.DIAG_MCU_S = 0
    log = t1._utils.Bridge.log

    # Al arrancar: NINGUN angulo a la caja (antes: servo(2, 180) = girar sin
    # fin), y un servo_off por si quedaba algo girando.
    assert not [a for n, a in log if n == "servo" and a[0] == config.SERVO_KEY], \
        "al arrancar no puede mandarse un angulo al servo continuo de la caja"
    assert ("servo_off", (config.SERVO_KEY,)) in log

    # La mision tira la caja: llega al MCU como spin(2, 70, 300).
    time.sleep(0.15)
    m.ui._start()
    m.mission.key_deposited = False
    m.mission.state = S.DEPOSIT_KEY
    m.mission._key_phase = "enter"
    m.mission._phase_until = time.monotonic() - 1
    m.link._sense.color_stable = config.C_YELLOW
    m.control_loop()
    assert ("spin", (config.SERVO_KEY, 70, 300)) in log, \
        [e for e in log if e[0] == "spin"]

    # Una orden ("key", angulo) se RECHAZA en vez de mandarse.
    antes = len([e for e in log if e[0] == "servo"])
    m.run_commands([("key", 180)])
    assert len([e for e in log if e[0] == "servo"]) == antes

    # Los botones de la pagina usan los mismos valores que la mision.
    m.ui.__dict__.pop("_x", None)
    m.ui._key_spin(config.KEY_RETRACT_ANGLE, config.KEY_RETRACT_S)
    assert ("spin", (config.SERVO_KEY, 115, 300)) in log

    # Rearmar ya no mueve el servo: solo lo para y resetea la mision.
    m.mission.key_deposited = True
    antes = len([e for e in log if e[0] in ("servo", "spin")])
    m.ui._key_reset()
    assert not m.mission.key_deposited
    assert len([e for e in log if e[0] in ("servo", "spin")]) == antes
    ok("main.py: nada a la caja al arrancar, spin(2, 70, 300) llega al MCU, "
       "('key', x) se rechaza")


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
