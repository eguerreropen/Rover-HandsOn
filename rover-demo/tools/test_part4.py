#!/usr/bin/env python3
"""
Pruebas de la parte 4 (agarrar la bandera y volver a casa):

    python3 tools/test_part4.py

Necesita OpenCV para las de distancia (el detector se prueba con frames
sinteticos de verdad); si no lo hay, esas se omiten.
"""
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "python"))

import test_part3 as t3                                  # noqa: E402  (stubs)
from test_part3 import CV, cv2, np, BLUE                 # noqa: E402
from test_part1 import sense, ok, PASSED, FAILED         # noqa: E402

import config                                            # noqa: E402
import steer                                             # noqa: E402
from mission import Mission, S                           # noqa: E402
import importlib.util as _ilu                            # noqa: E402

# calib_dist.py es un script, no un modulo: se carga por ruta para poder
# probar el MISMO ajuste que se le pide al usuario que ejecute.
_spec = _ilu.spec_from_file_location(
    "tools_calib", os.path.join(ROOT, "tools", "calib_dist.py"))
tools_calib = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(tools_calib)
sys.modules["tools_calib"] = tools_calib


MOVES = ("grip", "lift", "key")


def servo_cmds(action):
    """Solo las ordenes que MUEVEN un servo. Fuera quedan el LED (que va por
    su cuenta) y los apagados (que llegan cuando el eje ha llegado, no en el
    momento de la maniobra, y se comprueban aparte)."""
    return [(n, v) for n, v in action.commands if n in MOVES]


def off_cmds(action):
    return [n for n, _ in action.commands if n.endswith("_off")]


def correr(m, n, **kw):
    """n ciclos de FSM. Devuelve la ultima accion."""
    a = None
    for _ in range(n):
        a = m.step(sense(**kw.pop("sense_kw", {})) if False else sense(**kw), 
                   config.CONTROL_DT, flag=kw.get("flag"))
    return a


# ================================================ DISTANCIA (el fallo grande)
def escena(d_cm):
    """La bandera a d_cm, apoyada en el suelo y vista desde una camara baja:
    su base cae fuera del encuadre por abajo, que es el caso real."""
    h = config.FOCAL_PX * config.FLAG_HEIGHT_CM / d_cm
    w = config.FOCAL_PX * config.FLAG_DIAM_CM / d_cm
    f = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    x0 = int(320 - w / 2)
    cv2.rectangle(f, (max(0, x0), max(0, int(480 - h))),
                  (min(639, int(x0 + w)), 479), BLUE, -1)
    return f


def test_la_distancia_sale_solo_del_ancho():
    """La distancia ya NO usa el alto para nada.

    El alto se satura: con el cuarto superior borrado quedan 360 px utiles y
    la bandera deja de caber a partir de ~23 cm; a partir de ahi el alto se
    queda clavado y la distancia con el, y  d <= GRAB_DISTANCE_CM  no se
    cumple nunca. Y ademas el alto MIENTE: la bandera es un cilindro cuyo
    diametro son 5 cm se mire desde donde se mire, pero cuyos 15 cm de alto
    solo se miden si se ven los dos bordes, y la base queda tapada.
    """
    if not CV:
        print("       (sin cv2: se omite)")
        return
    from vision import Vision
    v = Vision("blue")

    previa = None
    for d in (40, 32, 25, 20, 16, config.GRAB_DISTANCE_CM):
        fl = v.process(escena(d))
        assert fl is not None, f"a {d} cm se pierde la deteccion"
        est = fl.dist_raw_cm
        assert abs(est - d) < max(2.0, d * 0.08), f"a {d} cm estima {est:.1f}"
        if previa is not None:
            assert est < previa - 1.0, (
                f"la distancia DEJO DE BAJAR: {previa:.1f} -> {est:.1f} cm")
        previa = est
    ok("distancia: sale del ancho y sigue bajando hasta el agarre")


def test_el_alto_no_influye_en_la_distancia():
    """Prueba directa: dos manchas del MISMO ancho y distinto alto tienen que
    dar la MISMA distancia. Si alguien vuelve a meter el alto en la formula,
    esto falla."""
    if not CV:
        return
    from vision import Vision
    v = Vision("blue")

    def barra(w_px, h_px):
        f = np.full((480, 640, 3), (235, 235, 235), np.uint8)
        cv2.rectangle(f, (int(320 - w_px / 2), int(300 - h_px)),
                      (int(320 + w_px / 2), 300), BLUE, -1)
        return f

    a = v.process(barra(60, 180))
    b = v.process(barra(60, 300))       # mismo ancho, alto muy distinto
    assert a and b, "las dos se detectan"
    assert abs(a.dist_raw_cm - b.dist_raw_cm) < 0.01, (
        f"el alto influye: {a.dist_raw_cm:.2f} vs {b.dist_raw_cm:.2f} cm")
    ok("el alto NO entra en la distancia: mismo ancho, misma lectura")


def test_el_desfase_constante_necesita_dos_puntos():
    """LA RAZON DE QUE 'CAMBIO LA FOCAL Y SIGUE SIN CUADRAR DE CERCA'.

    El modelo de camara mide desde el CENTRO OPTICO; las distancias utiles se
    miden desde la pinza. Entre uno y otro hay un numero fijo de cm que
    NINGUNA focal puede absorber, porque no es un factor de escala: es un
    sumando. Se demuestra con numeros, que es la unica forma de que convenza.
    """
    from tools_calib import ajusta        # el mismo ajuste que usa la pagina
    A_real, B_real = 2775.0, 5.0
    ancho = lambda d: A_real / (d - B_real)

    # Un solo punto, calibrando lejos: solo puede despejar A.
    d_cal = 80.0
    A_solo = d_cal * ancho(d_cal)
    err_lejos = A_solo / ancho(d_cal) - d_cal
    err_cerca = A_solo / ancho(14.0) - 14.0
    assert abs(err_lejos) < 0.01, "en el punto de calibracion siempre cuadra"
    assert abs(err_cerca) > 3.0, (
        f"y de cerca falla por {err_cerca:.1f} cm: es el sintoma que se ve")

    # Dos puntos: A y B salen exactos.
    A, B = ajusta([(80.0, ancho(80.0)), (20.0, ancho(20.0))])
    assert abs(A - A_real) < 1.0 and abs(B - B_real) < 0.05, (A, B)
    for d in (60.0, 30.0, 14.0):
        assert abs(A / ancho(d) + B - d) < 0.05, f"y cuadra a {d} cm"
    ok(f"desfase constante: con 1 punto falla {err_cerca:+.1f} cm de cerca; "
       f"con 2 sale exacto")


def test_el_error_x_se_corrige_si_esta_recortada_de_lado():
    """Si media bandera esta fuera por la izquierda, el centro de la caja que
    se ve esta MAS A LA DERECHA que el centro real: el rover creeria estar mas
    centrado de lo que esta, y el agarre exige |error| < GRAB_ALIGN_TOL."""
    if not CV:
        return
    from vision import Vision
    v = Vision("blue")
    f = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    cv2.rectangle(f, (0, 200), (70, 380), BLUE, -1)      # cortada por la izda
    fl = v.process(f)
    assert fl is not None and fl.cut_lft
    ingenuo = (fl.cx - 0.5) * 2.0
    assert fl.error_x <= ingenuo, "el error corregido no puede ser menor"
    assert abs(fl.error_x) > config.GRAB_ALIGN_TOL, (
        "con media bandera fuera de cuadro NO puede dar por centrada")
    ok("error_x: recortada de lado -> no se da por centrada")


# ==================================================== ALINEACION ANTES DE AGARRAR
def test_no_agarra_descentrada():
    """Cerrar la pinza con la bandera a un lado del eje la TIRA, y una bandera
    tumbada ya no se recoge. Primero se centra, sin avanzar."""
    m = Mission()
    m.key_deposited = True
    s = sense()

    m.force_state(S.FLAG_REACHED, s)
    torcida = t3.FakeFlag(error_x=0.45, distance_cm=config.GRAB_DISTANCE_CM)
    a = m.step(s, config.CONTROL_DT, flag=torcida)
    assert m.state == S.FLAG_REACHED, "descentrada: NO agarra todavia"
    assert a.throttle == 0, "y NO avanza: a esta distancia solo empujaria"
    assert a.turn != 0, "pivota para centrarse"
    izq = m.step(s, config.CONTROL_DT,
                 flag=t3.FakeFlag(error_x=-0.45, distance_cm=14.0))
    assert izq.turn * a.turn < 0, "gira al otro lado si el error cambia de signo"

    centrada = t3.FakeFlag(error_x=0.02, distance_cm=config.GRAB_DISTANCE_CM)
    m.step(s, config.CONTROL_DT, flag=centrada)
    assert m.state == S.GRAB, "centrada: ahora si"
    ok("agarre: primero centrar (pivotando, sin avanzar), luego cerrar")


def test_compromiso_si_se_pierde_muy_cerca():
    """A menos de ~10 cm la bandera ni pasa el filtro de ancho: se pierde
    justo en el ultimo palmo. Abandonar ahi es no agarrarla nunca."""
    m = Mission()
    m.key_deposited = True
    s = sense()

    m.force_state(S.APPROACH, s)
    m.step(s, config.CONTROL_DT,
           flag=t3.FakeFlag(error_x=0.0,
                            distance_cm=config.GRAB_COMMIT_CM - 0.5))
    assert m.state == S.APPROACH
    m.step(s, config.CONTROL_DT, flag=None)      # desaparece
    assert m.state == S.GRAB, "se perdio DENTRO del compromiso: agarra igual"

    m2 = Mission()
    m2.key_deposited = True
    m2.force_state(S.APPROACH, s)
    m2.step(s, config.CONTROL_DT,
            flag=t3.FakeFlag(error_x=0.0, distance_cm=config.GRAB_COMMIT_CM + 40))
    t0 = time.monotonic()
    while m2.state == S.APPROACH and time.monotonic() - t0 < config.LOST_TARGET_S + 1.0:
        m2.step(s, config.CONTROL_DT, flag=None)
        time.sleep(0.02)
    assert m2.state == S.PATROL, "lejos y perdida: se abandona, no se agarra al aire"
    ok("compromiso: se agarra si se pierde cerca; se abandona si se pierde lejos")


# ============================================================ SECUENCIA DE AGARRE
def test_secuencia_de_agarre():
    """Pinza, luego elevacion. Nunca los dos a la vez, y siempre parado: dos
    ZOSKAY de 20 kg arrancando juntos es el pico de corriente mas alto."""
    m = Mission()
    m.key_deposited = True
    s = sense()
    m.force_state(S.GRAB, s)

    orden = []
    t0 = time.monotonic()
    while m.state == S.GRAB and time.monotonic() - t0 < 6.0:
        a = m.step(s, config.CONTROL_DT)
        assert a.throttle == 0 and a.turn == 0, "el agarre se hace PARADO"
        sc = servo_cmds(a)
        assert len(sc) <= 1, f"los servos se mueven de uno en uno: {sc}"
        orden += sc
        time.sleep(0.02)

    assert orden[0] == ("grip", config.SERVO_GRIP_CLOSE), f"primero la pinza: {orden}"
    assert orden[1] == ("lift", config.SERVO_LIFT_UP), f"luego la elevacion: {orden}"
    assert config.GRAB_PAUSE_S > 0, "tiene que haber pausa entre una y otra"
    assert m.has_flag, "queda marcada como agarrada"
    assert m.state == S.CARRY, (
        "con RETURN_AFTER_GRAB = True encadena solo a la vuelta a casa")
    ok("agarre: cerrar pinza -> subir elevacion -> vuelta a casa")


def test_el_encadenado_a_casa_es_un_interruptor():
    """Encadena solo (lo pedido en pista). El interruptor sigue ahi para
    volver a pararse en HOLDING mientras se depura una etapa suelta."""
    assert config.RETURN_AFTER_GRAB is True
    prev = config.RETURN_AFTER_GRAB
    try:
        config.RETURN_AFTER_GRAB = False
        m = Mission()
        m.key_deposited = True
        s = sense()
        m.force_state(S.GRAB, s)
        t0 = time.monotonic()
        while m.state == S.GRAB and time.monotonic() - t0 < 8.0:
            m.step(s, config.CONTROL_DT)
            time.sleep(0.02)
        assert m.state == S.HOLDING, "con False se para tras levantar"
    finally:
        config.RETURN_AFTER_GRAB = prev
    ok("RETURN_AFTER_GRAB: encadena a casa, y el interruptor lo para en HOLDING")


def test_no_persigue_otra_bandera_llevando_una():
    m = Mission()
    m.key_deposited = True
    m.has_flag = True
    assert not m._hunt_allowed(), (
        "con una bandera en la pinza, APPROACH conduciria hacia otra y la "
        "tiraria con la que lleva")
    ok("con bandera agarrada, el cerrojo de persecucion se cierra")


# ============================================================== VUELTA A CASA
def test_la_caja_amarilla_ya_no_esta_prohibida():
    """AVOID_YELLOW_AFTER_DROP = False (18/09): cruzar el amarillo ya no
    interrumpe nada. Coste asumido: el rover puede empujar su propia caja."""
    m = Mission()
    m.key_deposited = True
    m.force_state(S.CARRY, sense())
    m.step(sense(color_stable=config.C_YELLOW), config.CONTROL_DT)
    assert m.state == S.CARRY, "sigue camino a casa sin rebotar"

    prev = config.AVOID_YELLOW_AFTER_DROP
    try:
        config.AVOID_YELLOW_AFTER_DROP = True
        m2 = Mission()
        m2.key_deposited = True
        m2.force_state(S.CARRY, sense())
        m2.step(sense(color_stable=config.C_YELLOW), config.CONTROL_DT)
        assert m2.state == S.AVOID_ZONE, "con True vuelve a estar vetado"
    finally:
        config.AVOID_YELLOW_AFTER_DROP = prev
    ok("el amarillo ya no interrumpe la vuelta a casa (interruptor disponible)")


def test_una_interrupcion_devuelve_a_donde_estaba():
    """EL FALLO QUE ESTO EVITA: EDGE devolvia SIEMPRE a PATROL. El rover con
    la bandera rebotaba en el borde y se le olvidaba que volvia a casa -- y
    peor, PATROL habria intentado perseguir otra bandera con una puesta."""
    m = Mission()
    m.key_deposited = True
    m.force_state(S.CARRY, sense())
    m._carry_armed = True
    assert m._resume == S.CARRY

    negro = sense(black_mask=0b0001)          # FL pisa negro
    m.step(negro, config.CONTROL_DT)
    assert m.state == S.EDGE, "el borde manda tambien llevando la bandera"
    assert m._resume == S.CARRY, "pero se recuerda a donde volver"
    t0 = time.monotonic()
    while m.state == S.EDGE and time.monotonic() - t0 < 5.0:
        m.step(sense(), config.CONTROL_DT)
        time.sleep(0.02)
    assert m.state == S.CARRY, f"vuelve a CARRY, no a PATROL (esta en {m.state})"

    # Y sin bandera, lo de siempre.
    m2 = Mission()
    m2.force_state(S.PATROL, sense())
    m2.step(negro, config.CONTROL_DT)
    assert m2._resume == S.PATROL
    ok("borde y esquive devuelven al estado que se estaba haciendo, no a PATROL")


# =================================================================== ENTREGA
def test_entrega_baja_antes_de_abrir():
    """El orden importa: abrir en alto deja caer la bandera desde la altura
    del brazo y lo mas probable es que ruede fuera de la zona."""
    m = Mission()
    m.key_deposited = True
    m.force_state(S.DELIVER, sense())
    assert m.has_flag

    orden = []
    t0 = time.monotonic()
    while m.state == S.DELIVER and time.monotonic() - t0 < 8.0:
        a = m.step(sense(color_stable=config.TEAM_COLOR), config.CONTROL_DT)
        orden += servo_cmds(a)
        time.sleep(0.02)

    nombres = [n for n, _ in orden]
    assert nombres[:2] == ["lift", "grip"], f"bajar y LUEGO abrir: {orden}"
    assert dict(orden)["lift"] == config.SERVO_LIFT_DOWN
    assert dict(orden)["grip"] == config.SERVO_GRIP_OPEN
    assert not m.has_flag and m.flag_delivered
    assert m.state == S.DONE
    ok("entrega: entrar, bajar, abrir, retroceder -> DONE")


def test_mision_completa_de_principio_a_fin():
    """APPROACH -> FLAG_REACHED -> GRAB -> CARRY -> DELIVER -> DONE, con un
    borde negro por el medio."""
    m = Mission()
    m.key_deposited = True
    visto = [m.state]

    def paso(n=1, **kw):
        fl = kw.pop("flag", None)
        for _ in range(n):
            m.step(sense(**kw), config.CONTROL_DT, flag=fl)
            if m.state != visto[-1]:
                visto.append(m.state)

    cerca = config.GRAB_DISTANCE_CM - 0.5
    m.force_state(S.PATROL, sense())
    visto.append(m.state)
    paso(flag=t3.FakeFlag(error_x=0.3, distance_cm=90))       # -> APPROACH
    paso(3, flag=t3.FakeFlag(error_x=0.1, distance_cm=40))
    paso(flag=t3.FakeFlag(error_x=0.02, distance_cm=cerca))   # -> FLAG_REACHED
    paso(flag=t3.FakeFlag(error_x=0.02, distance_cm=cerca))   # -> GRAB
    t0 = time.monotonic()
    while m.state == S.GRAB and time.monotonic() - t0 < 6.0:
        paso(); time.sleep(0.02)
    assert m.state == S.CARRY and m.has_flag, "encadena solo a la vuelta"
    paso(2, color_stable=config.C_WHITE)                      # de vuelta
    paso(color_stable=config.TEAM_COLOR)                      # -> DELIVER
    t0 = time.monotonic()
    while m.state == S.DELIVER and time.monotonic() - t0 < 8.0:
        paso(color_stable=config.TEAM_COLOR); time.sleep(0.02)

    esperado = [S.IDLE, S.PATROL, S.APPROACH, S.FLAG_REACHED, S.GRAB,
                S.CARRY, S.DELIVER, S.DONE]
    assert visto == esperado, f"recorrido: {visto}"
    assert m.flag_delivered and not m.has_flag
    ok("mision completa: patrulla -> bandera -> agarre -> casa -> entrega")


# ==================================================================== PAGINA
class _FakeLink:
    def __init__(self):
        self.calls = []
        self._sense = sense()
    def sense(self):    return self._sense
    def servo(self, sid, deg):
        self.calls.append(("servo", sid, deg)); return True
    def servo_off(self, sid):
        self.calls.append(("off", sid)); return True
    def stop(self):     self.calls.append(("stop",))


class _FakeTracker:
    def calib_dict(self, s): return {}


def test_la_pagina_controla_el_brazo():
    from ui import RoverUI
    m = Mission()
    link = _FakeLink()
    ui = RoverUI(m, link, _FakeTracker())

    ui._arm(config.SERVO_GRIP, config.SERVO_GRIP_CLOSE)
    assert link.calls[-1] == ("servo", config.SERVO_GRIP, config.SERVO_GRIP_CLOSE)
    ui._arm(config.SERVO_LIFT, config.SERVO_LIFT_UP)
    assert link.calls[-1] == ("servo", config.SERVO_LIFT, config.SERVO_LIFT_UP)
    ui._arm_off(config.SERVO_LIFT)
    assert link.calls[-1] == ("off", config.SERVO_LIFT)

    link.calls.clear()
    ui._arm_home()
    assert ("servo", config.SERVO_GRIP, config.SERVO_GRIP_OPEN) in link.calls
    assert ("servo", config.SERVO_LIFT, config.SERVO_LIFT_DOWN) in link.calls

    # Rearmar la bandera: la deja sin agarrar Y devuelve el brazo a su sitio,
    # porque si no habria que colocar la bandera con la pinza cerrada.
    m.has_flag = True
    m.flag_delivered = True
    link.calls.clear()
    ui._flag_reset()
    assert not m.has_flag and not m.flag_delivered
    assert ("servo", config.SERVO_GRIP, config.SERVO_GRIP_OPEN) in link.calls
    ok("la pagina mueve pinza y elevacion, y rearma la bandera con el brazo")


def test_los_estados_nuevos_son_forzables():
    from ui import mission_states
    d = Mission().as_dict()
    for st in (S.GRAB, S.HOLDING, S.CARRY, S.DELIVER, S.DONE):
        assert st in d["forceable"], f"{st} tiene que tener boton"
        assert st in list(mission_states())
    for st in (S.GRAB, S.CARRY, S.DELIVER):
        assert st in d["moving"], f"{st} mueve el rover: exige sensores"
    assert S.DONE not in d["moving"], "DONE esta parado"
    assert S.HOLDING not in d["moving"], "HOLDING esta parado"

    # Los que mueven se rechazan sin TCRT, tambien los nuevos.
    m = Mission()
    ciego = sense()
    ciego.flags &= ~0x1000
    for st in (S.GRAB, S.CARRY, S.DELIVER):
        okk, why = m.force_state(st, ciego)
        assert not okk and "TCRT" in why, f"{st}: {okk} {why}"
    ok("los estados de la parte 4 tienen boton y respetan los mismos cerrojos")


def test_ordenes_del_brazo_traducidas_a_rpc():
    """run_commands es el UNICO sitio donde una orden de la FSM se convierte
    en un RPC. Si falta una, la FSM 'manda' algo que no llega a ningun sitio."""
    import main
    link = _FakeLink()
    real, main.link = main.link, link
    try:
        main.run_commands([("grip", config.SERVO_GRIP_CLOSE),
                           ("lift", config.SERVO_LIFT_UP),
                           ("lift_off", None)])
    finally:
        main.link = real
    assert link.calls == [("servo", config.SERVO_GRIP, config.SERVO_GRIP_CLOSE),
                          ("servo", config.SERVO_LIFT, config.SERVO_LIFT_UP),
                          ("off", config.SERVO_LIFT)], link.calls
    ok("run_commands traduce grip / lift / lift_off a los RPC del MCU")


def test_el_margen_antes_de_saturar_el_ancho_y_como_se_compensa():
    """EL NUMERO MAS AJUSTADO DE LA PARTE 4.

    El ancho tambien se satura: cuando la bandera llena los 640 px, medirla
    mas ancha es imposible y la distancia se queda clavada -- el mismo fallo
    que tenia el alto, solo que 20 cm mas cerca. Con A = 2775 eso pasa a
    4.3 cm, y el agarre esta a 6: quedan 1.7 cm.

    1.7 cm es POCO, y la unica defensa es no llegar rapido: por eso existe el
    tramo de arrastre (FINAL_APPROACH_CM / SPEED_FINAL) y por eso el filtro de
    mediana se apaga de cerca en vez de anadir 0.3 s de retardo justo ahi.
    Esta prueba fija esos tres numeros juntos: si alguien baja
    GRAB_DISTANCE_CM o quita el arrastre, se entera aqui.
    """
    a = config.FLAG_DIST_A
    d_sat = a / (640.0 * config.FLAG_MAX_WIDTH_FRAC)   # el ancho llena el cuadro
    margen = config.GRAB_DISTANCE_CM - d_sat

    assert margen > 0, (
        f"el ancho se satura a {d_sat:.1f} cm y el agarre esta a "
        f"{config.GRAB_DISTANCE_CM}: la condicion no se cumple nunca")
    assert margen >= 1.0, f"solo {margen:.2f} cm de margen: insuficiente"

    # Con tan poco margen, el arrastre final es obligatorio.
    assert config.FINAL_APPROACH_CM > config.GRAB_DISTANCE_CM
    assert config.SPEED_FINAL < config.SPEED_CREEP, (
        "el ultimo tramo tiene que ser MAS LENTO que el creep")
    assert steer.approach_speed(config.GRAB_DISTANCE_CM + 1) == config.SPEED_FINAL
    assert steer.approach_speed(config.GRAB_DISTANCE_CM) == 0

    # Y el filtro no puede anadir retardo en ese tramo.
    w_agarre = a / config.GRAB_DISTANCE_CM
    assert w_agarre > config.FLAG_DIST_TRUST_PX, (
        f"a la distancia de agarre el ancho es {w_agarre:.0f} px y el umbral "
        f"de confianza {config.FLAG_DIST_TRUST_PX}: el filtro seguiria "
        f"metiendo 0.3 s de retardo con solo {margen:.1f} cm de margen")

    # El compromiso tiene que estar pegado al agarre, no 16 cm por encima.
    assert config.GRAB_DISTANCE_CM < config.GRAB_COMMIT_CM <= config.GRAB_DISTANCE_CM + 5
    ok(f"agarre a {config.GRAB_DISTANCE_CM} cm, el ancho satura a {d_sat:.1f}: "
       f"{margen:.1f} cm de margen, con arrastre y sin retardo de filtro")


def test_el_filtro_de_distancia_se_apaga_de_cerca():
    """De lejos el ancho son 28 px y +-2 px son +-7 %: el filtro gana mucho.
    De cerca son 462 px y esos +-2 px son +-0.4 %: el filtro no aporta nada y
    su retardo es puro riesgo. Asi que se apaga solo."""
    if not CV:
        return
    from vision import Vision
    v = Vision("blue")

    # Lejos: la mediana actua (la lectura filtrada puede diferir de la cruda).
    for _ in range(3):
        v.process(escena(60))
    assert len(v._dist_win) > 1, "de lejos se acumula ventana"

    # Cerca: ventana vacia y la distancia usada es la cruda.
    fl = v.process(escena(config.GRAB_DISTANCE_CM + 1))
    assert fl.w_px >= config.FLAG_DIST_TRUST_PX
    assert v._dist_win == [], "de cerca el filtro se apaga"
    assert fl.distance_cm == fl.dist_raw_cm, "y se usa la lectura cruda"
    ok("el filtro de distancia se apaga cuando la medida ya es fiable")


def test_el_area_de_referencia_es_lo_que_limita_el_alcance():
    """FLAG_NORM_AREA_PX era un 4000 escrito a mano dentro de detect(). Ahora
    esta en config, porque es la palanca del alcance: con la focal medida, la
    bandera cubre 4000 px a 74 cm, y mas lejos el factor de area resta.

    Y baja el alcance SIN ayudar a la cinta del suelo: una cinta ocupa 15000 px
    o mas, su factor de area ya vale 1.0 y bajar el umbral no puede subirlo.
    """
    if not CV:
        return
    from vision import Vision, score_flag

    a_flag = 3000.0        # bandera lejana
    a_cinta = 15600.0      # cinta del suelo de 260x60
    alto, _ = score_flag(0.95, 3.0, a_flag, 4000.0, False)
    bajo, _ = score_flag(0.95, 3.0, a_flag, 1500.0, False)
    assert bajo > alto, "bajar el area de referencia sube la puntuacion de lejos"

    c_alto, pa = score_flag(0.95, 0.23, a_cinta, 4000.0, False)
    c_bajo, pb = score_flag(0.95, 0.23, a_cinta, 1500.0, False)
    assert pa[2] == pb[2] == 1.0, "la cinta ya saturaba el factor de area"
    assert c_alto == c_bajo, (
        "bajar el area de referencia NO puede subir la puntuacion de la cinta")

    # Y de verdad, con el detector: la misma escena con los dos valores.
    prev = config.FLAG_NORM_AREA_PX
    try:
        h = config.FOCAL_PX * 15 / 110.0
        w = config.FOCAL_PX * 5 / 110.0
        f = np.full((480, 640, 3), (232, 232, 232), np.uint8)
        cv2.rectangle(f, (int(320 - w / 2), int(300 - h)),
                      (int(320 + w / 2), 300), BLUE, -1)
        config.FLAG_NORM_AREA_PX = 4000.0
        s_lejos = Vision("blue").process(f)
        config.FLAG_NORM_AREA_PX = 1500.0
        s_cerca = Vision("blue").process(f)
        assert s_cerca is not None and s_cerca.score > (s_lejos.score if s_lejos else 0)
    finally:
        config.FLAG_NORM_AREA_PX = prev
    ok("el area de referencia amplia el alcance sin favorecer a la cinta")


def test_hay_pausa_entre_cerrar_la_pinza_y_levantar():
    """La pinza ya ha llegado a su angulo, pero la bandera sigue
    acomodandose dentro y el chasis balanceandose por el tiron del cierre.
    Levantar en ese instante es cuando se escapa."""
    m = Mission()
    m.key_deposited = True
    s = sense()
    m.force_state(S.GRAB, s)

    t_grip = t_lift = None
    t0 = time.monotonic()
    while m.state == S.GRAB and time.monotonic() - t0 < 8.0:
        a = m.step(s, config.CONTROL_DT)
        for n, _ in servo_cmds(a):
            if n == "grip" and t_grip is None:
                t_grip = time.monotonic()
            if n == "lift" and t_lift is None:
                t_lift = time.monotonic()
        time.sleep(0.02)

    assert t_grip and t_lift, "las dos ordenes tienen que salir"
    hueco = t_lift - t_grip
    esperado = config.GRIP_CLOSE_S + config.GRAB_PAUSE_S
    assert abs(hueco - esperado) < 0.25, (
        f"entre cerrar y levantar pasaron {hueco:.2f} s; se esperaban "
        f"{esperado:.2f} (cierre {config.GRIP_CLOSE_S} + pausa "
        f"{config.GRAB_PAUSE_S})")
    # Y la pausa no la interrumpe un borde negro: el rover ya esta parado.
    # sense() NUEVO: el de arriba tiene ya varios segundos y force_state lo
    # rechaza por telemetria caducada -- que es justo lo que debe hacer.
    m2 = Mission(); m2.key_deposited = True
    assert m2.force_state(S.GRAB, sense())[0]
    m2._grab_phase = "pause"
    m2._phase_until = time.monotonic() + 1.0
    m2.step(sense(black_mask=0b0001), config.CONTROL_DT)
    assert m2.state == S.GRAB, "un borde no interrumpe la pausa"
    ok(f"pausa de {config.GRAB_PAUSE_S} s entre cerrar la pinza y levantar")


def test_la_llegada_a_casa_es_directa():
    """El "armado" (ver otro color antes, o esperar N segundos) se ELIMINO.

    Protegia de un caso que no puede darse: que el rover agarrara la bandera
    pisando ya su propia cinta. La bandera rival esta siempre sobre LA ZONA
    RIVAL, asi que en el instante del agarre el rover pisa el color del
    ENEMIGO, nunca el suyo. A cambio, la condicion se atasco en pista:
    entrando en CARRY levantado en el aire el APDS lee "?", que no armaba, y
    el rover no entrego pese a estar sobre su cinta.
    """
    assert not hasattr(config, "CARRY_ARM_AFTER_S"), "el armado ya no existe"
    m = Mission()
    m.key_deposited = True
    m.force_state(S.CARRY, sense())
    assert not hasattr(m, "_carry_armed"), "ni su estado interno"

    # Primera lectura del color propio: entrega. Sin preambulos.
    m.step(sense(color_stable=config.TEAM_COLOR), config.CONTROL_DT)
    assert m.state == S.DELIVER, "entrega en la PRIMERA lectura"

    # Y en el aire ("?") sigue caminando, que es lo unico que puede hacer.
    m2 = Mission()
    m2.key_deposited = True
    m2.force_state(S.CARRY, sense())
    a = m2.step(sense(color=config.C_UNKNOWN, color_stable=config.C_UNKNOWN),
                config.CONTROL_DT)
    assert m2.state == S.CARRY and a.throttle == config.CARRY_SPEED
    ok("la llegada a casa es directa: primera lectura del color propio")


def test_suelta_en_seco_al_llegar_a_casa():
    """La zona propia es FINA: 1.1 s de avance a ciegas la cruzaban entera y
    la bandera acababa fuera por el otro lado -- el mismo fallo que tuvo la
    caja amarilla."""
    assert config.CARRY_ENTER_ADVANCE_S == 0.0
    m = Mission()
    m.key_deposited = True
    m.force_state(S.CARRY, sense())
    a = m.step(sense(color_stable=config.TEAM_COLOR), config.CONTROL_DT)
    assert m.state == S.DELIVER
    assert a.throttle == 0 and a.turn == 0, (
        f"tiene que parar en seco, no avanzar ({a.throttle})")
    ok("suelta en seco al leer su color, sin avanzar dentro de la zona")


def test_el_retroceso_final_dura_lo_que_dice():
    """FALLO ENCONTRADO EN LA TRAZA (18/09): la espera generica de _deliver
    devolvia cero para TODAS las fases, incluida "back". Resultado: el
    retroceso duraba UN ciclo de control (50 ms) en vez de DELIVER_BACK_S, y
    el rover se quedaba practicamente encima de la bandera recien soltada --
    justo lo que el retroceso existe para evitar."""
    m = Mission()
    m.key_deposited = True
    m.force_state(S.DELIVER, sense())

    ticks_atras = 0
    t0 = time.monotonic()
    while m.state == S.DELIVER and time.monotonic() - t0 < 8.0:
        a = m.step(sense(color_stable=config.TEAM_COLOR), config.CONTROL_DT)
        if a.throttle < 0:
            ticks_atras += 1
        time.sleep(0.05)

    # A 0.05 s por tick, DELIVER_BACK_S deberia dar bastantes mas de uno.
    minimo = int(config.DELIVER_BACK_S / 0.05 * 0.6)
    assert ticks_atras >= minimo, (
        f"solo {ticks_atras} ciclos de retroceso; con DELIVER_BACK_S = "
        f"{config.DELIVER_BACK_S} s deberian ser al menos {minimo}. "
        f"El retroceso se esta cortando a un solo tick.")
    ok(f"el retroceso de la entrega dura {config.DELIVER_BACK_S} s de verdad "
       f"({ticks_atras} ciclos)")


def test_la_pinza_termina_de_abrirse_antes_de_retroceder():
    """DELIVER_OPEN_S valia 0.7 y el asentamiento de la pinza 0.9: el rover
    empezaba a retroceder con la pinza AUN ABRIENDOSE y se llevaba la bandera
    por delante. Importa mas ahora que se suelta en el borde de una zona fina."""
    assert config.DELIVER_OPEN_S >= config.settle_s("grip"), (
        f"abrir dura {config.DELIVER_OPEN_S} s y el servo tarda "
        f"{config.settle_s('grip')}: se retrocede con la pinza a medias")
    assert config.DELIVER_DOWN_S >= config.settle_s("lift")
    ok("cada fase de la entrega cubre el recorrido de su servo")


def test_la_llegada_no_espera_al_color_estable():
    """Cruzando la cinta de casa no siempre da tiempo a dos lecturas seguidas:
    el mismo motivo por el que el deposito de la llave no se disparaba."""
    m = Mission()
    m.key_deposited = True
    m.force_state(S.CARRY, sense())
    m.step(sense(color=config.TEAM_COLOR, color_stable=config.C_WHITE),
           config.CONTROL_DT)
    assert m.state == S.DELIVER, (
        "una sola lectura del color del equipo tiene que bastar")
    ok("la llegada a casa se dispara con una sola lectura")


# ==================================================== APAGADO DE LOS SERVOS
def test_toda_orden_de_servo_programa_su_apagado():
    """Un ZOSKAY de 20 kg parado contra su tope no sujeta mejor por estar
    alimentado: zumba, se calienta y hunde el rail del UBEC toda la ronda.
    Cada orden de servo tiene que programar su propio corte de pulsos.

    Y UNO POR SERVO, no uno compartido: si los tres compartieran plazo, la
    orden al gripper reiniciaria el del brazo y el brazo se quedaria encendido
    -- que es justo lo que se quiere evitar, y en silencio."""
    m = Mission()
    m.key_deposited = True
    s = sense()
    m.force_state(S.GRAB, s)

    vistos, apagados = [], []
    t0 = time.monotonic()
    while m.state == S.GRAB and time.monotonic() - t0 < 6.0:
        a = m.step(s, config.CONTROL_DT)
        vistos += [n for n, _ in servo_cmds(a)]
        apagados += off_cmds(a)
        time.sleep(0.02)
    # HOLDING: se siguen emitiendo los apagados pendientes
    t0 = time.monotonic()
    while time.monotonic() - t0 < config.SERVO_SETTLE_S + 0.3:
        apagados += off_cmds(m.step(s, config.CONTROL_DT))
        time.sleep(0.02)

    assert vistos == ["grip", "lift"], vistos
    assert set(apagados) == {"grip_off", "lift_off"}, (
        f"cada servo movido tiene que acabar apagado: movidos {vistos}, "
        f"apagados {apagados}")
    assert m._detach_at == {}, "y sin plazos colgando"
    ok("cada orden de servo programa su apagado, con un plazo independiente")


def test_el_apagado_espera_a_que_el_eje_llegue():
    """write() es asincrono: cortar los pulsos antes de tiempo deja el eje a
    medio recorrido. El apagado va SIEMPRE por detras de SERVO_SETTLE_S."""
    assert config.settle_s("grip") >= 0.8, "tiene que cubrir el recorrido"
    m = Mission()
    m.key_deposited = True
    s = sense()
    m.force_state(S.GRAB, s)
    a = m.step(s, config.CONTROL_DT)          # settle -> close: manda grip
    while not servo_cmds(a):
        a = m.step(s, config.CONTROL_DT)
        time.sleep(0.02)
    assert off_cmds(a) == [], "no se apaga en la misma orden que mueve"
    plazo = m._detach_at["grip"] - time.monotonic()
    assert config.settle_s("grip") - 0.1 <= plazo <= config.settle_s("grip"), plazo
    ok("el apagado se programa SERVO_SETTLE_S despues de la orden, no antes")


def test_el_gripper_se_puede_dejar_alimentado():
    """El interruptor que importa. Los otros dos servos apagados no pierden
    nada; el gripper SI: si el reductor no aguanta el cierre, apagarlo SUELTA
    la bandera y no hay sensor que avise."""
    prev = dict(config.SERVO_AUTO_OFF)
    try:
        config.SERVO_AUTO_OFF["grip"] = False
        m = Mission()
        m.key_deposited = True
        s = sense()
        m.force_state(S.GRAB, s)
        apagados = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            apagados += off_cmds(m.step(s, config.CONTROL_DT))
            time.sleep(0.02)
        assert "lift_off" in apagados, "el brazo si se apaga"
        assert "grip_off" not in apagados, (
            "con grip: False la pinza queda alimentada mientras sujeta")
    finally:
        config.SERVO_AUTO_OFF.clear()
        config.SERVO_AUTO_OFF.update(prev)
    ok("SERVO_AUTO_OFF['grip'] = False deja la pinza alimentada sujetando")


def test_la_caja_se_tira_con_los_valores_medidos():
    """SERVO CONTINUO (28/09). Valores medidos en la App de prueba servo-test
    con la caja montada: tirar = 70 durante 0.3 s, recoger = 115 durante 0.3 s.
    Y la FSM los manda como GIROS CON TIEMPO, que el MCU para solo -- no como
    angulos, que dejarian el servo girando sin fin."""
    assert (config.KEY_DROP_ANGLE, config.KEY_DROP_S) == (70, 0.3)
    assert (config.KEY_RETRACT_ANGLE, config.KEY_RETRACT_S) == (115, 0.3)
    assert not hasattr(config, "SERVO_KEY_RELEASE"), "ya no hay angulo de soltar"
    assert not hasattr(config, "SERVO_KEY_HOLD"), "ni de retener"
    assert "key" not in config.SERVO_AUTO_OFF, (
        "la caja no usa el apagado desde el MPU: la para el MCU al acabar el giro")

    m = Mission()
    m.start_patrol(sense())
    m.step(sense(color_stable=config.C_YELLOW), config.CONTROL_DT)
    assert m.state == S.DEPOSIT_KEY
    giros = []
    t0 = time.monotonic()
    while len(giros) < 2 and time.monotonic() - t0 < 6.0:
        a = m.step(sense(color_stable=config.C_YELLOW), config.CONTROL_DT)
        giros += [v for n, v in a.commands if n == "key_spin"]
        assert not [n for n, _ in a.commands if n == "key"], "nunca un angulo"
        time.sleep(0.02)
    assert giros == [(70, 0.3), (115, 0.3)], giros
    ok("la caja: tirar 70/0.3 s y recoger 115/0.3 s, como giros con tiempo")


# ============================================== VELOCIDAD GLOBAL (SPEED_SCALE)
def test_las_rectas_se_compensan_y_los_giros_NO():
    """LA CORRECCION DE PISTA DEL 18/09.

    Cuando se anadio SPEED_SCALE, los tiempos de giro se dividian entre el
    factor suponiendo que los grados por segundo son proporcionales a la
    velocidad normalizada. EN UN PIVOTE SOBRE EL EJE ESO ES FALSO: las ruedas
    patinan de lado y patinan MAS cuanto mas rapido van, asi que a velocidad
    baja se giran mas grados de lo que predice el modelo.

    Medido: con el factor a 0.55 la compensacion llevaba EDGE_TURN_S a 1.45 s
    y el rover giraba casi 180 grados para esquivar un borde.

    Los GIROS se calibran ahora por observacion (tabla de config, ~124 grados
    por segundo). Las RECTAS si se siguen compensando: ahi el patinaje es
    mucho menor y la distancia importa de verdad.
    """
    k = config.SPEED_SCALE

    # Rectas: velocidad x tiempo tiene que dar lo mismo que con el factor a 1.
    for v, t, v0, t0 in [
            (config.ESCAPE_SPEED, config.KEY_BACKOFF_S,         260, 1.0),
            (config.ESCAPE_SPEED, config.DELIVER_BACK_S,        260, 0.8)]:
        assert abs(v - round(v0 * k)) <= 1, f"velocidad {v} != {v0} x {k}"
        assert abs(v * t - v0 * t0) / (v0 * t0) < 0.03, (
            f"recta descompensada: {v}x{t:.2f} frente a {v0}x{t0}")

    # Los dos avances dentro de zona valen 0 (el rover se para en seco al ver
    # el color), y 0 escalado sigue siendo 0: no entran en la comprobacion.
    assert config.ZONE_ENTER_ADVANCE_S == 0.0
    assert config.CARRY_ENTER_ADVANCE_S == 0.0

    # Giros: valores crudos, NO divididos entre el factor.
    for nombre, t, original in [("EDGE_TURN_S", config.EDGE_TURN_S, 0.8),
                                ("KEY_TURN_S", config.KEY_TURN_S, 1.6),
                                ("AVOID_TURN_S", config.AVOID_TURN_S, 0.9)]:
        assert abs(t - original / k) > 0.1, (
            f"{nombre} = {t} sigue compensado ({original}/{k} = {original/k:.2f}): "
            f"a esta velocidad eso gira casi el doble de lo que deberia")

    # Y el escape del borde tiene que girar bastante MENOS que media vuelta.
    assert config.EDGE_TURN_S < config.KEY_TURN_S / 2, (
        f"EDGE_TURN_S ({config.EDGE_TURN_S}) deberia ser mucho menor que media "
        f"vuelta ({config.KEY_TURN_S}): esquivar un borde no es darse la vuelta")
    ok(f"rectas compensadas, giros crudos: esquivar el borde gira "
       f"~{config.EDGE_TURN_S / config.KEY_TURN_S * 180:.0f} grados, no 180")


def test_el_escape_del_borde_es_mas_corto():
    """Retroceder menos al evitar el borde: apenas hace falta separarse para
    poder pivotar, y cada centimetro de retroceso es tiempo perdido."""
    assert config.EDGE_BACK_S <= 0.6, config.EDGE_BACK_S
    assert config.AVOID_BACK_S <= 0.6, config.AVOID_BACK_S
    # Pero el retroceso tras soltar la llave SI se mantiene largo: ahi hay una
    # caja delante de la que hay que alejarse de verdad.
    assert config.KEY_BACKOFF_S > config.EDGE_BACK_S * 2
    ok("el escape del borde retrocede menos, el de la llave no")


def test_lo_que_el_factor_NO_toca():
    """Un servo tarda lo que tarda en recorrer su arco, y una deteccion
    caduca cuando caduca: ninguna de las dos cosas depende de lo rapido que
    ande el rover. Escalarlas seria un error silencioso."""
    assert config.GRIP_CLOSE_S == 0.8
    assert config.LIFT_UP_S == 0.9
    assert config.DELIVER_DOWN_S == 0.9 and config.DELIVER_OPEN_S == 1.0
    assert config.KEY_DROP_S == 0.3 and config.KEY_RETRACT_S == 0.3
    # Los giros de la caja NO se escalan con SPEED_SCALE: son del servo, no de
    # las ruedas, y se midieron tal cual en la App de prueba.
    assert config.GRAB_SETTLE_S == 0.3
    assert config.LOST_TARGET_S == 1.5 and config.DETECTION_STALE_S == 0.7
    # Y las distancias en cm son fisicas: no tienen nada que ver con esto.
    assert config.GRAB_DISTANCE_CM == 6.0
    assert config.FINAL_APPROACH_CM == 14.0 and config.GRAB_COMMIT_CM == 9.0
    ok("el factor NO toca tiempos de servo, caducidades ni distancias en cm")


def test_el_perfil_de_aproximacion_sigue_siendo_de_tres_tramos():
    """Escalar no puede alterar el ORDEN de las velocidades: si el arrastre
    dejara de ser el tramo mas lento, el ultimo palmo se recorreria rapido y
    el rover se metería dentro de la bandera."""
    assert (config.SPEED_FINAL < config.SPEED_CREEP < config.SPEED_APPROACH), (
        f"orden roto: final {config.SPEED_FINAL}, creep {config.SPEED_CREEP}, "
        f"approach {config.SPEED_APPROACH}")
    assert steer.approach_speed(100) == config.SPEED_APPROACH
    assert steer.approach_speed(config.GRAB_DISTANCE_CM + 1) == config.SPEED_FINAL
    assert steer.approach_speed(config.GRAB_DISTANCE_CM) == 0
    ok("el perfil sigue siendo crucero > rampa > arrastre > parado")


if __name__ == "__main__":
    print("ROVER H07 - demo parte 4 (agarrar la bandera y volver) - pruebas\n")
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
