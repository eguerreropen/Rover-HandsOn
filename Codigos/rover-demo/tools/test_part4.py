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


SERVOS = ("grip", "lift", "key", "lift_off", "key_off")


def servo_cmds(action):
    """Solo las ordenes de servo. El LED de bandera se manda por su cuenta y
    no forma parte de la secuencia mecanica."""
    return [(n, v) for n, v in action.commands if n in SERVOS]


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
    assert m.has_flag, "queda marcada como agarrada"
    assert m.state == S.HOLDING, (
        "con RETURN_AFTER_GRAB = False la secuencia TERMINA aqui: acercarse, "
        "agarrar, levantar, quieto")
    a = m.step(s, config.CONTROL_DT)
    assert a.throttle == 0 and a.turn == 0, "y se queda quieto"
    ok("agarre: cerrar pinza -> subir elevacion -> quieto (no encadena solo)")


def test_el_encadenado_a_casa_es_un_interruptor():
    """La vuelta a casa sigue implementada y probada; solo esta apagada."""
    prev = config.RETURN_AFTER_GRAB
    try:
        config.RETURN_AFTER_GRAB = True
        m = Mission()
        m.key_deposited = True
        s = sense()
        m.force_state(S.GRAB, s)
        t0 = time.monotonic()
        while m.state == S.GRAB and time.monotonic() - t0 < 6.0:
            m.step(s, config.CONTROL_DT)
            time.sleep(0.02)
        assert m.state == S.CARRY, "con True si encadena"
    finally:
        config.RETURN_AFTER_GRAB = prev
    assert not config.RETURN_AFTER_GRAB, "y por defecto esta APAGADO"
    ok("RETURN_AFTER_GRAB: apagado por defecto, la vuelta sigue disponible")


def test_no_persigue_otra_bandera_llevando_una():
    m = Mission()
    m.key_deposited = True
    m.has_flag = True
    assert not m._hunt_allowed(), (
        "con una bandera en la pinza, APPROACH conduciria hacia otra y la "
        "tiraria con la que lleva")
    ok("con bandera agarrada, el cerrojo de persecucion se cierra")


# ============================================================== VUELTA A CASA
def test_vuelta_a_casa_por_transicion_de_color():
    """La llegada se detecta por TRANSICION. Si el rover agarra la bandera
    pisando ya su propia cinta, 'estoy sobre el color del equipo' seria cierto
    desde el primer ciclo y soltaria la bandera sin ir a ninguna parte."""
    m = Mission()
    m.key_deposited = True
    s_casa = sense()
    m.force_state(S.CARRY, s_casa)

    # Ciclo 1: ya sobre la cinta propia. NO debe entregar.
    for _ in range(5):
        a = m.step(sense(color_stable=config.TEAM_COLOR), config.CONTROL_DT)
    assert m.state == S.CARRY, "sobre su color desde el principio: NO entrega"
    assert a.throttle > 0, "sigue avanzando"

    # Sale de la zona: se arma.
    m.step(sense(color_stable=config.C_WHITE), config.CONTROL_DT)
    assert m._carry_armed, "ha visto otro color: ya cuenta la siguiente vez"

    # Vuelve a entrar: ahora si.
    m.step(sense(color_stable=config.TEAM_COLOR), config.CONTROL_DT)
    assert m.state == S.DELIVER, "transicion a su color: entrega"
    ok("vuelta a casa: la zona se detecta por TRANSICION, no por estar encima")


def test_la_caja_amarilla_sigue_prohibida_llevando_la_bandera():
    m = Mission()
    m.key_deposited = True
    m.force_state(S.CARRY, sense())
    m._carry_armed = True
    m.step(sense(color_stable=config.C_YELLOW), config.CONTROL_DT)
    assert m.state == S.AVOID_ZONE, "el amarillo con la caja dentro sigue vetado"
    ok("la caja depositada sigue siendo zona prohibida con la bandera encima")


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
    # AQUI TERMINA lo que se ha pedido para esta etapa. Lo de abajo solo se
    # llega lanzandolo a mano desde la pagina, y se prueba asi mismo.
    assert m.state == S.HOLDING and m.has_flag
    m.force_state(S.CARRY, sense())
    visto.append(m.state)
    paso(3, color_stable=config.C_WHITE)                      # CARRY, se arma
    paso(color_stable=config.TEAM_COLOR)                      # -> DELIVER
    t0 = time.monotonic()
    while m.state == S.DELIVER and time.monotonic() - t0 < 8.0:
        paso(color_stable=config.TEAM_COLOR); time.sleep(0.02)

    esperado = [S.IDLE, S.PATROL, S.APPROACH, S.FLAG_REACHED, S.GRAB,
                S.HOLDING, S.CARRY, S.DELIVER, S.DONE]
    assert visto == esperado, f"recorrido: {visto}"
    assert m.flag_delivered and not m.has_flag
    ok("recorrido completo: patrulla -> bandera -> agarre -> QUIETO, y a mano "
       "casa -> entrega")


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
