#!/usr/bin/env python3
"""
Pruebas de la parte 3 (camara y bandera) SIN robot:  python3 tools/test_part3.py

Necesita OpenCV en el PC (pip install opencv-python-headless numpy): el
detector se prueba con frames SINTETICOS de verdad, no con stubs. Si no hay
cv2, esas pruebas se omiten y se avisa.
"""
import os
import sys
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "python"))

# --- stub del periferico de camara de App Lab: entrega frames sinteticos ---
try:
    import cv2
    import numpy as np
    CV = True
except ImportError:
    CV = False

_periph = types.ModuleType("arduino.app_peripherals")
_camera = types.ModuleType("arduino.app_peripherals.camera")


class FakeCamera:
    """Se hace pasar por arduino.app_peripherals.camera.Camera."""
    scene = "flag_center"

    def __init__(self, *a, **kw):
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def capture(self):
        time.sleep(1.0 / 30)
        return make_frame(FakeCamera.scene)


_camera.Camera = FakeCamera
_periph.camera = _camera

import test_part1 as t1                              # noqa: E402  (stubs de App Lab)
sys.modules["arduino"].app_peripherals = _periph
sys.modules.update({"arduino.app_peripherals": _periph,
                    "arduino.app_peripherals.camera": _camera})
from test_part1 import sense, ok, PASSED, FAILED     # noqa: E402

import config                                        # noqa: E402
import steer                                         # noqa: E402
from mission import Mission, S                       # noqa: E402

BLUE = (200, 60, 20)          # BGR: azul saturado (el rival por defecto)
RED = (20, 20, 220)


# El color de la bandera RIVAL depende de config.TEAM, y este fichero se
# ejecuta con el TEAM que este configurado. Fijarlo a azul hacia que, con
# TEAM = "blue" (rival rojo), la escena sintetica no contuviera nada que el
# detector buscara -- y el fallo parecia de main.py, no de la prueba.
ENEMY_BGR = BLUE if config.ENEMY == "blue" else RED


def make_frame(scene):
    f = np.full((480, 640, 3), (235, 235, 235), np.uint8)      # pista blanca
    if scene == "flag_enemy":
        cv2.rectangle(f, (300, 200), (340, 320), ENEMY_BGR, -1)
    elif scene == "flag_center":
        cv2.rectangle(f, (300, 200), (340, 320), BLUE, -1)      # 40x120, aspecto 3
    elif scene == "flag_right":
        cv2.rectangle(f, (500, 220), (530, 320), BLUE, -1)
    elif scene == "flag_left_far":
        cv2.rectangle(f, (100, 240), (118, 300), BLUE, -1)      # 18x60
    elif scene == "tape_only":
        cv2.rectangle(f, (40, 400), (600, 430), BLUE, -1)       # ancha y baja
    elif scene == "own_flag":
        cv2.rectangle(f, (300, 200), (340, 320), RED, -1)
    elif scene == "ceiling":
        cv2.rectangle(f, (300, 5), (340, 105), BLUE, -1)
    elif scene == "flag_and_tape":
        cv2.rectangle(f, (300, 200), (340, 320), BLUE, -1)
        cv2.rectangle(f, (40, 400), (600, 430), BLUE, -1)
    return f


def cmds(action):
    return dict(action.commands)


class FakeFlag:
    def __init__(self, error_x=0.0, distance_cm=100.0, score=0.8):
        self.error_x, self.distance_cm, self.score = error_x, distance_cm, score
        self.t = time.monotonic()


# ============================================================ DETECTOR
def test_detector_sintetico():
    if not CV:
        print("       (sin cv2 en este PC: se omite)")
        return
    from vision import Vision
    v = Vision("blue")
    assert v.available and v.backend == "app_peripherals.camera"
    fl = v.process(make_frame("flag_center"))
    assert fl and abs(fl.error_x) < 0.05 and abs(fl.h_px - 120) <= 6, "bandera centrada (px del frame completo)"
    assert fl.distance_cm and abs(fl.distance_cm - config.FOCAL_PX * 15 / fl.h_px) < 0.1
    fl = v.process(make_frame("flag_right"))
    assert fl and fl.error_x > 0.5, "bandera a la derecha -> error positivo"
    assert v.process(make_frame("tape_only")) is None, "la CINTA azul del suelo NO es bandera"
    assert v.process(make_frame("own_flag")) is None, "la bandera propia (roja) NO"
    assert v.process(make_frame("ceiling")) is None, "mancha en el cuarto superior NO"
    fl = v.process(make_frame("flag_and_tape"))
    assert fl and abs(fl.h_px - 120) <= 6, "con cinta y bandera a la vez, elige la bandera"
    ok("detector: centrada, desplazada, rechaza cinta/propia/techo, elige bien")


def test_foto_bajo_demanda():
    """Sin video en vivo: el lazo NO codifica JPEG; la foto se genera solo
    cuando se pide, y sobre una copia (el lazo puede estar usando el frame)."""
    if not CV:
        return
    from vision import Vision
    v = Vision("blue")
    assert v.snapshot_jpeg() is None, "sin frames todavia, no hay foto"
    frame = make_frame("flag_center")
    v.process(frame)
    original = frame.copy()
    jpg = v.snapshot_jpeg()
    assert jpg and len(jpg) > 1000, "la foto se genera bajo demanda"
    assert (frame == original).all(), "dibujar la foto NO debe tocar el frame del lazo"
    ok("detector: centrada, desplazada, rechaza cinta/propia/techo, elige bien")


def test_hilo_de_vision_entrega_flag():
    if not CV:
        return
    from vision import Vision
    FakeCamera.scene = "flag_center"
    v = Vision("blue")
    v.start()
    time.sleep(0.4)
    fl = v.latest_flag()
    assert fl is not None and fl.age < config.DETECTION_STALE_S
    st = v.stats()
    assert st["frames"] >= 3 and st["available"]
    FakeCamera.scene = "tape_only"
    time.sleep(0.9)
    assert v.latest_flag() is None, "sin bandera en escena, la deteccion caduca"
    v.stop()
    ok("hilo de vision: captura, detecta y la deteccion caduca al desaparecer")


# ============================================================== DIRECCION
def test_steer():
    assert steer.steer_from_error(0.0) == 0.0
    assert steer.steer_from_error(config.STEER_DEADBAND * 0.9) == 0.0, "zona muerta"
    assert steer.steer_from_error(0.5) > 0 and steer.steer_from_error(-0.5) < 0
    assert abs(steer.steer_from_error(1.0)) <= config.TURN_MAX
    assert steer.steer_from_error(0.8) > steer.steer_from_error(0.3), "monotona"
    assert steer.approach_speed(200) == config.SPEED_APPROACH
    assert steer.approach_speed(config.GRAB_DISTANCE_CM) == 0
    mid = steer.approach_speed((config.GRAB_DISTANCE_CM + config.APPROACH_SLOW_CM) / 2)
    assert config.SPEED_CREEP < mid < config.SPEED_APPROACH
    ok("direccion: zona muerta, signo, saturacion; perfil de velocidad")


# =================================================================== FSM
def test_cerrojo_del_reglamento():
    m = Mission()
    m.start_patrol(sense())
    a = m.step(sense(), 0.05, FakeFlag())
    assert m.state == S.PATROL, "con la llave a bordo NO se persigue"
    assert a.throttle == config.PATROL_SPEED
    assert cmds(a).get("flagled") == 1, "pero SI se senaliza: LED parpadeando"
    a = m.step(sense(), 0.05, FakeFlag())
    assert "flagled" not in cmds(a), "el LED se manda solo cuando cambia"
    m.key_deposited = True
    m.step(sense(), 0.05, FakeFlag())
    assert m.state == S.APPROACH, "con la llave depositada, a por ella"
    ok("cerrojo del reglamento: ver si, perseguir solo con la llave depositada")


def test_override_para_banco():
    old = config.FLAG_REQUIRES_KEY
    try:
        config.FLAG_REQUIRES_KEY = False
        m = Mission()
        m.start_patrol(sense())
        m.step(sense(), 0.05, FakeFlag())
        assert m.state == S.APPROACH
    finally:
        config.FLAG_REQUIRES_KEY = old
    ok("FLAG_REQUIRES_KEY = False permite probar la persecucion en el banco")


def test_persecucion_y_llegada():
    m = Mission()
    m.key_deposited = True
    m.start_patrol(sense())
    m.step(sense(), 0.05, FakeFlag(error_x=0.6, distance_cm=120))
    assert m.state == S.APPROACH
    a = m.step(sense(), 0.05, FakeFlag(error_x=0.6, distance_cm=120))
    assert a.turn > 0, "bandera a la derecha -> girar a la derecha"
    assert 0 < a.throttle < config.SPEED_APPROACH, "avanza menos cuanto mas descentrado"
    a = m.step(sense(), 0.05, FakeFlag(error_x=-0.6, distance_cm=120))
    assert a.turn < 0
    a = m.step(sense(), 0.05, FakeFlag(error_x=0.0, distance_cm=120))
    assert a.turn == 0 and a.throttle == config.SPEED_APPROACH, "centrada y lejos: recto a tope"
    a = m.step(sense(), 0.05, FakeFlag(error_x=0.0, distance_cm=30))
    assert 0 < a.throttle < config.SPEED_APPROACH, "cerca: frena"
    a = m.step(sense(), 0.05, FakeFlag(error_x=0.0, distance_cm=config.GRAB_DISTANCE_CM - 1))
    assert m.state == S.FLAG_REACHED
    assert cmds(a).get("flagled") == 2, "LED fijo en el mismo ciclo de la llegada"
    a = m.step(sense(), 0.05, FakeFlag(error_x=0.0, distance_cm=10))
    assert a.throttle == 0 and a.turn == 0 and not a.commands, "parado, sin repetir el LED"
    ok("persecucion: giro hacia la bandera, frena al acercarse, llega y se para")


def test_bandera_perdida():
    m = Mission()
    m.key_deposited = True
    m.start_patrol(sense())
    m.step(sense(), 0.05, FakeFlag())
    assert m.state == S.APPROACH
    a = m.step(sense(), 0.05, None)
    assert m.state == S.APPROACH and a.throttle == 0, "sin verla: espera parado"
    assert "flagled" not in cmds(a), "un frame sin bandera NO apaga el LED: se la sigue"
    m._last_flag_seen -= config.LOST_TARGET_S + 0.1
    a = m.step(sense(), 0.05, None)
    assert m.state == S.PATROL, "perdida demasiado tiempo: vuelve a patrullar"
    assert cmds(a).get("flagled") == 0, "LED apagado al perderla"
    ok("bandera perdida: espera LOST_TARGET_S y vuelve a patrullar")


def test_prioridades_en_persecucion():
    m = Mission()
    m.key_deposited = True
    m.start_patrol(sense())
    m.step(sense(), 0.05, FakeFlag())
    m.step(sense(black_mask=0b0001), 0.05, FakeFlag())
    assert m.state == S.EDGE, "el borde negro manda sobre la bandera"

    # El amarillo YA NO manda sobre la bandera (AVOID_YELLOW_AFTER_DROP =
    # False, 18/09): rebotar en la zona central cortaba la persecucion justo
    # cuando el rover iba a por una bandera al otro lado.
    m2 = Mission()
    m2.key_deposited = True
    m2.start_patrol(sense())
    m2.step(sense(), 0.05, FakeFlag())
    m2.step(sense(color_stable=config.C_YELLOW), 0.05, FakeFlag())
    assert m2.state == S.APPROACH, "cruzar el amarillo ya no interrumpe la persecucion"

    prev = config.AVOID_YELLOW_AFTER_DROP
    try:
        config.AVOID_YELLOW_AFTER_DROP = True
        m3 = Mission()
        m3.key_deposited = True
        m3.start_patrol(sense())
        m3.step(sense(), 0.05, FakeFlag())
        m3.step(sense(color_stable=config.C_YELLOW), 0.05, FakeFlag())
        assert m3.state == S.AVOID_ZONE, "con True vuelve a mandar la caja"
    finally:
        config.AVOID_YELLOW_AFTER_DROP = prev
    ok("prioridades: el borde negro manda; el amarillo ya no (interruptor)")


def test_main_con_camara_falsa():
    if not CV:
        return
    FakeCamera.scene = "flag_enemy"          # del color del RIVAL configurado
    import main
    m = main
    m.config.CONTROL_DT = 0.0
    m.config.DIAG_MCU_S = 0
    assert m.vision.available
    time.sleep(0.5)
    snap = m.snapshot()
    assert snap["flag"] is not None and "cx_px" in snap["flag"]
    assert snap["vision"]["frames"] > 0
    import json
    json.dumps(snap)
    # la deteccion llega a la FSM y el LED sale por el Bridge
    m.mission.key_deposited = False
    m.ui._start()
    for _ in range(3):
        m.control_loop()
    log = t1._utils.Bridge.log
    assert ("flagled", (1,)) in log, "LED parpadeo mandado al MCU al ver la bandera"
    assert m.mission.state == S.PATROL, "sin llave depositada no persigue"
    m.ui._estop()
    m.vision.stop()
    ok("main.py: camara falsa -> deteccion -> FSM -> LED por el Bridge; telemetria JSON")



# ================================================== PUNTUACION Y RECORTE
def test_puntuacion_bandera_recortada():
    """EL FALLO QUE MOTIVO ESTA FORMULA.

    En el banco, con la bandera cerca, la mascara salia perfecta (162x226 px,
    relleno 0.97) y la deteccion se rechazaba con puntuacion 0.36. La causa no
    eran los umbrales de color: era la formula, que castigaba el aspecto con
    el error ABSOLUTO  1/(1+|aspecto-3|).  Con aspecto 1.40 eso da 0.38 y
    hunde el producto.

    Y el aspecto valia 1.40 porque la bandera estaba CORTADA por el borde
    inferior (ocupaba y=250..476 de 480): su alto no cabia en la imagen, asi
    que ese 1.40 no medía la forma de nada.
    """
    from vision import score_flag
    fill, aspect, area, norm = 0.97, 1.40, 4000.0, 4000.0

    # 0.5 era el umbral con el que se observo el fallo en el banco; el
    # mensaje decia literalmente "puntuacion 0.36 < 0.5".
    viejo = fill * (1.0 / (1.0 + abs(aspect - 3.0))) * 1.0
    assert abs(viejo - 0.37) < 0.02, f"reproduce el 0.36 del banco: {viejo:.2f}"
    assert viejo < 0.5, "por eso se rechazaba con el umbral de entonces"
    # OJO: con MIN_CONFIDENCE = 0.35 la formula vieja se habria colado por
    # 0.02. Bajar el umbral tapa este caso pero no arregla la formula, y
    # ademas abarata TODAS las demas detecciones, cinta del suelo incluida.
    assert config.MIN_CONFIDENCE < viejo < 0.5, (
        "margen actual sobre la formula vieja: "
        f"{viejo - config.MIN_CONFIDENCE:.2f}. Es minusculo, y es la razon "
        "por la que el arreglo tiene que estar en la formula, no en el umbral")

    s_rec, partes = score_flag(fill, aspect, area, norm, clipped=True)
    assert s_rec > 0.9, f"recortada: el aspecto no se penaliza ({s_rec:.2f})"
    assert partes[1] == 1.0, "f_aspecto = 1.0 cuando esta recortada"

    s_ent, partes = score_flag(fill, aspect, area, norm, clipped=False)
    assert config.MIN_CONFIDENCE < s_ent < s_rec, (
        f"entera: se penaliza pero con error RELATIVO, no absoluto ({s_ent:.2f})")
    assert s_ent > viejo, "el error relativo castiga menos que el absoluto"
    ok(f"puntuacion: bandera recortada {viejo:.2f} (vieja) -> {s_rec:.2f} (nueva)")


def test_recorte_se_detecta_en_el_frame():
    """La marca de recortada sale del detector, no se pone a mano."""
    if not CV:
        return
    from vision import Vision
    v = Vision("blue")
    entera = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    cv2.rectangle(entera, (300, 200), (340, 320), BLUE, -1)
    fl = v.process(entera)
    assert fl and not fl.parts[3], "bandera con margen: NO recortada"

    cortada = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    cv2.rectangle(cortada, (250, 250), (412, 479), BLUE, -1)   # llega al borde
    fl = v.process(cortada)
    assert fl, "la bandera cortada SE DETECTA (era el fallo del banco)"
    assert fl.parts[3], "y se marca como recortada"
    assert fl.score > 0.5, f"con buena puntuacion, no 0.36 ({fl.score:.2f})"
    ok("recorte: se detecta solo al tocar el borde, y salva la deteccion")


def test_el_ancho_es_lo_unico_que_frena_la_cinta():
    """AVISO CONVERTIDO EN PRUEBA.

    Con FLAG_MIN_ASPECT = 0.1 (medido en el banco) el filtro de aspecto ya no
    rechaza NADA por ancho: una cinta del suelo lo pasa. Lo unico que la
    separa de una bandera es FLAG_MAX_WIDTH_FRAC. Esta prueba lo deja escrito
    con numeros: si alguien sube ese valor sin pensarlo, aqui se entera.
    """
    if not CV:
        return
    from vision import Vision
    v = Vision("blue")
    limite = config.FLAG_MAX_WIDTH_FRAC * 640

    estrecha = int(limite * 0.8)
    f = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    cv2.rectangle(f, (60, 380), (60 + estrecha, 440), BLUE, -1)
    fl = v.process(f)
    assert fl is not None, (
        f"con min_aspect={config.FLAG_MIN_ASPECT} una cinta de {estrecha}px "
        f"({estrecha/640:.2f} del ancho) PASA por bandera. Es el agujero "
        f"conocido: el unico filtro que queda es el ancho.")

    ancha = int(limite * 1.2)
    f = np.full((480, 640, 3), (235, 235, 235), np.uint8)
    cv2.rectangle(f, (20, 380), (20 + ancha, 440), BLUE, -1)
    assert v.process(f) is None, "por encima del limite de ancho, se rechaza"
    ok(f"cinta: el unico filtro que la para es el ancho "
       f"(limite {config.FLAG_MAX_WIDTH_FRAC} = {int(limite)} px de 640)")


# ================================================== SALTO DE ESTADO
def test_salto_de_estado_deja_la_fsm_coherente():
    """Cada estado forzado tiene que quedar en su PRIMERA fase, no en una
    intermedia. El caso que lo motiva: DEPOSIT_KEY con _key_phase = None cae
    por la rama 'drop', ve el plazo vencido y se va a BACKOFF_KEY sin soltar
    nada -- el boton 'probar deposito' no probaria el deposito."""
    m = Mission()
    s = sense()

    okk, why = m.force_state(S.DEPOSIT_KEY, s)
    assert okk, why
    assert m.state == S.DEPOSIT_KEY and m._key_phase == "enter", \
        f"DEPOSIT_KEY debe entrar por 'enter', no por '{m._key_phase}'"
    a = m.step(s, config.CONTROL_DT)
    assert m.state == S.DEPOSIT_KEY, "no salta a BACKOFF sin haber soltado"
    assert a.throttle == 0, (
        "con ZONE_ENTER_ADVANCE_S = 0 el rover se para en seco al ver el "
        "amarillo; antes avanzaba y se pasaba la zona entera")

    for st, fase in ((S.BACKOFF_KEY, "back"), (S.AVOID_ZONE, "back")):
        assert m.force_state(st, s)[0]
        assert m.state == st and m._key_phase == fase, f"{st} -> fase {fase}"
        assert m._phase_until > time.monotonic(), f"{st}: plazo en el futuro"

    assert m.force_state(S.EDGE, s)[0]
    assert m.state == S.EDGE and m._edge_phase in ("back", "turn")
    assert m._edge_until > time.monotonic()
    assert m._key_phase is None, "una fase de llave vieja no sobrevive al salto"

    assert m.force_state(S.APPROACH, s)[0]
    a = m.step(s, config.CONTROL_DT, flag=None)
    assert m.state == S.APPROACH, ("APPROACH forzado no se cae a PATROL en el "
                                   "primer ciclo sin bandera")

    assert m.force_state(S.IDLE, s)[0] and m.state == S.IDLE
    ok("salto de estado: cada estado forzado entra por su primera fase")


def test_salto_de_estado_respeta_los_cerrojos():
    m = Mission()

    ciego = sense()
    ciego.flags &= ~0x1000            # TCRT frontales no montados
    for st in (S.PATROL, S.EDGE, S.DEPOSIT_KEY, S.APPROACH):
        okk, why = m.force_state(st, ciego)
        assert not okk and "TCRT" in why, f"{st} sin TCRT frontales: {okk} {why}"
    okk, _ = m.force_state(S.IDLE, ciego)
    assert okk, "IDLE no mueve nada: no exige sensores"
    okk, _ = m.force_state(S.FLAG_REACHED, ciego)
    assert okk, "FLAG_REACHED esta parado: tampoco los exige"

    viejo = sense()
    viejo.flags |= 0x001               # watchdog del MCU disparado
    okk, why = m.force_state(S.PATROL, viejo)
    assert not okk and "telemetria" in why, why

    m2 = Mission()
    m2.estop("prueba")
    okk, why = m2.force_state(S.PATROL, sense())
    assert not okk and "emergencia" in why, "desde ESTOP no se fuerza nada"
    m2.clear_estop()
    assert m2.force_state(S.PATROL, sense())[0], "tras rearmar, si"

    okk, why = m2.force_state("VOLAR", sense())
    assert not okk and "desconocido" in why
    ok("salto de estado: los que mueven exigen TCRT + telemetria; ESTOP manda")


def test_la_pagina_ofrece_exactamente_los_estados_forzables():
    """La lista de botones sale de la FSM, no de una copia en el HTML."""
    from ui import mission_states
    m = Mission()
    d = m.as_dict()
    assert d["forceable"] == list(Mission.FORCEABLE) == list(mission_states())
    assert S.ESTOP not in d["forceable"], (
        "ESTOP no se 'salta': tiene su propio boton y su propio rearme")
    for st in d["moving"]:
        assert st in d["forceable"], "todo estado que mueve debe ser forzable"
    assert S.IDLE not in d["moving"] and S.FLAG_REACHED not in d["moving"]
    ok("la pagina dibuja un boton por estado, con la lista que da la FSM")


# ==================================================================== LED RGB
def test_el_led_rgb_sale_de_TEAM_y_llega_al_mcu():
    """UN SOLO sitio decide el equipo. TEAM_LED, ENEMY y TEAM_COLOR salen
    todos de config.TEAM: si alguien pudiera fijarlos por separado, el rover
    podria perseguir una bandera y encenderse del color contrario."""
    assert config.TEAM in ("red", "blue")
    assert config.TEAM_LED == (1 if config.TEAM == "blue" else 0)
    assert config.ENEMY != config.TEAM
    esperado = config.C_BLUE if config.TEAM == "blue" else config.C_RED
    assert config.TEAM_COLOR == esperado, (
        "el color de casa y el del LED tienen que ser el mismo equipo")

    from protocol import RoverLink
    link = RoverLink.__new__(RoverLink)
    llamadas = []
    link._call = lambda *a: (llamadas.append(a), True)[1]
    assert link.set_team_led(config.TEAM_LED)
    assert llamadas[-1] == ("team", config.TEAM_LED)
    assert link.led_test(1, 0, 0, 1500)
    assert llamadas[-1] == ("ledtest", 1, 0, 0, 1500)
    ok(f"LED RGB: TEAM = {config.TEAM} -> tono {config.TEAM_LED}, "
       f"y la orden llega al MCU")


def test_el_ritmo_del_led_no_toca_el_tono():
    """El LED RGB hace DOS trabajos: el tono dice el equipo y el ritmo dice
    que pasa con la bandera. La FSM solo puede tocar el ritmo -- si pudiera
    cambiar el tono, el rover pareceria del otro equipo justo en el momento
    mas visible de la ronda."""
    m = Mission()
    m.key_deposited = True
    m.start_patrol(sense())
    modos, todas = [], []
    for fl in [None, FakeFlag()] + [FakeFlag(distance_cm=5)] * 4:
        a = m.step(sense(), 0.05, fl) if fl else m.step(sense(), 0.05)
        modos += [v for n, v in a.commands if n == "flagled"]
        todas += [n for n, _ in a.commands]
    assert modos, "la FSM manda el ritmo"
    assert all(v in (0, 1, 2) for v in modos), modos
    assert "team" not in todas, "la FSM nunca toca el tono"
    ok("la FSM manda el RITMO del LED, nunca el tono (el equipo no cambia)")


if __name__ == "__main__":
    print("ROVER H07 - demo parte 3 (camara y bandera) - pruebas\n")
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
