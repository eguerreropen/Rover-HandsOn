"""
ROVER H07 - DEMO PARTE 1 - App del MPU (Arduino UNO Q, Linux/Python)
=====================================================================

Esta parte: identificar el color de las lineas (negro/amarillo/azul/rojo) y
no salirse de la pista. La pagina web muestra lo que leen los sensores.

Reparto:
    MPU (aqui)   estados del rover, clasificacion del color, pagina web
    MCU (sketch) TCRT, APDS crudo, motores y el reflejo de borde (<10 ms)

Lazo de control a CONTROL_HZ:
    1. ultima telemetria del MCU (la trae un hilo aparte; no bloquea)
    2. clasificar el color y filtrar estabilidad
    3. FSM -> accion (avance, giro)
    4. mezclar a ruedas + rampa
    5. drive() al MCU  <- ademas alimenta su watchdog
"""

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Salida linea a linea: la App no corre en una terminal y sin esto los print()
# aparecen a rafagas de 8 KB, inutiles para calibrar en vivo.
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except (AttributeError, ValueError):
    pass

from arduino.app_utils import App          # noqa: E402

import config                              # noqa: E402
from colors import ColorTracker            # noqa: E402
from mission import Mission, S             # noqa: E402
from protocol import PressCounter, RoverLink   # noqa: E402
from ui import RoverUI                     # noqa: E402
from vision import Vision                  # noqa: E402

link = RoverLink()
mission = Mission()
tracker = ColorTracker()
vision = Vision(config.ENEMY)
ui = RoverUI(mission, link, tracker, vision)
team_button = PressCounter()       # boton de equipo (A4, lo cuenta el MCU)
_reboots_seen = 0                  # para reenviar el tono del LED si el MCU se reinicia

_last_tick = time.monotonic()
_overruns = 0
_last_diag = 0.0
_slew = [0, 0]

# Un cambio de color es un EVENTO, no un valor continuo: no tiene por que
# esperar al siguiente refresco periodico para dibujarse. Esta senal despierta
# al hilo de telemetria en cuanto ocurre.
#
# Se hace con una senal y no llamando a ui.push() desde el lazo de control a
# proposito: el envio a la pagina podria bloquear, y el lazo de control es
# justo lo que NO puede bloquearse (si se calla 400 ms, el MCU para los
# motores). El lazo marca; el hilo de telemetria envia.
_push_now = threading.Event()


# ------------------------------------------------------------ hilos auxiliares
def sense_poller():
    """sense() es sincrona: va en su hilo para no acoplar el lazo de control a
    la latencia del Bridge."""
    period = 1.0 / config.SENSE_POLL_HZ
    while True:
        link.poll()
        time.sleep(period)


def telemetry_pusher():
    period = 1.0 / config.UI_PUSH_HZ
    while True:
        ui.push(snapshot())
        _push_now.wait(period)     # despierta antes si hubo un evento
        _push_now.clear()


def snapshot():
    s = link.sense()
    flag = vision.latest_flag()
    return {
        "flag": flag.as_dict() if flag else None,
        "vision": vision.stats(),
        "mcu": s.as_dict(),
        "fsm": mission.as_dict(),
        "team": {"team": config.TEAM, "enemy": config.ENEMY,
                 "can_change": mission.can_change_team},
        "key": {"deposited": mission.key_deposited,
                "angle": s.key_angle,
                "drop": [config.KEY_DROP_ANGLE, config.KEY_DROP_S],
                "retract": [config.KEY_RETRACT_ANGLE, config.KEY_RETRACT_S],
                "powered": s.key_powered,
                "moving": s.servo_moving},
        "calib": tracker.calib_dict(s),
        "events": tracker.events[-12:],
        "cmd": {"left": link.last_command[0], "right": link.last_command[1]},
        "diag": {
            "mpu_hz": round(link.mpu_drive_hz, 1),
            "mcu_hz": round(link.mcu_drive_hz, 1),
            "lost": link.mcu_drives_lost,
            "loop_max_ms": link.mcu_loop_max_ms,
            "slow_passes": link.mcu_slow_passes,
            "max_drive_gap_ms": round(link.max_drive_gap_ms),
            "mcu_drive_gap_ms": link.mcu_max_drive_gap_ms,
            "future_marks": link.mcu_future_marks,
            "reboots": link.mcu_reboots,
            "errors": link.error_count,
            "overruns": _overruns,
        },
        "thresholds": {"black_below": config.LINE_BLACK_BELOW,
                       "dark_clear": config.COLOR_DARK_CLEAR,
                       "bright_clear": config.COLOR_BRIGHT_CLEAR,
                       "steer_deadband": config.STEER_DEADBAND},
    }


# ------------------------------------------------------- ejecucion de ordenes
SERVO_IDS = {"key": config.SERVO_KEY,
             "grip": config.SERVO_GRIP,
             "lift": config.SERVO_LIFT}


def run_commands(commands):
    """Traduce las ordenes de la FSM a llamadas al MCU. Es el UNICO sitio
    donde una orden de la mision se convierte en un RPC."""
    for name, value in commands:
        # Tabla y no una cadena de elif: con tres servos y sus tres apagados
        # eran ocho ramas, y anadir un servo significaba acordarse de dos
        # sitios. Aqui olvidarse es imposible.
        if name == "key_spin":
            # Servo CONTINUO de la caja: (angulo, segundos). Gira y el MCU lo
            # para solo -- no hay "key" a secas, porque un servo continuo
            # mandado a un angulo se quedaria girando sin fin.
            deg, secs = value
            link.spin(config.SERVO_KEY, int(deg), int(round(secs * 1000)))
        elif name == "key":
            print("[main] ORDEN RECHAZADA: ('key', angulo) dejaria el servo "
                  "continuo de la caja girando sin fin. Usa ('key_spin', ...).")
        elif name in SERVO_IDS:
            link.servo(SERVO_IDS[name], int(value))
        elif name.endswith("_off") and name[:-4] in SERVO_IDS:
            link.servo_off(SERVO_IDS[name[:-4]])
        elif name == "flagled":
            link.set_flag_led(int(value))
        else:
            print(f"[main] orden desconocida: {name}={value}")


# ------------------------------------------------------------ equipo (A4)
def toggle_team(presses=1):
    """Alterna rojo <-> azul, SOLO fuera de ronda (IDLE, DONE, ESTOP).

    En plena ronda se ignora: cambiaria que bandera persigue y que cinta es
    casa a mitad de camino. Si se pulsa n veces entre dos lecturas, cuenta la
    paridad (dos pulsaciones = mismo equipo), igual que si se hubieran visto
    una a una."""
    if not mission.can_change_team:
        print(f"[equipo] boton A4 IGNORADO: en ronda ({mission.state}). "
              f"Sigue {config.TEAM.upper()}.")
        return False
    if presses % 2:
        config.set_team("red" if config.TEAM == "blue" else "blue")
        vision.set_enemy(config.ENEMY)
    ok = link.set_team_led(config.TEAM_LED)
    print(f"[equipo] AHORA {config.TEAM.upper()}: persigue la bandera "
          f"{config.ENEMY.upper()}, casa = cinta "
          f"{config.COLOR_NAMES[config.TEAM_COLOR]}"
          f"{'' if ok else '   (AVISO: el MCU no acepto el color del LED)'}")
    _push_now.set()
    return True


def resend_team_led_after_reboot():
    """Si el MCU se reinicia (caida de tension) pierde el tono del LED y
    vuelve a su valor por defecto, pero Python sigue en el equipo elegido
    con el boton. Se reenvia para que el LED no mienta."""
    global _reboots_seen
    if link.mcu_reboots != _reboots_seen:
        _reboots_seen = link.mcu_reboots
        link.set_team_led(config.TEAM_LED)
        print(f"[equipo] MCU reiniciado: LED otra vez en {config.TEAM.upper()}")


# --------------------------------------------------------------- lazo control
def mix(throttle, turn):
    """(avance, giro) -> (izq, der). Al saturar se recorta el avance, nunca el
    giro: un rover que no puede girar se sale de la pista."""
    left, right = throttle + turn, throttle - turn
    over = max(abs(left), abs(right)) - 1000
    if over > 0:
        cut = min(abs(throttle), over)
        throttle -= cut if throttle > 0 else -cut
        left, right = throttle + turn, throttle - turn
    return int(max(-1000, min(1000, left))), int(max(-1000, min(1000, right)))


def slew(left, right):
    for i, tgt in enumerate((left, right)):
        cur = _slew[i]
        if tgt > cur:
            _slew[i] = min(tgt, cur + config.SLEW_PER_TICK)
        elif tgt < cur:
            _slew[i] = max(tgt, cur - config.SLEW_PER_TICK)
    return _slew[0], _slew[1]


def diag(sense, now):
    """
    Una linea por segundo con lo MEDIDO en los dos extremos del enlace.

        mpu->mcu   ordenes/s enviadas frente a recibidas
        hueco      intervalo mas largo entre dos drive() ENVIADOS (lado MPU)
        recib      intervalo mas largo entre dos drive() RECIBIDOS (lado MCU)
                   Los dos juntos son el discriminador principal:
                     hueco alto              -> se callo el MPU
                     hueco bajo, recib alto  -> el transporte las AGRUPA
                     los dos bajos           -> ni uno ni otro
        loop       pasada mas lenta del MCU en ESTA ventana (maximo historico
                   entre parentesis) y cuantas se han pasado de LOOP_SLOW_MS.
                   Un maximo suelto no distingue "una vez" de "siempre"; el
                   contador si.
        lazoMPU    ciclos del lazo de control que se fueron de tiempo.
    """
    global _last_diag
    if not config.DIAG_MCU_S or now - _last_diag < config.DIAG_MCU_S:
        return
    _last_diag = now
    print(f"[mcu] mpu->mcu {link.mpu_drive_hz:4.1f}->{link.mcu_drive_hz:4.1f} Hz  "
          f"perdidos {link.mcu_drives_lost}  "
          f"hueco {link.max_drive_gap_ms:.0f} ms  "
          f"recib {link.mcu_max_drive_gap_ms} ms  "
          f"loop {sense.loop_max_ms}/{link.mcu_loop_max_ms} ms x{link.mcu_slow_passes}  "
          f"lazoMPU {_overruns}  fut {link.mcu_future_marks}  "
          f"wd={int(sense.watchdog)} reinicios={link.mcu_reboots} "
          f"errores={link.error_count}")


_wd_seen = False


def diag_watchdog(sense):
    """
    Cuando salta el watchdog del MCU, el log tiene que decir POR QUE. El
    veredicto se emite por orden de evidencia: primero lo medido.
    """
    global _wd_seen
    if not sense.watchdog:
        _wd_seen = False
        return
    if _wd_seen:
        return
    _wd_seen = True
    print("[diag] WATCHDOG DEL MCU. Lo que dicen las medidas:")
    print(f"[diag]   drive() ENVIADOS: hueco max {link.max_drive_gap_ms:.0f} ms")
    print(f"[diag]   drive() RECIBIDOS: hueco max "
          f"{link.mcu_max_drive_gap_ms} ms   (el watchdog salta a "
          f"{config.MCU_WATCHDOG_MS})")
    print(f"[diag]   lazo del MCU: {link.mcu_loop_max_ms} ms el peor, "
          f"{link.mcu_slow_passes} pasadas lentas")
    print(f"[diag]   ordenes enviadas/recibidas: {link.drive_calls}/"
          f"{link.mcu_drives}  (perdidas ~{link.mcu_drives_lost})")
    print(f"[diag]   ciclos lentos del MPU: {_overruns}   "
          f"errores de bridge: {link.error_count}   "
          f"reinicios del MCU: {link.mcu_reboots}")
    print(f"[diag]   marcas de tiempo 'del futuro' en el MCU: "
          f"{link.mcu_future_marks}")
    if link.mcu_future_marks:
        print("[diag]   -> RESTA DE TIEMPOS DESBORDADA (bug corregido en el")
        print("[diag]      sketch). Un handler del Bridge escribio su marca")
        print("[diag]      despues de que loop() tomara su 'now', y la resta")
        print("[diag]      sin signo dio ~4.29e9 ms. Si este contador sube y")
        print("[diag]      el watchdog ya NO salta, esa era la causa.")
    elif link.mcu_reboots:
        print("[diag]   -> EL MCU SE REINICIO: caida de tension.")
    elif link.max_drive_gap_ms >= config.MCU_WATCHDOG_MS:
        print("[diag]   -> SE CALLO EL MPU. El lazo de Python se quedo sin")
        print("[diag]      mandar ordenes ese tiempo; el MCU hizo lo correcto.")
        print("[diag]      Mira los ciclos lentos y que mas corre en la placa.")
    elif link.mcu_loop_max_ms >= config.MCU_WATCHDOG_MS:
        print("[diag]   -> SE BLOQUEO EL LAZO DEL MCU. Los handlers del Bridge")
        print("[diag]      se despachan ahi, asi que las ordenes esperaron.")
        print("[diag]      Sospechoso numero uno: una lectura I2C del APDS.")
    elif link.mcu_max_drive_gap_ms >= config.MCU_WATCHDOG_MS:
        print("[diag]   -> LAS ORDENES LLEGAN EN RAFAGAS. El MPU las manda")
        print(f"[diag]      cada {link.max_drive_gap_ms:.0f} ms como mucho, pero"
              f" al MCU le llegan")
        print(f"[diag]      agrupadas cada {link.mcu_max_drive_gap_ms} ms. El"
              f" transporte las acumula;")
        print("[diag]      entre rafaga y rafaga el MCU se queda sin ordenes.")
        print("[diag]      ARREGLO: DRIVE_USE_CALL = True en config.py, que")
        print("[diag]      impone contrapresion y elimina el agrupamiento.")
    elif link.mcu_drive_hz < link.mpu_drive_hz * 0.7:
        print("[diag]   -> SE PIERDEN TRAMAS en el transporte.")
    else:
        print("[diag]   -> ningun extremo se paso del limite por si solo:")
        print("[diag]      probablemente una rafaga corta de tramas perdidas.")


def control_loop():
    global _last_tick, _overruns
    now = time.monotonic()
    dt = now - _last_tick
    _last_tick = now
    if dt > config.CONTROL_DT * 2.5:
        _overruns += 1
        if _overruns <= 5 or _overruns % 100 == 0:
            print(f"[loop] ciclo lento: {dt*1000:.0f} ms (#{_overruns})")

    sense = link.sense()
    if tracker.apply_to(sense, now):
        _push_now.set()            # el color cambio: a la pagina, ya
    diag(sense, now)

    diag_watchdog(sense)

    # Boton de equipo (A4). El MCU cuenta las pulsaciones con antirrebote;
    # aqui solo se mira si hay alguna NUEVA. Cada pulsacion alterna.
    n = team_button.new_presses(sense.btn_presses)
    if n:
        toggle_team(n)
    resend_team_led_after_reboot()

    flag = vision.latest_flag()
    action = mission.step(sense, dt, flag)
    if action.commands:
        run_commands(action.commands)
    left, right = mix(action.throttle, action.turn)
    if mission.state in (S.IDLE, S.ESTOP):
        _slew[0] = _slew[1] = 0
        left = right = 0
    left, right = slew(left, right)

    link.drive(left, right)      # SIEMPRE, tambien (0,0): alimenta el watchdog

    time.sleep(max(0.0, config.CONTROL_DT - (time.monotonic() - now)))


# -------------------------------------------------------------------- arranque
def startup():
    print("=" * 62)
    print("ROVER H07 - DEMO PARTE 3: lineas + borde + llave + bandera (camara)")
    print("=" * 62)

    # ---- SONDEO DE VIDA DEL SKETCH, lo PRIMERO -------------------------
    # Localiza el fallo sin depender del monitor serie (que en App Lab sobre
    # Windows puede no mostrar nada). Ver rpc_alive() en el sketch.
    ETAPAS = {0: "no arranco", 1: "Bridge", 2: "motores", 3: "TCRT",
              4: "APDS (I2C)", 5: "servo de la llave", 6: "LED (setup completo)",
              7: "loop() corriendo"}
    stage = link.alive()
    if stage is None:
        print("[init] " + "=" * 58)
        print("[init] EL SKETCH NO CONTESTA NI AL SONDEO BASICO (alive).")
        print("[init] Ese RPC lo atiende el hilo del Bridge, no el lazo del")
        print("[init] sketch, asi que contestaria incluso con setup() colgado.")
        print("[init] Que no conteste significa que el SKETCH NO ESTA CORRIENDO:")
        print("[init]   -> no compilo, o no se subio al MCU.")
        print("[init]   -> MIRA LA PESTANA 'App launch' de App Lab: ahi sale la")
        print("[init]      compilacion del sketch, y ahi estara el error.")
        print("[init] " + "=" * 58)
    elif stage < 7:
        print("[init] " + "=" * 58)
        print(f"[init] EL SKETCH ARRANCO PERO SE QUEDO EN LA ETAPA {stage}/7: "
              f"{ETAPAS.get(stage, '?')}")
        print("[init] Lo siguiente que iba a hacer es lo que se colgo:")
        print(f"[init]   -> {ETAPAS.get(stage + 1, '?')}")
        if stage == 3:
            print("[init] Etapa 4 es el APDS: un I2C atascado bloquea para")
            print("[init] siempre. Revisa SDA/SCL (D20/D21), 3V3 y masa.")
        print("[init] " + "=" * 58)
    else:
        print("[init] sketch vivo y loop() corriendo (etapa 7/7)")

    link.stop()

    # El umbral de negro vive en config.py: se manda al MCU al arrancar. Se
    # reintenta porque el sketch puede tardar unos segundos mas que la App.
    for intento in range(10):
        if link.configure_lines(config.LINE_BLACK_BELOW, config.LINE_HYSTERESIS):
            print(f"[init] umbral de negro {config.LINE_BLACK_BELOW} "
                  f"(hist. {config.LINE_HYSTERESIS}) enviado al MCU")
            break
        time.sleep(0.5)
    else:
        print("[init] AVISO: el MCU no acepto cfg_lines; usa su valor por defecto.")

    # Servo de la caja: CONTINUO desde el 28/09. NO se le manda ningun angulo
    # al arrancar -- antes se mandaba "retener" (180), que en un servo
    # continuo es "maxima velocidad, sin fin". El MCU ya arranca sin pulsos;
    # este servo_off es por si quedo algo girando de una ejecucion anterior.
    link.servo_off(config.SERVO_KEY)
    print("[init] servo de la caja PARADO (continuo, sin pulsos). Carga la "
          "llave a mano con el brazo en su sitio.")

    # Brazo a posicion de partida: pinza ABIERTA y elevacion ABAJO. Se manda
    # aqui y no en el sketch a proposito -- begin() del MCU no engancha estos
    # dos para que el brazo se pueda colocar a mano sin que un servo de 20 kg
    # empuje. La primera orden es esta, y ya con la App en marcha.
    if (link.servo(config.SERVO_GRIP, config.SERVO_GRIP_OPEN)
            and link.servo(config.SERVO_LIFT, config.SERVO_LIFT_DOWN)):
        print(f"[init] brazo en posicion: pinza abierta ({config.SERVO_GRIP_OPEN}), "
              f"elevacion abajo ({config.SERVO_LIFT_DOWN}).")
    else:
        print("[init] AVISO: el MCU no acepto las ordenes del brazo.")

    # Y SE APAGAN. La puesta a punto es una orden de servo como cualquier otra,
    # asi que sigue la misma regla: mover, esperar a que el eje llegue, cortar
    # los pulsos. Si no, los tres se quedarian zumbando desde el arranque hasta
    # la primera maniobra -- que puede ser toda la espera antes de la ronda.
    apagables = [n for n in ("grip", "lift")
                 if config.SERVO_AUTO_OFF.get(n, False)]
    time.sleep(max(config.settle_s(n) for n in ("grip", "lift")))
    for nombre in apagables:
        link.servo_off(SERVO_IDS[nombre])
    print(f"[init] apagados tras colocarse: {', '.join(apagables) or 'ninguno'}.")

    # Aviso de velocidades demasiado bajas para arrancar. No se recorta nada:
    # un recorte silencioso convierte "va al 75 %" en una mentira y el fallo
    # se busca en el sitio equivocado. Mejor que se vea aqui.
    if config.SPEED_SCALE != 1.0:
        print(f"[init] SPEED_SCALE = {config.SPEED_SCALE}: el rover anda al "
              f"{config.SPEED_SCALE:.0%} y los tiempos de maniobra estan "
              f"compensados para conservar angulos y distancias.")
    flojas = [(n, getattr(config, n)) for n in
              ("PATROL_SPEED", "SPEED_APPROACH", "ESCAPE_SPEED", "CARRY_SPEED",
               "SPEED_CREEP", "SPEED_FINAL", "ESCAPE_TURN", "GRAB_ALIGN_TURN")
              if getattr(config, n) < config.SPEED_MOVE_FLOOR]
    if flojas:
        print(f"[init] AVISO: por debajo de SPEED_MOVE_FLOOR "
              f"({config.SPEED_MOVE_FLOOR}) el rover puede zumbar sin moverse:")
        for n, v in flojas:
            print(f"[init]        {n} = {v}")
        print("[init]        Si se queda plantado, sube ese numero en config.py "
              "(no SPEED_SCALE) o mide SPEED_MOVE_FLOOR de verdad.")

    # Tono del LED RGB = equipo. Se manda una vez: el equipo no cambia a
    # mitad de ronda, y el MCU lo recuerda.
    if link.set_team_led(config.TEAM_LED):
        print(f"[init] LED RGB en {config.TEAM.upper()}. El TONO es el equipo "
              f"y el RITMO la bandera (fijo / lento = a la vista / rapido = "
              f"en su poder).")
    else:
        print("[init] AVISO: el MCU no acepto el color de equipo del LED RGB.")

    latency = link.ping_ms()
    if latency is None:
        print("[init] AVISO: el MCU no responde. Revisa que el sketch este cargado.")
    else:
        print(f"[init] Bridge OK, latencia {latency:.1f} ms")

    s = link.poll()
    if s is not None:
        print(f"[init] TCRT normalizados: FL={s.raw_fl} FR={s.raw_fr} "
              f"RL={s.raw_rl} RR={s.raw_rr}  (negro si < {config.LINE_BLACK_BELOW})")
        print(f"[init] APDS piso: {'OK' if s.floor_sensor_ok else 'NO DETECTADO'}"
              f"   r={s.r} g={s.g} b={s.b} c={s.c}")
        if not s.front_sensors_ok:
            print("[init] AVISO: sin TCRT frontales: la patrulla no arrancara.")

    vision.start()
    print(f"[init] vision: {vision.stats()}")
    if config.FLAG_REQUIRES_KEY:
        print("[init] cerrojo del reglamento ACTIVO: la bandera solo se persigue "
              "con la llave depositada.")
    else:
        print("[init] AVISO: FLAG_REQUIRES_KEY = False. Solo para pruebas de banco.")

    threading.Thread(target=sense_poller, daemon=True, name="sense").start()
    threading.Thread(target=telemetry_pusher, daemon=True, name="telemetry").start()
    if s is not None and s.btn_presses < 0:
        print("[init] AVISO: el sketch no manda el boton de equipo (A4). "
              "Recarga el sketch de esta version.")
    print(f"[init] equipo {config.TEAM.upper()} (config.TEAM). El boton A4 lo "
          f"alterna fuera de ronda; el LED RGB muestra el vigente.")
    print("[init] IDLE. Pulsa 'Iniciar patrulla' en la pagina (puerto 7000).")


startup()
App.run(user_loop=control_loop)
