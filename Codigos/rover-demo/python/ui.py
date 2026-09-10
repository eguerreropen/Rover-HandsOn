"""
ui.py - Pagina web de lecturas de sensores (brick web_ui de App Lab).

SOLO rutas GET fijas. COMPROBADO EN LA PLACA en la version anterior:
expose_api() con POST invoca el handler SIN el cuerpo de la peticion
(responde 200 y actua con valores por defecto). Con GET a rutas fijas la
accion va en la ruta y no hay nada que perder.

Telemetria: send_message() (socket.io) a UI_PUSH_HZ; si el navegador no tiene
socket.io, la pagina sondea /api/state.
"""

import config
from mission import Mission


def mission_states():
    """Los estados a los que la pagina puede saltar. Sale de la propia FSM:
    si manana se anade uno, el boton aparece solo."""
    return Mission.FORCEABLE


class RoverUI:
    def __init__(self, mission, link, tracker, vision=None):
        self.mission = mission
        self.link = link
        self.tracker = tracker
        self.vision = vision
        self.ui = None
        self.available = False
        self._last = None
        try:
            from arduino.app_bricks.web_ui import WebUI
            self.ui = WebUI()
            self.available = True
        except Exception as e:                     # noqa: BLE001
            print(f"[ui] web_ui no disponible ({e}). Se sigue sin pagina.")
            return
        self._wire()

    def _wire(self):
        expose = getattr(self.ui, "expose_api", None)
        if not callable(expose):
            print("[ui] AVISO: expose_api no existe; solo telemetria.")
            return
        routes = {
            "/api/state":        lambda: self.snapshot(),
            "/api/patrol/start": self._start,
            "/api/patrol/stop":  self._stop,
            "/api/estop":        self._estop,
            "/api/clear":        self._clear,
            "/api/calib":        self._calib,
            # --- llave (parte 2). Rutas fijas: expose_api no pasa parametros ---
            "/api/key/release":  lambda: self._key(config.SERVO_KEY_RELEASE),
            "/api/key/hold":     lambda: self._key(config.SERVO_KEY_HOLD),
            "/api/key/reset":    self._key_reset,
            "/api/key/off":       self._key_off,
            # --- brazo (parte 4): pinza y elevacion ---
            "/api/arm/grip/open":  lambda: self._arm(config.SERVO_GRIP,
                                                     config.SERVO_GRIP_OPEN),
            "/api/arm/grip/close": lambda: self._arm(config.SERVO_GRIP,
                                                     config.SERVO_GRIP_CLOSE),
            "/api/arm/grip/off":   lambda: self._arm_off(config.SERVO_GRIP),
            "/api/arm/lift/down":  lambda: self._arm(config.SERVO_LIFT,
                                                     config.SERVO_LIFT_DOWN),
            "/api/arm/lift/up":    lambda: self._arm(config.SERVO_LIFT,
                                                     config.SERVO_LIFT_UP),
            "/api/arm/lift/off":   lambda: self._arm_off(config.SERVO_LIFT),
            "/api/arm/home":       self._arm_home,
            "/api/flag/reset":     self._flag_reset,
        }
        # Angulos fijos del brazo, para encontrar los topes sin recompilar.
        # Es como se confirmo que 0 es abierto y 90 cerrado.
        for deg in (0, 20, 40, 60, 90, 120, 150, 180):
            routes[f"/api/arm/grip/deg/{deg}"] = (
                lambda d=deg: self._arm(config.SERVO_GRIP, d))
            routes[f"/api/arm/lift/deg/{deg}"] = (
                lambda d=deg: self._arm(config.SERVO_LIFT, d))
        # --- saltar directamente a un estado (pruebas) ---
        # Una ruta FIJA por estado, no /api/state/<nombre> con parametro:
        # expose_api NO pasa el cuerpo de la peticion (comprobado en la placa),
        # y con rutas fijas no hay nada que perder por el camino.
        for st in mission_states():
            routes[f"/api/goto/{st}"] = (lambda n=st: self._goto(n))
        # Angulos fijos para encontrar los topes del servo sin recompilar.
        for deg in (0, 30, 60, 90, 120, 150, 180):
            routes[f"/api/key/deg/{deg}"] = (lambda d=deg: self._key(d))
        try:
            for path, fn in routes.items():
                expose("GET", path, fn)
            print("[ui] rutas GET activas. Prueba: curl http://<ip>:7000/api/state")
        except Exception as e:                     # noqa: BLE001
            print(f"[ui] no se pudieron registrar las rutas: {e}")

        # Foto BAJO DEMANDA, sin video en vivo. Solo cuesta cuando se pide.
        if self.vision is not None:
            try:
                expose("GET", "/api/snapshot", self._snapshot)
                print("[ui] foto con overlay en http://<ip>:7000/api/snapshot")
            except Exception as e:                 # noqa: BLE001
                print(f"[ui] sin foto: {e}")

    def _snapshot(self):
        """Un JPEG con el recuadro de la deteccion, para calibrar los HSV."""
        jpg = self.vision.snapshot_jpeg() if self.vision else None
        if jpg is None:
            return {"ok": False, "error": "sin frame de camara"}
        try:
            from fastapi import Response
        except ImportError:
            return {"ok": False, "error": "fastapi no disponible"}
        return Response(content=jpg, media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    # --------------------------------------------------------------- rutas
    def _start(self):
        ok = self.mission.start_patrol(self.link.sense())
        return {"ok": ok, "state": self.mission.state}

    def _stop(self):
        self.mission.stop_patrol()
        return {"ok": True, "state": self.mission.state}

    def _estop(self):
        self.mission.estop("boton de la pagina")
        self.link.stop()
        return {"ok": True, "state": self.mission.state}

    def _clear(self):
        self.mission.clear_estop()
        return {"ok": True, "state": self.mission.state}

    def _key(self, deg):
        """Mueve el servo de la llave a mano (calibracion y demo)."""
        ok = self.link.servo(config.SERVO_KEY, deg)
        return {"ok": ok, "deg": deg}

    def _key_reset(self):
        self.mission.reset_key()
        # Re-engancha el servo en retencion: si estaba apagado tras soltar,
        # asi vuelve a sujetar la llave que vas a cargar.
        self.link.servo(config.SERVO_KEY, config.SERVO_KEY_HOLD)
        return {"ok": True, "key_deposited": self.mission.key_deposited}

    # ----------------------------------------------------------- brazo (p.4)
    def _arm(self, sid, deg):
        return {"ok": self.link.servo(sid, deg), "id": sid, "deg": deg}

    def _arm_off(self, sid):
        """Apaga un servo del brazo (detach).

        La PINZA apagada SUELTA la bandera: solo tiene sentido con el brazo
        vacio. La ELEVACION apagada solo se sostiene si el mecanismo lo hace
        por si mismo -- es justo la prueba que decide LIFT_DETACH_AFTER_UP:
        sube el brazo CON la bandera, apaga, y mira si aguanta."""
        return {"ok": self.link.servo_off(sid), "id": sid}

    def _arm_home(self):
        """Brazo a posicion de partida: pinza abierta, elevacion abajo."""
        a = self.link.servo(config.SERVO_GRIP, config.SERVO_GRIP_OPEN)
        b = self.link.servo(config.SERVO_LIFT, config.SERVO_LIFT_DOWN)
        return {"ok": bool(a and b)}

    def _flag_reset(self):
        """Repetir la parte 4: la bandera vuelve a contar como no agarrada, y
        el brazo a posicion de partida para poder colocarla."""
        self.mission.reset_flag()
        self._arm_home()
        return {"ok": True, "has_flag": self.mission.has_flag}

    def _goto(self, name):
        """Salta a un estado sin recorrer los anteriores. La FSM decide si se
        puede: los estados que mueven exigen los mismos sensores y el mismo
        enlace que 'Iniciar patrulla'."""
        ok, why = self.mission.force_state(name, self.link.sense())
        if not ok:
            print(f"[ui] salto a {name} RECHAZADO: {why}")
        return {"ok": ok, "state": self.mission.state, "error": why}

    def _key_off(self):
        """Apaga el servo a mano (sin pulsos). Cualquier orden lo reengancha."""
        return {"ok": self.link.servo_off(config.SERVO_KEY)}

    def _calib(self):
        s = self.link.sense()
        return {"ok": True,
                "lines": [s.raw_fl, s.raw_fr, s.raw_rl, s.raw_rr],
                "color": self.tracker.calib_dict(s)}

    # ------------------------------------------------------------- salida
    def snapshot(self):
        return self._last or {"fsm": {"state": "?"}}

    def push(self, payload):
        self._last = payload
        if not self.available:
            return
        try:
            self.ui.send_message("rover_telemetry", payload)
        except Exception as e:                     # noqa: BLE001
            print(f"[ui] no se pudo enviar telemetria: {e}")
