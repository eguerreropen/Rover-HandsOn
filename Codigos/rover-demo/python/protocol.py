"""
protocol.py - Unico punto de contacto con el MCU (Bridge de App Lab).

 1. El contrato (nombres y formato de cada RPC) vive aqui y en sketch.ino.
 2. El Bridge puede fallar; la excepcion NO sube al lazo de control: se
    absorbe y se cuenta.
 3. drive() va por notify() (sin esperar respuesta) para no meter la latencia
    del Bridge dentro del lazo. notify() NO tiene acuse de recibo: por eso el
    MCU devuelve en sense() cuantos drive() ha recibido, y aqui se compara
    con los enviados. Es la unica medida directa de perdida en el enlace.
 4. TODO acceso al Bridge pasa por UN cerrojo: es un enlace serie y dos hilos
    escribiendo a la vez entrelazan tramas.
"""

import threading
import time

from arduino.app_utils import Bridge

import config


class Sense:
    """Foto del estado del MCU. Formato: sketch.ino -> rpc_sense() (15 campos)."""

    __slots__ = ("left", "right", "black_mask", "raw_fl", "raw_fr", "raw_rl",
                 "raw_rr", "r", "g", "b", "c", "flags", "mcu_ms",
                 "drives", "loop_max_ms", "slow_passes", "drive_gap_ms",
                 "future_marks", "key_angle", "t",
                 # --- rellenados por el MPU (colors.py) ---
                 "color", "color_stable")

    def __init__(self):
        self.left = self.right = 0
        self.black_mask = 0
        self.raw_fl = self.raw_fr = self.raw_rl = self.raw_rr = 0
        self.r = self.g = self.b = self.c = 0
        self.flags = 0
        self.mcu_ms = 0
        self.drives = 0
        self.loop_max_ms = 0        # pasada mas lenta de ESTA ventana
        self.slow_passes = 0        # acumulado de pasadas > LOOP_SLOW_MS
        self.drive_gap_ms = 0       # hueco mas largo entre drive() RECIBIDOS
        self.future_marks = 0       # marcas de tiempo "del futuro" en el MCU
        self.key_angle = -1         # angulo actual del servo de la llave
        self.t = 0.0
        self.color = config.C_UNKNOWN          # clasificacion instantanea
        self.color_stable = config.C_UNKNOWN   # tras el filtro de estabilidad

    # --- bits de estado (definidos en sketch.ino) ---
    @property
    def watchdog(self):        return bool(self.flags & 0x001)
    @property
    def black_lock(self):      return bool(self.flags & 0x002)
    @property
    def key_powered(self):     return bool(self.flags & 0x020)   # con pulsos
    @property
    def servo_moving(self):    return bool(self.flags & 0x040)
    @property
    def floor_sensor_ok(self): return bool(self.flags & 0x080)
    @property
    def front_latched(self):   return bool(self.flags & 0x400)
    @property
    def rear_latched(self):    return bool(self.flags & 0x800)
    @property
    def front_sensors_ok(self): return bool(self.flags & 0x1000)
    @property
    def rear_sensors_ok(self):  return bool(self.flags & 0x2000)
    @property
    def color_disabled(self):   return bool(self.flags & 0x4000)

    # --- borde negro por esquina/lado (mascara instantanea) ---
    @property
    def black_fl(self): return bool(self.black_mask & 0b0001)
    @property
    def black_fr(self): return bool(self.black_mask & 0b0010)
    @property
    def black_rl(self): return bool(self.black_mask & 0b0100)
    @property
    def black_rr(self): return bool(self.black_mask & 0b1000)
    @property
    def front_black(self): return bool(self.black_mask & 0b0011)
    @property
    def rear_black(self):  return bool(self.black_mask & 0b1100)

    @property
    def any_black(self):
        """Sensor viendo negro AHORA, o enclavamiento todavia puesto."""
        return self.black_mask != 0 or self.front_latched or self.rear_latched

    @property
    def effective_black_mask(self):
        """Mascara para decidir la maniobra de escape. Si el sensor ya paso la
        cinta pero el enclavamiento sigue, se reconstruye desde los
        enclavamientos (la instantanea seria 0 y no diria hacia donde)."""
        if self.black_mask:
            return self.black_mask
        m = 0
        if self.front_latched: m |= 0b0011
        if self.rear_latched:  m |= 0b1100
        return m

    @property
    def age(self):
        return time.monotonic() - self.t if self.t else float("inf")

    def as_dict(self):
        return {
            "left": self.left, "right": self.right,
            "black_mask": self.black_mask,
            "lines": [self.raw_fl, self.raw_fr, self.raw_rl, self.raw_rr],
            "front_latched": self.front_latched, "rear_latched": self.rear_latched,
            "front_sensors_ok": self.front_sensors_ok,
            "rear_sensors_ok": self.rear_sensors_ok,
            "rgbc": [self.r, self.g, self.b, self.c],
            "color": config.COLOR_NAMES.get(self.color, "?"),
            "color_stable": config.COLOR_NAMES.get(self.color_stable, "?"),
            "apds_ok": self.floor_sensor_ok and not self.color_disabled,
            "watchdog": self.watchdog, "black_lock": self.black_lock,
            "age_s": round(self.age, 2) if self.t else None,
            "loop_max_ms": self.loop_max_ms,
            "slow_passes": self.slow_passes,
            "drive_gap_ms": self.drive_gap_ms,
            "future_marks": self.future_marks,
            "key_angle": self.key_angle,
            "key_powered": self.key_powered,
            "servo_moving": self.servo_moving,
        }


class RoverLink:
    _bus = threading.Lock()            # serializa TODO acceso al Bridge

    def __init__(self):
        self._lock = threading.Lock()
        self._sense = Sense()
        self._errors = 0
        self._last_cmd = (0, 0)
        self._last_mcu_ms = None
        self.mcu_reboots = 0
        self.drive_calls = 0
        self.drive_fails = 0
        # --- instrumentacion del enlace ---
        self.mcu_drives = 0
        self.mcu_drives_lost = 0
        self.mcu_loop_max_ms = 0       # maximo historico del lazo del MCU
        self.mcu_slow_passes = 0
        self.mcu_max_drive_gap_ms = 0   # lo mismo, medido en el MCU
        self.mcu_future_marks = 0       # ver elapsedSince() en el sketch
        # HUECO MAS LARGO ENTRE DOS drive() DEL LADO MPU.
        # Es la medida que faltaba: si este numero pasa de CMD_TIMEOUT_MS, el
        # que se callo fue la Pi... perdon, el MPU, y el watchdog del MCU hizo
        # exactamente su trabajo. Si se queda muy por debajo y el watchdog
        # salta igual, el problema esta al otro lado o en el transporte.
        self.max_drive_gap_ms = 0.0
        self._last_drive_t = None
        self._rate_ref = None
        self.mcu_drive_hz = 0.0
        self.mpu_drive_hz = 0.0

    # -------------------------------------------------------------- ordenes
    def drive(self, left, right):
        left = int(_clamp(left, -1000, 1000))
        right = int(_clamp(right, -1000, 1000))
        self._last_cmd = (left, right)
        now = time.monotonic()
        if self._last_drive_t is not None:
            gap = (now - self._last_drive_t) * 1000.0
            if gap > self.max_drive_gap_ms:
                self.max_drive_gap_ms = gap
        self._last_drive_t = now
        self.drive_calls += 1
        # notify() es fire-and-forget: el MPU puede adelantarse y las ordenes
        # se acumulan en el transporte. call() espera respuesta, o sea que
        # impone CONTRAPRESION: no se manda la siguiente hasta que la anterior
        # llego. Cuesta la latencia de ida y vuelta dentro del ciclo (unos ms
        # de los 50 que dura), y a cambio elimina el agrupamiento y hace
        # visibles las perdidas. Ver config.DRIVE_USE_CALL.
        if getattr(config, "DRIVE_USE_CALL", False):
            ok = self._call("drive", left, right) is not None
        else:
            ok = self._notify("drive", left, right)
        if not ok:
            self.drive_fails += 1
        return ok

    def stop(self):
        self._last_cmd = (0, 0)
        return self._call("stop") is not None

    def servo(self, sid, deg):
        """Servo a un angulo crudo. El MCU bloquea los motores 250 ms."""
        return self._call("servo", int(sid), int(deg)) is not None

    def alive(self):
        """Etapa de arranque del sketch (0..7), o None si no contesta.

        Es el sondeo que NO depende del monitor serie: lo atiende el hilo del
        Bridge del MCU, asi que responde aunque su setup() o su loop() esten
        colgados. Ver rpc_alive() en el sketch.
        """
        r = self._call("alive")
        try:
            return int(r)
        except (TypeError, ValueError):
            return None

    def set_flag_led(self, mode):
        """LED de bandera: 0 apagado, 1 parpadeo, 2 fijo."""
        return self._call("flagled", int(mode)) is not None

    def servo_off(self, sid):
        """Apaga el servo (detach). Esperar SERVO_SETTLE_S desde la ultima
        orden: si no, el eje se queda a medio recorrido."""
        return self._call("servo_off", int(sid)) is not None

    def configure_lines(self, black_below, hysteresis):
        """Manda el umbral de negro de config.py al MCU."""
        return self._call("cfg_lines", int(black_below), int(hysteresis)) is not None

    def ping_ms(self):
        t0 = time.monotonic()
        if self._call("ping") is None:
            return None
        return (time.monotonic() - t0) * 1000.0

    # ----------------------------------------------------------- telemetria
    def poll(self):
        s = _parse_sense(self._call("sense"))
        if s is not None:
            # Si millis() del MCU retrocede, el MCU se reinicio (brownout).
            if self._last_mcu_ms is not None and s.mcu_ms < self._last_mcu_ms:
                self.mcu_reboots += 1
                print(f"[link] *** EL MCU SE REINICIO *** (uptime {self._last_mcu_ms}"
                      f" -> {s.mcu_ms} ms): caida de tension. Revisa alimentacion.")
            self._last_mcu_ms = s.mcu_ms
            self._track(s)
            with self._lock:
                prev = self._sense
                # el color lo pone colors.py sobre el Sense vigente: se hereda
                # para que un poll() no lo borre entre dos clasificaciones
                s.color, s.color_stable = prev.color, prev.color_stable
                self._sense = s
        return s

    def sense(self):
        with self._lock:
            return self._sense

    def _track(self, s):
        """Compara ordenes enviadas por el MPU con recibidas por el MCU."""
        if s.loop_max_ms > self.mcu_loop_max_ms:
            self.mcu_loop_max_ms = s.loop_max_ms
        self.mcu_slow_passes = s.slow_passes
        if s.drive_gap_ms > self.mcu_max_drive_gap_ms:
            self.mcu_max_drive_gap_ms = s.drive_gap_ms
        self.mcu_future_marks = s.future_marks
        prev = self.mcu_drives
        self.mcu_drives = s.drives
        if s.drives < prev:                    # contadores del MCU a cero
            self._rate_ref = None
            return
        now = time.monotonic()
        if self._rate_ref is None:
            self._rate_ref = (s.drives, self.drive_calls, now)
            return
        d0, c0, t0 = self._rate_ref
        dt = now - t0
        if dt >= 1.0:
            self.mcu_drive_hz = (s.drives - d0) / dt
            self.mpu_drive_hz = (self.drive_calls - c0) / dt
            lost = (self.drive_calls - c0) - (s.drives - d0)
            if lost > 0:
                self.mcu_drives_lost += lost
            self._rate_ref = (s.drives, self.drive_calls, now)

    # ------------------------------------------------------------------ misc
    @property
    def error_count(self): return self._errors

    @property
    def last_command(self): return self._last_cmd

    def _call(self, name, *args):
        try:
            with RoverLink._bus:
                return Bridge.call(name, *args)
        except Exception as e:                     # noqa: BLE001
            self._bump(name, e)
            return None

    def _notify(self, name, *args):
        try:
            with RoverLink._bus:
                Bridge.notify(name, *args)
            return True
        except Exception as e:                     # noqa: BLE001
            self._bump(name, e)
            return False

    def _bump(self, what, exc):
        self._errors += 1
        if self._errors <= 5 or self._errors % 50 == 0:
            print(f"[link] fallo en {what} (#{self._errors}): {type(exc).__name__}: {exc}")
        # Veredicto una sola vez: muchos fallos seguidos SIN haber recibido
        # jamas un frame no es un enlace ruidoso, es un MCU que no esta.
        if self._errors == 20 and self._last_mcu_ms is None:
            print("[link] ============================================================")
            print("[link] 20 fallos seguidos y NI UN frame de telemetria recibido.")
            print("[link] El cliente del Bridge espera hasta 10 s por llamada: fallos")
            print("[link] tan rapidos significan que el router NO tiene el metodo, o")
            print("[link] sea que el MCU nunca ejecuto provide_safe(): el sketch no")
            print("[link] arranco o no llego a registrar los RPC.")
            print("[link] MIRA: 1) la compilacion del sketch en la consola de App Lab")
            print("[link]       2) el monitor del MCU: debe decir '[MCU] boot 1/6'...")
            print("[link]       3) el texto de la excepcion de arriba")
            print("[link] ============================================================")


def _parse_sense(raw):
    """
    CSV: vl,vr,mask,FL,FR,RL,RR,r,g,b,c,flags,ms,drives,loopmax
         [,slow[,dgap[,fut[,key]]]]

    Se toleran tramas de 15 campos a proposito: si en la placa quedo un
    sketch anterior, la App sigue funcionando y solo deja el contador nuevo
    a cero, en vez de morir con "sense ilegible" en bucle.
    """
    if raw is None:
        return None
    try:
        text = raw.decode() if isinstance(raw, (bytes, bytearray)) else str(raw)
        p = [x.strip() for x in text.split(",")]
        if len(p) < 15:
            raise ValueError(f"esperaba 15 campos, llegaron {len(p)}")
        s = Sense()
        (s.left, s.right, s.black_mask,
         s.raw_fl, s.raw_fr, s.raw_rl, s.raw_rr,
         s.r, s.g, s.b, s.c,
         s.flags, s.mcu_ms, s.drives, s.loop_max_ms) = (int(x) for x in p[:15])
        if len(p) >= 16:
            s.slow_passes = int(p[15])
        if len(p) >= 17:
            s.drive_gap_ms = int(p[16])
        if len(p) >= 18:
            s.future_marks = int(p[17])
        if len(p) >= 19:
            s.key_angle = int(p[18])
        s.t = time.monotonic()
        return s
    except Exception as e:                         # noqa: BLE001
        print(f"[link] sense ilegible: {raw!r} ({e})")
        return None


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)
