"""
mission.py - Estados del rover. Parte 1 (borde y colores) + parte 2 (llave).

    IDLE         parado, esperando orden desde la pagina
    PATROL       avanza recto identificando el color bajo el rover
    EDGE         el MCU enclavo un borde negro: alejarse, girar, seguir
    DEPOSIT_KEY  pisa amarillo con la llave a bordo: entra, para, la suelta
    BACKOFF_KEY  retrocede (la caja cae DELANTE), gira ~180, rearma el servo
    AVOID_ZONE   llave ya depositada y pisa amarillo: la caja esta ahi dentro,
                 rebotar como en el borde negro
    APPROACH     PARTE 3: la camara ve la bandera rival y la llave ya esta
                 depositada: ir hacia ella centrandola, frenar al llegar
    FLAG_REACHED a distancia de agarre: parado, LED fijo, se alinea si hace falta
    GRAB         PARTE 4: cierra la pinza y sube la elevacion (parado)
    HOLDING      bandera agarrada y en alto, rover quieto. FIN de la parte 4
                 mientras RETURN_AFTER_GRAB sea False
    CARRY        con la bandera en alto, busca la cinta de SU color
    DELIVER      en su zona: baja, abre la pinza y retrocede
    DONE         mision cumplida, parado
    ESTOP        parada de emergencia, pegajosa

CERROJO DEL REGLAMENTO (parte 3): "si un robot busca la bandera del oponente
antes de depositar la caja, pierde la ronda". VER la bandera es pasivo y se
senaliza siempre (LED parpadeando); PERSEGUIRLA solo con key_deposited. No
es una transicion que un bug pueda saltarse: esta en un solo sitio,
_hunt_allowed(), y APPROACH no se entra sin pasar por ahi.

COMO VUELVE A CASA, Y POR QUE ASI
---------------------------------
El rover NO tiene encoders: no hay odometria, no sabe donde esta, y por tanto
no puede "ir a la coordenada de su base". Lo unico que sabe con certeza es que
color pisa. Asi que la vuelta es reactiva: avanza rebotando en el borde negro
igual que en PATROL, y termina cuando el APDS ve la cinta de SU color.

Eso no es una ruta, es una BUSQUEDA: el tiempo hasta llegar no esta acotado y
depende de donde estuviera la bandera. Si hiciera falta acotarlo, la mejora
numero uno del proyecto siguen siendo los encoders.

La FSM es PURA: recibe la telemetria y devuelve una Accion (avance, giro,
ordenes a actuadores). No toca el Bridge. Se prueba en un PC sin robot.

POR QUE EL AMARILLO SE VUELVE ZONA PROHIBIDA DESPUES DE DEPOSITAR
-----------------------------------------------------------------
La llave es amarilla y cae sobre cinta amarilla: para la camara es invisible,
asi que esquivarla "viendola" no es una opcion. En cambio, sabemos DONDE esta:
dentro de la zona amarilla, a por lo menos ZONE_ENTER_ADVANCE_S de avance
del borde. Si el rover nunca vuelve a entrar en el amarillo, nunca la pisa.
Es una garantia geometrica, no perceptiva, y por eso es fiable.

El coste: el rover no vuelve a cruzar la zona neutra. Para la patrulla del
demo no importa; para la mision completa (ir a la base rival, que esta al
otro lado) habra que rodearla, y eso lo resolvera la camara en la parte 4.
"""

import time

import config
import steer


class S:
    IDLE = "IDLE"
    PATROL = "PATROL"
    EDGE = "EDGE"
    DEPOSIT_KEY = "DEPOSIT_KEY"
    BACKOFF_KEY = "BACKOFF_KEY"
    AVOID_ZONE = "AVOID_ZONE"
    APPROACH = "APPROACH"
    FLAG_REACHED = "FLAG_REACHED"
    GRAB = "GRAB"
    HOLDING = "HOLDING"
    CARRY = "CARRY"
    DELIVER = "DELIVER"
    DONE = "DONE"
    ESTOP = "ESTOP"


class Action:
    __slots__ = ("throttle", "turn", "commands")

    def __init__(self, throttle=0, turn=0, commands=None):
        self.throttle = throttle
        self.turn = turn
        self.commands = commands or []      # [("grip", angulo), ("key_spin", (ang, s)), ...]


def escape_from_edge(black_mask):
    """
    Maniobra a partir de que esquinas ven negro (bit0 FL, bit1 FR, bit2 RL,
    bit3 RR). Devuelve (avance, giro) en {-1, 0, +1}:
      solo frente -> retroceder y girar hacia el lado limpio
      solo atras  -> avanzar y girar hacia el lado limpio
      ambos       -> chasis cruzado sobre la linea: solo girar sobre el eje
    """
    fl, fr = bool(black_mask & 1), bool(black_mask & 2)
    rl, rr = bool(black_mask & 4), bool(black_mask & 8)
    front, rear = fl or fr, rl or rr
    left_side, right_side = fl or rl, fr or rr
    if left_side and not right_side:
        turn = 1            # borde a la izquierda -> escapar a la derecha
    elif right_side and not left_side:
        turn = -1
    else:
        turn = 1            # deterministico: se puede reproducir en pruebas
    if front and rear:
        return 0, turn
    if front:
        return -1, turn
    if rear:
        return 1, turn
    return 0, turn


class Mission:
    def __init__(self):
        self.state = S.IDLE
        self.prev_state = S.IDLE
        self.reason = ""
        self._entered = time.monotonic()
        # --- borde ---
        self._edge_phase = None
        self._edge_until = 0.0
        self._edge_move = 0
        self._edge_dir = 1
        # --- llave ---
        self.key_deposited = False
        self.key_drops = 0                  # veces que se ha soltado (pruebas)
        self._key_phase = None
        self._phase_until = 0.0
        self._avoid_dir = 1                 # alterna: evita bucles contra una pared
        # Apagado pendiente, UNO POR SERVO: {"key": t, "grip": t, "lift": t}.
        # t es el instante a partir del cual el eje ya ha tenido tiempo de
        # llegar y se le pueden cortar los pulsos. Se emite desde step() este
        # el rover en el estado que este: el apagado no depende de la
        # maniobra, solo del tiempo transcurrido desde la orden.
        #
        # Un plazo por servo y no uno solo: si los tres compartieran plazo, una
        # orden al gripper reiniciaria el del brazo y el brazo se quedaria
        # encendido, que es justo lo que se quiere evitar.
        self._detach_at = {}
        # --- bandera (parte 3) ---
        self._last_flag_seen = 0.0
        self._led_mode = 0                  # ultimo modo mandado al LED
        self._hunt_blocked_said = False
        self.flag_seen = False              # para la pagina
        # --- bandera agarrada (parte 4) ---
        self.has_flag = False
        self.flag_delivered = False
        self._grab_phase = None
        self._last_dist = None              # ultima distancia buena vista
        # A donde volver cuando termine una interrupcion (borde negro, caja
        # amarilla). Sin esto, EDGE devolvia SIEMPRE a PATROL: el rover con la
        # bandera en la pinza rebotaba en el borde y se le olvidaba que estaba
        # volviendo a casa -- y peor, PATROL habria intentado perseguir otra
        # bandera llevando una puesta.
        self._resume = S.PATROL

    # ---------------------------------------------------------- servos
    def _servo(self, name, deg, keep_on=False):
        """Orden a un servo, con su apagado programado.

        keep_on=True para las ordenes que SUJETAN algo (la llave a HOLD, la
        pinza cerrada sobre la bandera): ahi el servo tiene que seguir
        alimentado, porque sujetar no es moverse. Sin esto, la orden de
        "retener" programaba su propio apagado y el rover patrullaba con la
        llave sin pulsos, o sea sin retener nada.

        TODAS las ordenes de servo de la FSM pasan por aqui. Es la unica forma
        de que no se escape ninguna: si un estado nuevo emite ("grip", x) a
        mano, ese servo se queda encendido para siempre y el sintoma -el UBEC
        hundido a mitad de ronda- no se parece en nada a la causa.

        Hace lo mismo que los botones "Apagar" de la pagina, que ya estaban
        probados: mover, esperar a que el eje llegue, cortar los pulsos.
        """
        if config.SERVO_AUTO_OFF.get(name, False) and not keep_on:
            # Asentamiento POR SERVO: depende de los grados que recorre y de
            # la carga. Los 170 de la llave contra la caja necesitan el doble
            # que los 40 del brazo.
            self._detach_at[name] = time.monotonic() + config.settle_s(name)
        else:
            # Sin apagado automatico: y ademas se cancela uno pendiente, para
            # que un plazo viejo no apague un servo que acaba de recibir una
            # orden nueva.
            self._detach_at.pop(name, None)
        return (name, deg)

    # ------------------------------------------------------------- control
    def request(self, new_state, reason="", _force=False):
        if new_state == self.state:
            return
        if self.state == S.ESTOP and not _force:
            print(f"[fsm] transicion a {new_state} IGNORADA: en ESTOP ({reason})")
            return
        self.prev_state, self.state = self.state, new_state
        self._entered = time.monotonic()
        self.reason = reason
        print(f"[fsm] {self.prev_state} -> {self.state} ({reason})")

    def estop(self, reason="manual"):
        self.request(S.ESTOP, reason, _force=True)

    def clear_estop(self):
        if self.state == S.ESTOP:
            self.request(S.IDLE, "rearme manual", _force=True)

    def start_patrol(self, sense):
        """La patrulla NO arranca sin los TCRT frontales ni sin telemetria
        fresca: son lo unico que impide salirse por delante."""
        if self.state == S.ESTOP:
            print("[fsm] patrulla IGNORADA: parada de emergencia activa. Rearma primero.")
            return False
        if not sense.front_sensors_ok:
            print("[fsm] patrulla RECHAZADA: el MCU no reporta los TCRT frontales.")
            return False
        if sense.watchdog or sense.age > 1.0:
            print("[fsm] patrulla RECHAZADA: sin telemetria fresca del MCU.")
            return False
        self.request(S.PATROL, "orden desde la pagina")
        return True

    # Estados en los que el rover NO esta en ronda: el unico momento en que se
    # puede cambiar de equipo. En plena ronda el boton A4 se IGNORA -- cambiar
    # de equipo a mitad cambiaria que bandera persigue y que cinta es casa.
    TEAM_CHANGE_STATES = (S.IDLE, S.DONE, S.ESTOP)

    @property
    def can_change_team(self):
        return self.state in self.TEAM_CHANGE_STATES

    def stop_patrol(self):
        if self.state not in (S.IDLE, S.ESTOP):
            self.request(S.IDLE, "orden desde la pagina")

    # ------------------------------------------------- forzar estado (pruebas)
    #
    # POR QUE ESTO EXISTE Y POR QUE NO ES UN request() PELADO
    #
    # Probar la parte N obligaba a recorrer todas las anteriores: para ver el
    # rebote en amarillo habia que depositar la llave primero. Estos botones
    # saltan directamente al estado que se quiere probar.
    #
    # Pero un request() a secas NO VALE, por dos razones:
    #
    # 1. Cada estado con maniobra guarda su fase en _key_phase/_edge_phase y
    #    su plazo en _phase_until/_edge_until. Entrar sin ponerlos deja la
    #    maquina en un punto que su propio codigo no contempla. Ejemplo real:
    #    DEPOSIT_KEY con _key_phase = None cae por la rama "drop", encuentra
    #    _phase_until = 0 (ya vencido) y se va a BACKOFF_KEY sin haber soltado
    #    nada. O sea: el boton "probar deposito" no probaria el deposito.
    #    Por eso cada estado tiene aqui su propia preparacion.
    #
    # 2. Un boton que arranca los motores no puede saltarse las mismas
    #    comprobaciones que "Iniciar patrulla". Sin TCRT frontales o sin
    #    telemetria fresca, el rover conduce ciego hacia el borde. Los estados
    #    que MUEVEN exigen lo mismo que start_patrol(); los que no (IDLE,
    #    FLAG_REACHED) no exigen nada.
    #
    # ESTOP sigue siendo pegajoso: desde ahi no se fuerza nada sin rearmar.

    # Estados que ponen los motores en marcha -> exigen sensores y enlace.
    _MOVING = (S.PATROL, S.EDGE, S.DEPOSIT_KEY, S.BACKOFF_KEY,
               S.AVOID_ZONE, S.APPROACH, S.GRAB, S.CARRY, S.DELIVER)

    FORCEABLE = (S.IDLE, S.PATROL, S.EDGE, S.DEPOSIT_KEY, S.BACKOFF_KEY,
                 S.AVOID_ZONE, S.APPROACH, S.FLAG_REACHED, S.GRAB, S.HOLDING,
                 S.CARRY, S.DELIVER, S.DONE)

    def force_state(self, name, sense):
        """Salta al estado 'name' dejandolo COHERENTE. Devuelve (ok, motivo)."""
        name = (name or "").upper()
        if name not in self.FORCEABLE:
            return False, f"estado desconocido o no forzable: {name}"
        if self.state == S.ESTOP:
            return False, "parada de emergencia activa: rearma primero"
        if name in self._MOVING:
            if not sense.front_sensors_ok:
                return False, "el MCU no reporta los TCRT frontales"
            if sense.watchdog or sense.age > 1.0:
                return False, "sin telemetria fresca del MCU"

        now = time.monotonic()
        # Se limpian las dos maquinas de fases antes de armar la que toca: un
        # _edge_phase viejo secuestraria el estado nuevo en el primer ciclo.
        self._key_phase = None
        self._edge_phase = None
        self._grab_phase = None
        self._phase_until = 0.0
        self._edge_until = 0.0

        if name == S.DEPOSIT_KEY:
            # Se entra por la fase "enter" (avanzar dentro de la zona), que es
            # como llega desde PATROL. Asi el boton prueba la secuencia entera:
            # entrar, parar, soltar. Si la llave ya consta depositada, la
            # secuencia no tendria sentido: se rearma sola.
            if self.key_deposited:
                self.key_deposited = False
                print("[fsm] forzado DEPOSIT_KEY: la llave se vuelve a dar por "
                      "a bordo (cargala a mano antes de probar)")
            self._key_phase = "enter"
            self._phase_until = now + config.ZONE_ENTER_ADVANCE_S
        elif name == S.BACKOFF_KEY:
            self._key_phase = "back"
            self._phase_until = now + config.KEY_BACKOFF_S
        elif name == S.AVOID_ZONE:
            self._key_phase = "back"
            self._phase_until = now + config.AVOID_BACK_S
            self._avoid_dir *= -1
        elif name == S.EDGE:
            # Sin negro real bajo los sensores no hay mascara de la que sacar
            # el lado de escape. Se usa la que haya; si es 0, escape_from_edge
            # devuelve un giro determinista y la maniobra se ve igual.
            self._edge_move, self._edge_dir = escape_from_edge(
                sense.effective_black_mask)
            self._edge_phase = "turn" if self._edge_move == 0 else "back"
            self._edge_until = now + (config.EDGE_TURN_S if self._edge_move == 0
                                      else config.EDGE_BACK_S)
        elif name == S.APPROACH:
            # APPROACH sin bandera a la vista se abandona solo tras
            # LOST_TARGET_S. Se le regala esa ventana desde ya para que el
            # boton no se caiga a PATROL antes de que pongas la bandera.
            self._last_flag_seen = now
            # Y sin distancia previa: si no, el compromiso de agarre podria
            # dispararse con un valor de una persecucion anterior.
            self._last_dist = None
        elif name == S.GRAB:
            # Entra por la primera fase de la secuencia, como si acabara de
            # llegar desde FLAG_REACHED. Asi el boton prueba el agarre entero.
            self._grab_phase = "blind" if config.GRAB_BLIND_S > 0 else "settle"
            self._phase_until = now + (config.GRAB_BLIND_S
                                       if self._grab_phase == "blind"
                                       else config.GRAB_SETTLE_S)
        elif name == S.CARRY:
            # Se da por agarrada: es lo que hace util al boton (probar la
            # vuelta a casa sin tener que agarrar de verdad cada vez).
            self.has_flag = True
            self._resume = S.CARRY
        elif name == S.DELIVER:
            self.has_flag = True
            self._key_phase = "enter"
            self._phase_until = now + config.CARRY_ENTER_ADVANCE_S
            self._resume = S.DELIVER
        elif name == S.HOLDING:
            self.has_flag = True
            self._resume = S.HOLDING
        elif name in (S.IDLE, S.DONE):
            self._resume = S.PATROL
            if not self._hunt_allowed():
                print("[fsm] forzado APPROACH con la llave a bordo. El cerrojo "
                      "del reglamento NO se ha tocado: PATROL sigue sin "
                      "perseguir. Esto es solo para probar el control.")

        if name == self.state:
            # request() ignora la transicion a uno mismo. Reentrar al mismo
            # estado es util (reinicia la maniobra desde su primera fase), asi
            # que el cronometro se reinicia a mano.
            self._entered = now
            self.reason = "reiniciado desde la pagina"
            print(f"[fsm] {self.state} REINICIADO desde la pagina")
        else:
            self.request(name, "forzado desde la pagina", _force=True)
        return True, ""

    @staticmethod
    def _sees_color(sense, target):
        """El color 'target' bajo el rover, con UNA sola lectura si
        ZONE_TRIGGER_INSTANT. El filtro de estabilidad exige N lecturas
        seguidas y cruzando una cinta no siempre da tiempo a dos."""
        if sense.color_stable == target:
            return True
        return (getattr(config, "ZONE_TRIGGER_INSTANT", False)
                and sense.color == target)

    @staticmethod
    def _sees_yellow(sense):
        """Amarillo bajo el rover.

        Con ZONE_TRIGGER_INSTANT basta UNA lectura. El filtro de estabilidad
        exige N lecturas seguidas y cruzando la cinta solo daba tiempo a una:
        la pagina mostraba el amarillo apareciendo un instante y el deposito
        no se disparaba nunca. Ver la nota de config.ZONE_TRIGGER_INSTANT.
        """
        return Mission._sees_color(sense, config.C_YELLOW)

    def _hunt_allowed(self):
        """Cerrojo duro del reglamento. Unico punto de decision.

        Dos condiciones, no una:
          - la llave depositada (reglamento), y
          - no llevar ya una bandera. Perseguir la segunda llevando la primera
            no esta en el reto y, sobre todo, APPROACH conduce hacia una
            bandera con otra en la pinza: la tira.
        """
        if self.has_flag:
            return False
        return self.key_deposited or not config.FLAG_REQUIRES_KEY

    def reset_key(self):
        """Para repetir la prueba: vuelve a considerar la llave a bordo. Hay
        que haberla cargado a mano antes de pulsar Patrullar otra vez.
        (El re-enganche del servo lo manda la pagina: ver ui._key_reset.)"""
        self.key_deposited = False
        self._detach_at.pop("key", None)   # por si acaso: la caja ya no usa esto
        print("[fsm] llave REARMADA: se considera a bordo otra vez")

    def reset_flag(self):
        """Para repetir la prueba de la parte 4: la bandera vuelve a contar
        como NO agarrada y NO entregada. Coloca la bandera a mano y abre la
        pinza (el boton de la pagina manda las dos ordenes)."""
        self.has_flag = False
        self.flag_delivered = False
        self._grab_phase = None
        self._last_dist = None
        self._resume = S.PATROL
        print("[fsm] bandera REARMADA: se considera no agarrada")

    @property
    def elapsed(self):
        return time.monotonic() - self._entered

    # ---------------------------------------------------------------- step
    def step(self, sense, dt, flag=None):
        if self.state == S.ESTOP:
            return Action()

        active = self.state not in (S.IDLE, S.ESTOP)
        self.flag_seen = flag is not None
        if flag is not None:
            self._last_flag_seen = time.monotonic()

        # Apagado del servo pendiente: se cumple en cuanto el eje ha tenido
        # tiempo de llegar, en el estado que sea.
        pending = []
        ahora = time.monotonic()
        for name in [n for n, t in self._detach_at.items() if ahora >= t]:
            del self._detach_at[name]
            pending.append((name + "_off", None))
            print(f"[servo] {name} APAGADO (sin pulsos)")

        # Enlace caido: no se conduce a ciegas.
        if sense.watchdog and active:
            self.request(S.IDLE, "watchdog del MCU")
            return Action(0, 0, pending)

        # Borde negro: manda sobre todo lo que se mueva. La unica excepcion es
        # la fase de SOLTAR la llave, que ocurre parado: se termina (el servo
        # ya esta girando) y el borde se atiende justo despues.
        # NO se interrumpe una maniobra de servo con el rover ya parado: el
        # servo esta girando, cortarla a medias deja la pinza a medio cerrar o
        # el brazo a medio subir, y el borde sigue estando ahi un ciclo
        # despues. Las fases que avanzan (enter, blind, back) SI se
        # interrumpen: esas si mueven el rover hacia el borde.
        busy_parado = (
            (self.state == S.DEPOSIT_KEY
             and self._key_phase in ("drop", "wait", "retract"))
            or (self.state == S.GRAB
                and self._grab_phase in ("settle", "close", "pause", "lift"))
            or (self.state == S.DELIVER and self._key_phase in ("down", "open")))
        if (sense.any_black and active and self.state != S.EDGE
                and not busy_parado):
            self._begin_edge(sense)

        handler = {S.PATROL: self._patrol, S.EDGE: self._edge,
                   S.DEPOSIT_KEY: self._deposit_key,
                   S.BACKOFF_KEY: self._backoff_key,
                   S.AVOID_ZONE: self._avoid_zone,
                   S.APPROACH: self._approach,
                   S.FLAG_REACHED: self._flag_reached,
                   S.GRAB: self._grab,
                   S.HOLDING: self._holding,
                   S.CARRY: self._carry,
                   S.DELIVER: self._deliver,
                   S.DONE: self._done}.get(self.state)
        if handler in (self._patrol, self._approach, self._flag_reached):
            action = handler(sense, flag)
        else:
            action = handler(sense) if handler else Action()

        # LED de bandera: se manda SOLO cuando cambia el modo, no cada ciclo.
        pending += self._led_commands(flag)
        if pending:
            action.commands = pending + action.commands
        return action

    def _led_commands(self, flag):
        # 2 = en control (llegada). 1 = detectada: mientras se ve O mientras
        # se la sigue (APPROACH), aunque un frame suelto no la traiga -- un
        # LED que parpadea y se apaga a cada frame perdido no senaliza nada.
        if self.state in (S.FLAG_REACHED, S.GRAB, S.HOLDING, S.CARRY,
                          S.DELIVER, S.DONE):
            mode = 2
        elif flag is not None or self.state == S.APPROACH:
            mode = 1
        else:
            mode = 0
        if mode == self._led_mode:
            return []
        self._led_mode = mode
        return [("flagled", mode)]

    # ------------------------------------------------------------- patrulla
    def _patrol(self, sense, flag=None):
        self._resume = S.PATROL
        # El color ESTABLE, no el instantaneo: una lectura suelta en el filo
        # de una cinta no debe disparar la secuencia de la llave.
        if self._sees_yellow(sense):
            if not self.key_deposited:
                self._key_phase = "enter"
                self._phase_until = time.monotonic() + config.ZONE_ENTER_ADVANCE_S
                self.request(S.DEPOSIT_KEY, "zona amarilla: a depositar")
                # Con ZONE_ENTER_ADVANCE_S = 0 se para EN SECO en este mismo
                # ciclo. Devolver SPEED_CREEP aqui mandaria un tick de avance
                # (50 ms) que no pinta nada si lo que se quiere es parar.
                if config.ZONE_ENTER_ADVANCE_S <= 0:
                    return Action(0, 0)
                return Action(config.SPEED_CREEP, 0)
            # Con la llave ya depositada el amarillo ES la caja. Rebotar en el
            # cortaba la patrulla cada vez que se cruzaba la zona central,
            # justo cuando el rover deberia estar yendo a por la bandera.
            # DESACTIVADO: ver config.AVOID_YELLOW_AFTER_DROP.
            if getattr(config, "AVOID_YELLOW_AFTER_DROP", True):
                self._begin_avoid()
                return Action()

        # Bandera a la vista: perseguirla SOLO si el reglamento lo permite.
        if flag is not None:
            if self._hunt_allowed():
                self._hunt_blocked_said = False
                self.request(S.APPROACH, f"bandera {config.ENEMY} vista "
                                         f"(score {flag.score:.2f})")
                return Action()
            if not self._hunt_blocked_said:
                self._hunt_blocked_said = True
                print("[fsm] bandera VISTA pero la llave sigue a bordo: NO se "
                      "persigue (reglamento). Se senaliza con el LED.")
        return Action(config.PATROL_SPEED, 0)

    # ------------------------------------------------------------- bandera
    def _approach(self, sense, flag=None):
        # La caja: con la llave depositada, el amarillo es donde esta. Mandaba
        # sobre la bandera; DESACTIVADO (config.AVOID_YELLOW_AFTER_DROP), que
        # es justo lo que impedia acercarse a una bandera al otro lado.
        if (getattr(config, "AVOID_YELLOW_AFTER_DROP", True)
                and self.key_deposited and self._sees_yellow(sense)):
            self._begin_avoid()
            return Action()

        if flag is None:
            # COMPROMISO. Si la bandera se perdio estando ya muy cerca, no se
            # abandona: a esa distancia estaba delante del rover, y lo mas
            # probable es que se haya salido del encuadre (a menos de ~10 cm
            # ni siquiera pasa el filtro de ancho). Abandonar aqui es la
            # unica forma segura de no agarrarla nunca.
            if (self._last_dist is not None
                    and self._last_dist <= config.GRAB_COMMIT_CM):
                self._begin_grab(f"perdida a {self._last_dist:.0f} cm: "
                                 f"estaba delante, se compromete")
                return Action()
            if time.monotonic() - self._last_flag_seen > config.LOST_TARGET_S:
                self.request(S.PATROL, "bandera perdida")
                self._resume = S.PATROL
            return Action()                  # parado, esperando a verla otra vez

        d = flag.distance_cm
        self._last_dist = d
        if d is not None and d <= config.GRAB_DISTANCE_CM:
            self.request(S.FLAG_REACHED, f"bandera al alcance ({d:.0f} cm)")
            return Action()

        turn = steer.steer_from_error(flag.error_x)
        base = steer.approach_speed(d)
        # Menos avance cuanto mas descentrado: girar y avanzar a la vez con
        # error grande describe una espiral que no converge.
        alignment = max(0.0, 1.0 - abs(flag.error_x))
        return Action(base * alignment, turn)

    def _flag_reached(self, sense, flag=None):
        """Ya esta a distancia de agarre. Falta lo que la distancia no dice:
        que este CENTRADA. Cerrar la pinza con la bandera a un lado del eje la
        tira en vez de agarrarla, y una bandera tumbada ya no se recoge."""
        if flag is None:
            # Se perdio justo aqui: estaba delante. Se agarra igual (ver el
            # compromiso de _approach).
            self._begin_grab("sin deteccion al alcance: se compromete")
            return Action()

        self._last_dist = flag.distance_cm
        if abs(flag.error_x) > config.GRAB_ALIGN_TOL:
            # Pivota SIN avanzar: ya esta a la distancia justa, avanzar aqui
            # solo sirve para empujar la bandera.
            sign = 1.0 if flag.error_x > 0 else -1.0
            return Action(0, config.GRAB_ALIGN_TURN * sign * config.STEER_SIGN)

        self._begin_grab(f"centrada (error {flag.error_x:+.2f}) a "
                         f"{flag.distance_cm:.0f} cm")
        return Action()

    # ------------------------------------------------------- agarre (parte 4)
    def _begin_grab(self, reason):
        self._grab_phase = "blind" if config.GRAB_BLIND_S > 0 else "settle"
        self._phase_until = time.monotonic() + (
            config.GRAB_BLIND_S if self._grab_phase == "blind"
            else config.GRAB_SETTLE_S)
        self.request(S.GRAB, reason)

    def _grab(self, sense):
        """Cierra la pinza y sube la elevacion. TODO parado y DE UNO EN UNO.

        Dos servos de 20 kg arrancando a la vez, o uno arrancando con los
        motores en marcha, es el pico de corriente mas alto del rover: es lo
        que hunde el rail de 5 V y reinicia placas. Se serializa a proposito,
        y el MCU ademas bloquea los motores MOVE_LOCK_MS tras cada orden.
        """
        now = time.monotonic()
        if now < self._phase_until:
            # En "blind" se avanza despacio; en el resto, quieto.
            return Action(config.SPEED_CREEP if self._grab_phase == "blind" else 0, 0)

        if self._grab_phase == "blind":
            self._grab_phase = "settle"
            self._phase_until = now + config.GRAB_SETTLE_S
            return Action()

        if self._grab_phase == "settle":
            self._grab_phase = "close"
            self._phase_until = now + config.GRIP_CLOSE_S
            print(f"[bandera] CERRANDO pinza: servo a {config.SERVO_GRIP_CLOSE}")
            return Action(0, 0, [self._servo("grip", config.SERVO_GRIP_CLOSE)])

        if self._grab_phase == "close":
            # PAUSA antes de levantar. La pinza ya ha llegado a su angulo,
            # pero la bandera sigue acomodandose dentro y el chasis
            # balanceandose por el tiron del cierre: levantar en ese instante
            # es cuando se escapa. Ademas separa los dos picos de corriente
            # (la pinza ya se ha apagado sola antes de que arranque el brazo).
            self._grab_phase = "pause"
            self._phase_until = now + config.GRAB_PAUSE_S
            return Action()

        if self._grab_phase == "pause":
            self._grab_phase = "lift"
            self._phase_until = now + config.LIFT_UP_S
            # Se da por agarrada AQUI, cuando empieza a subir. No hay sensor
            # en la pinza: el codigo NO PUEDE SABER si hay algo dentro. Si
            # cerro en vacio, el rover se vuelve a casa con las manos vacias y
            # no se entera. Ver la nota del README sobre esta laguna.
            self.has_flag = True
            print(f"[bandera] SUBIENDO: elevacion a {config.SERVO_LIFT_UP}")
            return Action(0, 0, [self._servo("lift", config.SERVO_LIFT_UP)])

        # fase "lift" terminada
        self._grab_phase = None
        # El apagado del brazo ya va programado por _servo("lift", ...) de la
        # fase anterior: aqui no hay nada especial que hacer.
        cmds = []
        # PARADA DELIBERADA. Con RETURN_AFTER_GRAB = False la secuencia acaba
        # aqui: acercarse, agarrar, levantar, quieto. La vuelta a casa sigue
        # implementada y probada y se lanza a mano con el boton CARRY de la
        # pagina. Encadenar sola una etapa que aun no se ha visto funcionar en
        # el suelo solo sirve para no saber cual de las dos fallo.
        if getattr(config, "RETURN_AFTER_GRAB", False):
            self._resume = S.CARRY
            self.request(S.CARRY, "bandera agarrada: volviendo a la zona propia")
        else:
            self._resume = S.HOLDING
            self.request(S.HOLDING, "bandera agarrada y en alto: fin de la "
                                    "secuencia (RETURN_AFTER_GRAB = False)")
        return Action(0, 0, cmds)

    def _holding(self, sense):
        """Quieto con la bandera en alto. Se sale con Parar, o con el boton
        CARRY si se quiere probar la vuelta a casa."""
        return Action()

    # -------------------------------------------------- vuelta a casa (p. 4)
    def _carry(self, sense):
        """Lleva la bandera buscando la cinta de SU color, y entrega en la
        primera lectura. Sin condiciones previas: ver la nota de
        config.CARRY_ENTER_ADVANCE_S sobre por que el "armado" sobraba."""
        col = sense.color_stable

        # La caja de la llave: prohibida solo si AVOID_YELLOW_AFTER_DROP.
        if (getattr(config, "AVOID_YELLOW_AFTER_DROP", True)
                and col == config.C_YELLOW and self.key_deposited):
            self._begin_avoid()
            return Action()

        if self._sees_color(sense, config.TEAM_COLOR):
            # DIRECTO: la primera lectura del color propio entrega. No hay
            # "armado" previo -- no hace falta, porque la bandera rival esta
            # siempre sobre LA ZONA RIVAL: en el instante del agarre el rover
            # pisa el color del ENEMIGO, nunca el suyo, asi que "estoy en casa"
            # no puede ser cierto por accidente al empezar.
            self._key_phase = "enter"
            self._phase_until = time.monotonic() + config.CARRY_ENTER_ADVANCE_S
            self._resume = S.DELIVER
            self.request(S.DELIVER, "zona propia: soltando la bandera")
            # Con el avance a cero se para EN SECO: la zona propia es fina y
            # cualquier avance a ciegas la cruza entera.
            if config.CARRY_ENTER_ADVANCE_S <= 0:
                return Action(0, 0)
            return Action(config.SPEED_CREEP, 0)

        return Action(config.CARRY_SPEED, 0)

    # ------------------------------------------------------- entrega (p. 4)
    def _deliver(self, sense):
        """Entra un poco en la zona, baja, abre y retrocede.

        El orden importa: bajar ANTES de abrir. Abrir en alto deja caer la
        bandera desde la altura del brazo y lo mas probable es que ruede fuera
        de la zona o se quede tumbada. Y retroceder al final evita volcarla al
        girar con el brazo todavia encima.
        """
        now = time.monotonic()
        if self._key_phase == "enter":
            if now < self._phase_until:
                return Action(config.SPEED_CREEP, 0)
            self._key_phase = "down"
            self._phase_until = now + config.DELIVER_DOWN_S
            print(f"[bandera] BAJANDO: elevacion a {config.SERVO_LIFT_DOWN}")
            return Action(0, 0, [self._servo("lift", config.SERVO_LIFT_DOWN)])

        if now < self._phase_until:
            # ESPERA CON LA ACCION DE LA FASE, no siempre parado. La fase
            # "back" RETROCEDE, y aqui habia un fallo: este return devolvia
            # cero para todas las fases, asi que el retroceso solo duraba UN
            # ciclo de control (50 ms) en vez de DELIVER_BACK_S. Se veia en la
            # traza como un -143 suelto seguido de ceros: el rover se quedaba
            # practicamente encima de la bandera recien soltada.
            if self._key_phase == "back":
                return Action(-config.ESCAPE_SPEED, 0)
            return Action()          # down / open: quieto mientras el servo va

        if self._key_phase == "down":
            self._key_phase = "open"
            self._phase_until = now + config.DELIVER_OPEN_S
            self.has_flag = False
            self.flag_delivered = True
            print(f"[bandera] SOLTANDO: pinza a {config.SERVO_GRIP_OPEN}")
            return Action(0, 0, [self._servo("grip", config.SERVO_GRIP_OPEN)])

        if self._key_phase == "open":
            self._key_phase = "back"
            self._phase_until = now + config.DELIVER_BACK_S
            return Action(-config.ESCAPE_SPEED, 0)

        self._key_phase = None
        self._resume = S.DONE
        self.request(S.DONE, "bandera entregada en la zona propia")
        return Action()

    def _done(self, sense):
        # Parado con el LED fijo. Se sale con "Parar" desde la pagina.
        return Action()

    # ---------------------------------------------------------------- llave
    def _deposit_key(self, sense):
        """Tirar la caja con el servo CONTINUO, y recoger el brazo.

            enter    (0 s: se para en seco al ver el amarillo)
            drop     gira KEY_DROP_ANGLE durante KEY_DROP_S     -> tira la caja
            wait     quieto KEY_WAIT_AFTER_DROP_S               -> la caja se asienta
            retract  gira KEY_RETRACT_ANGLE durante KEY_RETRACT_S -> recoge el brazo
            -> BACKOFF_KEY

        Cada giro lo para EL MCU cuando se acaba su tiempo (Actuators::spin);
        aqui solo se espera lo mismo mas un margen antes de la siguiente orden.

        POR QUE SE RECOGE EL BRAZO ANTES DE RETROCEDER, y no despues como se
        hacia con el servo de posicion: en pista, el brazo extendido se
        enganchaba al dar marcha atras. Recogido primero, no hay nada que
        enganchar. Y como la caja ya esta en el suelo y el brazo se aparta de
        ella al recogerse, no hay riesgo de volver a tocarla.
        """
        now = time.monotonic()
        if self._key_phase == "enter":
            if now < self._phase_until:
                return Action(config.SPEED_CREEP, 0)
            # Parado: TIRAR. Se marca DEPOSITADA en el instante en que el
            # servo arranca, no cuando termina la maniobra: si algo
            # interrumpe despues, la caja ya esta en el suelo.
            self._key_phase = "drop"
            self._phase_until = now + config.KEY_DROP_S + config.KEY_SPIN_MARGIN_S
            self.key_deposited = True
            self.key_drops += 1
            print(f"[llave] TIRANDO la caja: servo a {config.KEY_DROP_ANGLE} "
                  f"durante {config.KEY_DROP_S} s")
            return Action(0, 0, [("key_spin", (config.KEY_DROP_ANGLE,
                                               config.KEY_DROP_S))])

        # fase "drop": quieto mientras el servo gira y la caja cae
        if self._key_phase == "drop":
            if now < self._phase_until:
                return Action()
            # ESPERA PARADO antes de moverse. La caja acaba de caer delante
            # del rover y todavia esta rebotando y asentandose; arrancar en
            # ese instante la arrastra o la vuelca, y ocurre justo en el punto
            # ciego de delante.
            self._key_phase = "wait"
            self._phase_until = now + config.KEY_WAIT_AFTER_DROP_S
            print(f"[llave] TIRADA. Quieto {config.KEY_WAIT_AFTER_DROP_S} s "
                  f"antes de recoger el brazo.")
            return Action()

        # fase "wait": quieto del todo, y al final RECOGER el brazo
        if self._key_phase == "wait":
            if now < self._phase_until:
                return Action()
            self._key_phase = "retract"
            self._phase_until = (now + config.KEY_RETRACT_S
                                 + config.KEY_SPIN_MARGIN_S)
            print(f"[llave] RECOGIENDO el brazo: servo a "
                  f"{config.KEY_RETRACT_ANGLE} durante {config.KEY_RETRACT_S} s")
            return Action(0, 0, [("key_spin", (config.KEY_RETRACT_ANGLE,
                                               config.KEY_RETRACT_S))])

        # fase "retract": quieto mientras el brazo vuelve
        if now < self._phase_until:
            return Action()
        # Un borde que aparecio mientras se soltaba se atiende AHORA, en este
        # mismo ciclo: el escape del borde ya retrocede si el negro esta
        # delante, que es donde esta la caja, asi que no la pisa.
        if sense.any_black:
            self._begin_edge(sense)
            return Action()
        self._key_phase = "back"
        self._phase_until = now + config.KEY_BACKOFF_S
        self.request(S.BACKOFF_KEY, "llave depositada y brazo recogido: alejandose")
        return Action(-config.ESCAPE_SPEED, 0)

    def _backoff_key(self, sense):
        now = time.monotonic()
        if self._key_phase == "back":
            # La caja esta DELANTE: retroceder es lo unico que la aleja sin
            # pisarla. Girar encima la arrastraria.
            if now < self._phase_until:
                return Action(-config.ESCAPE_SPEED, 0)
            self._key_phase = "turn"
            self._phase_until = now + config.KEY_TURN_S
            # Ya NO se rearma aqui el servo: el brazo se recogio en DEPOSIT_KEY,
            # antes de dar marcha atras (con el de posicion se rearmaba en este
            # punto, y el brazo extendido se enganchaba al retroceder).
            return Action(0, config.ESCAPE_TURN)

        # fase "turn": media vuelta para irse en direccion contraria
        if now < self._phase_until:
            return Action(0, config.ESCAPE_TURN)
        self._key_phase = None
        self._resume = S.PATROL
        self.request(S.PATROL, "vuelta a patrullar; el amarillo es zona prohibida")
        return Action()

    # ---------------------------------------------------------- evitar zona
    def _begin_avoid(self):
        # A donde volver despues. Si la interrupcion llega DESDE otra
        # interrupcion (borde dentro de esquive), _resume ya trae el destino
        # bueno y no se pisa.
        if self.state not in (S.EDGE, S.AVOID_ZONE):
            self._resume = S.CARRY if self.has_flag else S.PATROL
        self._key_phase = "back"
        self._phase_until = time.monotonic() + config.AVOID_BACK_S
        self._avoid_dir *= -1           # alterna el lado de escape
        self.request(S.AVOID_ZONE, "amarillo con la llave ya depositada: rebotar")

    def _avoid_zone(self, sense):
        now = time.monotonic()
        if self._key_phase == "back":
            if now < self._phase_until:
                return Action(-config.ESCAPE_SPEED, 0)
            self._key_phase = "turn"
            self._phase_until = now + config.AVOID_TURN_S
        if now < self._phase_until:
            return Action(0, config.ESCAPE_TURN * self._avoid_dir)
        self._key_phase = None
        self.request(self._resume, "zona amarilla esquivada")
        return Action()

    # ---------------------------------------------------------------- borde
    def _begin_edge(self, sense):
        if self.state not in (S.EDGE, S.AVOID_ZONE):
            self._resume = S.CARRY if self.has_flag else S.PATROL
        self._edge_move, self._edge_dir = escape_from_edge(sense.effective_black_mask)
        self._edge_phase = "turn" if self._edge_move == 0 else "back"
        self._edge_until = time.monotonic() + (
            config.EDGE_TURN_S if self._edge_move == 0 else config.EDGE_BACK_S)
        self._key_phase = None          # cualquier maniobra de llave se abandona
        where = ("frente" if sense.front_black or sense.front_latched else "") + \
                ("+atras" if sense.rear_black or sense.rear_latched else "")
        self.request(S.EDGE, f"borde negro en {where or '?'} "
                             f"(mascara {sense.effective_black_mask:04b})")

    def _edge(self, sense):
        now = time.monotonic()
        if self._edge_phase == "back":
            if now >= self._edge_until and not sense.any_black:
                self._edge_phase = "turn"          # y se gira YA, este ciclo
                self._edge_until = now + config.EDGE_TURN_S
            else:
                return Action(self._edge_move * config.ESCAPE_SPEED, 0)

        if now >= self._edge_until:
            if sense.any_black:
                self._begin_edge(sense)
                return Action()
            self.request(self._resume, "borde superado")
            return Action()
        return Action(0, config.ESCAPE_TURN * self._edge_dir)

    def as_dict(self):
        return {"state": self.state, "prev": self.prev_state,
                "reason": self.reason, "elapsed_s": round(self.elapsed, 1),
                "key_deposited": self.key_deposited,
                "key_drops": self.key_drops,
                "key_phase": self._key_phase,
                "hunt_allowed": self._hunt_allowed(),
                "flag_seen": self.flag_seen,
                "has_flag": self.has_flag,
                "team_color": config.COLOR_NAMES[config.TEAM_COLOR],
                "flag_delivered": self.flag_delivered,
                "grab_phase": self._grab_phase,
                "resume": self._resume,
                "led": self._led_mode,
                # La pagina dibuja un boton por cada uno: la lista sale de
                # aqui, no del HTML, para que no se desincronicen.
                "forceable": list(self.FORCEABLE),
                "moving": list(self._MOVING)}
