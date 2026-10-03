"""
config.py - Todos los numeros ajustables de la parte 1 (lado MPU).

Regla: si un numero aparece en la logica, esta mal. Va aqui.

CALIBRACION HEREDADA de las pruebas anteriores (mismos sensores, mismos
pines, mismo montaje):
  TCRT   escala 0..4095, polaridad invertida en el sketch, umbral 3500
  APDS   ganancia 4X, integracion 10 ms, LED blanco: clear <40 negro, >170 blanco
Los umbrales de amarillo/rojo/azul NO se llegaron a medir: esta parte sirve
para medirlos (ver README, "Calibrar los colores").
"""

# =================================================================== CONTROL
CONTROL_HZ    = 20.0
CONTROL_DT    = 1.0 / CONTROL_HZ
SENSE_POLL_HZ = 20.0           # telemetria del MCU (hilo aparte)
# 10 Hz y no 5: la pagina es el instrumento de medida de esta parte, y a 5 Hz
# cada cambio esperaba hasta 200 ms solo para dibujarse. Ademas main.py fuerza
# un envio INMEDIATO en cuanto cambia el color, asi que el periodo solo manda
# para los valores continuos, no para los eventos.
UI_PUSH_HZ    = 10.0           # refresco de la pagina
MCU_WATCHDOG_MS = 400          # = CMD_TIMEOUT_MS del sketch (solo diagnostico)
assert CONTROL_HZ >= 5.0, "El watchdog del MCU (400 ms) exige >= 5 Hz"

# Linea [mcu] periodica con los contadores del enlace (ordenes enviadas
# contra recibidas, loop() mas lento del MCU). 0 = callada.
# 1.0 y no 2.0: la ventana con que se calculan las tasas es de 1 s, asi que
# imprimir cada 2 s descartaba una ventana de cada dos -- y el fallo podia
# estar justo en la que no se veia.
DIAG_MCU_S = 1.0

# drive() por call() en vez de notify().
#
# notify() no espera respuesta, asi que el MPU puede adelantarse y las ordenes
# se acumulan en el transporte: al MCU le llegan AGRUPADAS. El total cuadra y
# ningun lazo se atasca, pero entre rafaga y rafaga el MCU se queda sin
# ordenes y su watchdog salta. Es el unico caso que encaja con "ninguno de los
# dos extremos se paso del limite y aun asi salta".
#
# call() espera respuesta: no se manda la siguiente orden hasta que la
# anterior llego, con lo que el agrupamiento desaparece. Cuesta la latencia de
# ida y vuelta dentro del ciclo (unos ms de los 50 disponibles).
#
# Ponlo a True si el diagnostico dice que las ordenes llegan en rafagas.
DRIVE_USE_CALL = False

# =============================================== VELOCIDAD GLOBAL DEL ROVER
# Un solo mando para "que vaya mas despacio". 1.0 = como estaba, 0.75 = tres
# cuartos, 0.5 = la mitad. TODAS las velocidades y giros de mas abajo salen de
# multiplicar por esto, asi que no hay que tocarlas una a una ni se puede
# olvidar ninguna.
SPEED_SCALE = 0.55

# POR QUE ESTO SI ES UN 0.75 DE VERDAD. Las velocidades van normalizadas
# 0..1000 y el MCU las convierte a PWM asi:
#
#     pwm = MOTOR_DEADZONE_PWM + (MOTOR_MAX_PWM - MOTOR_DEADZONE_PWM) * v / 1000
#
# MOTOR_DEADZONE_PWM es el PWM al que el motor EMPIEZA a girar. O sea que la
# rueda se mueve a una velocidad proporcional a (pwm - zona_muerta), que es
# proporcional a v. Por eso multiplicar v por 0.75 da ~0.75 de velocidad real
# y no una fraccion rara. (Cerca de cero la linealidad es peor: el rozamiento
# estatico es mayor que el dinamico. Ver SPEED_MOVE_FLOOR.)


def _spd(v):
    """Velocidad o giro escalado. Se redondea a entero: el MCU los quiere asi."""
    return int(round(v * SPEED_SCALE))


def _dur(t):
    """Duracion de un AVANCE o RETROCESO en linea recta, compensada.

    OJO: SOLO para movimiento RECTO. Los GIROS ya no pasan por aqui, y la
    razon esta en la nota "POR QUE LOS GIROS YA NO SE COMPENSAN" de la seccion
    de patrulla. En resumen: en un pivote sobre el eje la relacion entre
    velocidad y grados por segundo NO es proporcional (a mas velocidad, mas
    patinaje), asi que compensar por tiempo se pasa de largo.

    ESTO ES LO QUE SE OLVIDA Y ROMPE EL COMPORTAMIENTO. Las maniobras a ciegas
    -girar 180 grados, retroceder 15 cm- no miden nada: recorren
    velocidad x tiempo. Si se baja la velocidad al 75 % y se deja el tiempo
    igual, el giro de 180 grados pasa a ser de 135 y el retroceso se queda
    corto. Dividir entre el factor mantiene el ANGULO y la DISTANCIA, que es
    lo que estaba calibrado; lo unico que cambia es que se tarda mas.

    NO se aplica a los tiempos que no son movimiento (recorrido de un servo,
    caducidad de una deteccion): esos no dependen de lo rapido que ande el
    rover.
    """
    return round(t / SPEED_SCALE, 2)


# Por debajo de esta velocidad normalizada el rover NO SE MUEVE: zumba y se
# queda parado, porque el PWM resultante no vence el rozamiento estatico.
# main.py avisa al arrancar si alguna velocidad escalada queda por debajo.
# NO ESTA MEDIDO. Para medirlo: teleop o patrulla bajando de 10 en 10 hasta
# que deje de arrancar desde parado, y pon aqui ese valor + 20 de margen.
SPEED_MOVE_FLOOR = 110

# ============================================================ TCRT (BORDE)
# Se mandan al MCU al arrancar (cfg_lines). Convencion normalizada:
# menos = mas oscuro, escala 0..4095.
#
# VALIDADO EN PISTA (22/09): 2000. Con este umbral el rover no se sale y no
# frena donde no debe.
#
# AVISO: LAS MEDIDAS DE 03/09 YA NO DESCRIBEN ESTE HARDWARE. Aquellas decian
# (normalizado) peor negro 3108 y peor no-negro 3908, y con esos numeros un
# umbral de 2000 no detectaria NINGUN negro -- 3108 no esta por debajo de
# 2000. Que en pista funcione significa que las lecturas han cambiado: otra
# altura de montaje, otra cinta, otra luz, u otro modulo.
#
# Consecuencia practica: el analisis de margenes de mas abajo esta OBSOLETO y
# no sirve para decidir nada. Si hay que retocar esto, NO uses aquellos
# numeros: vuelve a medir con el boton "Leer TCRT" de la pagina, sobre blanco
# y sobre negro, y pon el umbral en el punto medio. Son dos minutos y evita
# ajustar a ciegas el dia de la competencia.
#
# --- historico, NO vigente (03/09) --------------------------------------
# Crudos: blanco 141..177, negro 987..1014, cintas de color 172..181.
# Normalizado: peor negro 3108, peor no-negro 3908.
#     umbral   margen sobre el peor NEGRO   margen bajo el no-negro mas bajo
#     3500              392                            408
#     3200               92                            708
# El intercambio no es simetrico: un falso negro para el rover donde no debia
# (molesto); un negro perdido lo saca de la pista (ronda perdida). Sigue
# siendo cierto como CRITERIO aunque los numeros ya no valgan.
LINE_BLACK_BELOW = 2000
LINE_HYSTERESIS  = 120
LINE_ADC_MAX     = 4095        # = LINE_ADC_MAX del sketch (12 bits)

# ====================================================== APDS (COLOR DE ZONA)
# MEDIDO con 4X / 10 ms / LED blanco montado.
#
# COLOR_DARK_CLEAR VALIDADO EN PISTA (22/09): 2. Ha ido 40 -> 18 -> 2, y el
# recorrido tiene sentido: cada bajada resolvio una cinta que se estaba
# clasificando como NEGRO en vez de como su color.
#
# Con 2, el "negro por color" practicamente no dispara nunca -- y ESO ESTA
# BIEN, porque del borde negro se encargan los TCRT, no el APDS. El APDS solo
# tiene que distinguir amarillo / rojo / azul / blanco, y cualquier umbral de
# oscuridad que se coma una de esas cintas hace mas dano que bien.
#
# Sospecha de por que hizo falta AHORA y no antes: el AZUL es la cinta que
# menos luz devuelve, y la zona propia azul (TEAM = "blue") no se leia hasta
# la parte 4. Con 18, el azul caia por debajo y se clasificaba como negro, asi
# que el rover nunca reconocia su casa. Encaja con lo que se vio en pista.
COLOR_DARK_CLEAR   = 2       # clear por debajo -> negro   (MEDIDO 22/09)
COLOR_BRIGHT_CLEAR = 170     # clear por encima (y sin tinte) -> blanco

# Clasificacion por PROPORCIONES r/g/b (invariantes a la cantidad de luz).
# PROVISIONALES: se ajustan con las lecturas de cada cinta (README).
# VALIDADAS EN PISTA (03/09): con estos valores las cuatro cintas
# (negro, amarillo, azul, rojo) y el blanco se identifican correctamente.
COLOR_NEUTRAL_SPREAD = 0.12  # max - min de las fracciones para llamarlo neutro
COLOR_DOMINANT       = 0.42  # fraccion minima del canal dominante (rojo/azul)
YELLOW_MAX_BLUE      = 0.24  # amarillo: azul hundido...
YELLOW_MIN_RED       = 0.30  # ...y rojo y verde altos a la vez
YELLOW_MIN_GREEN     = 0.30

# SI EL MARGEN DEL NEGRO SE QUEDA CORTO: subir APDS_GAIN en sketch/config.h de
# APDS9960_AGAIN_4X a 16X multiplica TODAS las lecturas por ~4. El negro y la
# cinta mas oscura mantienen su relacion, pero la separacion pasa de unas
# pocas cuentas a cuatro veces mas, y el ruido pesa cuatro veces menos.
# Coste: hay que RE-MEDIR COLOR_DARK_CLEAR y COLOR_BRIGHT_CLEAR, porque
# escalan con la ganancia (18 y 170 valen solo para 4X).

# Filtro de estabilidad: una clase se da por buena cuando se repite N lecturas
# seguidas (a 20 Hz, 3 = 150 ms). Sin esto la pagina parpadea con cada
# lectura ruidosa al cruzar el borde de una cinta.
# ESTE NUMERO TIENE UN COSTE EN CENTIMETROS, no solo en parpadeo.
#
# El filtro exige N lecturas seguidas antes de dar un color por bueno. Con el
# APDS a 25 ms, N=2 son ~50 ms; N=3 eran ~150 ms. Y 150 ms a 25 cm/s son
# 3,7 cm recorridos: MAS ANCHO QUE LA CINTA. O sea que con N=3 el rover podia
# cruzar una linea de color entera sin llegar a declararla nunca, y el fallo
# solo se ve moviendose, no probando a mano.
#
# N=2 deja el margen justo (unos 1,2 cm a esa velocidad). Si aparecen colores
# fantasma en el borde de una cinta, sube a 3 y baja la velocidad; si se pasa
# cintas de largo, baja APDS_POLL_MS en el sketch antes que tocar esto.
COLOR_STABLE_N = 2

# EL DEPOSITO DE LA LLAVE NO ESPERA AL COLOR ESTABLE. Con True basta UNA
# lectura de amarillo para empezar la secuencia.
#
# EL FALLO QUE ARREGLA: en pista la pagina mostraba el amarillo apareciendo un
# instante y el rover no depositaba nada. La causa es este mismo filtro: el
# deposito miraba el color ESTABLE, que exige N lecturas seguidas, y cruzando
# la cinta solo daba tiempo a una. El color instantaneo cambiaba (por eso se
# veia en la pagina) pero el estable no llegaba a cuajar nunca.
#
# EL INTERCAMBIO, QUE NO ES SIMETRICO:
#   amarillo perdido -> la llave no se deposita NUNCA y la ronda no empieza
#   amarillo falso   -> la llave cae donde no debia
# Lo primero es seguro; lo segundo es improbable (el amarillo exige rojo Y
# verde altos con el azul hundido: ni el blanco ni el negro ni el rojo ni el
# azul se le parecen) y ademas solo puede pasar UNA vez por ronda.
#
# El resto del sistema sigue usando el color estable: esto es SOLO el disparo
# del deposito. Si en pista aparecen depositos fantasma, ponlo en False.
ZONE_TRIGGER_INSTANT = True

C_UNKNOWN, C_WHITE, C_BLACK, C_YELLOW, C_RED, C_BLUE = 0, 1, 2, 3, 4, 5
COLOR_NAMES = {0: "?", 1: "blanco", 2: "negro", 3: "amarillo", 4: "rojo", 5: "azul"}

# ================================================================= LLAVE
# PARTE 2: depositar la llave en la zona amarilla (neutra).
#
# Secuencia: PATROL ve amarillo -> DEPOSIT_KEY (para, TIRA la caja girando el
# servo, espera a que caiga, RECOGE el brazo) -> BACKOFF_KEY (retrocede, gira
# ~180) -> PATROL.
#
# La caja cae DELANTE del rover (confirmado): por eso se retrocede para salir.

SERVO_KEY        = 2       # id del servo en el MCU (0 elevacion, 1 gripper, 2 llave)

# SERVO DE LA CAJA: DE ROTACION CONTINUA (360) DESDE EL 28/09.
#
# Ya no hay "angulo de retener" ni "angulo de soltar": un servo continuo no va
# a una posicion, gira a una VELOCIDAD. El "angulo" que se le manda es sentido
# y velocidad (90 ~ parado; cuanto mas lejos de 90, mas rapido hacia ese
# lado), y el movimiento es ese giro DURANTE UN TIEMPO. El MCU cuenta el tiempo
# y se para solo (Actuators::spin), igual que en la App de prueba servo-test.
#
# VALORES DEL 28/09 (segunda ronda, sustituyen a 85/100):
KEY_DROP_ANGLE    = 70     # TIRAR la caja: sentido A (20 del neutro)
KEY_DROP_S        = 0.3
KEY_RETRACT_ANGLE = 115    # RECOGER el brazo: sentido B (25 del neutro)
KEY_RETRACT_S     = 0.3
#
# ASIMETRIA: 70 esta a 20 grados del neutro y 115 a 25, con el mismo tiempo.
# Un servo continuo no gira igual en los dos sentidos -- el neutro real no
# esta en 90 clavado --, asi que la vuelta casi nunca es el espejo exacto de
# la ida. Si el brazo vuelve DE MAS (choca con el tope) o DE MENOS (queda
# asomado), se corrige el TIEMPO de recoger, no el angulo.
#
# Lejos del neutro el servo es mucho menos sensible al voltaje que a 85, pero
# tambien GIRA BASTANTE MAS en los mismos 0.3 s: si la caja sale disparada o
# el brazo golpea el tope, se acorta el tiempo (0.2, 0.25).

# Margen que espera la FSM despues de cada giro, sobre la duracion del propio
# giro, antes de pasar a lo siguiente. El giro lo para el MCU; esto es solo
# para no mandar la siguiente orden con el servo todavia en marcha.
KEY_SPIN_MARGIN_S = 0.15

# APAGADO AUTOMATICO DE LOS SERVOS (detach). Un servo solo esta encendido
# mientras se mueve: se le manda el angulo, se le dan SERVO_SETTLE_S para que
# el eje llegue, y se le corta el tren de pulsos.
#
# POR QUE. Un ZOSKAY de 20 kg parado contra su tope no sujeta mejor por estar
# alimentado: zumba, se calienta y hunde el rail del UBEC durante toda la
# ronda. Sin pulsos deja de empujar y el reductor lo retiene solo. Es
# exactamente lo que hacen los botones "Apagar" de la pagina, que ya estaban
# probados; esto lo aplica solo, sin tener que pulsarlos.
#
# Se re-engancha con cualquier orden nueva (el MCU hace attach antes de write).
#
# UN INTERRUPTOR POR SERVO, y no uno global, por el gripper: ver el aviso.
SERVO_AUTO_OFF = {
    # La CAJA no esta aqui: es un servo continuo y se para SOLA en el MCU al
    # terminar su giro (Actuators::spin). No necesita apagado desde el MPU.
    "lift": True,   # el brazo arriba lo aguanta el reductor
    "grip": True,   # <-- LEE EL AVISO DE ABAJO ANTES DE DEJARLO EN True
}

# SUJETAR NO ES MOVERSE. Las ordenes que sujetan algo se marcan con
# keep_on=True en Mission._servo() y NO programan apagado:
#     pinza  cerrada   -> segun SERVO_AUTO_OFF["grip"] (ver aviso)
#     pinza  abierta   -> se apaga
#     brazo            -> siempre se apaga
# (Antes la llave "retenia" a 180 con keep_on. Con el servo continuo eso ya no
# existe: la llave se sujeta sola con el reductor, sin pulsos.)

# AVISO SOBRE EL GRIPPER. Los otros dos servos apagados no pierden nada. El
# gripper SI: si el reductor no aguanta el cierre por si solo, apagarlo SUELTA
# LA BANDERA, y no hay sensor que avise -- el rover seguiria su camino creyendo
# que la lleva.
#
# LA PRUEBA, un minuto: agarra la bandera con la pinza, pulsa "Apagar pinza" en
# la pagina, y tira suavemente de la bandera hacia arriba.
#   - Si aguanta: dejalo en True y el rover se ahorra la corriente de bloqueo.
#   - Si se suelta o cede: pon "grip": False. Entonces la pinza queda alimentada
#     mientras lleve la bandera, que es el precio de no tener un cierre
#     mecanico.
# Mientras no hagas esa prueba, el riesgo esta acotado: RETURN_AFTER_GRAB es
# False, asi que la secuencia termina parada en HOLDING y lo ves enseguida.

# Cuanto se espera desde la orden antes de cortar los pulsos. write() es
# asincrono y el eje tarda en llegar: apagar antes lo deja a medio camino.
#
# POR SERVO, Y NO UNO PARA LOS TRES. El tiempo depende de los GRADOS que
# recorre cada uno y de la carga que empuja:
#
#     gripper      0 -> 90  =  90 grados, cerrando sobre la bandera
#     elevacion    0 -> 40  =  40 grados, subiendo la bandera
#
# (La llave ya no esta aqui: su servo es continuo y el MCU lo para solo.)
SERVO_SETTLE_S   = 0.8                  # por defecto
SERVO_SETTLE_BY_ID = {
    "grip": 0.9,
    "lift": 0.8,
}


def settle_s(name):
    return SERVO_SETTLE_BY_ID.get(name, SERVO_SETTLE_S)

# AVANCE DENTRO DE LA ZONA antes de soltar. AHORA EN CERO: el rover SE PARA
# EN SECO en cuanto el APDS ve amarillo, y suelta ahi mismo.
#
# POR QUE ERA 0.8 (que con el factor 0.55 se convertia en 1.45 s de avance).
# El APDS avisa cuando EL SENSOR pisa el amarillo, o sea con el frente del
# rover en el BORDE de la zona; la caja cae delante del frente, asi que
# entrar un poco deberia hacer que quedara dentro.
#
# POR QUE AHORA ES 0. En pista se vio lo contrario de lo previsto: con 1.45 s
# de avance el rover marcaba la llave como depositada y "seguia de largo" --
# recorria la zona entera y soltaba fuera, o ya saliendo. El avance a ciegas
# resulto mayor que la propia zona.
#
# EL INTERCAMBIO, QUE NO ES GRATIS. Parar en el instante de ver el amarillo
# significa parar en el BORDE de la zona, y la caja cae DELANTE del rover: si
# el frente esta justo en la linea, la caja puede quedar FUERA. Es el error
# contrario al que teniamos.
#
# COMO AJUSTARLO: mira donde cae la caja.
#   - cae dentro            -> dejalo en 0
#   - cae sobre la linea o fuera POR DELANTE -> sube de 0.1 en 0.1
#   - vuelve a pasarse de largo -> bajalo
# Es un numero, se cambia sin recompilar, y una prueba por intento basta.
ZONE_ENTER_ADVANCE_S = 0.0
SPEED_CREEP          = _spd(170)   # velocidad de entrada en la zona

# ESPERA PARADO DESPUES DE TIRAR LA CAJA, antes de recoger el brazo y de
# moverse. Es aparte de KEY_DROP_S a proposito: aquel es el GIRO DEL SERVO,
# este es que la caja termine de caer, de rebotar y de quedarse quieta. Si el
# rover arranca mientras la caja aun se esta asentando, la arrastra o la vuelca
# con el propio movimiento -- y eso pasa justo delante de el, donde no lo ve.
KEY_WAIT_AFTER_DROP_S = 1.0
KEY_BACKOFF_S   = _dur(1.0)  # retroceso tras soltar (COMPENSADO: es distancia,
                             # y aqui SI importa: hay que alejarse de la caja)
KEY_TURN_S      = 1.45   # ~180 grados a 124 grados/s (ver la tabla de la
                         # seccion PATRULLA). SIN compensar: con el factor a
                         # 0.55 la compensacion lo llevaba a 2.9 s, que a esa
                         # velocidad son casi DOS vueltas enteras.
                         # Medir: cronometra un giro completo a ESCAPE_TURN y
                         # divide por dos. Si se queda corto, el rebote en
                         # amarillo (AVOID_ZONE) lo corrige igual.

# Rebote en amarillo despues de depositar: mismo esquema que el borde negro.
AVOID_BACK_S    = 0.50   # bajado, igual que EDGE_BACK_S
AVOID_TURN_S    = 0.73   # ~90 grados. SIN compensar (ver PATRULLA).

# ==================================================== SERVOS: ANGULOS MEDIDOS
# Medidos fisicamente en el rover.
#
#   Elevacion del gripper   0 (abajo)  ->  40 (arriba)
#   Gripper                 0 (abierta) ->  90 (cerrada)
#   Caja (llave)            servo CONTINUO: ver KEY_DROP_* / KEY_RETRACT_*
SERVO_LIFT       = 0       # id del servo en el MCU (enum Id de actuators.h)
SERVO_GRIP       = 1
SERVO_LIFT_DOWN  = 0
SERVO_LIFT_UP    = 40
SERVO_GRIP_OPEN  = 0       # CONFIRMADO 04/09: 0 abierto, 90 cerrado
SERVO_GRIP_CLOSE = 90

# ================================================================== VISION
# PARTE 3: buscar la bandera rival con la webcam USB.
#
# La camara la abre el periferico de App Lab (arduino.app_peripherals.camera),
# que elige sola la primera webcam USB y entrega frames BGR. Sin indices de
# /dev/video que adivinar: en el UNO Q, video0 y video1 son el decodificador
# Venus del Qualcomm, no la camara, y el indice de la webcam cambia al
# reiniciar.
TEAM  = "blue"                         # "red" o "blue": equipo AL ARRANCAR.
                                       # El boton de A4 lo alterna fuera de
                                       # ronda (ver set_team, al final).
                                       # De aqui salen TRES cosas: que bandera
                                       # perseguir (ENEMY), que cinta es casa
                                       # (TEAM_COLOR) y de que color se
                                       # enciende el LED RGB. Un solo sitio.
TEAM_LED = 1 if TEAM == "blue" else 0  # lo que entiende el MCU: 0 rojo, 1 azul
ENEMY = "blue" if TEAM == "red" else "red"   # el color de la bandera a buscar

# INTERRUPTOR DE DIAGNOSTICO. Con False la camara no se abre y no hay hilo de
# vision: si un problema del enlace con el MCU desaparece al ponerlo a False,
# la vision es la causa (CPU/GIL); si sigue igual, no lo es. Es la prueba A/B
# mas barata que existe para separar las dos cosas.
CAMERA_ENABLED = True

CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480
CAMERA_FPS    = 10          # la deteccion no necesita mas y la CPU lo agradece
CAMERA_CODEC  = "MJPG"      # sin esto el USB no da fps por encima de 640x480

# La deteccion se hace sobre el frame REDUCIDO a esta escala (0.5 = 320x240):
# cuatro veces menos pixeles que procesar en cada etapa (desenfoque, HSV,
# morfologia, contornos). Una bandera de 5x15 cm sigue siendo decenas de
# pixeles de alto a media pista; se pierde alcance en el extremo lejano, no
# precision. Las coordenadas se devuelven en pixeles del frame COMPLETO.
DETECT_SCALE = 0.5

# NO hay video en vivo: la pagina no lo necesita y codificar JPEG en cada
# frame era trabajo tirado. Queda una foto BAJO DEMANDA en /api/snapshot, que
# solo cuesta cuando la pides y sirve para calibrar los rangos HSV (ver como
# recuadra la bandera de verdad, no solo sus numeros).
SNAPSHOT_JPEG_QUALITY = 80

# Distancia focal en pixeles. MEDIDA 04/09 con esta camara y esta bandera:
#     h_px = 98 px con la bandera a 85 cm  ->  FOCAL_PX = 98 * 85 / 15 = 555
# (a 100 cm no llegaba a detectarla; ver la nota de alcance mas abajo).
#
# LO QUE CAMBIA RESPECTO AL 700 INVENTADO. La distancia escala con la focal,
# asi que un 26% de error en la focal es un 26% de error en cada distancia:
#     a 14 cm reales, con 700 el rover creeria estar a 17.6 cm
#     y por tanto dispararia el agarre a 11.1 cm reales, 2.9 cm ENCIMA de la
#     bandera. Con la pinza cerrandose ahi, la tira en vez de agarrarla.
#
# LIMITES QUE SALEN DE ESTA FOCAL (utiles para leer la pagina):
#     por debajo de 23.1 cm el alto ya no cabe -> la distancia sale del ANCHO
#     por debajo de  7.2 cm el ancho pasa de FLAG_MAX_WIDTH_FRAC -> se rechaza
FOCAL_PX       = 555.0
FLAG_HEIGHT_CM = 15.0     # solo para la puntuacion (el aspecto ideal, 3.0)
FLAG_DIAM_CM   = 5.0      # esta es la que mide: ver FLAG_DIST_A

# ======================================================== DISTANCIA (ANCHO)
# La distancia sale SOLO del ancho de la bandera. Ni el alto ni la altura de
# 15 cm entran en el calculo. Dos razones, y la segunda es la de fondo:
#
#   1. Geometria. Con el cuarto superior borrado por el horizonte quedan
#      360 px utiles: el alto se satura a partir de ~23 cm y la distancia se
#      queda clavada. El ancho no se satura hasta ~7 cm.
#   2. Fisica. La bandera es un CILINDRO: su diametro son 5 cm se mire desde
#      donde se mire. Su "altura" son 15 cm solo si se ven el borde de arriba
#      Y el de abajo, y la base queda tapada por el suelo, la sombra o el
#      propio borde del encuadre. El alto es la dimension que miente.
#
# EL MODELO TIENE DOS TERMINOS, NO UNO:
#
#       d = FLAG_DIST_A / w_px + FLAG_DIST_B
#
# B es un DESPLAZAMIENTO CONSTANTE en cm. Existe porque el modelo de camara
# mide desde el CENTRO OPTICO de la lente y las distancias utiles se miden
# desde el frente del rover o desde la pinza. Esa diferencia es un numero
# fijo, y sin B NO HAY NINGUNA FOCAL QUE CUADRE EN TODO EL RANGO:
#
#       con un desfase real de 5 cm, calibrando a 85 cm el error es del 6 %
#       (no se nota) y a 14 cm es del 36 % (el agarre falla)
#
# Es justo el sintoma de "cambio la focal y sigue sin cuadrar de cerca".
#
# COMO SE CALIBRA (dos medidas, cinta metrica, tres minutos):
#   1. Bandera a una distancia LEJOS (~80 cm). Anota la distancia real y el
#      "ancho w_px" que muestra la pagina.
#   2. Lo mismo CERCA (~20 cm).
#   3. python3 tools/calib_dist.py 80 <w1> 20 <w2>
#      -> imprime las dos lineas para pegar aqui.
# Mide siempre desde el MISMO sitio del rover (mejor la pinza, que es lo que
# tiene que llegar a la bandera): B absorbe cual sea, pero tiene que ser
# siempre el mismo.
FLAG_DIST_A    = 2775.0    # VALIDADO en pista 04/09 (= focal 555 x 5 cm)
FLAG_DIST_B    = 0.0       # VALIDADO en pista: el desfase resulto no importar
                           # en el rango util. Si algun dia deja de cuadrar de
                           # cerca, es lo primero a recalibrar (dos puntos).

# Mediana de las ultimas N lecturas de distancia. El ancho es un rasgo TRES
# VECES MAS PEQUENO que el alto, asi que un pixel de error en la mascara pesa
# el triple: a 100 cm son ~28 px y +-2 px ya son +-7 %. La mediana tira los
# saltos sueltos (un frame con la mascara partida) sin apenas retardo:
# 3 lecturas a 10 fps son 0.3 s. 1 = sin filtro.
FLAG_DIST_MEDIAN_N = 3

# ...PERO SOLO CUANDO HACE FALTA. El filtro existe porque de lejos el ancho es
# diminuto: a 100 cm son ~28 px y +-2 px son +-7 %. De cerca es al reves: a
# 6 cm el ancho son 462 px y esos mismos +-2 px son +-0.4 %. Ahi el filtro no
# aporta nada y su retardo (0.3 s) es puro riesgo, justo en el tramo donde
# solo hay 1.7 cm de margen.
#
# Por encima de este ancho en pixeles, la lectura se usa CRUDA.
FLAG_DIST_TRUST_PX = 120.0     # ~23 cm con A = 2775

# Rangos HSV de OpenCV (H 0-179, S/V 0-255).
# MEDIDOS EN EL BANCO DE CAMARA (04/09) con la bandera real y la luz de
# trabajo. El rojo cruza el 0: por eso lleva dos tramos.
HSV_RED  = [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (179, 255, 255))]
HSV_BLUE = [((95, 110, 60), (130, 255, 255))]

# Filtro geometrico: separa la BANDERA (alta y estrecha) de la CINTA del suelo
# (ancha y aplastada), que es del mismo color. Sin esto el rover persigue las
# lineas de la pista.
#
# VALORES DEL BANCO (04/09). Y UNA ADVERTENCIA QUE HAY QUE LEER:
#
# FLAG_MIN_ASPECT paso de 1.5 a 0.1, y FLAG_MAX_WIDTH_FRAC de 0.45 a 0.6. Eso
# es abrir de par en par los dos filtros que existen SOLO para rechazar la
# cinta del suelo. Reproducido en el banco con escenas sinteticas:
#
#   mancha roja de 260x60 px (aspecto 0.23, 41% del ancho de la imagen)
#     con 1.5 / 0.45  -> RECHAZADA: "aspecto 0.23 < 1.5 (mancha ANCHA)"
#     con 0.1 / 0.60  -> ACEPTADA COMO BANDERA, puntuacion 0.50 > 0.35
#                        (relleno 0.96 x aspecto 0.52 x area 1.00)
#
# Es decir: con esta configuracion un trozo de cinta roja visto de frente
# puede pasar por bandera. En APPROACH eso significa que el rover conduce
# hacia una linea del suelo. NO esta comprobado que ocurra con la cinta real
# (depende de a que angulo y a que distancia la vea la camara): esta
# comprobado que el filtro ya no lo impide.
#
# ACTUALIZACION 04/09: FLAG_MAX_WIDTH_FRAC subio a 1.0. Es NECESARIO para
# llegar a 6 cm -- ahi la bandera ocupa el 72 % del encuadre y con 0.6 se
# rechazaba justo cuando hay que agarrarla. Pero con eso YA NO QUEDA NINGUN
# FILTRO GEOMETRICO: min_aspect 0.1 y max_width 1.0 dejan pasar cualquier
# mancha del color rival que este bajo el horizonte.
#
# HAY UNA SALIDA QUE NO CUESTA NADA, y son los numeros de esta tabla. Como
# el alto se recorta al horizonte, el aspecto de la BANDERA nunca baja de
# ~0.6 aunque este pegada:
#
#     distancia   ancho px   alto px (recortado)   aspecto
#        40 cm        69            208             3.01
#        20 cm       139            360             2.59
#        10 cm       278            360             1.30
#         6 cm       462            360             0.78   <- distancia de agarre
#         4.5 cm     617            360             0.58
#
#     cinta del suelo, tipica                       0.10 - 0.30
#
# FLAG_MIN_ASPECT = 0.5 conserva la bandera en TODO el rango util (hasta
# 4.5 cm, muy por debajo de los 6 de agarre) y rechaza la cinta. Es un solo
# numero y no toca nada mas. NO lo he puesto porque tus 0.1 estan medidos en
# pista y los mios no; pruebalo cuando quieras comprobar la cinta.
FLAG_MIN_AREA_PX    = 400
FLAG_MIN_ASPECT     = 0.1      # alto/ancho; la bandera real es 15/5 = 3.0
FLAG_MAX_ASPECT     = 7.0
FLAG_MIN_FILL       = 0.45     # area del contorno / area de su caja
FLAG_MAX_WIDTH_FRAC = 1.0      # DESACTIVADO. Ver el aviso de abajo.
HORIZON_FRAC        = 0.25     # se ignora el cuarto superior (techo, focos, publico)
MIN_CONFIDENCE      = 0.35

# Area (en px del frame COMPLETO) a partir de la cual el factor de area vale
# 1.0. Por debajo, la puntuacion baja proporcionalmente. ES LO QUE LIMITA EL
# ALCANCE, y hasta ahora era un 4000 escrito a mano dentro de vision.detect().
#
# Con FOCAL_PX = 555 la bandera cubre 4000 px a 74 cm: mas lejos de eso el
# factor de area empieza a restar y la deteccion se cae sola aunque la mascara
# este perfecta. Coincide con lo observado: a 85 cm si, a 100 cm no.
#
# BAJARLO AMPLIA EL ALCANCE Y **NO** AYUDA A LA CINTA DEL SUELO, que es lo que
# normalmente frena estos aflojes: una cinta ocupa 15000 px o mas, su factor de
# area ya vale 1.0 y bajar el umbral no puede subirlo. Es de los pocos ajustes
# que solo tienen un lado.
#     1500 -> full marks a ~121 cm      1000 -> ~148 cm
# El banco tiene un deslizador para esto: mide antes de decidir.
# BAJADO de 4000 a 1500 en pista (18/09): con 4000 la bandera solo puntuaba
# pleno dentro de 74 cm, y mas lejos el factor de area hundia la deteccion
# aunque la mascara fuera perfecta -- por eso, tras dejar la llave, el rover
# no iba a por una bandera lejana: simplemente NO LA VEIA.
#     1500 -> puntuacion plena hasta ~121 cm
# Y no favorece a la cinta del suelo: una cinta ocupa 15000 px o mas, su
# factor de area ya valia 1.0 y bajar el umbral no puede subirlo.
FLAG_NORM_AREA_PX   = 1500.0

# Cuanto castiga la PUNTUACION que el aspecto se aleje del ideal (3.0), como
# error RELATIVO: factor = 1 / (1 + error_relativo * FLAG_ASPECT_TOL).
# 1.0 = suave, 3.0 = severo, 0 = no castiga nada.
#
# Existe porque la version anterior usaba el error ABSOLUTO sin tolerancia:
# 1/(1+|aspecto-3|). Una bandera perfecta con aspecto 1.40 daba factor 0.38 y
# la puntuacion se hundia a 0.36 -> rechazada con la mascara impecable. Desde
# fuera parecia un problema de umbrales de color; era la formula.
FLAG_ASPECT_TOL     = 1.0
DETECTION_STALE_S   = 0.7      # una deteccion mas vieja se descarta
LOST_TARGET_S       = 1.5      # sin verla este tiempo en APPROACH -> se abandona

# ============================================================ PERSECUCION
# CERROJO DEL REGLAMENTO: "si un robot busca la bandera antes de depositar la
# caja, pierde la ronda". Ver la bandera es pasivo y se senaliza siempre;
# PERSEGUIRLA solo con la llave depositada. Para probar la camara en el banco
# sin depositar, pon esto a False... y vuelve a ponerlo a True.
FLAG_REQUIRES_KEY = True

SPEED_APPROACH   = _spd(260)
APPROACH_SLOW_CM = 60.0    # a partir de aqui, de SPEED_APPROACH a SPEED_CREEP
GRAB_DISTANCE_CM = 6.0     # MEDIDO 04/09: es donde la bandera queda mejor
                           # colocada para que la pinza la abrace.
                           #
                           # LIMITE FISICO DEL SENSOR, para tenerlo presente:
                           # con A = 2775 el ancho llena los 640 px a 4.3 cm.
                           # Por debajo de ahi el ancho se satura y la
                           # distancia se queda clavada, igual que le pasaba
                           # al alto. Entre 6 y 4.3 hay 1.7 cm de margen: es
                           # poco, y es la razon del arrastre final de abajo.

# ACERCAMIENTO FINAL. Por debajo de esta distancia el rover va MUCHO mas
# despacio, y no es un capricho:
#
#   con 1.7 cm de margen antes de que el sensor se sature, cualquier retardo
#   en el lazo se come el margen entero. A velocidad de creep un solo ciclo
#   de control (50 ms) ya recorre varios milimetros, y el filtro de mediana
#   anadia 0.3 s de retardo = varios centimetros. El rover no se pasaria por
#   poco: se metera dentro de la bandera y la tumbaria.
#
# Dos medidas contra eso: esta velocidad final, y que el filtro de mediana se
# APAGUE solo cuando la medida ya es fiable (FLAG_DIST_TRUST_PX).
FINAL_APPROACH_CM = 14.0   # a partir de aqui, arrastre
SPEED_FINAL       = _spd(120)  # apenas por encima de la zona muerta de los
                           # motores. Si el rover no se mueve a esta
                           # velocidad, subela de 10 en 10: es el primer
                           # numero a tocar si se queda plantado a 12 cm.
# DIRECCION DE APROXIMACION. SUAVIZADA en pista (18/09): con los valores
# anteriores el rover hacia bandazos persiguiendo la bandera.
#
#   giro = STEER_KP * |error|^STEER_CURVE,  recortado a TURN_MAX
#
# Los tres numeros tiran en la misma direccion:
#   KP mas bajo      -> menos giro para el mismo error
#   CURVE mas alta   -> mucho menos giro cerca del centro, igual de firme lejos
#   DEADBAND mas alta-> no corrige nada dentro de +-0.10
#
# Con error 0.5:  antes 451*0.5^1.5 = 160   ahora 286*0.5^1.8 = 82  (la mitad)
#
# LA ZONA MUERTA IMPORTA MAS DE LO QUE PARECE: el ancho de la bandera es un
# rasgo pequeno y su centro baila un par de pixeles entre frames. Con la zona
# muerta baja, el rover corrige ese ruido, y corregir ruido es exactamente
# "hacer bandazos".
TURN_MAX         = _spd(380)
STEER_KP         = 520.0 * SPEED_SCALE   # giro = KP * |error|^CURVE, error en [-1,1]
STEER_DEADBAND   = 0.10    # centrado: no se corrige (evita perseguir el ruido)
STEER_CURVE      = 1.8     # >1: suave cerca del centro, firme lejos
STEER_SIGN       = 1.0     # -1 si el rover gira hacia el lado contrario

# ================================================================ PATRULLA
# Velocidades normalizadas [-1000, 1000], las que entiende el MCU.
# Empezar bajo: si se sale, el margen de frenado no da para esa velocidad y
# ese dato ES el resultado de la prueba. Subir de 50 en 50.
PATROL_SPEED  = _spd(300)
ESCAPE_SPEED  = _spd(260)    # retroceso/avance para salir del borde
ESCAPE_TURN   = _spd(340)    # giro sobre el eje
# ---- escape del borde negro ----
# MEDIDO EN PISTA (18/09, con SPEED_SCALE = 0.55): el rover pivota a unos
# 124 grados por segundo. De ahi salen estos tiempos, que ya NO se compensan.
#
#     grados deseados   segundos
#          45             0.36
#          75             0.60     <- EDGE_TURN_S
#          90             0.73
#         180             1.45     <- KEY_TURN_S
#
# Antes esto giraba casi 180 grados para esquivar un borde: demasiado. 75 es
# de sobra para salir de la linea y deja al rover mirando a otro sitio sin
# darse la vuelta entera.
EDGE_BACK_S   = 0.50         # alejandose del borde. BAJADO de 1.27 s: apenas
                             # hace falta separarse para poder pivotar.
EDGE_TURN_S   = 0.60         # ~75 grados. SIN compensar (ver nota abajo).
SLEW_PER_TICK = _spd(80)     # rampa del lado MPU (el MCU tiene la suya).
                             # Escalada tambien: asi el TIEMPO que tarda en
                             # alcanzar la velocidad de crucero no cambia.

# POR QUE LOS GIROS YA NO SE COMPENSAN
# ------------------------------------
# Cuando se anadio SPEED_SCALE, los tiempos de giro se dividian entre el
# factor para conservar el angulo, suponiendo que los grados por segundo son
# proporcionales a la velocidad normalizada. EN UN PIVOTE SOBRE EL EJE ESO ES
# FALSO: las ruedas patinan de lado contra el suelo, y patinan MAS cuanto mas
# rapido van. A velocidad baja hay menos patinaje, asi que se giran mas grados
# por cada unidad de velocidad x tiempo de lo que predice el modelo lineal.
#
# Resultado medido en pista: con el factor a 0.55 la compensacion llevaba
# EDGE_TURN_S a 1.45 s y el rover giraba casi 180 grados para esquivar un
# borde, cuando esos 0.8 s originales hacian unos 90.
#
# Los giros se calibran ahora POR OBSERVACION, con la tabla de arriba, y a la
# velocidad real a la que se va a correr. Si cambias SPEED_SCALE, vuelve a
# cronometrar un giro completo y rehaz la tabla: 360 / segundos = grados/s.
#
# Los AVANCES RECTOS si se siguen compensando (_dur): ahi el patinaje es mucho
# menor y la distancia importa (la caja tiene que caer DENTRO de la zona).

# ============================================ PARTE 4: AGARRAR Y VOLVER A CASA
# Secuencia: APPROACH -> (centrada y cerca) -> GRAB -> CARRY -> DELIVER -> DONE
#
#   GRAB     parado: cierra la pinza, espera, sube la elevacion, espera
#   CARRY    lleva la bandera buscando LA CINTA DEL COLOR DEL EQUIPO con el
#            APDS del suelo. Sin encoders no hay odometria: el rover no sabe
#            donde esta y no puede "ir a una coordenada". Buscar la cinta es
#            lo unico que el hardware permite de forma fiable.
#   DELIVER  en su zona: baja la elevacion, abre la pinza, suelta
#   DONE     mision cumplida, parado

# --- el disparo del agarre ---
# NO basta con estar cerca: hay que estar CENTRADO. Si el rover cierra la
# pinza con la bandera a 20 grados de su eje, la tira en vez de agarrarla.
# Con |error_x| por encima de esto y ya a distancia, pivota sin avanzar.
GRAB_ALIGN_TOL   = 0.10    # |error_x| maximo para cerrar la pinza
GRAB_ALIGN_TURN  = _spd(220)  # giro de alineacion final (pivote, sin avanzar).
                              # NO SE BAJA aunque los demas giros se hayan
                              # suavizado: escalado a 0.55 ya vale 121, y
                              # SPEED_MOVE_FLOOR es 110. Un poco menos y el
                              # rover no pivotaria -- se quedaria zumbando
                              # delante de la bandera sin llegar a centrarse
                              # nunca, que es peor que un giro brusco.

# COMPROMISO FINAL. Muy cerca la bandera puede salirse del encuadre por
# arriba o por abajo segun donde este montada la camara, y entonces la
# deteccion se pierde justo en el ultimo palmo. Si se perdio DENTRO de esta
# distancia, no se abandona: estaba delante, se avanza a ciegas y se agarra.
GRAB_COMMIT_CM   = 9.0     # ATADO A GRAB_DISTANCE_CM: si se pierde la
                           # deteccion por debajo de esto, la bandera estaba
                           # delante y se agarra igual. Con el agarre a 6 cm,
                           # 22 (el valor de cuando el agarre estaba a 14)
                           # significaba cerrar la pinza 16 cm ANTES de llegar.
GRAB_BLIND_S     = _dur(0.0)  # avance a ciegas tras el compromiso (COMPENSADO).
                           # 0 = no avanzar, agarrar donde esta.
                           # MEDIR: si al comprometerse la pinza se queda
                           # corta, sube esto de 0.1 en 0.1 (a SPEED_CREEP).

# --- tiempos de la maniobra de agarre ---
# Los servos se mueven DE UNO EN UNO y con los motores parados. Dos ZOSKAY de
# 20 kg arrancando a la vez, o uno arrancando con los motores, es el pico de
# corriente mas alto del rover: es lo que hunde el rail y reinicia placas.
GRAB_SETTLE_S    = 0.3     # el chasis deja de balancearse antes de cerrar
GRIP_CLOSE_S     = 0.8     # la pinza cierra sobre la bandera
GRAB_PAUSE_S     = 0.5     # PAUSA entre cerrar la pinza y levantar el brazo.
                           # La pinza ya ha llegado a su angulo, pero la
                           # bandera todavia se esta acomodando dentro y el
                           # chasis balanceandose por el tiron del cierre.
                           # Levantar en ese instante es cuando se escapa.
                           # Ademas separa los dos picos de corriente: la
                           # pinza ya se ha apagado (SERVO_AUTO_OFF) antes de
                           # que arranque la elevacion.
LIFT_UP_S        = 0.9     # la elevacion sube con la bandera dentro
# El apagado de los tres servos lo lleva SERVO_AUTO_OFF (arriba). Aqui ya no
# hay nada que decidir: cada orden de servo programa su propio apagado.

# PARARSE DESPUES DE LEVANTAR. Con False, la secuencia termina en HOLDING:
# bandera agarrada y en alto, rover quieto. La vuelta a la zona propia sigue
# implementada y probada, y se puede lanzar a mano con el boton CARRY de la
# pagina; ponlo a True cuando quieras que encadene sola.
RETURN_AFTER_GRAB = True

# EVITAR LA ZONA AMARILLA DESPUES DE DEPOSITAR. DESACTIVADO en pista (18/09).
#
# La idea era buena sobre el papel: la llave es amarilla y cae sobre cinta
# amarilla, asi que la camara no puede verla; pero SI sabemos que esta dentro
# de la zona, o sea que no volver a entrar en el amarillo garantiza no pisarla.
# Una garantia geometrica, no perceptiva.
#
# El coste en pista resulto mayor que el beneficio: la zona amarilla esta en
# el centro y rebotar en ella cada vez que se cruza corta la patrulla
# constantemente, justo cuando el rover deberia estar yendo a por la bandera.
#
# CON ESTO EN False EL ROVER PUEDE EMPUJAR LA CAJA QUE ACABA DE DEPOSITAR.
# Si en pista se ve que la arrastra, vuelve a ponerlo en True: el estado
# AVOID_ZONE sigue implementado y probado, solo esta desconectado.
AVOID_YELLOW_AFTER_DROP = False

# --- la vuelta a casa ---
TEAM_COLOR = C_RED if TEAM == "red" else C_BLUE   # la cinta de nuestra zona
CARRY_SPEED = _spd(260)    # mas lento que PATROL_SPEED: lleva carga en alto y
                           # el centro de gravedad esta mas arriba

# LA LLEGADA A CASA ES DIRECTA: la primera lectura de TEAM_COLOR entrega.
#
# Hubo un "armado" (ver otro color antes, o esperar N segundos) para evitar
# que el rover entregara en el sitio si agarraba la bandera pisando ya su
# propia cinta. ELIMINADO: ese caso NO PUEDE DARSE. La bandera rival esta
# siempre sobre LA ZONA RIVAL, asi que en el instante del agarre el rover esta
# sobre el color del ENEMIGO, nunca sobre el suyo. La condicion protegia de
# algo imposible, y a cambio se atasco en pista: entrando en CARRY levantado
# en el aire el APDS lee "?", que no armaba, y el rover no entrego pese a
# estar sobre su cinta.
#
# Menos estado que mantener, menos que pueda fallar.

# AVANCE DENTRO DE LA ZONA antes de soltar: CERO. El rover suelta en el
# instante en que el APDS ve su color.
#
# Existia por el mismo motivo que en la llave -el sensor va bajo el chasis, o
# sea que avisa con el rover en el BORDE de la zona- pero la zona propia es
# FINA: 1.1 s de avance a ciegas la cruzaban entera y la bandera acababa
# fuera por el otro lado. Con la caja amarilla paso exactamente eso.
#
# La bandera se deja donde esta el rover, no delante (primero baja el brazo y
# luego abre), asi que soltar en el borde no la deja fuera como pasaba con la
# caja. Si aun asi queda medio fuera, sube esto de 0.1 en 0.1.
CARRY_ENTER_ADVANCE_S = 0.0

# --- la entrega ---
DELIVER_DOWN_S   = 0.9     # baja la elevacion
DELIVER_OPEN_S   = 1.0     # abre la pinza. SUBIDO de 0.7: tiene que cubrir
                           # settle_s("grip") = 0.9, o el rover empezaba a
                           # retroceder con la pinza AUN ABRIENDOSE y se
                           # llevaba la bandera por delante. Importa mas ahora
                           # que se suelta en el borde de una zona fina: ahi no
                           # sobra ni un centimetro.
DELIVER_BACK_S   = _dur(0.8)  # retrocede para no volcarla (COMPENSADO: distancia)


# ============================================================ CAMBIO DE EQUIPO
def set_team(team):
    """Cambia de equipo EN MARCHA (boton A4). Recalcula todo lo que sale de
    TEAM, en el mismo sitio donde se define, para que no haya dos formulas.

    Funciona porque el resto del codigo lee config.ENEMY / config.TEAM_COLOR
    en el momento de usarlo, no copiados al arrancar. La unica copia es la
    de la camara (Vision guarda su rango HSV): por eso main.py llama tambien
    a vision.set_enemy(). Si alguna vez se copia uno de estos valores en otro
    sitio, hay que avisarle aqui.
    """
    global TEAM, TEAM_LED, ENEMY, TEAM_COLOR
    if team not in ("red", "blue"):
        raise ValueError(f"equipo desconocido: {team!r}")
    TEAM = team
    TEAM_LED = 1 if TEAM == "blue" else 0
    ENEMY = "blue" if TEAM == "red" else "red"
    TEAM_COLOR = C_RED if TEAM == "red" else C_BLUE
    return TEAM
