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

# ============================================================ TCRT (BORDE)
# Se mandan al MCU al arrancar (cfg_lines). Convencion normalizada:
# menos = mas oscuro, escala 0..4095.
#
# MEDIDO EN PISTA (crudos: blanco 141..177, negro 987..1014, cintas de color
# 172..181). Normalizado: peor negro 3108, peor no-negro 3908.
#
# BAJADO A 3200 (antes 3500). Conviene saber que se gana y que se pierde,
# porque el intercambio NO es simetrico:
#
#     umbral   margen sobre el peor NEGRO   margen bajo el no-negro mas bajo
#     3500              392                            408
#     3200               92                            708
#
# Es decir: 3200 hace mucho mas dificil confundir una cinta de COLOR con
# negro, y mucho mas facil NO VER un negro real. Y esos dos errores no
# cuestan lo mismo:
#     falso negro  -> el rover se para donde no debia: molesto
#     negro perdido-> el rover se sale de la pista: ronda perdida
#
# 92 cuentas siguen siendo ~2,5 veces la banda de ruido del blanco (36), asi
# que funciona. Pero es el umbral con menos margen del sistema en el lado que
# mas caro se paga: si en pista ves que cruza una linea negra sin frenar,
# esto es lo primero que hay que subir.
LINE_BLACK_BELOW = 3200
LINE_HYSTERESIS  = 120
LINE_ADC_MAX     = 4095        # = LINE_ADC_MAX del sketch (12 bits)

# ====================================================== APDS (COLOR DE ZONA)
# MEDIDO con 4X / 10 ms / LED blanco montado. Rango util de "clear": ~40..170,
# o sea ~130 cuentas: poco margen. Si hace falta mas, subir la ganancia en el
# sketch a 16X y RE-MEDIR estos dos (escalan con ella).
# MEDIDO EN PISTA (03/09): 40 daba falsos negros -- alguna cinta de color
# quedaba por debajo y se clasificaba como negro. Con 18 todas las cintas
# salen bien. OJO AL MARGEN: 18 deja menos hueco entre el negro y la cinta
# de color mas oscura que 40, y ese hueco es lo unico que separa las dos
# clases. Si en la sede cambia la luz o la altura del sensor, es el primer
# umbral que se rompe. Ver la nota de ganancia mas abajo.
COLOR_DARK_CLEAR   = 18      # clear por debajo -> negro   (MEDIDO)
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

C_UNKNOWN, C_WHITE, C_BLACK, C_YELLOW, C_RED, C_BLUE = 0, 1, 2, 3, 4, 5
COLOR_NAMES = {0: "?", 1: "blanco", 2: "negro", 3: "amarillo", 4: "rojo", 5: "azul"}

# ================================================================= LLAVE
# PARTE 2: depositar la llave en la zona amarilla (neutra).
#
# Secuencia: PATROL ve amarillo -> DEPOSIT_KEY (entra en la zona, para, servo
# a RELEASE) -> BACKOFF_KEY (retrocede, gira ~180, rearma el servo) -> PATROL.
# Tras depositar, el amarillo es ZONA PROHIBIDA: la caja esta dentro y el
# rover rebota en el borde de la zona igual que en el negro (AVOID_ZONE).
#
# La caja cae DELANTE del rover (confirmado): por eso se retrocede para salir.

SERVO_KEY        = 2       # id del servo en el MCU (0 elevacion, 1 gripper, 2 llave)
# MEDIDO EN EL ROVER (04/09): retener = 180, soltar = 0.
SERVO_KEY_HOLD   = 180     # retiene la llave. Debe coincidir con
                           # SERVO_KEY_HOLD_DEFAULT del sketch (posicion de arranque)
SERVO_KEY_RELEASE = 0      # la tumba. Si el servo zumba contra el tope, sube a 10.
KEY_REARM        = True    # volver a HOLD tras retroceder (listo para otra prueba)

# APAGAR el servo una vez depositada la llave (detach en el MCU). Un servo de
# 20 kg parado contra su tope se queda en corriente de bloqueo: zumba, se
# calienta y hunde el rail del UBEC durante toda la patrulla, sin sujetar
# nada. Sin pulsos deja de empujar. Se re-engancha solo con cualquier orden
# nueva (Retener, Soltar, Rearmar llave).
KEY_DETACH_AFTER_DROP = True
# Cuanto esperar desde la ULTIMA orden al servo antes de apagarlo: write() es
# asincrono y el eje tarda ~600 ms en llegar; apagar antes lo deja a medias.
SERVO_SETTLE_S   = 0.7

# AVANCE DENTRO DE LA ZONA antes de soltar. El APDS avisa cuando EL SENSOR
# pisa el amarillo, o sea con el frente del rover en el BORDE de la zona. La
# caja cae delante del frente, asi que para que quede dentro hay que entrar
# un poco. La zona mide 40 cm y el rover ~30: hay sitio, pero no de sobra.
#
# COMO SE MIDE (cronometro, 10 minutos):
#   1. velocidad real a SPEED_CREEP: rover en el suelo, teleop... en esta
#      parte no hay teleop; usa la patrulla con PATROL_SPEED = SPEED_CREEP y
#      cronometra 50 cm -> v (cm/s)
#   2. d = distancia desde el frente hasta donde cae la caja + ~8 cm de margen
#   3. ZONE_ENTER_ADVANCE_S = d / v      (y comprueba que d < 25 cm para no
#      asomar por el otro lado de la zona)
ZONE_ENTER_ADVANCE_S = 0.8     # PROVISIONAL: medir
SPEED_CREEP          = 170     # velocidad de entrada en la zona

KEY_RELEASE_S   = 1.0    # tiempo para que el servo gire y la caja caiga
KEY_BACKOFF_S   = 1.0    # retroceso tras soltar: aleja el frente de la caja
KEY_TURN_S      = 1.6    # PROVISIONAL: tiempo de pivote para ~180 grados.
                         # Medir: cronometra un giro completo a ESCAPE_TURN y
                         # divide por dos. Si se queda corto, el rebote en
                         # amarillo (AVOID_ZONE) lo corrige igual.

# Rebote en amarillo despues de depositar: mismo esquema que el borde negro.
AVOID_BACK_S    = 0.6
AVOID_TURN_S    = 0.9    # ~90-100 grados: sale del rumbo sin dar la vuelta entera

# ==================================================== SERVOS: ANGULOS MEDIDOS
# Medidos fisicamente en el rover (parte 3). GUARDADOS para las partes que los
# usan; el gripper y la elevacion se conectan en la parte 4.
#
#   Elevacion del gripper   0 (abajo)  ->  40 (arriba)
#   Gripper                 0 (?)      ->  90 (?)      <- confirmar cual es abierto
#   Caja (llave)            135        ->   0          <- ver nota
#
# NOTA LLAVE: la parte 2 sigue usando HOLD=0 / RELEASE=180 (los valores con los
# que se probo). El rango medido ahora es 135 -> 0, o sea que los angulos de
# la parte 2 estan PENDIENTES DE ACTUALIZAR junto con ZONE_ENTER_ADVANCE_S y
# KEY_TURN_S. Cuando se haga: SERVO_KEY_HOLD = 135, SERVO_KEY_RELEASE = 0 (si
# 135 es la posicion que sujeta), y SERVO_KEY_HOLD_DEFAULT = 135 en el sketch.
SERVO_LIFT       = 0       # id del servo en el MCU (enum Id de actuators.h)
SERVO_GRIP       = 1
SERVO_LIFT_DOWN  = 0
SERVO_LIFT_UP    = 40
SERVO_GRIP_OPEN  = 0       # CONFIRMADO 04/09: 0 abierto, 90 cerrado
SERVO_GRIP_CLOSE = 90
# Aplicados ya arriba (SERVO_KEY_HOLD / SERVO_KEY_RELEASE).
SERVO_KEY_HOLD_MEASURED    = 180
SERVO_KEY_RELEASE_MEASURED = 0

# ================================================================== VISION
# PARTE 3: buscar la bandera rival con la webcam USB.
#
# La camara la abre el periferico de App Lab (arduino.app_peripherals.camera),
# que elige sola la primera webcam USB y entrega frames BGR. Sin indices de
# /dev/video que adivinar: en el UNO Q, video0 y video1 son el decodificador
# Venus del Qualcomm, no la camara, y el indice de la webcam cambia al
# reiniciar.
TEAM  = "red"                          # "red" o "blue": nuestro equipo
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
FLAG_NORM_AREA_PX   = 4000.0

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

SPEED_APPROACH   = 260
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
SPEED_FINAL       = 120    # apenas por encima de la zona muerta de los
                           # motores. Si el rover no se mueve a esta
                           # velocidad, subela de 10 en 10: es el primer
                           # numero a tocar si se queda plantado a 12 cm.
TURN_MAX         = 520
STEER_KP         = 820.0   # giro = KP * |error|^CURVE, con error en [-1, 1]
STEER_DEADBAND   = 0.07    # centrado: no se corrige (evita perseguir el ruido)
STEER_CURVE      = 1.5     # >1: suave cerca del centro, firme lejos
STEER_SIGN       = 1.0     # -1 si el rover gira hacia el lado contrario

# ================================================================ PATRULLA
# Velocidades normalizadas [-1000, 1000], las que entiende el MCU.
# Empezar bajo: si se sale, el margen de frenado no da para esa velocidad y
# ese dato ES el resultado de la prueba. Subir de 50 en 50.
PATROL_SPEED  = 300
ESCAPE_SPEED  = 260          # retroceso/avance para salir del borde
ESCAPE_TURN   = 340          # giro sobre el eje
EDGE_BACK_S   = 0.7          # tiempo alejandose del borde
EDGE_TURN_S   = 0.8          # tiempo girando
SLEW_PER_TICK = 80           # rampa del lado MPU (el MCU tiene la suya)

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
GRAB_ALIGN_TURN  = 220     # giro suave de alineacion final (sin avanzar)

# COMPROMISO FINAL. Muy cerca la bandera puede salirse del encuadre por
# arriba o por abajo segun donde este montada la camara, y entonces la
# deteccion se pierde justo en el ultimo palmo. Si se perdio DENTRO de esta
# distancia, no se abandona: estaba delante, se avanza a ciegas y se agarra.
GRAB_COMMIT_CM   = 9.0     # ATADO A GRAB_DISTANCE_CM: si se pierde la
                           # deteccion por debajo de esto, la bandera estaba
                           # delante y se agarra igual. Con el agarre a 6 cm,
                           # 22 (el valor de cuando el agarre estaba a 14)
                           # significaba cerrar la pinza 16 cm ANTES de llegar.
GRAB_BLIND_S     = 0.0     # avance a ciegas tras el compromiso.
                           # 0 = no avanzar, agarrar donde esta.
                           # MEDIR: si al comprometerse la pinza se queda
                           # corta, sube esto de 0.1 en 0.1 (a SPEED_CREEP).

# --- tiempos de la maniobra de agarre ---
# Los servos se mueven DE UNO EN UNO y con los motores parados. Dos ZOSKAY de
# 20 kg arrancando a la vez, o uno arrancando con los motores, es el pico de
# corriente mas alto del rover: es lo que hunde el rail y reinicia placas.
GRAB_SETTLE_S    = 0.3     # el chasis deja de balancearse antes de cerrar
GRIP_CLOSE_S     = 0.8     # la pinza cierra sobre la bandera
LIFT_UP_S        = 0.9     # la elevacion sube con la bandera dentro
# La pinza NO se apaga: apagarla suelta la bandera. Paga corriente de
# retencion toda la vuelta, y es el precio de no tener un cierre mecanico.
# La elevacion SI puede apagarse si el mecanismo se sostiene solo (reductor,
# tornillo sinfin, tope). PRUEBA: sube el brazo con la bandera, pulsa "Apagar
# elevacion" en la pagina y mira si aguanta. Si cae, dejalo en False.
LIFT_DETACH_AFTER_UP = False

# PARARSE DESPUES DE LEVANTAR. Con False, la secuencia termina en HOLDING:
# bandera agarrada y en alto, rover quieto. La vuelta a la zona propia sigue
# implementada y probada, y se puede lanzar a mano con el boton CARRY de la
# pagina; ponlo a True cuando quieras que encadene sola.
RETURN_AFTER_GRAB = False

# --- la vuelta a casa ---
TEAM_COLOR = C_RED if TEAM == "red" else C_BLUE   # la cinta de nuestra zona
CARRY_SPEED = 260          # mas lento que PATROL_SPEED: lleva carga en alto y
                           # el centro de gravedad esta mas arriba

# La zona se detecta por TRANSICION, no por estar encima. Si el rover agarra
# la bandera pisando ya su propia cinta, "estar sobre el color del equipo"
# seria cierto desde el primer ciclo y soltaria la bandera en el sitio, sin
# haber ido a ninguna parte. Con la transicion hace falta ver primero otro
# color; a partir de ahi, el siguiente TEAM_COLOR es una llegada de verdad.
CARRY_ENTER_ADVANCE_S = 0.6   # avance dentro de la zona antes de soltar, para
                              # que la bandera quede DENTRO y no en el borde

# --- la entrega ---
DELIVER_DOWN_S   = 0.9     # baja la elevacion
DELIVER_OPEN_S   = 0.7     # abre la pinza
DELIVER_BACK_S   = 0.8     # retrocede para no volcarla al girar
