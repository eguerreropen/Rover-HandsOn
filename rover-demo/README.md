# Rover H07 — demo, partes 1–4: líneas, borde, llave, bandera y regreso

Arduino UNO Q. Código desde cero, por partes. **Esta parte hace solo dos
cosas**: identifica el color bajo el rover (negro / amarillo / azul / rojo /
blanco) y no se sale de la pista. La página web muestra lo que leen los
sensores. Nada más: ni servos, ni cámara, ni LEDs de equipo — llegan en las
partes siguientes sin tocar lo que aquí ya funcione.

```
rover-demo/
├── app.yaml                 la App de App Lab (brick web_ui)
├── sketch/                  MCU: TCRT, APDS crudo, motores, reflejo de borde
│   ├── sketch.ino  config.h  lines.h  motors.h  colorsense.h  sketch.yaml
├── python/                  MPU: estados, clasificación de color, página
│   ├── main.py  config.py  protocol.py  mission.py  colors.py  ui.py
├── assets/index.html        la página (puerto 7000)
└── tools/test_part1.py      17 pruebas (parte 1) · test_part2.py  13 (llave) · test_part3.py  18 (cámara) · test_part4.py 33 · test_team.py 8 (botón de equipo) · test_alive.py  3 (sondeo) · test_sketch.sh (MCU)
```

## Qué hace cada lado

**MCU (sketch)** — lee los 4 TCRT-5000 (12 bits, normalizados: menos = más
oscuro), lee r,g,b,c crudos del APDS-9960 del piso (Wire, D20/D21), mueve
los motores y ejecuta el **reflejo de borde con enclavamiento**: al pisar
negro frena en <10 ms y no deja avanzar hacia ese lado hasta que los
sensores lleven 250 ms limpios *y* el MPU haya dejado de empujar. Watchdog:
sin órdenes 400 ms → parada.

**MPU (Python)** — estados `IDLE → PATROL → EDGE → PATROL…` (+ `ESTOP`
pegajoso). En PATROL avanza recto; cuando el MCU enclava un borde pasa a EDGE:
se aleja (atrás si el negro está delante, adelante si está detrás, solo giro
si está cruzado), gira hacia el lado limpio y vuelve a PATROL. **Clasifica el
color en el MPU** por proporciones r/g/b con filtro de estabilidad (3
lecturas seguidas) y registra cada cambio en la página y en la consola.

**Todos los umbrales viven en `python/config.py`.** El de negro se manda al
MCU al arrancar (`cfg_lines`). Calibrar = editar `config.py` y reiniciar la
App; el sketch no se recompila.

## Cargar y probar

1. Abre la carpeta `rover-demo` como App en Arduino App Lab y ejecútala.
   Compila el sketch (librerías: Adafruit APDS9960 1.3.1, BusIO 1.17.4,
   MsgPack 0.4.2) y arranca `python/main.py`.
2. Consola: debe salir `[MCU] APDS piso: OK`, `[init] Bridge OK` y las cuatro
   lecturas TCRT. **FL está en A1 y FR en A0** (cables cambiados entre sí a
   propósito): compruébalo tapando un sensor y viendo qué casilla reacciona
   en la página. Si alguna vez vuelves a moverlos, el único sitio que hay que
   tocar es `PIN_LINE_*` en `sketch/config.h`. Si un TCRT no está montado, ponlo en `LINE_PRESENT_MASK`
   (config.h): un pin al aire finge negro y planta el rover.
3. Página: en un navegador de tu PC, `http://<ip-del-uno-q>:7000/` (la IP
   de la placa la ves en App Lab, o en la propia placa con `hostname -I`;
   el puerto 7000 es el que abre el brick `web_ui` y no hay que declararlo).
   Con el rover **en la mano**:
   pásalo sobre cada cinta y mira el color, y acerca una esquina a la cinta
   negra: su casilla se pone roja y el enclavamiento marca «no avanzar».
4. Rover en pista blanca, lejos del borde → **Iniciar patrulla**. Debe
   avanzar, frenar en el negro, retroceder, girar y seguir. Si se sale,
   baja `PATROL_SPEED`; si nunca se sale, súbelo de 50 en 50: la velocidad a
   la que empieza a salirse es el dato que buscamos.
   Barra espaciadora = parada de emergencia (pegajosa: «Rearmar» para salir).

## Calibración de colores — HECHA (03/09)

Las cuatro cintas (negro, amarillo, azul, rojo) y el blanco se identifican
correctamente. Las reglas por fracciones r/g/b quedaron **como estaban**; lo
único que hubo que mover fue el umbral de negro:

| Parámetro | Valor | Estado |
|---|---|---|
| `COLOR_DARK_CLEAR` | **18** (era 40) | medido en pista |
| `COLOR_BRIGHT_CLEAR` | 170 | medido |
| `COLOR_NEUTRAL_SPREAD` | 0.12 | validado |
| `COLOR_DOMINANT` | 0.42 | validado |
| `YELLOW_*` | 0.24 / 0.30 / 0.30 | validado |
| TCRT (`LINE_BLACK_BELOW`) | 3500 | funcionando |

**Lo que hay que vigilar:** bajar de 40 a 18 significa que alguna cinta de
color leía un *clear* entre 18 y 40 y se clasificaba como negra. Es decir, el
hueco entre «negro» y «la cinta más oscura» es más estrecho de lo que
pensábamos, y ese hueco es lo único que separa las dos clases. Si en la sede
cambia la luz ambiente o se mueve la altura del sensor, es el primer umbral
que se rompe.

**Si quieres margen real:** sube `APDS_GAIN` de `APDS9960_AGAIN_4X` a `16X` en
`sketch/config.h`. Multiplica todas las lecturas por ~4, así que la separación
entre negro y la cinta más oscura pasa de unas pocas cuentas a cuatro veces
más y el ruido pesa cuatro veces menos. Coste: hay que **re-medir**
`COLOR_DARK_CLEAR` y `COLOR_BRIGHT_CLEAR`, porque escalan con la ganancia.

Nota tranquilizadora: esto **no afecta a la seguridad** del rover. El borde
negro que decide la ronda lo detectan los TCRT, no el APDS. Un fallo de
clasificación aquí es un error de identificación de zona, no un rover
saliéndose de la pista.

## Orden de encendido

Enciende la placa **antes** que motores y servos. No por el arranque —el
UNO Q tarda unos segundos en alimentarse y arrancar, y eso es normal— sino
por seguridad: con el L298N energizado mientras Linux arranca, sus entradas
están en estado indefinido y los motores pueden dar tirones con el rover en
la mesa. El MCU (STM32) arranca en milisegundos y su watchdog los mantiene
parados, pero la etapa de potencia no debería estar viva antes que la lógica.

La fuente del UNO Q debe ser de **5 V / 3 A con Power Delivery** y con cable
USB-C de 3 A: es lo que la placa pide por PD. Con menos, los fallos aparecen
de forma intermitente y bajo carga, que es la peor forma de que aparezcan.

## Alimentación por USB-C con OTG: dos avisos

1. El adaptador tiene que ser un **hub/OTG con entrada de alimentación**
   (Power Delivery o Y con conector de carga): un OTG simple no alimenta la
   placa, y en modo host la placa no recibe energía del periférico. Es el
   montaje que documenta Arduino para usar periféricos USB en el UNO Q.
2. Alimentar el UNO Q por USB desde una fuente separada de la batería de
   motores es **buena noticia**: aísla la placa de las caídas de tensión
   del L298N, que era la hipótesis eléctrica del watchdog intermitente.
   Pero la masa de esa fuente y la de la batería **deben unirse** (GND
   común con el L298N) o las señales PWM/IN no valen nada. Y los motores
   siguen sin poder tomar ni un miliamperio del USB.

## Diagnóstico del enlace

La tarjeta «Enlace con el MCU» y la línea `[mcu]` de la consola comparan
órdenes enviadas contra recibidas y muestran el `loop()` más lento del MCU.
Si reaparece el watchdog intermitente de la vez anterior, esos números
dicen si es transporte, bloqueo del MCU o electricidad — y el experimento
`PATROL_SPEED = 0` (mismo software, motores sin corriente) separa lo
eléctrico del resto en dos minutos.

## PARTE 2 — depositar la llave

**Qué hace.** Al pulsar «Iniciar patrulla» el rover avanza y rebota en los
bordes negros (parte 1) hasta que el APDS pisa **amarillo**. Entonces: entra
en la zona a velocidad lenta durante `ZONE_ENTER_ADVANCE_S`, se para, gira
el servo de la llave a `SERVO_KEY_RELEASE` (**10°**) y la tumba, espera a que
caiga, **retrocede** (la caja cae delante), da media vuelta, rearma el servo
y sigue patrullando. A partir de ahí **el amarillo es zona prohibida**: cada
vez que lo pisa, retrocede y gira, igual que en el negro.

**Por qué no se esquiva con la cámara.** La llave es amarilla y cae sobre
cinta amarilla: para la segmentación por color es invisible. En cambio
*sabemos dónde está*: dentro de la zona amarilla, a por lo menos el avance
de entrada del borde. Si el rover no vuelve a entrar en el amarillo, no la
pisa nunca. Es una garantía geométrica, no perceptiva, y por eso es fiable.
Coste: el rover no vuelve a cruzar la zona neutra durante la patrulla. Para
la misión completa (la base rival está al otro lado) habrá que rodearla, y
eso lo resuelve la cámara en la parte 4.

**Sin cámara, «buscar el amarillo» es rebotar hasta pisarlo.** En una pista
de 250×120 con la zona de 40×40 en el centro se encuentra, pero no hay
garantía de tiempo. Para el demo, coloca el rover **encarado a la zona**: una
pasada recta la cruza.

### Cableado y alimentación del servo

Servo de la llave en **D10** (PWM). Señal al D10, masa a la común, y
**alimentación desde un UBEC propio**, jamás del 5 V de la placa: un ZOSKAY
de 20 kg contra su tope tira varios amperios y eso mata placas — es la
hipótesis principal de por qué murió el UNO Q anterior.

**Soltar es a 10°, no a 0.** A 0 el servo empujaba contra el final de carrera
y se quedaba en corriente de bloqueo zumbando. 10° tumba la caja igual sin
llegar al tope.

Y **el servo se apaga solo** en cuanto el eje ha llegado, como los otros dos
— ver «Los servos solo están encendidos mientras se mueven» más abajo.

### Cómo cargar la llave

1. Enciende. El MCU pone el servo en `SERVO_KEY_HOLD_DEFAULT` (0) al
   instante; la App, al arrancar, manda `SERVO_KEY_HOLD` de `config.py`.
   Cuando la consola diga `[init] servo de la llave en retencion`, coloca la
   caja a mano.
2. Rover sobre blanco, lejos del borde, encarado a la zona amarilla.
3. «Iniciar patrulla».
4. Para repetir: recoge la caja, cárgala, **«Rearmar llave»** (vuelve a
   considerarla a bordo), «Iniciar patrulla».

Los botones «Soltar (180)» y «Retener» mueven el servo a mano, para ver el
mecanismo sin patrullar. Ángulos fijos por `curl` para encontrar el tope de
retención: `curl http://<ip>:7000/api/key/deg/30` (0, 30, … 180).

### Dos números que hay que medir

| Parámetro | Qué decide | Cómo se mide |
|---|---|---|
| `ZONE_ENTER_ADVANCE_S` (0,8 s provisional) | que la caja caiga **dentro** de la zona | velocidad real a `SPEED_CREEP` (cronómetro, 50 cm) → `d / v`, con `d` = distancia del frente al punto de caída + 8 cm de margen. Comprueba `d < 25 cm`: la zona mide 40 y el rover ya está dentro |
| `KEY_TURN_S` (1,6 s provisional) | la media vuelta tras soltar | cronometra una vuelta completa girando a `ESCAPE_TURN` y divide por dos. Si se queda corto no pasa nada grave: el rebote en amarillo corrige |

El APDS avisa cuando **el sensor** pisa el amarillo, es decir con el frente
del rover en el borde de la zona. Con `ZONE_ENTER_ADVANCE_S` demasiado
corto la caja cae sobre la línea; demasiado largo, el rover asoma por el
otro lado de la zona.

### Lo que la telemetría enseña ahora

La tarjeta «Llave» muestra a bordo / depositada, el ángulo real del servo
(lo reporta el MCU, no lo que la App cree haber mandado) y las veces que se
ha soltado. El bit «moviéndose» dura 600 ms tras cada orden: mientras está
activo, el MCU mantiene los motores parados 250 ms (pico de corriente).

## PARTE 3 — buscar la bandera rival con la cámara

**Qué hace.** La webcam USB (por el hub OTG) busca un cilindro del color
rival (`ENEMY`, el contrario de `TEAM`). Cuando lo ve: el **LED de bandera
(D13) parpadea** y la página lo muestra en el visor de puntería. Si además
la llave ya está depositada, entra en `APPROACH`: gira hacia la bandera
(control proporcional con zona muerta), avanza más despacio cuanto más
descentrada y cuanto más cerca, y a `GRAB_DISTANCE_CM` se para en
`FLAG_REACHED` con el **LED fijo**. La parte 4 pone ahí el agarre.

**Cerrojo del reglamento.** «Si un robot busca la bandera antes de depositar
la caja, pierde la ronda.» *Ver* es pasivo y se señaliza siempre;
*perseguir* solo con `key_deposited`. Está en un único sitio
(`_hunt_allowed`) y `APPROACH` no se entra sin pasar por ahí. Para probar la
cámara en el banco sin depositar: `FLAG_REQUIRES_KEY = False`… y vuelve a
`True`. La página dice en rojo «BLOQUEADA» mientras la llave está a bordo.

**Cómo se abre la cámara.** Con el periférico de App Lab
(`arduino.app_peripherals.camera.Camera`): elige sola la primera webcam USB
y entrega frames BGR. Eso evita el problema clásico del UNO Q: `/dev/video0`
y `video1` son el decodificador Venus del Qualcomm (abren pero no dan
frames) y el índice de la webcam cambia al reiniciar.

**NO hay `requirements.txt`, y es a propósito.** La imagen del contenedor ya
trae una compilación propia de Arduino: `opencv-python-headless 4.13.0+1ddb20b`.
Ese `+1ddb20b` es una *versión local* (PEP 440) que no existe en PyPI, así que
en cuanto un `requirements.txt` menciona opencv, el resolvedor intenta anclarla,
no la encuentra en ningún índice y aborta la resolución entera:

```
× No solution found when resolving dependencies:
  Because there is no version of opencv-python-headless==4.13.0+1ddb20b …
```

Declara ahí **solo lo que la imagen no traiga**. Lo mismo aplica a `numpy` y a
cualquier cosa que `arduino_app_bricks` ya arrastre.

**Sin vídeo en vivo.** La página no lo necesita: la detección se ve en el
visor de puntería. El lazo de visión solo detecta — no dibuja ni codifica
JPEG —, que es donde estaba la mitad del coste. Queda **una foto bajo
demanda** en `/api/snapshot` (botón «Ver foto con overlay»): se genera solo
cuando la pides, sobre una copia del último frame, y es lo que se usa para
calibrar los HSV. El hilo de visión sigue siendo el único que hace
`capture()`.

**Por qué no una red neuronal.** La bandera es un cilindro liso de 5×15 cm
en color plano: segmentación HSV + filtro geométrico corre en pocos ms, es
determinista y se recalibra en dos minutos si cambia la luz. El filtro
geométrico (aspecto, relleno, ancho máximo, cuarto superior ignorado) es lo
que separa la **bandera** (alta y estrecha) de la **cinta del suelo** (ancha
y aplastada), que es del mismo color. Está probado con frames sintéticos: la
cinta sola no dispara, la bandera propia no dispara, y con las dos a la vez
elige la bandera. **Lee el aviso de abajo sobre los valores del 04/09**: con
la calibración actual el margen contra la cinta es más estrecho que antes.

### La puntuación, y el fallo que la hizo cambiar (04/09)

Una detección solo cuenta si su **puntuación** pasa `MIN_CONFIDENCE`. Es un
producto de tres factores, y la página **los enseña por separado**:

```
puntuación = relleno × factor_aspecto × factor_área
```

En el banco de cámara apareció el caso que obligó a rehacerla: máscara
perfecta (162×226 px, relleno 0,97), forma correcta, y **rechazada con 0,36**.
Las dos causas eran mías:

1. El factor de aspecto usaba el error **absoluto**, `1/(1+|aspecto−3|)`, sin
   forma de aflojarlo. Con aspecto 1,40 eso da 0,38 y hunde el producto.
   Ahora el error es **relativo** al ideal y `FLAG_ASPECT_TOL` lo modula.
2. Ese 1,40 no medía nada: la bandera estaba **cortada por el borde inferior**
   (ocupaba y = 250…476 de 480). A menos de ~40 cm no cabe en el encuadre, su
   alto está truncado y el aspecto medido es ruido. Ahora el detector marca la
   mancha como **RECORTADA** cuando toca un borde y **no penaliza el aspecto**.

El caso real pasa de **0,36 a 0,97**. La lección práctica: si la máscara se ve
bien y aun así rechaza, mira el desglose de la página antes de tocar umbrales
— dice cuál de los tres factores falla.

### AVISO sobre la calibración del 04/09

Los valores medidos en el banco incluyen `FLAG_MIN_ASPECT = 0.1` (antes 1,5) y
`FLAG_MAX_WIDTH_FRAC = 0.6` (antes 0,45). Eso **abre los dos filtros que
existen solo para rechazar la cinta del suelo**. Reproducido con escenas
sintéticas:

| mancha roja de 260×60 px (aspecto 0,23; 41 % del ancho) | resultado |
|---|---|
| con 1,5 / 0,45 | rechazada: «aspecto 0,23 < 1,5 (mancha ANCHA)» |
| con 0,1 / 0,60 | **aceptada como bandera**, puntuación 0,50 > 0,35 |

No está comprobado que ocurra con la cinta real —depende del ángulo y la
distancia a los que la vea la cámara—; está comprobado que **el filtro ya no
lo impide**. Para salir de la duda en dos minutos: pon un trozo de cinta roja
delante de la cámara del banco, a la altura y ángulo del rover, y mira si sale
como DETECTADA.

Si sale, el arreglo **no** es subir `FLAG_MIN_ASPECT` (eso vuelve a tumbar la
bandera cortada, que es lo que nos trajo aquí), sino **bajar
`FLAG_MAX_WIDTH_FRAC`**: la bandera de cerca sigue siendo estrecha, la cinta
es ancha. El ancho es el único de los dos discriminantes que la bandera cortada
no rompe. `test_part3.py` lo deja escrito con números.

### El visor de puntería

La tarjeta dibuja, a escala de la imagen de la cámara: la cruz central, el
bounding box y su **centro**, la línea de error hasta la cruz, una **estela**
de los últimos 3 s (se ve cómo el centro se acerca a la cruz al girar), y
abajo la barra de error con la **zona muerta** del control sombreada. Verde
= centrado; ámbar con «GIRAR IZQ/DER» si no. Los números: centro en px,
error (−1…+1), distancia, **h_px** (para calibrar la focal), puntuación y
fps. La *sparkline* guarda el error 10 s: si el control converge, la línea se
aplana sobre el eje; si oscila, hay que bajar `STEER_KP`.

### Calibrar (dos cosas, cinco minutos)

1. **HSV.** Ya calibrados en el banco de cámara (04/09) y copiados a
   `config.py`. Para verificar en el rover: pulsa «Ver foto con overlay» (o
   `curl http://<ip>:7000/api/snapshot -o f.jpg`) y comprueba que el recuadro
   cae sobre la bandera. Con luz de sede distinta hay que repetir la
   calibración **en el banco**, que es donde se ve la máscara.
2. **Focal — MEDIDA 04/09.** `h_px = 98` con la bandera a **85 cm** →
   `FOCAL_PX = 98 × 85 / 15 = 555`. Se mide a la distancia que sea, mientras
   la midas con cinta métrica y el visor **no** diga RECORTADA (si lo dice, el
   alto está truncado y la medida no vale).

### Alcance: por qué a 100 cm no la vio

Con `FOCAL_PX = 555` la bandera cubre 4000 px² a **74 cm**. Ese 4000 es
`FLAG_NORM_AREA_PX`: el área a partir de la cual el factor de área de la
puntuación vale 1. Más lejos, ese factor empieza a restar y la detección se
cae sola **aunque la máscara sea perfecta** — cuadra con lo observado (85 cm
sí, 100 cm no).

Es la palanca del alcance, y tiene la propiedad rara de **no tener
contrapartida contra la cinta del suelo**: una cinta ocupa 15 000 px o más, su
factor de área ya vale 1,0 y bajar el umbral no puede subirlo. Aflojar aquí
ayuda solo a la bandera.

| `FLAG_NORM_AREA_PX` | puntuación plena hasta |
|---|---|
| 4000 (actual) | ~74 cm |
| 1500 | ~121 cm |
| 1000 | ~148 cm |

**Antes de tocarlo, comprueba en el banco cuál es el problema de verdad:**
bandera a 100 cm y mira la lista de rechazos.

- Aparece con motivo **«puntuación … × área 0,5x»** → es esto: baja el «Área
  de referencia» en el banco hasta que detecte, y tráeme el número.
- **No aparece en la lista** → el problema es de **color**, no de puntuación.
  Mira la máscara: a 100 cm la bandera son 28 px de ancho (14 en el frame
  reducido por `DETECT_SCALE = 0.5`), y a ese tamaño el desenfoque y la
  morfología se la comen. Ahí la salida es `DETECT_SCALE = 0.75` o `1.0`, a
  costa de CPU — vigila `proc_ms` en la página.

### Ángulos de servos medidos (guardados para la parte 4)

| Servo | Rango medido | En `config.py` |
|---|---|---|
| Elevación del gripper | 0° (abajo) → 40° (arriba) | `SERVO_LIFT_DOWN / UP` |
| Gripper | 0° → 90° (confirmar cuál es abierto) | `SERVO_GRIP_OPEN / CLOSE` |
| Caja (llave) | 135° → 0° | `SERVO_KEY_*_MEASURED` — **pendiente de aplicar en la parte 2** |

Confirmado 04/09: pinza **0 = abierta, 90 = cerrada**. Aplicado.

## Velocidad global: `SPEED_SCALE`

Un solo mando para «que vaya más despacio». **Ahora está en `0.75`**: el rover
anda al 75 % de lo que andaba. `1.0` lo devuelve a como estaba.

Todas las velocidades y giros de `config.py` salen de multiplicar por ese
factor, así que no hay que tocarlas una a una ni se puede olvidar ninguna.

### Es un 0,75 de verdad, no aproximado

Las velocidades van normalizadas 0–1000 y el MCU las convierte a PWM así:

```
pwm = MOTOR_DEADZONE_PWM + (MOTOR_MAX_PWM - MOTOR_DEADZONE_PWM) * v / 1000
```

`MOTOR_DEADZONE_PWM` es el PWM al que el motor **empieza** a girar, así que la
rueda va a una velocidad proporcional a `pwm − zona_muerta`, que es
proporcional a `v`. Por eso multiplicar por 0,75 da ~0,75 de velocidad real.

### Lo que NO se puede escalar igual, y por qué

**Las maniobras a ciegas recorren velocidad × tiempo.** Girar 180°, retroceder
15 cm — ninguna mide nada, las dos dependen de cuánto tiempo se mueve el rover
y a qué velocidad. Si se baja la velocidad al 75 % y se deja el tiempo igual,
**el giro de 180° pasa a ser de 135° y el retroceso se queda corto**. El
comportamiento cambiaría sin que nadie lo haya pedido.

Por eso esos tiempos se **dividen** entre el factor, para conservar el ángulo y
la distancia que estaban calibrados. Lo único que cambia es que se tarda más:

| | antes | ahora |
|---|---|---|
| `EDGE_BACK_S` / `EDGE_TURN_S` | 0,7 / 0,8 | 0,93 / 1,07 |
| `KEY_BACKOFF_S` / `KEY_TURN_S` | 1,0 / 1,6 | 1,33 / **2,13** |
| `AVOID_BACK_S` / `AVOID_TURN_S` | 0,6 / 0,9 | 0,80 / 1,20 |
| `ZONE_ENTER_ADVANCE_S` | 0,8 | 1,07 |
| `CARRY_ENTER_ADVANCE_S` | 0,6 | 0,80 |
| `DELIVER_BACK_S` | 0,8 | 1,07 |

**Y los tiempos que no son movimiento no se tocan.** Un servo tarda lo que
tarda en recorrer su arco y una detección caduca cuando caduca: ni `GRIP_CLOSE_S`
ni `LIFT_UP_S` ni `LOST_TARGET_S` dependen de lo rápido que ande el rover.
Tampoco las distancias en cm (`GRAB_DISTANCE_CM` y compañía), que son físicas.

`test_part4.py` fija las tres reglas: velocidad × tiempo constante, tiempos de
servo intactos, y el perfil de aproximación sigue siendo crucero > rampa >
arrastre > parado.

### AVISO: `SPEED_FINAL` se queda en 90

| | antes | ahora |
|---|---|---|
| `PATROL_SPEED` | 300 | 225 |
| `SPEED_APPROACH` / `ESCAPE_SPEED` / `CARRY_SPEED` | 260 | 195 |
| `SPEED_CREEP` | 170 | 128 |
| **`SPEED_FINAL`** | **120** | **90** |
| `ESCAPE_TURN` | 340 | 255 |
| `TURN_MAX` | 520 | 390 |
| `GRAB_ALIGN_TURN` | 220 | 165 |

`SPEED_FINAL` ya estaba «apenas por encima de la zona muerta» a 120. A **90**
es muy probable que el rover **zumbe sin moverse** en el último tramo: cerca de
cero la linealidad se rompe porque el rozamiento estático es mayor que el
dinámico, y el motor no arranca.

No lo he recortado en silencio: un recorte oculto convierte «va al 75 %» en una
mentira y el fallo se busca en el sitio equivocado. En su lugar, `main.py`
**avisa al arrancar** cuando una velocidad queda por debajo de
`SPEED_MOVE_FLOOR`.

**Si se planta a 12 cm de la bandera, sube `SPEED_FINAL` a mano** (de 10 en 10)
— no bajes `SPEED_SCALE`, que es lo contrario de lo que hace falta.

`SPEED_MOVE_FLOOR = 110` **no está medido**. Para medirlo: patrulla bajando
`PATROL_SPEED` de 10 en 10 hasta que el rover deje de arrancar desde parado, y
pon aquí ese valor más ~20 de margen.

## Los servos solo están encendidos mientras se mueven

`SERVO_AUTO_OFF` aplica sola la misma secuencia que hacían los botones
«Apagar» de la página: **mandar el ángulo → esperar `SERVO_SETTLE_S` a que el
eje llegue → cortar los pulsos**. Se re-enganchan con cualquier orden nueva
(el MCU hace `attach` antes de `write`).

```
t=0.30s  grip 90          cierra la pinza
t=1.01s  grip_off         el eje ya llegó: sin pulsos
t=1.11s  lift 40          sube el brazo
t=1.82s  lift_off         sin pulsos
```

Un ZOSKAY de 20 kg parado contra su tope **no sujeta mejor por estar
alimentado**: zumba, se calienta y hunde el rail del UBEC durante toda la
ronda. Sin pulsos el reductor lo retiene solo.

**Un plazo de apagado por servo, no uno compartido.** Si los tres compartieran
plazo, una orden a la pinza reiniciaría el del brazo y el brazo se quedaría
encendido — en silencio, y con un síntoma (el UBEC hundido a mitad de ronda)
que no se parece en nada a la causa.

**Todas las órdenes pasan por `Mission._servo()`.** Es lo único que garantiza
que no se escape ninguna: si un estado nuevo emitiera `("grip", x)` a mano, ese
servo se quedaría encendido para siempre.

La puesta a punto del arranque también se apaga: colocar el brazo es una orden
de servo como cualquier otra, y si no, los tres se quedarían zumbando desde que
enciendes hasta la primera maniobra — que puede ser toda la espera antes de la
ronda.

La página tiene ahora una fila **«Servos alimentados»** que dice cuáles tienen
pulsos ahora mismo (el MCU lo reporta con un bit por servo). **En reposo debe
decir «ninguno».**

### AVISO: la pinza es el único que puede perder algo

Los otros dos servos apagados no pierden nada. La pinza sí: **si el reductor no
aguanta el cierre por sí solo, apagarla suelta la bandera** — y no hay sensor
que avise, así que el rover seguiría su camino creyendo que la lleva.

**La prueba, un minuto:** agarra la bandera con la pinza, pulsa «Apagar pinza»
en la página, y tira suavemente de la bandera hacia arriba.

- Aguanta → deja `"grip": True`.
- Cede o se suelta → pon `"grip": False` en `SERVO_AUTO_OFF`. La pinza queda
  alimentada mientras lleve la bandera, que es el precio de no tener un cierre
  mecánico.

Mientras no hagas esa prueba el riesgo está acotado: `RETURN_AFTER_GRAB` es
`False`, la secuencia termina parada en `HOLDING` y lo ves enseguida.

### Un detalle a vigilar

Cada movimiento ahora hace `attach` → `write` → `detach`. En una ronda son
media docena de ciclos, nada preocupante, pero si alguna vez el MCU se cae
después de mucho trastear con los botones de la página, sospecha de eso: no
está comprobado que el `detach` de zephyr libere lo que reserva el `attach`.

## Ajustes de pista del 18/09

`SPEED_SCALE` está en **0,55**. Seis cambios sobre lo que se vio corriendo:

### 1. Los giros ya no se compensan con `SPEED_SCALE` — y era culpa mía

Cuando añadí el factor, dividí los tiempos de giro entre él «para conservar el
ángulo», asumiendo que los grados por segundo son proporcionales a la velocidad
normalizada. **En un pivote sobre el eje eso es falso:** las ruedas patinan de
lado contra el suelo, y patinan *más* cuanto más rápido van. A velocidad baja
hay menos patinaje, así que se giran más grados de los que predice el modelo.

Resultado: con el factor a 0,55 la compensación llevaba `EDGE_TURN_S` a 1,45 s
y el rover giraba **casi 180° para esquivar un borde**, cuando los 0,8 s
originales hacían unos 90.

Ahora los giros se calibran **por observación**, medidos a la velocidad real:

```
~124 grados por segundo  (medido en pista, SPEED_SCALE = 0.55)

  45 grados -> 0.36 s
  75 grados -> 0.60 s   <- EDGE_TURN_S
  90 grados -> 0.73 s   <- AVOID_TURN_S
 180 grados -> 1.45 s   <- KEY_TURN_S
```

`KEY_TURN_S` estaba aún peor: la compensación lo dejaba en 2,9 s, que a esa
velocidad son **casi dos vueltas enteras** en vez de media.

**Si cambias `SPEED_SCALE`, vuelve a cronometrar un giro completo** y rehaz la
tabla: `360 / segundos = grados/s`. Los avances rectos sí se siguen
compensando: ahí el patinaje es mucho menor y la distancia importa (la caja
tiene que caer dentro de la zona).

### 2. Retrocede menos al evitar el borde

`EDGE_BACK_S` de 1,27 s a **0,50**. Apenas hace falta separarse para poder
pivotar. `AVOID_BACK_S` igual. `KEY_BACKOFF_S` **no** se toca: ahí hay una caja
delante de la que sí hay que alejarse de verdad.

### 3. La dirección de aproximación, más suave

| | antes | ahora |
|---|---|---|
| `STEER_KP` (base) | 820 | **520** |
| `STEER_CURVE` | 1,5 | **1,8** |
| `STEER_DEADBAND` | 0,07 | **0,10** |
| `TURN_MAX` (base) | 520 | **380** |

Con error 0,5 el giro pasa de 160 a 82 — la mitad. **La zona muerta importa más
de lo que parece:** el ancho de la bandera es un rasgo pequeño y su centro baila
un par de píxeles entre frames; con la zona muerta baja, el rover corrige ese
ruido, y corregir ruido es exactamente «hacer bandazos».

`GRAB_ALIGN_TURN` **no se baja**: escalado a 0,55 ya vale 121 y
`SPEED_MOVE_FLOOR` es 110. Un poco menos y el rover no pivotaría — se quedaría
zumbando delante de la bandera sin centrarse nunca, que es peor que un giro
brusco.

### 4. Una sola lectura de amarillo dispara el depósito

`ZONE_TRIGGER_INSTANT = True`. El síntoma era «la página muestra el amarillo un
instante y no deposita». La causa era el filtro de estabilidad: el depósito
miraba el color **estable**, que exige 2 lecturas seguidas, y cruzando la cinta
solo daba tiempo a una. El instantáneo cambiaba —por eso se veía en la página—
pero el estable no cuajaba nunca.

El intercambio no es simétrico: **amarillo perdido = la llave no se deposita
jamás**; amarillo falso = cae donde no debía. Lo primero es seguro, lo segundo
improbable (el amarillo exige rojo *y* verde altos con el azul hundido) y solo
puede pasar una vez por ronda. El resto del sistema sigue usando el color
estable: esto es **solo** el disparo del depósito.

### 5. Pausa entre cerrar la pinza y levantar

`GRAB_PAUSE_S = 0.5`. La pinza ya llegó a su ángulo, pero la bandera sigue
acomodándose dentro y el chasis balanceándose por el tirón del cierre: levantar
en ese instante es cuando se escapa. De paso separa los dos picos de corriente
— la pinza ya se apagó sola antes de que arranque la elevación.

### 6. Alcance de la cámara, y el amarillo deja de ser zona prohibida

**«No se acerca a una bandera lejana tras dejar la llave» eran dos cosas a la
vez**, y las dos están tocadas:

`FLAG_NORM_AREA_PX` de 4000 a **1500**. Con 4000 la bandera solo puntuaba pleno
dentro de 74 cm y más lejos el factor de área hundía la detección aunque la
máscara fuera perfecta: el rover **no la veía**. Con 1500 llega a ~121 cm. No
favorece a la cinta del suelo (una cinta ya ocupa 15 000 px, su factor ya valía
1,0).

`AVOID_YELLOW_AFTER_DROP = False`. La garantía geométrica era buena sobre el
papel —no entrar en el amarillo = no pisar la caja— pero la zona está en el
centro y rebotar en ella cortaba la patrulla constantemente, justo cuando el
rover debería ir a por la bandera.

> **Coste asumido: el rover puede empujar la caja que acaba de depositar.** Si
> en pista se ve que la arrastra, vuelve a `True`. El estado `AVOID_ZONE` sigue
> implementado y probado, solo está desconectado.

### 7. Se para en seco al ver el amarillo, y espera 1 s tras soltar

```
t=0.00  ve amarillo  ->  PARA EN SECO           (antes: avanzaba 1,45 s)
t=0.05  suelta la llave, servo a 10°
t=0.75  el servo se apaga solo
t=1.06  SOLTADA. Quieto 1 s más mientras la caja se asienta
t=2.06  recién ahora retrocede
```

**El síntoma:** marcaba la llave como depositada y «seguía de largo».
`ZONE_ENTER_ADVANCE_S` valía 0,8 s, pero con `SPEED_SCALE = 0.55` la
compensación lo convertía en **1,45 s de avance a ciegas** — más de lo que mide
la zona. El rover entraba, la cruzaba entera y soltaba fuera o ya saliendo.

Ahora `ZONE_ENTER_ADVANCE_S = 0`: para en el instante en que el APDS ve
amarillo. Y `KEY_WAIT_AFTER_DROP_S = 1.0` mantiene el rover **completamente
quieto** un segundo después de soltar, aparte de `KEY_RELEASE_S`. Los dos
tiempos cubren cosas distintas: `KEY_RELEASE_S` es el recorrido del servo,
`KEY_WAIT_AFTER_DROP_S` es que la caja termine de caer, rebotar y quedarse
quieta. Arrancar mientras se asienta la arrastra, y ocurre justo en el punto
ciego de delante.

Ni la suelta ni la espera se interrumpen por un borde negro: el rover ya está
parado, y cortar a medias deja la llave a medio soltar o la caja arrastrada. El
borde se atiende en cuanto terminan.

> **El intercambio, que no es gratis.** Parar al ver el amarillo es parar en el
> **borde** de la zona, y la caja cae **delante** del rover: si el frente queda
> justo sobre la línea, la caja puede quedar fuera — el error contrario al que
> teníamos. **Mira dónde cae:** si queda dentro, déjalo en 0; si cae sobre la
> línea o fuera por delante, sube `ZONE_ENTER_ADVANCE_S` de 0,1 en 0,1.

### 8. El servo no soltaba la caja — tres causas, dos tuyas y una mía

Tus dos hipótesis eran correctas, y había una tercera peor.

**a) La señal no le daba tiempo.** La llave recorre **170°** (180 → 10) *y
empujando la caja*. Un ZOSKAY de 20 kg va a ~0,20 s por cada 60° **sin carga**:
eso ya son 0,57 s, y con carga bastante más. `SERVO_SETTLE_S` valía **0,7 s
para los tres servos**, así que a la llave se le cortaba la corriente casi
encima del final del recorrido — y con algo de resistencia, se quedaba a medias.

Ahora el asentamiento es **por servo**, según los grados que recorre y la carga:

| servo | recorrido | asentamiento |
|---|---|---|
| llave | 170° contra la caja | **1,5 s** |
| pinza | 90° | 0,9 s |
| elevación | 40° | 0,8 s |

`KEY_RELEASE_S` sube de 1,0 a **1,8 s**, para que la fase de suelta no termine
antes que el propio asentamiento.

**b) Empezaba apagado.** Cierto, y peor de lo que pensabas: no es que tardara
en arrancar, es que **patrullaba con la llave sin pulsos**.

**c) Sujetar no es moverse, y esa distinción se me escapó.** Al generalizar el
apagado automático, la orden de **RETENER** (`SERVO_KEY_HOLD`) también
programaba su apagado. O sea que el rover patrullaba con el servo de la llave
detachado — justo lo contrario de lo que significa «retener». La parte 2
original solo apagaba *después* de soltar; al unificarlo lo rompí.

Ahora las órdenes que **sujetan** algo llevan `keep_on=True` y no programan
apagado:

```
llave  a HOLD     -> alimentada (sujeta la caja)
llave  a RELEASE  -> se apaga (ya no hay nada que sujetar)
pinza  cerrada    -> según SERVO_AUTO_OFF["grip"]
pinza  abierta    -> se apaga
brazo             -> siempre se apaga
```

La puesta a punto del arranque también: la llave queda **alimentada**.

```
0.00s  ve amarillo -> para en seco
0.05s  suelta, servo a 10°
1.56s  el servo se apaga (1,5 s de asentamiento, no 0,7)
1.86s  SOLTADA. Quieto 1 s más
2.88s  retrocede
```

### 9. La vuelta a casa se atascaba — y el síntoma lo explicaste tú

`RETURN_AFTER_GRAB = True`: tras levantar la bandera, vuelve solo a su zona.

**El bug que viste al levantar el rover por una esquina.** La llegada solo
contaba si el rover estaba «armado», y armarse exigía **ver un color que no
fuera el del equipo ni «?»**. En el aire el APDS no ve nada y clasifica **«?»**,
que no arma. Al dejarlo sobre su propia cinta azul seguía desarmado, así que
**no entregó pese a leer el color correcto** — y entregó «más tarde, de forma
aleatoria», cuando por fin cruzó otro color y se armó.

**El armado se eliminó del todo (19/09).** Protegía de un caso que no puede
darse: que el rover agarrase la bandera pisando ya su propia cinta. **La
bandera rival está siempre sobre la zona rival**, así que en el instante del
agarre el rover pisa el color del enemigo, nunca el suyo. La condición
protegía de algo imposible y a cambio se atascaba. Ahora la primera lectura del
color propio entrega, sin preámbulos.

Y la llegada ya **no espera al color estable**: una sola lectura del color del
equipo basta, igual que el depósito de la llave. Cruzando la cinta no siempre
da tiempo a dos lecturas seguidas.

`TEAM = "blue"` (rival rojo), como lo tenías configurado.

## Ajustes del 19/09

### La llegada a casa, sin condiciones previas

El «armado» (ver otro color antes, o esperar 2 s) **se eliminó**. Protegía de
que el rover entregase en el sitio si agarraba la bandera pisando ya su propia
cinta — un caso que **no puede darse**: la bandera rival está siempre sobre la
zona rival, así que al agarrarla el rover pisa el color del **enemigo**, nunca
el suyo. Menos estado que mantener, menos que pueda fallar.

### Suelta en seco al llegar

`CARRY_ENTER_ADVANCE_S = 0`. La zona propia es **fina**, y 1,1 s de avance a
ciegas la cruzaban entera: la bandera acababa fuera por el otro lado — el mismo
fallo que tuvo la caja amarilla.

Aquí no hay el riesgo que sí tiene la llave: la bandera se deja **donde está el
rover** (primero baja el brazo, luego abre la pinza), no delante. Soltar en el
borde no la deja fuera.

### Dos defectos que salieron al revisar esto

**La pinza no terminaba de abrirse antes de retroceder.** `DELIVER_OPEN_S`
valía 0,7 s y el asentamiento de la pinza 0,9: el rover arrancaba marcha atrás
con la pinza **aún abriéndose** y se llevaba la bandera por delante. Subido a
**1,0**. Importa más ahora que se suelta en el borde de una zona fina.

**El retroceso final duraba un solo ciclo.** La espera genérica de `_deliver`
devolvía cero para *todas* las fases, incluida `back`. En la traza se veía como
un `-143` suelto seguido de ceros: el retroceso duraba **50 ms** en vez de
`DELIVER_BACK_S`, y el rover se quedaba prácticamente encima de la bandera
recién soltada — justo lo que el retroceso existe para evitar. Lo encontró la
traza de la secuencia, no una prueba; ahora hay una que cuenta los ciclos de
marcha atrás.

```
0.20s  ve su color -> para en seco
0.25s  baja el brazo
1.16s  abre la pinza
2.16s  retrocede ... 1,45 s de verdad
3.62s  DONE
```

## Calibración validada en pista (22/09) — todo funcionando

```python
LINE_BLACK_BELOW = 2000    # antes 3200   (y también en sketch/config.h)
COLOR_DARK_CLEAR = 2       # antes 18
```

### El «clear» del negro a 2: por qué tiene sentido

Ha ido **40 → 18 → 2**, y cada bajada resolvió una cinta que se estaba
clasificando como **negro** en vez de como su color.

Con 2, el «negro por color» prácticamente no dispara nunca — **y eso está
bien**: del borde negro se encargan los TCRT, no el APDS. El APDS solo tiene
que separar amarillo, rojo, azul y blanco, y cualquier umbral de oscuridad que
se coma una de esas cintas hace más daño que bien.

**Por qué hizo falta ahora y no antes:** el azul es la cinta que menos luz
devuelve, y la zona propia azul (`TEAM = "blue"`) no se leía hasta la parte 4.
Con 18, el azul caía por debajo y se clasificaba como negro — el rover nunca
reconocía su casa.

(Había un `assert COLOR_DARK_CLEAR >= 5` en las pruebas. Era **mío, no medido**,
y estorbaba. Fuera.)

### ⚠ Las medidas del 03/09 ya no describen este hardware

Aquellas decían (normalizado) **peor negro 3108, peor no-negro 3908**. Con esos
números, un umbral de **2000 no detectaría ningún negro** — 3108 no está por
debajo de 2000. Que en pista funcione significa que **las lecturas han
cambiado**: otra altura de montaje, otra cinta, otra luz, u otro módulo.

Consecuencia práctica: **la tabla de márgenes de `config.py` está obsoleta** y
no sirve para decidir nada. Si hay que retocar el umbral, **no uses aquellos
números**: pulsa «Leer TCRT» en la página sobre blanco y sobre negro, y pon el
umbral en el punto medio. Son dos minutos, y evitan ajustar a ciegas el día de
la competencia.

Lo que **sí** sigue siendo válido es el criterio: los dos errores no cuestan lo
mismo. Un falso negro para el rover donde no debía (molesto); un negro perdido
lo saca de la pista (ronda perdida). Ante la duda, margen del lado del negro.

## LED RGB de equipo (23/09) — R=D13, G=D12, B=D11

`PIN_LED_FLAG` era **D13**. Al montar el RGB con R en D13 el LED de bandera
deja de existir como LED aparte, así que el RGB hace **los dos trabajos** en dos
canales independientes:

```
EL TONO  dice de qué EQUIPO es el rover      rojo o azul, SIEMPRE
EL RITMO dice qué pasa con la BANDERA
    fijo              normal
    parpadeo lento    bandera rival a la vista
    parpadeo rápido   bandera rival en su poder
```

El tono **no cambia nunca**: si la bandera se señalizara con el color, el rover
dejaría de mostrar su equipo justo en el momento más visible de la ronda.

`TEAM = "blue"` en `config.py` produce las tres cosas: qué bandera perseguir,
qué cinta es casa y el tono del LED. Una prueba falla si se desacoplan.

**⚠ Ánodo o cátodo común, sin comprobar.** En la tarjeta de la cámara hay
botones Rojo / Verde / Azul / Blanco / Apagar. Si al pulsar «Rojo» se encienden
el verde y el azul, es ánodo común: `RGB_COMMON_ANODE 1` en `sketch/config.h`.

Solo D11 es PWM de los tres: cada canal está encendido o apagado.

## Servo de la caja: DE ROTACIÓN CONTINUA (28/09)

El servo de la caja pasó de uno de posición (180°) a uno **continuo (360°)**.
Ya no va a un ángulo: **gira a una velocidad**. `write(90)` ≈ parado, y cuanto
más lejos de 90, más rápido hacia ese lado.

### Valores vigentes (28/09, sustituyen a 85/100)

```python
KEY_DROP_ANGLE    = 70     # tirar la caja
KEY_DROP_S        = 0.3
KEY_RETRACT_ANGLE = 115    # recoger el brazo
KEY_RETRACT_S     = 0.3
```

### La secuencia

```
0.00 s  ve amarillo -> PARA
0.05 s  gira 70° durante 0.3 s   -> tira la caja
0.50 s  quieto 1 s               -> la caja se asienta
1.51 s  gira 115° durante 0.3 s  -> recoge el brazo
1.96 s  retrocede
3.82 s  media vuelta -> patrulla
```

**El brazo se recoge ANTES de retroceder**, no después como con el servo de
posición. En pista el brazo extendido se enganchaba al dar marcha atrás;
recogido primero, no hay nada que enganchar.

### El tiempo lo cuenta el MCU, no Python

`Actuators::spin(id, ángulo, ms)` gira y **se para solo** en `tick()`. Tres
razones:

- **Es como se calibró.** La App de prueba temporiza en el MCU; si el rover lo
  hiciera desde Python (a 20 Hz, más la latencia del Bridge) tendría ±50 ms de
  error — un **17 % de un giro de 0,3 s**. Los 0,3 s medidos no serían 0,3 s.
- **Si la App de Python se cae a mitad del giro, el servo se para igual.**
- **Se para quitando los pulsos**, no con `write(90)`: el neutro de un servo
  continuo casi nunca es 90 exacto y suele arrastrar un poco.

### Lo que se quitó, porque ahora sería peligroso

Con el servo continuo, `write(180)` significa **máxima velocidad, sin fin**. Y
el código mandaba 180 a la caja en tres sitios:

| dónde | ahora |
|---|---|
| `begin()` del MCU, al dar corriente | **ningún servo se engancha al arrancar** |
| `main.py`, «retener» al arrancar | `servo_off` por si quedaba algo girando |
| rearme tras soltar, con `keep_on=True` | **eliminado**: el brazo se recoge con un giro |

Y para que no vuelva a pasar por un descuido, **la caja solo se puede mover con
`spin()`**: el MCU rechaza `servo(KEY, x)`, y `main.py` rechaza la orden
`("key", ángulo)` de la FSM. No existe ninguna orden —de la misión, de un botón
de la página, o de un `curl` a mano— que pueda dejarla girando. Tope de
seguridad en el MCU: `KEY_SPIN_MAX_MS = 3000`.

En la página, «Soltar / Retener / ángulos» se sustituyeron por **Tirar caja**,
**Recoger brazo** y **Parar servo**, con los mismos valores que la misión.
«Rearmar llave» ya no mueve el servo: el brazo se coloca a mano al cargar.

### Por qué 70/115 y qué vigilar

La primera medición (85/100) quedaba a 5° del neutro, en la zona donde un
servo continuo es **más sensible al voltaje**: con la batería a media carga
podía no arrancar. 70 y 115 están a 20° y 25°, así que ese riesgo baja mucho.

El precio: **en los mismos 0,3 s gira bastante más**. Si la caja sale
disparada o el brazo golpea el tope al recoger, se **acorta el tiempo**
(0,2–0,25 s), no se vuelve a acercar el ángulo al neutro.

La asimetría (20° de ida, 25° de vuelta) es normal: el neutro real no está en
90 clavado. Si el brazo no vuelve exactamente a su sitio, se ajusta
`KEY_RETRACT_S`.

### Las pruebas cazan lo que importa

Lo que hay que comprobar de un servo continuo no es «va a tal ángulo» —no va a
ninguno— sino que **se para**. Las pruebas del MCU y de la misión están
verificadas rompiendo el código a propósito, y cazan: la caja enganchada al
arrancar, `tick()` que no para el giro, `setRaw` aceptando la caja, la resta sin
signo de `millis()`, `main.py` volviendo a mandar «retener», `act.tick()`
comentado en `loop()`, la FSM mandando un ángulo a la caja, y el brazo
recogido después de retroceder.

Dos de esas las **pasaba** la primera versión de las pruebas: la resta sin
signo (que falla *antes* de que `millis()` dé la vuelta, no después) y el
`act.tick()` comentado (el texto seguía dentro del comentario). Arregladas.

## Botón de equipo en A4 (28/09)

Un pulsador entre **A4 y GND** alterna el equipo **rojo ↔ azul**. Sin
resistencia: el MCU usa el pull-up interno. **No lo conectes a 5 V**: la lógica
del UNO Q es de 3,3 V. (A4 en el UNO Q es PC1 y no tiene nada que ver con el I2C
del APDS, que va por D20/D21 —en el UNO R3 A4 era SDA, aquí no—.)

**Qué hace cada pulsación** (solo en IDLE, DONE o ESTOP):

- cambia la **bandera que busca la cámara** (rival roja ↔ azul);
- cambia la **cinta que cuenta como casa** para entregar;
- cambia el **tono del LED RGB**: el LED es lo que dice el equipo vigente.

**En plena ronda se ignora**, y lo pulsado durante la ronda **no se aplica
después**. Un roce en pista no cambia nada.

`config.TEAM` es el equipo **al arrancar la App**. El botón no se guarda: si la
App se reinicia, vuelve a `config.TEAM`. **Mira el LED antes de cada ronda.**

### Cómo está hecho

- **MCU (`sketch/button.h`)**: antirrebote de 30 ms y un **contador de
  pulsaciones** que viaja en `sense()` (campo 20; el bit `0x8000` de flags dice
  si está pulsado ahora). Se manda un contador y no el nivel porque Python lee
  a unas decenas de Hz y una pulsación corta podría caer entre dos lecturas.
- **Si arranca ya pulsado** (atascado, dedo encima) no cuenta hasta soltar y
  volver a pulsar. Sin botón conectado, el pull-up deja el pin en alto y nunca
  cuenta.
- **Python** mira las pulsaciones nuevas en cada ciclo. Lo pulsado antes de
  arrancar la App no cuenta; si el MCU se reinicia (el contador baja), tampoco,
  y además se reenvía el tono del LED, que el MCU pierde al reiniciarse.
- `config.set_team()` recalcula `ENEMY`, `TEAM_LED` y `TEAM_COLOR` en el mismo
  sitio donde se definen. Funciona porque el resto del código los lee al
  usarlos; la única copia es la de la cámara, que se cambia con
  `vision.set_enemy()` (y descarta el frame que estuviera procesando con el
  color viejo).

La página muestra el **equipo vigente** y el **estado del botón** (suelto /
PULSADO · N pulsaciones) en «Estado y control». Si pulsas y el contador no
sube, es cableado; si sube y el equipo no cambia, el rover está en ronda.

## Saltar a un estado desde la página (pruebas)

La tarjeta **«Saltar a un estado»** tiene un botón por estado. Sirve para
probar la parte que te interesa sin recorrer las anteriores: ver el rebote en
amarillo ya no obliga a depositar la llave primero.

Dos cosas que no son un `request()` pelado, y por qué:

- **Cada estado entra por su primera fase.** Los estados con maniobra guardan
  su fase (`_key_phase`, `_edge_phase`) y su plazo. Entrar sin ponerlos deja
  la máquina en un punto que su propio código no contempla: `DEPOSIT_KEY` con
  la fase a `None` cae por la rama «drop», ve el plazo vencido y se va a
  `BACKOFF_KEY` **sin haber soltado nada** — o sea, el botón «probar depósito»
  no probaría el depósito. Cada estado tiene su preparación en
  `Mission.force_state()`.
- **Los que mueven exigen lo mismo que «Iniciar patrulla»**: TCRT frontales
  presentes y telemetría fresca. Van marcados con ⚙. Sin eso el rover
  conduciría ciego hacia el borde. El salto se rechaza y el motivo sale en la
  página.

`ESTOP` no está en la lista: tiene su propio botón y su propio rearme, y desde
ESTOP no se fuerza nada. Pulsar el estado en el que ya estás **reinicia** su
maniobra desde la primera fase. La lista de botones la manda la propia FSM
(`fsm.forceable`), no una copia en el HTML: si mañana se añade un estado, el
botón aparece solo.

`APPROACH` forzado con la llave a bordo mueve el rover para que puedas probar
el control, pero **no toca el cerrojo del reglamento**: `PATROL` sigue sin
perseguir por su cuenta.

## PARTE 4 — agarrar la bandera y volver a la zona propia

**Qué hace.** Desde `APPROACH`, cuando la bandera está a `GRAB_DISTANCE_CM`
pasa a `FLAG_REACHED`; ahí se **centra** antes de tocarla; cierra la pinza,
sube la elevación, y vuelve buscando **la cinta de su color** con el APDS del
suelo; al llegar entra un poco en la zona, baja el brazo, abre la pinza,
retrocede y termina en `DONE`.

```
PATROL ─ve bandera─> APPROACH ─a 6 cm─> FLAG_REACHED ─centrada─> GRAB
                                                                  │
                                        HOLDING <──pinza cerrada, brazo arriba
                                           │
              (a mano, botón CARRY)        ▼
                          CARRY ─entra en su cinta─> DELIVER ─> DONE
```

**La secuencia termina en `HOLDING`**, a propósito: `RETURN_AFTER_GRAB = False`.
Acercarse, agarrar, levantar, quieto. La vuelta a casa está implementada y
probada, pero encadenar sola una etapa que aún no se ha visto funcionar en el
suelo solo sirve para no saber cuál de las dos falló. Se lanza a mano con el
botón `CARRY`; cuando funcione, pon el interruptor a `True`.

### El acercamiento, en tres tramos

| distancia | velocidad | por qué |
|---|---|---|
| > 60 cm | `SPEED_APPROACH` 260 | crucero |
| 60 → 14 cm | rampa 260 → 170 | frena con la distancia |
| 14 → 6 cm | `SPEED_FINAL` **120** | **arrastre** |
| ≤ 6 cm | 0 | parado, se centra y agarra |

### El número más ajustado de la parte 4

El ancho **también se satura**: cuando la bandera llena los 640 px, medirla
más ancha es imposible y la distancia se queda clavada — el mismo fallo que
tenía el alto, solo que 20 cm más cerca. Con `FLAG_DIST_A = 2775` eso pasa a
**4,3 cm**, y el agarre está a **6**: quedan **1,7 cm**.

1,7 cm es poco, y la única defensa es no llegar rápido. De ahí dos cosas:

- **El tramo de arrastre** (`FINAL_APPROACH_CM` / `SPEED_FINAL`). A velocidad
  de creep un solo ciclo de control ya recorre varios milímetros; el rover no
  se pasaría por poco, se metería dentro de la bandera.
- **El filtro de mediana se apaga de cerca.** Existe porque de lejos el ancho
  es diminuto (a 100 cm son ~28 px y ±2 px son ±7 %). A 6 cm son 462 px y esos
  mismos ±2 px son ±0,4 %: ahí el filtro no aporta nada y sus 0,3 s de retardo
  son puro riesgo, justo donde solo hay 1,7 cm de margen. Por encima de
  `FLAG_DIST_TRUST_PX` la lectura se usa cruda.

Si en pista ves que se mete dentro de la bandera: baja `SPEED_FINAL` antes que
subir `GRAB_DISTANCE_CM`, y si aun así, sube `FINAL_APPROACH_CM`.

### La distancia sale SOLO del ancho

Ni el alto ni los 15 cm entran en el cálculo. Dos razones:

**Geometría.** Con el cuarto superior borrado por el horizonte quedan 360 px
útiles de alto. Con la focal medida, la bandera deja de caber a partir de
~23 cm y su alto se queda **clavado** en 360 px — y la distancia estimada con
él, así que `d <= GRAB_DISTANCE_CM` no se cumplía nunca y el rover **empujaba
la bandera** por la pista. El ancho no se satura hasta ~7 cm.

**Física, y es la razón de fondo.** La bandera es un **cilindro**: su diámetro
son 5 cm se mire desde donde se mire, y la silueta horizontal es una medida
limpia. Su «altura» son 15 cm solo si se ven el borde de arriba **y** el de
abajo, y en la práctica la base queda tapada por el suelo, por su sombra o por
el borde del encuadre. **El alto es la dimensión que miente.**

El mismo ancho da además el **centrado**: el centro horizontal de la caja es
`error_x`. Y si la mancha toca un borde lateral, parte de la bandera está
fuera de cuadro y su centro real está más allá del que se ve — el error se
corrige hacia ese lado, para que el rover no se dé por centrado con media
bandera fuera.

### El modelo tiene DOS términos, y por eso ajustar la focal no bastaba

```
distancia = FLAG_DIST_A / ancho_px + FLAG_DIST_B
```

`B` es un **desplazamiento constante** en centímetros. El modelo de cámara
estenopeica mide desde el **centro óptico de la lente**; las distancias que
importan (y las que mides con cinta métrica) se toman desde la **pinza**.
Entre un punto y otro hay un número fijo de centímetros, y **ninguna focal lo
puede absorber**, porque no es un factor de escala: es un sumando.

Ese es exactamente el síntoma de «cambio la focal y sigue sin cuadrar de
cerca». Con un desfase real de 5 cm y calibrando a 80 cm:

| distancia real | lo que da un modelo de un solo punto | error |
|---|---|---|
| 80 cm | 80,0 cm | 0 % — en el punto de calibración siempre cuadra |
| 20 cm | 16,0 cm | **−20 %** |
| 14 cm | 10,6 cm | **−24 %** — el agarre falla |

### Calibrar: dos puntos, tres minutos

**Con la página** (lo más cómodo): tarjeta «Calibrar la distancia (dos
puntos)». Escribes la distancia real, pulsas «Capturar ancho», repites cerca,
y te da las dos líneas para pegar en `config.py`. También te dice cuánto habría
fallado con un solo punto, para que veas si el desfase importaba.

**Desde la terminal:**

```
python3 tools/calib_dist.py 80 <ancho1> 20 <ancho2>
```

Reglas de la medida:

- Una **lejos** (~80 cm) y otra **cerca** (~20 cm). Cuanto más separadas,
  mejor sale `B`; dos medidas parecidas dan un `B` basura.
- Mide siempre desde el **mismo punto del rover las dos veces**. Lo más útil
  es la **pinza**: así `GRAB_DISTANCE_CM` se lee directo, sin conversiones.
- Que la bandera **no** toque un borde lateral (la página lo marca con ⚠): el
  ancho estaría truncado.

### El ancho es más ruidoso que el alto, y hay que compensarlo

Es la contrapartida honesta de medir por el ancho: 5 cm frente a 15 son un
rasgo **tres veces más pequeño**, así que un píxel de error en la máscara pesa
el triple. A 100 cm el ancho son ~28 px en el frame completo (14 en el
reducido por `DETECT_SCALE = 0.5`), y ahí ±2 px ya son ±7 % de distancia.

Por eso la distancia que usa el control es la **mediana de las 3 últimas**
lecturas (`FLAG_DIST_MEDIAN_N`). Mediana y no media: un frame en el que la
máscara se parte da un ancho absurdo, y una media lo reparte entre todas las
lecturas mientras que una mediana lo tira. A 10 fps son 0,3 s de retardo. La
página muestra las dos: la filtrada y, entre paréntesis, la cruda — si bailan
mucho, el problema está en la máscara, no en el filtro.

### Centrar antes de cerrar

Estar cerca no basta: hay que estar **centrado**. Cerrar la pinza con la
bandera a un lado del eje la **tira**, y una bandera tumbada ya no se recoge.
En `FLAG_REACHED`, si `|error_x| > GRAB_ALIGN_TOL` el rover **pivota sin
avanzar** — avanzar a esa distancia solo sirve para empujarla.

### El compromiso de agarre

Muy cerca la bandera puede salirse del encuadre (por abajo, por arriba, o
porque el filtro de ancho la rechaza por debajo de ~10 cm). Si la detección se
pierde **dentro de `GRAB_COMMIT_CM`**, no se abandona: a esa distancia estaba
delante, así que se agarra igual. Sin esto, perder la bandera en el último
palmo significaría no agarrarla nunca. Si se pierde lejos, se abandona
normalmente.

### La secuencia mecánica

Pinza primero, elevación después, **de una en una y con el rover parado**. Dos
ZOSKAY de 20 kg arrancando a la vez —o uno arrancando con los motores en
marcha— es el pico de corriente más alto del rover: es lo que hunde el rail de
5 V y reinicia placas. El MCU además bloquea los motores `MOVE_LOCK_MS` tras
cada orden de servo. Un borde negro **no interrumpe** una fase de servo con el
rover ya parado (cortarla deja la pinza a medio cerrar); sí interrumpe las
fases que avanzan.

En la entrega el orden es al revés y también importa: **bajar antes de abrir**.
Abrir en alto deja caer la bandera desde la altura del brazo y lo más probable
es que ruede fuera de la zona.

### Cómo vuelve a casa, y qué limita eso

**El rover no tiene encoders.** No hay odometría, no sabe dónde está, y no
puede «ir a la coordenada de su base». Lo único que sabe con certeza es qué
color pisa. Así que la vuelta es reactiva: avanza rebotando en el borde negro
igual que en `PATROL` y termina cuando el APDS ve `TEAM_COLOR`.

Eso **no es una ruta, es una búsqueda**: el tiempo hasta llegar no está acotado
y depende de dónde estuviera la bandera. Si hiciera falta acotarlo, los
encoders siguen siendo la mejora número uno del proyecto.

La llegada es **directa**: la primera lectura del color propio entrega. Hubo un
«armado» previo y se quitó — ver el apartado de los ajustes del 19/09.

**Las interrupciones vuelven a donde estaban.** `EDGE` devolvía *siempre* a
`PATROL`; con la bandera en la pinza eso significaba que el rover rebotaba en
el borde y se le olvidaba que volvía a casa — y peor, `PATROL` habría intentado
perseguir otra bandera llevando una puesta. Ahora `_resume` recuerda el destino
y `_hunt_allowed()` se cierra en cuanto `has_flag`.

### LAGUNA CONOCIDA: no hay forma de saber si la pinza agarró algo

No hay sensor en el gripper. `has_flag` se pone a `True` porque el código
**mandó** cerrar la pinza, no porque sepa que hay algo dentro. Si cierra en
vacío —bandera mal centrada, se cayó, se atascó— el rover se vuelve a su zona
con las manos vacías, ejecuta la entrega y marca la misión cumplida.

Esto no se puede arreglar con software. Las salidas reales son un microswitch
en el dedo de la pinza (lo más barato y fiable), el segundo APDS-9960 mirando
al interior de la pinza, o una lectura de corriente del servo. Mientras tanto:
**mira el rover**, no la página, en la prueba de agarre.

### AVISO: ya no queda ningún filtro geométrico

`FLAG_MAX_WIDTH_FRAC = 1.0` **es necesario** para llegar a 6 cm — ahí la
bandera ocupa el 72 % del encuadre y con 0,6 se rechazaba justo cuando hay que
agarrarla. Pero junto con `FLAG_MIN_ASPECT = 0.1` deja pasar **cualquier**
mancha del color rival que esté bajo el horizonte.

Hay una salida que no cuesta nada. Como el alto se recorta al horizonte, el
aspecto de la bandera **nunca baja de ~0,6** aunque esté pegada:

| distancia | ancho px | alto px (recortado) | aspecto |
|---|---|---|---|
| 40 cm | 69 | 208 | 3,01 |
| 20 cm | 139 | 360 | 2,59 |
| 10 cm | 278 | 360 | 1,30 |
| **6 cm** | 462 | 360 | **0,78** |
| 4,5 cm | 617 | 360 | 0,58 |
| *cinta del suelo* | | | *0,10 – 0,30* |

`FLAG_MIN_ASPECT = 0.5` conserva la bandera en todo el rango útil (hasta
4,5 cm, muy por debajo de los 6 de agarre) y rechaza la cinta. Un solo número,
sin tocar nada más. No está aplicado porque tus 0,1 están medidos en pista;
pruébalo cuando quieras comprobar la cinta.

### Ángulos del brazo

| Servo | Abierto/abajo | Cerrado/arriba | Pin |
|---|---|---|---|
| Pinza | **0°** | **90°** | D9 |
| Elevación | **0°** | **40°** | D6 |

`begin()` del MCU **no engancha** estos dos servos, y es a propósito: el brazo
se coloca a mano antes de la ronda, y un servo de 20 kg empujando mientras lo
colocas es una forma de romper el mecanismo. Se enganchan solos con la primera
orden, que la manda `main.py` al arrancar («posición de partida»).

**`LIFT_DETACH_AFTER_UP` está pendiente de decidir con una prueba física:** sube
el brazo *con la bandera*, pulsa «Apagar elevación» en la página y mira si
aguanta solo. Si aguanta, ponlo a `True` y el rover se ahorra la corriente de
retención durante toda la vuelta. Si cae, déjalo en `False`. La **pinza no se
apaga nunca** con la bandera dentro: apagarla la suelta.

### Repetir la prueba

«Rearmar bandera» en la página: vuelve a considerarla no agarrada **y** manda
el brazo a posición de partida (si no, tendrías que colocar la bandera con la
pinza cerrada).

## Pruebas del código del MCU (no solo compilar)

```
sh tools/test_sketch.sh
```

Compila el sketch entero con stubs y además **ejecuta** pruebas de
`Actuators`, `Motors` y `Signals`. Existe por un fallo real: una llamada a
`Servo::write()` antes de `attach()` **compilaba perfectamente** y tumbaba el
MCU dentro de `setup()`, dejando el rover mudo dos sesiones.

Los stubs de `tools/stubs/` emulan la semántica de **zephyr**, no la de AVR, y
esa diferencia es justo el punto:

| | AVR | zephyr (UNO Q) |
|---|---|---|
| `write()` sin `attach()` | `if (channel < MAX_SERVOS)` → se ignora | **sin comprobación**: indexa `servos[255]` sobre un array de 16 |

Por eso el stub de Servo **aborta** si se escribe antes de enganchar: ese
error se caza aquí y no en la placa. (Comprobado: si se reintroduce el bug,
la prueba falla.)

## Si el MCU no responde («method not available»)

Síntoma: la página con todo a cero, «Sensores frontales / traseros: NO / NO»,
y en la consola `method sense not available (2)` una y otra vez. **Eso no es
un enlace roto: es que el MCU no tiene registrados los RPC.**

Al arrancar, la App hace un **sondeo de vida** y lo dice en una línea:

```
[init] sketch vivo y loop() corriendo (etapa 7/7)          ← todo bien
[init] EL SKETCH ARRANCO PERO SE QUEDO EN LA ETAPA 3/7     ← se colgó un begin()
[init] EL SKETCH NO CONTESTA NI AL SONDEO BASICO (alive)   ← no compiló / no se subió
```

El truco: `alive` es el único RPC registrado con `provide()` en vez de
`provide_safe()`, así que **lo atiende el hilo propio del Bridge** y contesta
aunque `setup()` o `loop()` estén colgados. Y no depende del monitor serie,
que en App Lab sobre Windows tiene un bug conocido y puede no mostrar nada.

Las etapas: 1 Bridge · 2 motores · 3 TCRT · 4 APDS (I2C) · 5 servo · 6 LED ·
7 `loop()` corriendo. **El número que sale es lo último que terminó; lo que
colgó es lo siguiente.**

Además, los RPC ahora se registran **antes** que el hardware. Con el orden
anterior, un `begin()` colgado dejaba el sketch sin ningún método registrado,
y desde el MPU eso se ve idéntico a «el sketch no está cargado».

**Si el sondeo dice que no contesta**, el error está en la pestaña
**«App launch»** de App Lab, que es donde sale la compilación del sketch (la
pestaña «Python» solo muestra la App). Los dos errores que ya nos han pasado
ahí: `Missing Profile name … has no default profile` e `invalid library
reference: Servo ()`.

**Al actualizar el proyecto, borra la carpeta vieja antes de descomprimir.**
Descomprimir encima NO borra los archivos que yo haya eliminado — por ejemplo
`python/requirements.txt`, que si sigue ahí vuelve a romper la resolución de
dependencias.

## PENDIENTE de medir — en orden de urgencia

Lo primero de la lista **bloquea la parte 4**: sin la focal medida, la
distancia estimada es un número inventado y el agarre dispara donde no debe.

| # | Qué | Ahora | Cómo se mide |
|---|---|---|---|
| ~~1~~ | ~~`FOCAL_PX`~~ | **555 — MEDIDO 04/09** (98 px a 85 cm) | hecho |
| 1 | Alcance: por qué no ve a 100 cm | `FLAG_NORM_AREA_PX = 4000` | banco, bandera a 100 cm, leer el motivo de rechazo (ver «Alcance» arriba) |
| 2 | `GRAB_DISTANCE_CM` | 14 | coloca la bandera donde la pinza la abrace bien y lee «distancia» en la página. Es cámara→bandera, **no** frente del chasis→bandera |
| 3 | `LIFT_DETACH_AFTER_UP` | `False` | sube el brazo con la bandera, «Apagar elevación», ¿aguanta? |
| 4 | `CARRY_ENTER_ADVANCE_S` | 0,6 s | que la bandera quede dentro de la zona, no en el borde |
| 5 | `ZONE_ENTER_ADVANCE_S` | 0,8 s | de la parte 2: que la caja caiga dentro |
| 6 | `KEY_TURN_S` | 1,6 s | de la parte 2: cronometra un giro completo y divide por dos |
| 7 | `PATROL_SPEED` máximo | 300 | subir de 50 en 50 hasta que el margen de frenado deje de bastar |
| 8 | `GRAB_BLIND_S` | 0 s | solo si al comprometerse la pinza se queda corta: subir de 0,1 en 0,1 |

Además: `SERVO_KEY_HOLD` / `SERVO_KEY_RELEASE` siguen en 180 / 0 (probados y
funcionando); el rango medido 135 → 0 sigue **sin aplicar** a propósito.

## Orden de las primeras pruebas de la parte 4

1. **Sin bandera y con el rover en la mano.** «Abrir pinza» / «Cerrar pinza» /
   «Bajar» / «Subir»: comprueba topes y que nada choca con el chasis.
2. **Focal** (punto 1 de la tabla). Sin esto no sigas.
3. **Calibrar A y B** con la tarjeta de la página (dos puntos, medidos desde
   la pinza). Después acerca la bandera a mano y comprueba que la distancia
   baja de forma continua **y cuadra con la cinta métrica en todo el rango**,
   no solo en los dos puntos que capturaste.
4. **Agarre estático:** rover parado, bandera delante a la distancia buena,
   botón `GRAB`. Mira el rover, no la página: la página *no sabe* si agarró.
5. **Vuelta a casa sola:** botón `CARRY` con la bandera ya colocada a mano en
   la pinza. Comprueba que rebota en el borde y **vuelve a CARRY**, no a
   PATROL.
6. **Entrega:** botón `DELIVER` sobre tu cinta.
7. **Todo seguido**, desde «Iniciar patrulla», con la llave cargada.

## Siguiente parte (no incluida aquí)

5. LEDs de equipo (D11 rojo / D12 azul) y verificación del agarre (microswitch
   en el dedo de la pinza o segundo APDS-9960 mirando dentro).
