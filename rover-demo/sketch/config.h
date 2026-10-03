#pragma once
/*
 * config.h - Pines y valores por defecto del MCU (Arduino UNO Q, STM32U585).
 *
 * PARTE 1 DEL DEMO: identificar lineas y no salirse de la pista.
 * Solo esta lo necesario para eso: 4 TCRT-5000, el APDS-9960 del piso y la
 * traccion. Servos, LEDs de equipo y APDS del gripper llegan en partes
 * posteriores.
 *
 * LOS PINES SON LOS MISMOS que en la version anterior: el cableado no cambio.
 *
 * NIVELES (esto quema placas): todo el GPIO es de 3,3 V. Los pines
 * ANALOGICOS NO toleran 5 V (max ~3,6 V): los TCRT-5000 van alimentados a
 * 3,3 V, NUNCA a 5 V. D0/D1 no se tocan: el Bridge usa Serial1.
 *
 * DONDE ESTA CADA UMBRAL
 * ----------------------
 * Los umbrales de calibracion viven en python/config.py, en el MPU. Este
 * archivo solo lleva VALORES POR DEFECTO para que el reflejo de borde
 * funcione aunque la App Python no haya arrancado; al arrancar, la App manda
 * los suyos con cfg_lines(). Calibrar = editar config.py y reiniciar la App,
 * sin recompilar el sketch.
 */

// ================================================================ TRACCION
// L298N: ENA/ENB por PWM (solo D3, D5, D6, D9, D10, D11 tienen PWM), INx por nivel.
//
// AJUSTADO EN BANCO (03/09): los dos motores giraban al reves, y se corrigio
// intercambiando IN1 con IN2 en CADA motor. Los enables (3 y 5) no se
// tocaron. Intercambiar el par IN1/IN2 invierte el sentido de giro, que es
// exactamente lo mismo que hacen MOTOR_L_INVERT / MOTOR_R_INVERT: si algun
// dia prefieres dejar los pines en el orden del cableado fisico y corregir
// el sentido por software, pon los INVERT a true y devuelve estos numeros a
// su orden. Las dos formas son equivalentes; lo que NO hay que hacer es las
// dos a la vez, porque se cancelan.
#define PIN_MOT_L_EN     3      // ENA (PWM)
#define PIN_MOT_L_IN1    4
#define PIN_MOT_L_IN2    2
#define PIN_MOT_R_EN     5      // ENB (PWM)
#define PIN_MOT_R_IN1    8
#define PIN_MOT_R_IN2    7

#define MOTOR_L_INVERT   false // true si esa rueda gira al reves
#define MOTOR_R_INVERT   false

// MEDIDO en el suelo con peso real y bateria: arranca a PWM 76; 83 = +9 % de
// margen para que siga respondiendo con la bateria a media carga.
#define MOTOR_DEADZONE_PWM   83
#define MOTOR_MAX_PWM       255
#define MOTOR_SLEW_PER_TICK  14   // rampa: evita picos de corriente

// ================================================================== SERVOS
// PARTE 4: los tres. Pines PWM del UNO Q: D3, D5, D6, D9, D10, D11.
#define PIN_SERVO_LIFT   6      // elevacion del gripper
#define PIN_SERVO_GRIP   9      // pinza
#define PIN_SERVO_KEY   10      // biela-manivela: tumba la llave

// Angulos de ARRANQUE de elevacion y gripper. Al contrario que la llave,
// estos NO se enganchan en begin(): el brazo se coloca a mano antes de la
// ronda y un servo que empuja mientras lo colocas es una forma de romperlo.
// Se enganchan solos con la primera orden del MPU (ver Actuators::move).
// MEDIDOS EN EL ROVER: elevacion 0 abajo / 40 arriba, gripper 0 abierto /
// 90 cerrado. Tienen que coincidir con SERVO_LIFT_* y SERVO_GRIP_* de
// python/config.py.
#define SERVO_LIFT_DOWN_DEFAULT   0
#define SERVO_GRIP_OPEN_DEFAULT   0

// SERVO DE LA CAJA: DE ROTACION CONTINUA (360) DESDE EL 28/09.
//
// Ya no va a una POSICION: gira a una VELOCIDAD. write(90) ~ parado, y cuanto
// mas lejos de 90, mas rapido hacia ese lado. Por eso aqui YA NO HAY angulo de
// retencion al arrancar: con el codigo anterior, begin() mandaba write(180) al
// dar corriente, que en un servo continuo es "maxima velocidad, sin fin".
//
// La caja se tira con Actuators::spin(): gira un tiempo y SE PARA SOLO, con el
// reloj del MCU -- el mismo mecanismo con el que se calibro en la App de
// prueba (servo-test), asi que los 0.3 s medidos alli son los 0.3 s de aqui.
//
// Tope de seguridad: ninguna orden hace girar la caja mas de esto. Los giros
// de verdad son de ~0.3 s; un numero absurdo del MPU no puede dejarlo girando.
#define KEY_SPIN_MAX_MS   3000

// ALIMENTACION: el servo de 20 kg NUNCA se alimenta del 5 V de la placa.
// UBEC propio desde la bateria, con la masa unida a la comun. Un ZOSKAY
// contra su tope mecanico tira varios amperios: es lo que mata placas.
#define SERVO_MOVE_MS    600    // tiempo estimado de recorrido completo
#define MOVE_LOCK_MS     250    // motores bloqueados tras mover un servo

// ============================================================== LED RGB
// UN SOLO LED RGB hace los DOS trabajos de senalizacion.
//
// EL PIN D13 ESTABA OCUPADO. Era el LED de bandera (y el integrado de la
// placa). Al montar el RGB con R en D13 ya no hay dos LEDs: hay uno. Asi que
// el RGB senaliza las dos cosas a la vez, y sin ambiguedad:
//
//     EL TONO dice DE QUE EQUIPO es el rover   (rojo o azul, siempre)
//     EL RITMO dice que pasa con la bandera:
//         fijo            normal
//         parpadeo lento  bandera rival A LA VISTA
//         parpadeo rapido bandera rival EN SU PODER
//
// El tono NUNCA cambia, asi que un juez que mire de reojo sigue viendo el
// equipo correcto pase lo que pase. Si se hiciera al reves -cambiar el color
// para senalizar la bandera- el rover pareceria del otro equipo justo en el
// momento mas visible de la ronda.
#define PIN_LED_R          13     // tambien es el LED integrado de la placa
#define PIN_LED_G          12
#define PIN_LED_B          11

// COMUN ANODO o COMUN CATODO: cambia la polaridad de los tres canales.
//   0 = comun CATODO  (pata comun a GND,  HIGH enciende)
//   1 = comun ANODO   (pata comun a 5 V,  LOW  enciende)
// NO ESTA COMPROBADO cual es el tuyo. Si al pulsar "Rojo" en la pagina se
// enciende todo menos el rojo, es el otro: cambia este 0 por 1 y recarga.
#define RGB_COMMON_ANODE    0

// Solo D11 es PWM de los tres (D12 y D13 no lo son en el UNO Q), asi que no
// hay mezcla de intensidades: cada canal esta encendido o apagado. Con eso
// salen los 7 colores basicos, y solo hacen falta dos.
#define FLAG_BLINK_MS     200     // semiperiodo del parpadeo LENTO (a la vista)
#define FLAG_BLINK_FAST_MS 90     // semiperiodo del parpadeo RAPIDO (en poder)

// ====================================================== BOTON DE EQUIPO
/*
 * Pulsador para ELEGIR EL EQUIPO (rojo / azul) en A4, entre el pin y GND
 * (sin resistencia: se usa el pull-up interno). Suelto = HIGH, pulsado = LOW.
 *
 * A4 en el UNO Q es PC1 y NO comparte nada con el I2C del APDS (D20/D21 =
 * PB10/PB11): no es como en el UNO R3, donde A4 era SDA.
 *
 * El MCU solo CUENTA pulsaciones (con antirrebote); que hacer con ellas lo
 * decide el MPU, que es quien sabe de equipos (camara, zona de casa, LED).
 * Se manda un CONTADOR y no el nivel "pulsado ahora" a proposito: el MPU lee
 * a 20-50 Hz y una pulsacion rapida podria caer entre dos lecturas. Un
 * contador no se pierde, y un boton atascado no cuenta mas de una vez.
 */
#define PIN_TEAM_BTN          A4
#define TEAM_BTN_DEBOUNCE_MS  30    // el nivel tiene que aguantar esto quieto

// ======================================================= SENSORES DE LINEA
/*
 * Cuatro TCRT-5000, uno en cada esquina, leidos por ADC.
 *
 *        FL  o---------------o  FR      <- frente
 *            |    chasis     |
 *        RL  o---------------o  RR      <- atras
 *
 * Mascara de bits: bit0 FL, bit1 FR, bit2 RL, bit3 RR (igual en Python).
 */
// OJO: FL va en A1 y FR en A0 (invertidos respecto al orden natural).
// Se cambiaron los dos cables entre si para facilitar el conexionado, y el
// cambio se refleja AQUI y solo aqui: la mascara de bits (bit0 = FL,
// bit1 = FR), la maniobra de escape y las etiquetas de la pagina siguen
// hablando de esquinas, no de pines. Si alguna vez vuelves a moverlos, este
// es el unico sitio que hay que tocar.
#define PIN_LINE_FL     A1    // frontal IZQUIERDO  <- cable en A1
#define PIN_LINE_FR     A0    // frontal DERECHO    <- cable en A0
#define PIN_LINE_RL     A2
#define PIN_LINE_RR     A3

/*
 * QUE SENSORES ESTAN MONTADOS. Un pin analogico sin sensor FLOTA y su ruido
 * suele caer por debajo del umbral de negro: el rover "ve" un borde
 * permanente y se planta sin llegar a ninguna linea (paso en las pruebas
 * anteriores). Los no declarados se fuerzan a "pista limpia".
 * OJO: un sensor ausente NO protege. Sin los dos frontales, la patrulla no
 * arranca (lo impide la App).
 */
#define LINE_PRESENT_MASK   0b1111

/*
 * RESOLUCION DEL ADC: fijada a 12 bits en Lines::begin(). El ADC del
 * STM32U585 es configurable y analogRead() podria arrancar en 10 bits; sin
 * fijarla, ningun umbral significa nada. Toda la calibracion previa esta en
 * escala 0..4095.
 */
#define LINE_ADC_BITS       12
#define LINE_ADC_MAX      ((1 << LINE_ADC_BITS) - 1)   // 4095

/*
 * POLARIDAD: MEDIDO con estos modulos: el negro SUBE (blanco ~160, negro
 * ~1000 crudos). Con LINE_INVERT 1 se normaliza en lines.h (v = MAX - v) y
 * TODO lo demas habla en la convencion "menos = mas oscuro".
 */
#define LINE_INVERT           1

/*
 * UMBRAL DE NEGRO por defecto (normalizado). MEDIDO: peor negro 3108, peor
 * no-negro 3908. La App lo sobreescribe con el valor de config.py via
 * cfg_lines(); este solo cuenta si la App no ha arrancado.
 *
 * TIENE QUE COINCIDIR con LINE_BLACK_BELOW de python/config.py. Si divergen,
 * el rover se comporta de una forma durante el arranque y de otra despues, y
 * ese es un fallo muy desagradable de diagnosticar.
 */
#define LINE_BLACK_BELOW_DEFAULT  2000
#define LINE_HYST_DEFAULT          120
#define LINE_SAMPLES                 4   // promedio por lectura

// ================================================= SENSOR DE COLOR (I2C)
/*
 * APDS-9960 del piso en Wire (SDA = D20, SCL = D21). El MCU manda r,g,b,c
 * CRUDOS; la clasificacion la hace el MPU (python/colors.py).
 *
 * Ganancia e integracion: los umbrales de config.py estan MEDIDOS con 4X y
 * 10 ms. Si cambias esto, hay que volver a medirlos.
 *
 * El APDS NO ilumina para el color (su LED es IR, para proximidad): el LED
 * BLANCO apuntando al suelo junto al sensor es obligatorio.
 */
#define APDS_GAIN            APDS9960_AGAIN_4X
#define APDS_INTEGRATION_MS  10
// 25 ms (~40 Hz) y no 50: el sensor es el eslabon mas lento de la cadena de
// deteccion de color, y cada ms suyo se paga dos veces (una en la lectura y
// otra en el filtro de estabilidad del MPU, que necesita N lecturas). La
// integracion es de 10 ms, asi que 25 es holgado. Si el I2C se resiente, el
// guardian de COLOR_MAX_MS/COLOR_SLOW_LIMIT lo desconecta solo.
#define APDS_POLL_MS         25    // ~40 Hz

// Si una lectura I2C tarda mas de esto COLOR_SLOW_LIMIT veces seguidas, el
// APDS se desconecta: un I2C colgado bloquea loop(), y loop() es quien corre
// el reflejo de borde y recibe las ordenes del MPU.
#define COLOR_MAX_MS         15
#define COLOR_SLOW_LIMIT      5

// =============================================================== SEGURIDAD
// El enclavamiento del borde se suelta solo tras EDGE_LATCH_CLEAR_MS de
// pista limpia Y cuando el MPU deja de empujar hacia el lado prohibido.
// No bajar de ~150 ms: a velocidad alta el bloqueo duraria menos que el
// cruce de la cinta (bug real de la primera version).
#define EDGE_LATCH_CLEAR_MS  250

#define CMD_TIMEOUT_MS    400     // sin ordenes del MPU -> parada
#define CONTROL_TICK_MS    10     // 100 Hz

// Una pasada de loop() por encima de esto se cuenta como "lenta". El tick es
// de 10 ms; 50 ms ya es cinco veces el presupuesto y merece contarse. Sirve
// para distinguir "se atasco una vez" de "se atasca continuamente".
#define LOOP_SLOW_MS       50
