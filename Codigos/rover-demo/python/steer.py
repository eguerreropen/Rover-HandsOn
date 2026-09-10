"""steer.py - Ley de direccion y perfil de velocidad para perseguir la bandera."""

import config


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def steer_from_error(error):
    """
    error: posicion horizontal del objetivo en [-1, 1] (negativo = izquierda).
    Devuelve el giro en unidades del MCU. Positivo = a la derecha (la mezcla
    resta el giro a la rueda derecha).

    Curva no lineal: cerca del centro responde poco (no persigue el ruido del
    detector) y lejos responde fuerte. Con una camara a 15 fps un proporcional
    lineal y agresivo oscila siempre.
    """
    if abs(error) < config.STEER_DEADBAND:
        return 0.0
    sign = 1.0 if error >= 0 else -1.0
    shaped = sign * (abs(error) ** config.STEER_CURVE)
    return clamp(config.STEER_SIGN * config.STEER_KP * shaped,
                 -config.TURN_MAX, config.TURN_MAX)


def approach_speed(distance_cm):
    """Perfil de velocidad por distancia, en TRES tramos:

        > APPROACH_SLOW_CM    SPEED_APPROACH   crucero
        hasta FINAL_APPROACH  rampa            de crucero a creep
        hasta GRAB_DISTANCE   SPEED_FINAL      arrastre
        <= GRAB_DISTANCE      0                parado

    EL TRAMO DE ARRASTRE NO ES DECORATIVO. Con el agarre a 6 cm y el sensor
    saturandose a 4.3, quedan 1.7 cm de margen. A velocidad de creep un solo
    ciclo de control ya recorre varios milimetros y cualquier retardo se come
    el margen entero: el rover no se pasa por poco, se mete dentro de la
    bandera. El ultimo tramo se recorre despacio a proposito.
    """
    if distance_cm is None:
        return config.SPEED_APPROACH
    if distance_cm <= config.GRAB_DISTANCE_CM:
        return 0
    final_cm = getattr(config, "FINAL_APPROACH_CM", config.GRAB_DISTANCE_CM)
    if distance_cm <= final_cm:
        return config.SPEED_FINAL
    if distance_cm <= config.APPROACH_SLOW_CM:
        span = config.APPROACH_SLOW_CM - final_cm
        f = (distance_cm - final_cm) / span if span > 0 else 0.0
        return int(config.SPEED_CREEP + f * (config.SPEED_APPROACH - config.SPEED_CREEP))
    return config.SPEED_APPROACH
