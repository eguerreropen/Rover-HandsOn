"""
colors.py - Clasificacion del color de zona a partir de r,g,b,c del APDS.

Se hace en el MPU a proposito: calibrar es editar config.py y reiniciar la
App, sin recompilar el sketch.

METODO: proporciones, no valores absolutos. Si sube la luz suben los cuatro
canales a la vez y las fracciones r/(r+g+b) etc. se mantienen. El "clear"
solo decide los extremos (negro / blanco).

ORDEN DE LAS REGLAS (importa):
  1. clear bajo            -> negro
  2. canales equilibrados  -> blanco si clear alto, si no desconocido
  3. amarillo              -> ANTES que rojo: el amarillo tambien tiene el
                              canal rojo alto; lo distingue el azul hundido
  4. rojo / azul           -> canal dominante por encima de COLOR_DOMINANT
"""

import config


def fractions(r, g, b):
    total = float(r + g + b)
    if total < 1.0:
        return None
    return r / total, g / total, b / total


def classify(r, g, b, c):
    if c < config.COLOR_DARK_CLEAR:
        return config.C_BLACK
    f = fractions(r, g, b)
    if f is None:
        return config.C_UNKNOWN
    fr, fg, fb = f
    mx, mn = max(f), min(f)

    if (mx - mn) < config.COLOR_NEUTRAL_SPREAD:
        return config.C_WHITE if c > config.COLOR_BRIGHT_CLEAR else config.C_UNKNOWN

    if (fb < config.YELLOW_MAX_BLUE and fr > config.YELLOW_MIN_RED
            and fg > config.YELLOW_MIN_GREEN):
        return config.C_YELLOW
    if fr == mx and fr > config.COLOR_DOMINANT:
        return config.C_RED
    if fb == mx and fb > config.COLOR_DOMINANT:
        return config.C_BLUE
    return config.C_UNKNOWN


class Stable:
    """
    Filtro de estabilidad: una clase pasa a ser "la actual" cuando se repite
    COLOR_STABLE_N lecturas seguidas. Devuelve (clase_estable, cambio).
    """

    def __init__(self, n=None):
        self.n = n or config.COLOR_STABLE_N
        self.current = config.C_UNKNOWN
        self._cand = config.C_UNKNOWN
        self._count = 0

    def update(self, color):
        if color == self.current:
            self._cand, self._count = color, 0
            return self.current, False
        if color == self._cand:
            self._count += 1
        else:
            self._cand, self._count = color, 1
        if self._count >= self.n:
            self.current = color
            self._count = 0
            return self.current, True
        return self.current, False


class ColorTracker:
    """Aplica clasificacion + estabilidad al Sense de cada ciclo y lleva el
    registro de cambios (lo que la pagina muestra como 'eventos')."""

    def __init__(self):
        self.stable = Stable()
        self.events = []                    # [(t_s, nombre)] ultimos cambios
        self._t0 = None

    def apply_to(self, sense, now):
        if self._t0 is None:
            self._t0 = now
        if not sense.floor_sensor_ok or sense.color_disabled:
            sense.color = sense.color_stable = config.C_UNKNOWN
            return False
        sense.color = classify(sense.r, sense.g, sense.b, sense.c)
        sense.color_stable, changed = self.stable.update(sense.color)
        if changed:
            name = config.COLOR_NAMES.get(sense.color_stable, "?")
            self.events.append((round(now - self._t0, 1), name))
            del self.events[:-30]
            print(f"[color] zona bajo el rover: {name}   "
                  f"(r={sense.r} g={sense.g} b={sense.b} c={sense.c})")
        return changed

    def calib_dict(self, sense):
        f = fractions(sense.r, sense.g, sense.b)
        return {
            "r": sense.r, "g": sense.g, "b": sense.b, "c": sense.c,
            "fr": round(f[0], 3) if f else None,
            "fg": round(f[1], 3) if f else None,
            "fb": round(f[2], 3) if f else None,
            "clase": config.COLOR_NAMES.get(sense.color, "?"),
            "estable": config.COLOR_NAMES.get(sense.color_stable, "?"),
        }
