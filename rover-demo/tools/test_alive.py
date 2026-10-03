#!/usr/bin/env python3
"""
Prueba del sondeo de vida del sketch:  python3 tools/test_alive.py

Comprueba que main.py nombra correctamente la etapa donde se colgo el MCU,
que es todo el valor de este diagnostico: sin el, "no compilo", "se colgo el
I2C" y "el sketch no esta cargado" se ven exactamente igual desde el MPU.
"""
import io
import os
import sys
import contextlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "python"))

import test_part1 as t1                       # noqa: E402  (instala los stubs)
from test_part1 import ok, PASSED, FAILED     # noqa: E402


def arranque_con_alive(valor):
    """Reimporta main con el MCU simulado devolviendo ese 'alive'."""
    for mod in ("main", "ui", "vision", "mission", "protocol", "colors", "config", "steer"):
        sys.modules.pop(mod, None)
    import config
    config.CAMERA_ENABLED = False           # sin camara: aqui se prueba el enlace
    config.DIAG_MCU_S = 0

    class Stub(t1.StubBridge):
        def call(self, name, *a):
            if name == "alive":
                if valor is None:
                    raise ValueError("Request 'alive' failed: method alive not available (2)")
                return valor
            return super().call(name, *a)

    t1._utils.Bridge = Stub()
    sys.modules["arduino.app_utils"].Bridge = t1._utils.Bridge
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        import main                          # noqa: F401  (importar ES la prueba)
    return buf.getvalue()


def test_sketch_ausente():
    out = arranque_con_alive(None)
    assert "NO CONTESTA NI AL SONDEO BASICO" in out
    assert "App launch" in out, "debe decir DONDE mirar la compilacion"
    ok("alive sin respuesta -> 'el sketch no esta corriendo' + donde mirar")


def test_colgado_en_i2c():
    out = arranque_con_alive(3)
    assert "ETAPA 3/7" in out and "TCRT" in out
    assert "APDS" in out and "I2C" in out, "debe nombrar lo SIGUIENTE, que es lo que colgo"
    assert "D20/D21" in out, "y donde mirar el cableado"
    ok("alive=3 -> se colgo al inicializar el APDS, con la pista del I2C")


def test_todo_bien():
    out = arranque_con_alive(7)
    assert "etapa 7/7" in out and "NO CONTESTA" not in out
    ok("alive=7 -> sketch vivo, sin ruido de diagnostico")


if __name__ == "__main__":
    print("ROVER H07 - sondeo de vida del sketch - pruebas\n")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
            except AssertionError as e:
                FAILED.append(name)
                print(f"  FALLO  {name}: {e}")
            except Exception as e:             # noqa: BLE001
                FAILED.append(name)
                print(f"  ERROR  {name}: {type(e).__name__}: {e}")
                import traceback
                traceback.print_exc()
    print(f"\n{len(PASSED)} OK, {len(FAILED)} fallidas")
    sys.exit(1 if FAILED else 0)
