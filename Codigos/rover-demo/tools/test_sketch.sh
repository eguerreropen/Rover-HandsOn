#!/bin/sh
# Comprueba el codigo del MCU en un PC: compila el sketch entero y EJECUTA
# las pruebas de actuators/motors/signals.
#
# Los stubs emulan la semantica de zephyr (ver tools/stubs/). No sustituyen a
# compilar en App Lab, pero cazan justo lo que a App Lab se le escapa: codigo
# que compila y luego tumba el MCU en tiempo de ejecucion.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
STUBS="$HERE/stubs"
SKETCH="$HERE/../sketch"

echo "== sintaxis del sketch completo =="
g++ -std=gnu++17 -fsyntax-only -Wall -Wextra -I"$STUBS" -I"$SKETCH" "$STUBS/sketch_main.cpp"
echo "   OK"
echo
echo "== pruebas ejecutables del MCU =="
g++ -std=gnu++17 -Wall -Wextra -I"$STUBS" -I"$SKETCH" "$HERE/test_actuators.cpp" -o /tmp/test_actuators
/tmp/test_actuators
