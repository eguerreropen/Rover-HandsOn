#!/usr/bin/env python3
"""
Calibra la distancia por ancho con DOS medidas.

    python3 tools/calib_dist.py  <d1_cm> <w1_px>  <d2_cm> <w2_px>  [mas pares...]

Ejemplo:
    python3 tools/calib_dist.py 80 35 20 139

POR QUE DOS Y NO UNA
--------------------
Con una sola medida solo se puede despejar la focal, y eso da el modelo

    d = A / w_px

que asume que la distancia se mide desde el CENTRO OPTICO de la lente. Pero
las distancias que importan (y las que se miden con cinta metrica) se toman
desde el frente del rover o desde la pinza. Entre un punto y otro hay un
numero fijo de centimetros, y ese desfase NO se puede absorber cambiando A:

    con un desfase real de 5 cm, calibrando a 85 cm el error es del 6 %
    (invisible) y a 14 cm es del 36 % (el agarre falla)

Ese es exactamente el sintoma de "ajusto la focal y sigue sin cuadrar de
cerca". El modelo con dos terminos

    d = A / w_px + B

lo absorbe en B, y con dos medidas a dos distancias distintas A y B salen de
un sistema de dos ecuaciones. Con tres o mas pares se ajusta por minimos
cuadrados sobre x = 1/w_px, que ademas da el error residual: si el residuo es
grande, el problema no es la calibracion.

COMO TOMAR LAS MEDIDAS
----------------------
- Una LEJOS (~80 cm) y otra CERCA (~20 cm). Cuanto mas separadas, mejor sale
  B. Dos medidas parecidas dan un B basura.
- Mide siempre desde el MISMO punto del rover. Lo mas util es la PINZA, que es
  lo que tiene que llegar a la bandera: asi GRAB_DISTANCE_CM se lee directo.
- El "ancho w_px" lo da la pagina, en el visor de puntería.
- Que la bandera este centrada y NO diga "recortada de lado": si toca un
  borde lateral, el ancho esta truncado y la medida no vale.
"""
import sys


def ajusta(pares):
    """Minimos cuadrados de d = A*x + B con x = 1/w_px."""
    n = len(pares)
    xs = [1.0 / w for _, w in pares]
    ds = [d for d, _ in pares]
    sx, sd = sum(xs), sum(ds)
    sxx = sum(x * x for x in xs)
    sxd = sum(x * d for x, d in zip(xs, ds))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        raise ValueError("las medidas estan a la misma distancia: separalas mas")
    a = (n * sxd - sx * sd) / den
    b = (sd - a * sx) / n
    return a, b


def main(argv):
    if len(argv) < 4 or len(argv) % 2:
        print(__doc__)
        return 1
    nums = [float(v) for v in argv]
    pares = list(zip(nums[0::2], nums[1::2]))
    for d, w in pares:
        if d <= 0 or w <= 0:
            print(f"medida invalida: {d} cm, {w} px")
            return 1

    a, b = ajusta(pares)

    print("\nMedidas:")
    for d, w in pares:
        est = a / w + b
        print(f"  {d:6.1f} cm  ->  {w:6.1f} px   (el modelo da {est:6.1f}, "
              f"error {est - d:+.2f} cm)")

    resid = max(abs(a / w + b - d) for d, w in pares)
    print(f"\n  FLAG_DIST_A    = {a:.1f}")
    print(f"  FLAG_DIST_B    = {b:.2f}")
    print(f"\nPega esas dos lineas en python/config.py.")

    # Un solo punto habria dado esto: sirve para ver cuanto importaba B.
    d0, w0 = pares[0]
    a_solo = d0 * w0
    peor = max(pares, key=lambda p: abs(a_solo / p[1] - p[0]))
    print(f"\nCon UN SOLO punto ({d0:.0f} cm) habria salido A = {a_solo:.0f}, "
          f"B = 0, y a {peor[0]:.0f} cm ese modelo daria "
          f"{a_solo / peor[1]:.1f} cm ({a_solo / peor[1] - peor[0]:+.1f}).")

    if abs(b) < 1.0:
        print("B es casi cero: el desfase no era el problema. Si la distancia "
              "sigue sin cuadrar, mira el residuo y la mascara.")
    if resid > 2.0:
        print(f"\nAVISO: residuo de {resid:.1f} cm. El modelo 1/w_px NO esta "
              f"describiendo bien tus medidas. Antes de tocar nada mas, "
              f"comprueba que el ancho que lees es el de la BANDERA y no el "
              f"de un reflejo o una sombra pegada (mira la mascara en el "
              f"banco), y que ninguna medida estaba recortada de lado.")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
