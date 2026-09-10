"""
vision.py - Webcam USB: detecta la bandera rival y sirve el video con overlay.

NO DECLARES OpenCV EN UN requirements.txt. La imagen del contenedor ya trae
una compilacion PROPIA de Arduino: opencv-python-headless 4.13.0+1ddb20b. Ese
"+1ddb20b" es una version local (PEP 440) que no existe en PyPI, asi que en
cuanto un requirements.txt menciona opencv, el resolvedor intenta anclarla,
no la encuentra en ningun indice y falla:

    x No solution found when resolving dependencies:
      Because there is no version of opencv-python-headless==4.13.0+1ddb20b...

Lo mismo vale para numpy y para cualquier otra cosa que el paquete
arduino_app_bricks ya arrastre. Se declara SOLO lo que la imagen no trae.

LA CAMARA LA ABRE EL PERIFERICO DE APP LAB (arduino.app_peripherals.camera).
Elige sola la primera webcam USB, la configura por V4L2/OpenCV y entrega
frames numpy en BGR. Eso resuelve dos problemas del UNO Q que la version
anterior tenia que rodear a mano: /dev/video0 y video1 son el decodificador
Venus (abren, pero no dan frames) y el indice de la webcam cambia entre
reinicios. Fuera del contenedor (PC de pruebas) se cae a cv2.VideoCapture.

UN SOLO LECTOR DE LA CAMARA. Este hilo es el UNICO que llama a capture().
El stream /camera de la pagina NO lee la camara: consume el JPEG que este
hilo deja ya dibujado (bbox, centro, cruz, barra de error). Si usaramos
ui.expose_camera() ademas, dos hilos se pelearian por el mismo dispositivo y
cada uno veria la mitad de los frames, sin overlay.

POR QUE NO UNA RED NEURONAL: la bandera es un cilindro liso de 5 x 15 cm en
color plano. Segmentacion HSV + filtro geometrico corre en pocos ms, es
determinista y se recalibra en dos minutos si cambia la luz. El filtro
geometrico es lo que separa la bandera (ALTA y ESTRECHA) de la cinta del
suelo (ANCHA y APLASTADA), que es del mismo color.
"""

import threading
import time

import config

try:
    import cv2
    import numpy as np
    CV_OK = True
    _CV_ERR = None
except ImportError as _e:                          # noqa: N816
    cv2 = None
    np = None
    CV_OK = False
    _CV_ERR = _e

try:
    from arduino.app_peripherals.camera import Camera as _AppCamera
    APP_CAMERA_OK = True
except Exception:                                  # noqa: BLE001
    _AppCamera = None
    APP_CAMERA_OK = False


def distance_from(w_px):
    """Distancia en cm A PARTIR DEL ANCHO, y nada mas.

    POR QUE EL ANCHO Y NO EL ALTO
    -----------------------------
    1. GEOMETRIA. Con el cuarto superior borrado por el horizonte quedan
       360 px utiles de alto. Con la focal medida, la bandera deja de caber a
       partir de ~23 cm: su alto se queda clavado en 360 px y la distancia
       estimada se queda clavada con el. El ancho no se satura hasta ~7 cm.

    2. FISICA, y esta es la razon de fondo. La bandera es un CILINDRO. Su
       diametro son 5 cm SIEMPRE, se mire desde donde se mire: la silueta
       horizontal es una medida limpia. Su "altura" son 15 cm solo si se ven
       el borde de arriba y el de abajo, y en la practica la base queda
       tapada por el propio suelo, por la sombra o por el borde del encuadre.
       El alto es la dimension que MIENTE con mas facilidad.

    EL MODELO NO ES SOLO f*D/w, Y ESO IMPORTA
    -----------------------------------------
        d = FLAG_DIST_A / w_px + FLAG_DIST_B

    El termino B es un DESPLAZAMIENTO CONSTANTE, y existe porque el modelo de
    camara estenopeica mide desde el CENTRO OPTICO de la lente, mientras que
    las distancias utiles (y las que se miden con cinta metrica) se toman
    desde el frente del rover o desde la pinza. Esa diferencia es un numero
    fijo de centimetros, y sin B no hay ninguna focal que cuadre en todo el
    rango:

        con un desfase real de 5 cm, calibrando a 85 cm el error es del 6 %
        (invisible), y a 14 cm es del 36 % (el agarre falla).

    Es exactamente el sintoma de "ajusto la focal y sigue sin cuadrar de
    cerca". Con dos medidas a dos distancias conocidas, A y B salen solos:
    tools/calib_dist.py hace la cuenta.

    Con FLAG_DIST_B = 0 esto se reduce al modelo de siempre, asi que la
    calibracion de un solo punto sigue valiendo como punto de partida.
    """
    if w_px <= 0:
        return None
    a = float(getattr(config, "FLAG_DIST_A",
                      config.FOCAL_PX * config.FLAG_DIAM_CM))
    b = float(getattr(config, "FLAG_DIST_B", 0.0))
    d = a / w_px + b
    return d if d > 0 else None


def score_flag(fill, aspect, area_small, norm_area, clipped):
    """Puntuacion 0..1 y su desglose. MISMA FORMULA QUE EL BANCO DE CAMARA
    (camara-test/python/main.py: puntuar()). Si cambias una, cambia la otra o
    calibrar en el banco deja de significar nada para el rover.

    DOS COSAS QUE LA VERSION ANTERIOR HACIA MAL
    -------------------------------------------
    1. Castigaba el aspecto con  1 / (1 + |aspecto - 3|),  error ABSOLUTO y
       sin forma de aflojarlo. Una bandera real con aspecto 1.40 daba factor
       0.38: la puntuacion caia a 0.36 y se rechazaba con la mascara
       perfecta. Ahora el error es RELATIVO al ideal y FLAG_ASPECT_TOL lo
       modula.

    2. No sabia que la mancha estaba CORTADA por el borde del encuadre. Si la
       bandera no cabe entera -a menos de ~40 cm no cabe- su alto medido esta
       truncado y su aspecto no significa nada. Castigarlo es castigar ruido.
       Con clipped=True el aspecto no puntua en absoluto.
    """
    ideal = config.FLAG_HEIGHT_CM / config.FLAG_DIAM_CM
    rel_err = abs(aspect - ideal) / ideal
    tol = float(getattr(config, "FLAG_ASPECT_TOL", 1.0))
    f_aspect = 1.0 if clipped else 1.0 / (1.0 + rel_err * tol)
    f_area = min(1.0, area_small / norm_area)
    return fill * f_aspect * f_area, (round(fill, 2), round(f_aspect, 2),
                                      round(f_area, 2), clipped)


class Flag:
    """Una deteccion. Coordenadas normalizadas [0,1] y tambien en pixeles,
    porque el visor de la pagina y la calibracion de FOCAL_PX usan pixeles."""

    __slots__ = ("cx", "cy", "w", "h", "x_px", "y_px", "w_px", "h_px",
                 "area", "score", "distance_cm", "dist_raw_cm", "t", "parts",
                 "cut_lft", "cut_rgt")

    def __init__(self, cx, cy, w, h, x_px, y_px, w_px, h_px, area, score,
                 distance_cm, t, parts=None, cut_lft=False, cut_rgt=False):
        self.cx, self.cy, self.w, self.h = cx, cy, w, h
        self.x_px, self.y_px, self.w_px, self.h_px = x_px, y_px, w_px, h_px
        self.area, self.score, self.distance_cm, self.t = area, score, distance_cm, t
        # (relleno, f_aspecto, f_area, recortada): el desglose de la
        # puntuacion. La pagina lo muestra porque "puntuacion baja" a secas no
        # dice cual de los tres factores la hundio.
        self.parts = parts or (0.0, 0.0, 0.0, False)
        # Sin filtrar. process() sustituye distance_cm por la mediana de las
        # ultimas N y deja aqui la lectura cruda, para poder ver el ruido.
        self.dist_raw_cm = distance_cm
        # Bordes laterales que toca: los usa error_x (ver ahi).
        self.cut_lft, self.cut_rgt = cut_lft, cut_rgt

    @property
    def error_x(self):
        """Desviacion horizontal respecto al centro, en [-1, 1]. Negativo =
        a la izquierda.

        CORRECCION POR RECORTE LATERAL. Si la mancha toca el borde izquierdo,
        parte de la bandera esta FUERA del encuadre por ese lado: su centro
        real esta mas a la izquierda que el centro de la caja que se ve. Sin
        corregir, el rover cree estar mas centrado de lo que esta -- y como el
        agarre exige |error| < GRAB_ALIGN_TOL, podria cerrar la pinza con
        media bandera fuera de cuadro. Se toma el error como AL MENOS el del
        borde que toca: prudente en la direccion correcta.
        """
        e = (self.cx - 0.5) * 2.0
        if self.cut_lft and not self.cut_rgt:
            return min(e, -abs(e), -self.w)
        if self.cut_rgt and not self.cut_lft:
            return max(e, abs(e), self.w)
        return e

    @property
    def age(self):
        return time.monotonic() - self.t

    def as_dict(self):
        return {"cx": round(self.cx, 3), "cy": round(self.cy, 3),
                "w": round(self.w, 3), "h": round(self.h, 3),
                "cx_px": int(self.x_px + self.w_px / 2),
                "cy_px": int(self.y_px + self.h_px / 2),
                "w_px": int(self.w_px), "h_px": int(self.h_px),
                "error_x": round(self.error_x, 3),
                "score": round(self.score, 2),
                "fill": self.parts[0], "f_aspect": self.parts[1],
                "f_area": self.parts[2], "clipped": bool(self.parts[3]),
                "dist_raw_cm": (round(self.dist_raw_cm, 1)
                                if self.dist_raw_cm else None),
                "cut_side": ("izq" if self.cut_lft else
                             "der" if self.cut_rgt else None),
                "distance_cm": round(self.distance_cm, 1) if self.distance_cm else None,
                "age_s": round(self.age, 2)}


class Vision:
    def __init__(self, enemy=None):
        self.enemy = enemy or config.ENEMY
        self._ranges = config.HSV_RED if self.enemy == "red" else config.HSV_BLUE
        self._lock = threading.Lock()
        self._flag = None
        self._frames = 0
        self._fps = 0.0
        self._fps_t0 = time.monotonic()
        self._fps_n = 0
        self._proc_ms = 0.0
        self._last_frame_t = 0.0
        self._cam = None
        self._cap = None
        self._running = False
        self.available = False
        self.error = None
        self.backend = None
        self.size = (config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
        self._frame = None                 # ultimo frame crudo, para la foto
        # Ventana para la MEDIANA de la distancia. El ancho de la bandera son
        # 5 cm frente a 15 de alto: es un rasgo TRES VECES MAS PEQUENO, asi
        # que un pixel de error en la mascara pesa el triple. A 100 cm el
        # ancho son ~28 px en el frame completo (14 en el reducido), y ahi
        # +-2 px ya son +-7 % de distancia. La mediana de 3 quita los saltos
        # sueltos sin anadir practicamente retardo (0.3 s a 10 fps).
        self._dist_win = []

        if not getattr(config, "CAMERA_ENABLED", True):
            self.error = "desactivada en config.CAMERA_ENABLED"
            print("[vision] DESACTIVADA por configuracion (CAMERA_ENABLED = False).")
            return

        if not CV_OK:
            self.error = f"OpenCV no disponible: {_CV_ERR}"
            print(f"[vision] {self.error}")
            print("[vision] revisa python/requirements.txt (opencv-python-headless)")
            return
        try:
            self._open()
            self.available = True
            print(f"[vision] camara OK ({self.backend}), buscando bandera "
                  f"{self.enemy.upper()} a {self.size[0]}x{self.size[1]}")
        except Exception as e:                     # noqa: BLE001
            self.error = str(e)
            print(f"[vision] SIN CAMARA: {e}. El rover sigue sin vision.")

    # ------------------------------------------------------------- camara
    def _open(self):
        if APP_CAMERA_OK:
            # Periferico de App Lab: source=None = primera webcam USB libre.
            self._cam = _AppCamera(resolution=self.size, fps=config.CAMERA_FPS,
                                   codec=config.CAMERA_CODEC)
            self._cam.start()
            self.backend = "app_peripherals.camera"
            return
        # Fuera del contenedor (PC): OpenCV directo, probando que de frames.
        for idx in range(0, 6):
            cap = cv2.VideoCapture(idx)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 0:
                self._cap = cap
                self.backend = f"cv2 /dev/video{idx}"
                return
            cap.release()
        raise RuntimeError("ninguna camara utilizable")

    def _grab(self):
        if self._cam is not None:
            return self._cam.capture()             # ya limita a CAMERA_FPS
        ok, frame = self._cap.read()
        if not ok:
            return None
        time.sleep(1.0 / config.CAMERA_FPS)
        return frame

    # --------------------------------------------------------------- hilo
    def start(self):
        if not self.available or self._running:
            return
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="vision").start()

    def stop(self):
        self._running = False
        try:
            if self._cam is not None:
                self._cam.stop()
            if self._cap is not None:
                self._cap.release()
        except Exception:                          # noqa: BLE001
            pass

    def _loop(self):
        fails = 0
        while self._running:
            try:
                frame = self._grab()
            except Exception as e:                 # noqa: BLE001
                fails += 1
                if fails in (1, 10) or fails % 100 == 0:
                    print(f"[vision] error de captura #{fails}: {e}")
                time.sleep(0.2)
                continue
            if frame is None:
                time.sleep(0.02)
                continue
            fails = 0
            self.process(frame)

    def process(self, frame):
        """Un frame -> deteccion. NADA MAS.

        Ni dibujo ni JPEG: sin video en vivo, el camino caliente es solo
        detectar. El frame se guarda por referencia (la camara devuelve un
        array nuevo en cada captura, asi que no hay copia) por si se pide una
        foto con snapshot_jpeg().
        """
        t0 = time.monotonic()
        flag = self.detect(frame)
        self._smooth_distance(flag)
        now = time.monotonic()
        with self._lock:
            self._flag = flag
            self._frame = frame
            self._frames += 1
            self._last_frame_t = now
            self._proc_ms = (now - t0) * 1000.0
            self._fps_n += 1
            if now - self._fps_t0 >= 1.0:
                self._fps = self._fps_n / (now - self._fps_t0)
                self._fps_n, self._fps_t0 = 0, now
        return flag

    def _smooth_distance(self, flag):
        """Mediana de las ultimas N lecturas. MEDIANA y no media: un frame en
        el que la mascara se parte da un ancho absurdo, y una media lo reparte
        entre todas las lecturas mientras que una mediana lo tira."""
        n = int(getattr(config, "FLAG_DIST_MEDIAN_N", 3))
        if flag is None or flag.dist_raw_cm is None or n <= 1:
            self._dist_win.clear()
            return
        # FILTRAR SOLO CUANDO HACE FALTA. De lejos el ancho es diminuto (a
        # 100 cm, ~28 px: +-2 px son +-7 %) y el filtro gana mucho. De cerca
        # es al reves (a 6 cm son 462 px: los mismos +-2 px son +-0.4 %) y su
        # retardo de 0.3 s es puro riesgo, justo en el tramo donde solo hay
        # 1.7 cm de margen antes de que el sensor se sature. Asi que por
        # encima de este ancho se usa la lectura CRUDA.
        if flag.w_px >= float(getattr(config, "FLAG_DIST_TRUST_PX", 1e9)):
            self._dist_win.clear()
            return
        self._dist_win.append(flag.dist_raw_cm)
        if len(self._dist_win) > n:
            self._dist_win.pop(0)
        v = sorted(self._dist_win)
        flag.distance_cm = v[len(v) // 2]

    # ---------------------------------------------------------- deteccion
    def detect(self, frame):
        H0, W0 = frame.shape[:2]
        scale = float(getattr(config, "DETECT_SCALE", 1.0))
        if scale < 0.999:
            small = cv2.resize(frame, (int(W0 * scale), int(H0 * scale)),
                               interpolation=cv2.INTER_AREA)
        else:
            small, scale = frame, 1.0
        inv = 1.0 / scale
        h_img, w_img = small.shape[:2]
        hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = None
        for lo, hi in self._ranges:
            m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        # La bandera mide 15 cm: mirando al frente nunca esta en el cuarto
        # superior. Eso descarta techo, focos y publico.
        mask[:int(h_img * config.HORIZON_FRAC), :] = 0

        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        # Los umbrales de AREA estan definidos en pixeles del frame completo:
        # en el reducido escalan con scale^2.
        min_area = config.FLAG_MIN_AREA_PX * scale * scale
        norm_area = float(getattr(config, "FLAG_NORM_AREA_PX", 4000.0)) * scale * scale
        horizon_row = int(h_img * config.HORIZON_FRAC)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h == 0:
                continue
            aspect = h / float(w)
            fill = area / float(w * h)
            width_frac = w / float(w_img)
            # RECORTADA: la mancha toca un borde del encuadre (la linea del
            # horizonte cuenta como borde: por encima se borro la mascara).
            # Su alto real no cabe en la imagen, asi que su aspecto es una
            # medida falsa y no se puede puntuar con el.
            # Que borde toca importa, no solo SI toca: el alto se estropea
            # con los bordes de arriba/abajo y el ancho con los de los lados.
            cut_top = y <= horizon_row + 1
            cut_bot = y + h >= h_img - 2
            cut_lft = x <= 1
            cut_rgt = x + w >= w_img - 2
            clipped = cut_top or cut_bot or cut_lft or cut_rgt
            # El alto ya no se usa para medir; cut_top/cut_bot solo cuentan
            # para saber si la mancha esta recortada (puntuacion).
            # --- los tres filtros que descartan la cinta del suelo ---
            if not (config.FLAG_MIN_ASPECT <= aspect <= config.FLAG_MAX_ASPECT):
                continue
            if fill < config.FLAG_MIN_FILL:
                continue
            if width_frac > config.FLAG_MAX_WIDTH_FRAC:
                continue
            score, parts = score_flag(fill, aspect, area, norm_area, clipped)
            if score < config.MIN_CONFIDENCE:
                continue
            if best is None or score > best[0]:
                best = (score, x, y, w, h, area, parts, cut_lft, cut_rgt)

        if best is None:
            return None
        score, x, y, w, h, area, parts, cut_lft, cut_rgt = best
        # De vuelta a pixeles del frame completo: FOCAL_PX y el visor hablan
        # en esa escala, y asi DETECT_SCALE se puede cambiar sin recalibrar.
        x, y, w, h = x * inv, y * inv, w * inv, h * inv
        area *= inv * inv
        distance = distance_from(w)
        return Flag(cx=(x + w / 2.0) / W0, cy=(y + h / 2.0) / H0,
                    w=w / float(W0), h=h / float(H0),
                    x_px=x, y_px=y, w_px=w, h_px=h,
                    area=area, score=score, distance_cm=distance,
                    t=time.monotonic(), parts=parts,
                    cut_lft=cut_lft, cut_rgt=cut_rgt)

    # ------------------------------------------------------------ overlay
    def _draw(self, frame, flag):
        h, w = frame.shape[:2]
        cx0, cy0 = w // 2, h // 2
        # cruz central y banda de zona muerta
        cv2.line(frame, (cx0, 0), (cx0, h), (90, 90, 90), 1)
        cv2.line(frame, (0, cy0), (w, cy0), (90, 90, 90), 1)
        dz = int(config.STEER_DEADBAND * w / 2)
        cv2.rectangle(frame, (cx0 - dz, h - 14), (cx0 + dz, h - 4), (70, 120, 70), -1)
        # horizonte ignorado
        cv2.line(frame, (0, int(h * config.HORIZON_FRAC)),
                 (w, int(h * config.HORIZON_FRAC)), (60, 60, 60), 1)
        if flag is None:
            cv2.putText(frame, f"buscando {self.enemy}", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
            return
        x, y, bw, bh = int(flag.x_px), int(flag.y_px), int(flag.w_px), int(flag.h_px)
        fx, fy = x + bw // 2, y + bh // 2
        centered = abs(flag.error_x) < config.STEER_DEADBAND
        col = (0, 220, 0) if centered else (0, 200, 255)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), col, 2)
        cv2.circle(frame, (fx, fy), 5, col, -1)
        cv2.line(frame, (cx0, fy), (fx, fy), col, 1)        # error horizontal
        # marcador en la barra inferior
        mx = int(cx0 + flag.error_x * w / 2)
        cv2.rectangle(frame, (mx - 4, h - 16), (mx + 4, h - 2), col, -1)
        d = f"{flag.distance_cm:.0f} cm" if flag.distance_cm else "?"
        if flag.parts[3]:               # recortada por el borde
            cv2.putText(frame, "RECORTADA", (w - 110, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
        cv2.putText(frame, f"err {flag.error_x:+.2f}  {d}  w {bw}px  s {flag.score:.2f}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1)

    # ------------------------------------------------------------ lectura
    def latest_flag(self):
        with self._lock:
            f = self._flag
        if f is None or f.age > config.DETECTION_STALE_S:
            return None
        return f

    def snapshot_jpeg(self):
        """Una foto del ultimo frame con el overlay dibujado. BAJO DEMANDA:
        se dibuja y se codifica aqui, no en el lazo de deteccion. Sirve para
        calibrar los rangos HSV viendo si recuadra la bandera correcta."""
        with self._lock:
            frame, flag = self._frame, self._flag
        if frame is None:
            return None
        img = frame.copy()          # copia: el lazo puede estar usando el original
        self._draw(img, flag)
        ok, enc = cv2.imencode(".jpg", img,
                               [int(cv2.IMWRITE_JPEG_QUALITY), config.SNAPSHOT_JPEG_QUALITY])
        return enc.tobytes() if ok else None

    def stats(self):
        with self._lock:
            return {"available": self.available, "backend": self.backend,
                    "enemy": self.enemy, "frames": self._frames,
                    "fps": round(self._fps, 1),
                    "proc_ms": round(self._proc_ms, 1),
                    "detect_scale": getattr(config, "DETECT_SCALE", 1.0),
                    "width": self.size[0], "height": self.size[1],
                    "last_frame_age_s": (round(time.monotonic() - self._last_frame_t, 2)
                                         if self._last_frame_t else None),
                    "error": self.error}
