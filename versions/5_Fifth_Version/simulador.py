#!/usr/bin/env python3
"""
Simulador de navegación sobre mapas con Niebla de Guerra semi-transparente,
Replanning en tiempo real (A*), modo de dibujo de obstáculos con ratón y
soporte para múltiples mapas (600x800 original y Mapa_Ex02_NOrobots_OK.bmp de GitHub).

- Mapas soportados:
  1) Mapa original procedural: 600 px x 800 px
  2) Mapa BMP (Reto 05): Mapa_Ex02_NOrobots_OK.bmp (150x100 escalado a 600x400 px)
- Modos:
  * 🖌️ Modo Pintar con ratón: Haz clic y arrastra para dibujar paredes/obstáculos.
  * 🗺️ Cambiar de mapa: Alterna en un clic entre el mapa 600x800 y el mapa BMP.
- Niebla de guerra: Gris clara y semi-transparente (siluetas tenues de obstáculos visibles).
- Trayectoria: Naranja vibrante visible durante todo el trayecto.
- Punto inicial: Pequeñito y sin color.
- Punto final: Rojo inicialmente; cambia a verde al llegar el punto azul.
- Campo de visión: Radio de 15 px (descubre la niebla y obstáculos sobre la marcha).
"""

import heapq
import math
import os
import random
import sys
import urllib.request

try:
    import tkinter as tk
except ImportError:
    print("\n" + "=" * 65)
    print("ERROR: El módulo 'tkinter' no está instalado en este entorno Python.")
    print("Para instalarlo en Ubuntu/Debian, ejecuta en tu terminal:")
    print("    sudo apt update && sudo apt install python3-tk")
    print("=" * 65 + "\n")
    sys.exit(1)

try:
    from PIL import Image
    PIL_DISPONIBLE = True
except ImportError:
    PIL_DISPONIBLE = False


# ==========================================
# CONFIGURACIÓN GENERAL Y COLORES
# ==========================================
RADIO_INICIAL = 2.5    # Pequeñito y sin color
RADIO_FINAL = 5.0      # Punto final
RADIO_PUNTO = 4.0      # Radio del punto azul en píxeles
RADIO_VISION = 15.0    # Radio del campo de visión (15 px)
MARGEN_SEGURIDAD = 6.0 # Margen alrededor de los obstáculos para evitar colisiones
TAM_CELDA = 6          # Resolución interna de celda para A* (px)

# Colores de la interfaz
COLOR_NIEBLA = "#cbd5e1"         # Gris claro suave (Tailwind Slate-300)
COLOR_DESPEJADO = "#ffffff"      # Blanco puro para el terreno descubierto
COLOR_OBS_TRANSLUCIDO = "#94a3b8" # Silueta tenue visible bajo la niebla semi-transparente
COLOR_BORDE_TRANSLUCIDO = "#94a3b8"
COLOR_OBSTACULO = "#334155"      # Gris oscuro sólido para obstáculos descubiertos
COLOR_BORDE_OBS = "#1e293b"

# Trayectoria (Naranja vibrante)
COLOR_RASTRO = "#ea580c"         # Naranja intenso para la trayectoria recorrida
COLOR_CAMINO_PLAN = "#fb923c"    # Naranja claro discontinuo para la ruta futura

# Punto Final
COLOR_FIN_PENDIENTE = "#ef4444"  # Rojo antes de llegar
COLOR_FIN_BORDE_PEND = "#b91c1c"
COLOR_FIN_ALCANZADO = "#22c55e"  # Verde al ser alcanzado
COLOR_FIN_BORDE_ALC = "#15803d"

URL_GITHUB_BMP = (
    "https://raw.githubusercontent.com/mariaarroniz/"
    "-26-27-IYA051---Inteligencia-Artificial/main/Reto%2005/Mapa_Ex02_NOrobots_OK.bmp"
)


# ==========================================
# FUNCIONES AUXILIARES DE GEOMETRÍA Y A*
# ==========================================
def distancia_euclidea(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def distancia_punto_rectangulo(px: float, py: float, rect: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = rect
    cx = max(x1, min(px, x2))
    cy = max(y1, min(py, y2))
    return math.hypot(px - cx, py - cy)


def obstaculo_bloquea_trayectoria(
    puntos_restantes: list[tuple[float, float]],
    rect: tuple[int, int, int, int],
    holgura: float = MARGEN_SEGURIDAD + RADIO_PUNTO
) -> bool:
    x1, y1, x2, y2 = rect
    min_x, max_x = x1 - holgura, x2 + holgura
    min_y, max_y = y1 - holgura, y2 + holgura
    for px, py in puntos_restantes:
        if min_x <= px <= max_x and min_y <= py <= max_y:
            return True
    return False


def buscar_camino_a_star(
    inicio: tuple[float, float],
    fin: tuple[float, float],
    celdas_bloqueadas: set[tuple[int, int]],
    cols: int,
    rows: int
) -> list[tuple[float, float]] | None:
    if not celdas_bloqueadas:
        return [inicio, fin]

    gx0 = int(round(inicio[0] / TAM_CELDA))
    gy0 = int(round(inicio[1] / TAM_CELDA))
    gx1 = int(round(fin[0] / TAM_CELDA))
    gy1 = int(round(fin[1] / TAM_CELDA))

    # Asegurar que el punto de partida y fin no se bloqueen a sí mismos
    bloqueadas_temp = celdas_bloqueadas.copy()
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            bloqueadas_temp.discard((gx0 + dx, gy0 + dy))
            bloqueadas_temp.discard((gx1 + dx, gy1 + dy))

    start = (gx0, gy0)
    goal = (gx1, gy1)

    movimientos = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (-1, -1, 1.4142)
    ]

    pq = [(distancia_euclidea(start, goal), 0, 0.0, start)]
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    contador = 0

    while pq:
        _, _, g, actual = heapq.heappop(pq)

        if actual == goal:
            path = []
            curr = goal
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            puntos_px = [(gx * TAM_CELDA, gy * TAM_CELDA) for gx, gy in path]
            puntos_px[0] = inicio
            puntos_px[-1] = fin
            return puntos_px

        if g > g_score.get(actual, float("inf")):
            continue

        cx, cy = actual
        for dx, dy, coste in movimientos:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < cols and 0 <= ny < rows:
                if (nx, ny) in bloqueadas_temp:
                    continue
                if dx != 0 and dy != 0:
                    if (cx + dx, cy) in bloqueadas_temp and (cx, cy + dy) in bloqueadas_temp:
                        continue

                nuevo_g = g + coste
                if nuevo_g < g_score.get((nx, ny), float("inf")):
                    g_score[(nx, ny)] = nuevo_g
                    came_from[(nx, ny)] = actual
                    contador += 1
                    h = distancia_euclidea((nx, ny), goal)
                    heapq.heappush(pq, (nuevo_g + h, contador, nuevo_g, (nx, ny)))

    return None


def linea_libre(
    p1: tuple[float, float],
    p2: tuple[float, float],
    celdas_bloqueadas: set[tuple[int, int]]
) -> bool:
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    pasos = max(1, int(dist / (TAM_CELDA / 2)))
    for i in range(pasos + 1):
        t = i / pasos
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        gx = int(round(x / TAM_CELDA))
        gy = int(round(y / TAM_CELDA))
        if (gx, gy) in celdas_bloqueadas:
            return False
    return True


def suavizar_camino(
    camino: list[tuple[float, float]],
    celdas_bloqueadas: set[tuple[int, int]]
) -> list[tuple[float, float]]:
    if not camino or len(camino) <= 2:
        return camino

    suavizado = [camino[0]]
    idx_actual = 0
    while idx_actual < len(camino) - 1:
        siguiente_idx = len(camino) - 1
        while siguiente_idx > idx_actual + 1:
            if linea_libre(camino[idx_actual], camino[siguiente_idx], celdas_bloqueadas):
                break
            siguiente_idx -= 1
        suavizado.append(camino[siguiente_idx])
        idx_actual = siguiente_idx

    return suavizado


def interpolar_puntos(waypoints: list[tuple[float, float]], paso_px: float = 2.0) -> list[tuple[float, float]]:
    if not waypoints:
        return []
    puntos = []
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        num_pasos = max(1, int(dist / paso_px))
        for step in range(num_pasos):
            t = step / num_pasos
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            puntos.append((x, y))
    puntos.append(waypoints[-1])
    return puntos


# ==========================================
# INTERFAZ GRÁFICA Y CONTROLADOR
# ==========================================
class SimuladorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Simulador de Navegación A* con Niebla y Replanning")
        self.root.resizable(False, False)

        # Modo de mapa activo: "ORIGINAL" (600x800) o "BMP" (600x400)
        self.tipo_mapa = "ORIGINAL"
        self.ancho_mapa = 600
        self.alto_mapa = 800
        self.punto_inicial = (50.0, 50.0)
        self.punto_final = (550.0, 750.0)

        # Modo de pintar obstáculos con ratón
        self.modo_pintar = False
        self.ultimo_punto_pintado = (0, 0)

        # Estado del mundo
        self.obstaculos_reales: list[tuple[int, int, int, int]] = []
        self.obstaculos_descubiertos: set[tuple[int, int, int, int]] = set()
        self.celdas_bloqueadas_conocidas: set[tuple[int, int]] = set()
        self.celdas_bloqueadas_totales: set[tuple[int, int]] = set()

        # Opciones
        self.var_niebla = tk.BooleanVar(value=True)

        # Estado del agente
        self.meta_alcanzada = False
        self.pos_actual = self.punto_inicial
        self.historial_recorrido: list[tuple[float, float]] = [self.punto_inicial]
        self.animando = False
        self.puntos_trayectoria: list[tuple[float, float]] = []
        self.indice_trayectoria = 0
        self.delay_ms = 20
        self.velocidad_paso = 2.0

        # Almacenar obstáculos por mapa
        self.cache_obs_original: list[tuple[int, int, int, int]] = []
        self.cache_obs_bmp: list[tuple[int, int, int, int]] = []

        self._crear_interfaz()
        self._inicializar_mapa_original()

    def _crear_interfaz(self):
        contenedor = tk.Frame(self.root, padx=8, pady=8)
        contenedor.pack()

        # Barra superior de controles
        barra = tk.Frame(contenedor)
        barra.pack(fill="x", pady=(0, 6))

        # Fila 1: Dimensiones y selector de niebla
        f1 = tk.Frame(barra)
        f1.pack(fill="x", pady=(0, 4))

        self.lbl_info = tk.Label(
            f1,
            text=f"Mapa: {self.ancho_mapa}x{self.alto_mapa} px  |  Visión: {int(RADIO_VISION)} px",
            font=("Arial", 10, "bold"),
            fg="#1e293b"
        )
        self.lbl_info.pack(side="left")

        chk_niebla = tk.Checkbutton(
            f1,
            text="🌫️ Niebla de guerra",
            variable=self.var_niebla,
            command=self._on_toggle_niebla,
            font=("Arial", 9, "bold"),
            fg="#0f172a"
        )
        chk_niebla.pack(side="right")

        # Fila 2: Botones de manipulación de mapa y dibujo
        f2 = tk.Frame(barra)
        f2.pack(fill="x", pady=(0, 4))

        self.btn_cambiar_mapa = tk.Button(
            f2,
            text="🗺️ Cambiar a Mapa BMP (Reto 05)",
            command=self.alternar_mapa,
            bg="#e0e7ff",
            fg="#3730a3",
            font=("Arial", 9, "bold"),
            cursor="hand2"
        )
        self.btn_cambiar_mapa.pack(side="left", padx=(0, 4))

        self.btn_pintar = tk.Button(
            f2,
            text="🖌️ Pintar con ratón",
            command=self.alternar_modo_pintar,
            bg="#f1f5f9",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_pintar.pack(side="left", padx=(0, 4))

        self.btn_generar_obs = tk.Button(
            f2,
            text="🎲 Obstáculos aleatorios",
            command=self.generar_obstaculos_aleatorios,
            bg="#f1f5f9",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_generar_obs.pack(side="left", padx=(0, 4))

        self.btn_borrar_obs = tk.Button(
            f2,
            text="🗑️ Borrar",
            command=self.borrar_obstaculos,
            bg="#f1f5f9",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_borrar_obs.pack(side="right")

        # Fila 3: Botones de simulación
        f3 = tk.Frame(barra)
        f3.pack(fill="x", pady=(0, 4))

        self.btn_iniciar = tk.Button(
            f3,
            text="▶ Iniciar recorrido",
            command=self.iniciar_movimiento,
            bg="#2563eb",
            fg="white",
            font=("Arial", 9, "bold"),
            relief="raised",
            cursor="hand2"
        )
        self.btn_iniciar.pack(side="left", fill="x", expand=True, padx=(0, 4))

        self.btn_reiniciar = tk.Button(
            f3,
            text="↺ Reiniciar",
            command=self.reiniciar_todo,
            bg="#f1f5f9",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_reiniciar.pack(side="right")

        # Fila 4: Mensaje de estado
        self.lbl_estado = tk.Label(
            barra,
            text="Listo. Pulsa '▶ Iniciar recorrido' o '🖌️ Pintar con ratón' para dibujar.",
            font=("Arial", 9),
            fg="#334155"
        )
        self.lbl_estado.pack(fill="x", pady=(2, 0))

        # Lienzo (Canvas)
        self.canvas = tk.Canvas(
            contenedor,
            width=self.ancho_mapa,
            height=self.alto_mapa,
            bg=COLOR_NIEBLA,
            highlightthickness=1,
            highlightbackground="#94a3b8"
        )
        self.canvas.pack()

        # Eventos del ratón para pintar/borrar
        self.canvas.bind("<Button-1>", self._on_canvas_press)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<Button-3>", self._on_canvas_right_click)
        self.canvas.bind("<B3-Motion>", self._on_canvas_right_click)

    def _cols_rows(self) -> tuple[int, int]:
        return self.ancho_mapa // TAM_CELDA, self.alto_mapa // TAM_CELDA

    def _habilitar_botones(self, habilitar: bool):
        estado = "normal" if habilitar else "disabled"
        self.btn_iniciar.config(state=estado)
        self.btn_cambiar_mapa.config(state=estado)
        self.btn_pintar.config(state=estado)
        self.btn_generar_obs.config(state=estado)
        self.btn_borrar_obs.config(state=estado)
        self.btn_reiniciar.config(state=estado)

    # ==========================================
    # GESTIÓN Y CAMBIO DE MAPAS
    # ==========================================
    def _inicializar_mapa_original(self):
        """Inicializa el mapa original de 600x800 px."""
        self.tipo_mapa = "ORIGINAL"
        self.ancho_mapa = 600
        self.alto_mapa = 800
        self.punto_inicial = (50.0, 50.0)
        self.punto_final = (550.0, 750.0)
        self.canvas.config(width=self.ancho_mapa, height=self.alto_mapa)
        self.btn_cambiar_mapa.config(text="🗺️ Cambiar a Mapa BMP (Reto 05)")
        self.btn_generar_obs.config(state="normal")
        self.lbl_info.config(text=f"Mapa: {self.ancho_mapa}x{self.alto_mapa} px (Original)  |  Visión: {int(RADIO_VISION)} px")

        if not self.cache_obs_original:
            self.generar_obstaculos_aleatorios()
        else:
            self.obstaculos_reales = list(self.cache_obs_original)
            self._reconstruir_celdas_bloqueadas()
            self.reiniciar_todo()

    def _obtener_ruta_bmp(self) -> str:
        """Localiza o descarga el archivo Mapa_Ex02_NOrobots_OK.bmp."""
        directorio = os.path.dirname(os.path.abspath(__file__))
        ruta_local = os.path.join(directorio, "Mapa_Ex02_NOrobots_OK.bmp")
        if not os.path.exists(ruta_local):
            try:
                self.lbl_estado.config(text="Descargando Mapa_Ex02_NOrobots_OK.bmp desde GitHub...")
                self.root.update_idletasks()
                urllib.request.urlretrieve(URL_GITHUB_BMP, ruta_local)
            except Exception as e:
                print("Error al descargar mapa BMP:", e)
        return ruta_local

    def _cargar_obstaculos_bmp(self) -> list[tuple[int, int, int, int]]:
        """Carga el BMP y combina sus píxeles negros en rectángulos 2D optimizados."""
        ruta_bmp = self._obtener_ruta_bmp()
        if not os.path.exists(ruta_bmp) or not PIL_DISPONIBLE:
            return []

        im = Image.open(ruta_bmp)
        w, h = im.size
        escala = 4  # 150x100 -> 600x400

        black_pixels = {(x, y) for y in range(h) for x in range(w) if im.getpixel((x, y)) == (0, 0, 0)}

        # Runs horizontales
        rows_runs = []
        for y in range(h):
            runs = []
            in_run = False
            rx0 = 0
            for x in range(w):
                if (x, y) in black_pixels:
                    if not in_run:
                        in_run = True
                        rx0 = x
                else:
                    if in_run:
                        runs.append((rx0, x))
                        in_run = False
            if in_run:
                runs.append((rx0, w))
            rows_runs.append(runs)

        # Fusión vertical de rectángulos
        active = []
        finished = []
        for y, row in enumerate(rows_runs):
            new_active = []
            matched = set()
            for rect in active:
                x0, x1, y0, _ = rect
                if (x0, x1) in row:
                    new_active.append((x0, x1, y0, y + 1))
                    matched.add((x0, x1))
                else:
                    finished.append(rect)
            for run in row:
                if run not in matched:
                    new_active.append((run[0], run[1], y, y + 1))
            active = new_active
        finished.extend(active)

        return [(x0 * escala, y0 * escala, x1 * escala, y1 * escala) for x0, x1, y0, y1 in finished]

    def alternar_mapa(self):
        """Alterna entre el mapa 600x800 original y el mapa BMP de GitHub."""
        if self.animando:
            return

        if self.tipo_mapa == "ORIGINAL":
            # Guardar obstáculos actuales del mapa original
            self.cache_obs_original = list(self.obstaculos_reales)

            # Cargar mapa BMP (150x100 -> 600x400)
            if not self.cache_obs_bmp:
                self.cache_obs_bmp = self._cargar_obstaculos_bmp()

            if not self.cache_obs_bmp:
                self.lbl_estado.config(text="⚠️ No se pudo cargar el archivo Mapa_Ex02_NOrobots_OK.bmp.")
                return

            self.tipo_mapa = "BMP"
            self.ancho_mapa = 600
            self.alto_mapa = 400
            # Puntos dentro del componente principal navegable del mapa BMP
            self.punto_inicial = (24.0, 92.0)
            self.punto_final = (584.0, 296.0)

            self.canvas.config(width=self.ancho_mapa, height=self.alto_mapa)
            self.btn_cambiar_mapa.config(text="🗺️ Volver a Mapa 600x800")
            self.btn_generar_obs.config(state="disabled") # En el mapa BMP los muros son fijos
            self.lbl_info.config(text=f"Mapa: {self.ancho_mapa}x{self.alto_mapa} px (BMP Ex02)  |  Visión: {int(RADIO_VISION)} px")

            self.obstaculos_reales = list(self.cache_obs_bmp)
            self._reconstruir_celdas_bloqueadas()
            self.reiniciar_todo()
            self.lbl_estado.config(text="Cargado Mapa_Ex02_NOrobots_OK.bmp (600x400). Pulsa '▶ Iniciar recorrido'.")
        else:
            self._inicializar_mapa_original()
            self.lbl_estado.config(text="Restaurado mapa original de 600x800 px.")

    # ==========================================
    # MODO PINTAR CON EL RATÓN
    # ==========================================
    def alternar_modo_pintar(self):
        """Activa o desactiva la brocha para dibujar obstáculos con el ratón."""
        if self.animando:
            return

        self.modo_pintar = not self.modo_pintar
        if self.modo_pintar:
            self.btn_pintar.config(text="🖌️ Modo Pintar: ACTIVO", bg="#fef08a", fg="#854d0e")
            self.canvas.config(cursor="crosshair")
            self.lbl_estado.config(
                text="🖌️ Modo Pintar: Arrastra el botón izquierdo para crear obstáculos, o botón derecho para borrarlos."
            )
        else:
            self.btn_pintar.config(text="🖌️ Pintar con ratón", bg="#f1f5f9", fg="black")
            self.canvas.config(cursor="")
            self.lbl_estado.config(text="Modo Pintar desactivado.")

    def _on_canvas_press(self, event):
        if self.animando or not self.modo_pintar:
            return
        self.ultimo_punto_pintado = (event.x, event.y)
        self._estampar_obstaculo(event.x, event.y)

    def _on_canvas_drag(self, event):
        if self.animando or not self.modo_pintar:
            return
        # Espaciar los trazos para no saturar
        dx = event.x - self.ultimo_punto_pintado[0]
        dy = event.y - self.ultimo_punto_pintado[1]
        if math.hypot(dx, dy) >= 10:
            self.ultimo_punto_pintado = (event.x, event.y)
            self._estampar_obstaculo(event.x, event.y)

    def _estampar_obstaculo(self, x: float, y: float):
        """Dibuja un bloque de obstáculo en la posición del ratón."""
        if distancia_euclidea((x, y), self.punto_inicial) < 25 or \
           distancia_euclidea((x, y), self.punto_final) < 25:
            return

        tam = 14
        x1 = max(0, int(x - tam // 2))
        y1 = max(0, int(y - tam // 2))
        x2 = min(self.ancho_mapa, int(x + tam // 2))
        y2 = min(self.alto_mapa, int(y + tam // 2))

        rect = (x1, y1, x2, y2)
        self.obstaculos_reales.append(rect)

        # Dibujar según niebla
        if self.var_niebla.get():
            self.canvas.create_rectangle(
                x1, y1, x2, y2,
                fill=COLOR_OBS_TRANSLUCIDO, outline=COLOR_BORDE_TRANSLUCIDO,
                width=1.0, tags="obstaculo_tenue"
            )
        else:
            self.canvas.create_rectangle(
                x1, y1, x2, y2,
                fill=COLOR_OBSTACULO, outline=COLOR_BORDE_OBS,
                width=1.5, tags="obstaculo"
            )

        self._agregar_celdas_grid(rect)
        self.canvas.tag_raise("camino_a_star")
        self.canvas.tag_raise("rastro_recorrido")
        self.canvas.tag_raise("punto_inicial")
        self.canvas.tag_raise("punto_final")
        self.canvas.tag_raise("rango_vision")
        self.canvas.tag_raise("punto_azul")

    def _on_canvas_right_click(self, event):
        """Elimina obstáculos bajo el puntero del ratón al hacer clic/arrastre derecho."""
        if self.animando or not self.modo_pintar:
            return

        cx, cy = event.x, event.y
        borrados = False
        i = len(self.obstaculos_reales) - 1
        while i >= 0:
            x1, y1, x2, y2 = self.obstaculos_reales[i]
            if x1 - 6 <= cx <= x2 + 6 and y1 - 6 <= cy <= y2 + 6:
                self.obstaculos_reales.pop(i)
                borrados = True
            i -= 1

        if borrados:
            self._reconstruir_celdas_bloqueadas()
            self._redibujar_todo()

    # ==========================================
    # CELDAS BLOQUEADAS Y GESTIÓN DE OBSTÁCULOS
    # ==========================================
    def _agregar_celdas_grid(self, rect: tuple[int, int, int, int]):
        cols, rows = self._cols_rows()
        x1, y1, x2, y2 = rect
        gx1 = max(0, int((x1 - MARGEN_SEGURIDAD) // TAM_CELDA))
        gx2 = min(cols - 1, int((x2 + MARGEN_SEGURIDAD) // TAM_CELDA))
        gy1 = max(0, int((y1 - MARGEN_SEGURIDAD) // TAM_CELDA))
        gy2 = min(rows - 1, int((y2 + MARGEN_SEGURIDAD) // TAM_CELDA))
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                self.celdas_bloqueadas_totales.add((gx, gy))

    def _reconstruir_celdas_bloqueadas(self):
        self.celdas_bloqueadas_totales.clear()
        for rect in self.obstaculos_reales:
            self._agregar_celdas_grid(rect)

    def _registrar_obstaculo_descubierto(self, obs: tuple[int, int, int, int]):
        self.obstaculos_descubiertos.add(obs)
        cols, rows = self._cols_rows()
        x1, y1, x2, y2 = obs
        gx1 = max(0, int((x1 - MARGEN_SEGURIDAD) // TAM_CELDA))
        gx2 = min(cols - 1, int((x2 + MARGEN_SEGURIDAD) // TAM_CELDA))
        gy1 = max(0, int((y1 - MARGEN_SEGURIDAD) // TAM_CELDA))
        gy2 = min(rows - 1, int((y2 + MARGEN_SEGURIDAD) // TAM_CELDA))
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                self.celdas_bloqueadas_conocidas.add((gx, gy))

    def generar_obstaculos_aleatorios(self):
        if self.animando or self.tipo_mapa != "ORIGINAL":
            return

        self.borrar_obstaculos()
        self.lbl_estado.config(text="Generando obstáculos aleatorios...")
        self.root.update_idletasks()

        intentos = 15
        cols, rows = self._cols_rows()
        for _ in range(intentos):
            candidatos = []
            cantidad = random.randint(22, 32)
            bloqueadas_test = set()

            for _ in range(cantidad):
                w = random.randint(35, 75)
                h = random.randint(35, 75)
                x1 = random.randint(20, self.ancho_mapa - w - 20)
                y1 = random.randint(20, self.alto_mapa - h - 20)
                x2 = x1 + w
                y2 = y1 + h

                if distancia_euclidea(((x1 + x2) / 2, (y1 + y2) / 2), self.punto_inicial) < 70 or \
                   distancia_euclidea(((x1 + x2) / 2, (y1 + y2) / 2), self.punto_final) < 70:
                    continue

                candidatos.append((x1, y1, x2, y2))
                gx1 = max(0, int((x1 - MARGEN_SEGURIDAD) // TAM_CELDA))
                gx2 = min(cols - 1, int((x2 + MARGEN_SEGURIDAD) // TAM_CELDA))
                gy1 = max(0, int((y1 - MARGEN_SEGURIDAD) // TAM_CELDA))
                gy2 = min(rows - 1, int((y2 + MARGEN_SEGURIDAD) // TAM_CELDA))
                for gx in range(gx1, gx2 + 1):
                    for gy in range(gy1, gy2 + 1):
                        bloqueadas_test.add((gx, gy))

            if buscar_camino_a_star(self.punto_inicial, self.punto_final, bloqueadas_test, cols, rows) is not None:
                self.obstaculos_reales = candidatos
                break
        else:
            self.obstaculos_reales = candidatos[:15]

        self._reconstruir_celdas_bloqueadas()
        self.reiniciar_todo()
        self.lbl_estado.config(
            text=f"Se crearon {len(self.obstaculos_reales)} obstáculos. Pulsa '▶ Iniciar recorrido'."
        )

    def borrar_obstaculos(self):
        if self.animando:
            return
        self.obstaculos_reales.clear()
        self.celdas_bloqueadas_totales.clear()
        self.reiniciar_todo()
        self.lbl_estado.config(text="Obstáculos eliminados. Mapa libre.")

    # ==========================================
    # RENDERIZADO Y NIEBLA
    # ==========================================
    def _despejar_niebla(self, x: float, y: float, radio: float = RADIO_VISION):
        if not self.var_niebla.get():
            return
        self.canvas.create_oval(
            x - radio, y - radio,
            x + radio, y + radio,
            fill=COLOR_DESPEJADO, outline="",
            tags="area_despejada"
        )

    def _redibujar_todo(self):
        self.canvas.delete("all")

        if self.var_niebla.get():
            self.canvas.config(bg=COLOR_NIEBLA)

            # Siluetas tenues de los obstáculos bajo la niebla
            for x1, y1, x2, y2 in self.obstaculos_reales:
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=COLOR_OBS_TRANSLUCIDO, outline=COLOR_BORDE_TRANSLUCIDO,
                    width=1.0, tags="obstaculo_tenue"
                )

            # Despejar alrededor del punto de partida
            self._despejar_niebla(self.punto_inicial[0], self.punto_inicial[1], RADIO_VISION + 4)

            # Despejar a lo largo del recorrido ya realizado
            for hx, hy in self.historial_recorrido:
                self._despejar_niebla(hx, hy, RADIO_VISION)

            # Obstáculos descubiertos en gris oscuro sólido
            for x1, y1, x2, y2 in self.obstaculos_descubiertos:
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=COLOR_OBSTACULO, outline=COLOR_BORDE_OBS,
                    width=1.5, tags="obstaculo"
                )
        else:
            self.canvas.config(bg=COLOR_DESPEJADO)
            for x1, y1, x2, y2 in self.obstaculos_reales:
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=COLOR_OBSTACULO, outline=COLOR_BORDE_OBS,
                    width=1.5, tags="obstaculo"
                )

        # Trayectoria recorrida en NARANJA
        if len(self.historial_recorrido) >= 2:
            coords = []
            for px, py in self.historial_recorrido:
                coords.extend([px, py])
            self.canvas.create_line(
                *coords,
                fill=COLOR_RASTRO, width=2.5,
                capstyle="round", joinstyle="round",
                tags="rastro_recorrido"
            )

        # Trayectoria planificada futura
        self._redibujar_linea_camino()

        # Punto Inicial (Sin color y pequeñito)
        x0, y0 = self.punto_inicial
        r_ini = RADIO_INICIAL
        self.canvas.create_oval(
            x0 - r_ini, y0 - r_ini, x0 + r_ini, y0 + r_ini,
            fill="#ffffff", outline="#000000", width=1.2,
            tags="punto_inicial"
        )

        # Punto Final (Rojo inicialmente, verde al ser alcanzado)
        x1, y1 = self.punto_final
        r_fin = RADIO_FINAL
        col_fin = COLOR_FIN_ALCANZADO if self.meta_alcanzada else COLOR_FIN_PENDIENTE
        bor_fin = COLOR_FIN_BORDE_ALC if self.meta_alcanzada else COLOR_FIN_BORDE_PEND
        self.canvas.create_oval(
            x1 - r_fin, y1 - r_fin, x1 + r_fin, y1 + r_fin,
            fill=col_fin, outline=bor_fin, width=1.5,
            tags="punto_final"
        )

        # Campo de visión y Punto Móvil
        xa, ya = self.pos_actual
        if self.var_niebla.get():
            self.canvas.create_oval(
                xa - RADIO_VISION, ya - RADIO_VISION,
                xa + RADIO_VISION, ya + RADIO_VISION,
                outline="#2563eb", width=1, dash=(2, 2),
                tags="rango_vision"
            )

        col_punto = COLOR_FIN_ALCANZADO if self.meta_alcanzada else "#2563eb"
        bor_punto = COLOR_FIN_BORDE_ALC if self.meta_alcanzada else "#1d4ed8"
        self.canvas.create_oval(
            xa - RADIO_PUNTO, ya - RADIO_PUNTO,
            xa + RADIO_PUNTO, ya + RADIO_PUNTO,
            fill=col_punto, outline=bor_punto, width=1.5,
            tags="punto_azul"
        )

    def _actualizar_posicion_grafica(self):
        xa, ya = self.pos_actual

        if self.var_niebla.get():
            self._despejar_niebla(xa, ya, RADIO_VISION)
            self.canvas.delete("rango_vision")
            self.canvas.create_oval(
                xa - RADIO_VISION, ya - RADIO_VISION,
                xa + RADIO_VISION, ya + RADIO_VISION,
                outline="#2563eb", width=1, dash=(2, 2),
                tags="rango_vision"
            )

        self.canvas.coords(
            "punto_azul",
            xa - RADIO_PUNTO, ya - RADIO_PUNTO,
            xa + RADIO_PUNTO, ya + RADIO_PUNTO
        )

        self.canvas.tag_raise("camino_a_star")
        self.canvas.tag_raise("rastro_recorrido")
        self.canvas.tag_raise("obstaculo")
        self.canvas.tag_raise("punto_inicial")
        self.canvas.tag_raise("punto_final")
        self.canvas.tag_raise("rango_vision")
        self.canvas.tag_raise("punto_azul")

    def _on_toggle_niebla(self):
        self._redibujar_todo()

    # ==========================================
    # MOVIMIENTO, DETECCIÓN Y REPLANNING
    # ==========================================
    def iniciar_movimiento(self):
        if self.animando:
            return

        if distancia_euclidea(self.pos_actual, self.punto_final) < 5:
            self.reiniciar_todo()

        self.meta_alcanzada = False
        self.canvas.itemconfig("punto_final", fill=COLOR_FIN_PENDIENTE, outline=COLOR_FIN_BORDE_PEND)
        self.canvas.itemconfig("punto_azul", fill="#2563eb", outline="#1d4ed8")

        self.historial_recorrido = [self.pos_actual]
        self.canvas.delete("rastro_recorrido")

        waypoints = [self.pos_actual, self.punto_final]
        self.puntos_trayectoria = interpolar_puntos(waypoints, paso_px=self.velocidad_paso)
        self.indice_trayectoria = 0

        self._dibujar_trayectoria(waypoints)
        self.animando = True
        self._habilitar_botones(False)
        self.lbl_estado.config(text="Explorando... Trayectoria recta inicial hacia la meta.")
        self._paso_animacion()

    def _paso_animacion(self):
        if not self.animando:
            return

        # 1. Avanzar un paso y trazar la trayectoria recorrida en naranja
        if self.indice_trayectoria < len(self.puntos_trayectoria):
            pos_anterior = self.pos_actual
            self.pos_actual = self.puntos_trayectoria[self.indice_trayectoria]
            self.indice_trayectoria += 1
            self.historial_recorrido.append(self.pos_actual)

            self.canvas.create_line(
                pos_anterior[0], pos_anterior[1],
                self.pos_actual[0], self.pos_actual[1],
                fill=COLOR_RASTRO, width=2.5,
                capstyle="round", tags="rastro_recorrido"
            )
            self._actualizar_posicion_grafica()
        else:
            self.animando = False
            self.meta_alcanzada = True
            self.canvas.itemconfig("punto_final", fill=COLOR_FIN_ALCANZADO, outline=COLOR_FIN_BORDE_ALC)
            self.canvas.itemconfig("punto_azul", fill=COLOR_FIN_ALCANZADO, outline=COLOR_FIN_BORDE_ALC)
            self.canvas.tag_raise("punto_final")
            self.canvas.tag_raise("punto_azul")
            self._habilitar_botones(True)
            self.lbl_estado.config(text="🎯 ¡Meta alcanzada! El punto final se ha puesto en verde.")
            return

        # 2. Detección sensorial en el radio de 15 px
        xa, ya = self.pos_actual
        nuevos_obstaculos = []
        for obs in self.obstaculos_reales:
            if obs not in self.obstaculos_descubiertos:
                dist = distancia_punto_rectangulo(xa, ya, obs)
                if dist <= RADIO_VISION:
                    nuevos_obstaculos.append(obs)
                    self._registrar_obstaculo_descubierto(obs)
                    x1, y1, x2, y2 = obs
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2,
                        fill=COLOR_OBSTACULO, outline=COLOR_BORDE_OBS,
                        width=1.5, tags="obstaculo"
                    )

        # 3. Replanning con A* si un nuevo obstáculo corta la trayectoria
        if nuevos_obstaculos:
            puntos_restantes = self.puntos_trayectoria[self.indice_trayectoria:]
            bloqueado = any(obstaculo_bloquea_trayectoria(puntos_restantes, obs) for obs in nuevos_obstaculos)

            if bloqueado:
                cols, rows = self._cols_rows()
                nuevo_camino = buscar_camino_a_star(
                    self.pos_actual, self.punto_final,
                    self.celdas_bloqueadas_conocidas, cols, rows
                )
                if nuevo_camino:
                    waypoints = suavizar_camino(nuevo_camino, self.celdas_bloqueadas_conocidas)
                    self.puntos_trayectoria = interpolar_puntos(waypoints, paso_px=self.velocidad_paso)
                    self.indice_trayectoria = 0
                    self._dibujar_trayectoria(waypoints)
                    self.lbl_estado.config(
                        text=f"⚠️ ¡Muro detectado a {int(RADIO_VISION)} px! Recalculando con A* en tiempo real."
                    )
                else:
                    self.animando = False
                    self._habilitar_botones(True)
                    self.lbl_estado.config(text="⛔ ¡Atrapado! Los obstáculos bloquean completamente el acceso a la meta.")
                    return

        self.root.after(self.delay_ms, self._paso_animacion)

    def _dibujar_trayectoria(self, waypoints: list[tuple[float, float]]):
        self.canvas.delete("camino_a_star")
        if len(waypoints) < 2:
            return
        coords = []
        for x, y in waypoints:
            coords.extend([x, y])
        self.canvas.create_line(
            *coords,
            fill=COLOR_CAMINO_PLAN, width=2,
            dash=(4, 3), tags="camino_a_star",
            capstyle="round", joinstyle="round"
        )
        self.canvas.tag_raise("camino_a_star")
        self.canvas.tag_raise("rastro_recorrido")
        self.canvas.tag_raise("obstaculo")
        self.canvas.tag_raise("punto_inicial")
        self.canvas.tag_raise("punto_final")
        self.canvas.tag_raise("rango_vision")
        self.canvas.tag_raise("punto_azul")

    def _redibujar_linea_camino(self):
        puntos_restantes = self.puntos_trayectoria[self.indice_trayectoria:]
        if len(puntos_restantes) >= 2:
            coords = []
            salto = max(1, len(puntos_restantes) // 50)
            for pt in puntos_restantes[::salto]:
                coords.extend([pt[0], pt[1]])
            coords.extend([puntos_restantes[-1][0], puntos_restantes[-1][1]])
            self.canvas.create_line(
                *coords,
                fill=COLOR_CAMINO_PLAN, width=2,
                dash=(4, 3), tags="camino_a_star"
            )

    def reiniciar_todo(self):
        self.animando = False
        self.meta_alcanzada = False
        self.pos_actual = self.punto_inicial
        self.historial_recorrido = [self.punto_inicial]
        self.indice_trayectoria = 0
        self.puntos_trayectoria = []
        self.obstaculos_descubiertos.clear()
        self.celdas_bloqueadas_conocidas.clear()

        self._redibujar_todo()
        self._habilitar_botones(True)
        self.lbl_estado.config(text="Listo. Pulsa '▶ Iniciar recorrido' o '🖌️ Pintar con ratón'.")


def main():
    root = tk.Tk()
    app = SimuladorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
