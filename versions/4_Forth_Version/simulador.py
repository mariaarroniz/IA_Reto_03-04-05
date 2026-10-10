#!/usr/bin/env python3
"""
Simulador de navegación sobre un mapa de 600 px x 800 px con Niebla de Guerra semi-transparente
y Replanning en tiempo real (A* con evasión dinámica de obstáculos).

- Dimensiones del mapa: 600 px x 800 px
- Niebla: Gris claro y semi-transparente (permite entrever siluetas tenues de obstáculos)
- Trayectoria: Color naranja vibrante (permanece visible a medida que avanza)
- Punto inicial: Sin color, pequeñito en (50, 50)
- Punto final: Rojo inicialmente en (550, 750); cambia a color verde al llegar el punto azul
- Punto móvil: Azul (Radio 4 px); cambia a verde al completar el objetivo
- Campo de visión: Radio de 15 px (descubre la niebla y obstáculos sobre la marcha)
- Navegación: Inicialmente traza una línea recta al objetivo y, al descubrir obstáculos
  en su trayectoria dentro del rango de visión, replanifica en tiempo real con A*.
"""

import heapq
import math
import random
import sys

try:
    import tkinter as tk
except ImportError:
    print("\n" + "=" * 65)
    print("ERROR: El módulo 'tkinter' no está instalado en este entorno Python.")
    print("Para instalarlo en Ubuntu/Debian, ejecuta en tu terminal:")
    print("    sudo apt update && sudo apt install python3-tk")
    print("=" * 65 + "\n")
    sys.exit(1)


# ==========================================
# CONFIGURACIÓN DEL MAPA Y PARÁMETROS
# ==========================================
ANCHO_MAPA = 600   # Ancho en píxeles
ALTO_MAPA = 800    # Alto en píxeles

# Coordenadas de inicio y fin
PUNTO_INICIAL = (50.0, 50.0)
PUNTO_FINAL = (550.0, 750.0)

# Parámetros visuales de los puntos
RADIO_INICIAL = 2.5    # Pequeñito y sin color
RADIO_FINAL = 5.0      # Punto final
RADIO_PUNTO = 4.0      # Radio del punto azul en píxeles
RADIO_VISION = 15.0    # Radio del campo de visión (según consigna: 15 px)
MARGEN_SEGURIDAD = 6.0 # Margen alrededor de los obstáculos para evitar colisiones

# Resolución de la cuadrícula interna para A* (cada celda mide 6x6 px para alto rendimiento)
TAM_CELDA = 6
COLS = ANCHO_MAPA // TAM_CELDA  # 100 columnas
ROWS = ALTO_MAPA // TAM_CELDA   # 133 filas

# Colores de la interfaz (Niebla gris clara y semi-transparente)
COLOR_NIEBLA = "#cbd5e1"         # Gris claro suave para la niebla (Tailwind Slate-300)
COLOR_DESPEJADO = "#ffffff"      # Blanco puro para el terreno descubierto por la visión

# Obstáculos: tenues bajo la niebla semi-transparente, sólidos al ser descubiertos
COLOR_OBS_TRANSLUCIDO = "#94a3b8" # Silueta tenue visible a través de la niebla
COLOR_BORDE_TRANSLUCIDO = "#94a3b8"
COLOR_OBSTACULO = "#334155"      # Gris oscuro sólido para los obstáculos descubiertos
COLOR_BORDE_OBS = "#1e293b"

# Trayectoria: Naranja vibrante
COLOR_RASTRO = "#ea580c"         # Naranja intenso para la trayectoria recorrida visible
COLOR_CAMINO_PLAN = "#fb923c"    # Naranja claro discontinuo para la trayectoria planificada futura

# Colores del Punto Final
COLOR_FIN_PENDIENTE = "#ef4444"  # Rojo antes de llegar
COLOR_FIN_BORDE_PEND = "#b91c1c"
COLOR_FIN_ALCANZADO = "#22c55e"  # Verde al llegar el punto azul
COLOR_FIN_BORDE_ALC = "#15803d"


# ==========================================
# FUNCIONES AUXILIARES DE GEOMETRÍA Y A*
# ==========================================
def distancia_euclidea(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """Calcula la distancia euclídea entre dos puntos."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def distancia_punto_rectangulo(px: float, py: float, rect: tuple[int, int, int, int]) -> float:
    """Calcula la distancia mínima exacta entre un punto y un rectángulo (x1, y1, x2, y2)."""
    x1, y1, x2, y2 = rect
    cx = max(x1, min(px, x2))
    cy = max(y1, min(py, y2))
    return math.hypot(px - cx, py - cy)


def obstaculo_bloquea_trayectoria(
    puntos_restantes: list[tuple[float, float]],
    rect: tuple[int, int, int, int],
    holgura: float = MARGEN_SEGURIDAD + RADIO_PUNTO
) -> bool:
    """Comprueba si algún punto de la trayectoria futura colisiona con el obstáculo."""
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
    celdas_bloqueadas: set[tuple[int, int]]
) -> list[tuple[float, float]] | None:
    """
    Encuentra la ruta de menor coste hacia el objetivo evitando las celdas conocidas.
    Si no hay obstáculos conocidos, devuelve directamente la línea recta.
    """
    if not celdas_bloqueadas:
        return [inicio, fin]

    gx0 = int(round(inicio[0] / TAM_CELDA))
    gy0 = int(round(inicio[1] / TAM_CELDA))
    gx1 = int(round(fin[0] / TAM_CELDA))
    gy1 = int(round(fin[1] / TAM_CELDA))

    # Asegurar que el punto de partida y fin no queden bloqueados por el margen de seguridad
    bloqueadas_temp = celdas_bloqueadas.copy()
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            bloqueadas_temp.discard((gx0 + dx, gy0 + dy))
            bloqueadas_temp.discard((gx1 + dx, gy1 + dy))

    start = (gx0, gy0)
    goal = (gx1, gy1)

    # 8 direcciones de movimiento con costes Euclídeos
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
            # Convertir de cuadrícula a coordenadas de píxeles
            puntos_px = [(gx * TAM_CELDA, gy * TAM_CELDA) for gx, gy in path]
            puntos_px[0] = inicio
            puntos_px[-1] = fin
            return puntos_px

        if g > g_score.get(actual, float("inf")):
            continue

        cx, cy = actual
        for dx, dy, coste in movimientos:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < COLS and 0 <= ny < ROWS:
                if (nx, ny) in bloqueadas_temp:
                    continue
                # Evitar cortar esquinas si ambos lados adyacentes están bloqueados
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
    """Comprueba si el segmento recto entre p1 y p2 no corta ninguna celda bloqueada."""
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
    """
    Optimiza la trayectoria con Raycasting / Línea de visión directa:
    conecta esquinas con tramos rectos para obtener el camino euclídeo más corto.
    """
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
    """Genera pasos continuos a velocidad constante a lo largo de los waypoints."""
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
# INTERFAZ GRÁFICA Y SIMULADOR
# ==========================================
class SimuladorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Simulador 600x800 con Niebla de Guerra y Replanning (A*)")
        self.root.resizable(False, False)

        # Estado del mundo
        self.obstaculos_reales: list[tuple[int, int, int, int]] = []
        self.obstaculos_descubiertos: set[tuple[int, int, int, int]] = set()
        self.celdas_bloqueadas_conocidas: set[tuple[int, int]] = set()

        # Opciones
        self.var_niebla = tk.BooleanVar(value=True)

        # Estado del agente y meta
        self.meta_alcanzada = False
        self.pos_actual = PUNTO_INICIAL
        self.historial_recorrido: list[tuple[float, float]] = [PUNTO_INICIAL]
        self.animando = False
        self.puntos_trayectoria: list[tuple[float, float]] = []
        self.indice_trayectoria = 0
        self.delay_ms = 20  # ~50 FPS
        self.velocidad_paso = 2.0 # Píxeles por fotograma

        self._crear_interfaz()
        self.reiniciar_todo()

    def _crear_interfaz(self):
        contenedor = tk.Frame(self.root, padx=8, pady=8)
        contenedor.pack()

        # Barra superior de controles (compacta)
        barra_controles = tk.Frame(contenedor)
        barra_controles.pack(fill="x", pady=(0, 6))

        # Fila 1: Información y Checkbox de Niebla
        fila_info = tk.Frame(barra_controles)
        fila_info.pack(fill="x", pady=(0, 4))

        lbl_dimensiones = tk.Label(
            fila_info,
            text=f"Mapa: {ANCHO_MAPA}x{ALTO_MAPA} px  |  Visión: {int(RADIO_VISION)} px",
            font=("Arial", 10, "bold"),
            fg="#1e293b"
        )
        lbl_dimensiones.pack(side="left")

        chk_niebla = tk.Checkbutton(
            fila_info,
            text="🌫️ Niebla de guerra activa",
            variable=self.var_niebla,
            command=self._on_toggle_niebla,
            font=("Arial", 9, "bold"),
            fg="#0f172a"
        )
        chk_niebla.pack(side="right")

        # Fila 2: Botones de acción
        fila_botones = tk.Frame(barra_controles)
        fila_botones.pack(fill="x", pady=(0, 4))

        self.btn_generar_obs = tk.Button(
            fila_botones,
            text="🎲 Obstáculos aleatorios",
            command=self.generar_obstaculos_aleatorios,
            bg="#f1f5f9",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_generar_obs.pack(side="left", padx=(0, 4))

        self.btn_borrar_obs = tk.Button(
            fila_botones,
            text="🗑️ Borrar obstáculos",
            command=self.borrar_obstaculos,
            bg="#f1f5f9",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_borrar_obs.pack(side="left", padx=(0, 6))

        self.btn_iniciar = tk.Button(
            fila_botones,
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
            fila_botones,
            text="↺ Reiniciar",
            command=self.reiniciar_todo,
            bg="#f1f5f9",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_reiniciar.pack(side="right")

        # Fila 3: Etiqueta de estado
        self.lbl_estado = tk.Label(
            barra_controles,
            text="Listo. Pulsa '🎲 Obstáculos aleatorios' o '▶ Iniciar recorrido'.",
            font=("Arial", 9),
            fg="#334155"
        )
        self.lbl_estado.pack(fill="x", pady=(2, 0))

        # Lienzo (Canvas) de 600 x 800 píxeles con la nueva niebla gris clara
        self.canvas = tk.Canvas(
            contenedor,
            width=ANCHO_MAPA,
            height=ALTO_MAPA,
            bg=COLOR_NIEBLA,
            highlightthickness=1,
            highlightbackground="#94a3b8"
        )
        self.canvas.pack()

        # Clic para añadir/quitar obstáculo manual
        self.canvas.bind("<Button-1>", self._on_canvas_click)

    def _habilitar_botones(self, habilitar: bool):
        estado = "normal" if habilitar else "disabled"
        self.btn_iniciar.config(state=estado)
        self.btn_generar_obs.config(state=estado)
        self.btn_borrar_obs.config(state=estado)
        self.btn_reiniciar.config(state=estado)

    # ==========================================
    # RENDERIZADO Y NIEBLA DE GUERRA
    # ==========================================
    def _despejar_niebla(self, x: float, y: float, radio: float = RADIO_VISION):
        """Despeja un círculo de niebla en el mapa (lo vuelve blanco puro)."""
        if not self.var_niebla.get():
            return
        self.canvas.create_oval(
            x - radio, y - radio,
            x + radio, y + radio,
            fill=COLOR_DESPEJADO,
            outline="",
            tags="area_despejada"
        )

    def _redibujar_todo(self):
        """Redibuja el mapa según el estado de la niebla, obstáculos y trayectoria."""
        self.canvas.delete("all")

        if self.var_niebla.get():
            self.canvas.config(bg=COLOR_NIEBLA)

            # Siluetas tenues de los obstáculos bajo la niebla semi-transparente
            for x1, y1, x2, y2 in self.obstaculos_reales:
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=COLOR_OBS_TRANSLUCIDO,
                    outline=COLOR_BORDE_TRANSLUCIDO,
                    width=1.0,
                    tags="obstaculo_tenue"
                )

            # Despejar el área alrededor del punto de inicio (blanco puro)
            self._despejar_niebla(PUNTO_INICIAL[0], PUNTO_INICIAL[1], RADIO_VISION + 4)

            # Despejar a lo largo de todo el camino recorrido por el agente
            for hx, hy in self.historial_recorrido:
                self._despejar_niebla(hx, hy, RADIO_VISION)

            # Dibujar los obstáculos descubiertos (en la zona despejada) en gris oscuro sólido
            for x1, y1, x2, y2 in self.obstaculos_descubiertos:
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=COLOR_OBSTACULO,
                    outline=COLOR_BORDE_OBS,
                    width=1.5,
                    tags="obstaculo"
                )
        else:
            # Modo sin niebla: fondo blanco y todos los obstáculos sólidos visibles
            self.canvas.config(bg=COLOR_DESPEJADO)
            for x1, y1, x2, y2 in self.obstaculos_reales:
                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=COLOR_OBSTACULO,
                    outline=COLOR_BORDE_OBS,
                    width=1.5,
                    tags="obstaculo"
                )

        # Redibujar la trayectoria recorrida acumulada en color NARANJA (visible permanentemente)
        if len(self.historial_recorrido) >= 2:
            coords_recorrido = []
            for px, py in self.historial_recorrido:
                coords_recorrido.extend([px, py])
            self.canvas.create_line(
                *coords_recorrido,
                fill=COLOR_RASTRO,
                width=2.5,
                capstyle="round",
                joinstyle="round",
                tags="rastro_recorrido"
            )

        # Línea de la trayectoria planificada hacia adelante en color naranja
        self._redibujar_linea_camino()

        # Punto Inicial (Sin color y pequeñito)
        x0, y0 = PUNTO_INICIAL
        r_ini = RADIO_INICIAL
        self.canvas.create_oval(
            x0 - r_ini, y0 - r_ini, x0 + r_ini, y0 + r_ini,
            fill="#ffffff", outline="#000000", width=1.2,
            tags="punto_inicial"
        )

        # Punto Final (Rojo inicialmente, verde al ser alcanzado)
        x1, y1 = PUNTO_FINAL
        r_fin = RADIO_FINAL
        col_fin = COLOR_FIN_ALCANZADO if self.meta_alcanzada else COLOR_FIN_PENDIENTE
        bor_fin = COLOR_FIN_BORDE_ALC if self.meta_alcanzada else COLOR_FIN_BORDE_PEND
        self.canvas.create_oval(
            x1 - r_fin, y1 - r_fin, x1 + r_fin, y1 + r_fin,
            fill=col_fin, outline=bor_fin, width=1.5,
            tags="punto_final"
        )

        # Círculo de visión y Punto Móvil (Agente)
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
        """Actualiza la posición del punto azul, campo de visión y asegura el orden visual."""
        xa, ya = self.pos_actual

        if self.var_niebla.get():
            # Despejar la niebla en la nueva posición (blanco)
            self._despejar_niebla(xa, ya, RADIO_VISION)

            # Actualizar círculo de visión
            self.canvas.delete("rango_vision")
            self.canvas.create_oval(
                xa - RADIO_VISION, ya - RADIO_VISION,
                xa + RADIO_VISION, ya + RADIO_VISION,
                outline="#2563eb", width=1, dash=(2, 2),
                tags="rango_vision"
            )

        # Mover el punto azul
        self.canvas.coords(
            "punto_azul",
            xa - RADIO_PUNTO, ya - RADIO_PUNTO,
            xa + RADIO_PUNTO, ya + RADIO_PUNTO
        )

        # Asegurar orden visual en pantalla: el rastro naranja y los puntos siempre en primer plano
        self.canvas.tag_raise("camino_a_star")
        self.canvas.tag_raise("rastro_recorrido")
        self.canvas.tag_raise("obstaculo")
        self.canvas.tag_raise("punto_inicial")
        self.canvas.tag_raise("punto_final")
        self.canvas.tag_raise("rango_vision")
        self.canvas.tag_raise("punto_azul")

    def _on_toggle_niebla(self):
        """Maneja el cambio del interruptor de niebla."""
        self._redibujar_todo()

    # ==========================================
    # GESTIÓN DE OBSTÁCULOS
    # ==========================================
    def generar_obstaculos_aleatorios(self):
        """Genera obstáculos aleatorios asegurando que exista una ruta factible global."""
        if self.animando:
            return

        self.borrar_obstaculos()
        self.lbl_estado.config(text="Generando obstáculos...")
        self.root.update_idletasks()

        # Crear entre 22 y 32 bloques rectangulares
        intentos = 15
        for _ in range(intentos):
            candidatos = []
            cantidad = random.randint(22, 32)
            celdas_bloqueadas_test = set()

            for _ in range(cantidad):
                w = random.randint(35, 75)
                h = random.randint(35, 75)
                x1 = random.randint(20, ANCHO_MAPA - w - 20)
                y1 = random.randint(20, ALTO_MAPA - h - 20)
                x2 = x1 + w
                y2 = y1 + h

                # No colocar cerca de inicio o fin
                if distancia_euclidea(((x1 + x2) / 2, (y1 + y2) / 2), PUNTO_INICIAL) < 70 or \
                   distancia_euclidea(((x1 + x2) / 2, (y1 + y2) / 2), PUNTO_FINAL) < 70:
                    continue

                candidatos.append((x1, y1, x2, y2))
                # Registrar celdas para validar que haya camino
                gx1 = max(0, int((x1 - MARGEN_SEGURIDAD) // TAM_CELDA))
                gx2 = min(COLS - 1, int((x2 + MARGEN_SEGURIDAD) // TAM_CELDA))
                gy1 = max(0, int((y1 - MARGEN_SEGURIDAD) // TAM_CELDA))
                gy2 = min(ROWS - 1, int((y2 + MARGEN_SEGURIDAD) // TAM_CELDA))
                for gx in range(gx1, gx2 + 1):
                    for gy in range(gy1, gy2 + 1):
                        celdas_bloqueadas_test.add((gx, gy))

            # Comprobar con A* que existe un camino factible en el mapa
            if buscar_camino_a_star(PUNTO_INICIAL, PUNTO_FINAL, celdas_bloqueadas_test) is not None:
                self.obstaculos_reales = candidatos
                break
        else:
            self.obstaculos_reales = candidatos[:15]

        self.reiniciar_todo()
        self.lbl_estado.config(
            text=f"Se generaron {len(self.obstaculos_reales)} obstáculos bajo la niebla. Pulsa '▶ Iniciar recorrido'."
        )

    def borrar_obstaculos(self):
        """Elimina todos los obstáculos."""
        if self.animando:
            return
        self.obstaculos_reales.clear()
        self.reiniciar_todo()
        self.lbl_estado.config(text="Obstáculos eliminados. Mapa limpio.")

    def _on_canvas_click(self, event):
        """Permite hacer clic para añadir o quitar obstáculos manualmente."""
        if self.animando:
            return

        cx, cy = event.x, event.y
        if distancia_euclidea((cx, cy), PUNTO_INICIAL) < 40 or distancia_euclidea((cx, cy), PUNTO_FINAL) < 40:
            return

        # Si se hace clic sobre un obstáculo existente, eliminarlo
        for i, (x1, y1, x2, y2) in enumerate(self.obstaculos_reales):
            if x1 <= cx <= x2 and y1 <= cy <= y2:
                self.obstaculos_reales.pop(i)
                self.reiniciar_todo()
                self.lbl_estado.config(text="Obstáculo manual eliminado.")
                return

        # Si no, añadir un bloque de 50x50 px
        w, h = 50, 50
        x1 = max(0, cx - w // 2)
        y1 = max(0, cy - h // 2)
        x2 = min(ANCHO_MAPA, x1 + w)
        y2 = min(ALTO_MAPA, y1 + h)
        self.obstaculos_reales.append((x1, y1, x2, y2))
        self.reiniciar_todo()
        self.lbl_estado.config(text="Obstáculo manual añadido.")

    def _registrar_obstaculo_descubierto(self, obs: tuple[int, int, int, int]):
        """Agrega un obstáculo descubierto a la base de conocimiento del agente."""
        self.obstaculos_descubiertos.add(obs)
        x1, y1, x2, y2 = obs
        gx1 = max(0, int((x1 - MARGEN_SEGURIDAD) // TAM_CELDA))
        gx2 = min(COLS - 1, int((x2 + MARGEN_SEGURIDAD) // TAM_CELDA))
        gy1 = max(0, int((y1 - MARGEN_SEGURIDAD) // TAM_CELDA))
        gy2 = min(ROWS - 1, int((y2 + MARGEN_SEGURIDAD) // TAM_CELDA))
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                self.celdas_bloqueadas_conocidas.add((gx, gy))

    # ==========================================
    # MOVIMIENTO, SENSOR Y REPLANNING (ONLINE A*)
    # ==========================================
    def iniciar_movimiento(self):
        """Inicia el movimiento con trayectoria recta inicial y replanning sobre la marcha."""
        if self.animando:
            return

        # Si ya llegó al final, reiniciar primero
        if distancia_euclidea(self.pos_actual, PUNTO_FINAL) < 5:
            self.reiniciar_todo()

        # Restablecer estado de la meta y colores iniciales
        self.meta_alcanzada = False
        self.canvas.itemconfig("punto_final", fill=COLOR_FIN_PENDIENTE, outline=COLOR_FIN_BORDE_PEND)
        self.canvas.itemconfig("punto_azul", fill="#2563eb", outline="#1d4ed8")

        # Iniciar rastro recorrido desde la posición inicial
        self.historial_recorrido = [self.pos_actual]
        self.canvas.delete("rastro_recorrido")

        # Trayectoria inicial: línea recta directa (el agente no conoce los obstáculos bajo la niebla)
        waypoints_iniciales = [self.pos_actual, PUNTO_FINAL]
        self.puntos_trayectoria = interpolar_puntos(waypoints_iniciales, paso_px=self.velocidad_paso)
        self.indice_trayectoria = 0

        self._dibujar_trayectoria(waypoints_iniciales)
        self.animando = True
        self._habilitar_botones(False)
        self.lbl_estado.config(
            text="Explorando... Trayectoria recta inicial hacia el objetivo."
        )
        self._paso_animacion()

    def _paso_animacion(self):
        """Bucle de ejecución paso a paso con detección sensorial y replanning en tiempo real."""
        if not self.animando:
            return

        # 1. Avanzar la posición del agente y trazar su recorrido visible en NARANJA
        if self.indice_trayectoria < len(self.puntos_trayectoria):
            pos_anterior = self.pos_actual
            self.pos_actual = self.puntos_trayectoria[self.indice_trayectoria]
            self.indice_trayectoria += 1
            self.historial_recorrido.append(self.pos_actual)

            # Dibujar el nuevo segmento de la trayectoria recorrida (color naranja permanente)
            self.canvas.create_line(
                pos_anterior[0], pos_anterior[1],
                self.pos_actual[0], self.pos_actual[1],
                fill=COLOR_RASTRO,
                width=2.5,
                capstyle="round",
                tags="rastro_recorrido"
            )

            self._actualizar_posicion_grafica()
        else:
            # ¡Llegada al punto final!
            self.animando = False
            self.meta_alcanzada = True

            # Cambiar el punto final (y el punto azul) a color verde
            self.canvas.itemconfig("punto_final", fill=COLOR_FIN_ALCANZADO, outline=COLOR_FIN_BORDE_ALC)
            self.canvas.itemconfig("punto_azul", fill=COLOR_FIN_ALCANZADO, outline=COLOR_FIN_BORDE_ALC)
            self.canvas.tag_raise("punto_final")
            self.canvas.tag_raise("punto_azul")

            self._habilitar_botones(True)
            self.lbl_estado.config(text="🎯 ¡Objetivo alcanzado con éxito! El punto final se ha puesto en verde.")
            return

        # 2. Sensor de visión: escanear obstáculos dentro del radio de 15 px
        xa, ya = self.pos_actual
        nuevos_obstaculos = []
        for obs in self.obstaculos_reales:
            if obs not in self.obstaculos_descubiertos:
                dist = distancia_punto_rectangulo(xa, ya, obs)
                if dist <= RADIO_VISION:
                    nuevos_obstaculos.append(obs)
                    self._registrar_obstaculo_descubierto(obs)
                    # Dibujar de inmediato el obstáculo revelado en pantalla en gris sólido
                    x1, y1, x2, y2 = obs
                    self.canvas.create_rectangle(
                        x1, y1, x2, y2,
                        fill=COLOR_OBSTACULO,
                        outline=COLOR_BORDE_OBS,
                        width=1.5,
                        tags="obstaculo"
                    )

        # 3. Comprobar si algún obstáculo recién descubierto bloquea el camino futuro
        if nuevos_obstaculos:
            puntos_restantes = self.puntos_trayectoria[self.indice_trayectoria:]
            bloqueado = False
            for obs in nuevos_obstaculos:
                if obstaculo_bloquea_trayectoria(puntos_restantes, obs):
                    bloqueado = True
                    break

            # 4. REPLANNING DINÁMICO: Recalcular la trayectoria usando A*
            if bloqueado:
                nuevo_camino = buscar_camino_a_star(
                    self.pos_actual, PUNTO_FINAL, self.celdas_bloqueadas_conocidas
                )
                if nuevo_camino:
                    waypoints_suaves = suavizar_camino(nuevo_camino, self.celdas_bloqueadas_conocidas)
                    self.puntos_trayectoria = interpolar_puntos(waypoints_suaves, paso_px=self.velocidad_paso)
                    self.indice_trayectoria = 0
                    self._dibujar_trayectoria(waypoints_suaves)
                    self.lbl_estado.config(
                        text=f"⚠️ ¡Obstáculo detectado a {int(RADIO_VISION)} px! Trayectoria recalculada con A*."
                    )
                else:
                    self.animando = False
                    self._habilitar_botones(True)
                    self.lbl_estado.config(
                        text="⛔ ¡Atrapado! Los obstáculos impiden llegar al objetivo."
                    )
                    return

        # Continuar siguiente fotograma
        self.root.after(self.delay_ms, self._paso_animacion)

    def _dibujar_trayectoria(self, waypoints: list[tuple[float, float]]):
        """Dibuja la línea de la trayectoria planificada futura hacia el objetivo en naranja."""
        self.canvas.delete("camino_a_star")
        if len(waypoints) < 2:
            return
        coords = []
        for x, y in waypoints:
            coords.extend([x, y])
        self.canvas.create_line(
            *coords,
            fill=COLOR_CAMINO_PLAN,
            width=2,
            dash=(4, 3),
            tags="camino_a_star",
            capstyle="round",
            joinstyle="round"
        )
        self.canvas.tag_raise("camino_a_star")
        self.canvas.tag_raise("rastro_recorrido")
        self.canvas.tag_raise("obstaculo")
        self.canvas.tag_raise("punto_inicial")
        self.canvas.tag_raise("punto_final")
        self.canvas.tag_raise("rango_vision")
        self.canvas.tag_raise("punto_azul")

    def _redibujar_linea_camino(self):
        """Restaura la línea de la trayectoria planificada futura si se redibuja el lienzo."""
        puntos_restantes = self.puntos_trayectoria[self.indice_trayectoria:]
        if len(puntos_restantes) >= 2:
            coords = []
            salto = max(1, len(puntos_restantes) // 50)
            for pt in puntos_restantes[::salto]:
                coords.extend([pt[0], pt[1]])
            coords.extend([puntos_restantes[-1][0], puntos_restantes[-1][1]])
            self.canvas.create_line(
                *coords,
                fill=COLOR_CAMINO_PLAN,
                width=2,
                dash=(4, 3),
                tags="camino_a_star"
            )

    def reiniciar_todo(self):
        """Reinicia la simulación, la niebla y coloca al agente en el inicio."""
        self.animando = False
        self.meta_alcanzada = False
        self.pos_actual = PUNTO_INICIAL
        self.historial_recorrido = [PUNTO_INICIAL]
        self.indice_trayectoria = 0
        self.puntos_trayectoria = []
        self.obstaculos_descubiertos.clear()
        self.celdas_bloqueadas_conocidas.clear()

        self._redibujar_todo()
        self._habilitar_botones(True)
        self.lbl_estado.config(text="Listo. Pulsa '▶ Iniciar recorrido' para navegar el mapa.")


def main():
    root = tk.Tk()
    app = SimuladorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
