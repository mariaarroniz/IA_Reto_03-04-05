#!/usr/bin/env python3
"""
Simulador de navegación sobre un mapa de 100 px x 100 px con evasión de obstáculos.
- Punto inicial: Rojo
- Punto final: Verde
- Punto móvil: Azul
- Algoritmo: A* (A-Estrella) con suavizado de ruta (manera más eficaz y directa)
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
# CONFIGURACIÓN DEL MAPA Y ESCALA
# ==========================================
ANCHO_MAPA = 100   # Ancho lógico del mapa en píxeles
ALTO_MAPA = 100    # Alto lógico del mapa en píxeles

# Factor de escala visual para que sea cómodo de ver en pantalla (100x100 unidades -> 400x400 px)
# Si necesitas que el lienzo mida exactamente 100x100 px físicos, cambia ESCALA = 1.
ESCALA = 4

# Coordenadas en el mapa [0, 100]
PUNTO_INICIAL = (15, 15)   # (x, y) - Rojo
PUNTO_FINAL = (85, 85)     # (x, y) - Verde

# Radio del punto (en unidades del mapa)
RADIO = 3

# Margen de seguridad para evitar que el círculo del punto azul colisione con las esquinas
MARGEN_SEGURIDAD = 2


# ==========================================
# ALGORITMO A* Y SUAVIZADO DE RUTA
# ==========================================
def distancia_euclidea(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    """Calcula la distancia euclídea entre dos puntos."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def linea_libre(p1: tuple[int, int], p2: tuple[int, int], bloqueados: set[tuple[int, int]]) -> bool:
    """Comprueba si el segmento recto entre p1 y p2 está completamente libre de obstáculos."""
    x0, y0 = p1
    x1, y1 = p2
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist == 0:
        return True
    
    pasos = max(1, int(dist * 2))
    for i in range(pasos + 1):
        t = i / pasos
        x = round(x0 + t * (x1 - x0))
        y = round(y0 + t * (y1 - y0))
        if (x, y) in bloqueados:
            return False
    return True


def suavizar_camino(camino: list[tuple[int, int]], bloqueados: set[tuple[int, int]]) -> list[tuple[int, int]]:
    """
    Optimiza la trayectoria eliminando waypoints intermedios redundantes
    mediante comprobación de línea de visión directa (raycasting).
    Convierte el camino en los segmentos rectos más cortos posibles.
    """
    if not camino or len(camino) <= 2:
        return camino

    suavizado = [camino[0]]
    idx_actual = 0
    while idx_actual < len(camino) - 1:
        siguiente_idx = len(camino) - 1
        while siguiente_idx > idx_actual + 1:
            if linea_libre(camino[idx_actual], camino[siguiente_idx], bloqueados):
                break
            siguiente_idx -= 1
        suavizado.append(camino[siguiente_idx])
        idx_actual = siguiente_idx

    return suavizado


def buscar_camino_a_star(
    inicio: tuple[int, int],
    fin: tuple[int, int],
    bloqueados: set[tuple[int, int]],
    ancho: int = ANCHO_MAPA,
    alto: int = ALTO_MAPA
) -> list[tuple[int, int]] | None:
    """
    Encuentra la ruta óptima de menor coste entre inicio y fin usando A*.
    Utiliza movimiento en 8 direcciones con costes Euclídeos (1 y sqrt(2)).
    """
    if inicio in bloqueados or fin in bloqueados:
        return None

    # Movimientos en 8 direcciones: (dx, dy, coste)
    movimientos = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (-1, -1, 1.4142)
    ]

    pq = [(distancia_euclidea(inicio, fin), 0, 0.0, inicio)]
    g_score: dict[tuple[int, int], float] = {inicio: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    contador = 0

    while pq:
        _, _, g, actual = heapq.heappop(pq)

        if actual == fin:
            # Reconstruir camino
            camino = []
            curr = fin
            while curr in came_from:
                camino.append(curr)
                curr = came_from[curr]
            camino.append(inicio)
            camino.reverse()
            return camino

        if g > g_score.get(actual, float("inf")):
            continue

        cx, cy = actual
        for dx, dy, coste in movimientos:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < ancho and 0 <= ny < alto:
                if (nx, ny) in bloqueados:
                    continue
                # Evitar cortar esquinas si ambos lados adyacentes están bloqueados
                if dx != 0 and dy != 0:
                    if (cx + dx, cy) in bloqueados and (cx, cy + dy) in bloqueados:
                        continue

                nuevo_g = g + coste
                if nuevo_g < g_score.get((nx, ny), float("inf")):
                    g_score[(nx, ny)] = nuevo_g
                    came_from[(nx, ny)] = actual
                    contador += 1
                    h = distancia_euclidea((nx, ny), fin)
                    heapq.heappush(pq, (nuevo_g + h, contador, nuevo_g, (nx, ny)))

    return None


# ==========================================
# INTERFAZ GRÁFICA
# ==========================================
class SimuladorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Simulador de Movimiento A* (100x100)")
        self.root.resizable(False, False)

        # Estado de obstáculos y colisiones
        self.obstaculos_rects: list[tuple[int, int, int, int]] = []
        self.celdas_bloqueadas: set[tuple[int, int]] = set()

        # Estado de animación
        self.animando = False
        self.puntos_trayectoria: list[tuple[float, float]] = []
        self.indice_trayectoria = 0
        self.delay_ms = 20  # Milisegundos entre fotogramas (~50 FPS)

        self._crear_interfaz()
        self._dibujar_elementos_estaticos()
        self._crear_punto_azul()

    def _a_pantalla(self, x: float, y: float) -> tuple[float, float]:
        """Convierte coordenadas del mapa a píxeles de pantalla según la escala."""
        return x * ESCALA, y * ESCALA

    def _crear_interfaz(self):
        contenedor = tk.Frame(self.root, padx=12, pady=12)
        contenedor.pack()

        # Título / información
        info_label = tk.Label(
            contenedor,
            text=f"Mapa: {ANCHO_MAPA} px x {ALTO_MAPA} px  |  Escala visual: x{ESCALA}",
            font=("Arial", 10, "bold")
        )
        info_label.pack(pady=(0, 6))

        # Lienzo (Canvas) en blanco
        self.canvas = tk.Canvas(
            contenedor,
            width=ANCHO_MAPA * ESCALA,
            height=ALTO_MAPA * ESCALA,
            bg="white",
            highlightthickness=1,
            highlightbackground="#9ca3af"
        )
        self.canvas.pack()

        # Evento opcional: clic en el mapa para añadir/quitar un obstáculo manual
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        # Etiqueta de estado
        self.lbl_estado = tk.Label(
            contenedor,
            text="Listo. Genera obstáculos o pulsa 'Mover punto azul'.",
            font=("Arial", 9),
            fg="#1f2937",
            wraplength=380,
            justify="center"
        )
        self.lbl_estado.pack(pady=(6, 4))

        # Panel de botones - Fila 1: Gestión de obstáculos
        fila_obs = tk.Frame(contenedor)
        fila_obs.pack(fill="x", pady=(2, 4))

        self.btn_generar_obs = tk.Button(
            fila_obs,
            text="🎲 Obstáculos aleatorios",
            command=self.generar_obstaculos_aleatorios,
            bg="#f3f4f6",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_generar_obs.pack(side="left", expand=True, fill="x", padx=(0, 4))

        self.btn_borrar_obs = tk.Button(
            fila_obs,
            text="🗑️ Borrar obstáculos",
            command=self.borrar_obstaculos,
            bg="#f3f4f6",
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_borrar_obs.pack(side="right", expand=True, fill="x", padx=(4, 0))

        # Panel de botones - Fila 2: Control de movimiento y reseteo
        fila_mov = tk.Frame(contenedor)
        fila_mov.pack(fill="x", pady=(2, 0))

        self.btn_iniciar = tk.Button(
            fila_mov,
            text="▶ Mover punto azul (A*)",
            command=self.iniciar_movimiento,
            bg="#2563eb",
            fg="white",
            font=("Arial", 9, "bold"),
            relief="raised",
            cursor="hand2"
        )
        self.btn_iniciar.pack(side="left", expand=True, fill="x", padx=(0, 4))

        self.btn_reiniciar = tk.Button(
            fila_mov,
            text="↺ Reiniciar",
            command=self.reiniciar_punto,
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_reiniciar.pack(side="right", padx=(4, 0))

    def _habilitar_botones(self, habilitar: bool):
        estado = "normal" if habilitar else "disabled"
        self.btn_iniciar.config(state=estado)
        self.btn_generar_obs.config(state=estado)
        self.btn_borrar_obs.config(state=estado)
        self.btn_reiniciar.config(state=estado)

    def _dibujar_elementos_estaticos(self):
        """Dibuja el punto inicial (rojo) y final (verde)."""
        x0_s, y0_s = self._a_pantalla(*PUNTO_INICIAL)
        x1_s, y1_s = self._a_pantalla(*PUNTO_FINAL)
        r_s = RADIO * ESCALA

        # Punto Inicial (Rojo)
        self.canvas.create_oval(
            x0_s - r_s, y0_s - r_s,
            x0_s + r_s, y0_s + r_s,
            fill="#dc2626",
            outline="#991b1b",
            width=1.5,
            tags="punto_inicial"
        )

        # Punto Final (Verde)
        self.canvas.create_oval(
            x1_s - r_s, y1_s - r_s,
            x1_s + r_s, y1_s + r_s,
            fill="#16a34a",
            outline="#166534",
            width=1.5,
            tags="punto_final"
        )

    def _crear_punto_azul(self):
        """Crea el punto azul en la posición inicial."""
        x0_s, y0_s = self._a_pantalla(*PUNTO_INICIAL)
        r_s = RADIO * ESCALA

        self.punto_azul = self.canvas.create_oval(
            x0_s - r_s, y0_s - r_s,
            x0_s + r_s, y0_s + r_s,
            fill="#2563eb",
            outline="#1e40af",
            width=1.5,
            tags="punto_azul"
        )

    # ==========================================
    # GESTIÓN DE OBSTÁCULOS
    # ==========================================
    def _actualizar_celdas_bloqueadas(self):
        """Calcula el conjunto de celdas bloqueadas incluyendo el margen de seguridad."""
        self.celdas_bloqueadas.clear()
        for x1, y1, x2, y2 in self.obstaculos_rects:
            # Inflar según margen de seguridad para que el radio del punto azul no choque
            min_x = max(0, x1 - MARGEN_SEGURIDAD)
            max_x = min(ANCHO_MAPA - 1, x2 + MARGEN_SEGURIDAD)
            min_y = max(0, y1 - MARGEN_SEGURIDAD)
            max_y = min(ALTO_MAPA - 1, y2 + MARGEN_SEGURIDAD)
            for x in range(min_x, max_x + 1):
                for y in range(min_y, max_y + 1):
                    self.celdas_bloqueadas.add((x, y))

    def _dibujar_obstaculos(self):
        """Dibuja todos los obstáculos en el canvas."""
        self.canvas.delete("obstaculo")
        for x1, y1, x2, y2 in self.obstaculos_rects:
            px1, py1 = self._a_pantalla(x1, y1)
            px2, py2 = self._a_pantalla(x2, y2)
            self.canvas.create_rectangle(
                px1, py1, px2, py2,
                fill="#4b5563",
                outline="#1f2937",
                width=1.5,
                tags="obstaculo"
            )
        # Asegurar que los puntos clave estén por encima de los obstáculos
        self.canvas.tag_raise("camino_a_star")
        self.canvas.tag_raise("punto_inicial")
        self.canvas.tag_raise("punto_final")
        self.canvas.tag_raise("punto_azul")

    def borrar_obstaculos(self):
        """Elimina todos los obstáculos del mapa."""
        if self.animando:
            return
        self.obstaculos_rects.clear()
        self.celdas_bloqueadas.clear()
        self.canvas.delete("obstaculo")
        self.canvas.delete("camino_a_star")
        self.reiniciar_punto()
        self.lbl_estado.config(text="Obstáculos eliminados. El camino es directo.")

    def generar_obstaculos_aleatorios(self):
        """Genera obstáculos rectangulares aleatorios garantizando que exista un camino factible."""
        if self.animando:
            return

        self.borrar_obstaculos()

        radio_seguridad_inicio_fin = RADIO + MARGEN_SEGURIDAD + 6
        intentos_maximos = 15

        for _ in range(intentos_maximos):
            candidatos = []
            cantidad_obstaculos = random.randint(10, 16)

            for _ in range(cantidad_obstaculos):
                w = random.randint(6, 14)
                h = random.randint(6, 14)
                x1 = random.randint(2, ANCHO_MAPA - w - 2)
                y1 = random.randint(2, ALTO_MAPA - h - 2)
                x2 = x1 + w
                y2 = y1 + h

                # Comprobar que no bloquee directamente el inicio ni el final
                dist_inicio = distancia_euclidea(((x1 + x2) / 2, (y1 + y2) / 2), PUNTO_INICIAL)
                dist_fin = distancia_euclidea(((x1 + x2) / 2, (y1 + y2) / 2), PUNTO_FINAL)

                if dist_inicio > radio_seguridad_inicio_fin and dist_fin > radio_seguridad_inicio_fin:
                    candidatos.append((x1, y1, x2, y2))

            # Probar si con estos obstáculos existe un camino con A*
            self.obstaculos_rects = candidatos
            self._actualizar_celdas_bloqueadas()
            camino_prueba = buscar_camino_a_star(PUNTO_INICIAL, PUNTO_FINAL, self.celdas_bloqueadas)

            if camino_prueba is not None:
                break
        else:
            # Si en ningún intento se halló camino libre, reducir a pocos obstáculos seguros
            self.obstaculos_rects = self.obstaculos_rects[:6]
            self._actualizar_celdas_bloqueadas()

        self._dibujar_obstaculos()
        self.lbl_estado.config(
            text=f"Se crearon {len(self.obstaculos_rects)} obstáculos. Presiona 'Mover punto azul (A*)'."
        )

    def _on_canvas_click(self, event):
        """Permite al usuario hacer clic en el mapa para colocar o quitar un obstáculo personalizado."""
        if self.animando:
            return

        # Coordenadas lógicas en [0, 100]
        x_mapa = int(event.x / ESCALA)
        y_mapa = int(event.y / ESCALA)

        # No colocar sobre inicio o fin
        if distancia_euclidea((x_mapa, y_mapa), PUNTO_INICIAL) < 8 or \
           distancia_euclidea((x_mapa, y_mapa), PUNTO_FINAL) < 8:
            return

        # Tamaño del bloque manual: 8x8 px
        w, h = 8, 8
        x1 = max(0, x_mapa - w // 2)
        y1 = max(0, y_mapa - h // 2)
        x2 = min(ANCHO_MAPA, x1 + w)
        y2 = min(ALTO_MAPA, y1 + h)

        # Si ya hace clic dentro de un obstáculo existente, lo borra; si no, lo agrega
        indice_existente = None
        for i, (ox1, oy1, ox2, oy2) in enumerate(self.obstaculos_rects):
            if ox1 <= x_mapa <= ox2 and oy1 <= y_mapa <= oy2:
                indice_existente = i
                break

        if indice_existente is not None:
            self.obstaculos_rects.pop(indice_existente)
            self.lbl_estado.config(text="Obstáculo manual eliminado.")
        else:
            self.obstaculos_rects.append((x1, y1, x2, y2))
            self.lbl_estado.config(text="Obstáculo manual añadido.")

        self.canvas.delete("camino_a_star")
        self._actualizar_celdas_bloqueadas()
        self._dibujar_obstaculos()

    # ==========================================
    # MOVIMIENTO Y ANIMACIÓN
    # ==========================================
    def iniciar_movimiento(self):
        """Calcula el camino más eficaz con A* y anima el punto azul sorteando los obstáculos."""
        if self.animando:
            return

        self._actualizar_celdas_bloqueadas()

        # Calcular camino óptimo mediante A*
        camino_grid = buscar_camino_a_star(PUNTO_INICIAL, PUNTO_FINAL, self.celdas_bloqueadas)

        if not camino_grid:
            self.lbl_estado.config(
                text="⚠️ No existe camino libre entre inicio y fin con los obstáculos actuales."
            )
            return

        # Optimizar camino eliminando zig-zags de la cuadrícula (camino euclídeo más corto)
        waypoints = suavizar_camino(camino_grid, self.celdas_bloqueadas)

        # Dibujar la línea de la trayectoria calculada
        self._dibujar_trayectoria(waypoints)

        # Interpolar puntos a velocidad uniforme (0.8 px por fotograma)
        self.puntos_trayectoria = self._interpolar_puntos(waypoints, paso_px=0.8)
        self.indice_trayectoria = 0

        # Iniciar animación
        self.animando = True
        self._habilitar_botones(False)
        self.lbl_estado.config(
            text=f"Navegando ruta óptima (A*)... {len(waypoints) - 1} tramos directos."
        )
        self._dar_paso_animacion()

    def _dibujar_trayectoria(self, waypoints: list[tuple[int, int]]):
        """Dibuja en el canvas la línea del camino calculado."""
        self.canvas.delete("camino_a_star")
        coords = []
        for x, y in waypoints:
            px, py = self._a_pantalla(x, y)
            coords.extend([px, py])

        if len(coords) >= 4:
            self.canvas.create_line(
                *coords,
                fill="#60a5fa",
                width=2,
                dash=(4, 3),
                tags="camino_a_star",
                capstyle="round",
                joinstyle="round"
            )
        self.canvas.tag_raise("punto_inicial")
        self.canvas.tag_raise("punto_final")
        self.canvas.tag_raise("punto_azul")

    def _interpolar_puntos(self, waypoints: list[tuple[int, int]], paso_px: float = 0.8) -> list[tuple[float, float]]:
        """Genera pasos uniformes a lo largo de los waypoints para una animación continua."""
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

    def _dar_paso_animacion(self):
        """Avanza un paso de la animación del punto azul."""
        if not self.animando:
            return

        if self.indice_trayectoria < len(self.puntos_trayectoria):
            x, y = self.puntos_trayectoria[self.indice_trayectoria]
            x_s, y_s = self._a_pantalla(x, y)
            r_s = RADIO * ESCALA
            self.canvas.coords(
                self.punto_azul,
                x_s - r_s, y_s - r_s,
                x_s + r_s, y_s + r_s
            )
            self.indice_trayectoria += 1
            self.root.after(self.delay_ms, self._dar_paso_animacion)
        else:
            self.animando = False
            self._habilitar_botones(True)
            self.lbl_estado.config(
                text=f"¡Llegó al punto verde con éxito sorteando todos los obstáculos!"
            )

    def reiniciar_punto(self):
        """Devuelve el punto azul a la posición inicial."""
        self.animando = False
        self.indice_trayectoria = 0
        x0_s, y0_s = self._a_pantalla(*PUNTO_INICIAL)
        r_s = RADIO * ESCALA
        self.canvas.coords(
            self.punto_azul,
            x0_s - r_s, y0_s - r_s,
            x0_s + r_s, y0_s + r_s
        )
        self._habilitar_botones(True)
        self.lbl_estado.config(text="Listo. Presiona 'Mover punto azul (A*)'.")


def main():
    root = tk.Tk()
    app = SimuladorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
