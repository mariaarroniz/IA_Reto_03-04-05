#!/usr/bin/env python3
"""
Simulador de movimiento en línea recta sobre un mapa de 100 px x 100 px.
- Punto inicial: Rojo
- Punto final: Verde
- Punto móvil: Azul
"""

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

# Factor de escala visual para que sea cómodo de ver en pantallas modernas:
# ESCALA = 4 mostrará una ventana con un lienzo de 400x400 px físicos,
# manteniendo la cuadrícula y coordenadas en el rango [0, 100].
# Si deseas que la ventana mida estrictamente 100x100 px en pantalla, cambia ESCALA = 1.
ESCALA = 4

# Coordenadas en el mapa [0, 100]
PUNTO_INICIAL = (15, 15)   # (x, y) - Rojo
PUNTO_FINAL = (85, 85)     # (x, y) - Verde

# Radio del punto (en unidades del mapa)
RADIO = 3


class SimuladorApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Simulador de Movimiento (100x100)")
        self.root.resizable(False, False)

        # Estado de animación
        self.animando = False
        self.paso_actual = 0
        self.total_pasos = 100   # Cantidad de pasos para recorrer la línea
        self.delay_ms = 20       # Milisegundos entre cada paso (~50 FPS)

        self._crear_interfaz()
        self._dibujar_elementos_estaticos()
        self._crear_punto_azul()

    def _a_pantalla(self, x: float, y: float) -> tuple[float, float]:
        """Convierte coordenadas del mapa (0-100) a píxeles de pantalla según la escala."""
        return x * ESCALA, y * ESCALA

    def _crear_interfaz(self):
        # Marco contenedor
        contenedor = tk.Frame(self.root, padx=12, pady=12)
        contenedor.pack()

        # Título / información
        info_label = tk.Label(
            contenedor,
            text=f"Mapa: {ANCHO_MAPA} px x {ALTO_MAPA} px  |  Escala: x{ESCALA}",
            font=("Arial", 10, "bold")
        )
        info_label.pack(pady=(0, 8))

        # Lienzo (Canvas) en blanco
        self.canvas = tk.Canvas(
            contenedor,
            width=ANCHO_MAPA * ESCALA,
            height=ALTO_MAPA * ESCALA,
            bg="white",
            highlightthickness=1,
            highlightbackground="#aaaaaa"
        )
        self.canvas.pack()

        # Etiqueta de estado
        self.lbl_estado = tk.Label(
            contenedor,
            text="Listo. Presiona 'Mover punto azul'.",
            font=("Arial", 9),
            fg="#333333"
        )
        self.lbl_estado.pack(pady=(8, 4))

        # Panel de botones
        panel_botones = tk.Frame(contenedor)
        panel_botones.pack(fill="x", pady=4)

        self.btn_iniciar = tk.Button(
            panel_botones,
            text="▶ Mover punto azul",
            command=self.iniciar_movimiento,
            bg="#2563eb",
            fg="white",
            font=("Arial", 9, "bold"),
            relief="raised",
            cursor="hand2"
        )
        self.btn_iniciar.pack(side="left", expand=True, fill="x", padx=(0, 4))

        self.btn_reiniciar = tk.Button(
            panel_botones,
            text="↺ Reiniciar",
            command=self.reiniciar,
            font=("Arial", 9),
            cursor="hand2"
        )
        self.btn_reiniciar.pack(side="right", padx=(4, 0))

    def _dibujar_elementos_estaticos(self):
        """Dibuja el punto inicial (rojo), final (verde) y la línea guía."""
        x0_s, y0_s = self._a_pantalla(*PUNTO_INICIAL)
        x1_s, y1_s = self._a_pantalla(*PUNTO_FINAL)
        r_s = RADIO * ESCALA

        # Línea guía punteada que une inicio y final
        self.canvas.create_line(
            x0_s, y0_s, x1_s, y1_s,
            fill="#d1d5db",
            dash=(3, 3),
            tags="linea_guia"
        )

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

    def iniciar_movimiento(self):
        """Inicia la animación del punto azul en línea recta."""
        if self.animando:
            return

        # Si ya llegó al final, reinicia antes de comenzar
        if self.paso_actual >= self.total_pasos:
            self.reiniciar()

        self.animando = True
        self.btn_iniciar.config(state="disabled")
        self.lbl_estado.config(text="En movimiento hacia el punto verde...")
        self._dar_paso()

    def _dar_paso(self):
        """Calcula e interpola la posición en línea recta paso a paso."""
        if not self.animando:
            return

        x0, y0 = PUNTO_INICIAL
        x1, y1 = PUNTO_FINAL

        # Fracción t en el intervalo [0.0, 1.0]
        t = self.paso_actual / self.total_pasos

        # Interpolación lineal (línea recta)
        x_actual = x0 + t * (x1 - x0)
        y_actual = y0 + t * (y1 - y0)

        # Actualizar posición en pantalla
        x_s, y_s = self._a_pantalla(x_actual, y_actual)
        r_s = RADIO * ESCALA
        self.canvas.coords(
            self.punto_azul,
            x_s - r_s, y_s - r_s,
            x_s + r_s, y_s + r_s
        )

        if self.paso_actual < self.total_pasos:
            self.paso_actual += 1
            self.root.after(self.delay_ms, self._dar_paso)
        else:
            self.animando = False
            self.btn_iniciar.config(state="normal")
            self.lbl_estado.config(text=f"¡Llegó al punto final! ({x1}, {y1})")

    def reiniciar(self):
        """Devuelve el punto azul al punto de partida."""
        self.animando = False
        self.paso_actual = 0
        x0_s, y0_s = self._a_pantalla(*PUNTO_INICIAL)
        r_s = RADIO * ESCALA
        self.canvas.coords(
            self.punto_azul,
            x0_s - r_s, y0_s - r_s,
            x0_s + r_s, y0_s + r_s
        )
        self.btn_iniciar.config(state="normal")
        self.lbl_estado.config(text="Listo. Presiona 'Mover punto azul'.")


def main():
    root = tk.Tk()
    app = SimuladorApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()

