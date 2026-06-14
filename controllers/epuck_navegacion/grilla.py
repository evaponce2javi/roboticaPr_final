# -*- coding: utf-8 -*-
"""
grilla.py — Grilla de ocupación 2D para planificación (Paso 2 de la metodología).

Representa el entorno como una matriz NumPy (0 = libre, 1 = ocupado) construida
desde la configuración de obstáculos del escenario. Incluye:
  * mapeo mundo→grilla y grilla→mundo (centro de celda),
  * marcado de obstáculos rectangulares y circulares (conservador: cualquier
    celda parcialmente ocupada se marca como ocupada),
  * inflado por dilatación con disco (espacio de configuración),
  * chequeo de línea de visión entre celdas (para el suavizado de rutas).

Convención de índices: una celda es la tupla (i, j) donde i = columna (eje X)
y j = fila (eje Y); la matriz se indexa como celdas[j, i].
"""

import math

import numpy as np


class GrillaOcupacion:
    """Grilla de ocupación rectangular alineada con los ejes del mundo."""

    def __init__(self, limites, resolucion):
        self.x_min = float(limites["x_min"])
        self.x_max = float(limites["x_max"])
        self.y_min = float(limites["y_min"])
        self.y_max = float(limites["y_max"])
        self.resolucion = float(resolucion)
        self.n_cols = int(math.ceil((self.x_max - self.x_min) / self.resolucion))
        self.n_filas = int(math.ceil((self.y_max - self.y_min) / self.resolucion))
        # 0 = libre, 1 = ocupado
        self.celdas = np.zeros((self.n_filas, self.n_cols), dtype=np.uint8)

    # ------------------------------------------------------------------ mapeo
    def mundo_a_grilla(self, x, y):
        """Convierte coordenadas de mundo (m) a celda (i, j), con recorte."""
        i = int(math.floor((x - self.x_min) / self.resolucion))
        j = int(math.floor((y - self.y_min) / self.resolucion))
        i = min(max(i, 0), self.n_cols - 1)
        j = min(max(j, 0), self.n_filas - 1)
        return (i, j)

    def grilla_a_mundo(self, i, j):
        """Devuelve el centro (x, y) en metros de la celda (i, j)."""
        x = self.x_min + (i + 0.5) * self.resolucion
        y = self.y_min + (j + 0.5) * self.resolucion
        return (x, y)

    def dentro(self, i, j):
        return 0 <= i < self.n_cols and 0 <= j < self.n_filas

    def es_libre(self, i, j):
        return self.dentro(i, j) and self.celdas[j, i] == 0

    # ------------------------------------------------------- marcado de mapas
    def marcar_rectangulo(self, x_min, x_max, y_min, y_max):
        """Marca como ocupadas todas las celdas que tocan el rectángulo."""
        i0 = int(math.floor((x_min - self.x_min) / self.resolucion))
        i1 = int(math.ceil((x_max - self.x_min) / self.resolucion))
        j0 = int(math.floor((y_min - self.y_min) / self.resolucion))
        j1 = int(math.ceil((y_max - self.y_min) / self.resolucion))
        i0, i1 = max(i0, 0), min(i1, self.n_cols)
        j0, j1 = max(j0, 0), min(j1, self.n_filas)
        if i0 < i1 and j0 < j1:
            self.celdas[j0:j1, i0:i1] = 1

    def marcar_circulo(self, cx, cy, radio):
        """Marca como ocupadas las celdas cuyo centro cae dentro del círculo
        (expandido media diagonal de celda para ser conservador)."""
        r_efectivo = radio + 0.5 * self.resolucion * math.sqrt(2.0)
        i0, j0 = self.mundo_a_grilla(cx - radio - self.resolucion,
                                     cy - radio - self.resolucion)
        i1, j1 = self.mundo_a_grilla(cx + radio + self.resolucion,
                                     cy + radio + self.resolucion)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                x, y = self.grilla_a_mundo(i, j)
                if (x - cx) ** 2 + (y - cy) ** 2 <= r_efectivo ** 2:
                    self.celdas[j, i] = 1

    def marcar_bordes(self, espesor_m):
        """Marca un marco de celdas ocupadas en el perímetro (paredes)."""
        k = max(1, int(math.ceil(espesor_m / self.resolucion)))
        self.celdas[:k, :] = 1
        self.celdas[-k:, :] = 1
        self.celdas[:, :k] = 1
        self.celdas[:, -k:] = 1

    # ---------------------------------------------------------------- inflado
    def inflar(self, radio_m):
        """Devuelve una NUEVA grilla con los obstáculos dilatados con un disco
        de radio `radio_m` (espacio de configuración): si el centro del robot
        está en una celda libre de la grilla inflada, el robot completo cabe.
        """
        r = int(math.ceil(radio_m / self.resolucion))
        origen = self.celdas.astype(bool)
        resultado = origen.copy()
        alto, ancho = origen.shape
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if di * di + dj * dj > r * r or (di == 0 and dj == 0):
                    continue
                # desplazar `origen` en (dj, di) y acumular con OR
                src_j = slice(max(0, -dj), alto - max(0, dj))
                dst_j = slice(max(0, dj), alto - max(0, -dj))
                src_i = slice(max(0, -di), ancho - max(0, di))
                dst_i = slice(max(0, di), ancho - max(0, -di))
                resultado[dst_j, dst_i] |= origen[src_j, src_i]
        nueva = GrillaOcupacion(
            {"x_min": self.x_min, "x_max": self.x_max,
             "y_min": self.y_min, "y_max": self.y_max},
            self.resolucion)
        nueva.celdas = resultado.astype(np.uint8)
        return nueva

    # ------------------------------------------------------- línea de visión
    def linea_libre(self, celda_a, celda_b, paso_frac=0.5):
        """True si el segmento entre los centros de `celda_a` y `celda_b`
        atraviesa solo celdas libres (muestreo sub-celda, tipo supercover)."""
        xa, ya = self.grilla_a_mundo(*celda_a)
        xb, yb = self.grilla_a_mundo(*celda_b)
        dist = math.hypot(xb - xa, yb - ya)
        if dist < 1e-9:
            return self.es_libre(*celda_a)
        n = max(1, int(math.ceil(dist / (self.resolucion * paso_frac))))
        for k in range(n + 1):
            t = k / n
            x = xa + t * (xb - xa)
            y = ya + t * (yb - ya)
            if not self.es_libre(*self.mundo_a_grilla(x, y)):
                return False
        return True


# =========================================================== fábricas de grilla
def construir_grilla(escenario):
    """Construye la grilla de ocupación (sin inflar) desde un escenario de
    `config.py` (límites, resolución y lista de obstáculos)."""
    grilla = GrillaOcupacion(escenario["limites"], escenario["resolucion"])
    for obs in escenario.get("obstaculos", []):
        if obs["tipo"] == "rect":
            grilla.marcar_rectangulo(obs["x_min"], obs["x_max"],
                                     obs["y_min"], obs["y_max"])
        elif obs["tipo"] == "circulo":
            grilla.marcar_circulo(obs["cx"], obs["cy"], obs["radio"])
        else:
            raise ValueError(f"Tipo de obstáculo desconocido: {obs['tipo']}")
    return grilla


def inflar_para_planificar(grilla, radio_inflado, inflar_bordes=True):
    """Devuelve la grilla inflada (espacio de configuración) lista para A*."""
    inflada = grilla.inflar(radio_inflado)
    if inflar_bordes:
        inflada.marcar_bordes(radio_inflado)
    return inflada
