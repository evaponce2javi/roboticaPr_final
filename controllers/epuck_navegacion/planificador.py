# -*- coding: utf-8 -*-
"""
planificador.py — A* sobre grilla de ocupación + suavizado (Pasos 3 y 4).

Características exigidas por la Línea A:
  * Conectividad-8 con costo diagonal sqrt(2).
  * Heurística octile (admisible y consistente para 8-conectividad → óptimo).
  * Prevención de corner-cutting: un movimiento diagonal solo es válido si
    AMBAS celdas ortogonales adyacentes están libres.
  * Manejo explícito del caso "no existe ruta" (devuelve None).
  * Suavizado por línea de visión (string-pulling) sobre la grilla inflada.
"""

import heapq
import math

SQRT2 = math.sqrt(2.0)

# (di, dj, costo) — 4 movimientos rectos y 4 diagonales
MOVIMIENTOS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2),
]


def heuristica_octile(a, b):
    """Distancia octile: exacta para 8-conectividad sin obstáculos."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def a_estrella(grilla, inicio, meta):
    """A* clásico sobre la grilla inflada.

    Args:
        grilla: GrillaOcupacion (ya inflada).
        inicio, meta: celdas (i, j).
    Returns:
        Lista de celdas inicio→meta, o None si no existe ruta.
    """
    if not grilla.es_libre(*inicio) or not grilla.es_libre(*meta):
        return None
    if inicio == meta:
        return [inicio]

    abiertos = [(heuristica_octile(inicio, meta), 0.0, inicio)]
    costo_g = {inicio: 0.0}
    padre = {inicio: None}
    cerrados = set()

    while abiertos:
        _, g_actual, actual = heapq.heappop(abiertos)
        if actual in cerrados:
            continue
        if actual == meta:
            # reconstrucción de la ruta
            ruta = []
            nodo = actual
            while nodo is not None:
                ruta.append(nodo)
                nodo = padre[nodo]
            ruta.reverse()
            return ruta
        cerrados.add(actual)

        i, j = actual
        for di, dj, costo in MOVIMIENTOS:
            vecino = (i + di, j + dj)
            if not grilla.es_libre(*vecino):
                continue
            # Prevención de corner-cutting en diagonales
            if di != 0 and dj != 0:
                if not grilla.es_libre(i + di, j) or not grilla.es_libre(i, j + dj):
                    continue
            nuevo_g = g_actual + costo
            if nuevo_g < costo_g.get(vecino, math.inf):
                costo_g[vecino] = nuevo_g
                padre[vecino] = actual
                f = nuevo_g + heuristica_octile(vecino, meta)
                heapq.heappush(abiertos, (f, nuevo_g, vecino))
    return None  # frontera agotada: no existe ruta


def buscar_celda_libre_cercana(grilla, celda, radio_max_celdas=None):
    """BFS desde `celda` hasta la celda libre más cercana (o None).
    Útil cuando la pose inicial o la meta caen dentro del inflado."""
    if grilla.es_libre(*celda):
        return celda
    if radio_max_celdas is None:
        radio_max_celdas = max(grilla.n_cols, grilla.n_filas)
    visitadas = {celda}
    cola = [celda]
    pasos = 0
    while cola and pasos <= radio_max_celdas:
        siguiente = []
        for (i, j) in cola:
            for di, dj, _ in MOVIMIENTOS:
                v = (i + di, j + dj)
                if v in visitadas or not grilla.dentro(*v):
                    continue
                if grilla.es_libre(*v):
                    return v
                visitadas.add(v)
                siguiente.append(v)
        cola = siguiente
        pasos += 1
    return None


def suavizar_ruta(grilla, ruta, paso_frac=0.5):
    """String-pulling: desde cada vértice salta al punto más lejano de la ruta
    con línea de visión libre, eliminando zig-zag y giros innecesarios."""
    if ruta is None or len(ruta) <= 2:
        return list(ruta) if ruta else ruta
    suave = [ruta[0]]
    k = 0
    while k < len(ruta) - 1:
        j = len(ruta) - 1
        while j > k + 1 and not grilla.linea_libre(ruta[k], ruta[j], paso_frac):
            j -= 1
        suave.append(ruta[j])
        k = j
    return suave


def planificar(grilla, inicio_xy, meta_xy, paso_frac=0.5):
    """Pipeline completo de planificación: mundo→celdas, A*, suavizado,
    celdas→waypoints en coordenadas de mundo.

    Returns:
        (waypoints, ruta_mundo) donde:
          waypoints  — lista [(x, y), ...] a seguir (el último es la meta exacta)
          ruta_mundo — la ruta completa de A* en coordenadas de mundo (figuras)
        o None si no existe ruta.
    """
    celda_ini = grilla.mundo_a_grilla(*inicio_xy)
    celda_meta = grilla.mundo_a_grilla(*meta_xy)

    # si inicio/meta caen en zona inflada, usar la celda libre más cercana
    celda_ini = buscar_celda_libre_cercana(grilla, celda_ini)
    celda_meta = buscar_celda_libre_cercana(grilla, celda_meta)
    if celda_ini is None or celda_meta is None:
        return None

    ruta = a_estrella(grilla, celda_ini, celda_meta)
    if ruta is None:
        return None

    suave = suavizar_ruta(grilla, ruta, paso_frac)
    waypoints = [grilla.grilla_a_mundo(i, j) for (i, j) in suave[1:]]
    if waypoints:
        waypoints[-1] = (float(meta_xy[0]), float(meta_xy[1]))  # meta exacta
    else:
        waypoints = [(float(meta_xy[0]), float(meta_xy[1]))]
    ruta_mundo = [grilla.grilla_a_mundo(i, j) for (i, j) in ruta]
    return waypoints, ruta_mundo
