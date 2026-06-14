# -*- coding: utf-8 -*-
"""
registro.py — Logging a CSV y persistencia de la planificación (Paso 7).

Guarda por paso de simulación todas las señales exigidas por el enunciado
(t, pose odométrica, pose ground-truth si existe, comandos de rueda, lecturas
IR crudas y filtradas, estimación Kalman, estado de la máquina, flag de
casi-colisión y waypoint actual), además de la ruta planificada y las grillas
(original e inflada) para que `analisis/analizar.py` pueda superponer la ruta
planificada con la trayectoria realmente ejecutada.
"""

import csv
import json
import os

import numpy as np


class RegistroCSV:
    """Escritor CSV incremental con columnas fijas."""

    def __init__(self, ruta_archivo, columnas):
        os.makedirs(os.path.dirname(ruta_archivo), exist_ok=True)
        self.columnas = list(columnas)
        self._archivo = open(ruta_archivo, "w", newline="", encoding="utf-8")
        self._escritor = csv.DictWriter(self._archivo, fieldnames=self.columnas)
        self._escritor.writeheader()
        self.ruta_archivo = ruta_archivo

    def registrar(self, valores):
        """Escribe una fila; las columnas ausentes quedan vacías y los floats
        se redondean para mantener archivos compactos."""
        fila = {}
        for col in self.columnas:
            v = valores.get(col, "")
            if isinstance(v, float):
                fila[col] = f"{v:.6f}"
            else:
                fila[col] = v
        self._escritor.writerow(fila)

    def cerrar(self):
        if not self._archivo.closed:
            self._archivo.flush()
            self._archivo.close()


def guardar_ruta_planificada(dir_datos, escenario, waypoints, ruta_mundo):
    """Guarda los waypoints suavizados y la ruta completa de A* (en mundo)."""
    os.makedirs(dir_datos, exist_ok=True)
    ruta_wp = os.path.join(dir_datos, f"ruta_waypoints_{escenario}.csv")
    with open(ruta_wp, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for (x, y) in waypoints:
            w.writerow([f"{x:.6f}", f"{y:.6f}"])
    ruta_celdas = os.path.join(dir_datos, f"ruta_celdas_{escenario}.csv")
    with open(ruta_celdas, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for (x, y) in ruta_mundo:
            w.writerow([f"{x:.6f}", f"{y:.6f}"])


def guardar_grilla(dir_datos, escenario, grilla, grilla_inflada, cfg_escenario,
                   hay_gt):
    """Guarda las grillas (npy) y los metadatos (json) del escenario."""
    os.makedirs(dir_datos, exist_ok=True)
    np.save(os.path.join(dir_datos, f"grilla_{escenario}.npy"), grilla.celdas)
    np.save(os.path.join(dir_datos, f"grilla_inflada_{escenario}.npy"),
            grilla_inflada.celdas)
    meta = {
        "escenario": escenario,
        "limites": cfg_escenario["limites"],
        "resolucion": cfg_escenario["resolucion"],
        "pose_inicial": list(cfg_escenario["pose_inicial"]),
        "meta": list(cfg_escenario["meta"]),
        "hay_ground_truth": bool(hay_gt),
    }
    with open(os.path.join(dir_datos, f"grilla_meta_{escenario}.json"),
              "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
