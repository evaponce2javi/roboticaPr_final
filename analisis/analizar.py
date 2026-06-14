# -*- coding: utf-8 -*-
"""
analizar.py — Análisis experimental (Paso 7 y métricas de la sección 10).

Lee los CSV generados por el controlador (datos/) y produce automáticamente:

  Figuras (figuras/):
    mapa_<escenario>.png     grilla de ocupación + ruta planificada (A* y
                             waypoints suavizados) vs. trayectoria ejecutada
                             (odometría y ground-truth si existe)
    errores_<escenario>.png  distancia a la ruta planificada en el tiempo y,
                             si hay GT, error de posición/orientación odométrico
    senales_<escenario>.png  señales IR crudas vs. filtradas (EMA) y distancia
                             frontal medida vs. estimada por Kalman

  Métricas (datos/metricas_<escenario>.csv + tabla en consola), por corrida:
    éxito, tiempo hasta la meta, longitud planificada y ejecutada, error medio
    y máximo a la ruta, casi-colisiones, giros innecesarios, error odométrico
    (vs. GT), estabilidad (desv. estándar) de señales crudas/filtradas/Kalman,
    y porcentaje de ejecuciones exitosas sobre todas las corridas registradas.

Uso:
    python analisis/analizar.py                  # procesa todos los escenarios
    python analisis/analizar.py --escenario simple
"""

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DIR_RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR_DATOS = os.path.join(DIR_RAIZ, "datos")
DIR_FIGURAS = os.path.join(DIR_RAIZ, "figuras")
sys.path.insert(0, os.path.join(DIR_RAIZ, "controllers", "epuck_navegacion"))

import config as cfg  # noqa: E402  (constantes compartidas con el controlador)

UMBRAL_GIRO = 0.5          # [rad/s] |w| mínimo para considerar un giro activo
SENSORES_FRONTALES = cfg.IDX_IR_FRONTALES


# ============================================================== carga de datos
def cargar_log(ruta):
    """Carga un log CSV como diccionario {columna: np.ndarray}; los valores
    vacíos quedan como NaN y 'estado' se conserva como arreglo de strings."""
    with open(ruta, newline="", encoding="utf-8") as f:
        lector = csv.DictReader(f)
        columnas = {c: [] for c in lector.fieldnames}
        for fila in lector:
            for c in lector.fieldnames:
                columnas[c].append(fila[c])
    datos = {}
    for c, valores in columnas.items():
        if c == "estado":
            datos[c] = np.array(valores, dtype=object)
        else:
            datos[c] = np.array(
                [float(v) if v not in ("", None) else np.nan for v in valores])
    return datos


def cargar_ruta(ruta):
    if not os.path.exists(ruta):
        return None
    puntos = np.loadtxt(ruta, delimiter=",", skiprows=1, ndmin=2)
    return puntos


def trayectoria_ejecutada(datos):
    """Devuelve (puntos, fuente): GT si está disponible, si no odometría."""
    if "x_gt" in datos and np.isfinite(datos["x_gt"]).any():
        return np.column_stack([datos["x_gt"], datos["y_gt"]]), "ground-truth"
    return np.column_stack([datos["x_odom"], datos["y_odom"]]), "odometría"


# ==================================================================== métricas
def longitud_polilinea(puntos):
    if puntos is None or len(puntos) < 2:
        return 0.0
    difer = np.diff(puntos, axis=0)
    return float(np.sum(np.hypot(difer[:, 0], difer[:, 1])))


def distancia_punto_segmento(p, a, b):
    ab = b - a
    norma2 = float(np.dot(ab, ab))
    if norma2 < 1e-12:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab)) / norma2))
    proy = a + t * ab
    return float(np.linalg.norm(p - proy))


def distancias_a_ruta(puntos, ruta):
    """Distancia mínima de cada punto ejecutado a la polilínea planificada."""
    if ruta is None or len(ruta) < 2:
        return np.full(len(puntos), np.nan)
    distancias = np.empty(len(puntos))
    for k, p in enumerate(puntos):
        distancias[k] = min(
            distancia_punto_segmento(p, ruta[s], ruta[s + 1])
            for s in range(len(ruta) - 1))
    return distancias


def contar_eventos(flags):
    """Cuenta transiciones 0→1 (eventos discretos, no pasos)."""
    f = np.nan_to_num(flags, nan=0.0).astype(int)
    if len(f) == 0:
        return 0
    return int(np.sum((f[1:] == 1) & (f[:-1] == 0)) + (f[0] == 1))


def contar_giros_innecesarios(w_cmd, estados):
    """Cambios de signo de w (con |w| > UMBRAL_GIRO) durante FOLLOW_PATH:
    aproxima los zig-zag / correcciones bruscas que una ruta suave no exige."""
    mascara = (estados == "FOLLOW_PATH") & np.isfinite(w_cmd) \
        & (np.abs(w_cmd) > UMBRAL_GIRO)
    signos = np.sign(w_cmd[mascara])
    if len(signos) < 2:
        return 0
    return int(np.sum(signos[1:] != signos[:-1]))


def metricas_corrida(datos, ruta_plan):
    """Calcula todas las métricas de la sección 10 para UNA corrida."""
    m = {}
    estados = datos["estado"]
    exito = bool(np.any(estados == "GOAL_REACHED"))
    m["exito"] = int(exito)
    if exito:
        idx_meta = int(np.argmax(estados == "GOAL_REACHED"))
        m["t_meta_s"] = float(datos["t"][idx_meta])
    else:
        m["t_meta_s"] = np.nan

    ejecutada, fuente = trayectoria_ejecutada(datos)
    m["fuente_trayectoria"] = fuente
    m["long_planificada_m"] = longitud_polilinea(ruta_plan)
    m["long_ejecutada_m"] = longitud_polilinea(ejecutada)

    d_ruta = distancias_a_ruta(ejecutada, ruta_plan)
    m["err_ruta_medio_m"] = float(np.nanmean(d_ruta)) if len(d_ruta) else np.nan
    m["err_ruta_max_m"] = float(np.nanmax(d_ruta)) if len(d_ruta) else np.nan

    m["casi_colisiones"] = contar_eventos(datos["casi_colision"])
    m["giros_innecesarios"] = contar_giros_innecesarios(datos["w_cmd"], estados)

    # error de odometría vs. ground-truth (si existe)
    if np.isfinite(datos["x_gt"]).any():
        err_pos = np.hypot(datos["x_odom"] - datos["x_gt"],
                           datos["y_odom"] - datos["y_gt"])
        err_phi = np.abs(np.arctan2(
            np.sin(datos["phi_odom"] - datos["phi_gt"]),
            np.cos(datos["phi_odom"] - datos["phi_gt"])))
        m["err_odom_medio_m"] = float(np.nanmean(err_pos))
        m["err_odom_max_m"] = float(np.nanmax(err_pos))
        m["err_phi_medio_rad"] = float(np.nanmean(err_phi))
    else:
        m["err_odom_medio_m"] = np.nan
        m["err_odom_max_m"] = np.nan
        m["err_phi_medio_rad"] = np.nan

    # estabilidad de mediciones: desv. estándar del máximo frontal
    crudo = np.max(np.column_stack(
        [datos[f"ps{k}"] for k in SENSORES_FRONTALES]), axis=1)
    filtrado = np.max(np.column_stack(
        [datos[f"ps{k}_f"] for k in SENSORES_FRONTALES]), axis=1)
    m["std_ir_cruda"] = float(np.nanstd(crudo))
    m["std_ir_filtrada"] = float(np.nanstd(filtrado))
    if np.isfinite(datos["dist_kalman"]).any():
        m["std_dist_kalman_m"] = float(np.nanstd(datos["dist_kalman"]))
    else:
        m["std_dist_kalman_m"] = np.nan
    return m


# ===================================================================== figuras
def figura_mapa(escenario, meta, grilla, datos, ruta_celdas, waypoints):
    fig, ax = plt.subplots(figsize=(7, 7))
    lim = meta["limites"]
    extent = [lim["x_min"], lim["x_max"], lim["y_min"], lim["y_max"]]
    ax.imshow(grilla, origin="lower", extent=extent, cmap="Greys",
              vmin=0, vmax=1, alpha=0.85)
    if ruta_celdas is not None:
        ax.plot(ruta_celdas[:, 0], ruta_celdas[:, 1], "c-", lw=1.2,
                label="Ruta A* (celdas)")
    if waypoints is not None:
        ax.plot(waypoints[:, 0], waypoints[:, 1], "b.-", lw=1.8, ms=7,
                label="Waypoints suavizados")
    ax.plot(datos["x_odom"], datos["y_odom"], "r-", lw=1.5,
            label="Trayectoria (odometría)")
    if np.isfinite(datos["x_gt"]).any():
        ax.plot(datos["x_gt"], datos["y_gt"], "g--", lw=1.5,
                label="Trayectoria (ground-truth)")
    x0, y0, _ = meta["pose_inicial"]
    xm, ym = meta["meta"]
    ax.plot(x0, y0, "ks", ms=9, label="Inicio")
    ax.plot(xm, ym, "k*", ms=14, label="Meta")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Escenario '{escenario}': ruta planificada vs. ejecutada")
    ax.legend(loc="best", fontsize=8)
    ax.set_aspect("equal")
    ruta_fig = os.path.join(DIR_FIGURAS, f"mapa_{escenario}.png")
    fig.savefig(ruta_fig, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return ruta_fig


def figura_errores(escenario, datos, ruta_plan):
    hay_gt = np.isfinite(datos["x_gt"]).any()
    n_paneles = 3 if hay_gt else 1
    fig, ejes = plt.subplots(n_paneles, 1, figsize=(8, 3 * n_paneles),
                             sharex=True)
    if n_paneles == 1:
        ejes = [ejes]
    ejecutada, fuente = trayectoria_ejecutada(datos)
    d_ruta = distancias_a_ruta(ejecutada, ruta_plan)
    ejes[0].plot(datos["t"], d_ruta, "b-", lw=1)
    ejes[0].set_ylabel("dist. a la ruta [m]")
    ejes[0].set_title(f"Escenario '{escenario}': desviación respecto a la ruta "
                      f"planificada ({fuente})")
    ejes[0].grid(alpha=0.3)
    if hay_gt:
        err_pos = np.hypot(datos["x_odom"] - datos["x_gt"],
                           datos["y_odom"] - datos["y_gt"])
        err_phi = np.arctan2(np.sin(datos["phi_odom"] - datos["phi_gt"]),
                             np.cos(datos["phi_odom"] - datos["phi_gt"]))
        ejes[1].plot(datos["t"], err_pos, "r-", lw=1)
        ejes[1].set_ylabel("error posición\nodometría [m]")
        ejes[1].grid(alpha=0.3)
        ejes[2].plot(datos["t"], np.degrees(err_phi), "m-", lw=1)
        ejes[2].set_ylabel("error orientación\nodometría [°]")
        ejes[2].grid(alpha=0.3)
    else:
        print(f"  [AVISO] '{escenario}': sin ground-truth — se omite el error "
              "odométrico (añade GPS+Compass al e-puck para medirlo).")
    ejes[-1].set_xlabel("t [s]")
    ruta_fig = os.path.join(DIR_FIGURAS, f"errores_{escenario}.png")
    fig.savefig(ruta_fig, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return ruta_fig


def figura_senales(escenario, datos):
    fig, (eje1, eje2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    crudo = np.max(np.column_stack(
        [datos[f"ps{k}"] for k in SENSORES_FRONTALES]), axis=1)
    filtrado = np.max(np.column_stack(
        [datos[f"ps{k}_f"] for k in SENSORES_FRONTALES]), axis=1)
    eje1.plot(datos["t"], crudo, color="0.6", lw=0.8, label="IR frontal cruda")
    eje1.plot(datos["t"], filtrado, "b-", lw=1.2, label="IR frontal filtrada (EMA)")
    eje1.set_ylabel("lectura IR")
    eje1.set_title(f"Escenario '{escenario}': señales crudas vs. filtradas y "
                   "estimación Kalman")
    eje1.legend(fontsize=8)
    eje1.grid(alpha=0.3)
    eje2.plot(datos["t"], datos["dist_ir"], "c.", ms=2,
              label="distancia medida (IR linealizado)")
    eje2.plot(datos["t"], datos["dist_kalman"], "k-", lw=1.2,
              label="distancia estimada (Kalman encoder–IR)")
    eje2.set_ylabel("distancia frontal [m]")
    eje2.set_xlabel("t [s]")
    eje2.legend(fontsize=8)
    eje2.grid(alpha=0.3)
    ruta_fig = os.path.join(DIR_FIGURAS, f"senales_{escenario}.png")
    fig.savefig(ruta_fig, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return ruta_fig


# ======================================================================== main
def procesar_escenario(escenario):
    print(f"\n=== Escenario: {escenario} ===")
    ruta_meta = os.path.join(DIR_DATOS, f"grilla_meta_{escenario}.json")
    logs = sorted(glob.glob(os.path.join(DIR_DATOS, f"log_{escenario}_*.csv")))
    if not os.path.exists(ruta_meta) or not logs:
        print("  Sin datos (ejecuta primero la simulación en Webots).")
        return

    with open(ruta_meta, encoding="utf-8") as f:
        meta = json.load(f)
    grilla = np.load(os.path.join(DIR_DATOS, f"grilla_{escenario}.npy"))
    waypoints = cargar_ruta(
        os.path.join(DIR_DATOS, f"ruta_waypoints_{escenario}.csv"))
    ruta_celdas = cargar_ruta(
        os.path.join(DIR_DATOS, f"ruta_celdas_{escenario}.csv"))
    # polilínea planificada de referencia: inicio + waypoints suavizados
    inicio = np.array(meta["pose_inicial"][:2])
    ruta_plan = (np.vstack([inicio, waypoints])
                 if waypoints is not None else None)

    os.makedirs(DIR_FIGURAS, exist_ok=True)
    filas = []
    for ruta_log in logs:
        datos = cargar_log(ruta_log)
        m = metricas_corrida(datos, ruta_plan)
        m["log"] = os.path.basename(ruta_log)
        filas.append(m)

    # figuras con la corrida más reciente
    datos_ultimo = cargar_log(logs[-1])
    fig1 = figura_mapa(escenario, meta, grilla, datos_ultimo,
                       ruta_celdas, waypoints)
    fig2 = figura_errores(escenario, datos_ultimo, ruta_plan)
    fig3 = figura_senales(escenario, datos_ultimo)
    print(f"  Figuras: {fig1}\n           {fig2}\n           {fig3}")

    # tabla de métricas (todas las corridas) + resumen
    columnas = ["log", "exito", "t_meta_s", "long_planificada_m",
                "long_ejecutada_m", "err_ruta_medio_m", "err_ruta_max_m",
                "casi_colisiones", "giros_innecesarios", "err_odom_medio_m",
                "err_odom_max_m", "err_phi_medio_rad", "std_ir_cruda",
                "std_ir_filtrada", "std_dist_kalman_m", "fuente_trayectoria"]
    ruta_metricas = os.path.join(DIR_DATOS, f"metricas_{escenario}.csv")
    with open(ruta_metricas, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=columnas)
        w.writeheader()
        for fila in filas:
            w.writerow({c: fila.get(c, "") for c in columnas})
    print(f"  Métricas por corrida: {ruta_metricas}")

    exitos = sum(f["exito"] for f in filas)
    print(f"  Corridas: {len(filas)} | exitosas: {exitos} "
          f"({100.0 * exitos / len(filas):.1f} %)")
    ultima = filas[-1]
    print("  Última corrida:")
    for clave in columnas[1:]:
        print(f"    {clave:22s} = {ultima.get(clave)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--escenario", default=None,
                        help="procesar solo un escenario (p. ej. 'simple')")
    args = parser.parse_args()

    if args.escenario:
        escenarios = [args.escenario]
    else:
        metas = glob.glob(os.path.join(DIR_DATOS, "grilla_meta_*.json"))
        escenarios = sorted(os.path.basename(m)[len("grilla_meta_"):-len(".json")]
                            for m in metas)
    if not escenarios:
        print("No hay datos en datos/. Ejecuta primero la simulación en Webots.")
        return
    for esc in escenarios:
        procesar_escenario(esc)


if __name__ == "__main__":
    main()
