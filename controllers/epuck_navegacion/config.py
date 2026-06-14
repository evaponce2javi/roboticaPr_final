# -*- coding: utf-8 -*-
"""
config.py — Configuración central del proyecto (Línea A: Planificación de rutas).

ESTE ES EL ÚNICO ARCHIVO QUE EL USUARIO DEBE EDITAR para adaptar el sistema a
sus mundos de Webots (.wbt). Toda constante del sistema vive aquí: no hay
"números mágicos" en el resto del código.

Sistema de coordenadas (SUPUESTO DOCUMENTADO):
    Se asume Webots moderno (R2022a o posterior) con coordenadas ENU:
    plano del piso = X–Y, eje Z hacia arriba, y norte de la brújula = +X.
    Si tu mundo usa la convención antigua NUE (piso X–Z, Y arriba) debes
    migrar el mundo a ENU (recomendado) o adaptar `leer_pose_gt` en
    `epuck_navegacion.py` y el ajuste `CORRECCION_BRUJULA` más abajo.
"""

import math
import os

# =============================================================================
# 1. PARÁMETROS FÍSICOS DEL E-PUCK
#    (valores nominales del PROTO de Webots; verificar contra la documentación)
# =============================================================================
RADIO_RUEDA = 0.0205        # [m] radio de rueda r
DIST_ENTRE_RUEDAS = 0.052   # [m] distancia entre ruedas L (eje)
VEL_MAX_MOTOR = 6.28        # [rad/s] velocidad angular máxima de cada motor
RADIO_ROBOT = 0.035         # [m] radio del cuerpo del e-puck

# Nombres de dispositivos en el PROTO e-puck
MOTOR_IZQ = "left wheel motor"
MOTOR_DER = "right wheel motor"
ENCODER_IZQ = "left wheel sensor"
ENCODER_DER = "right wheel sensor"
SENSORES_IR = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
NOMBRE_GPS = "gps"          # añadir un GPS llamado "gps" en turretSlot (opcional)
NOMBRE_BRUJULA = "compass"  # añadir un Compass llamado "compass" en turretSlot (opcional)

# =============================================================================
# 2. ESCENARIOS  (EDITAR PARA REFLEJAR TUS MUNDOS .wbt)
#    Cada escenario describe: límites del arena, resolución de la grilla,
#    pose inicial (x, y, phi), meta (x, y) y lista de obstáculos en
#    coordenadas de MUNDO. Los obstáculos pueden ser:
#       {"tipo": "rect",    "x_min":..., "x_max":..., "y_min":..., "y_max":...}
#       {"tipo": "circulo", "cx":..., "cy":..., "radio":...}
#    IMPORTANTE: la pose inicial debe coincidir con los campos translation y
#    rotation del e-puck en el .wbt (phi = 0 → robot mirando hacia +X con
#    rotation 0 0 1 0).
# =============================================================================
ESCENARIO_ACTIVO = "simple"   # <- cambiar a "complejo" para el segundo mundo

ESCENARIOS = {
    # ---- Escenario 1: pocos obstáculos, ruta relativamente directa ----------
    # Ejemplo pensado para una RectangleArena de 1 m x 1 m centrada en (0,0),
    # una caja alargada en el centro y un obstáculo cilíndrico.
    "simple": {
        "limites": {"x_min": -0.5, "x_max": 0.5, "y_min": -0.5, "y_max": 0.5},
        "resolucion": 0.02,                 # [m] tamaño de celda
        "pose_inicial": (-0.40, -0.40, 0.0),
        "meta": (0.40, 0.40),
        "inflar_bordes": True,              # tratar las paredes del arena como obstáculo
        "obstaculos": [
            {"tipo": "rect", "x_min": -0.05, "x_max": 0.05,
             "y_min": -0.30, "y_max": 0.30},
            {"tipo": "circulo", "cx": 0.25, "cy": -0.10, "radio": 0.06},
        ],
    },
    # ---- Escenario 2: pasillos angostos y rutas alternativas ----------------
    "complejo": {
        "limites": {"x_min": -0.5, "x_max": 0.5, "y_min": -0.5, "y_max": 0.5},
        "resolucion": 0.02,
        "pose_inicial": (-0.42, -0.42, 0.0),
        "meta": (0.42, 0.42),
        "inflar_bordes": True,
        "obstaculos": [
            # muro vertical que nace en la pared inferior (paso por arriba)
            {"tipo": "rect", "x_min": -0.30, "x_max": -0.20,
             "y_min": -0.50, "y_max": 0.15},
            # muro vertical que nace en la pared superior (paso por abajo)
            {"tipo": "rect", "x_min": 0.00, "x_max": 0.10,
             "y_min": -0.15, "y_max": 0.50},
            # muro horizontal adosado a la pared derecha
            {"tipo": "rect", "x_min": 0.30, "x_max": 0.50,
             "y_min": -0.20, "y_max": -0.10},
            {"tipo": "circulo", "cx": -0.05, "cy": -0.35, "radio": 0.05},
            {"tipo": "circulo", "cx": 0.35, "cy": 0.20, "radio": 0.05},
        ],
    },
}

# =============================================================================
# 3. PLANIFICACIÓN (grilla + A*)
# =============================================================================
MARGEN_SEGURIDAD = 0.015                        # [m] holgura extra sobre el radio
RADIO_INFLADO = RADIO_ROBOT + MARGEN_SEGURIDAD  # [m] inflado total (esp. de config.)
PASO_MUESTREO_LDV = 0.5    # fracción de celda usada al muestrear la línea de visión

# =============================================================================
# 4. CONTROL DE SEGUIMIENTO DE WAYPOINTS (Lab 1: cinemática diferencial)
# =============================================================================
KP_ANGULAR = 4.0           # ganancia proporcional sobre el error de orientación
KP_LINEAL = 1.5            # ganancia proporcional sobre la distancia al waypoint
V_MAX = 0.08               # [m/s] velocidad lineal máxima de avance
W_MAX = 4.0                # [rad/s] velocidad angular máxima comandada
UMBRAL_WAYPOINT = 0.05     # [m] distancia para considerar alcanzado un waypoint
UMBRAL_META = 0.04         # [m] distancia para declarar GOAL_REACHED

# =============================================================================
# 5. CAPA REACTIVA (Lab 2: navegación reactiva con histéresis)
#    Lecturas IR del e-puck: ~0–4096, mayor = obstáculo más cerca, no lineal.
# =============================================================================
IR_UMBRAL_AVOID_ON = 140.0     # entra a AVOID si el frontal máx. supera esto
IR_UMBRAL_AVOID_OFF = 90.0     # sale de AVOID si el frontal máx. baja de esto
IR_UMBRAL_STOP = 1000.0        # riesgo de colisión: v = 0, solo girar
IR_UMBRAL_CASI_COLISION = 400.0  # umbral del flag de casi-colisión (métrica)
IR_PISO_RUIDO = 80.0           # bajo esto la lectura se considera ruido
V_EVASION = 0.02               # [m/s] avance lento durante la evasión
W_EVASION = 2.0                # [rad/s] giro reactivo
PASOS_BLOQUEO_REPLAN = 150     # pasos seguidos en AVOID antes de re-planificar
RADIO_OBSTACULO_DETECTADO = 0.05  # [m] radio del obstáculo marcado al re-planificar

IDX_IR_FRONTALES = [0, 1, 6, 7]   # ps0, ps7 frente; ps1, ps6 diagonales
IDX_IR_DERECHA = [0, 1, 2]
IDX_IR_IZQUIERDA = [5, 6, 7]

# =============================================================================
# 6. FILTRADO Y FUSIÓN (Lab 2: filtrado simple + filtro de Kalman 1D)
# =============================================================================
EMA_ALPHA = 0.35           # filtro exponencial sobre cada sensor IR
KALMAN_P0 = 1e-2           # covarianza inicial del estado (distancia frontal)
KALMAN_Q = 1e-5            # ruido de proceso (incertidumbre de la odometría)
KALMAN_R = 4e-4            # ruido de medición (~(2 cm)^2 del IR linealizado)
DIST_IR_MAX = 0.07         # [m] alcance útil aproximado del IR

# Tabla aproximada valor IR -> distancia [m] para linealizar la medición.
# CALIBRAR contra la lookupTable del PROTO e-puck de tu versión de Webots.
TABLA_IR = [
    (4095.0, 0.000), (2211.0, 0.005), (1465.0, 0.010), (676.0, 0.015),
    (383.0, 0.020), (234.0, 0.030), (158.0, 0.040), (120.0, 0.050),
    (104.0, 0.060), (67.0, 0.070),
]

# =============================================================================
# 7. GROUND-TRUTH
# =============================================================================
USAR_GT_PARA_CONTROL = False   # False: la navegación usa SOLO odometría (el GT
                               # queda para registro/análisis). True: usa GT.
CORRECCION_BRUJULA = 0.0       # [rad] offset si el norte del mundo no es +X

# =============================================================================
# 8. RUTAS DE ARCHIVOS Y VARIOS
# =============================================================================
_DIR_CONTROLADOR = os.path.dirname(os.path.abspath(__file__))
DIR_RAIZ = os.path.abspath(os.path.join(_DIR_CONTROLADOR, "..", ".."))
DIR_DATOS = os.path.join(DIR_RAIZ, "datos")
DIR_FIGURAS = os.path.join(DIR_RAIZ, "figuras")

PASOS_EXTRA_TRAS_META = 15     # pasos extra registrados tras llegar/detenerse


def obtener_escenario(nombre=None):
    """Devuelve el diccionario del escenario activo (o del nombre indicado)."""
    clave = nombre if nombre is not None else ESCENARIO_ACTIVO
    if clave not in ESCENARIOS:
        raise KeyError(
            f"Escenario '{clave}' no definido en config.ESCENARIOS "
            f"(disponibles: {list(ESCENARIOS)})")
    return ESCENARIOS[clave]
