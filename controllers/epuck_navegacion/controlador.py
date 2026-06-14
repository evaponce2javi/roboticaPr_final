# -*- coding: utf-8 -*-
"""
controlador.py — Seguimiento de waypoints con cinemática diferencial (Paso 5).

Reutiliza directamente el control cinemático del LABORATORIO 1:
  v = (v_r + v_l)/2 ,  w = (v_r - v_l)/L
y su inversa para convertir el comando (v, w) en velocidades de rueda:
  v_l = v - w·L/2 ;  v_r = v + w·L/2 ;  motor = v_rueda / r   (saturado)

Ley de control proporcional "gira primero, avanza después":
  e_ang = normaliza(atan2(wy - y, wx - x) - phi)
  w     = sat(Kp_ang · e_ang, ±W_MAX)
  v     = sat(Kp_lin · distancia, V_MAX) · max(0, cos(e_ang))
El factor cos(e_ang) reduce suavemente la velocidad lineal cuando el error
angular es grande y la anula cuando |e_ang| ≥ π/2 (giro en el lugar).
"""

import math


def normalizar_angulo(angulo):
    """Normaliza un ángulo al intervalo (-pi, pi]."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


class ControladorWaypoints:
    """Controlador proporcional de seguimiento de waypoints (robot diferencial)."""

    def __init__(self, kp_angular, kp_lineal, v_max, w_max,
                 radio_rueda, dist_ruedas, vel_max_motor):
        self.kp_angular = kp_angular
        self.kp_lineal = kp_lineal
        self.v_max = v_max
        self.w_max = w_max
        self.radio_rueda = radio_rueda
        self.dist_ruedas = dist_ruedas
        self.vel_max_motor = vel_max_motor

    def calcular(self, pose, objetivo):
        """Calcula el comando (v, w) hacia el waypoint `objetivo`.

        Args:
            pose: (x, y, phi) estimada (odometría o ground-truth).
            objetivo: (wx, wy) en coordenadas de mundo.
        Returns:
            (v, w, distancia, error_angular)
        """
        x, y, phi = pose
        wx, wy = objetivo
        distancia = math.hypot(wx - x, wy - y)
        heading_deseado = math.atan2(wy - y, wx - x)
        error_angular = normalizar_angulo(heading_deseado - phi)

        w = max(-self.w_max, min(self.w_max, self.kp_angular * error_angular))
        v_nominal = min(self.kp_lineal * distancia, self.v_max)
        factor_giro = max(0.0, math.cos(error_angular))  # frena si gira mucho
        v = v_nominal * factor_giro
        return v, w, distancia, error_angular

    def a_velocidades_rueda(self, v, w):
        """Convierte (v, w) a velocidades angulares de motor [rad/s].

        La saturación es PROPORCIONAL (escala ambas ruedas por igual) para
        preservar la curvatura comandada en lugar de recortar una sola rueda.
        """
        v_l = v - w * self.dist_ruedas / 2.0
        v_r = v + w * self.dist_ruedas / 2.0
        w_l = v_l / self.radio_rueda
        w_r = v_r / self.radio_rueda
        mayor = max(abs(w_l), abs(w_r))
        if mayor > self.vel_max_motor:
            escala = self.vel_max_motor / mayor
            w_l *= escala
            w_r *= escala
        return w_l, w_r
