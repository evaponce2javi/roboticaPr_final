# -*- coding: utf-8 -*-
"""
odometria.py — Estimación de pose por encoders (Paso 6, modelo de la sección 7).

Ecuaciones implementadas (idénticas a las del enunciado / Laboratorio 2):
    Δs_r = r·Δθ_r ,  Δs_l = r·Δθ_l
    Δs   = (Δs_r + Δs_l)/2
    Δφ   = (Δs_r − Δs_l)/L
    x_k  = x_{k−1} + Δs·cos(φ_{k−1} + Δφ/2)
    y_k  = y_{k−1} + Δs·sin(φ_{k−1} + Δφ/2)
    φ_k  = normaliza(φ_{k−1} + Δφ)        # (−π, π]
"""

import math

from controlador import normalizar_angulo


class Odometria:
    """Integra los encoders de rueda para estimar la pose (x, y, phi)."""

    def __init__(self, pose_inicial, radio_rueda, dist_ruedas):
        self.x, self.y, self.phi = (float(pose_inicial[0]),
                                    float(pose_inicial[1]),
                                    float(pose_inicial[2]))
        self.radio_rueda = radio_rueda
        self.dist_ruedas = dist_ruedas
        self._prev_izq = None
        self._prev_der = None
        self.ultimo_ds = 0.0   # último avance lineal Δs (usado por el Kalman)
        self.ultimo_dphi = 0.0

    @property
    def pose(self):
        return (self.x, self.y, self.phi)

    def actualizar(self, angulo_izq, angulo_der):
        """Actualiza la pose con las lecturas absolutas de los encoders [rad].

        Maneja con gracia la primera lectura y posibles NaN iniciales de los
        sensores de posición de Webots (antes del primer paso de simulación).
        """
        if angulo_izq is None or angulo_der is None or \
                math.isnan(angulo_izq) or math.isnan(angulo_der):
            self.ultimo_ds = 0.0
            self.ultimo_dphi = 0.0
            return self.pose
        if self._prev_izq is None:
            self._prev_izq = angulo_izq
            self._prev_der = angulo_der
            self.ultimo_ds = 0.0
            self.ultimo_dphi = 0.0
            return self.pose

        delta_izq = angulo_izq - self._prev_izq
        delta_der = angulo_der - self._prev_der
        self._prev_izq = angulo_izq
        self._prev_der = angulo_der

        ds_l = self.radio_rueda * delta_izq
        ds_r = self.radio_rueda * delta_der
        ds = (ds_r + ds_l) / 2.0
        dphi = (ds_r - ds_l) / self.dist_ruedas

        # integración con el ángulo medio (mejor aproximación del arco)
        self.x += ds * math.cos(self.phi + dphi / 2.0)
        self.y += ds * math.sin(self.phi + dphi / 2.0)
        self.phi = normalizar_angulo(self.phi + dphi)

        self.ultimo_ds = ds
        self.ultimo_dphi = dphi
        return self.pose
