# -*- coding: utf-8 -*-
"""
filtro.py — Filtrado simple y filtro de Kalman 1D (Paso 6, herencia del Lab 2).

Dos capas de filtrado, como en el Laboratorio 2:
  1. FiltroEMA: filtro exponencial (media móvil exponencial) aplicado a cada
     uno de los 8 sensores IR para atenuar el ruido de alta frecuencia que
     alimenta la capa reactiva. Se registran señal cruda y filtrada.
  2. Kalman1D: filtro de Kalman escalar que FUSIONA encoder e IR para estimar
     la distancia d al obstáculo frontal:
        predicción : d_k|k-1 = d_{k-1} − Δs        (Δs viene de la odometría)
                     P_k|k-1 = P_{k-1} + Q
        corrección : K = P/(P+R) ;  d_k = d + K·(z − d) ;  P_k = (1−K)·P
     donde z es la lectura IR frontal linealizada con la tabla de calibración.
     Nota: el modelo de predicción asume avance aproximadamente frontal hacia
     el obstáculo; es una simplificación documentada, válida en FOLLOW_PATH.
"""

import numpy as np


class FiltroEMA:
    """Filtro exponencial: y_k = alpha·z_k + (1−alpha)·y_{k−1}."""

    def __init__(self, alpha):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha debe estar en (0, 1]")
        self.alpha = alpha
        self.valor = None

    def actualizar(self, medicion):
        if self.valor is None:
            self.valor = float(medicion)
        else:
            self.valor = self.alpha * float(medicion) + (1.0 - self.alpha) * self.valor
        return self.valor


class Kalman1D:
    """Filtro de Kalman escalar con modelo x_k = x_{k−1} + u_k + ruido."""

    def __init__(self, x_inicial, p_inicial, q_proceso, r_medicion):
        self.x = float(x_inicial)
        self.p = float(p_inicial)
        self.q = float(q_proceso)
        self.r = float(r_medicion)

    def predecir(self, u=0.0):
        """Etapa de predicción con entrada de control u (p. ej. −Δs)."""
        self.x += u
        self.p += self.q
        return self.x

    def corregir(self, z):
        """Etapa de corrección con la medición z."""
        k = self.p / (self.p + self.r)
        self.x += k * (float(z) - self.x)
        self.p = (1.0 - k) * self.p
        return self.x

    def reiniciar(self, x_inicial, p_inicial):
        self.x = float(x_inicial)
        self.p = float(p_inicial)


def ir_a_distancia(valor_ir, tabla, dist_max):
    """Linealiza una lectura IR del e-puck a distancia [m] interpolando la
    tabla de calibración (valor decreciente con la distancia).

    Args:
        valor_ir: lectura (cruda o filtrada) del sensor de proximidad.
        tabla: lista [(valor, distancia_m), ...] ordenada por valor decreciente.
        dist_max: distancia devuelta cuando la señal está bajo la tabla.
    """
    valores = np.array([fila[0] for fila in tabla], dtype=float)
    distancias = np.array([fila[1] for fila in tabla], dtype=float)
    orden = np.argsort(valores)            # np.interp exige x creciente
    valores = valores[orden]
    distancias = distancias[orden]
    if valor_ir <= valores[0]:
        return float(dist_max)
    if valor_ir >= valores[-1]:
        return float(distancias[-1])
    return float(np.interp(valor_ir, valores, distancias))
