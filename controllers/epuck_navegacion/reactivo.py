# -*- coding: utf-8 -*-
"""
reactivo.py — Capa reactiva de evitación con histéresis (Paso 6, Lab 2).

Define los estados de la arquitectura híbrida y la lógica reactiva que
interrumpe el seguimiento de la ruta cuando los IR frontales detectan un
obstáculo. La HISTÉRESIS (umbral de entrada > umbral de salida) evita
oscilaciones FOLLOW↔AVOID, exactamente como la máquina ADVANCE/TURN del
Laboratorio 2.

Niveles de respuesta según el riesgo:
  * frontal_max > IR_UMBRAL_STOP      → detenerse (v = 0) y girar en el lugar.
  * frontal_max > IR_UMBRAL_AVOID_ON  → evadir: avance lento + giro hacia el
                                        lado más despejado (regla Braitenberg).
  * bloqueo prolongado (contador)     → señal de re-planificación al nivel
                                        deliberativo.
"""

from enum import Enum


class Estado(Enum):
    """Estados de la máquina del controlador principal."""
    PLAN = "PLAN"
    FOLLOW_PATH = "FOLLOW_PATH"
    AVOID = "AVOID"
    GOAL_REACHED = "GOAL_REACHED"
    SIN_RUTA = "SIN_RUTA"


class CapaReactiva:
    """Evaluación de riesgo con histéresis + comando reactivo de evasión."""

    def __init__(self, umbral_on, umbral_off, umbral_stop, umbral_casi_colision,
                 idx_frontales, idx_izquierda, idx_derecha,
                 v_evasion, w_evasion):
        if umbral_off >= umbral_on:
            raise ValueError("Histéresis inválida: umbral_off debe ser < umbral_on")
        self.umbral_on = umbral_on
        self.umbral_off = umbral_off
        self.umbral_stop = umbral_stop
        self.umbral_casi_colision = umbral_casi_colision
        self.idx_frontales = idx_frontales
        self.idx_izquierda = idx_izquierda
        self.idx_derecha = idx_derecha
        self.v_evasion = v_evasion
        self.w_evasion = w_evasion
        self.en_evasion = False
        self.pasos_en_evasion = 0

    def evaluar(self, ir_filtrados):
        """Actualiza el estado de evasión con histéresis.

        Args:
            ir_filtrados: lista de 8 lecturas filtradas (EMA) ps0..ps7.
        Returns:
            (en_evasion, casi_colision): bools para la máquina de estados y
            para el flag de la métrica de casi-colisiones.
        """
        frontal_max = max(ir_filtrados[k] for k in self.idx_frontales)
        if not self.en_evasion and frontal_max > self.umbral_on:
            self.en_evasion = True
            self.pasos_en_evasion = 0
        elif self.en_evasion and frontal_max < self.umbral_off:
            self.en_evasion = False
            self.pasos_en_evasion = 0
        if self.en_evasion:
            self.pasos_en_evasion += 1
        casi_colision = max(ir_filtrados) > self.umbral_casi_colision
        return self.en_evasion, casi_colision

    def comando_evasion(self, ir_filtrados):
        """Comando reactivo (v, w): girar hacia el lado más despejado.

        Si la lectura frontal supera el umbral de parada, el avance se anula
        (v = 0) y el robot solo gira (riesgo inminente de colisión).
        """
        lectura_izq = max(ir_filtrados[k] for k in self.idx_izquierda)
        lectura_der = max(ir_filtrados[k] for k in self.idx_derecha)
        frontal_max = max(ir_filtrados[k] for k in self.idx_frontales)

        v = 0.0 if frontal_max > self.umbral_stop else self.v_evasion
        # obstáculo más a la izquierda → girar a la derecha (w < 0) y viceversa
        w = -self.w_evasion if lectura_izq >= lectura_der else self.w_evasion
        return v, w

    def bloqueado(self, limite_pasos):
        """True si lleva demasiados pasos seguidos en evasión (re-planificar)."""
        return self.pasos_en_evasion >= limite_pasos

    def reiniciar(self):
        self.en_evasion = False
        self.pasos_en_evasion = 0
