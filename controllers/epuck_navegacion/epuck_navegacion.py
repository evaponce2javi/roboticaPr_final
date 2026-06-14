# -*- coding: utf-8 -*-
"""
epuck_navegacion.py — Controlador principal de Webots (Línea A: Planificación).

Arquitectura híbrida deliberativa + reactiva:

  PLAN          construir grilla → inflar → A* → suavizar → waypoints
  FOLLOW_PATH   seguimiento de waypoints con control cinemático diferencial
                (Lab 1) y pose estimada por odometría de encoders (Lab 2)
  AVOID         capa reactiva prioritaria con IR + histéresis (Lab 2);
                ante bloqueo prolongado marca el obstáculo estimado en la
                grilla y RE-PLANIFICA desde la pose actual
  GOAL_REACHED  motores detenidos al llegar a la meta
  SIN_RUTA      no existe ruta factible: detención con gracia

Ground-truth: si el e-puck tiene GPS ("gps") y Compass ("compass") en su
turretSlot, la pose real se registra junto a la odométrica para el análisis.
Si no existen, el sistema degrada con gracia a odometría pura (queda avisado
en consola y con columnas GT vacías en el CSV).
"""

import datetime
import math
import os

from controller import Robot  # API de Webots (disponible solo dentro de Webots)

import config as cfg
import registro
from controlador import ControladorWaypoints, normalizar_angulo
from filtro import FiltroEMA, Kalman1D, ir_a_distancia
from grilla import construir_grilla, inflar_para_planificar
from odometria import Odometria
from planificador import planificar
from reactivo import CapaReactiva, Estado

# Columnas del log CSV (Paso 7 de la metodología)
COLUMNAS_LOG = (
    ["t", "estado", "wp_idx",
     "x_odom", "y_odom", "phi_odom",
     "x_gt", "y_gt", "phi_gt",
     "v_cmd", "w_cmd", "vl_motor", "vr_motor"]
    + [f"ps{k}" for k in range(8)]
    + [f"ps{k}_f" for k in range(8)]
    + ["dist_ir", "dist_kalman", "casi_colision"]
)


def inicializar_dispositivos(robot, timestep):
    """Obtiene y habilita motores, encoders, IR y (si existen) GPS/Compass."""
    motor_izq = robot.getDevice(cfg.MOTOR_IZQ)
    motor_der = robot.getDevice(cfg.MOTOR_DER)
    # modo velocidad: posición infinita + velocidad inicial nula
    motor_izq.setPosition(float("inf"))
    motor_der.setPosition(float("inf"))
    motor_izq.setVelocity(0.0)
    motor_der.setVelocity(0.0)

    enc_izq = robot.getDevice(cfg.ENCODER_IZQ)
    enc_der = robot.getDevice(cfg.ENCODER_DER)
    enc_izq.enable(timestep)
    enc_der.enable(timestep)

    sensores_ir = []
    for nombre in cfg.SENSORES_IR:
        sensor = robot.getDevice(nombre)
        sensor.enable(timestep)
        sensores_ir.append(sensor)

    # Ground-truth opcional (degradación con gracia si no está)
    gps, brujula = None, None
    try:
        gps = robot.getDevice(cfg.NOMBRE_GPS)
        if gps is not None:
            gps.enable(timestep)
    except Exception:
        gps = None
    try:
        brujula = robot.getDevice(cfg.NOMBRE_BRUJULA)
        if brujula is not None:
            brujula.enable(timestep)
    except Exception:
        brujula = None

    return {"motor_izq": motor_izq, "motor_der": motor_der,
            "enc_izq": enc_izq, "enc_der": enc_der,
            "ir": sensores_ir, "gps": gps, "brujula": brujula}


def leer_pose_gt(gps, brujula):
    """Pose ground-truth (x, y, phi) con GPS + Compass en mundo ENU.

    Supuesto: norte de la brújula = +X del mundo. El Compass entrega el vector
    norte expresado en el marco del robot; con norte = +X, la orientación es
    phi = −atan2(c_y, c_x). Ajustar `cfg.CORRECCION_BRUJULA` si tu WorldInfo
    define otro norte.
    """
    pos = gps.getValues()
    c = brujula.getValues()
    phi = normalizar_angulo(-math.atan2(c[1], c[0]) + cfg.CORRECCION_BRUJULA)
    return (pos[0], pos[1], phi)


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    nombre_escenario = cfg.ESCENARIO_ACTIVO
    escenario = cfg.obtener_escenario()

    dispositivos = inicializar_dispositivos(robot, timestep)
    # un paso de simulación para que los sensores entreguen lecturas válidas
    if robot.step(timestep) == -1:
        return

    hay_gt = dispositivos["gps"] is not None and dispositivos["brujula"] is not None
    if hay_gt:
        pose_inicial = leer_pose_gt(dispositivos["gps"], dispositivos["brujula"])
        print(f"[INFO] Ground-truth disponible (GPS + Compass). "
              f"Pose inicial GT: ({pose_inicial[0]:.3f}, {pose_inicial[1]:.3f}, "
              f"{pose_inicial[2]:.3f} rad)")
    else:
        pose_inicial = tuple(escenario["pose_inicial"])
        print("[AVISO] Sin GPS/Compass: se opera SOLO con odometría. "
              "Las columnas *_gt del CSV quedarán vacías y el análisis "
              "omitirá las métricas que requieren ground-truth.")

    # ---------------------------------------------------------------- PLAN --
    odometria = Odometria(pose_inicial, cfg.RADIO_RUEDA, cfg.DIST_ENTRE_RUEDAS)
    grilla = construir_grilla(escenario)
    grilla_plan = inflar_para_planificar(
        grilla, cfg.RADIO_INFLADO, escenario.get("inflar_bordes", True))

    resultado = planificar(grilla_plan,
                           (pose_inicial[0], pose_inicial[1]),
                           escenario["meta"], cfg.PASO_MUESTREO_LDV)
    if resultado is None:
        estado = Estado.SIN_RUTA
        waypoints, ruta_mundo = [], []
        print("[ERROR] A* no encontró ruta entre la pose inicial y la meta. "
              "Revisa obstáculos/inflado en config.py. El robot permanecerá "
              "detenido (estado SIN_RUTA).")
    else:
        estado = Estado.FOLLOW_PATH
        waypoints, ruta_mundo = resultado
        print(f"[INFO] Ruta planificada: {len(ruta_mundo)} celdas, "
              f"{len(waypoints)} waypoints tras suavizado.")
        registro.guardar_ruta_planificada(cfg.DIR_DATOS, nombre_escenario,
                                          waypoints, ruta_mundo)
    registro.guardar_grilla(cfg.DIR_DATOS, nombre_escenario,
                            grilla, grilla_plan, escenario, hay_gt)

    # -------------------------------------------------- filtros y controles --
    filtros_ema = [FiltroEMA(cfg.EMA_ALPHA) for _ in range(8)]
    kalman = Kalman1D(cfg.DIST_IR_MAX, cfg.KALMAN_P0, cfg.KALMAN_Q, cfg.KALMAN_R)
    kalman_activo = False
    reactiva = CapaReactiva(
        cfg.IR_UMBRAL_AVOID_ON, cfg.IR_UMBRAL_AVOID_OFF, cfg.IR_UMBRAL_STOP,
        cfg.IR_UMBRAL_CASI_COLISION, cfg.IDX_IR_FRONTALES,
        cfg.IDX_IR_IZQUIERDA, cfg.IDX_IR_DERECHA,
        cfg.V_EVASION, cfg.W_EVASION)
    seguidor = ControladorWaypoints(
        cfg.KP_ANGULAR, cfg.KP_LINEAL, cfg.V_MAX, cfg.W_MAX,
        cfg.RADIO_RUEDA, cfg.DIST_ENTRE_RUEDAS, cfg.VEL_MAX_MOTOR)

    marca = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log = registro.RegistroCSV(
        os.path.join(cfg.DIR_DATOS, f"log_{nombre_escenario}_{marca}.csv"),
        COLUMNAS_LOG)
    print(f"[INFO] Registrando en: {log.ruta_archivo}")

    # ------------------------------------------------------- bucle principal --
    t = 0.0
    wp_idx = 0
    pasos_finales = 0

    while robot.step(timestep) != -1:
        t += timestep / 1000.0

        # 1) percepción: encoders → odometría; GT si existe
        pose_odom = odometria.actualizar(dispositivos["enc_izq"].getValue(),
                                         dispositivos["enc_der"].getValue())
        pose_gt = (leer_pose_gt(dispositivos["gps"], dispositivos["brujula"])
                   if hay_gt else None)
        pose_control = (pose_gt if (cfg.USAR_GT_PARA_CONTROL and hay_gt)
                        else pose_odom)

        # 2) percepción: IR crudo → EMA → fusión Kalman encoder-IR
        ir_crudo = [s.getValue() for s in dispositivos["ir"]]
        ir_filtrado = [filtros_ema[k].actualizar(ir_crudo[k]) for k in range(8)]
        frontal_max = max(ir_filtrado[k] for k in cfg.IDX_IR_FRONTALES)
        if frontal_max > cfg.IR_PISO_RUIDO:
            dist_medida = ir_a_distancia(frontal_max, cfg.TABLA_IR,
                                         cfg.DIST_IR_MAX)
            if not kalman_activo:
                kalman.reiniciar(dist_medida, cfg.KALMAN_P0)
                kalman_activo = True
            else:
                kalman.predecir(-odometria.ultimo_ds)  # el avance acerca
                kalman.corregir(dist_medida)
            dist_kalman = max(0.0, kalman.x)
        else:
            kalman_activo = False
            dist_medida = ""
            dist_kalman = ""

        # 3) capa reactiva con histéresis + transiciones de la máquina
        en_evasion, casi_colision = reactiva.evaluar(ir_filtrado)
        if estado == Estado.FOLLOW_PATH and en_evasion:
            estado = Estado.AVOID
        elif estado == Estado.AVOID and not en_evasion:
            estado = Estado.FOLLOW_PATH
            reactiva.reiniciar()

        # 3b) bloqueo prolongado → marcar obstáculo estimado y RE-PLANIFICAR
        if estado == Estado.AVOID and reactiva.bloqueado(cfg.PASOS_BLOQUEO_REPLAN):
            d_obs = dist_kalman if isinstance(dist_kalman, float) else 0.05
            ox = pose_control[0] + (cfg.RADIO_ROBOT + d_obs) * math.cos(pose_control[2])
            oy = pose_control[1] + (cfg.RADIO_ROBOT + d_obs) * math.sin(pose_control[2])
            print(f"[AVISO] Bloqueo en t={t:.2f}s: marcando obstáculo estimado "
                  f"en ({ox:.3f}, {oy:.3f}) y re-planificando...")
            grilla.marcar_circulo(ox, oy, cfg.RADIO_OBSTACULO_DETECTADO)
            grilla_plan.marcar_circulo(ox, oy,
                                       cfg.RADIO_OBSTACULO_DETECTADO + cfg.RADIO_INFLADO)
            nuevo = planificar(grilla_plan,
                               (pose_control[0], pose_control[1]),
                               escenario["meta"], cfg.PASO_MUESTREO_LDV)
            if nuevo is None:
                estado = Estado.SIN_RUTA
                print("[ERROR] Re-planificación sin ruta factible: detención.")
            else:
                waypoints, ruta_mundo = nuevo
                wp_idx = 0
                estado = Estado.FOLLOW_PATH
                reactiva.reiniciar()
                registro.guardar_ruta_planificada(cfg.DIR_DATOS, nombre_escenario,
                                                  waypoints, ruta_mundo)
                registro.guardar_grilla(cfg.DIR_DATOS, nombre_escenario,
                                        grilla, grilla_plan, escenario, hay_gt)
                print(f"[INFO] Nueva ruta: {len(waypoints)} waypoints.")

        # 4) generación de comandos según el estado
        if estado == Estado.FOLLOW_PATH and waypoints:
            objetivo = waypoints[wp_idx]
            v, w, distancia, _ = seguidor.calcular(pose_control, objetivo)
            es_ultimo = (wp_idx == len(waypoints) - 1)
            umbral = cfg.UMBRAL_META if es_ultimo else cfg.UMBRAL_WAYPOINT
            if distancia < umbral:
                if es_ultimo:
                    estado = Estado.GOAL_REACHED
                    v, w = 0.0, 0.0
                    print(f"[INFO] META ALCANZADA en t={t:.2f}s "
                          f"(pose estimada: {pose_control[0]:.3f}, "
                          f"{pose_control[1]:.3f}).")
                else:
                    wp_idx += 1
        elif estado == Estado.AVOID:
            v, w = reactiva.comando_evasion(ir_filtrado)
        else:  # GOAL_REACHED o SIN_RUTA: motores detenidos
            v, w = 0.0, 0.0

        vl_motor, vr_motor = seguidor.a_velocidades_rueda(v, w)
        dispositivos["motor_izq"].setVelocity(vl_motor)
        dispositivos["motor_der"].setVelocity(vr_motor)

        # 5) registro por paso de simulación
        fila = {"t": t, "estado": estado.value, "wp_idx": wp_idx,
                "x_odom": pose_odom[0], "y_odom": pose_odom[1],
                "phi_odom": pose_odom[2],
                "x_gt": pose_gt[0] if pose_gt else "",
                "y_gt": pose_gt[1] if pose_gt else "",
                "phi_gt": pose_gt[2] if pose_gt else "",
                "v_cmd": v, "w_cmd": w,
                "vl_motor": vl_motor, "vr_motor": vr_motor,
                "dist_ir": dist_medida, "dist_kalman": dist_kalman,
                "casi_colision": int(casi_colision)}
        for k in range(8):
            fila[f"ps{k}"] = ir_crudo[k]
            fila[f"ps{k}_f"] = ir_filtrado[k]
        log.registrar(fila)

        # 6) cierre ordenado tras la meta o sin ruta
        if estado in (Estado.GOAL_REACHED, Estado.SIN_RUTA):
            pasos_finales += 1
            if pasos_finales >= cfg.PASOS_EXTRA_TRAS_META:
                break

    dispositivos["motor_izq"].setVelocity(0.0)
    dispositivos["motor_der"].setVelocity(0.0)
    log.cerrar()
    print(f"[INFO] Fin de la corrida. Estado final: {estado.value}. "
          f"Log: {log.ruta_archivo}")


if __name__ == "__main__":
    main()
