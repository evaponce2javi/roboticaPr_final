# -*- coding: utf-8 -*-
"""Genera los mundos Webots desde controllers/epuck_navegacion/config.py."""

from pathlib import Path
import sys

DIR_RAIZ = Path(__file__).resolve().parents[1]
DIR_CONTROLADOR = DIR_RAIZ / "controllers" / "epuck_navegacion"
sys.path.insert(0, str(DIR_CONTROLADOR))

import config as cfg  # noqa: E402

ALTURA_OBSTACULO = 0.20


def f(valor):
    """Formato compacto y estable para numeros VRML."""
    return f"{float(valor):.6g}"


def bloque_rect(indice, obs):
    ancho = obs["x_max"] - obs["x_min"]
    fondo = obs["y_max"] - obs["y_min"]
    cx = (obs["x_min"] + obs["x_max"]) / 2.0
    cy = (obs["y_min"] + obs["y_max"]) / 2.0
    z = ALTURA_OBSTACULO / 2.0
    return f"""Solid {{
  translation {f(cx)} {f(cy)} {f(z)}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.85 0.35 0.12
        roughness 0.6
      }}
      geometry Box {{
        size {f(ancho)} {f(fondo)} {f(ALTURA_OBSTACULO)}
      }}
    }}
  ]
  name "cfg_rect_{indice}"
  boundingObject Box {{
    size {f(ancho)} {f(fondo)} {f(ALTURA_OBSTACULO)}
  }}
  locked TRUE
}}"""


def bloque_circulo(indice, obs):
    z = ALTURA_OBSTACULO / 2.0
    radio = obs["radio"]
    return f"""Solid {{
  translation {f(obs["cx"])} {f(obs["cy"])} {f(z)}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.15 0.45 0.85
        roughness 0.5
      }}
      geometry Cylinder {{
        radius {f(radio)}
        height {f(ALTURA_OBSTACULO)}
      }}
    }}
  ]
  name "cfg_circulo_{indice}"
  boundingObject Cylinder {{
    radius {f(radio)}
    height {f(ALTURA_OBSTACULO)}
  }}
  locked TRUE
}}"""


def bloque_obstaculo(indice, obs):
    if obs["tipo"] == "rect":
        return bloque_rect(indice, obs)
    if obs["tipo"] == "circulo":
        return bloque_circulo(indice, obs)
    raise ValueError(f"Tipo de obstaculo desconocido: {obs['tipo']}")


def generar_mundo(nombre, escenario):
    limites = escenario["limites"]
    ancho = limites["x_max"] - limites["x_min"]
    fondo = limites["y_max"] - limites["y_min"]
    pose = escenario["pose_inicial"]
    obstaculos = "\n".join(
        bloque_obstaculo(i, obs)
        for i, obs in enumerate(escenario.get("obstaculos", []))
    )
    return f"""#VRML_SIM R2025a utf8
# Generated from controllers/epuck_navegacion/config.py. Do not hand-edit obstacles.

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/RectangleArena.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"

WorldInfo {{
  coordinateSystem "ENU"
}}
Viewpoint {{
  orientation 0 0 1 0
  position 0 0 5.2
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
RectangleArena {{
  floorSize {f(ancho)} {f(fondo)}
}}
{obstaculos}
E-puck {{
  translation {f(pose[0])} {f(pose[1])} 0
  rotation 0 0 1 {f(pose[2])}
  kinematic TRUE
  controller "epuck_navegacion"
  controllerArgs [
    "{nombre}"
  ]
  turretSlot [
    Compass {{
    }}
    GPS {{
    }}
  ]
}}
"""


def main():
    for nombre, escenario in cfg.ESCENARIOS.items():
        ruta = Path(__file__).with_name(f"{nombre}.wbt")
        ruta.write_text(generar_mundo(nombre, escenario), encoding="utf-8")
        print(f"Generado {ruta}")


if __name__ == "__main__":
    main()
