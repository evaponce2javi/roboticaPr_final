Mundos Webots del proyecto.

Los obstaculos de `simple.wbt` y `complejo.wbt` se generan desde
`controllers/epuck_navegacion/config.py`. Ese archivo es la fuente de verdad:
si cambias un obstaculo planificado, actualiza `config.py` y ejecuta:

```powershell
python worlds\generar_mundos.py
```

Ambos mundos usan un `RectangleArena` de 3 m x 3 m centrado en el origen.
