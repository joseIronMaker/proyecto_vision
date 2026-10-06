# Data Model: Paquete ROS 2 de percepción para navegación

**Feature**: `001-paquete-percepcion-nav` | **Fecha**: 2026-10-06 | **Spec**: [spec.md](spec.md)

No hay base de datos. Las entidades son mensajes estándar en tránsito (principio II) y
estructuras internas de las funciones puras. En la columna "Representación" se indica el tipo
ROS, o el tipo de Python cuando la entidad no sale del proceso.

## Entidades

### Cuadro de imagen

| Campo | Representación | Regla |
|---|---|---|
| imagen | `sensor_msgs/Image`, `bgr8` (cámara) o `rgb8` (simulación) | Se convierte con `cv_bridge` a `bgr8` al consumirla |
| `header.stamp` | `builtin_interfaces/Time` | Reloj del nodo de cámara (FR-003) o de la simulación; nunca cero |
| `header.frame_id` | string | No vacío; marco óptico (FR-071) |

### Detección 2D

| Campo | Representación | Regla |
|---|---|---|
| contenedor | `vision_msgs/Detection2DArray` | `header` = el del cuadro (FR-013) |
| centro | `bbox.center.position.{x,y}` (px) | Dentro de la imagen |
| tamaño | `bbox.size_x`, `bbox.size_y` (px) | `> 0` (SC-004) |
| clase | `results[0].hypothesis.class_id` | Nombre COCO no vacío |
| score | `results[0].hypothesis.score` | `0 ≤ score ≤ 1` y `≥ conf_threshold` |

Interno (puro): `BBox2D(cx, cy, w, h)` en píxeles, en `float`.

### Imagen de profundidad e intrínsecos

| Campo | Representación | Regla |
|---|---|---|
| profundidad | `sensor_msgs/Image` | `32FC1` → metros, escala 1.0; `16UC1`/`mono16` → milímetros, escala 0.001; otra codificación → se descarta (FR-032) |
| válido | píxel de profundidad | Finito y `> 0` (descarta 0, NaN, ±inf) |
| intrínsecos | `sensor_msgs/CameraInfo.k` | `fx = k[0]`, `fy = k[4]`, `cx = k[2]`, `cy = k[5]`; `fx, fy > 0` (FR-031) |
| alineación | supuesto | Profundidad del mismo tamaño que el color y en el mismo marco (Assumptions) |

Interno (puro): `Intrinsics(fx, fy, cx, cy)`.

### Ventana de profundidad (derivada)

- Centro = centro del bbox; lados = `max(1, round(w·f))` × `max(1, round(h·f))`, con
  `f = depth_window_fraction`.
- Se recorta a los límites de la imagen (caso límite: bbox en el borde).
- `Z = mediana(valores válidos · escala)` si hay al menos `min_valid_pixels` válidos; si no,
  `None` y la detección se omite con una advertencia.

### Detección 3D

| Campo | Representación | Regla |
|---|---|---|
| contenedor | `vision_msgs/Detection3DArray` | `header` = el de la detección 2D (estampa y marco óptico) |
| posición | `results[0].pose.pose.position` = `bbox.center.position` | `X = (u − cx)·Z/fx`, `Y = (v − cy)·Z/fy`, `Z` (US3, escenario 1) |
| clase y score | `results[0].hypothesis` | Copiados de la 2D (US3, escenario 4) |
| tamaño | `bbox.size` | `(w·Z/fx, h·Z/fy, 0)`; con `estimate_center`, `size.z = w·Z/fx` |
| centro estimado | `estimate_center = true` (FR-036) | El punto se desplaza sobre su rayo hasta `Z' = Z + (w·Z/fx)/2`, y `X`, `Y` escalan con `Z'/Z` |

### Meta de navegación

| Campo | Representación | Regla |
|---|---|---|
| vista previa | `geometry_msgs/PoseStamped` | `frame_id = map_frame`; estampa = la de la detección |
| posición | `pose.position` | `g = o − d·(o − r)/‖o − r‖`, `z = 0` (FR-042) |
| orientación | `pose.orientation` | Solo yaw: `atan2(o_y − g_y, o_x − g_x)` |
| envío | `NavigateToPose.Goal.pose` | Igual a la vista previa; solo si `dry_run = false` |

Interno (puro): `compute_goal(robot_xy, object_xy, d, t) → (x, y, yaw) | None`. Devuelve `None`
si `‖o − r‖ ≤ d + t`.

### Árbol de marcos

`map → odom → base_link → camera_link → <marco óptico>`. Detalle por modo en
[contracts/ros-interfaces.md](contracts/ros-interfaces.md#árbol-de-marcos-tf2-rep-105).

### Silla objetivo (simulación)

| Atributo | Valor |
|---|---|
| Fuente | Macro `percepcion_nav/freecad/make_chair.py` (FreeCAD), versión 2: butaca con estructura de madera y cojines azules |
| Modelo SDF | `models/freecad_chair/{model.config, model.sdf, meshes/}` |
| Visual | Dos mallas exportadas (`chair_frame.stl`, `chair_cushions.stl`), cada una con su material |
| Colisión | 11 cajas: 4 patas, 2 brazos, 2 travesaños laterales, travesaño frontal, cojín del asiento y cojín del respaldo (inclinado 10°) |
| Faldón frontal | Travesaño frontal (0.28–0.33 m) más el frente del cojín del asiento (0.33–0.44 m). Impide que la ventana de profundidad vea el fondo entre las patas (R4, R10) |
| Dimensiones de referencia | Asiento a 0.44 m, altura total 0.90 m, huella 0.52 (fondo) × 0.56 (ancho) m |
| Pose real | Mundo `(-5.0, 0.0, 0.0, yaw π)` = `map (3.0, 0.0)` (R3), documentada en README y reporte |
| Punto de referencia | Centro de la huella (origen del modelo). SC-008 y SC-009 se miden desde aquí |
| Respaldo | `OpenRobotics/Chair` o `OpenRobotics/WoodenChair` de Fuel (FR-067) |

### Tabla de validado frente a diseño

Vive en el README y en el reporte. Tiene una fila por componente: cámara, detector,
visualización, RGB-D, emisor de metas y simulación. Columnas: "Cómo se validó" (pruebas, `dry_run`,
simulación, video), "Evidencia" (archivo, minuto del video) y "Qué quedó sin probar"
(principio VI).

## Transiciones de estado

### Nodo de cámara

```text
          abre OK                     cuadro OK
OPENING ───────────► STREAMING ◄────────────────┐
   ▲  │ falla              │  │                 │
   │  ▼                    │  └─────────────────┘
RETRYING ◄─────────────────┘ lectura falla (no es fin de archivo) → libera
   (temporizador reconnect_period_sec)
STREAMING ── fin de archivo ──► loop ? reinicia a cuadro 0 : SHUTDOWN (libera y se apaga)
```

El fin de archivo se distingue de una falla porque la posición alcanza `CAP_PROP_FRAME_COUNT`.
Una fuente de dispositivo nunca tiene fin de archivo.

### Emisor de metas

```text
                 detección válida y dist > d + t
        ┌───────────────────────────────────────────┐
        │                                           ▼
      IDLE ◄──── resultado (SUCCEEDED / ABORTED / CANCELED) ─── ACTIVE
        ▲                                           ▲   │
        │ rechazada o servidor ausente              │   │ objetivo movido > goal_update_threshold
        └────────────── SENDING ────────────────────┘   │ y dist > d + t → nueva meta (reemplaza)
                          ▲  aceptada                   ▼
                          └──────────────────────── SENDING
```

- En `dry_run`, el nodo nunca sale de `IDLE`: calcula, publica la vista previa y no envía.
- Las respuestas o resultados de una meta reemplazada se ignoran (se comparan los
  identificadores de meta).
- Con dist ≤ d + t no se envía meta en ningún estado (FR-042).

## Validaciones transversales

- Todo mensaje publicado con `header` tiene estampa no cero y `frame_id` no vacío (FR-071).
- Las funciones puras (`geometry.py`, `goal_planning.py`) no importan `rclpy` ni mensajes de ROS
  (principio V).
