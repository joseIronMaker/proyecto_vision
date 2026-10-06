# Contrato de interfaces ROS 2

**Feature**: `001-paquete-percepcion-nav` | Requisitos: FR-001–FR-072 | Principios I, II y III

Este contrato es la fuente del diagrama de nodos (`docs/diagrama_nodos.md`) y debe coincidir con
`rqt_graph` (US2, escenario 3). Todos los nombres de tópico son valores por defecto y todos son
remapeables.

## Nodos y ejecutables

| Ejecutable (`ros2 run percepcion_nav …`) | Nodo | Responsabilidad | Lanzamientos |
|---|---|---|---|
| `camera_node` | `camera` | Leer video o dispositivo y publicar cuadros | pipeline 2D, `dry_run` |
| `detector_node` | `detector` | Inferencia YOLO → `Detection2DArray` | los tres |
| `visualizer_node` | `visualizer` | Dibujar detecciones sobre su cuadro | los tres |
| `rgbd_localizer_node` | `rgbd_localizer` | Detección 2D + profundidad → `Detection3DArray` | `dry_run`, simulación |
| `nav2_goal_sender_node` | `nav2_goal_sender` | Detección 3D → meta `NavigateToPose` | `dry_run`, simulación |
| `synthetic_depth_node` | `synthetic_depth` | Herramienta de prueba: profundidad e intrínsecos sintéticos (FR-035) | solo `dry_run` |

## Tópicos

QoS: **R** = reliable, **BE** = best effort, **V** = volatile, **TL** = transient local;
`kN` = keep last N.

| Tópico (por defecto) | Tipo | Publica (QoS) | Suscribe (QoS) | `frame_id` | Frecuencia |
|---|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (`bgr8`) | `camera` (R, V, k5) | `detector` (BE, k1); `visualizer` (BE, k30 vía `message_filters`); `synthetic_depth` (BE, k5) | `camera_color_optical_frame` (parámetro) | `fps` (15 Hz por defecto) |
| `/detections` | `vision_msgs/Detection2DArray` | `detector` (R, V, k10) | `visualizer` (R, k30); `rgbd_localizer` (R, k30) | el de la imagen | ≤ frecuencia de imagen |
| `/detections_image` | `sensor_msgs/Image` (`bgr8`) | `visualizer` (R, V, k5) | `rqt_image_view`, RViz | el de la imagen | = frecuencia de detección |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (`32FC1` m o `16UC1` mm) | `synthetic_depth` (R, V, k5) o cámara simulada | `rgbd_localizer` (BE, k30 vía `message_filters`) | el de la imagen | = frecuencia de imagen |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `synthetic_depth` (R, V, k5) o cámara simulada | `rgbd_localizer` (BE, k1; guarda el último) | el de la imagen | = frecuencia de imagen |
| `/detections_3d` | `vision_msgs/Detection3DArray` | `rgbd_localizer` (R, V, k10) | `nav2_goal_sender` (R, k10) | marco óptico | ≤ frecuencia de detección |
| `/goal_pose_preview` | `geometry_msgs/PoseStamped` | `nav2_goal_sender` (R, TL, k1) | RViz, `ros2 topic echo` | `map` (parámetro) | por meta calculada |

### Contenido obligatorio de los mensajes

**`Detection2DArray`** (FR-012–FR-015):
- `header` = copia exacta del `header` de la imagen de entrada; cada `Detection2D.header` también.
- `bbox.center.position.{x,y}`: centro en píxeles; `bbox.center.theta = 0.0`.
- `bbox.size_x`, `bbox.size_y`: ancho y alto en píxeles, ambos mayores que 0 (SC-004).
- `results[0].hypothesis.class_id`: nombre COCO (`"chair"`, `"person"`, …), no vacío.
- `results[0].hypothesis.score`: entre 0 y 1.
- `results[0].pose`: sin usar (valores por defecto).
- Si no hay detecciones, se publica igual un arreglo vacío con el `header` de la imagen.

**`Detection3DArray`** (FR-033):
- `header` = `header` de la `Detection2DArray` de origen (estampa y marco óptico).
- `results[0].hypothesis` = clase y score de la detección 2D de origen.
- `results[0].pose.pose.position` = `bbox.center.position` = `(X, Y, Z)` en metros, en el marco
  óptico. `orientation` = identidad.
- `bbox.size.x = w·Z/fx`, `bbox.size.y = h·Z/fy` (extensión métrica aproximada),
  `bbox.size.z = 0.0` (desconocida).
- Las detecciones sin profundidad válida se omiten, y si ninguna es válida se publica el arreglo
  vacío.

**`PoseStamped` de vista previa** (FR-046):
- `header.frame_id` = `map`; `header.stamp` = estampa de la detección de origen.
- `pose.position.z = 0`; la orientación es solo un giro (yaw) hacia el objeto.

## Acción

| Nombre | Tipo | Cliente | Servidor | Comportamiento |
|---|---|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `nav2_goal_sender` (solo con `dry_run:=false`) | `bt_navigator` de Nav2 | `wait_for_server(timeout)`. Envío y resultado asíncronos, con callbacks de aceptación, progreso (registro limitado en frecuencia) y resultado. Una meta nueva reemplaza a la activa. |

`goal.pose` = misma pose que `/goal_pose_preview`. `goal.behavior_tree` = `""` (el árbol por
defecto de Nav2).

## Árbol de marcos (tf2, REP 105)

```text
map ──► odom ──► base_link ──► camera_link ──► camera_color_optical_frame      (dry_run)
map ──► odom ──► base_link ──► … ──► oakd_rgb_camera_optical_frame             (simulación)
```

| Transformación | `dry_run` | Simulación |
|---|---|---|
| `map → odom` | `static_transform_publisher` (identidad) | AMCL |
| `odom → base_link` | `static_transform_publisher` (identidad) | Puente `/tf` de Gazebo (DiffDrive) |
| `base_link → cámara` | `static_transform_publisher`: `(cam_x, cam_y, cam_z)` = `(0.10, 0, 0.30)` por defecto y después rpy `(−π/2, 0, −π/2)` hacia el marco óptico. Los nombres de marco son argumentos de launch ([parameters.md](parameters.md#argumentos-de-los-lanzamientos)) | `robot_state_publisher` (URDF del TB4) |

Consultas del emisor de metas: `map ← marco óptico` (el punto) y `map ← base_link` (la pose del
robot), ambas en la estampa de la detección y con el tiempo límite `tf_timeout_sec`.

## Remapeos por modo

| Tópico del paquete | Pipeline 2D | `dry_run` | Simulación |
|---|---|---|---|
| `/camera/image_raw` | `camera` | `camera` | → `/rgbd_camera/image` |
| `/camera/depth/image_raw` | (no se usa) | `synthetic_depth` | → `/rgbd_camera/depth_image` |
| `/camera/camera_info` | (no se usa) | `synthetic_depth` | → `/rgbd_camera/camera_info` |

En simulación no se lanzan `camera` ni `synthetic_depth` (FR-035, FR-063), y todos los nodos
usan `use_sim_time:=true` (FR-064).

## Errores recuperables por nodo (principio IV, FR-072)

| Nodo | Condición | Reacción | Registro |
|---|---|---|---|
| `camera` | La fuente no abre o se pierde | Libera, reintenta cada `reconnect_period_sec` | error, limitado |
| `camera` | Cuadro ilegible (no es fin de archivo) | Libera y reconecta | advertencia, limitada |
| `camera` | Fin de video | Reinicia (`loop`) o apaga el nodo de forma limpia | info |
| `detector` | Falla la conversión de un cuadro | Descarta y sigue | advertencia, limitada |
| `detector` | Pesos inexistentes o ilegibles **al arrancar** | **Falla rápido** (no recuperable) | fatal |
| `visualizer` | Falla la conversión | Descarta y sigue | advertencia, limitada |
| `rgbd_localizer` | Sin intrínsecos todavía | No publica | advertencia, limitada |
| `rgbd_localizer` | Profundidad de la ventana inválida | Omite esa detección | advertencia, limitada |
| `rgbd_localizer` | Codificación de profundidad no soportada | Descarta el par | advertencia, limitada |
| `nav2_goal_sender` | `TransformException` | No calcula meta | advertencia, limitada |
| `nav2_goal_sender` | Servidor ausente (vence el tiempo de espera) | No envía; reintenta con la siguiente detección | advertencia, limitada |
| `nav2_goal_sender` | Meta rechazada, abortada o cancelada | Registra el estado y vuelve a `IDLE` | advertencia |
| `synthetic_depth` | Falla la conversión del cuadro | Descarta y sigue | advertencia, limitada |
