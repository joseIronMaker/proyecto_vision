# Contrato de parámetros

**Feature**: `001-paquete-percepcion-nav` | Requisitos: FR-002, FR-011, FR-022, FR-030–FR-035,
FR-040–FR-046, FR-070

Cada parámetro se declara con `declare_parameter(nombre, valor_por_defecto, descriptor)` y se
documenta con el mismo valor en `config/params.yaml` (principio I). `config/params_sim.yaml` solo
sobrescribe lo que cambia en simulación. Todos los nodos aceptan además `use_sim_time`
(estándar de ROS 2).

## `camera` (`camera_node`)

| Parámetro | Tipo | Defecto | Descripción |
|---|---|---|---|
| `source` | string | `""` (lo fija el launch con la ruta del video) | Ruta de video, o índice de dispositivo si es numérico (`"0"`) |
| `fps` | double | `15.0` | Frecuencia de publicación (Hz) |
| `frame_id` | string | `camera_color_optical_frame` | `header.frame_id` de cada cuadro |
| `loop` | bool | `true` | Reiniciar el video al terminar; si es `false`, el nodo se apaga |
| `reconnect_period_sec` | double | `2.0` | Periodo de reintento al no abrir o perder la fuente (SC-005: ≤ 10 s) |

## `detector` (`detector_node`)

| Parámetro | Tipo | Defecto | Descripción |
|---|---|---|---|
| `model_path` | string | `""` (lo fija el launch con `…/models/yolo11n.pt`) | Pesos de YOLO. Si no existe el archivo, el nodo falla al arrancar |
| `conf_threshold` | double | `0.40` | Score mínimo para publicar una detección |
| `device` | string | `cpu` | Dispositivo de inferencia (`cpu`, `cuda:0`) |
| `imgsz` | int | `640` | Tamaño de entrada del modelo |
| `class_filter` | string | `""` | Clases COCO separadas por comas; vacío = todas (FR-011) |

## `visualizer` (`visualizer_node`)

| Parámetro | Tipo | Defecto | Descripción |
|---|---|---|---|
| `show_window` | bool | `false` | Abrir una ventana local de OpenCV además del tópico (FR-022) |
| `sync_queue_size` | int | `30` | Cola del `TimeSynchronizer` exacto |

## `rgbd_localizer` (`rgbd_localizer_node`)

| Parámetro | Tipo | Defecto | Descripción |
|---|---|---|---|
| `sync_slop_sec` | double | `0.02` | Diferencia máxima entre estampas al emparejar detección y profundidad (clarificación) |
| `sync_queue_size` | int | `30` | Cola del `ApproximateTimeSynchronizer` |
| `depth_window_fraction` | double | `0.30` | Lado de la ventana central como fracción del bbox (0–1] (FR-032) |
| `min_valid_pixels` | int | `5` | Mínimo de píxeles válidos para aceptar la mediana |
| `estimate_center` | bool | `false` | Estima el centro del objeto avanzando sobre el rayo la mitad de su ancho métrico (huella cuadrada) y publica ese ancho en `bbox.size.z`; `false` publica la cara visible (FR-036) |

## `nav2_goal_sender` (`nav2_goal_sender_node`)

| Parámetro | Tipo | Defecto | Descripción |
|---|---|---|---|
| `target_class` | string | `chair` | Clase objetivo (FR-040) |
| `map_frame` | string | `map` | Marco de la meta |
| `robot_base_frame` | string | `base_link` | Marco del robot para la línea robot–objeto |
| `standoff_distance` | double | `1.0` | Distancia de separación en m (clarificación) |
| `arrival_tolerance` | double | `0.30` | Margen sobre la separación; no se envía meta si dist ≤ d + t (clarificación) |
| `goal_update_threshold` | double | `0.50` | Movimiento mínimo del objetivo (m) para reemplazar una meta activa (FR-044) |
| `max_target_jump` | double | `1.5` | Salto máximo creíble del objetivo respecto al último usado (m); los objetos son estáticos, así que un salto mayor se descarta como error de estimación (FR-044) |
| `tf_timeout_sec` | double | `0.2` | Tiempo límite de las consultas tf2 (FR-041) |
| `server_timeout_sec` | double | `2.0` | Espera máxima del servidor de acción (FR-043) |
| `dry_run` | bool | `true` | Calcula y publica la vista previa sin enviar la meta (FR-045) |
| `action_name` | string | `navigate_to_pose` | Nombre del servidor de acción |

## `synthetic_depth` (`synthetic_depth_node`, herramienta de prueba)

| Parámetro | Tipo | Defecto | Descripción |
|---|---|---|---|
| `depth_m` | double | `2.5` | Profundidad constante publicada, en metros (`32FC1`) |
| `hfov_rad` | double | `1.20` | Campo de visión horizontal con el que se calculan `fx = fy = (w/2)/tan(hfov/2)`, `cx = w/2`, `cy = h/2` |

## Sobrescrituras en simulación (`config/params_sim.yaml`)

| Nodo | Parámetro | Valor |
|---|---|---|
| todos | `use_sim_time` | `true` |
| `nav2_goal_sender` | `dry_run` | `false` (FR-066) |
| `detector` | `class_filter` | `chair` (opcional, reduce el ruido en la demo) |
| `rgbd_localizer` | `estimate_center` | `true` (FR-036: SC-008 y SC-009 se miden desde el centro de la silla) |

`config/nav2_params_sim.yaml` (copia de los parámetros de Nav2) cambia dos valores: la pose
inicial de AMCL (R2) y `general_goal_checker.xy_goal_tolerance` de 0.25 a 0.10 m (R10).

## Argumentos de los lanzamientos

| Argumento | Lanzamientos | Defecto | Uso |
|---|---|---|---|
| `video` | pipeline 2D, `dry_run` | `$(find-pkg-share percepcion_nav)/media/sample.mp4` | → `camera.source` |
| `model` | los tres | `$(find-pkg-share percepcion_nav)/models/yolo11n.pt` | → `detector.model_path` |
| `params_file` | los tres | `$(find-pkg-share percepcion_nav)/config/params.yaml` | Archivo de parámetros |
| `viewer` | pipeline 2D, `dry_run`, simulación | `true` | Abrir `rqt_image_view` en `/detections_image` (SC-002: un comando muestra las detecciones) |
| `use_rviz` | `dry_run`, simulación | `true` | Abrir RViz con la configuración del paquete |
| `headless` | simulación | `True` | Sin la interfaz de Gazebo (render por software, R12). `True`/`False` con mayúscula: `tb4_simulation_launch.py` lo evalúa como expresión de Python |
| `dry_run` | simulación | `false` | Arrancar sin enviar metas (útil para narrar la demostración); se cambia en vivo con `ros2 param set /nav2_goal_sender dry_run false` |
| `chair_spawn_delay` | simulación | `10.0` | Espera antes de insertar la silla, en s (el mundo debe estar cargado) |
| `chair_model` | simulación | `freecad_chair` | Modelo a insertar (`freecad_chair` o el de respaldo de Fuel, FR-067) |
| `chair_x`, `chair_y`, `chair_yaw` | simulación | `-5.0`, `0.0`, `3.1416` | Pose real de la silla en el mundo (R3) |
| `map_frame` | `dry_run` | `map` | Marco raíz de las transformaciones estáticas → `nav2_goal_sender.map_frame` |
| `odom_frame` | `dry_run` | `odom` | Marco de odometría estático |
| `base_frame` | `dry_run` | `base_link` | Marco del robot → `nav2_goal_sender.robot_base_frame` |
| `camera_link_frame` | `dry_run` | `camera_link` | Marco del cuerpo de la cámara |
| `optical_frame` | `dry_run` | `camera_color_optical_frame` | Marco óptico (REP 103) → `camera.frame_id` |
| `cam_x`, `cam_y`, `cam_z` | `dry_run` | `0.10`, `0.0`, `0.30` | Posición de `camera_link` respecto a `base_frame`, en metros (R11) |

En `dry_run`, los nombres de marco de las transformaciones estáticas y los parámetros de los
nodos salen de los **mismos** argumentos. Así no puede haber desajuste entre ellos (principio I:
sin nombres de marco fijos en el código). La rotación `camera_link → marco óptico`
(rpy −π/2, 0, −π/2) es la convención fija de REP 103, no un valor ajustable.

## Constantes internas (no ajustables)

Estas constantes no son parámetros porque no cambian el comportamiento del sistema; solo
controlan la cadencia de los registros. Se declaran con nombre al inicio de cada módulo y se
documentan aquí (principio I).

| Constante | Módulo | Valor | Uso |
|---|---|---|---|
| `LOG_THROTTLE_SEC` | todos los nodos | `5.0` | Periodo mínimo entre advertencias repetidas (FR-072) |
| `FPS_REPORT_PERIOD_SEC` | `detector_node` | `10.0` | Periodo del registro de fps de inferencia (evidencia de SC-003) |
| `UNMATCHED_CHECK_PERIOD_SEC` | `rgbd_localizer_node` | `2.0` | Periodo de revisión de detecciones sin profundidad emparejada |
| Profundidades de QoS | todos los nodos | según [ros-interfaces.md](ros-interfaces.md#tópicos) | Parte del contrato de interfaces |
