# Contrato de lanzamientos

**Feature**: `001-paquete-percepcion-nav` | Requisitos: FR-051, FR-063–FR-066 | Argumentos en
[parameters.md](parameters.md#argumentos-de-los-lanzamientos)

## `pipeline_2d.launch.py` (US1)

**Lanza**: `camera`, `detector`, `visualizer` y, si `viewer:=true` (por defecto),
`rqt_image_view` sobre `/detections_image`.

**Garantiza**:
- En menos de 30 s, `/detections_image` publica cuadros anotados (SC-002).
- `/detections` publica a 5 Hz o más en CPU (SC-003).
- Con `video:=/ruta/inexistente`, `camera` registra el error y reintenta sin terminar (SC-005).

## `dry_run.launch.py` (US3 y US4 sin simulación)

**Lanza**:
- Todo lo de `pipeline_2d.launch.py`, más `synthetic_depth`, `rgbd_localizer` y
  `nav2_goal_sender` (con `dry_run:=true`).
- Cuatro `static_transform_publisher` con el árbol de marcos de `dry_run`
  ([ros-interfaces.md](ros-interfaces.md#árbol-de-marcos-tf2-rep-105)). Los nombres de marco y la
  posición de la cámara salen de los argumentos `map_frame`, `odom_frame`, `base_frame`,
  `camera_link_frame`, `optical_frame`, `cam_x`, `cam_y` y `cam_z`
  ([parameters.md](parameters.md#argumentos-de-los-lanzamientos)). Esos mismos argumentos alimentan
  `camera.frame_id`, `nav2_goal_sender.map_frame` y `nav2_goal_sender.robot_base_frame`.
- RViz opcional, mostrando `/goal_pose_preview`, `/detections_image` y tf.

**Garantiza**:
- Cuando el video muestra una silla, `/detections_3d` publica `z ≈ depth_m` (2.5 m) con la
  estampa del cuadro (US3, escenario 6).
- `/goal_pose_preview` publica una pose en `map` a 1.0 m del punto 3D, sobre la línea
  robot–objeto y orientada hacia él (US4; SC-007).
- No intenta conectarse a ningún servidor de acción.

## `simulation.launch.py` (US5)

**Lanza**:
1. `nav2_bringup/launch/tb4_simulation_launch.py`, con `params_file:=config/nav2_params_sim.yaml`
   (AMCL con pose inicial, R2) y pasando `headless` y `use_rviz`. El mundo y el mapa son `depot`.
2. `AppendEnvironmentVariable GZ_SIM_RESOURCE_PATH += share/percepcion_nav/models`.
3. `ros_gz_sim create` de la silla (`chair_model`) en `(chair_x, chair_y, 0, chair_yaw)`.
4. `detector`, `visualizer`, `rgbd_localizer` y `nav2_goal_sender` con `params.yaml` +
   `params_sim.yaml` y los remapeos a `/rgbd_camera/*`.

**No lanza**: `camera` ni `synthetic_depth` (FR-035, FR-063).

**Garantiza**:
- Robot localizado sin pasos manuales (FR-065).
- Detecciones `chair` con score ≥ `conf_threshold` (US5, escenario 1).
- La posición estimada de la silla en `map` queda a 0.30 m o menos de `(chair_x + 8.0, chair_y)`
  (SC-008).
- La acción `navigate_to_pose` termina con `SUCCEEDED` a 1.0 ± 0.30 m del centro de la huella de
  la silla, mirándola con 15° o menos de error, en al menos 4 de 5 ejecuciones (SC-009).
