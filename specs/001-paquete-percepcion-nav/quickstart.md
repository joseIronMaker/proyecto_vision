# Quickstart: validación de extremo a extremo

**Feature**: `001-paquete-percepcion-nav` | Guía de validación, no de implementación. Interfaces
en [contracts/](contracts/) y entidades en [data-model.md](data-model.md).

Cada escenario indica qué criterio demuestra. Los comandos asumen que estás en la raíz del
repositorio, que funciona también como workspace de colcon.

## 0. Requisitos previos

- Ubuntu 24.04 con ROS 2 Jazzy (`/opt/ros/jazzy`), 8 núcleos, sin GPU.
- Paquetes apt: `ros-jazzy-vision-msgs`, `ros-jazzy-cv-bridge`, `ros-jazzy-tf2-geometry-msgs`,
  `ros-jazzy-nav2-msgs`, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`,
  `ros-jazzy-nav2-minimal-tb4-sim`, `ros-jazzy-ros-gz`, `ros-jazzy-rqt-image-view`,
  `python3-venv`, `python3-pip` (el README da el comando `rosdep` equivalente).
- Internet en el primer arranque de la simulación (descarga el modelo Depot de Gazebo Fuel).

## 1. Preparación

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv && touch .venv/COLCON_IGNORE
source .venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install -r percepcion_nav/requirements.txt          # numpy<2, opencv 4.11, ultralytics fijo
python -c "import numpy, cv_bridge, ultralytics; print(numpy.__version__)"   # debe ser 1.x
```

Pesos y video: deben existir `percepcion_nav/models/yolo11n.pt` y `percepcion_nav/media/sample.mp4`
(vienen en el `.zip`; el README da el comando de descarga de los pesos).

## 2. Compilación y pruebas → SC-001

```bash
colcon build            # sin --symlink-install: ver research R6
source install/setup.bash
colcon test --packages-select percepcion_nav && colcon test-result --verbose
python -m pytest percepcion_nav/test/test_geometry.py percepcion_nav/test/test_goal_planning.py   # sin ROS
```

**Esperado**: 0 errores y 0 pruebas fallidas. La última línea corre sin el entorno de ROS
(principio V) y cubre SC-006 (error < 1 mm) y SC-007 (1 cm, < 1°). Funciona desde la raíz porque
`percepcion_nav/setup.cfg` declara `[tool:pytest] pythonpath = .`. Sin eso, `percepcion_nav` se
resolvería como la carpeta externa y fallaría la importación.

## 3. Pipeline 2D → US1, SC-002, SC-003, SC-004

```bash
ros2 launch percepcion_nav pipeline_2d.launch.py
# en otra terminal:
ros2 run rqt_image_view rqt_image_view /detections_image
ros2 topic echo /detections --once
ros2 topic hz /detections
```

**Esperado**:
- Imagen anotada con rectángulo, clase y score en menos de 30 s.
- `header` de `/detections` igual al de la imagen; `bbox.size_x/size_y > 0`, `class_id` no
  vacío y `score` entre 0 y 1.
- `hz` ≥ 5.

## 4. Robustez de la cámara → US1 escenarios 4 y 6, SC-005

```bash
ros2 launch percepcion_nav pipeline_2d.launch.py video:=/tmp/no_existe.mp4
# el log muestra el error y los reintentos cada 2 s; el proceso sigue vivo.
cp percepcion_nav/media/sample.mp4 /tmp/no_existe.mp4
# en 10 s o menos vuelve a publicar: ros2 topic hz /camera/image_raw
```

Con `loop:=false` al final del video, el nodo `camera` se apaga de forma limpia.

## 5. `dry_run` completo → US3 escenario 6, US4, SC-007

```bash
ros2 launch percepcion_nav dry_run.launch.py
ros2 topic echo /detections_3d --once        # z ≈ 2.5 (depth_m), estampa = la del cuadro
ros2 topic echo /goal_pose_preview --once    # frame_id: map
```

**Esperado**: con una silla en el video, `/goal_pose_preview` queda a 1.0 m del punto
3D, sobre la línea `base_link`–objeto, con `z = 0` y orientada hacia el objeto. Como el robot está
en el origen de `map`, `‖meta‖ ≈ ‖objeto‖ − 1.0`. El registro de `nav2_goal_sender` no menciona
ningún servidor de acción.

Comprobaciones de errores:
- Transformación no disponible (US4, escenario 2). Con el `dry_run` en marcha, lanzar una segunda
  instancia del emisor hacia un marco que no existe:
  `ros2 run percepcion_nav nav2_goal_sender_node --ros-args -r __node:=goal_sender_tf_test -p map_frame:=frame_inexistente`.
  Registra una `TransformException` (limitada en frecuencia) y no publica meta. Matar un
  `static_transform_publisher` **no** sirve como prueba, porque tf2 conserva para siempre las
  transformaciones de `/tf_static` ya recibidas.
- `ros2 param set /nav2_goal_sender dry_run false`: vence la espera del servidor, se registra y el
  nodo sigue vivo (US4, escenario 3).

## 6. Grafo → US2 escenario 3

```bash
ros2 run rqt_graph rqt_graph
```

**Esperado**: nodos, tópicos y tipos iguales a [contracts/ros-interfaces.md](contracts/ros-interfaces.md)
y a `docs/diagrama_nodos.md`.

## 7. Simulación → US5, SC-008, SC-009

**Prueba previa (R12)**, antes de integrar nada:

```bash
ros2 launch nav2_bringup tb4_simulation_launch.py headless:=True use_rviz:=False
ros2 topic hz /rgbd_camera/image           # debe publicar (≈10 Hz o menos con render por software)
```

**Sistema completo**:

```bash
ros2 launch percepcion_nav simulation.launch.py            # headless:=False para ver Gazebo
```

**Esperado**:
1. El robot queda localizado sin tocar RViz (FR-065): `/amcl_pose` publica `(0, 0)`. Para la
   pose real se usa `/odom` de Gazebo (origen en la aparición, que coincide con `map (0, 0)`).
   `gz model -m turtlebot4 -p` vence su tiempo de espera con el render por software.
2. `/detections` contiene `chair` con score ≥ 0.40.
3. El registro de `nav2_goal_sender` muestra el objetivo en `map`. Debe quedar a 0.30 m o menos
   de `(3.0, 0.0)`, el centro de la huella (SC-008).
4. La acción termina en `SUCCEEDED`. La distancia final entre el robot (`/odom`) y el centro de
   la huella de la silla `(3.0, 0.0)` es 1.0 ± 0.30 m y el error de orientación es ≤ 15°
   (SC-009).
5. Después de llegar no se envían metas nuevas (FR-042, tolerancia de llegada).

Repetir el paso 4 cinco veces y anotar los resultados en `docs/validacion_simulacion.md`. Se
cumple con al menos 4 de 5. Si la silla de FreeCAD no alcanza el score mínimo, usar
`chair_model:=<modelo de Fuel>` y declararlo (FR-067).

## 8. Entregables → SC-011

```bash
zip -r percepcion_nav.zip percepcion_nav -x '*/__pycache__/*'
ls -lh percepcion_nav.zip video_demo.mp4 reporte_tecnico.pdf     # cada uno ≤ 50 MB
```

**Esperado**: el `.zip` no contiene `build/`, `install/`, `log/` ni `.venv/`; hay 20 archivos
o menos en Moodle; el video dura de 2 a 4 minutos.
