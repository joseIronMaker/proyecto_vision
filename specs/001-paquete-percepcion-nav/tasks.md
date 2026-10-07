---

description: "Lista de tareas de implementación de 001-paquete-percepcion-nav"
---

# Tasks: Paquete ROS 2 de percepción para navegación

**Input**: Documentos de diseño en `/specs/001-paquete-percepcion-nav/`

**Prerequisites**: [plan.md](plan.md), [spec.md](spec.md), [research.md](research.md),
[data-model.md](data-model.md), [contracts/](contracts/), [quickstart.md](quickstart.md)

**Tests**: SÍ. La especificación exige pruebas automáticas (FR-034, FR-047, SC-001) y la
constitución (principio V) exige `pytest` sin ROS para la lógica numérica. Las pruebas de
`geometry.py`, `goal_planning.py` y de los constructores de mensajes se escriben **antes** y
deben **fallar** antes de implementar.

**Organization**: tareas agrupadas por historia de usuario, en el orden de prioridad de la
especificación (US1 y US2 son P1; US3 P2; US4 P3; US5 P4, obligatoria).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: se puede hacer en paralelo (archivo distinto, sin dependencias pendientes).
- **[Story]**: historia a la que pertenece (US1–US5).
- Las rutas son relativas a la raíz del repositorio, que es también el workspace de colcon.

## Path Conventions

- Paquete: `percepcion_nav/` (raíz del paquete `ament_python`).
- Módulo Python: `percepcion_nav/percepcion_nav/`.
- Pruebas: `percepcion_nav/test/`.
- Lanzamientos, configuración y modelos: `percepcion_nav/{launch,config,rviz,models}/`.
- Documentación entregable: `percepcion_nav/README.md` y `percepcion_nav/docs/`.

## Convenciones para todo el código (aplican a cada tarea)

- Identificadores en inglés; comentarios, docstrings y mensajes de log en español (FR-057).
- Todo parámetro se declara con `declare_parameter(nombre, defecto, ParameterDescriptor(description=...))`
  con el mismo valor que en `config/params.yaml` ([contracts/parameters.md](contracts/parameters.md)).
  No hay rutas, umbrales ni nombres de marco fijos fuera de esas declaraciones (principio I).
- Los avisos repetibles usan `self.get_logger().warning(msg, throttle_duration_sec=LOG_THROTTLE_SEC)`
  (FR-072). Los periodos internos (`LOG_THROTTLE_SEC`, `FPS_REPORT_PERIOD_SEC`,
  `UNMATCHED_CHECK_PERIOD_SEC`) son constantes con nombre al inicio de cada módulo, con los valores
  de [contracts/parameters.md](contracts/parameters.md#constantes-internas-no-ajustables). No se
  escriben números sueltos en el código.
- Ningún nodo importa otro nodo. La lógica compartida va en `geometry.py` y `goal_planning.py`, que
  **no** importan `rclpy` ni mensajes de ROS.
- QoS según [contracts/ros-interfaces.md](contracts/ros-interfaces.md#tópicos): publicadores
  `RELIABLE` + `VOLATILE`; suscriptores de imagen `BEST_EFFORT`. Excepción: `/goal_pose_preview`
  es `TRANSIENT_LOCAL`, profundidad 1.
- Cada `main()` hace `rclpy.init()`, crea el nodo, hace `spin` y en `finally` llama a
  `node.destroy_node()` y `rclpy.try_shutdown()`. `KeyboardInterrupt` sale limpio.
- Estilo compatible con `ament_flake8` (líneas ≤ 99) y `ament_pep257`.

---

## Phase 1: Setup (infraestructura compartida)

**Purpose**: esqueleto del paquete, entorno de Python y compilación base.

- [X] T001 Ejecutar `/speckit-git-validate` (la constitución lo exige antes de implementar) y
  confirmar la rama `001-paquete-percepcion-nav`. Después, crear `.gitignore` en la raíz con: `build/`, `install/`, `log/`, `.venv/`,
  `__pycache__/`, `*.pyc`, `*.pt`, `*.mp4`, `percepcion_nav/media/`, `*.zip`, `*.pdf` (salvo los
  que se agreguen explícitamente con `git add -f`)
- [X] T002 [P] Crear `percepcion_nav/package.xml`:
  - Formato 3, `name` `percepcion_nav`, `version` 0.1.0, `description` en español,
    `maintainer` Jose Balbuena, licencia Apache-2.0, `<build_type>ament_python</build_type>`.
  - `depend`: `rclpy`, `sensor_msgs`, `vision_msgs`, `geometry_msgs`, `cv_bridge`,
    `message_filters`, `tf2_ros`, `tf2_geometry_msgs`, `nav2_msgs`.
  - `exec_depend`: `launch`, `launch_ros`, `rviz2`, `rqt_image_view`, `nav2_bringup`,
    `nav2_minimal_tb4_sim`, `ros_gz_sim`.
  - `test_depend`: `ament_flake8`, `ament_pep257`, `python3-pytest`.
- [X] T003 [P] Crear `percepcion_nav/resource/percepcion_nav` (archivo vacío, marcador de ament) y
  `percepcion_nav/percepcion_nav/__init__.py` (vacío)
- [X] T004 [P] Crear `percepcion_nav/setup.cfg` con:
  - `[develop] script_dir=$base/lib/percepcion_nav`.
  - `[install] install_scripts=$base/lib/percepcion_nav`.
  - `[build_scripts] executable = /usr/bin/env python3`, para que los nodos usen el Python del
    `.venv` activo (research R6).
  - `[tool:pytest]` con `pythonpath = .`, para que `python -m pytest percepcion_nav/test/...` desde
    la raíz importe `percepcion_nav/percepcion_nav/` y no la carpeta externa (sin esto, las pruebas
    sin ROS fallan al importar).
- [X] T005 [P] Crear `percepcion_nav/setup.py`:
  - `packages=['percepcion_nav']`.
  - `data_files`: `resource/percepcion_nav` → `share/ament_index/resource_index/packages`,
    `package.xml`, `glob('launch/*.launch.py')`, `glob('config/*.yaml')` y `glob('rviz/*.rviz')`
    → `share/percepcion_nav/...`.
  - Copia recursiva con una función `files_under(dirname)` basada en `os.walk` de `models/` y
    `media/`, solo si existen (así `yolo11n.pt` y `sample.mp4` se instalan cuando están
    presentes).
  - `console_scripts` (FR-050): `camera_node = percepcion_nav.camera_node:main`,
    `detector_node`, `visualizer_node`, `rgbd_localizer_node`, `nav2_goal_sender_node` y
    `synthetic_depth_node`, con el mismo patrón.
  - `install_requires=['setuptools']`, `tests_require=['pytest']`.
- [X] T006 [P] Crear `percepcion_nav/requirements.txt` con `numpy<2`,
  `opencv-python==4.11.0.86` y `ultralytics==8.4.174`, más un comentario que explique que
  `torch`/`torchvision` se instalan antes desde `https://download.pytorch.org/whl/cpu`
  (research R6)
- [X] T007 [P] Crear `percepcion_nav/test/test_flake8.py` y `percepcion_nav/test/test_pep257.py`
  con las plantillas estándar de `ros2 pkg create --build-type ament_python`
  (`@pytest.mark.flake8` / `@pytest.mark.pep257`, `main_with_errors`). No incluir la prueba de
  copyright.
- [X] T008 Crear el entorno virtual: `python3 -m venv --system-site-packages .venv &&
  touch .venv/COLCON_IGNORE`; activar; `pip install torch torchvision --index-url
  https://download.pytorch.org/whl/cpu`; `pip install -r percepcion_nav/requirements.txt`.
  Verificar que `python -c "import numpy, cv_bridge, ultralytics; print(numpy.__version__)"`
  imprime 1.x (depende de T006).
- [X] T009 Descargar los pesos a `percepcion_nav/models/yolo11n.pt` desde
  `https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt` y verificar con
  `python -c "from ultralytics import YOLO; m=YOLO('percepcion_nav/models/yolo11n.pt');
  print(m.names[56])"` → `chair` (depende de T008)
- [X] T010 Compilar y probar la base: `source /opt/ros/jazzy/setup.bash && source .venv/bin/activate
  && colcon build --symlink-install && colcon test --packages-select percepcion_nav &&
  colcon test-result --verbose`, sin errores (depende de T001–T009)

---

## Phase 2: Foundational (requisitos que bloquean)

**Purpose**: parámetros documentados y video de muestra que usan US1, US3 y US4.

**⚠️ CRITICAL**: ninguna historia empieza sin esta fase.

- [X] T011 Crear `percepcion_nav/config/params.yaml` con una sección `ros__parameters` por nodo
  (`camera`, `detector`, `visualizer`, `rgbd_localizer`, `nav2_goal_sender`, `synthetic_depth`) y
  **todos** los parámetros y valores por defecto de
  [contracts/parameters.md](contracts/parameters.md), cada uno con un comentario en español que
  diga qué hace y a qué requisito responde. `class_filter` es una cadena `""`, no una lista
  (research R14). Al final, agregar un bloque de comentarios que documente los argumentos de
  lanzamiento de `dry_run` (`map_frame`, `odom_frame`, `base_frame`, `camera_link_frame`,
  `optical_frame`, `cam_x`, `cam_y`, `cam_z`) y las constantes internas, con sus valores por
  defecto ([contracts/parameters.md](contracts/parameters.md#argumentos-de-los-lanzamientos)).
- [ ] T012 [P] (Manual, autor) Grabar con el celular un video de menos de 1 min con personas y
  objetos COCO, incluidas sillas. Comprimirlo con `ffmpeg -i entrada.mp4 -vf scale=-2:720 -c:v
  libx264 -crf 28 -an percepcion_nav/media/sample.mp4`, que debe pesar 20 MB o menos (research
  R15). El archivo no se versiona.
- [X] T013 [P] Crear `percepcion_nav/docs/` y `percepcion_nav/docs/img/` con un `.gitkeep` en
  `img/`

**Checkpoint**: base lista; las historias pueden empezar.

---

## Phase 3: User Story 1 - Pipeline 2D de extremo a extremo (Priority: P1) 🎯 MVP

**Goal**: un comando lanza cámara, detector y visualización, y muestra las detecciones dibujadas
sobre el video real.

**Independent Test**: `ros2 launch percepcion_nav pipeline_2d.launch.py` muestra la imagen
anotada en `rqt_image_view`, y `ros2 topic echo /detections --once` muestra bbox, clase y score
con el `header` de la imagen ([quickstart.md](quickstart.md) §3 y §4).

### Tests for User Story 1 ⚠️

> Escribir primero; deben FALLAR antes de T015.

- [X] T014 [P] [US1] Crear `percepcion_nav/test/test_detection_msgs.py` con pruebas de las
  funciones de módulo de `percepcion_nav.detector_node`:
  - (a) `parse_class_filter("")` → conjunto vacío; `parse_class_filter(" chair, person ")` →
    `{"chair", "person"}`.
  - (b) `build_detection2d_array(header, boxes)`, donde `boxes` es una lista de
    `(cx, cy, w, h, class_name, score)`:
    - El `header` del arreglo y de cada `Detection2D` es igual al de entrada (FR-013).
    - `bbox.center.position.x/y` = `cx/cy`, `bbox.center.theta == 0.0`, `size_x/size_y` = `w/h`.
    - `results[0].hypothesis.class_id` = nombre y `results[0].hypothesis.score` = score.
  - (c) las cajas con tamaño `<= 0` se omiten, porque el tamaño debe ser "`> 0` (SC-004)".
  - (d) con `boxes=[]` el arreglo es vacío y conserva el `header` (FR-015).

### Implementation for User Story 1

- [X] T015 [P] [US1] Implementar `percepcion_nav/percepcion_nav/camera_node.py`, clase
  `CameraNode`, nodo `camera`:
  - Parámetros `source`, `fps`, `frame_id`, `loop` y `reconnect_period_sec`.
  - `source` numérico → índice de dispositivo `int`; si no, ruta de archivo.
  - Abrir con `cv2.VideoCapture`. Publicar en `/camera/image_raw`, QoS RELIABLE, profundidad 5,
    con `cv_bridge.cv2_to_imgmsg(frame, 'bgr8')`, `header.stamp = self.get_clock().now()` y
    `header.frame_id = frame_id` (FR-001–FR-003).
  - Máquina de estados de [data-model.md](data-model.md#nodo-de-cámara): `OPENING`,
    `STREAMING`, `RETRYING`.
  - Si la fuente no abre o `source` está vacío: error limitado y reintento con un temporizador
    de `reconnect_period_sec` (FR-004).
  - Si falla la lectura y no es fin de archivo: advertencia limitada, `release()` y reconexión
    (FR-005).
  - Fin de archivo: posición ≥ `CAP_PROP_FRAME_COUNT` y conteo > 0. Si `loop`, volver al cuadro
    0. Si no, registrar un info, poner `self.finished = True` y salir del `main` (que usa
    `while rclpy.ok() and not node.finished: rclpy.spin_once(node, timeout_sec=0.1)`).
  - `destroy_node()` libera la captura (FR-006).
- [X] T016 [P] [US1] Implementar `percepcion_nav/percepcion_nav/detector_node.py`, clase
  `DetectorNode`, nodo `detector`, más las funciones de módulo `parse_class_filter` y
  `build_detection2d_array` que exige T014:
  - Al arrancar, si `model_path` no existe: `get_logger().fatal(...)` con la ruta y
    `raise SystemExit(1)` antes de importar `ultralytics` (falla rápida FR-016, research R5).
  - Importar `from ultralytics import YOLO` dentro de `__init__`, no al nivel del módulo.
  - Suscripción a `/camera/image_raw` con QoS BEST_EFFORT, KEEP_LAST, profundidad 1 (FR-010).
  - `imgmsg_to_cv2(msg, desired_encoding='bgr8')` dentro de `try`; si falla, advertencia
    limitada y descartar (FR-016).
  - `model.predict(frame, conf=conf_threshold, imgsz=imgsz, device=device, verbose=False)`,
    dentro de `try/except Exception`: si falla, advertencia limitada, descartar el cuadro y seguir
    (principio IV).
  - Cajas desde `r.boxes.xywh`, `r.boxes.conf` y `r.boxes.cls`, con `model.names`.
  - Filtrar por `class_filter` (vacío = todas, FR-011) y publicar en `/detections` (RELIABLE,
    profundidad 10), incluso vacío (FR-015).
  - Registrar cada `FPS_REPORT_PERIOD_SEC` (limitado) los fps medidos de inferencia (evidencia de
    SC-003).
- [X] T017 [P] [US1] Implementar `percepcion_nav/percepcion_nav/visualizer_node.py`, clase
  `VisualizerNode`, nodo `visualizer`:
  - `message_filters.Subscriber` de `/camera/image_raw` (BEST_EFFORT) y de `/detections`
    (RELIABLE), con `message_filters.TimeSynchronizer([...], sync_queue_size)`, estampa exacta
    (FR-020).
  - Dibujar con `cv2.rectangle` cada bbox (esquinas a partir de centro y tamaño) y con
    `cv2.putText` `f"{class_id} {score:.2f}"`, con un color estable por clase (FR-021).
  - Publicar en `/detections_image` (`bgr8`, RELIABLE, profundidad 5) con el `header` de la
    imagen (FR-022).
  - Si `show_window`: `cv2.imshow` + `cv2.waitKey(1)`; `destroy_node()` llama a
    `cv2.destroyAllWindows()`.
  - Conversión fallida → advertencia limitada.
- [X] T018 [US1] Crear `percepcion_nav/launch/pipeline_2d.launch.py`:
  - Argumentos `video` (defecto `<share>/media/sample.mp4`), `model` (defecto
    `<share>/models/yolo11n.pt`), `params_file` (defecto `<share>/config/params.yaml`) y `viewer`
    (defecto `true`). `<share>` se obtiene con `FindPackageShare('percepcion_nav')`.
  - Nodos `camera` (`parameters=[params_file, {'source': video}]`), `detector`
    (`[params_file, {'model_path': model}]`) y `visualizer` (`[params_file]`).
  - `rqt_image_view` con argumento `/detections_image` si `viewer` es verdadero
    ([contracts/launch-files.md](contracts/launch-files.md)).
- [X] T019 [US1] Compilar y validar [quickstart.md](quickstart.md) §3:
  - Imagen anotada en menos de 30 s (SC-002).
  - Dos `ros2 topic echo /camera/image_raw --once --field header` consecutivos muestran una
    estampa creciente y un `frame_id` no vacío (US1, escenario 1).
  - `ros2 topic hz /detections` ≥ 5 (SC-003).
  - `ros2 topic echo /detections --once` con `header` igual al de la imagen, `size_x/size_y > 0`,
    `class_id` no vacío y score entre 0 y 1 (SC-004).
  - `colcon test` en verde, incluido T014.
- [X] T020 [US1] Validar [quickstart.md](quickstart.md) §4:
  - `video:=/tmp/no_existe.mp4` → error y reintentos sin terminar; al copiar el video ahí,
    vuelve a publicar en 10 s o menos (SC-005).
  - `loop:=false` → al terminar el video el nodo `camera` se apaga de forma limpia (US1,
    escenario 6).
  - Anotar los resultados para la tabla de validado frente a diseño.

**Checkpoint**: MVP funcional; demostrable por sí solo.

---

## Phase 4: User Story 2 - Documentación y entregables (Priority: P1)

**Goal**: README ejecutable, diagrama y decisiones para lo implementado hasta ahora. Las
historias US3–US5 agregan sus secciones y la fase final lo completa.

**Independent Test**: en un Ubuntu 24.04 con Jazzy limpio se sigue solo el README y se lanza el
pipeline 2D (US2, escenario 1). Lo ideal es que lo haga una persona ajena.

- [X] T021 [P] [US2] Escribir `percepcion_nav/README.md` en español. Secciones:
  1. Descripción y alcance.
  2. Requisitos: Ubuntu 24.04, Jazzy, sin GPU.
  3. Instalación: `sudo apt install` de las dependencias de
     [quickstart.md](quickstart.md) §0, `rosdep install --from-paths percepcion_nav -y
     --ignore-src`, venv con `--system-site-packages` y `COLCON_IGNORE`, torch CPU,
     `requirements.txt`, descarga de pesos, ubicación del video.
  4. Compilación y pruebas.
  5. Ejecución del pipeline 2D, con argumentos.
  6. Comandos de verificación (`topic echo`, `hz`, `rqt_graph`).
  7. Problemas conocidos: PEP 668, `numpy<2` y OpenCV 4.11, sin webcam en WSL2, `[build_scripts]`.
  8. Tabla "Validado frente a diseño" con las filas de cámara, detector y visualización y la
     evidencia de T019–T020 (principio VI).
- [X] T022 [P] [US2] Crear `README.md` en la raíz con una descripción breve y un enlace a
  `percepcion_nav/README.md` y a `specs/001-paquete-percepcion-nav/`
- [X] T023 [P] [US2] Crear `percepcion_nav/docs/diagrama_nodos.md`:
  - Diagrama Mermaid `flowchart LR` con los 6 nodos, los tópicos y sus tipos, la acción
    `navigate_to_pose` (línea punteada) y tf, igual que
    [contracts/ros-interfaces.md](contracts/ros-interfaces.md).
  - Una tabla de remapeos por modo (pipeline 2D, `dry_run` y simulación).
- [X] T024 [P] [US2] Crear `percepcion_nav/docs/decisiones.md` con una justificación breve por
  decisión (FR-054): reparto en nodos, tipos estándar, QoS (R7), marcos y REP 103/105, copia del
  `header` y emparejamiento por estampa (R8), cola de profundidad 1 en el detector, imagen anotada
  como tópico, YOLO11n en CPU (R5) y entorno de Python (R6). Dejar marcadas las secciones que se
  completan en US3–US5: profundidad, separación, `dry_run` y simulación.
- [ ] T025 [US2] (Manual) Con el pipeline 2D en marcha, capturar `rqt_graph` en
  `percepcion_nav/docs/img/rqt_graph_pipeline_2d.png` y la imagen anotada en
  `percepcion_nav/docs/img/pipeline_2d.png`. Verificar que el grafo coincide con
  `docs/diagrama_nodos.md` (US2, escenario 3) y enlazar ambas imágenes desde el README.

**Checkpoint**: entrega P1 completa (US1 y US2).

---

## Phase 5: User Story 3 - Posición 3D de un objeto detectado (Priority: P2)

**Goal**: convertir detecciones 2D, profundidad e intrínsecos en `Detection3DArray` en el marco
óptico, y ejecutarlo en `dry_run` con profundidad sintética.

**Independent Test**: `python -m pytest percepcion_nav/test/test_geometry.py` (sin ROS) en verde.
Con `dry_run.launch.py`, `/detections_3d` publica `z ≈ 2.5` con la estampa del cuadro (US3,
escenario 6).

### Tests for User Story 3 ⚠️

> Escribir primero; deben FALLAR antes de T027 y T029.

- [X] T026 [P] [US3] Crear `percepcion_nav/test/test_geometry.py` (solo `numpy` y `pytest`; prohibido
  importar `rclpy`) para `percepcion_nav.geometry`:
  - (a) `deproject(420, 140, 2.0, Intrinsics(500, 500, 320, 240))` → `(0.4, -0.4, 2.0)`. Además,
    una malla de puntos 3D proyectados y deproyectados con error < 1 mm (SC-006).
  - (b) `depth_scale_for_encoding`: "`32FC1` → metros, escala 1.0; `16UC1`/`mono16` → milímetros,
    escala 0.001; otra codificación → se descarta (FR-032)", es decir, `None` para `bgr8`.
  - (c) `central_window(cx, cy, w, h, fraction, img_w, img_h)`: "lados = `max(1, round(w·f))` ×
    `max(1, round(h·f))`" centrados, y recortada a la imagen cuando el bbox toca el borde. Devuelve
    `None` si queda vacía.
  - (d) `robust_depth(depth, window, scale, min_valid)`: un píxel es válido si es "Finito y `> 0`
    (descarta 0, NaN, ±inf)". Mediana de los válidos multiplicada por la escala; `None` si hay
    menos de `min_valid`; con `uint16` en mm devuelve metros (US3, escenarios 2 y 3).
  - (e) `intrinsics_from_k(k)` lee `k[0], k[4], k[2], k[5]`.
  - (f) `intrinsics_from_hfov(640, 480, 1.2)` da `fx = fy = 320/tan(0.6)`, `cx = 320`, `cy = 240`.
  - (g) `metric_size(w, h, z, intr)` = `(w·z/fx, h·z/fy)`.
- [X] T027 [US3] Ampliar `percepcion_nav/test/test_detection_msgs.py` con pruebas de
  `percepcion_nav.rgbd_localizer_node.build_detection3d(det2d, xyz, size_xy)`:
  - `results[0].hypothesis` copia clase y score.
  - `results[0].pose.pose.position` y `bbox.center.position` son `xyz`.
  - La orientación es identidad (`w = 1`) y `bbox.size = (sx, sy, 0)`.
  - Prueba aparte: el arreglo publicado conserva el `header` de la `Detection2DArray` de origen
    (US3, escenario 4).

### Implementation for User Story 3

- [X] T028 [US3] Implementar `percepcion_nav/percepcion_nav/geometry.py` (sin importar `rclpy` ni
  mensajes) con:
  - `Intrinsics` (`NamedTuple` `fx, fy, cx, cy`) e `intrinsics_from_k`.
  - `intrinsics_from_hfov`, `deproject`, `depth_scale_for_encoding`.
  - `central_window`, que devuelve `(x0, y0, x1, y1)` semiabierta, o `None`.
  - `robust_depth` y `metric_size`.
  - Docstrings con las ecuaciones `X = (u − cx)·Z/fx`, `Y = (v − cy)·Z/fy`. Hacer pasar T026.
- [X] T029 [P] [US3] Implementar `percepcion_nav/percepcion_nav/synthetic_depth_node.py`, clase
  `SyntheticDepthNode`, nodo `synthetic_depth`:
  - Suscribe `/camera/image_raw` (BEST_EFFORT, profundidad 5).
  - Por cada cuadro publica en `/camera/depth/image_raw` una imagen `32FC1` construida con
    `cv_bridge.cv2_to_imgmsg(np.full((h, w), depth_m, np.float32), encoding='32FC1')` (principio
    II).
  - Publica también en `/camera/camera_info` un `CameraInfo` con `width`, `height`,
    `distortion_model='plumb_bob'`, `d` = 5 ceros, `k`/`p` desde `geometry.intrinsics_from_hfov`
    y `r` identidad.
  - Ambos llevan el `header` del cuadro de entrada (FR-035), QoS RELIABLE, profundidad 5.
  - Depende de T028.
- [X] T030 [US3] Implementar `percepcion_nav/percepcion_nav/rgbd_localizer_node.py`, clase
  `RgbdLocalizerNode`, nodo `rgbd_localizer`, con la función de módulo `build_detection3d` de T027:
  - Emparejamiento: `message_filters.Subscriber` de `/detections` (RELIABLE) y
    `/camera/depth/image_raw` (BEST_EFFORT), con `ApproximateTimeSynchronizer(queue_size=
    sync_queue_size, slop=sync_slop_sec)` (FR-030, R8).
  - Intrínsecos: suscripción aparte a `/camera/camera_info` (BEST_EFFORT, profundidad 1) que
    guarda `geometry.intrinsics_from_k(msg.k)` (FR-031).
  - En el callback emparejado:
    - Sin intrínsecos: advertencia limitada y no publicar (US3, escenario 5).
    - Escala de `depth_scale_for_encoding`; si es `None`, advertencia limitada y descartar.
    - Convertir la profundidad con `imgmsg_to_cv2(depth, desired_encoding='passthrough')`, dentro
      de `try/except`: si falla, advertencia limitada y descartar el par (principio IV).
    - Por detección: `central_window` con `depth_window_fraction`, recortada a las dimensiones de
      la **profundidad** (`depth.shape`), no a las del color. Si el centro del bbox cae fuera de la
      imagen de profundidad, advertencia limitada y omitir. Después, `robust_depth` con
      `min_valid_pixels`, `deproject` y `metric_size`. Si `Z` es `None`, advertencia limitada y
      omitir la detección.
    - Publicar `Detection3DArray` en `/detections_3d` (RELIABLE, profundidad 10) con el `header`
      de la detección, incluso vacío (FR-033).
  - Detecciones sin pareja: registrar un callback extra en el `Subscriber` de detecciones que
    cuente las recibidas. Un temporizador de `UNMATCHED_CHECK_PERIOD_SEC` compara con las
    emparejadas y, si llegaron
    detecciones sin ninguna pareja, emite una advertencia limitada: "sin profundidad dentro de
    sync_slop_sec" (caso límite).
- [X] T031 [US3] Crear `percepcion_nav/launch/dry_run.launch.py`, versión base para US3:
  - Mismos argumentos que `pipeline_2d.launch.py` (`video`, `model`, `params_file`, `viewer`).
  - Nodos `camera`, `detector`, `visualizer`, `synthetic_depth` y `rgbd_localizer`, más
    `rqt_image_view` opcional.
  - Argumentos de marco y posición de la cámara: `map_frame` (`map`), `odom_frame` (`odom`),
    `base_frame` (`base_link`), `camera_link_frame` (`camera_link`), `optical_frame`
    (`camera_color_optical_frame`), `cam_x` (`0.10`), `cam_y` (`0.0`) y `cam_z` (`0.30`)
    ([contracts/parameters.md](contracts/parameters.md#argumentos-de-los-lanzamientos)). En el
    archivo no puede aparecer ningún nombre de marco como literal fuera de esos valores por
    defecto (principio I).
  - Cuatro `static_transform_publisher` de `tf2_ros` con argumentos con nombre, construidos con
    `LaunchConfiguration`:
    - `map_frame→odom_frame` y `odom_frame→base_frame` en identidad.
    - `base_frame→camera_link_frame` con `--x cam_x --y cam_y --z cam_z`.
    - `camera_link_frame→optical_frame` con `--roll -1.5708 --pitch 0 --yaw -1.5708` (convención
      fija de REP 103, research R11).
  - El nodo `camera` recibe además `{'frame_id': optical_frame}`, para que el marco de los cuadros
    coincida con el árbol.
- [X] T032 [US3] Validar:
  - `python -m pytest percepcion_nav/test/test_geometry.py` sin el entorno de ROS cargado.
  - `colcon test` en verde.
  - `ros2 launch percepcion_nav dry_run.launch.py`: `ros2 topic echo /detections_3d --once`
    muestra `z ≈ 2.5`, la estampa del cuadro y `frame_id` `camera_color_optical_frame` (US3,
    escenarios 4 y 6).
  - Sin el nodo `synthetic_depth`: aparece la advertencia de intrínsecos y no se publica
    (escenario 5).
- [X] T033 [US3] Agregar al `percepcion_nav/README.md` la sección "Componente RGB-D" (ecuaciones,
  ventana central con mediana, unidades, profundidad sintética) y la fila RGB-D de la tabla de
  validado frente a diseño. Completar en `percepcion_nav/docs/decisiones.md` la decisión sobre la
  estimación de profundidad y el marco óptico sin transformar.

**Checkpoint**: US1–US3 funcionan; el RGB-D está probado sin hardware.

---

## Phase 6: User Story 4 - Meta de navegación hacia el objeto (Priority: P3)

**Goal**: convertir la detección 3D de `chair` en una meta `NavigateToPose` en `map`, verificable
en `dry_run`.

**Independent Test**: `dry_run.launch.py` publica en `/goal_pose_preview` una pose en `map` a
1.0 m del objeto, orientada hacia él; `pytest` de `goal_planning` en verde (SC-007).

### Tests for User Story 4 ⚠️

> Escribir primero; deben FALLAR antes de T035.

- [X] T034 [P] [US4] Crear `percepcion_nav/test/test_goal_planning.py` (sin `rclpy`) para
  `percepcion_nav.goal_planning`:
  - (a) `compute_goal((0, 0), (2.5, 0), 1.0, 0.30)` → `(1.5, 0.0, 0.0)`.
  - (b) `compute_goal((1, 1), (4, 5), 1.0, 0.30)` → `(3.4, 4.2, atan2(4, 3))`. La distancia
    meta–objeto es 1.0 ± 0.01 m y el error de yaw es < 1° (SC-007).
  - (c) "Devuelve `None` si `‖o − r‖ ≤ d + t`": objetos a 1.30 m y 1.29 m → `None`; a 1.31 m →
    meta (FR-042).
  - (d) `yaw_to_quaternion(yaw)` = `(0, 0, sin(yaw/2), cos(yaw/2))`.
  - (e) `target_moved((0, 0), (0.3, 0.4), 0.5)` es `False` y `target_moved((0, 0), (0.3, 0.41), 0.5)`
    es `True`.
  - (f) `select_best([("person", 0.9, a), ("chair", 0.6, b), ("chair", 0.8, c)], "chair")` → `c`;
    sin candidatos de la clase → `None` (US4, escenario 6).

### Implementation for User Story 4

- [X] T035 [US4] Implementar `percepcion_nav/percepcion_nav/goal_planning.py` (sin `rclpy` ni
  mensajes) con `select_best`, `compute_goal` (`g = o − d·(o − r)/‖o − r‖`,
  `yaw = atan2(o_y − g_y, o_x − g_x)`), `yaw_to_quaternion` y `target_moved`. Hacer pasar T034.
- [X] T036 [US4] Implementar `percepcion_nav/percepcion_nav/nav2_goal_sender_node.py`, clase
  `Nav2GoalSenderNode`, nodo `nav2_goal_sender`:
  - Configuración:
    - Parámetros de [contracts/parameters.md](contracts/parameters.md#nav2_goal_sender-nav2_goal_sender_node).
      `dry_run` se lee con `get_parameter` en **cada** callback, para poder cambiarlo con
      `ros2 param set`.
    - tf2: `tf2_ros.Buffer()` y `TransformListener(buffer, self, spin_thread=True)` (R9);
      `import tf2_geometry_msgs` para registrar las conversiones.
    - Suscripción a `/detections_3d` (RELIABLE, profundidad 10). Publicador `/goal_pose_preview`
      (RELIABLE, TRANSIENT_LOCAL, profundidad 1). `ActionClient(NavigateToPose, action_name)`.
  - En el callback:
    - Elegir objetivo con `select_best` sobre `target_class`.
    - Transformar el punto: `buffer.transform(PointStamped(header=det.header, point=pos),
      map_frame, timeout=Duration(seconds=tf_timeout_sec))`.
    - Pose del robot: `buffer.lookup_transform(map_frame, robot_base_frame, det.header.stamp,
      timeout)`.
    - Ante `TransformException`: advertencia limitada y retorno (FR-041).
    - Calcular `compute_goal(robot, objeto, standoff_distance, arrival_tolerance)`. Si es `None`:
      info limitado ("robot dentro de la separación") y retorno (FR-042).
    - Construir `PoseStamped` (`frame_id = map_frame`, estampa de la detección, `z = 0`, cuaternión
      del yaw), publicarlo en `/goal_pose_preview` y registrar con info limitado el objetivo en
      `map` (evidencia de SC-008) (FR-046).
    - Si `dry_run`, terminar aquí (FR-045).
  - Envío (máquina de estados de [data-model.md](data-model.md#emisor-de-metas): `IDLE`,
    `SENDING`, `ACTIVE`):
    - Con estado `SENDING` o `ACTIVE` y sin `target_moved(último, nuevo, goal_update_threshold)`:
      no reenviar (FR-044).
    - Si `not client.server_is_ready()` y `not client.wait_for_server(timeout_sec=
      server_timeout_sec)`: advertencia limitada y retorno (FR-043).
    - `send_goal_async(goal, feedback_callback)` con un número de secuencia propio.
    - Respuesta: si se rechaza → advertencia y `IDLE`; si se acepta → `ACTIVE`.
    - `get_result_async` → registrar el estado (`SUCCEEDED`, `ABORTED` o `CANCELED`) y volver a
      `IDLE`, ignorando las respuestas y resultados de metas reemplazadas.
    - Progreso: info limitado con `distance_remaining`.
  - `destroy_node()` destruye el cliente de acción.
- [X] T037 [P] [US4] Crear `percepcion_nav/rviz/percepcion_nav.rviz`: marco fijo `map`; vistas
  `Grid`, `TF`, `Image` sobre `/detections_image` y `Pose` sobre `/goal_pose_preview` (QoS
  transient local).
- [X] T038 [US4] Ampliar `percepcion_nav/launch/dry_run.launch.py`: agregar el nodo
  `nav2_goal_sender` (`[params_file, {'dry_run': True, 'map_frame': map_frame,
  'robot_base_frame': base_frame}]`, con los argumentos de T031) y el argumento `use_rviz` (defecto
  `true`), que abre `rviz2 -d <share>/rviz/percepcion_nav.rviz`
  ([contracts/launch-files.md](contracts/launch-files.md#dry_runlaunchpy-us3-y-us4-sin-simulación))
- [X] T039 [US4] Validar [quickstart.md](quickstart.md) §5:
  - Con una silla en el video, `/goal_pose_preview` publica en `map` a 1.0 m del punto 3D, sobre
    la línea robot–objeto y con `z = 0`.
  - Transformación no disponible (US4, escenario 2): con el `dry_run` en marcha, ejecutar
    `ros2 run percepcion_nav nav2_goal_sender_node --ros-args -r __node:=goal_sender_tf_test -p
    map_frame:=frame_inexistente`. Aparece la advertencia de tf y no publica meta. **No** sirve
    matar un `static_transform_publisher`, porque tf2 conserva para siempre las transformaciones
    estáticas ya recibidas.
  - `ros2 param set /nav2_goal_sender dry_run false`: vence la espera del servidor y el nodo sigue
    vivo (US4, escenario 3).
  - `python -m pytest percepcion_nav/test/test_goal_planning.py` en verde.
- [X] T040 [US4] Agregar al `percepcion_nav/README.md` la sección "Conexión con Nav2 y `dry_run`"
  (cadena de tf, cálculo de la meta, separación y tolerancia de llegada, ciclo de vida de la
  acción) y la fila del emisor de metas en la tabla de validado frente a diseño. Completar en
  `percepcion_nav/docs/decisiones.md` las decisiones de separación, tolerancia de llegada y
  `dry_run`.

**Checkpoint**: US1–US4 completos con el camino (c) de validación.

---

## Phase 7: User Story 5 - Simulación en Gazebo de extremo a extremo (Priority: P4, obligatoria)

**Goal**: un comando abre Gazebo con el TurtleBot 4, la silla de FreeCAD y Nav2; el robot
detecta la silla, estima su posición y navega hasta quedar frente a ella.

**Independent Test**: [quickstart.md](quickstart.md) §7: SC-008 (error ≤ 0.30 m) y SC-009 (4 de
5 ejecuciones a 1.0 ± 0.30 m y ≤ 15°).

> T041 puede (y conviene) hacerse en paralelo desde la fase 3: solo mide si la simulación es
> viable en WSL2 (research R12). Es la única excepción al orden del principio VIII y está
> justificada en Complexity Tracking del plan. T041 **no** escribe código ni configuración de
> simulación. T042–T053 empiezan solo tras el checkpoint de US4.

- [X] T041 [P] [US5] Prueba previa de la simulación:
  - Ejecutar `ros2 launch nav2_bringup tb4_simulation_launch.py headless:=True use_rviz:=False`
    (necesita internet la primera vez: modelo Depot de Fuel).
  - Medir `ros2 topic hz /rgbd_camera/image` y `ros2 topic hz /clock` para estimar el factor de
    tiempo real.
  - Verificar con `ros2 topic echo /rgbd_camera/image --once --field header` que `frame_id` es
    `oakd_rgb_camera_optical_frame` (research R1). Verificar también la codificación de
    `/rgbd_camera/depth_image` (esperada `32FC1`).
  - Registrar todo en `percepcion_nav/docs/validacion_simulacion.md` (sección "Prueba previa").
  - Si `ogre2` falla con `llvmpipe`, probar `LIBGL_ALWAYS_SOFTWARE=1` y el motor `ogre`. Si nada
    funciona, **detenerse y avisar al usuario**, porque US5 es obligatoria.
- [ ] T042 [US5] (Manual, entorno) Instalar FreeCAD 1.0 (PPA `freecad-maintainers/freecad-stable`
  o AppImage) y conectar el servidor MCP de FreeCAD a Claude Code. Verificar que funciona
  `freecadcmd --version`. Si el MCP no se puede conectar, seguir solo con `freecadcmd`
  (research R4).
  **Estado**: FreeCAD 1.1.4 se usó sin instalar (AppImage extraída en el directorio temporal de la sesión, con `freecadcmd`). Falta que el autor instale FreeCAD y conecte el servidor MCP en su entorno.
- [X] T043 [US5] Escribir la macro `percepcion_nav/freecad/make_chair.py` (Python de FreeCAD, en
  mm). **Nota de implementación**: la geometría de abajo es la versión 1, que YOLO no reconoció;
  la entregada es la versión 2 (butaca con estructura y cojines, dos mallas; ver research R4):
  - Geometría:
    - Asiento: 450 × 450 × 40 mm, con la cara superior a 450 mm.
    - Cuatro patas: 40 × 40 × 430 mm, en las esquinas.
    - Respaldo: 450 de ancho × 40 de grueso × 450 mm de alto, sobre el borde trasero (lado −X).
    - Faldón frontal: 450 de ancho × 20 de grueso × 300 mm de alto, bajo el borde frontal del
      asiento (de 110 a 410 mm de altura), al ras del frente. Impide que la ventana central de
      profundidad vea el fondo entre las patas (research R4 y R10, SC-008).
    - El frente de la silla mira a +X; el origen está en el centro de la huella, a nivel del
      piso.
  - Fusión de los sólidos y exportación con `Mesh.export` a
    `percepcion_nav/models/freecad_chair/meshes/chair.stl`.
  - Ejecutarla por el MCP o con `freecadcmd percepcion_nav/freecad/make_chair.py`. La malla
    resultante **sí** se versiona (`git add -f` si hace falta).
- [X] T044 [P] [US5] Crear `percepcion_nav/models/freecad_chair/model.config` y `model.sdf`
  (SDF 1.9). **Nota de implementación**: con la versión 2 son dos visuales (estructura y cojines)
  y 11 cajas de colisión:
  - Modelo `static`.
  - Visual: malla `model://freecad_chair/meshes/chair.stl` con
    `<scale>0.001 0.001 0.001</scale>` y material madera (`diffuse` y `ambient` `0.55 0.35 0.20 1`).
  - Colisión: 7 cajas en metros que repiten las dimensiones de T043 (asiento, respaldo, faldón
    frontal, 4 patas),
    según [data-model.md](data-model.md#silla-objetivo-simulación).
- [X] T045 [P] [US5] Crear `percepcion_nav/config/nav2_params_sim.yaml`:
  - Copia de `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`.
  - En `amcl.ros__parameters`, agregar `set_initial_pose: true` e
    `initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}`.
  - Encabezado en comentario que diga de dónde viene, la fecha y el único cambio (research R2,
    FR-065).
- [X] T046 [P] [US5] Crear `percepcion_nav/config/params_sim.yaml` con las sobrescrituras de
  [contracts/parameters.md](contracts/parameters.md#sobrescrituras-en-simulación-configparams_simyaml):
  `/**: ros__parameters: use_sim_time: true`, `nav2_goal_sender.dry_run: false` y
  `detector.class_filter: "chair"`.
- [X] T047 [US5] Crear `percepcion_nav/launch/simulation.launch.py`:
  - Argumentos: `headless` (`true`), `use_rviz` (`true`), `chair_model` (`freecad_chair`),
    `chair_x` (`-5.0`), `chair_y` (`0.0`), `chair_yaw` (`3.1416`), `model` y `params_file`.
  - `AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', <share>/models)`.
  - Incluir `nav2_bringup/launch/tb4_simulation_launch.py` con
    `params_file:=<share>/config/nav2_params_sim.yaml` y pasando `headless` y `use_rviz`.
  - Insertar la silla con `ros_gz_sim create` dentro de un `OpaqueFunction`: si `chair_model`
    empieza con `http`, se pasa tal cual a `-file` (modelo de Fuel, FR-067); si no, se usa
    `<share>/models/<chair_model>/model.sdf`. También `-name target_chair`, `-x`, `-y`, `-z 0` y
    `-Y`.
  - Nodos `detector`, `visualizer`, `rgbd_localizer` y `nav2_goal_sender` con
    `parameters=[params_file, <share>/config/params_sim.yaml, {'model_path': model}]` y los remapeos
    `/camera/image_raw→/rgbd_camera/image`, `/camera/depth/image_raw→/rgbd_camera/depth_image` y
    `/camera/camera_info→/rgbd_camera/camera_info`.
  - No lanzar `camera` ni `synthetic_depth` (FR-035, FR-063).
- [X] T048 [US5] Compilar y lanzar `ros2 launch percepcion_nav simulation.launch.py`:
  - Comparar `/amcl_pose` con `gz model -m nav2_turtlebot4 -p`; la diferencia debe ser ≤ 0.10 m.
    Si no, ajustar `initial_pose` en `nav2_params_sim.yaml` (FR-065, R2).
  - Confirmar con `gz model -m target_chair -p` que la silla está en `(-5.0, 0.0)` del mundo.
  - Verificar en el log que no hay errores de extrapolación de tf en operación estable, es decir,
    después de la primera pose de AMCL y 30 s de tiempo simulado (US5, escenario 4; FR-064).
- [X] T049 [US5] Verificar la detección de la silla (US5, escenario 1): `/detections` debe
  contener `chair` con score ≥ 0.40. Verificar también que el objetivo estimado en `map` (log del
  emisor) cae dentro de la huella de la silla, a 0.25 m o menos de `(3.0, 0.0)`. Si no, reducir
  `depth_window_fraction` en `params_sim.yaml` y revisar el faldón (research R10).
  - Si no lo alcanza, ajustar en este orden: material o color en `model.sdf`, `chair_yaw` y
    `imgsz` (en `params_sim.yaml`).
  - Si aun así falla, relanzar con `chair_model:=https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair`
    (o `WoodenChair`) y registrar la sustitución para declararla (FR-067).
- [X] T050 [US5] Ejecutar 5 corridas completas y registrar en
  `percepcion_nav/docs/validacion_simulacion.md` una tabla con columnas: corrida, objetivo
  estimado en `map` (log del emisor), error frente al centro de la huella `(3.0, 0.0)` (SC-008,
  ≤ 0.30 m), pose final del robot (`gz model`), distancia al centro de la huella (1.0 ± 0.30 m),
  error de orientación (≤ 15°), resultado de la acción, metas enviadas durante la navegación
  (contadas en el log; solo hay reemplazos si el objetivo se movió más que
  `goal_update_threshold`, FR-044) y si hubo metas repetidas tras llegar (FR-042). SC-009 se
  cumple con al menos 4 de 5. Si alguna corrida termina con la meta rechazada o abortada,
  verificar que el nodo vuelve a `IDLE` y acepta una meta nueva (caso límite).
- [X] T051 [US5] Confirmar US5, escenario 5: `git diff --stat` entre US4 y US5 no toca ningún
  `*_node.py`; el cambio de fuente se hizo solo con remapeos y parámetros. Anotarlo en
  `percepcion_nav/docs/validacion_simulacion.md`.
- [ ] T052 [US5] (Manual) Capturar `percepcion_nav/docs/img/simulacion.png` (Gazebo o RViz con el
  robot frente a la silla) y `percepcion_nav/docs/img/rqt_graph_simulacion.png`.
  **Estado**: `docs/img/simulacion.png` listo (cámara simulada con detecciones). Falta la captura manual de `rqt_graph` en simulación.
- [X] T053 [US5] Agregar al `percepcion_nav/README.md` la sección "Simulación":
  - Dependencias apt, internet en el primer arranque y render por software.
  - Comando, argumentos y pose real de la silla.
  - Cómo regenerar la silla con FreeCAD.
  - Resultados de T050, y el respaldo de Fuel si se usó (FR-067).
  - La fila "Simulación" de la tabla de validado frente a diseño.

  Completar en `percepcion_nav/docs/decisiones.md` las decisiones de simulación (R1–R4, R12).

**Checkpoint**: las cinco historias cumplen sus criterios; la entrega está completa en lo
funcional.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: documentación final, entregables y verificación global.

- [ ] T054 [P] Sincronizar `percepcion_nav/docs/diagrama_nodos.md` con el grafo real: capturar
  `rqt_graph` del `dry_run` completo en `percepcion_nav/docs/img/rqt_graph_dry_run.png` y
  confirmar que coinciden nodos, tópicos y tipos (FR-053, US2 escenario 3)
  **Estado**: grafo verificado por introspección del sistema en ejecución (`docs/reporte/figuras/grafo_dry_run.png`, coincide con el diagrama). Falta la captura manual de `rqt_graph`.
- [X] T055 [P] Revisar `percepcion_nav/docs/decisiones.md`: debe cubrir todas las decisiones de
  FR-054 (reparto de nodos, tipos de mensaje, QoS, marcos, emparejamiento por estampa,
  estimación de profundidad, distancia de separación y `dry_run`), cada una con su porqué y sus
  alternativas descartadas
- [X] T056 Completar `percepcion_nav/README.md`:
  - Tabla de parámetros con enlace a `config/params.yaml`.
  - Los tres lanzamientos.
  - Verificación.
  - Problemas conocidos completos.
  - Tabla final de validado frente a diseño con evidencia por componente (archivo o minuto del
    video).
  - Limitaciones conocidas (principio VI, FR-052).
- [X] T057 Escribir `percepcion_nav/docs/reporte_tecnico.md` (1 a 2 páginas, FR-055) con:
  1. Diagrama.
  2. Decisiones.
  3. Componente RGB-D.
  4. Conexión con Nav2.
  5. Tabla de validado frente a diseño.
  6. Comparación con la posición real en simulación (de T050).
  7. Autoevaluación de los seis criterios de la rúbrica (0 a 4), cada uno con evidencia
     señalable (SC-012).
  **Estado**: el reporte está en LaTeX (`docs/reporte/reporte_tecnico.tex` → PDF de 2 páginas, con figuras generadas del sistema en ejecución). Tiene dos marcas rojas `[pendiente: …]` que se quitan al tener el video y la captura de `rqt_graph`.

  Exportarlo a `reporte_tecnico.pdf` y verificar que ocupa 1 o 2 páginas.
- [X] T058 Verificación limpia: `rm -rf build install log && colcon build && colcon test &&
  colcon test-result --verbose`, con 0 fallas, incluidas `flake8` y `pep257` (SC-001). Ejecutar
  además `python -m pytest percepcion_nav/test/test_geometry.py
  percepcion_nav/test/test_goal_planning.py` sin el entorno de ROS.
- [ ] T059 Reproducción en entorno limpio (SC-010; lo ideal es que lo haga una persona ajena): en
  un entorno limpio (una distro WSL
  nueva con Ubuntu 24.04 o un contenedor `ros:jazzy`), seguir **solo** el README para el pipeline
  2D, el `dry_run` y la simulación. Corregir el README por cada paso faltante o implícito
  encontrado.
- [ ] T060 (Manual) Grabar el video de demostración de 2 a 4 min siguiendo FR-056:
  1. Compilación.
  2. Pipeline 2D.
  3. `topic echo` y `hz` de `/detections`.
  4. `rqt_graph`.
  5. Fuente inexistente.
  6. `dry_run`.
  7. Navegación en simulación.
  8. Cierre con lo validado y lo diseñado.

  Comprimirlo con `ffmpeg -crf 28` a 50 MB o menos y guardarlo como `video_demo.mp4` en la raíz
  del repositorio (ignorado por git) (SC-011).
- [ ] T061 Empaquetar: `zip -r percepcion_nav.zip percepcion_nav -x '*/__pycache__/*'`. Verificar
  que no contiene `build/`, `install/`, `log/` ni `.venv/`, y que incluye `models/yolo11n.pt`,
  `media/sample.mp4` y las mallas de la silla. Confirmar que el `.zip`, el video y el PDF pesan
  50 MB o menos cada uno y que en Moodle hay 20 archivos o menos (FR-058, SC-011).
- [X] T062 Revisar las compuertas de calidad de la constitución (I–VIII) contra el código final.
  Actualizar la tabla "Constitution Check" de
  `specs/001-paquete-percepcion-nav/plan.md` si algo cambió y confirmar que las desviaciones
  siguen justificadas en "Complexity Tracking".

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (fase 1)**: sin dependencias.
- **Foundational (fase 2)**: depende de Setup; bloquea todas las historias.
- **US1 (fase 3)**: depende de la fase 2.
- **US2 (fase 4)**: depende de US1, porque documenta lo que ya corre. T022–T024 pueden empezar en
  paralelo a US1.
- **US3 (fase 5)**: depende de la fase 2 y del detector de US1 (T016) para el `dry_run`. Sus
  pruebas puras (T026, T028) no dependen de nada.
- **US4 (fase 6)**: depende de US3 (consume `/detections_3d` y amplía `dry_run.launch.py`). Sus
  pruebas puras (T034, T035) no dependen de nada.
- **US5 (fase 7)**: depende de US1, US3 y US4, porque reutiliza los cuatro nodos sin cambios. T041
  (prueba previa) no depende de nada y debe adelantarse.
- **Polish (fase 8)**: depende de las cinco historias.

### User Story Dependencies

```text
Setup ─► Foundational ─► US1 ─► US2 (base)
                          │
                          └─► US3 ─► US4 ─► US5 ─► Polish
T041 (prueba previa de simulación) ── en cualquier momento tras T010 (excepción justificada al principio VIII)
```

### Within Each User Story

- Las pruebas (T014, T026, T027, T034) se escriben primero y deben fallar.
- Módulos puros (`geometry.py`, `goal_planning.py`) antes que los nodos que los usan.
- Nodos antes que lanzamientos; lanzamientos antes que la validación; validación antes que la
  sección del README.
- Los archivos compartidos (`README.md`, `decisiones.md`, `dry_run.launch.py`,
  `test_detection_msgs.py`) se editan en secuencia, nunca en paralelo.

### Parallel Opportunities

- Setup: T002–T007 en paralelo.
- Foundational: T012 (grabar el video) y T013 en paralelo con T011.
- US1: T015, T016 y T017 (tres nodos, tres archivos) en paralelo tras T014.
- US2: T021–T024 en paralelo.
- US3: T026 en paralelo con todo US1; T029 en paralelo con T030 cuando T028 esté listo.
- US4: T034 y T037 en paralelo.
- US5: T041 en paralelo desde la fase 3; T044, T045 y T046 en paralelo.
- Polish: T054 y T055 en paralelo.

---

## Parallel Example: User Story 1

```bash
# Tras escribir T014 (pruebas de mensajes), los tres nodos son archivos independientes:
Task: "T015 Implementar camera_node.py en percepcion_nav/percepcion_nav/camera_node.py"
Task: "T016 Implementar detector_node.py en percepcion_nav/percepcion_nav/detector_node.py"
Task: "T017 Implementar visualizer_node.py en percepcion_nav/percepcion_nav/visualizer_node.py"
```

## Parallel Example: User Story 3 y 4 (lógica pura)

```bash
# Las pruebas y módulos puros no dependen de ROS ni de US1:
Task: "T026 Pruebas de geometría en percepcion_nav/test/test_geometry.py"
Task: "T034 Pruebas de metas en percepcion_nav/test/test_goal_planning.py"
```

## Parallel Example: User Story 5

```bash
Task: "T044 Modelo SDF en percepcion_nav/models/freecad_chair/model.sdf"
Task: "T045 Parámetros de Nav2 en percepcion_nav/config/nav2_params_sim.yaml"
Task: "T046 Sobrescrituras en percepcion_nav/config/params_sim.yaml"
```

---

## Implementation Strategy

### MVP First (User Story 1)

1. Fase 1 (Setup) y fase 2 (Foundational).
2. Fase 3 (US1): pipeline 2D.
3. **Detenerse y validar** con [quickstart.md](quickstart.md) §3–§4. Ya es una entrega
   demostrable (3 de los 6 criterios de la rúbrica).
4. Lanzar T041 (prueba previa de simulación) cuanto antes para saber si US5 es viable en WSL2.

### Incremental Delivery

1. US1 + US2 → entrega P1 (pipeline y documentación base).
2. US3 → RGB-D probado sin hardware y ejecutado en `dry_run`.
3. US4 → meta de Nav2 en `dry_run` (camino (c) completo).
4. US5 → simulación (obligatoria): RGB-D y Nav2 ejecutados de verdad.
5. Polish → reporte, video, `.zip` y reproducción limpia.

Cada incremento deja el paquete compilable. Hacer commit al cerrar cada tarea o grupo lógico con
Conventional Commits (`feat:`, `test:`, `docs:`, `chore:`), según la constitución.

---

## Notes

- [P] = archivo distinto y sin dependencias pendientes.
- Las tareas marcadas **(Manual)** requieren al autor: grabar video, capturar pantallas,
  instalar FreeCAD y su MCP, grabar la demo.
- Verificar que las pruebas fallan antes de implementar.
- Si una validación no se cumple, registrarlo con honestidad en la tabla de validado frente a
  diseño (principio VI). Nunca simular resultados.
