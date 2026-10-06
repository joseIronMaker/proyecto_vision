# Research: Paquete ROS 2 de percepción para navegación

**Feature**: `001-paquete-percepcion-nav` | **Fecha**: 2026-10-06 | **Plan**: [plan.md](plan.md)

Cada decisión se verificó contra el entorno real (`/opt/ros/jazzy`, PyPI y Gazebo Fuel) el
2026-10-06. Donde algo solo pudo inferirse, se indica cómo verificarlo en la implementación.

---

## R1. Simulación: TurtleBot 4 mínimo de Nav2

**Decision**: usar `nav2_minimal_tb4_sim` + `nav2_minimal_tb4_description` a través de
`nav2_bringup/launch/tb4_simulation_launch.py`, con el mundo y el mapa `depot` por defecto.

**Rationale**: ya está instalado (`ros-jazzy-nav2-minimal-tb4-sim` 1.0.1) junto con Gazebo
Harmonic (`gz` 8.11.0) y `ros_gz`. Hechos verificados en los archivos instalados:

| Hecho | Valor |
|---|---|
| Sensor | `rgbd_camera` (Gazebo), 320×240, `horizontal_fov` 1.25 rad, 10 Hz, recorte 0.3–100 m |
| Color | `/rgbd_camera/image` (puente `ros_gz_image`) |
| Profundidad | `/rgbd_camera/depth_image` (puente `ros_gz_image`, `32FC1` en metros) |
| Intrínsecos | `/rgbd_camera/camera_info` (puente `parameter_bridge`) |
| Marco óptico | `oakd_rgb_camera_optical_frame` (`<optical_frame_id>` del sensor) |
| Pose de aparición | mundo `(-8.0, 0.0, yaw 0)` |
| Navegación | `xy_goal_tolerance` 0.25 m, `yaw_goal_tolerance` 0.25 rad (14.3°), `robot_radius` 0.22 m, `inflation_radius` 0.70 m |

Las tolerancias de Nav2 caben dentro de SC-009 (±0.30 m y 15° o menos).

**Alternatives considered**:
- `turtlebot4_simulator` (no instalado, candidato apt 2.0.2): más pesado (nodos de Create 3,
  interfaz propia) y sin ventaja para esta actividad.
- `nav2_minimal_tb3_sim`: el TurtleBot 3 no trae cámara RGB-D.

**Verificar al implementar**: que el `header.frame_id` de `/rgbd_camera/image` y
`/rgbd_camera/depth_image` sea `oakd_rgb_camera_optical_frame`. Si no lo es, se agrega un
parámetro de anulación del marco en el localizador y se documenta.

## R2. Localización automática al arrancar (FR-065)

**Decision**: copiar `nav2_bringup/params/nav2_params.yaml` a `config/nav2_params_sim.yaml` y cambiar
solo AMCL: `set_initial_pose: true` e `initial_pose: {x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}`. El
archivo empieza con un encabezado que documenta la diferencia con el original.

**Rationale**: el archivo original no define `set_initial_pose`, así que AMCL espera una pose
desde RViz, lo que es un paso manual prohibido por FR-065. El mapa `depot` mide 30.2 × 15.35 m con
origen `(-7.14, -7.83)`. El modelo Depot está centrado en el origen del mundo, así que el marco
`map` es el mundo desplazado +8 m en x y la aparición del robot en el mundo `(-8, 0)` cae en
`map (0, 0)`.

**Alternatives considered**:
- Publicar `/initialpose` desde el launch con un `TimerAction`: depende de cuándo arranca AMCL y
  es frágil.
- `RewrittenYaml` de `nav2_common`: solo reescribe claves existentes y no puede agregar
  `initial_pose`.

**Verificar al implementar**: tras el arranque, comparar `/amcl_pose` con la pose real del robot
(`gz model -m nav2_turtlebot4 -p`). La diferencia debe quedar en 0.10 m o menos.

## R3. Ubicación de la silla en el mundo

**Decision**: la silla aparece en el mundo `(-5.0, 0.0, 0.0)`, que es `map (3.0, 0.0)`, a 3 m
frente al robot (dentro de 2.5–4 m, FR-062), con el frente hacia el robot. Se inserta con
`ros_gz_sim create` desde el launch, sin copiar el mundo `depot`.

**Rationale**: en el mapa `depot`, el corredor `x ∈ [0, 5] m`, `|y| ≤ 0.6 m` tiene 0 celdas
ocupadas de 2525 (verificado leyendo `depot.pgm`). La meta queda en `map ≈ (1.8, 0.0)`, lejos de
paredes.

**Alternatives considered**: copiar `depot.sdf` con la silla incluida. Se descarta porque
duplica un archivo que viene del sistema y obliga a mantenerlo.

**Tamaño esperado en la imagen**: con 320 px y 1.25 rad de campo de visión, `fx ≈ 222 px`. Una
silla de 0.9 m mide unos 66 px de alto a 3 m y unos 50 px a 4 m, un tamaño detectable por YOLO
nano.

## R4. Silla en FreeCAD y respaldo

**Decision**:
1. El modelo de la silla se define como una macro de Python para FreeCAD versionada en
   `percepcion_nav/freecad/make_chair.py`. La macro se ejecuta por el servidor MCP de FreeCAD (vía
   preferida) o con `freecadcmd` (alternativa sin interfaz).
2. La macro exporta una malla visual (`.obj` o `.stl`) y una colisión simplificada (cajas para
   asiento, respaldo y patas). Las mallas exportadas **sí** se versionan (menos de 1 MB), así que
   ejecutar el paquete no requiere FreeCAD.
3. El modelo SDF `models/freecad_chair/` lleva un material de color (madera o tela oscura) que
   contrasta con el piso, porque un render sin textura reduce el score.
4. La silla lleva un faldón frontal bajo el asiento, para que la ventana central de profundidad
   no vea el fondo entre las patas. Así la mediana queda dentro de la huella (R10, SC-008).
**Verificado al implementar** (FreeCAD 1.1.4, `freecadcmd` desde la AppImage extraída, sin
`sudo` ni instalación):

- La versión 1 (cajas de un solo color: asiento, respaldo macizo, faldón y patas) **no** se
  reconocía: el mejor score de `chair` fue 0.09, probando 3 colores, 4 orientaciones y 2
  distancias.
- En la misma escena, `OpenRobotics/Chair` de Fuel se detectó con 0.84–0.87. Eso demuestra que la
  resolución de la cámara alcanza y que el problema era la apariencia del modelo.
- La versión 2 imita los rasgos que YOLO sí reconoce: butaca con estructura de madera oscura
  (patas, brazos, travesaños) y cojines azules con bordes redondeados, en dos mallas con su
  propio material. Se detecta con 0.55–0.71.
5. Respaldo (FR-067): `OpenRobotics/Chair` o `OpenRobotics/WoodenChair` de Gazebo Fuel (ambos
   existen según la búsqueda en Fuel del 2026-10-06), con la misma pose.

**Rationale**: una macro es reproducible y defendible, y el MCP solo la ejecuta. Versionar la
malla elimina FreeCAD como dependencia para quien evalúa.

**Alternatives considered**: modelar a mano en la interfaz y exportar. No es reproducible ni se
puede revisar en git.

**Pendiente de entorno**: al 2026-10-06 FreeCAD no está instalado (`which freecad` vacío) ni hay
MCP conectado. Instalar FreeCAD 1.0 (AppImage o PPA `freecad-maintainers`) y el servidor MCP
antes de la historia 5. Si el MCP no funciona, `freecadcmd` basta.

## R5. Detector

**Decision**: `ultralytics==8.4.174` con pesos `yolo11n.pt` (COCO, 80 clases, `chair` = id 56),
inferencia en CPU, `imgsz` configurable (640 por defecto). El nodo verifica que el archivo de
pesos exista **antes** de llamar a `YOLO()` y, si no existe, falla de inmediato (FR-016).

**Rationale**: YOLO11n es el nano maduro y documentado de Ultralytics. En CPU de 8 núcleos rinde
holgadamente por encima de 5 fps (SC-003). La verificación previa evita que Ultralytics descargue
pesos en silencio al directorio de trabajo.

**Alternatives considered**:
- `yolo26n.pt`: sin NMS y más rápido en CPU. Se puede usar como reemplazo directo cambiando
  `model_path`. No se elige por defecto por tener menos historial.
- YOLOv8n: más antiguo, sin ventaja.

## R6. Entorno de Python y compatibilidad con `cv_bridge`

**Decision**:
- Entorno virtual en la raíz del repositorio: `python3 -m venv --system-site-packages .venv`,
  más `touch .venv/COLCON_IGNORE` para que colcon no lo recorra.
- `percepcion_nav/requirements.txt` con versiones fijas:
  `numpy<2`, `opencv-python==4.11.0.86`, `ultralytics==8.4.174`. `torch` y `torchvision` se
  instalan desde `https://download.pytorch.org/whl/cpu` (ruedas sin CUDA).
- `setup.cfg` con `[build_scripts] executable = /usr/bin/env python3`, para que los ejecutables
  instalados usen el Python del entorno virtual activo y no `/usr/bin/python3`.

**Rationale** (verificado en PyPI):
- `opencv-python` 4.12.0.88 y posteriores exigen `numpy>=2`, lo que rompe `cv_bridge` de Jazzy.
  4.11.0.86 acepta `numpy>=1.26` en Python 3.12.
- `ultralytics` 8.4.174 exige `opencv-python>=4.7.0,!=4.13.0.90` y `numpy>=1.23`, compatibles con
  las versiones fijadas.
- El sistema ya trae `numpy` 1.26.4 y `pytest` 7.4.4.
- Sin `[build_scripts]`, `ros2 run` lanza los nodos con `/usr/bin/python3` y no encuentra
  `ultralytics`.
- **Verificado al implementar**: `colcon build --symlink-install` usa `setup.py develop`, que
  **ignora** `[build_scripts]` y deja el shebang en `/usr/bin/python3` (el detector falla con
  `No module named 'ultralytics'`). Por eso se compila con `colcon build`, sin
  `--symlink-install`. La alternativa es `python -m colcon build --symlink-install` con el `.venv`
  activo, que deja la ruta del `.venv` fija en los ejecutables.

**Alternatives considered**: `pip install --break-system-packages` (lo prohíbe PEP 668 y
ensucia el sistema); `PYTHONPATH` manual (frágil y fácil de olvidar).

## R7. Calidad de servicio (QoS)

**Decision**: los publicadores son *reliable*; los suscriptores de imágenes son *best effort*
(perfil de sensor). Detalle por tópico en [contracts/ros-interfaces.md](contracts/ros-interfaces.md).

**Rationale**: un suscriptor *best effort* recibe de cualquier publicador, y un publicador
*reliable* sirve a cualquier suscriptor. Así los nodos son compatibles con los puentes de
`ros_gz_image`, `rqt_image_view`, RViz y `ros2 topic echo` sin ajustes. Con profundidad de cola 1
en el detector se procesa siempre el cuadro más reciente (FR-010).

**Alternatives considered**: perfil de sensor en los publicadores de imagen. Lo rechaza porque
deja fuera a los suscriptores *reliable*, como algunas configuraciones de `rqt_image_view`.

## R8. Emparejamiento por estampa

**Decision**:
- Visualización: `message_filters.TimeSynchronizer` (exacto) entre imagen y detecciones, con cola
  de 30 mensajes. Funciona porque el detector copia el `header` (FR-013, FR-020).
- Localizador: `message_filters.ApproximateTimeSynchronizer` entre detecciones y profundidad,
  `slop` de 0.02 s (parámetro, clarificación del 2026-10-06) y cola de 30 mensajes (parámetro).
  Los intrínsecos se guardan aparte (se usa el último recibido).

**Rationale**: las detecciones llegan con el retraso de la inferencia (100–300 ms en CPU). La
cola de 30 cubre más de 1 s de profundidad a 30 Hz y 3 s a 10 Hz. Los intrínsecos no cambian, así
que no hace falta emparejarlos.

## R9. tf2 en `rclpy`

**Decision**: `tf2_ros.Buffer` y `TransformListener(buffer, node, spin_thread=True)`. Se consulta
con la estampa de la detección y un tiempo límite configurable (0.2 s por defecto). La pose del
robot es `map → base_link` en la misma estampa. Las excepciones `TransformException` se capturan,
se registran con límite de frecuencia y se descarta la detección.

**Rationale**: con un ejecutor de un solo hilo, una consulta bloqueante dentro de un callback
nunca recibe `/tf` si el listener no tiene su propio hilo. Esa es una trampa conocida de `rclpy`.

**Cambio al implementar (verificado)**: `spin_thread=True` funciona, pero al apagar con Ctrl+C
el hilo del listener lanza `ExternalShutdownException` y deja una traza en el registro. Se
reemplazó por `TransformListener(buffer, node)` (sus suscripciones usan un
`ReentrantCallbackGroup`) más `MultiThreadedExecutor` en `main()`. Además, en `rclpy` el límite
de frecuencia del registro se guarda por línea de llamada, y una misma línea no puede cambiar de
severidad (`ValueError: Logger severity cannot be changed between calls`). Por eso cada aviso
limitado se emite en su propia línea, sin funciones auxiliares compartidas.

## R10. Política de metas

**Decision**: lógica pura en `goal_planning.py`:
- Meta: `g = o − d·(o − r)/‖o − r‖`, `yaw = atan2(o_y − g_y, o_x − g_x)`, `z = 0`, donde `o` es el
  objeto, `r` el robot (ambos en `map`) y `d` la separación (1.0 m).
- No se envía meta si `‖o − r‖ ≤ d + t`, con `t` = tolerancia de llegada (0.30 m).
- Con una meta activa no se reenvía salvo que el objetivo se mueva más de 0.5 m (parámetro). Si se
  reenvía, Nav2 reemplaza la meta anterior y el resultado de la meta vieja se ignora.

**Rationale**: la cámara mide la superficie visible. Con el faldón frontal (R4), la mediana de la
ventana central cae entre la cara frontal y el respaldo, es decir, dentro de la huella y a
0.225 m o menos del centro. El error de SC-008, medido contra el centro de la huella, queda así
dentro de 0.30 m.

La meta queda entre 0.78 y 1.23 m del centro, fuera de la zona inscrita (0.22 m más la mitad de
la silla) y con costo de inflación bajo. El margen de llegada evita las correcciones repetidas
(clarificación del 2026-10-06).

**Verificado en simulación y corregido**:

- Con la estimación sobre la cara visible, el objetivo quedó en `map` (2.75, 0.00), a 0.25 m del
  centro.
- Nav2 se detiene a menos de `xy_goal_tolerance` de la meta (0.25 m por defecto; el robot paraba
  unos 0.2 m antes). El resultado fue 1.37 m y luego 1.35 m del centro, por encima del límite de
  1.30 m de SC-009.

Correcciones:

1. `estimate_center` en el localizador (FR-036): desplaza el punto sobre su rayo la mitad del
   ancho métrico, es decir, del frente de la silla a su centro.
2. `xy_goal_tolerance` = 0.10 m en `nav2_params_sim.yaml`.
3. El emisor ignora para el envío las detecciones con estampa anterior al último resultado de
   navegación. Antes, una detección tomada justo antes de llegar provocaba una meta extra
   después del éxito, contra FR-042.

**Riesgo para SC-009 (análisis previo)**: en el peor caso se suman el sesgo de la estimación (±0.225 m) y la
tolerancia de Nav2 (0.25 m). El caso típico (estimación cerca del centro) cabe en ±0.30 m, y por
eso el criterio exige 4 de 5 corridas. T049 verifica pronto que la estimación cae dentro de la
huella. Si no, se reduce `depth_window_fraction`.

## R11. Marcos y profundidad sintética en `dry_run`

**Decision**:
- Transformaciones estáticas: `map → odom` (identidad), `odom → base_link` (identidad),
  `base_link → camera_link` (0.10, 0, 0.30 m), `camera_link → camera_color_optical_frame`
  (rpy −π/2, 0, −π/2, REP 103).
- Profundidad sintética: constante de 2.5 m por defecto, mayor que `d + t` = 1.3 m, para que el
  `dry_run` produzca una meta. Los intrínsecos se calculan por cuadro a partir del tamaño de la
  imagen y de un campo de visión horizontal configurable: 1.20 rad por defecto, típico de la
  cámara principal de un celular. Así `fx = fy = (w/2)/tan(hfov/2)` y `cx = w/2`, `cy = h/2`.

**Rationale**: funciona con cualquier resolución de video sin calibrar el celular. Con una
silla a 2.5 m enfrente, la meta esperada queda a 1.5 m del robot, un valor fácil de verificar
a mano.

## R12. Render de Gazebo en WSL2

**Decision**: asumir render por software. `glxinfo` reporta `llvmpipe (LLVM 20.1.2)` con
OpenGL 4.5 y `Accelerated: no`. La simulación arranca con `headless:=True` (sin la interfaz de
Gazebo) y RViz es opcional. La cámara de 320×240 a 10 Hz ya es ligera. Si `ogre2` falla, se
prueba `ogre` (render engine clásico).

**Rationale**: la especificación acepta que la simulación corra más lento que el tiempo real.

**Riesgo principal del plan**: la primera tarea de la historia 5 debe ser una prueba rápida que
lance la simulación original de Nav2 y mida `ros2 topic hz /rgbd_camera/image` y el factor de
tiempo real, antes de invertir en la silla y la integración.

## R13. Descargas en el primer arranque

**Decision**: documentar que el primer lanzamiento de la simulación necesita internet. El mundo
`depot.sdf` incluye `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot` y hoy la caché
`~/.gz/fuel` solo tiene `ground plane` y `sun`.

## R14. Parámetros: detalles de ROS 2

**Decision**:
- El filtro de clases es una cadena separada por comas (`""` = todas). Una lista vacía `[]` en un
  YAML de parámetros falla al cargar en ROS 2.
- Todos los avisos repetibles usan `logger.warning(..., throttle_duration_sec=N)`.
- El video y los pesos se pasan como argumentos de launch, con valores por defecto en
  `$(find-pkg-share percepcion_nav)/media/sample.mp4` y `.../models/yolo11n.pt`. En el código no
  hay rutas fijas.

## R15. Video de muestra, pesos y entregables

**Decision**: el video del celular (720p, menos de 1 min, comprimido a 20 MB o menos) y
`yolo11n.pt` (5.4 MB) **no** se versionan en git (`.gitignore`), pero **sí** van dentro del
`.zip` entregado, en `media/` y `models/`, para que el evaluador ejecute sin descargar nada. El
README también da el comando de descarga de los pesos.

## R16. Estrategia de pruebas

**Decision**:
- `test_geometry.py` y `test_goal_planning.py`: `pytest` puro, sin importar `rclpy`, ejecutable
  sin cargar el entorno de ROS (principio V). Cubren SC-006 y SC-007.
- `test_detection_msgs.py`: construcción de `Detection2D` y `Detection3D` (FR-014, FR-033),
  ejecutada con `colcon test` (requiere los mensajes de ROS).
- `test_flake8.py` y `test_pep257.py`: estilo con `ament_flake8` y `ament_pep257`.
- Las pruebas de extremo a extremo se hacen con los procedimientos de
  [quickstart.md](quickstart.md) y quedan grabadas en el video. No se usa `launch_testing`, para
  acotar el alcance.
- SC-008 y SC-009 se miden con la pose real de `gz model` frente a la estimada y la final, en 5
  ejecuciones registradas en `docs/validacion_simulacion.md`.
