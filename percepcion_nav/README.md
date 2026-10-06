# percepcion_nav

Paquete ROS 2 (Jazzy, `ament_python`) que integra en nodos desacoplados un pipeline de
percepción para navegación:

1. **Cámara** (`camera_node`): publica los cuadros de un video o dispositivo como `sensor_msgs/Image`.
2. **Detector** (`detector_node`): YOLO11n (COCO) publica `vision_msgs/Detection2DArray`.
3. **Visualización** (`visualizer_node`): dibuja las detecciones sobre su cuadro y publica
   `/detections_image`.
4. **Localizador RGB-D** (`rgbd_localizer_node`): detección 2D + profundidad + intrínsecos →
   `vision_msgs/Detection3DArray` en el marco óptico.
5. **Emisor de metas** (`nav2_goal_sender_node`): detección 3D de la clase objetivo → meta
   `nav2_msgs/action/NavigateToPose` en `map` (con modo `dry_run`).
6. **Profundidad sintética** (`synthetic_depth_node`): herramienta de prueba que permite ejecutar
   la cadena completa en `dry_run` sin cámara RGB-D.

Solo se usan mensajes estándar. Diagrama de nodos y tópicos en
[docs/diagrama_nodos.md](docs/diagrama_nodos.md); decisiones de diseño en
[docs/decisiones.md](docs/decisiones.md).

## Requisitos

- Ubuntu 24.04 (probado en WSL2) con ROS 2 Jazzy en `/opt/ros/jazzy`.
- CPU; no hace falta GPU (todo corre en CPU).
- Unos 3 GB libres para el entorno de Python (torch CPU + ultralytics).

## Instalación

Todos los comandos se ejecutan desde la raíz del repositorio, que funciona también como
workspace de colcon (el paquete está en `percepcion_nav/`).

### 1. Paquetes del sistema

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-nav2-msgs \
  ros-jazzy-rqt-image-view ros-jazzy-rqt-graph ros-jazzy-rviz2
# equivalente a partir de package.xml (si rosdep ya está inicializado):
# rosdep install --from-paths percepcion_nav -y --ignore-src
```

### 2. Entorno de Python

Ubuntu 24.04 no permite `pip install` global (PEP 668), así que se usa un entorno virtual que ve
los paquetes de ROS (`--system-site-packages`):

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE                 # colcon no debe recorrer el entorno virtual
source .venv/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install -r percepcion_nav/requirements.txt
python -c "import numpy, cv_bridge, ultralytics; print(numpy.__version__)"   # debe imprimir 1.x
```

`requirements.txt` fija `numpy<2` (el `cv_bridge` de Jazzy está compilado contra NumPy 1.x),
`opencv-python==4.11.0.86` (la última que acepta NumPy 1.x) y `ultralytics==8.4.174`.

### 3. Pesos del modelo y video de muestra

Ambos vienen dentro del `.zip` entregado. En un clon del repositorio (no se versionan):

```bash
curl -L -o percepcion_nav/models/yolo11n.pt \
  https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
mkdir -p percepcion_nav/media
cp /ruta/a/tu_video.mp4 percepcion_nav/media/sample.mp4   # o usa el argumento video:=...
```

## Compilación y pruebas

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build                          # importante: SIN --symlink-install (ver Problemas conocidos)
source install/setup.bash
colcon test --packages-select percepcion_nav && colcon test-result --verbose
```

Las pruebas de la lógica numérica también corren sin ROS:

```bash
python -m pytest percepcion_nav/test/test_geometry.py percepcion_nav/test/test_goal_planning.py
```

## Ejecución

En cada terminal nueva: `source /opt/ros/jazzy/setup.bash && source .venv/bin/activate &&
source install/setup.bash`.

### Pipeline 2D (cámara → detector → visualización)

```bash
ros2 launch percepcion_nav pipeline_2d.launch.py
```

| Argumento | Defecto | Uso |
|---|---|---|
| `video` | `share/percepcion_nav/media/sample.mp4` | Ruta del video, o índice de dispositivo (`"0"`) |
| `model` | `share/percepcion_nav/models/yolo11n.pt` | Pesos de YOLO |
| `params_file` | `share/percepcion_nav/config/params.yaml` | Parámetros de los nodos |
| `viewer` | `true` | Abre `rqt_image_view` sobre `/detections_image` |

Los parámetros de cada nodo, con su valor por defecto y su propósito, están en
[config/params.yaml](config/params.yaml).

## Componente RGB-D

El localizador (`rgbd_localizer_node`) recibe `/detections`, la profundidad alineada al color
(`/camera/depth/image_raw`) y los intrínsecos (`/camera/camera_info`). Empareja cada mensaje de
detecciones con la profundidad cuya estampa difiera 20 ms o menos (`sync_slop_sec`).

Para cada detección:

1. Toma una ventana central del bbox (30 % de su ancho y alto, `depth_window_fraction`),
   recortada a la imagen de profundidad.
2. Estima `Z` como la **mediana** de los píxeles válidos de la ventana (finitos y mayores que 0).
   La mediana es robusta al fondo que entra en el bbox y a los huecos de la profundidad. Si hay
   menos de `min_valid_pixels` válidos, la detección se descarta con una advertencia.
3. Convierte unidades según la codificación: `32FC1` está en metros; `16UC1` y `mono16` están en
   milímetros (× 0.001).
4. Deproyecta el centro del bbox con el modelo pinhole:
   `X = (u − cx)·Z/fx`, `Y = (v − cy)·Z/fy`, `Z`.

Publica `vision_msgs/Detection3DArray` en `/detections_3d` **en el marco óptico de la cámara**,
con el `header` (estampa y marco) de la detección 2D, la clase y el score de origen, y la
posición en `results[0].pose.pose.position` y `bbox.center.position`.

En `dry_run` no hay cámara RGB-D. La herramienta `synthetic_depth_node` publica, por cada cuadro
del video, una profundidad constante (`depth_m`, 2.5 m) y los intrínsecos de una cámara ideal
con campo de visión `hfov_rad` (1.20 rad), con la misma estampa del cuadro. Así se ejecuta el
localizador real con las detecciones reales del video.

## Conexión con Nav2 y `dry_run`

El emisor de metas (`nav2_goal_sender_node`) recibe `/detections_3d` y elige la detección de la
clase objetivo (`target_class`, por defecto `chair`) con mayor score.

**Cadena de tf.** Transforma el punto del marco óptico a `map` con tf2, consultando en la
**estampa de la detección** y con un tiempo límite (`tf_timeout_sec`). La pose del robot sale de
la transformación `map → base_link` en la misma estampa. Si alguna transformación no está
disponible, advierte (con límite de frecuencia) y no calcula meta.

**Cálculo de la meta.** Con `o` el objeto, `r` el robot y `d` la separación (`standoff_distance`,
1.0 m):

- Posición: `g = o − d·(o − r)/‖o − r‖`, es decir, a `d` del objeto sobre la línea robot–objeto.
- Orientación: `yaw = atan2(o_y − g_y, o_x − g_x)`, mirando al objeto.
- Altura: `z = 0`.
- Si `‖o − r‖ ≤ d + t`, con `t` = `arrival_tolerance` (0.30 m), **no** se envía meta. Esto evita
  correcciones repetidas después de llegar.

La meta calculada se publica siempre en `/goal_pose_preview` (`geometry_msgs/PoseStamped`,
transient local).

**Ciclo de vida de la acción** (`dry_run:=false`):

- El nodo espera al servidor `navigate_to_pose` como máximo `server_timeout_sec` y envía la meta
  con `NavigateToPose`.
- Registra la aceptación o el rechazo, el progreso (`distance_remaining`) y el resultado
  (`SUCCEEDED`, `ABORTED` o `CANCELED`).
- Mientras hay una meta en curso no reenvía, salvo que el objetivo se haya movido más de
  `goal_update_threshold` (0.5 m). En ese caso la nueva meta reemplaza a la anterior.
- Sin servidor, advierte y sigue vivo.

### `dry_run` (sin robot ni Gazebo)

```bash
ros2 launch percepcion_nav dry_run.launch.py            # use_rviz:=false para no abrir RViz
ros2 topic echo /detections_3d --once                    # z ≈ 2.5 m (profundidad sintética)
ros2 topic echo /goal_pose_preview --once                # meta en map, 1.0 m antes del objeto
```

Lanza la cámara, el detector, la visualización, `synthetic_depth`, el localizador y el emisor con
`dry_run:=true`. Además lanza cuatro transformaciones estáticas:
`map → odom → base_link → camera_link → camera_color_optical_frame`, con la cámara en
`(0.10, 0, 0.30)` m. Los nombres de marco y esa posición son argumentos del launch (`map_frame`,
`odom_frame`, `base_frame`, `camera_link_frame`, `optical_frame`, `cam_x`, `cam_y`, `cam_z`).
Esos mismos valores se pasan a los nodos.

Para probar casos de error:

```bash
# Transformación no disponible: segunda instancia hacia un marco inexistente
ros2 run percepcion_nav nav2_goal_sender_node --ros-args -r __node:=goal_sender_tf_test \
  -p map_frame:=frame_inexistente -r goal_pose_preview:=/goal_preview_tf_test
# Servidor de acción ausente
ros2 param set /nav2_goal_sender dry_run false
```

(Matar un `static_transform_publisher` no sirve para probar lo primero: tf2 conserva para siempre
las transformaciones estáticas ya recibidas.)

## Simulación en Gazebo (robot real simulado + Nav2)

### Dependencias adicionales

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb4-sim \
  ros-jazzy-ros-gz
```

El **primer** lanzamiento necesita internet: el mundo `depot` descarga el modelo del almacén de
Gazebo Fuel (queda en `~/.gz/fuel`).

### Ejecución

```bash
ros2 launch percepcion_nav simulation.launch.py              # headless:=False para ver Gazebo
```

Lanza:

- Gazebo Harmonic con el TurtleBot 4 mínimo de Nav2, su pila de navegación y RViz de Nav2
  (`use_rviz:=False` para omitirlo).
- La silla de FreeCAD a 3 m frente al robot.
- El detector, la visualización, el localizador y el emisor de metas (`dry_run:=false`),
  conectados a la cámara simulada solo por remapeos (`/rgbd_camera/*`).

AMCL se localiza solo (`config/nav2_params_sim.yaml` fija la pose inicial). El robot detecta la
silla, estima su posición y navega hasta quedar a 1 m de ella, mirándola.

| Argumento | Defecto | Uso |
|---|---|---|
| `headless` | `True` | Sin la interfaz de Gazebo. `True`/`False` con mayúscula (Nav2 lo evalúa en Python) |
| `use_rviz` | `true` | RViz de Nav2 |
| `viewer` | `true` | `rqt_image_view` sobre `/detections_image` |
| `dry_run` | `false` | `true` arranca sin enviar metas; luego `ros2 param set /nav2_goal_sender dry_run false` inicia la navegación |
| `chair_model` | `freecad_chair` | Modelo del paquete, o URL de Fuel como respaldo (FR-067) |
| `chair_x`, `chair_y`, `chair_yaw` | `-5.0`, `0.0`, `3.1416` | Pose **real** de la silla en el mundo; en `map` es `(chair_x + 8, chair_y)` |
| `chair_spawn_delay` | `10.0` | Espera antes de insertar la silla, en s |

En WSL2 sin GPU, Gazebo dibuja por software y corre a ~0.5× el tiempo real. Todos los nodos usan
el reloj de la simulación (`use_sim_time`), así que el sistema sigue siendo correcto, aunque más
lento.

### Panel de simulación (para la demostración)

Un solo comando abre el panel **e inicia la simulación de la silla**: carga el entorno de ROS y
del `.venv`, y compila si hace falta.

```bash
percepcion_nav/scripts/simulacion_silla.sh
```

O solo el panel, con el entorno ya cargado (con `--autostart` inicia la simulación al abrir):

```bash
ros2 run percepcion_nav sim_panel
```

En Windows con WSL se puede crear un acceso directo `.bat` con
`wsl.exe -d Ubuntu-24.04 -- bash -lc "~/proyecto_vision/proyecto_vision/percepcion_nav/scripts/simulacion_silla.sh"`.
`Ctrl+C` en la terminal o cerrar la ventana detiene toda la simulación.

Ventana con botones:

- **▶ Iniciar simulación**.
- **→ Iniciar navegación**: activa la navegación si se arrancó esperando.
- **■ Detener todo**: termina Gazebo, Nav2, puentes y nodos, sin dejar procesos vivos.
- **⟳ Reiniciar**.

Opciones (se aplican al iniciar): ver Gazebo, RViz de Nav2, ver detecciones (`rqt_image_view`) y
*esperar para navegar*. Con esta última, el robot ve la silla y publica la meta, pero no se
mueve hasta pulsar "Iniciar navegación", lo que sirve para narrar. Muestra el estado (iniciando,
lista, navegando, llegó) y el registro de los nodos de percepción.

Equivalente sin panel:

```bash
ros2 launch percepcion_nav simulation.launch.py headless:=False dry_run:=true
ros2 param set /nav2_goal_sender dry_run false        # en otra terminal, cuando se quiera navegar
```

### La silla de FreeCAD

La silla se genera con la macro `freecad/make_chair.py`: una butaca con estructura de madera y
cojines azules. Las mallas generadas (`models/freecad_chair/meshes/*.stl`) se versionan, así que
para ejecutar no hace falta FreeCAD. Para regenerarlas:

```bash
freecadcmd percepcion_nav/freecad/make_chair.py          # o Macro > Ejecutar en FreeCAD / MCP
```

También se puede abrir `freecad/chair.FCStd` en FreeCAD.

Para validar contra la pose real de la silla, el localizador estima en simulación el **centro**
del objeto y no su cara visible (`estimate_center: true` en `config/params_sim.yaml`).

## Verificación

```bash
ros2 topic echo /detections --once          # header = el de la imagen; bbox, clase y score
ros2 topic hz /detections                   # >= 5 Hz en CPU
ros2 topic echo /camera/image_raw --once --field header
ros2 run rqt_graph rqt_graph                # debe coincidir con docs/diagrama_nodos.md
ros2 launch percepcion_nav pipeline_2d.launch.py video:=/tmp/no_existe.mp4   # error + reintentos
```

## Reporte técnico

`docs/reporte/reporte_tecnico.tex` (LaTeX, 2 páginas) incluye el grafo, las capturas, los
resultados de la simulación y la autoevaluación. Para compilarlo:

```bash
cd percepcion_nav/docs/reporte && pdflatex reporte_tecnico.tex && pdflatex reporte_tecnico.tex
```

Las figuras están en `docs/reporte/figuras/` y se generaron del sistema en ejecución (ver
`docs/validacion_simulacion.md`).

## Problemas conocidos

- **`No module named 'ultralytics'` al lanzar**: el paquete se compiló con
  `--symlink-install`. En ese modo colcon usa `setup.py develop`, que ignora
  `[build_scripts] executable = /usr/bin/env python3` de `setup.cfg` y deja los ejecutables con
  `/usr/bin/python3`, que no ve el `.venv`. Solución: `rm -rf build install log && colcon build`
  con el `.venv` activo.
- **`cv_bridge` falla con NumPy 2**: instala con `requirements.txt` (fija `numpy<2` y OpenCV
  4.11). OpenCV 4.12 o posterior exige NumPy 2.
- **Sin webcam en WSL2**: WSL2 no expone `/dev/video*`. La fuente por defecto es un video;
  una webcam USB puede conectarse con `usbipd` y usarse con `video:=0`.
- **Rendimiento**: en CPU, YOLO11n a 640 px rinde entre 5 y 9 cuadros/s en el equipo de
  desarrollo. Si se necesita más, usa `imgsz: 320` en `config/params.yaml`.
- **`name 'true' is not defined` al lanzar la simulación**: `headless` debe ser `True` o `False`
  con mayúscula, porque `tb4_simulation_launch.py` de Nav2 lo evalúa como expresión de Python.
- **Metas rechazadas al arrancar la simulación**: durante los primeros segundos Nav2 todavía no
  está activo y rechaza metas. El emisor lo avisa (con límite de frecuencia) y reintenta con la
  siguiente detección.
- **`gz model` no responde** con la simulación cargada (render por software); para la pose real
  se usa `/odom` de Gazebo.

## Limitaciones

- No hay pruebas con hardware: ni webcam, ni cámara RGB-D, ni robot físico.
- La pose real del robot en simulación es la odometría ideal de Gazebo, no un sistema de
  medición independiente.
- El localizador supone profundidad alineada al color, con la misma resolución.
  `estimate_center` supone además una huella cuadrada (profundidad del objeto igual a su ancho).
- A 1 m de la silla, esta queda cortada en la imagen y deja de detectarse. No afecta la llegada,
  pero el sistema no sigue al objeto a esa distancia.

## Validado frente a diseño

| Componente | Cómo se validó | Evidencia | Qué quedó sin probar |
|---|---|---|---|
| Nodo de cámara | Ejecución real con un video; fuente inexistente con reintentos y recuperación en 1.9 s (≤ 10 s); fin de video con `loop:=false` termina con código 0 | Registros de la sesión de validación; video de demostración | Webcam física (WSL2 no la expone) |
| Detector | Ejecución real; `Detection2DArray` con `header` idéntico al de la imagen, `theta = 0`, tamaño > 0, clase y score en [0, 1]; 5.3–8.9 detecciones/s en CPU | `ros2 topic echo /detections`, `ros2 topic hz`; prueba `test_detection_msgs.py` | Rendimiento con GPU |
| Visualización | Ejecución real: primera imagen anotada a los 5.8 s del lanzamiento (≤ 30 s) | `/detections_image` | Nada |
| Emisor de metas (Nav2) | 9 pruebas `pytest` sin ROS (meta a 1.0 ± 0.01 m y orientación < 1°, regla de distancia con tolerancia, umbral de reemplazo, selección por score). En `dry_run` real: objeto en `map` (2.60, −0.89), meta (1.654, −0.564) a 1.000 m del objeto, yaw −18.8° hacia él, `z = 0`. Con marco inexistente advierte y no publica. Con `dry_run:=false` y sin servidor advierte cada ~6 s y sigue vivo | `test_goal_planning.py`; `ros2 topic echo /goal_pose_preview`; registros del emisor | Envío a un servidor Nav2 real, navegación y resultado de la acción (se ejecutan en simulación, historia 5) |
| Localizador RGB-D | 19 pruebas `pytest` sin ROS (deproyección con error < 1 mm, mediana con ceros, NaN e infinitos, mm → m, ventana recortada al borde); ejecución real en `dry_run`: `/detections_3d` con `z = 2.50` m (la profundidad sintética), marco óptico, clase y score de origen; sin intrínsecos, advierte y no publica | `test_geometry.py`, `test_detection_msgs.py`; `ros2 topic echo /detections_3d` | Profundidad de un sensor real: ruido, huecos y alineación profundidad–color (se ejercita en simulación, historia 5) |

| Simulación (RGB-D + Nav2 de extremo a extremo) | 5 corridas independientes en Gazebo con Nav2 real: silla de FreeCAD detectada (score 0.87 desde 3 m); estimación a 0.04 m del centro real (SC-008, 5/5); robot detenido a 1.03–1.10 m del centro y orientado con error ≤ 2.2° (SC-009, 5/5); 5/5 `SUCCEEDED`; ninguna meta extra tras llegar | [docs/validacion_simulacion.md](docs/validacion_simulacion.md), figuras del reporte | Robot y sensor físicos. La pose real se toma de la odometría ideal de Gazebo |

Nota: estas mediciones se hicieron con un video de prueba construido a partir de cuadros del
conjunto KITTI (autos y personas). El video del autor grabado con el celular (`sample.mp4`)
reemplaza a ese video en la demostración.

**Créditos de datos**: las capturas del pipeline 2D (`docs/img/pipeline_2d.png`,
`docs/reporte/figuras/pipeline_2d.png`) usan cuadros de *KITTI Vision Benchmark Suite* (Geiger
et al., 2013), con licencia CC BY-NC-SA 3.0, en un uso académico y no comercial.
