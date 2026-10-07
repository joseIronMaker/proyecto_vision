# Decisiones de diseño

Cada decisión con su porqué y las alternativas descartadas (FR-054). El detalle técnico y las
verificaciones están en `specs/001-paquete-percepcion-nav/research.md` (R1–R16).

## Reparto en nodos

**Decisión**: cinco nodos, uno por responsabilidad (cámara, detector, visualización,
localizador RGB-D y emisor de metas), más una herramienta de prueba (`synthetic_depth`). Se
comunican solo por tópicos, tf2 y una acción. La matemática vive en módulos sin ROS
(`geometry.py`, `goal_planning.py`).

**Por qué**: cada pieza se sustituye sin tocar las demás. El video se cambia por la cámara de
Gazebo con solo remapeos; Nav2 real se cambia por `dry_run` con un parámetro.

**Descartado**: un nodo monolítico, porque impide reemplazar piezas y probarlas por separado.

## Tipos de mensaje

**Decisión**: solo tipos estándar: `sensor_msgs/Image` y `CameraInfo`,
`vision_msgs/Detection2DArray` y `Detection3DArray`, `geometry_msgs/PoseStamped` y
`nav2_msgs/action/NavigateToPose`.

**Por qué**: así hay interoperabilidad con RViz, Nav2 y otras herramientas; además, la rúbrica
penaliza los mensajes personalizados.

**Descartado**: mensajes propios con la clase como texto plano. Duplican lo que ya resuelve
`vision_msgs`.

## Calidad de servicio (QoS)

**Decisión**: los publicadores son *reliable*. Los suscriptores de imagen son *best effort* y el
detector usa profundidad de cola 1.

**Por qué**: un suscriptor *best effort* recibe de cualquier publicador, como los puentes de
Gazebo o `rqt_image_view`. Un publicador *reliable* sirve a cualquier suscriptor. Con
profundidad 1, el detector procesa siempre el cuadro más reciente y descarta los atrasados, así
la latencia no se acumula cuando la inferencia es más lenta que la cámara.

**Descartado**: perfil de sensor en los publicadores de imagen, porque deja sin datos a
suscriptores *reliable*.

## Marcos y estampas

**Decisión**: marcos según REP 103 y 105 (`map → odom → base_link → camera_link → marco óptico`).
El detector copia el `header` de la imagen sin volver a estampar. Los datos que se transforman a
otro marco conservan la estampa.

**Por qué**: sin la misma estampa no se pueden emparejar detecciones con su imagen y su
profundidad, ni consultar tf2 en el instante de captura.

## Emparejamiento por estampa

**Decisión**: la visualización usa `TimeSynchronizer` exacto (la detección trae el `header`
idéntico al de la imagen). El localizador usa `ApproximateTimeSynchronizer` con diferencia máxima
de 20 ms y cola de 30 mensajes. Los intrínsecos se guardan aparte (el último recibido).

**Por qué**: las detecciones llegan con el retraso de la inferencia; la cola de 30 cubre más de
1 s de profundidad. El margen de 20 ms tolera cámaras RGB-D reales, donde color y profundidad no
comparten estampa, sin mezclar cuadros distintos (menos de un periodo de 30 fps).

## Imagen anotada como tópico

**Decisión**: la visualización publica `/detections_image`; la ventana local es opcional.

**Por qué**: un tópico aparece en `rqt_graph`, se puede grabar en un bag y se ve en remoto. Una
ventana local no tiene ninguna de esas ventajas.

## Modelo de detección

**Decisión**: YOLO11n preentrenado con COCO (`ultralytics==8.4.174`) en CPU. El nodo verifica
que el archivo de pesos exista antes de cargar el modelo y, si no existe, falla de inmediato.

**Por qué**: la actividad evalúa integración, no el modelo. El modelo nano rinde más de 5
cuadros/s en CPU y COCO incluye `chair`. La verificación previa evita que Ultralytics descargue
pesos en silencio.

**Descartado**: YOLO26n (reemplazo directo vía `model_path`, pero con menos historial) y entrenar
un modelo propio (fuera de alcance).

## Entorno de Python

**Decisión**: `.venv` con `--system-site-packages`, torch CPU y versiones fijas (`numpy<2`,
`opencv-python==4.11.0.86`, `ultralytics==8.4.174`). `setup.cfg` define
`[build_scripts] executable = /usr/bin/env python3` y se compila sin `--symlink-install`.

**Por qué**: PEP 668 impide instalar con pip en el sistema, y `cv_bridge` de Jazzy necesita
NumPy 1.x. Sin el cambio de shebang, los nodos corren con el Python del sistema y no encuentran
`ultralytics`. `--symlink-install` ignora ese cambio (verificado).

## Estimación de profundidad

**Decisión**: la profundidad de una detección es la mediana de los píxeles válidos (finitos y
mayores que 0) de una ventana central del bbox (30 % de su ancho y alto), recortada a la imagen.
Se deproyecta el centro del bbox con el modelo pinhole. El resultado se publica en el marco
óptico, sin transformar.

**Por qué**: el bbox incluye fondo y la profundidad tiene huecos (ceros o NaN). La ventana
central reduce el fondo y la mediana ignora los valores atípicos. Publicar en el marco óptico
deja al localizador sin necesidad de conocer el robot: transformar a `map` es responsabilidad de
quien navega.

**Descartado**:
- Promedio de todo el bbox: el fondo y los ceros lo sesgan.
- Profundidad del píxel central: un solo hueco la anula.
- Transformar a `map` en el localizador: acoplaría la percepción al árbol de marcos del robot.

**Pruebas**: el localizador se prueba sin hardware de dos formas. Las funciones puras se
verifican con `pytest` sin ROS. Además, se ejecuta en `dry_run` con profundidad sintética.

## Distancia de separación y tolerancia de llegada

**Decisión**: la meta se coloca a 1.0 m del objeto, sobre la línea robot–objeto y mirando hacia
él. No se envía meta si el robot ya está a 1.0 + 0.30 m o menos. Una meta en curso solo se
reemplaza si el objetivo se mueve más de 0.5 m.

**Por qué**: el objeto es un obstáculo en el mapa de costos de Nav2, así que una meta sobre él es
inalcanzable. Con 1.0 m, la meta queda fuera de la zona inscrita del robot (0.22 m) y de la mitad
de la silla. El margen de 0.30 m absorbe la tolerancia de llegada de Nav2 (0.25 m): sin él, el
robot recibiría metas nuevas de pocos centímetros después de llegar. El umbral de 0.5 m evita
reenviar metas por el ruido de la estimación mientras el robot avanza.

**Descartado**: 0.8 m (la meta queda cerca del inflado del obstáculo) y 1.5 m (con la silla a
2.5–4 m, el robot casi no se movería).

## `dry_run`

**Decisión**: el emisor tiene el parámetro `dry_run` (verdadero por defecto). Calcula la meta con
tf2 real y la publica en `/goal_pose_preview` sin enviarla. El lanzamiento `dry_run` agrega
transformaciones estáticas y profundidad sintética para ejecutar toda la cadena con el video.

**Por qué**: así se ejercita el mismo código (tf2, cálculo de la meta, localizador) sin robot ni
servidor de navegación, y se verifica con números conocidos: con profundidad de 2.5 m, el objeto
queda a 2.6 m del robot y la meta a 1.0 m de él.

**Descartado**: publicar una detección 3D a mano. Así no se ejercitan el localizador ni el
emparejamiento por estampa.

## tf2 dentro de callbacks

**Decisión**: el emisor usa un ejecutor multihilo. El `TransformListener` crea sus suscripciones
en un grupo de callbacks reentrante, así que `/tf` se sigue recibiendo mientras un callback
espera una transformación con tiempo límite.

**Por qué**: con un ejecutor de un solo hilo, una consulta bloqueante nunca recibe `/tf`. La otra
opción, el hilo propio del listener (`spin_thread=True`), funcionaba pero imprimía una excepción
`ExternalShutdownException` al apagar el nodo con Ctrl+C (verificado).

## Simulación

**Decisión**: se usa el TurtleBot 4 mínimo de Nav2 (`nav2_minimal_tb4_sim`, ya instalado) con
su cámara RGB-D simulada (320 × 240, 10 Hz), el mundo y el mapa `depot`, y Nav2 real. La silla
de FreeCAD se inserta con `ros_gz_sim create` a 3 m frente al robot, en el mundo `(-5, 0)`, que
es `map (3.0, 0.0)`. Los nodos de percepción se conectan a la cámara simulada solo con
remapeos: no se lanza ni el nodo de cámara ni la profundidad sintética.

**Por qué**: así se *ejecutan*, no solo se diseñan, el componente RGB-D y la conexión con Nav2.
No hace falta construir un robot propio ni un mapa nuevo.

**Descartado**: `turtlebot4_simulator` (no instalado y más pesado) y el TurtleBot 3 (sin cámara
RGB-D).

## Configuración de Nav2 para la simulación

**Decisión**: `config/nav2_params_sim.yaml` es una copia de los parámetros de Nav2 con dos
cambios:

- Pose inicial de AMCL en `map (0, 0, 0)`, la aparición del robot, para localizar sin pasos
  manuales (FR-065).
- `xy_goal_tolerance` de 0.25 a 0.10 m, para que el robot se detenga cerca de la meta (SC-009).

**Por qué**: el archivo original deja AMCL esperando una pose fijada a mano en RViz. Con la
tolerancia de 0.25 m, el robot paraba unos 0.2 m antes de la meta (medido).

**Descartado**: publicar `/initialpose` desde el launch con un temporizador, porque depende de
cuándo arranca AMCL.

## Centro del objeto frente a su cara visible

**Decisión**: en simulación el localizador estima el **centro** del objeto. Avanza sobre el rayo
la mitad del ancho métrico del bbox, suponiendo una huella cuadrada (`estimate_center`, FR-036).
Por defecto (por ejemplo, en `dry_run`) publica la cara visible.

**Por qué**: la cámara solo ve la cara frontal. Medido: la estimación quedó a 0.25 m del centro
de la silla y el robot terminó a 1.35 m de ese centro, fuera del límite de 1.30 m de SC-009. Con
la corrección, la estimación quedó a 0.04 m del centro.

**Descartado**:
- Bajar la separación en simulación para compensar: oculta el sesgo en lugar de corregirlo.
- Medir los criterios desde la cara visible: la pose real documentada de la silla es su centro.

## Silla de FreeCAD

**Decisión**: la silla es una macro de FreeCAD versionada (`freecad/make_chair.py`) que genera
una butaca: estructura de madera oscura y cojines azules con bordes redondeados, en dos mallas
con su propio material. Un travesaño frontal bajo el cojín impide que la profundidad vea el fondo
entre las patas. Las mallas se versionan, así que ejecutar el paquete no requiere FreeCAD.

**Por qué**: una primera versión de cajas de un solo color no fue reconocida por YOLO11n (score
0.09). La versión 2 alcanza 0.55–0.71. Detalles en
[validacion_simulacion.md](validacion_simulacion.md).
