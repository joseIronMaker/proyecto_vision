# Implementation Plan: Paquete ROS 2 de percepción para navegación

**Branch**: `001-paquete-percepcion-nav` | **Date**: 2026-10-06 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/001-paquete-percepcion-nav/spec.md`

**Note**: This template is filled in by the `/speckit-plan` command; its definition describes the execution workflow.

## Summary

Paquete `ament_python` llamado `percepcion_nav` con cinco nodos desacoplados (cámara, detector
YOLO11n, visualización, localizador RGB-D y emisor de metas a Nav2) y una herramienta de
profundidad sintética. Se conectan solo por tópicos estándar, tf2 y la acción `NavigateToPose`.
Se valida en tres capas:

1. Pipeline 2D real con un video del celular.
2. `dry_run` de la cadena completa con profundidad sintética y transformaciones estáticas.
3. Simulación obligatoria en Gazebo Harmonic con el TurtleBot 4 mínimo de Nav2, una silla
   modelada en FreeCAD y Nav2 real.

La lógica numérica (deproyección, profundidad robusta, cálculo y política de metas) vive en
funciones puras probadas con `pytest` sin ROS.

## Technical Context

**Language/Version**: Python 3.12 (`ament_python`) sobre ROS 2 Jazzy.

**Primary Dependencies**:
- ROS: `rclpy`, `sensor_msgs`, `vision_msgs` 4.x, `geometry_msgs`, `cv_bridge`,
  `message_filters`, `tf2_ros`, `tf2_geometry_msgs`, `nav2_msgs`.
- Python (venv): `ultralytics==8.4.174` (pesos `yolo11n.pt`), `torch` y `torchvision` CPU,
  `opencv-python==4.11.0.86`, `numpy<2` (1.26.4).
- Simulación (solo ejecución): Gazebo Harmonic (`gz` 8.11), `ros_gz_sim`, `ros_gz_bridge`,
  `ros_gz_image`, `nav2_bringup`, `nav2_minimal_tb4_sim`.
- Diseño: FreeCAD 1.0 + servidor MCP, o `freecadcmd` (R4).

**Storage**: archivos: YAML de parámetros, video de muestra, pesos, mallas de la silla. No hay
base de datos.

**Testing**:
- `pytest` puro para `geometry.py` y `goal_planning.py`, sin ROS.
- `colcon test` con pruebas de construcción de mensajes, `ament_flake8` y `ament_pep257`.
- Procedimientos de extremo a extremo en [quickstart.md](quickstart.md).

**Target Platform**: Ubuntu 24.04 en WSL2, 8 núcleos, 12 GB de RAM, sin GPU NVIDIA. OpenGL por
software (`llvmpipe`, GL 4.5).

**Project Type**: paquete ROS 2 (`ament_python`) con 6 ejecutables, 3 lanzamientos y modelos SDF.

**Performance Goals**:
- Detector a 5 fps o más en CPU (SC-003).
- Pipeline 2D visible en menos de 30 s (SC-002).
- Recuperación de la cámara en 10 s o menos (SC-005).
- La simulación puede ir más lenta que el tiempo real.

**Constraints**:
- Solo mensajes estándar.
- `numpy<2` por `cv_bridge`.
- Archivos de Moodle de 50 MB o menos, 20 archivos como máximo, video de 2 a 4 min.
- Sin rutas, umbrales ni marcos fijos en el código.
- Identificadores en inglés; comentarios y registros en español.

**Scale/Scope**:
- 1 robot, 1 cámara.
- Video de 720p y menos de 1 min.
- Cámara simulada de 320×240 a 10 Hz.
- Unas 6 clases de nodo y 2 módulos puros.

Todas las incógnitas técnicas quedaron resueltas en [research.md](research.md) (R1–R16). No queda
ningún NEEDS CLARIFICATION.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principio | Antes de la fase 0 | Después del diseño | Evidencia |
|---|---|---|---|
| I. Nodos desacoplados y contratos | ✅ | ✅ | Un nodo por responsabilidad y el sexto ejecutable es la herramienta de prueba. Lógica compartida en `geometry.py` y `goal_planning.py`. Contrato en [contracts/ros-interfaces.md](contracts/ros-interfaces.md) y parámetros en [contracts/parameters.md](contracts/parameters.md) |
| II. Solo mensajes estándar | ✅ | ✅ | `Image`, `CameraInfo`, `Detection2DArray`, `Detection3DArray`, `PoseStamped`, `NavigateToPose`. Campos de `Detection2D` según FR-014 |
| III. Tiempo y marcos | ✅ | ✅ | `header` copiado (detector, localizador, profundidad sintética). `TimeSynchronizer` y `ApproximateTimeSynchronizer` (R8). tf2 en la estampa con tiempo límite y listener con hilo propio (R9). REP 103 y 105. `use_sim_time` en simulación |
| IV. Robustez | ✅ | ✅ | Tabla de errores recuperables por nodo en el contrato. Falla rápida si faltan los pesos. Registros limitados en frecuencia. Liberación de la captura, la ventana y el cliente de acción |
| V. Validación verificable | ✅ | ✅ | Funciones puras con `pytest` sin ROS (R16). `dry_run` con el código real de tf2 y de la meta. `colcon build` y `colcon test` en el quickstart |
| VI. Honestidad | ✅ | ✅ | Tabla de validado frente a diseño (data-model) con evidencia por componente. Respaldo de la silla declarado (FR-067) |
| VII. Documentación y diseño propio | ✅ | ✅ | README, `docs/diagrama_nodos.md` generado del contrato, captura de `rqt_graph`, `docs/decisiones.md` (decisiones R1–R16) |
| VIII. Simplicidad y alcance | ⚠️ | ⚠️ justificado | Gazebo obligatorio, prueba previa anticipada (T041), silla en FreeCAD y sexto ejecutable: justificados en Complexity Tracking. Modelo preentrenado sin entrenamiento |

Restricciones técnicas de la constitución:
- `ament_python`, Python 3.12, `package.xml`, `setup.py`, `setup.cfg`, `resource/`, `launch/`,
  `config/` y `test/`: ✅
- Dependencias ROS declaradas: ✅. Se agregan `exec_depend` para los lanzamientos de simulación.
- `venv` con `--system-site-packages` y `numpy<2`: ✅ (R6)
- Video como fuente por defecto: ✅
- Entregables dentro de los límites de Moodle: ✅ (R15)

Flujo de Git: `/speckit-git-validate` ✓ (`001-paquete-percepcion-nav`, coincide con `specs/001-…`).

**Resultado**: PASA. La única desviación (VIII) está justificada abajo.

**Revisión final tras la implementación (T062, 2026-10-06)**: todas las compuertas se mantienen.

- I: ningún nodo importa a otro; no hay marcos, rutas ni umbrales fijos fuera de
  `declare_parameter` y de los argumentos de lanzamiento; las constantes internas están
  documentadas.
- II: no hay mensajes, servicios ni acciones personalizados.
- III: `header` propagado, `message_filters` y tf2 con tiempo límite en la estampa del dato.
- IV: cada error recuperable tiene manejo y aviso con límite de frecuencia, con una línea de
  llamada por severidad, porque `rclpy` no permite mezclarlas.
- V: 30 pruebas puras sin ROS y 40 en `colcon test`.
- VI y VII: README, diagrama, decisiones, validación y reporte, con lo pendiente marcado.
- VIII: los cambios de la implementación (FR-036 y silla v2) quedaron en la especificación y en
  research.

Quedan pendientes los entregables manuales: video del celular, capturas de `rqt_graph`, video de
demostración, `.zip` y reproducción en un entorno limpio.

## Project Structure

### Documentation (this feature)

```text
specs/001-paquete-percepcion-nav/
├── plan.md              # Este archivo
├── research.md          # Fase 0: decisiones R1–R16
├── data-model.md        # Fase 1: entidades, reglas y estados
├── quickstart.md        # Fase 1: validación de extremo a extremo
├── contracts/
│   ├── ros-interfaces.md   # tópicos, QoS, marcos, acción, errores
│   ├── parameters.md       # parámetros por nodo y argumentos de launch
│   └── launch-files.md     # qué lanza y qué garantiza cada launch
├── checklists/requirements.md
└── tasks.md             # Fase 2 (/speckit-tasks)
```

### Source Code (repository root)

```text
proyecto_vision/                          # raíz del repo = workspace de colcon
├── .gitignore                            # build/ install/ log/ .venv/ *.pt media/ *.mp4
├── README.md                             # apunta a percepcion_nav/README.md
└── percepcion_nav/                       # paquete ament_python (lo que se entrega en .zip)
    ├── package.xml
    ├── setup.py                          # 6 console_scripts y data_files
    ├── setup.cfg                         # [build_scripts] executable = /usr/bin/env python3
    ├── requirements.txt                  # pins de Python (R6)
    ├── resource/percepcion_nav
    ├── percepcion_nav/
    │   ├── __init__.py
    │   ├── camera_node.py
    │   ├── detector_node.py
    │   ├── visualizer_node.py
    │   ├── rgbd_localizer_node.py
    │   ├── nav2_goal_sender_node.py
    │   ├── synthetic_depth_node.py       # herramienta de prueba (FR-035)
    │   ├── geometry.py                   # puro: deproyección, ventana, mediana, unidades
    │   └── goal_planning.py              # puro: meta, regla de distancia, reemplazo
    ├── launch/
    │   ├── pipeline_2d.launch.py
    │   ├── dry_run.launch.py
    │   └── simulation.launch.py
    ├── config/
    │   ├── params.yaml                   # todos los valores por defecto
    │   ├── params_sim.yaml               # sobrescrituras de simulación
    │   └── nav2_params_sim.yaml          # copia de Nav2 con pose inicial de AMCL (R2)
    ├── rviz/percepcion_nav.rviz
    ├── models/
    │   ├── yolo11n.pt                    # no versionado; va en el .zip
    │   └── freecad_chair/{model.config, model.sdf, meshes/}
    ├── freecad/make_chair.py             # macro de FreeCAD (fuente de la silla; no se instala)
    ├── media/sample.mp4                  # no versionado; va en el .zip
    ├── test/
    │   ├── test_geometry.py              # puro
    │   ├── test_goal_planning.py         # puro
    │   ├── test_detection_msgs.py        # requiere mensajes de ROS
    │   ├── test_flake8.py
    │   └── test_pep257.py
    ├── docs/
    │   ├── diagrama_nodos.md             # Mermaid, sincronizado con el contrato
    │   ├── decisiones.md
    │   ├── validacion_simulacion.md      # 5 ejecuciones: SC-008, SC-009
    │   ├── img/                          # rqt_graph, pipeline 2D, simulación
    │   └── reporte_tecnico.md            # → PDF de 1 a 2 páginas
    └── README.md                         # README ejecutable (FR-052)
```

**Structure Decision**: un solo paquete en `percepcion_nav/` dentro de la raíz del repositorio,
que se usa directamente como workspace de colcon (`colcon build` en la raíz). El `.venv` lleva
`COLCON_IGNORE`. La documentación entregable vive dentro del paquete para que viaje en el
`.zip`. Los artefactos de Spec Kit (`specs/`, `.specify/`) se quedan fuera del paquete.

## Fases de implementación (guía para `/speckit-tasks`)

El orden sigue las prioridades de la especificación. Cada fase deja el paquete compilable
(constitución, Git).

| Fase | `tasks.md` | Historias | Contenido | Sale cuando |
|---|---|---|---|---|
| A. Base | 1–2 | — | Esqueleto del paquete, `.gitignore`, venv y pins, `params.yaml`, pruebas de estilo | `colcon build` y `colcon test` en verde |
| B. Pipeline 2D | 3 | US1 (P1) | `camera`, `detector`, `visualizer`, `pipeline_2d.launch.py`, pruebas de mensajes | Quickstart §3 y §4 |
| C. Documentación base | 4 | US2 (P1) | README (instalación, pipeline 2D), diagrama, `rqt_graph` | §3 se reproduce en un entorno limpio siguiendo solo el README |
| D. RGB-D | 5 | US3 (P2) | `geometry.py` + pruebas, `rgbd_localizer`, `synthetic_depth` | `pytest` puro en verde; §5 publica `/detections_3d` |
| E. Nav2 | 6 | US4 (P3) | `goal_planning.py` + pruebas, `nav2_goal_sender`, `dry_run.launch.py`, RViz | Quickstart §5 completo |
| F. Prueba previa de simulación | 7 (T041) | US5 (P4) | Simulación original de Nav2 en WSL2 (R12) | `/rgbd_camera/image` publica |
| G. Simulación | 7 (T042–T053) | US5 (P4) | FreeCAD (macro y malla), modelo SDF, `nav2_params_sim.yaml`, `simulation.launch.py` | Quickstart §7: SC-008 y SC-009 (4 de 5) |
| H. Cierre | 8 | US2 | Tabla de validado frente a diseño, reporte, video, `.zip` | Quickstart §8 y SC-010–SC-012 |

La fase F (solo T041) puede adelantarse en paralelo a C–E porque es una medición sin código de
simulación. Si falla, hay que avisar pronto, porque la simulación es obligatoria
(Clarifications). Esta excepción al orden del principio VIII está justificada en Complexity
Tracking. Todo lo demás de US5 (fase G) empieza solo tras el checkpoint de US4.

## Riesgos

| Riesgo | Impacto | Mitigación |
|---|---|---|
| Render por software (`llvmpipe`) demasiado lento o con fallas de `ogre2` | Bloquea US5, que es obligatoria | Prueba previa en la fase F; `headless:=True`; probar `ogre`; la especificación acepta ir más lento que el tiempo real |
| La silla de FreeCAD sin textura no alcanza el score | US5, escenario 1 | Material con color y contraste; probar varias poses de yaw; respaldo de Fuel declarado (FR-067) |
| FreeCAD o el MCP no se instalan bien | Retrasa la silla | Macro ejecutable con `freecadcmd`; la malla se versiona |
| La pose inicial de AMCL no coincide con la aparición | Localización mala, falla SC-008 | Verificación en R2 (`/amcl_pose` frente a `gz model`), ajustar `initial_pose` |
| El `frame_id` del sensor simulado no es el marco óptico | Falla la consulta de tf | Verificación en R1; parámetro de anulación si hace falta |
| Inferencia lenta junto con la simulación | Pocas detecciones | `imgsz` 320 en simulación (parámetro); `class_filter: chair` |
| La profundidad mide la superficie visible, no el centro de la silla, o ve el fondo entre las patas | Sesgo de hasta 0.225 m, o estimación fuera de la silla; falla SC-008 o SC-009 | Faldón frontal en la silla (R4); referencia en el centro de la huella (R10); T049 verifica pronto que la estimación cae dentro de la huella; ajustar `depth_window_fraction` |

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| Simulación en Gazebo obligatoria (principio VIII la considera extensión opcional) | Decisión del autor (Clarifications, 2026-10-06). Es la única forma de *ejecutar*, no solo diseñar, los criterios "Componente RGB-D" y "Conexión con Nav2". Su implementación (fase G, T042–T053) se aborda después de US1–US4, como pide VIII | Quedarse solo con el camino (c) (`dry_run`) deja RGB-D y Nav2 como diseño no ejecutado. Se respeta el orden: la simulación es la última fase funcional |
| Silla modelada en FreeCAD | Decisión del autor; el objeto meta es propio y defendible en el reporte | Usar directamente una silla de Fuel no cumple la decisión, así que queda como respaldo declarado (FR-067) |
| Prueba previa de simulación (T041) antes de cumplir los mínimos (VIII: las extensiones se abordan solo tras los mínimos) | US5 es obligatoria y el render por software de WSL2 (`llvmpipe`) es el mayor riesgo del plan (R12). Saber pronto si Gazebo funciona evita descubrir un bloqueo al final. T041 solo ejecuta la simulación original de Nav2 y mide; no escribe código ni configuración de simulación | Esperar al checkpoint de US4 para medir dejaría sin margen para reaccionar (cambiar de motor de render o de máquina) si la simulación no corre |
| Panel gráfico `sim_panel` (FR-068), pedido por el autor para la demostración | Iniciar, navegar a voluntad, detener y reiniciar la simulación completa sin terminales ni procesos huérfanos de Gazebo | Comandos en varias terminales: frágiles al grabar el video y dejan procesos vivos si se cierra mal |
| Sexto ejecutable: `synthetic_depth` (herramienta de prueba) | Permite que el `dry_run` ejercite el localizador real con detecciones reales del video (principio V) | Inyectar a mano una `Detection3DArray` no ejercita el localizador ni el emparejamiento por estampa |
