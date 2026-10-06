<!--
Sync Impact Report
- Version change: plantilla sin llenar → 1.0.0
- Principios: los cinco marcadores de la plantilla se reemplazan por ocho principios
  (I a VIII); el número de principios se amplía según permite el comando de constitución.
- Secciones añadidas: "Restricciones técnicas y de entrega", "Flujo de desarrollo con Spec Kit y Git".
- Secciones eliminadas: ninguna.
- Plantillas dependientes: plan-template.md (la "Constitution Check" debe usar las compuertas
  de la sección de flujo de desarrollo), spec-template.md y tasks-template.md no requieren cambios.
- TODO diferidos: ninguno.
- Nota: este archivo es un borrador en la raíz del proyecto. Para activarlo, copiarlo a
  .specify/memory/constitution.md o pasarlo como entrada a /speckit-constitution. Este comentario
  se elimina antes de hacer commit de la constitución.
-->

# Paquete ROS 2 de Percepción para Navegación Constitution

## Core Principles

### I. Nodos desacoplados con contratos explícitos

- Cada responsabilidad del pipeline (cámara, detección, visualización, localización RGB-D,
  envío de metas a Nav2) MUST vivir en su propio nodo y ejecutable.
- Los nodos MUST comunicarse solo por tópicos, servicios, acciones y tf2. Ningún nodo importa
  código de otro nodo; la lógica compartida va en módulos sin dependencias de ROS.
- Todo nombre de tópico, marco y valor ajustable MUST exponerse como parámetro ROS o ser
  remapeable. No se permiten rutas, umbrales ni nombres de tópico fijos en el código.
- El contrato de cada interfaz (nombre, tipo, QoS, `frame_id`, quién publica y quién suscribe)
  MUST estar documentado antes de implementarse y coincidir con el diagrama de nodos.

**Razón**: la actividad evalúa integración de sistemas. El desacoplamiento permite cambiar la
fuente de video por un bag o Gazebo, el modelo por otro, o Nav2 real por `dry_run`, sin tocar los
demás nodos.

### II. Solo mensajes estándar (NON-NEGOTIABLE)

- Las imágenes MUST publicarse como `sensor_msgs/Image` convertidas con `cv_bridge`.
- Las detecciones 2D MUST publicarse como `vision_msgs/Detection2DArray` y las 3D como
  `vision_msgs/Detection3DArray`.
- La navegación MUST usar la acción `nav2_msgs/action/NavigateToPose`.
- Queda prohibido definir mensajes, servicios o acciones personalizados.
- Cada `Detection2D` MUST poblar bbox (centro y tamaño en píxeles), `class_id` y `score`.

**Razón**: la rúbrica califica como Insuficiente el uso de mensajes personalizados y exige
`Detection2DArray` correctamente poblado. Los tipos estándar garantizan interoperabilidad con
RViz, Nav2 y el resto del ecosistema.

### III. Tiempo y marcos de referencia coherentes

- Todo mensaje con `header` MUST llevar `stamp` válido y `frame_id` no vacío.
- Los nodos que derivan datos de una imagen MUST copiar el `header` de la imagen de origen,
  sin volver a estampar el tiempo.
- Las sincronizaciones entre flujos MUST usar `message_filters` por estampa de tiempo.
- Las transformaciones de coordenadas MUST hacerse con tf2, consultando en la estampa del dato y
  con tiempo límite. Los marcos siguen REP 103 y REP 105.

**Razón**: sin estampas y marcos coherentes no es posible sincronizar detecciones con
profundidad ni transformar un punto de la cámara al mapa; la rúbrica pide timestamps y manejo de
tf2 explicado.

### IV. Robustez ante fallos

- Ningún nodo MUST terminar por un error recuperable: fuente desconectada, cuadro corrupto,
  conversión fallida, profundidad inválida, transformación no disponible o servidor de acción
  ausente.
- Cada error recuperable MUST registrarse con un mensaje accionable y con límite de frecuencia,
  y el nodo MUST seguir operando o reintentar.
- Los errores no recuperables al arranque (por ejemplo, modelo inexistente) MUST fallar rápido
  con un mensaje claro.
- Los recursos (captura de video, ventanas, clientes de acción) MUST liberarse al destruir el
  nodo.

**Razón**: el nivel Sobresaliente del nodo de cámara exige manejo de errores, y un sistema
robótico no debe caerse por una condición esperada del entorno.

### V. Validación verificable

- La lógica numérica (deproyección pinhole, conversión de unidades de profundidad, cálculo de la
  meta de navegación) MUST implementarse como funciones puras y tener pruebas `pytest` que se
  ejecuten sin ROS.
- El paquete MUST pasar `colcon build` y `colcon test` antes de cualquier entrega.
- El pipeline cámara → detección → visualización MUST demostrarse ejecutándose de extremo a
  extremo, con evidencia grabada.
- Los componentes que no pueden ejecutarse contra hardware MUST ejercitarse en un modo
  `dry_run` que use el código real (tf2, cálculo de meta) con datos o transformaciones simuladas.

**Razón**: sin hardware RGB-D ni robot, las pruebas y el `dry_run` son la evidencia de que el
diseño es técnicamente correcto.

### VI. Honestidad sobre lo validado (NON-NEGOTIABLE)

- La documentación MUST distinguir para cada componente qué se ejecutó y validó realmente y qué
  quedó como diseño no probado en hardware.
- Queda prohibido presentar como probado algo que solo se diseñó, o simular resultados en el
  video o el reporte.
- Las limitaciones conocidas MUST declararse en el README y en el reporte.

**Razón**: la actividad exige explícitamente explicar qué se validó y qué quedó como diseño, y
la integridad académica lo requiere.

### VII. Documentación ejecutable y diseño propio

- El README MUST permitir a una persona ajena compilar y ejecutar el sistema en un Ubuntu 24.04
  con ROS 2 Jazzy limpio, sin pasos implícitos.
- El diagrama de nodos y tópicos MUST mantenerse sincronizado con el código y acompañarse de una
  captura de `rqt_graph`.
- Cada decisión de arquitectura (tópicos, tipos, QoS, marcos, reparto de responsabilidades)
  MUST tener su justificación escrita.
- La IA generativa puede usarse para código boilerplate, pero el diseño del flujo de datos MUST
  ser comprendido y defendible por el equipo; toda decisión del diagrama MUST poder explicarse
  sin apoyo.

**Razón**: la rúbrica pide diagrama claro, README ejecutable y decisiones justificadas, y la
política del curso permite pedir la explicación de cualquier decisión.

### VIII. Simplicidad y alcance acotado

- El alcance MUST limitarse a los requisitos de la actividad; las extensiones (Gazebo, bag
  RGB-D, webcam por usbipd) son opcionales y solo se abordan tras cumplir los mínimos.
- Se reutiliza un modelo de detección preentrenado; entrenar o mejorar modelos queda fuera del
  alcance.
- Cualquier complejidad adicional MUST justificarse en la tabla "Complexity Tracking" del plan.

**Razón**: la actividad sugiere de 4 a 5 horas y evalúa integración, no rendimiento del modelo.

## Restricciones técnicas y de entrega

- **Plataforma**: Ubuntu 24.04 (WSL2) con ROS 2 Jazzy.
- **Paquete**: `ament_python`, Python 3.12, con `package.xml`, `setup.py`, `setup.cfg`,
  `resource/`, `launch/`, `config/` y `test/`.
- **Dependencias ROS**: `rclpy`, `sensor_msgs`, `vision_msgs`, `geometry_msgs`, `cv_bridge`,
  `message_filters`, `tf2_ros`, `tf2_geometry_msgs`, `nav2_msgs`; todas declaradas en
  `package.xml`.
- **Dependencias Python**: instaladas en un entorno virtual con `--system-site-packages`
  (PEP 668), con `numpy<2` fijado para compatibilidad con `cv_bridge` de Jazzy.
- **Fuente de imagen por defecto**: video grabado, porque WSL2 no expone `/dev/video*`.
- **Entregables**: paquete en `.zip` sin `build/`, `install/` ni `log/`; video de demostración;
  reporte técnico de 1 a 2 páginas con autoevaluación según la rúbrica.
- **Límites de Moodle**: 50 MB por archivo y un máximo de 20 archivos.

## Flujo de desarrollo con Spec Kit y Git

**Secuencia de comandos**

1. `/speckit-constitution` → `/speckit-specify` → `/speckit-clarify` → `/speckit-plan` →
   `/speckit-tasks` → `/speckit-analyze` → `/speckit-implement`.
2. Ninguna fase se salta. Los artefactos de cada funcionalidad viven en
   `specs/<###>-<slug>/` (`spec.md`, `plan.md`, `tasks.md` y anexos).

**Git (extensión `git` de Spec Kit)**

- El repositorio se inicializa con `/speckit-git-initialize` (hook obligatorio
  `before_constitution`) y el commit inicial `[Spec Kit] Initial commit`.
- Cada funcionalidad se desarrolla en una rama con numeración secuencial `{number}-{slug}`
  (por ejemplo `001-paquete-percepcion-nav`), creada por `/speckit-git-feature` (hook obligatorio
  `before_specify`). El número de la rama MUST coincidir con el directorio en `specs/`.
- `/speckit-git-validate` MUST pasar antes de planear, generar tareas o implementar.
- Los commits usan el estilo `fixed` configurado en `.specify/extensions/git/git-config.yml`
  para los artefactos de Spec Kit; los commits de código usan Conventional Commits
  (`feat:`, `fix:`, `docs:`, `test:`, `chore:`).
- Cada commit MUST dejar el paquete compilable. No se hace commit de `build/`, `install/`,
  `log/`, pesos de modelos, videos ni entornos virtuales; el `.gitignore` los excluye.
- Se integra a `main` solo cuando la funcionalidad cumple las compuertas siguientes.

**Compuertas de calidad (Constitution Check)**

Antes de la fase 0 del plan y de nuevo tras el diseño, MUST verificarse:

- [ ] I: cada nodo tiene una sola responsabilidad y su contrato de interfaces está documentado.
- [ ] II: solo tipos estándar; `Detection2DArray` con bbox, clase y score.
- [ ] III: `header` propagado, sincronización por estampa y tf2 con tiempo límite.
- [ ] IV: cada error recuperable identificado tiene manejo y registro.
- [ ] V: lógica numérica en funciones puras con pruebas; `dry_run` definido.
- [ ] VI: tabla de validado frente a diseño actualizada.
- [ ] VII: README, diagrama y decisiones al día.
- [ ] VIII: sin alcance fuera de la actividad o justificado en "Complexity Tracking".

Antes de la entrega, además: `colcon build` y `colcon test` sin errores, video grabado y
autoevaluación de la rúbrica completa.

## Governance

- Esta constitución prevalece sobre cualquier otra práctica del proyecto. Las especificaciones,
  planes y tareas MUST cumplirla, y `/speckit-analyze` MUST reportar las violaciones como
  críticas.
- Las enmiendas se proponen con `/speckit-constitution`, se documentan en el Sync Impact Report,
  se aprueban por todos los integrantes del equipo y se registran en un commit propio.
- Versionado semántico:
  - MAJOR: eliminación o redefinición incompatible de un principio.
  - MINOR: principio o sección nuevos, o guía ampliada de forma material.
  - PATCH: aclaraciones y correcciones de redacción.
- Toda revisión de código y cada ejecución de `/speckit-plan` MUST verificar las compuertas de
  calidad. Una violación solo se acepta si está justificada en "Complexity Tracking".
- La guía de ejecución del día a día está en `README.md` y en `pre-spec.md`.

**Version**: 1.0.0 | **Ratified**: 2026-10-06 | **Last Amended**: 2026-10-06
