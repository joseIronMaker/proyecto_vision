# Feature Specification: Paquete ROS 2 de percepción para navegación

**Feature Branch**: `001-paquete-percepcion-nav`

**Created**: 2026-10-06

**Status**: Draft

**Input**: User description: "pre-spec.md (borrador del 2026-10-06), más las decisiones de la sesión
de preguntas previa: validación por el camino (c) más simulación en Gazebo con Nav2, silla modelada
en FreeCAD como objeto meta, video real para el pipeline 2D, YOLO nano COCO y entrega individual."

## Clarifications

### Session 2026-10-06

- Q: ¿Qué estrategia de validación siguen el componente RGB-D y la conexión con Nav2? → A: El
  camino (c) es la base (pruebas unitarias y `dry_run` con transformaciones estáticas) y, una vez
  cumplidos los mínimos, se añade una simulación en Gazebo con Nav2.
- Q: ¿Qué modelo de detección preentrenado se usa? → A: YOLO nano con pesos COCO, con inferencia
  en CPU (el equipo de desarrollo no tiene GPU dedicada).
- Q: ¿Qué objeto es la meta de navegación en la simulación? → A: Una silla (clase COCO `chair`)
  modelada en FreeCAD, de preferencia por MCP, e insertada en el mundo simulado.
- Q: ¿Qué papel tiene el nodo de cámara si Gazebo entra en el alcance? → A: El nodo de cámara
  reproduce un video real grabado con el celular para el pipeline 2D. En simulación no se lanza y
  los nodos de percepción se conectan a la cámara simulada solo por remapeo.
- Q: ¿La entrega es individual o en equipo? → A: Individual.
- Q: ¿En qué idioma van el código y la documentación? → A: Identificadores en inglés; comentarios,
  mensajes de log, README y reporte en español.
- Q: En el lanzamiento `dry_run` sin Gazebo, ¿de dónde obtiene el localizador RGB-D la profundidad
  y los intrínsecos, si el video no trae profundidad? → A: De una herramienta de prueba de
  profundidad sintética que, por cada cuadro del video, publica una profundidad constante
  configurable y los intrínsecos con la misma estampa del cuadro, para que la cadena completa
  corra con detecciones reales del video.
- Q: Si la simulación en Gazebo no cumple sus criterios antes de la entrega, ¿la entrega sigue
  siendo válida? → A: No. La simulación es obligatoria: sin ella funcionando y cumpliendo sus
  criterios, la entrega no está completa.
- Q: Cuando el robot termina una navegación y sigue viendo la silla, ¿cuándo vuelve a enviar
  meta el emisor? → A: Solo si el robot está más lejos que la distancia de separación más una
  tolerancia de llegada configurable (0.30 m por defecto); la regla aplica siempre, también
  después de llegar.
- Q: Al emparejar detección y profundidad, ¿las estampas deben coincidir exactamente o se acepta
  una diferencia pequeña? → A: Aproximada, con una diferencia máxima configurable (20 ms por
  defecto); si ninguna profundidad cae dentro del margen, la detección se descarta con advertencia.
- Q: ¿Qué distancia de separación por defecto usa el emisor de metas? → A: 1.0 m; en simulación,
  la silla se coloca a entre 2.5 y 4 m de la pose inicial del robot.
- Q: (remediación de `/speckit-analyze`) ¿Desde qué punto se miden la posición real de la silla y
  la distancia final del robot? → A: Desde el centro de la huella de la silla (origen del modelo).
  La cámara mide la superficie visible, así que la silla lleva un faldón frontal que impide ver el
  fondo entre las patas. Así la profundidad estimada queda dentro de la huella, a 0.225 m o menos
  del centro.
- Q: (hallazgo de la implementación) ¿Cómo se evita que la estimación sobre la cara frontal
  sesgue SC-008 y SC-009? → A: Medido en simulación: con la superficie visible, el robot terminó
  a 1.35 m del centro de la silla (límite 1.30). El localizador gana un parámetro
  `estimate_center` (FR-036). Con él, estima el centro del objeto avanzando sobre el rayo la
  mitad de su ancho métrico (supuesto de huella cuadrada). Está desactivado por defecto (el
  `dry_run` sigue publicando la superficie) y se activa en simulación. Además, la silla de
  FreeCAD pasó a una versión 2 (butaca con estructura y cojines de dos colores), porque YOLO11n
  no reconocía la versión de cajas.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Pipeline 2D de extremo a extremo (Priority: P1)

Como evaluador, quiero lanzar un solo comando y ver las detecciones dibujadas sobre la imagen de un
video real, para confirmar que cámara, detector y visualización se comunican entre sí.

**Why this priority**: cubre tres de los seis criterios de la rúbrica (nodo de cámara, nodo
detector y verificación visual) y es la única parte que se ejecuta de verdad sin depender de
simulación. Por sí sola ya es una entrega demostrable.

**Independent Test**: lanzar el pipeline 2D con el video de muestra; se ve la imagen anotada y la
inspección del tópico de detecciones muestra mensajes con bbox, clase y score.

**Acceptance Scenarios**:

1. **Given** un video válido, **When** se lanza el pipeline 2D, **Then** el tópico de imagen
   publica cuadros con estampa de tiempo creciente y `frame_id` no vacío.
2. **Given** un cuadro con objetos de clases conocidas, **When** el detector lo procesa, **Then**
   publica un arreglo de detecciones 2D cuyo `header` es idéntico al de la imagen y cada detección
   trae bbox, clase y score.
3. **Given** un cuadro sin objetos reconocibles, **When** el detector lo procesa, **Then** publica
   un arreglo vacío con el `header` de la imagen.
4. **Given** una fuente inexistente o que se desconecta, **When** el nodo de cámara arranca o
   pierde la fuente, **Then** registra el error, sigue vivo, reintenta periódicamente y vuelve a
   publicar cuando la fuente regresa.
5. **Given** imagen y detecciones del mismo instante, **When** llegan a la visualización, **Then**
   la imagen anotada muestra rectángulo, clase y score de cada detección sobre el cuadro del que
   provienen.
6. **Given** que el video llega a su fin, **When** la repetición está activada, **Then** el video
   se reinicia; **When** está desactivada, **Then** el nodo se apaga de forma limpia.

---

### User Story 2 - Documentación y entregables (Priority: P1)

Como evaluador, quiero un README que pueda seguir paso a paso, un diagrama claro y un reporte con
las decisiones justificadas, para reproducir la demo y entender por qué el sistema es como es.

**Why this priority**: la documentación es un criterio completo de la rúbrica y condiciona la
calificación de los demás, porque el evaluador puede pedir explicar cualquier decisión. Crece con
cada historia: empieza con el pipeline 2D y se amplía con las demás.

**Independent Test**: en un Ubuntu 24.04 con ROS 2 Jazzy limpio (sin el historial del autor), se
sigue solo el README y se logra compilar y lanzar lo implementado hasta ese momento. Lo ideal es que
lo haga una persona ajena al proyecto.

**Acceptance Scenarios**:

1. **Given** un Ubuntu 24.04 con ROS 2 Jazzy limpio, **When** sigo el README, **Then** instalo
   las dependencias, compilo y lanzo el pipeline sin pasos faltantes ni implícitos.
2. **Given** el reporte técnico, **When** lo leo, **Then** encuentro el diagrama de nodos y
   tópicos, las decisiones justificadas, la tabla de lo validado frente a lo diseñado y la
   autoevaluación según la rúbrica.
3. **Given** el sistema en ejecución, **When** comparo su grafo de nodos con el diagrama
   documentado, **Then** ambos coinciden en nodos, tópicos y tipos.
4. **Given** el paquete y los entregables listos, **When** los preparo para Moodle, **Then**
   cada archivo pesa 50 MB o menos, hay 20 archivos o menos y no se incluyen artefactos de
   compilación.
5. **Given** el video de demostración, **When** lo reproduzco, **Then** en 2 a 4 minutos muestra
   cada paso del guion (FR-056) y cierra declarando qué se validó y qué quedó como diseño.

---

### User Story 3 - Posición 3D de un objeto detectado (Priority: P2)

Como integrador, quiero convertir una detección 2D, la profundidad y los intrínsecos de la cámara
en un punto 3D en el marco de la cámara, para alimentar a la navegación.

**Why this priority**: cubre el criterio "Componente RGB-D". Depende de que exista el detector
(historia 1), pero su lógica se valida sin hardware.

**Independent Test**: pruebas automáticas de la deproyección con profundidad sintética e
intrínsecos conocidos; ejecución en `dry_run` con el video y la herramienta de profundidad
sintética; además, ejecución con la cámara RGB-D simulada si la historia 5 está lista.

**Acceptance Scenarios**:

1. **Given** intrínsecos conocidos y profundidad constante `Z`, **When** se deproyecta el píxel
   `(u, v)`, **Then** el resultado es `X = (u − cx)·Z/fx`, `Y = (v − cy)·Z/fy`, `Z`.
2. **Given** una región del bbox con ceros o valores no numéricos en la profundidad, **When** se
   estima `Z`, **Then** se usa la mediana de los valores válidos de una ventana central del bbox;
   si no hay valores válidos, la detección se descarta con una advertencia.
3. **Given** profundidad entera en milímetros, **When** se procesa, **Then** se convierte a
   metros; **Given** profundidad flotante en metros, **Then** se usa sin conversión.
4. **Given** una detección 3D publicada, **When** se inspecciona, **Then** conserva la clase, el
   score y la estampa de la detección 2D de origen y su marco es el marco óptico de la cámara.
5. **Given** que aún no llegan los intrínsecos de la cámara, **When** llega una detección,
   **Then** no se publica detección 3D y se registra una advertencia con límite de frecuencia.
6. **Given** el `dry_run` con profundidad sintética constante `Z`, **When** el detector encuentra
   un objeto en un cuadro del video, **Then** el localizador publica su detección 3D con
   coordenada `z` igual a `Z` y la estampa del cuadro.

---

### User Story 4 - Meta de navegación hacia el objeto (Priority: P3)

Como integrador, quiero que una detección 3D de la clase objetivo se convierta en una meta de
navegación en el marco del mapa, para que el robot se acerque al objeto.

**Why this priority**: cubre el criterio "Conexión con Nav2". Depende de la historia 3; se valida
en modo `dry_run` con transformaciones estáticas, sin robot ni servidor de navegación.

**Independent Test**: lanzar el sistema completo en `dry_run` con el video, la profundidad
sintética y transformaciones estáticas; verificar que una silla del video produce una pose en el
tópico de vista previa de la meta.

**Acceptance Scenarios**:

1. **Given** una detección 3D de la clase objetivo en el marco óptico y la cadena de
   transformaciones disponible, **When** llega la detección, **Then** el punto se transforma al
   marco del mapa y se calcula una meta a la distancia de separación del objeto, sobre la línea
   robot–objeto y orientada hacia él.
2. **Given** que la transformación no está disponible, **When** llega la detección, **Then** se
   registra una advertencia y no se calcula ni envía meta.
3. **Given** `dry_run` desactivado y el servidor de navegación ausente, **When** se intenta enviar
   la meta, **Then** la espera del servidor expira y el nodo lo reporta sin bloquearse.
4. **Given** una meta activa, **When** llega otra detección del mismo objeto que se movió menos
   que el umbral configurado, **Then** no se reenvía la meta.
5. **Given** que el robot está a la distancia de separación más la tolerancia de llegada o más
   cerca (por ejemplo, tras terminar una navegación), **When** llega una detección, **Then** no se
   envía meta.
6. **Given** varias detecciones de la clase objetivo en el mismo mensaje, **When** se selecciona
   el objetivo, **Then** se usa la de mayor score.

---

### User Story 5 - Simulación en Gazebo de extremo a extremo (Priority: P4)

Como evaluador, quiero lanzar un solo comando que abra un entorno simulado con un robot con cámara
RGB-D, una silla modelada en FreeCAD y la pila de navegación, y ver que el robot detecta la silla,
estima su posición 3D y navega hasta quedar frente a ella.

**Why this priority**: es la evidencia más fuerte de los criterios "Componente RGB-D" y "Conexión
con Nav2", porque los ejecuta en lugar de solo diseñarlos. La constitución (principio VIII) la
clasifica como extensión, así que se aborda solo cuando las historias 1 a 4 cumplen sus criterios.
Aun así, es obligatoria: la entrega no está completa hasta que esta historia cumple sus criterios,
y su inclusión se justifica en "Complexity Tracking" del plan.

**Independent Test**: lanzar la simulación; observar la silla detectada en la imagen anotada,
comparar la posición 3D estimada con la posición real de la silla en el mundo y confirmar que la
acción de navegación termina con éxito frente a la silla.

**Acceptance Scenarios**:

1. **Given** la simulación lanzada con la silla visible desde la pose inicial del robot, **When**
   el detector procesa la imagen simulada, **Then** publica detecciones de la clase `chair` con
   score igual o mayor que el umbral configurado.
2. **Given** una detección de la silla y la profundidad simulada, **When** se estima su posición
   y se transforma al marco del mapa, **Then** queda a 0.30 m o menos de la posición real de la
   silla (centro de su huella) en el plano del suelo.
3. **Given** `dry_run` desactivado y la navegación activa, **When** se envía la meta, **Then** el
   robot navega, la acción termina con éxito y el robot se detiene a la distancia de separación
   del centro de la huella de la silla (±0.30 m), mirando hacia ella (error de orientación de 15°
   o menos).
4. **Given** la simulación activa, **When** los nodos estampan mensajes y consultan
   transformaciones, **Then** todos usan el reloj de la simulación y no aparecen errores de
   extrapolación en operación estable, es decir, después de que AMCL publica su primera pose y
   pasan 30 s de tiempo simulado.
5. **Given** que la cámara de la fuente cambia del video a la simulación, **When** se prepara el
   lanzamiento, **Then** basta con remapeos y parámetros, sin cambios de código.
6. **Given** que el detector no reconoce la silla de FreeCAD con el score mínimo, **When** se
   aplica el respaldo, **Then** se usa un modelo de silla de la biblioteca pública del simulador y
   la sustitución queda declarada en el README y en el reporte.

---

### Edge Cases

- El video termina: se reinicia si la repetición está activada; si no, el nodo se apaga limpio.
- La fuente no existe al arrancar o se pierde a mitad de la ejecución: error registrado, reintento
  periódico, el proceso no termina.
- Un cuadro llega corrupto o su conversión falla: se descarta con advertencia y se sigue.
- La inferencia es más lenta que la cámara: se procesa siempre el cuadro más reciente y se
  descartan los atrasados.
- El modelo de detección no existe o no carga al arrancar: el nodo falla de inmediato con un
  mensaje claro.
- No hay detecciones en un cuadro: se publica un arreglo vacío con el `header` de la imagen.
- El bbox toca o sale del borde de la imagen: la ventana de profundidad se recorta a la imagen.
- La profundidad del bbox es toda inválida (ceros o no numérica): la detección se descarta con
  advertencia.
- No llega profundidad con estampa dentro de la diferencia máxima respecto a la detección: la
  detección no se procesa y se registra una advertencia con límite de frecuencia.
- No hay detecciones de la clase objetivo: no se calcula meta.
- El robot llega a la silla y la sigue viendo: si está dentro de la distancia de separación más
  la tolerancia de llegada, no se envía meta nueva y no hay correcciones repetidas.
- La transformación al mapa no está disponible o expira: advertencia y no se envía meta.
- El servidor de navegación no responde, rechaza o aborta la meta: se registra el resultado y el
  nodo queda listo para una meta nueva.
- La simulación corre más lento que el tiempo real: las estampas y consultas siguen el reloj
  simulado y el sistema sigue siendo correcto, aunque más lento.
- La silla de FreeCAD no se detecta en la simulación: se aplica el respaldo declarado.

## Requirements *(mandatory)*

Las interfaces se nombran con sus tipos estándar porque la rúbrica y la constitución (principio
II) los exigen; no son una elección de implementación. Los nombres de tópico son valores por
defecto y todos son remapeables.

### Functional Requirements

**Nodo de cámara**

- **FR-001**: El nodo de cámara MUST publicar cada cuadro como `sensor_msgs/Image` en
  `/camera/image_raw`.
- **FR-002**: La fuente MUST configurarse por parámetro: ruta de un video o índice de dispositivo.
  También MUST ser configurables la frecuencia de publicación, el `frame_id` y la repetición del
  video.
- **FR-003**: Cada mensaje MUST llevar la estampa del reloj del nodo y el `frame_id` configurado.
- **FR-004**: Si la fuente no abre, el nodo MUST registrar el error y reintentar periódicamente,
  sin terminar el proceso.
- **FR-005**: Si falla la lectura de un cuadro, el nodo MUST registrar una advertencia con límite
  de frecuencia, liberar la fuente e intentar reconectar. Al terminar un video, MUST reiniciarlo
  si la repetición está activada o apagarse limpio si no.
- **FR-006**: El nodo MUST liberar la fuente al destruirse.

**Nodo detector**

- **FR-010**: El detector MUST suscribirse al tópico de imagen y procesar siempre el cuadro más
  reciente, descartando los atrasados.
- **FR-011**: El detector MUST usar un modelo preentrenado con clases COCO, con ruta del modelo,
  umbral de confianza, dispositivo de inferencia y filtro de clases configurables por parámetro.
  Con el filtro vacío MUST publicar todas las clases.
- **FR-012**: El detector MUST publicar `vision_msgs/Detection2DArray` en `/detections`.
- **FR-013**: El `header` del arreglo y de cada `Detection2D` MUST ser una copia del de la imagen
  de entrada.
- **FR-014**: Cada `Detection2D` MUST poblar `bbox.center.position` (x, y en píxeles),
  `bbox.center.theta = 0`, `bbox.size_x` y `bbox.size_y` en píxeles,
  `results[0].hypothesis.class_id` con el nombre de la clase y `results[0].hypothesis.score` en
  el rango de 0 a 1.
- **FR-015**: El detector MUST publicar un arreglo vacío cuando no haya detecciones.
- **FR-016**: Si el modelo no carga, el detector MUST fallar al arrancar con un mensaje claro. Si
  falla la conversión de un cuadro, MUST descartarlo y continuar.

**Verificación visual**

- **FR-020**: La visualización MUST emparejar imagen y detecciones por estampa de tiempo exacta.
- **FR-021**: La visualización MUST dibujar rectángulo, clase y score de cada detección.
- **FR-022**: La visualización MUST publicar la imagen anotada en `/detections_image` y MAY abrir
  una ventana local si un parámetro lo activa.

**Componente RGB-D**

- **FR-030**: El localizador MUST recibir las detecciones 2D, la profundidad alineada al color y
  los intrínsecos de la cámara, y emparejar detecciones con profundidad por estampa de tiempo de
  forma aproximada, con una diferencia máxima configurable (20 ms por defecto).
- **FR-031**: El localizador MUST leer `fx`, `fy`, `cx`, `cy` de los intrínsecos recibidos.
- **FR-032**: El localizador MUST estimar la profundidad de cada detección con la mediana de los
  valores válidos de una ventana central del bbox, de tamaño configurable, convirtiendo milímetros
  a metros cuando la codificación lo requiera.
- **FR-033**: El localizador MUST deproyectar con el modelo pinhole y publicar
  `vision_msgs/Detection3DArray` en `/detections_3d`, en el marco óptico de la cámara, con la
  clase y el score de origen y la posición en `results[0].pose.pose.position` y
  `bbox.center.position`.
- **FR-034**: La deproyección, la estimación de profundidad y la conversión de unidades MUST
  poder verificarse con pruebas automáticas sin levantar el sistema.
- **FR-036**: El localizador MUST ofrecer un parámetro `estimate_center` (falso por defecto).
  Con el parámetro activo, desplaza el punto deproyectado sobre su rayo una distancia igual a la
  mitad del ancho métrico del bbox, para estimar el centro del objeto en lugar de su cara
  visible, y publica ese ancho como `bbox.size.z`. En simulación MUST estar activo.
- **FR-035**: Para el `dry_run`, una herramienta de prueba de profundidad sintética MUST
  publicar, por cada cuadro de imagen recibido, una imagen de profundidad constante en metros
  (valor configurable) del mismo tamaño que el cuadro y los intrínsecos de la cámara
  (configurables), ambos con el `header` del cuadro. Esta herramienta MUST NOT lanzarse en
  simulación.

**Conexión con Nav2**

- **FR-040**: El emisor de metas MUST recibir las detecciones 3D y seleccionar la de la clase
  objetivo (parámetro, por defecto `chair`) con mayor score.
- **FR-041**: El emisor MUST transformar el punto al marco del mapa con la estampa de la detección
  y un tiempo de espera configurable, y tratar la transformación no disponible como error
  recuperable.
- **FR-042**: El emisor MUST calcular la meta a la distancia de separación configurable del
  objeto (1.0 m por defecto), sobre la línea robot–objeto, a altura cero y orientada hacia el objeto. Si el robot
  está a la distancia de separación más una tolerancia de llegada configurable (0.30 m por
  defecto) o más cerca, MUST NOT enviar meta; la regla aplica también después de una navegación
  terminada.
- **FR-043**: El emisor MUST enviar la meta con la acción `nav2_msgs/action/NavigateToPose` sobre
  `navigate_to_pose`, esperar al servidor con tiempo límite y registrar la aceptación, el progreso
  y el resultado.
- **FR-044**: El emisor MUST NOT reenviar metas mientras haya una activa, salvo que el objetivo
  se haya movido más que un umbral configurable. Como los objetos son estáticos, el emisor MUST
  descartar con una advertencia los objetivos que salten más que un máximo configurable (1.5 m
  por defecto) respecto al último usado. En simulación se observó un objetivo fantasma a ~7 m,
  cuando la profundidad vio el fondo a través de la silla tras llegar.
- **FR-045**: El emisor MUST ofrecer el parámetro `dry_run`, activado por defecto, que calcula la
  meta sin enviarla.
- **FR-046**: En ambos modos, el emisor MUST publicar la meta calculada como
  `geometry_msgs/PoseStamped` en `/goal_pose_preview`.
- **FR-047**: La lógica de cálculo de la meta MUST poder verificarse con pruebas automáticas sin
  levantar el sistema.

**Paquete, lanzamiento y documentación**

- **FR-050**: El sistema MUST entregarse como un solo paquete con cinco ejecutables
  independientes (cámara, detector, visualización, localizador RGB-D y emisor de metas), la
  herramienta de profundidad sintética (FR-035) como ejecutable de prueba aparte, un archivo de
  parámetros con todos los valores por defecto, archivos de lanzamiento y pruebas.
- **FR-051**: El paquete MUST incluir tres lanzamientos: pipeline 2D (cámara, detector y
  visualización); sistema completo en `dry_run` con el video, la profundidad sintética y
  transformaciones estáticas; y simulación en Gazebo con robot, silla, navegación y nodos de
  percepción.
- **FR-052**: El README MUST cubrir requisitos, instalación (incluido el entorno de Python y las
  dependencias de simulación), compilación, ejecución de cada lanzamiento, parámetros, comandos
  de verificación, problemas conocidos y qué se validó frente a qué quedó como diseño.
- **FR-053**: La documentación MUST incluir el diagrama de nodos y tópicos y capturas del grafo
  del sistema en ejecución, del pipeline 2D y de la simulación.
- **FR-054**: La documentación MUST justificar cada decisión de arquitectura: reparto de nodos,
  tipos de mensaje, calidad de servicio, marcos, emparejamiento por estampa, estimación de
  profundidad, distancia de separación y `dry_run`.
- **FR-055**: El reporte técnico MUST ocupar de 1 a 2 páginas y contener el diagrama, las
  decisiones, el componente RGB-D, la conexión con Nav2, la tabla de validado frente a diseño, la
  comparación con la posición real en simulación y la autoevaluación según la rúbrica.
- **FR-056**: El video de demostración MUST durar de 2 a 4 minutos y mostrar compilación, pipeline
  2D, inspección de detecciones, grafo de nodos, manejo de una fuente inexistente, `dry_run` y la
  navegación en simulación.
- **FR-057**: Los identificadores del código MUST estar en inglés; los comentarios, mensajes de
  log, README y reporte MUST estar en español.
- **FR-058**: El paquete entregado MUST excluir artefactos de compilación y respetar los límites
  de Moodle: 50 MB por archivo y 20 archivos como máximo.

**Simulación**

- **FR-060**: La simulación MUST reutilizar un robot estándar con cámara RGB-D, mapa y
  localización ya provistos por la pila de navegación, sin construir un robot propio.
- **FR-061**: El mundo simulado MUST incluir una silla modelada en FreeCAD, exportada a un formato
  que el simulador cargue, con geometría de colisión para que la navegación la trate como
  obstáculo y con su pose real documentada.
- **FR-062**: La silla MUST quedar visible para la cámara del robot desde su pose inicial, a una
  distancia de entre 2.5 y 4 m, para que el robot recorra al menos 1.2 m antes de quedar dentro
  de la distancia de separación más la tolerancia de llegada.
- **FR-063**: En simulación, el nodo de cámara MUST NOT lanzarse; el detector, la visualización y
  el localizador MUST consumir la imagen, la profundidad y los intrínsecos simulados solo mediante
  remapeos y parámetros.
- **FR-064**: Con la simulación activa, todos los nodos MUST usar el reloj de la simulación.
- **FR-065**: El lanzamiento de la simulación MUST dejar al robot localizado en el mapa sin pasos
  manuales.
- **FR-066**: La simulación MUST ejecutarse con `dry_run` desactivado, de modo que la meta llegue
  al servidor de navegación real del entorno simulado.
- **FR-067**: Si el detector no alcanza el score mínimo con la silla de FreeCAD, el sistema MAY
  usar un modelo de silla de la biblioteca pública del simulador; la sustitución MUST declararse
  en el README y en el reporte.

- **FR-068**: El paquete MAY incluir un panel gráfico (`sim_panel`) para iniciar, activar la
  navegación, detener y reiniciar toda la simulación. Es una herramienta de demostración, no
  forma parte del pipeline, y "Detener" MUST dejar sin procesos de la simulación.

**Comportamiento general**

- **FR-070**: Todo nombre de tópico MUST ser remapeable y todo marco, ruta, umbral o valor
  ajustable MUST ser un parámetro con valor por defecto documentado.
- **FR-071**: Todo mensaje con `header` MUST llevar estampa válida y `frame_id` no vacío; los
  datos derivados de una imagen MUST conservar su estampa.
- **FR-072**: Un error recuperable MUST NOT terminar un nodo; cada uno MUST registrarse con un
  mensaje accionable y con límite de frecuencia.

### Key Entities *(include if feature involves data)*

- **Cuadro de imagen**: imagen a color de la fuente, con estampa de captura y marco de la cámara.
- **Detección 2D**: objeto encontrado en un cuadro: rectángulo en píxeles (centro y tamaño),
  clase y score; hereda la estampa y el marco del cuadro.
- **Imagen de profundidad e intrínsecos**: distancia por píxel alineada al color (milímetros o
  metros) y parámetros de la cámara (`fx`, `fy`, `cx`, `cy`). Provienen de la cámara simulada o,
  en `dry_run`, de la herramienta de profundidad sintética.
- **Detección 3D**: posición en metros de un objeto en el marco óptico, con la clase, el score y
  la estampa de la detección 2D de origen.
- **Meta de navegación**: pose en el marco del mapa, a la distancia de separación del objeto y
  orientada hacia él.
- **Árbol de marcos**: cadena mapa → odometría → base del robot → cámara → marco óptico.
- **Silla objetivo**: modelo 3D de una silla hecho en FreeCAD, con geometría visual y de colisión,
  faldón frontal bajo el asiento y pose real conocida en el mundo simulado. Su posición real de
  referencia es el centro de su huella (origen del modelo).
- **Tabla de validado frente a diseño**: por componente, cómo se validó y qué quedó sin probar.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: El paquete compila y el 100 % de sus pruebas automáticas pasan en el entorno
  objetivo.
- **SC-002**: Un solo comando levanta el pipeline 2D y muestra detecciones dibujadas en menos de
  30 segundos.
- **SC-003**: El detector entrega resultados a 5 cuadros por segundo o más en el equipo de
  desarrollo, sin GPU.
- **SC-004**: El 100 % de las detecciones publicadas trae rectángulo de tamaño mayor que cero,
  clase no vacía y score entre 0 y 1.
- **SC-005**: Con la fuente ausente, el nodo de cámara sigue vivo y vuelve a publicar en 10
  segundos o menos desde que la fuente regresa.
- **SC-006**: La deproyección reproduce puntos sintéticos con un error menor a 1 mm.
- **SC-007**: En `dry_run`, la meta publicada queda a la distancia de separación del objeto con
  tolerancia de 1 cm y orientada hacia él con un error menor a 1°.
- **SC-008**: En simulación, la posición estimada de la silla queda a 0.30 m o menos de su
  posición real (centro de su huella).
- **SC-009**: En simulación, en al menos 4 de 5 ejecuciones el robot termina a la distancia de
  separación del centro de la huella de la silla (±0.30 m) y mirando hacia ella (error de 15° o
  menos).
- **SC-010**: En un entorno limpio (Ubuntu 24.04 con ROS 2 Jazzy recién instalado, sin el
  historial del autor), se reproducen el pipeline 2D, el `dry_run` y la simulación siguiendo solo
  el README.
- **SC-011**: Los entregables cumplen los límites de Moodle: archivos de 50 MB o menos, 20
  archivos como máximo y video de 2 a 4 minutos.
- **SC-012**: La autoevaluación del reporte asigna a cada uno de los seis criterios de la rúbrica
  un nivel respaldado por evidencia que se puede señalar en el video, el código o el reporte.

## Assumptions

- **Entorno**: Ubuntu 24.04 sobre WSL2 con ROS 2 Jazzy; 8 núcleos de CPU y sin GPU NVIDIA. Están
  instalados Gazebo Harmonic, la integración ROS–Gazebo, la pila de navegación y la simulación
  mínima del TurtleBot 4 de Nav2 (`nav2_minimal_tb4_sim`, no el paquete `turtlebot4_simulator`),
  que trae cámara RGB-D, mapas y localización.
- **Alcance**: una sola especificación con historias priorizadas, en lugar de varias
  funcionalidades separadas.
- **Modelo**: YOLO nano preentrenado con pesos COCO; entrenar o ajustar el modelo queda fuera del
  alcance. Los pesos se descargan o se instalan siguiendo el README y no se versionan.
- **Video fuente**: el autor graba con el celular un video corto (720p, menos de un minuto) con
  personas y objetos de clases COCO, sillas incluidas. WSL2 no expone webcam; la webcam por usbipd
  es opcional.
- **Clase objetivo**: `chair`, tanto en `dry_run` como en simulación.
- **Profundidad**: alineada al color, con la misma resolución e intrínsecos (se cumple en la
  cámara simulada).
- **FreeCAD**: la silla se modela con FreeCAD, de preferencia mediante un servidor MCP. Al
  2026-10-06 ni FreeCAD ni su MCP están disponibles en el entorno; MUST quedar instalado o
  conectado antes de implementar la historia 5.
- **Respaldo de la silla**: si el render sintético de la silla de FreeCAD no alcanza el score
  mínimo, se usa un modelo de silla de la biblioteca pública del simulador y se declara.
- **Simulación obligatoria**: la entrega se considera completa solo cuando las cinco historias
  cumplen sus criterios, incluida la simulación. El orden de trabajo sigue las prioridades (la
  simulación va al final), pero no hay respaldo que la sustituya en los entregables.
- **Rendimiento de la simulación**: sin GPU, Gazebo, la navegación y el detector juntos pueden
  correr más lento que el tiempo real; los criterios de la historia 5 no exigen tiempo real.
- **Entrega individual**: un solo autor y mantenedor en el paquete, el README y el reporte.
- **Rúbrica**: Insuficiente de 0 a 1, Aceptable de 2 a 3, Sobresaliente 4. La meta es
  Sobresaliente en los seis criterios.
- **Fuera de alcance**: mensajes personalizados, robot físico, construir un modelo de robot propio,
  generar mapas nuevos y objetos en movimiento.
