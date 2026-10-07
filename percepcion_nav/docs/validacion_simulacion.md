# Validación en simulación (US5)

Entorno: Ubuntu 24.04 en WSL2, 8 núcleos, sin GPU; Gazebo Harmonic (gz 8.11) con render por
software (`llvmpipe`, OpenGL 4.5); ROS 2 Jazzy con `nav2_minimal_tb4_sim`, mundo y mapa `depot`.
Fecha: 2026-10-06.

## Prueba previa (T041)

Simulación original de Nav2 (`tb4_simulation_launch.py headless:=True`), sin nada del paquete:

| Medición | Resultado |
|---|---|
| Primera imagen de `/rgbd_camera/image` | 23 s después del lanzamiento (incluye descargar el modelo Depot de Fuel) |
| Frecuencia de la cámara | ~5 Hz en tiempo real (10 Hz en tiempo simulado) |
| Factor de tiempo real | 0.49 |
| `frame_id` de color y profundidad | `oakd_rgb_camera_optical_frame` (confirma research R1) |
| Codificación de la profundidad | `32FC1` (metros) |
| Intrínsecos | `fx = fy = 221.8`, `cx = 160`, `cy = 120` (320 × 240) |
| AMCL | "Please set the initial pose": sin `nav2_params_sim.yaml` no localiza solo (confirma R2) |

Conclusión: el render por software alcanza; la simulación corre a la mitad del tiempo real, lo
que la especificación acepta.

## Pose real: cómo se mide

`gz model -m turtlebot4 -p` no responde con esta carga (el servicio de estado del mundo vence a
los 5 s), y el modo sin interfaz no publica un tópico de poses. Por eso la pose real del robot
se toma de `/odom` de Gazebo: la odometría del plugin DiffDrive, calculada de las ruedas
simuladas y sin ruido añadido. Su origen es la pose de aparición, mundo `(-8, 0, 0)` =
`map (0, 0, 0)`. La silla está en el mundo `(-5, 0)`, es decir, en `map (3.0, 0.0)`: es el centro
de su huella (punto de referencia de SC-008 y SC-009). Limitación declarada: es la odometría
ideal del simulador, no un *ground truth* independiente.

## Silla de FreeCAD: iteraciones de detección (T049)

Imagen de la cámara del robot en su pose inicial, con la silla a 3 m; YOLO11n con umbral 0.05,
evaluado con 320 y 640 px de entrada.

| Modelo | Mejor score de `chair` | Observación |
|---|---|---|
| FreeCAD v1: cajas de un color (café, blanco, negro o azul), 4 orientaciones, 3 y 3.5 m | 0.09 | Sin forma de silla para el modelo; lo confunde con `bench` o `car` |
| Fuel `OpenRobotics/Chair` (referencia) | 0.86 | La resolución de la cámara alcanza |
| Fuel `OpenRobotics/WoodenChair` | 0.33 | Respaldo abierto: la profundidad ve el fondo (estimación a 11 m) |
| **FreeCAD v2: butaca, estructura de madera y cojines azules** | **0.87** (640 px, pose inicial) | Supera el umbral de 0.40; es el modelo entregado. Durante el acercamiento, hasta 0.90 |

No se necesitó el respaldo de Fuel (FR-067): la silla de la simulación es la de FreeCAD.

## Hallazgos de la primera corrida completa y correcciones

Con la silla v2, la estimación sobre la cara visible y la tolerancia de Nav2 por defecto:

- Objetivo estimado en `map` (2.75, 0.00): 0.25 m del centro. Es justo la cara frontal del
  cojín, a 0.25 m del centro.
- El robot terminó a 1.35 m del centro (meta a 1.0 m de la cara frontal, más la parada de Nav2
  hasta 0.10–0.25 m antes de la meta). **No** cumple SC-009 (límite 1.30 m).
- Justo después del éxito se envió una meta extra, a partir de una detección tomada antes de
  llegar (contra FR-042).
- Mientras Nav2 arranca, rechaza metas; el aviso se repetía a ~5 Hz (contra FR-072).

Correcciones (research R10, FR-036):

1. `estimate_center: true` en simulación.
2. `xy_goal_tolerance: 0.10`.
3. El emisor ignora detecciones anteriores al último resultado.
4. El aviso de rechazo tiene límite de frecuencia.

## Corridas de aceptación (T050)

Cinco lanzamientos independientes de `simulation.launch.py` (sin RViz), con la configuración
final:
- silla v2;
- `estimate_center: true`;
- `xy_goal_tolerance: 0.10`;
- guarda de detecciones anteriores al último resultado.

Cada corrida termina 25 s después del resultado de la acción, para observar si se envían metas
de más.

| Corrida | Objetivo estimado en `map` | Error vs. centro (SC-008, ≤ 0.30 m) | Distancia final al centro (SC-009, 1.0 ± 0.30 m) | Error de orientación (≤ 15°) | Resultado | Metas aceptadas | Metas tras llegar | Localización tras llegar: tf `map→base_link` vs. odometría | Tiempo hasta el resultado |
|---|---|---|---|---|---|---|---|---|---|
| 1 | (3.04, 0.01) | 0.04 m ✓ | 1.025 m ✓ | 1.4° ✓ | SUCCEEDED | 1 | 0 | 0.022 m | 33 s |
| 2 | (3.04, 0.01) | 0.04 m ✓ | 1.088 m ✓ | 1.3° ✓ | SUCCEEDED | 1 | 0 | 0.074 m | 32 s |
| 3 | (3.04, 0.01) | 0.04 m ✓ | 1.071 m ✓ | 0.9° ✓ | SUCCEEDED | 1 | 0 | 0.049 m | 30 s |
| 4 | (3.04, 0.01) | 0.04 m ✓ | 1.097 m ✓ | 2.2° ✓ | SUCCEEDED | 1 | 0 | — | 29 s |
| 5 | (3.04, 0.01) | 0.04 m ✓ | 1.082 m ✓ | 2.2° ✓ | SUCCEEDED | 1 | 0 | — | 32 s |


Notas:

- **SC-008 se cumple en 5/5 corridas y SC-009 en 5/5** (se exige al menos 4 de 5).
- "Objetivo estimado" es la primera estimación, con el robot en su pose inicial. Durante el
  trayecto se registraron estimaciones entre (3.03, 0.01) y (3.07, 0.11).
- "Metas aceptadas" = 1. Antes de que Nav2 termine de activarse se rechazan varias metas (entre 7 y
  10 envíos por corrida), cuyo aviso tiene límite de frecuencia. Al activarse, Nav2 acepta una
  sola meta y no hay reemplazos.
- "Localización tras llegar" compara la pose del robot según tf (`map → base_link`, la que usa
  el emisor) con la odometría de Gazebo, en la última detección posterior a la llegada. En las
  corridas 4 y 5, a 1 m la silla queda cortada en la imagen y ya no se detecta, así que no hay
  registro posterior a la llegada ("—").
- Una corrida adicional, grabada para las figuras del reporte, terminó a 1.10 m del centro, con
  la silla detectada en 451 cuadros y scores de hasta 0.90.
- Evidencia: `docs/reporte/figuras/sim_camara.png` y `sim_trayectoria.png`.

## Hallazgos posteriores a las corridas

- **Colisión de `params_file`**: el argumento `params_file` de `simulation.launch.py` tiene el
  mismo nombre que el del launch de Nav2 incluido, y este lo sobrescribía. Los nodos de
  percepción recibían `nav2_params_sim.yaml` en lugar de `params.yaml`. Durante las 5 corridas
  usaron los valores declarados en su código, que son idénticos a `params.yaml` por diseño
  (principio I), más `params_sim.yaml`. Por eso **los resultados de la tabla siguen siendo
  válidos**. Se corrigió aislando el launch de Nav2 en un `GroupAction(scoped=True)`, y se
  verificó en ejecución que los nodos reciben `params.yaml` + `params_sim.yaml`.
- **Objetivo fantasma tras llegar**: en una corrida posterior, ya frente a la silla, la ventana de
  profundidad vio el fondo a través de un hueco de la silla. El objetivo saltó a ~(9.9, 0.9) y el
  robot recibió metas hacia allá. Como los objetos son estáticos, el emisor ahora descarta saltos
  mayores que `max_target_jump` = 1.5 m respecto al último objetivo usado (FR-044). El caso es
  intermitente: no apareció en las 5 corridas ni en las pruebas posteriores del panel.

## Cambio de fuente sin cambiar código (US5, escenario 5; T051)

Pasar del video a la cámara simulada solo requirió remapeos (`/camera/*` → `/rgbd_camera/*`) y
parámetros (`params_sim.yaml`, `use_sim_time`), sin modificar ningún nodo. Durante US5 sí
cambió código de dos nodos, por hallazgos de la validación y no por la fuente:

- `rgbd_localizer_node`: parámetro opcional `estimate_center` (FR-036).
- `nav2_goal_sender_node`: se ignoran detecciones anteriores al último resultado (FR-042), y el
  aviso de rechazo tiene límite de frecuencia (FR-072).

Ambos cambios también aplican al video y al `dry_run`. `estimate_center` está desactivado por
defecto.

