#!/usr/bin/env bash
# Demostración completa en una sola terminal. Enter avanza de un paso al siguiente.
#   1. Video: cámara desconectada -> reintentos -> se conecta -> detecciones dibujadas.
#   2. Grafo de nodos y tópicos (rqt_graph).
#   3. Simulación en Gazebo con Nav2: el robot detecta la silla y navega hasta ella.
#   Al final abre el reporte técnico (PDF).
# Uso: percepcion_nav/scripts/demo.sh   (desde cualquier carpeta)

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO" || exit 1
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate || { echo "Falta el entorno .venv: ver README, sección Instalación."; exit 1; }
if [ ! -f install/setup.bash ]; then
  echo "Compilando el paquete por primera vez..."
  colcon build || exit 1
fi
source install/setup.bash
export RCUTILS_LOGGING_BUFFERED_STREAM=0

VIDEO="$REPO/percepcion_nav/media/sample.mp4"
[ -f "$VIDEO" ] || VIDEO="$REPO/percepcion_nav/media/kitti_prueba.mp4"
TMP="$(mktemp -d /tmp/percepcion_demo.XXXX)"
FUENTE="$TMP/camara.mp4"
LOG="$TMP/pipeline.log"
LAUNCH_PGID=""
GRAPH_PID=""

titulo() { printf '\n\033[1;36m=== %s ===\033[0m\n' "$1"; }
aviso() { printf '\033[0;32m%s\033[0m\n' "$1"; }
esperar_enter() { printf '\n\033[1;33m>> %s\033[0m' "$1"; read -r _; }
mostrar_log() {
  sed 's/\x1b\[[0-9;]*m//g' "$LOG" | grep -E "\[(camera|detector)\]" \
    | sed -E 's/^\[[^]]+\] \[([A-Z]+)\] \[[0-9.]+\] (\[[a-z_]+\]):/\1 \2/' | tail -n "${1:-3}"
}
esperar_log() {
  for _ in $(seq "${2:-30}"); do grep -q "$1" "$LOG" && return 0; sleep 1; done
  return 1
}
detener_video() {
  [ -n "$GRAPH_PID" ] && kill "$GRAPH_PID" 2>/dev/null
  if [ -n "$LAUNCH_PGID" ]; then
    kill -INT -- "-$LAUNCH_PGID" 2>/dev/null
    for _ in $(seq 10); do kill -0 "$LAUNCH_PGID" 2>/dev/null || break; sleep 1; done
    kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null
  fi
  LAUNCH_PGID=""; GRAPH_PID=""
}
trap 'detener_video; rm -rf "$TMP"' EXIT

# --- 1. Cámara -> detector -> visualización ----------------------------------------------------
titulo "1/3 · Cámara → detector → visualización"
aviso "Arranco con la cámara desconectada (la fuente todavía no existe)..."
: > "$LOG"
setsid ros2 launch percepcion_nav pipeline_2d.launch.py video:="$FUENTE" > "$LOG" 2>&1 &
LAUNCH_PGID=$!
esperar_log "No se pudo abrir" 30
mostrar_log 2
esperar_enter "Enter para CONECTAR la cámara..."

cp "$VIDEO" "$FUENTE"
esperar_log "Fuente abierta" 15
mostrar_log 1
esperar_log "cargado en" 30
aviso "Detecciones en la ventana rqt_image_view. Un mensaje de /detections (vision_msgs/Detection2DArray):"
timeout 30 ros2 topic echo /detections --once 2>/dev/null \
  | python3 "$REPO/percepcion_nav/scripts/resumen_detecciones.py"
esperar_enter "Enter para ver el GRAFO de nodos..."

# --- 2. Grafo ---------------------------------------------------------------------------------
titulo "2/3 · Grafo de nodos y tópicos"
ros2 run rqt_graph rqt_graph > /dev/null 2>&1 &
GRAPH_PID=$!
esperar_enter "Enter para pasar a la SIMULACIÓN (se cierran el video y el grafo)..."
detener_video

# --- 3. Simulación ----------------------------------------------------------------------------
titulo "3/3 · Simulación: TurtleBot 4 + Nav2 + silla de FreeCAD"
aviso "Se abre el panel y arranca Gazebo. Cuando diga 'Lista', pulsa '→ Iniciar navegación'."
aviso "Al terminar, pulsa '■ Detener todo' y cierra el panel."
ros2 run percepcion_nav sim_panel --autostart 2>/dev/null

# --- Cierre: reporte --------------------------------------------------------------------------
REPORTE="$REPO/percepcion_nav/docs/reporte/reporte_tecnico.pdf"
if [ -f "$REPORTE" ] && command -v explorer.exe > /dev/null; then
  aviso "Abriendo el reporte técnico para el cierre..."
  explorer.exe "$(wslpath -w "$REPORTE")"
fi
aviso "Fin de la demostración."
