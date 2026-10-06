#!/usr/bin/env bash
# Abre el panel de simulación e inicia la simulación de la silla (Gazebo + Nav2 + percepción).
# Uso: percepcion_nav/scripts/simulacion_silla.sh   (desde cualquier carpeta)
# En el panel: espera "Lista: esperando Iniciar navegación" y pulsa "→ Iniciar navegación".

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO" || exit 1
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate || { echo "Falta el entorno .venv: ver README, sección Instalación."; exit 1; }
if [ ! -f install/setup.bash ]; then
  echo "Compilando el paquete por primera vez..."
  colcon build || exit 1
fi
source install/setup.bash
exec ros2 run percepcion_nav sim_panel --autostart
