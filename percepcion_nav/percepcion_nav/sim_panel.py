"""
Panel de control de la simulación (herramienta de demostración, no es parte del pipeline).

Ventana con botones para iniciar, iniciar la navegación, detener y reiniciar toda la simulación
(`simulation.launch.py`). Se ejecuta desde una terminal con el entorno cargado:

    ros2 run percepcion_nav sim_panel               # abre el panel
    ros2 run percepcion_nav sim_panel --autostart   # abre el panel e inicia la simulación

La simulación se lanza en su propio grupo de procesos, así que "Detener" termina también Gazebo,
Nav2 y los puentes. No es un nodo de ROS: usa `ros2 launch` y `ros2 param set`.
"""

import os
import queue
import re
import signal
import subprocess
import sys
import threading

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QPlainTextEdit, QPushButton, QVBoxLayout, QWidget)

# Constantes internas de la herramienta (cadencias de la interfaz, no del sistema).
POLL_PERIOD_MS = 200
STOP_GRACE_SEC = 12.0
LOG_MAX_LINES = 400

ANSI = re.compile(r'\x1b\[[0-9;]*m')
# '[proceso-N] [INFO] [estampa] [nodo]: mensaje' -> 'INFO  [nodo] mensaje'
ROS_LOG = re.compile(r'^\[[^\]]+\] \[(\w+)\] \[[\d.]+\] (\[[^\]]+\]): (.*)$')
OUR_NODES = ('[camera]', '[detector]', '[visualizer]', '[rgbd_localizer]', '[nav2_goal_sender]',
             '[spawn_chair]', '[synthetic_depth]')
# Procesos que pueden quedar vivos si la simulación no termina limpia (patrones anclados).
LEFTOVER_PATTERNS = ('^gz sim', '^ruby .*gz sim', '^/opt/ros/jazzy/lib/ros_gz_bridge/',
                     '^/opt/ros/jazzy/lib/ros_gz_image/',
                     '^/opt/ros/jazzy/lib/rclcpp_components/component_container')

STATES = {
    'stopped': ('Detenida', '#6b7280'),
    'starting': ('Iniciando…', '#d97706'),
    'running': ('En ejecución', '#2563eb'),
    'waiting': ('Lista: esperando “Iniciar navegación”', '#7c3aed'),
    'navigating': ('Navegando hacia la silla', '#0891b2'),
    'arrived': ('Llegó frente a la silla ✓', '#16a34a'),
    'stopping': ('Deteniendo…', '#d97706'),
}


def launch_command(show_gazebo, use_rviz, viewer, wait_to_navigate):
    """Arma el comando de `ros2 launch` según las opciones del panel."""
    return ['ros2', 'launch', 'percepcion_nav', 'simulation.launch.py',
            f'headless:={"False" if show_gazebo else "True"}',
            f'use_rviz:={"true" if use_rviz else "false"}',
            f'viewer:={"true" if viewer else "false"}',
            f'dry_run:={"true" if wait_to_navigate else "false"}']


def state_from_line(line, current):
    """Deduce el estado de la simulación a partir de una línea del registro."""
    if 'terminada: SUCCEEDED' in line:
        return 'arrived'
    if 'Enviando meta' in line or 'Nav2 aceptó la meta' in line:
        return 'navigating'
    # Listo cuando el emisor de metas ya ve el objetivo: la cadena completa funciona.
    if current == 'starting' and '[nav2_goal_sender]' in line and 'Objetivo "' in line:
        return 'running'
    return current


class SimPanel(QWidget):
    """Ventana con las acciones de la simulación y un registro filtrado de los nodos."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('percepcion_nav · Panel de simulación')
        self._process = None
        self._lines = queue.Queue()
        self._state = 'stopped'
        self._restart_pending = False
        self._stop_deadline = None
        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(POLL_PERIOD_MS)
        self._set_state('stopped')

    # --- Interfaz -------------------------------------------------------------------------

    def _build_ui(self):
        self.status = QLabel()
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setFont(QFont('Sans', 13, QFont.Bold))
        self.status.setMinimumHeight(38)

        options = QGroupBox('Opciones (se aplican al iniciar)')
        grid = QGridLayout(options)
        self.opt_gazebo = QCheckBox('Ver Gazebo (más lento sin GPU)')
        self.opt_rviz = QCheckBox('RViz de Nav2 (mapa, robot, ruta)')
        self.opt_viewer = QCheckBox('Ver detecciones (rqt_image_view)')
        self.opt_wait = QCheckBox('Esperar para navegar (dry_run al inicio)')
        boxes = (self.opt_gazebo, self.opt_rviz, self.opt_viewer, self.opt_wait)
        for i, box in enumerate(boxes):
            box.setChecked(True)
            grid.addWidget(box, i // 2, i % 2)

        self.btn_start = QPushButton('▶  Iniciar simulación')
        self.btn_nav = QPushButton('→  Iniciar navegación')
        self.btn_stop = QPushButton('■  Detener todo')
        self.btn_restart = QPushButton('⟳  Reiniciar')
        self.btn_start.clicked.connect(self.start)
        self.btn_nav.clicked.connect(self.start_navigation)
        self.btn_stop.clicked.connect(self.stop)
        self.btn_restart.clicked.connect(self.restart)
        buttons = QHBoxLayout()
        for button in (self.btn_start, self.btn_nav, self.btn_stop, self.btn_restart):
            button.setMinimumHeight(36)
            buttons.addWidget(button)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(LOG_MAX_LINES)
        self.log.setFont(QFont('Monospace', 9))
        self.log.setPlaceholderText('Registro de los nodos de percepción y de la navegación')

        layout = QVBoxLayout(self)
        layout.addWidget(self.status)
        layout.addWidget(options)
        layout.addLayout(buttons)
        layout.addWidget(self.log, stretch=1)
        self.resize(760, 480)

    def _set_state(self, state):
        self._state = state
        text, color = STATES[state]
        print(f'[panel] {text}', flush=True)
        self.status.setText(text)
        self.status.setStyleSheet(f'color: white; background: {color}; border-radius: 6px;')
        running = self._process is not None
        self.btn_start.setEnabled(not running)
        self.btn_nav.setEnabled(running and state in ('running', 'waiting'))
        self.btn_stop.setEnabled(running and state != 'stopping')
        self.btn_restart.setEnabled(running and state != 'stopping')
        for box in (self.opt_gazebo, self.opt_rviz, self.opt_viewer, self.opt_wait):
            box.setEnabled(not running)

    def _append(self, text):
        match = ROS_LOG.match(text)
        if match:
            text = f'{match.group(1):5s} {match.group(2)} {match.group(3)}'
        self.log.appendPlainText(text)

    # --- Acciones -------------------------------------------------------------------------

    def start(self):
        """Lanza la simulación completa en un grupo de procesos propio."""
        if self._process is not None:
            return
        command = launch_command(self.opt_gazebo.isChecked(), self.opt_rviz.isChecked(),
                                 self.opt_viewer.isChecked(), self.opt_wait.isChecked())
        self._append('$ ' + ' '.join(command))
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0', PYTHONUNBUFFERED='1')
        # Las ventanas de Gazebo, RViz y rqt usan la pantalla normal aunque el panel no la use.
        env.pop('QT_QPA_PLATFORM', None)
        self._process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True, bufsize=1,
                                         start_new_session=True, env=env)
        threading.Thread(target=self._read_output, args=(self._process,), daemon=True).start()
        self._set_state('starting')

    def start_navigation(self):
        """Desactiva dry_run en el emisor de metas para que el robot navegue a la silla."""
        self._append('$ ros2 param set /nav2_goal_sender dry_run false')
        threading.Thread(target=self._run_and_report,
                         args=(['ros2', 'param', 'set', '/nav2_goal_sender', 'dry_run', 'false'],),
                         daemon=True).start()

    def stop(self):
        """Envía Ctrl+C a todo el grupo; si no termina a tiempo, lo fuerza."""
        if self._process is None or self._state == 'stopping':
            return
        self._send_signal(signal.SIGINT)
        self._stop_deadline = STOP_GRACE_SEC * 1000.0
        self._set_state('stopping')

    def restart(self):
        """Detiene y vuelve a iniciar con las mismas opciones."""
        if self._process is None:
            self.start()
            return
        self._restart_pending = True
        self.stop()

    # --- Procesos -------------------------------------------------------------------------

    def _send_signal(self, sig):
        try:
            os.killpg(os.getpgid(self._process.pid), sig)
        except ProcessLookupError:
            pass

    def _read_output(self, process):
        for line in process.stdout:
            self._lines.put(ANSI.sub('', line.rstrip()))

    def _run_and_report(self, command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        self._lines.put((result.stdout or result.stderr).strip() or 'sin respuesta')

    def _cleanup_leftovers(self):
        for pattern in LEFTOVER_PATTERNS:
            subprocess.run(['pkill', '-9', '-f', pattern], capture_output=True)

    def _poll(self):
        while not self._lines.empty():
            line = self._lines.get()
            ours = any(tag in line for tag in OUR_NODES) or line.startswith(
                ('$', 'Set parameter', 'Setting parameter', 'sin respuesta'))
            # Al detener, los errores de Nav2 por apagado a mitad del arranque son esperables.
            if ours or ('ERROR' in line and self._state != 'stopping'):
                self._append(line)
            new_state = state_from_line(line, self._state)
            if self._state == 'starting' and new_state == 'running' and \
                    self.opt_wait.isChecked():
                new_state = 'waiting'
            if new_state != self._state and self._state != 'stopping':
                self._set_state(new_state)

        if self._process is None:
            return
        if self._state == 'stopping':
            self._stop_deadline -= POLL_PERIOD_MS
            if self._process.poll() is None and self._stop_deadline <= 0:
                self._append('La simulación no terminó a tiempo: se fuerza el cierre.')
                self._send_signal(signal.SIGKILL)
        if self._process.poll() is not None:
            self._cleanup_leftovers()
            self._append(f'Simulación terminada (código {self._process.returncode}).')
            self._process = None
            self._set_state('stopped')
            if self._restart_pending:
                self._restart_pending = False
                self.start()

    def closeEvent(self, event):  # noqa: N802 - nombre impuesto por Qt
        """Detiene la simulación al cerrar la ventana."""
        if self._process is not None:
            self._send_signal(signal.SIGINT)
            try:
                self._process.wait(timeout=STOP_GRACE_SEC)
            except subprocess.TimeoutExpired:
                self._send_signal(signal.SIGKILL)
            self._cleanup_leftovers()
        event.accept()


def main(args=None):
    argv = sys.argv if args is None else args
    app = QApplication(argv)
    panel = SimPanel()
    panel.show()
    # Ctrl+C o una terminación cierran la ventana, y al cerrarla se detiene la simulación.
    # El temporizador de sondeo del panel deja que Python atienda estas señales.
    signal.signal(signal.SIGINT, lambda *_: panel.close())
    signal.signal(signal.SIGTERM, lambda *_: panel.close())
    if '--autostart' in argv:
        QTimer.singleShot(500, panel.start)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
