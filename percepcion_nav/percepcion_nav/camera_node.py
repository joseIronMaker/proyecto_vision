"""Nodo de cámara: publica los cuadros de un video o dispositivo como sensor_msgs/Image."""

import cv2
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# Periodo mínimo entre advertencias repetidas (contracts/parameters.md, constantes internas).
LOG_THROTTLE_SEC = 5.0

STATE_RETRYING = 'RETRYING'
STATE_STREAMING = 'STREAMING'


class CameraNode(Node):
    """Lee una fuente de video con OpenCV y publica cada cuadro con estampa y marco."""

    def __init__(self):
        super().__init__('camera')
        self.declare_parameter('source', '', ParameterDescriptor(
            description='Ruta de video, o índice de dispositivo si es numérico.'))
        self.declare_parameter('fps', 15.0, ParameterDescriptor(
            description='Frecuencia de publicación en Hz.'))
        self.declare_parameter('frame_id', 'camera_color_optical_frame', ParameterDescriptor(
            description='header.frame_id de cada cuadro (marco óptico, REP 103).'))
        self.declare_parameter('loop', True, ParameterDescriptor(
            description='Reiniciar el video al terminar; si es false, el nodo se apaga.'))
        self.declare_parameter('reconnect_period_sec', 2.0, ParameterDescriptor(
            description='Periodo de reintento al no abrir o perder la fuente, en s.'))

        self._source = str(self.get_parameter('source').value).strip()
        self._frame_id = self.get_parameter('frame_id').value
        self._loop = bool(self.get_parameter('loop').value)
        fps = float(self.get_parameter('fps').value)
        reconnect_period = float(self.get_parameter('reconnect_period_sec').value)
        if fps <= 0.0 or reconnect_period <= 0.0:
            raise ValueError('fps y reconnect_period_sec deben ser mayores que 0.')

        self._bridge = CvBridge()
        self._capture = None
        self._is_file = False
        self._frame_count = 0
        self._state = STATE_RETRYING
        # El main() termina cuando el video acaba sin repetición (FR-005).
        self.finished = False

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self._publisher = self.create_publisher(Image, 'camera/image_raw', qos)

        self._open_source()
        self.create_timer(1.0 / fps, self._on_frame_timer)
        self.create_timer(reconnect_period, self._on_reconnect_timer)

    def _open_source(self):
        """Intenta abrir la fuente; registra el error y deja el reintento al temporizador."""
        if not self._source:
            self.get_logger().error(
                'El parámetro "source" está vacío: indica la ruta de un video o un índice de '
                'dispositivo. Se reintentará.', throttle_duration_sec=LOG_THROTTLE_SEC)
            return False

        target = int(self._source) if self._source.isdigit() else self._source
        capture = cv2.VideoCapture(target)
        if not capture.isOpened():
            capture.release()
            self.get_logger().error(
                f'No se pudo abrir la fuente "{self._source}". Se reintentará cada '
                f'{self.get_parameter("reconnect_period_sec").value} s.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return False

        self._capture = capture
        self._is_file = not isinstance(target, int)
        self._frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) if self._is_file else 0
        self._state = STATE_STREAMING
        self.get_logger().info(f'Fuente abierta: "{self._source}".')
        return True

    def _release(self):
        """Libera la captura actual, si existe."""
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        self._state = STATE_RETRYING

    def _at_end_of_file(self):
        """Indica si la última lectura fallida se debe al fin de un archivo de video."""
        if not self._is_file or self._capture is None or self._frame_count <= 0:
            return False
        position = int(self._capture.get(cv2.CAP_PROP_POS_FRAMES))
        return position >= self._frame_count - 1

    def _on_reconnect_timer(self):
        if self._state == STATE_RETRYING and not self.finished:
            self._open_source()

    def _on_frame_timer(self):
        if self._state != STATE_STREAMING or self._capture is None:
            return

        ok, frame = self._capture.read()
        if not ok or frame is None:
            if self._at_end_of_file():
                if self._loop:
                    self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self.get_logger().info('Fin del video: se reinicia desde el primer cuadro.')
                else:
                    self.get_logger().info('Fin del video sin repetición: el nodo se apaga.')
                    self._release()
                    self.finished = True
                return
            self.get_logger().warning(
                f'Falló la lectura de un cuadro de "{self._source}"; se libera la fuente y se '
                'intentará reconectar.', throttle_duration_sec=LOG_THROTTLE_SEC)
            self._release()
            return

        try:
            msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as exc:  # noqa: B902 - un cuadro corrupto no debe tumbar el nodo
            self.get_logger().warning(
                f'No se pudo convertir un cuadro: {exc}. Se descarta.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._publisher.publish(msg)

    def destroy_node(self):
        """Libera la fuente de video antes de destruir el nodo (FR-006)."""
        self._release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
