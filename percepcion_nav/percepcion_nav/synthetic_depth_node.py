"""
Herramienta de prueba (FR-035): profundidad e intrínsecos sintéticos para el modo dry_run.

Por cada cuadro de color publica una imagen de profundidad constante (32FC1, metros) del mismo
tamaño y un CameraInfo de una cámara ideal, ambos con el header del cuadro. Así el localizador
RGB-D se ejecuta con detecciones reales del video sin una cámara RGB-D. No se usa en simulación.
"""

from cv_bridge import CvBridge
import numpy as np
from percepcion_nav.geometry import intrinsics_from_hfov
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# Periodo mínimo entre advertencias repetidas (contracts/parameters.md, constantes internas).
LOG_THROTTLE_SEC = 5.0


class SyntheticDepthNode(Node):
    """Publica profundidad constante e intrínsecos alineados a cada cuadro de color."""

    def __init__(self):
        super().__init__('synthetic_depth')
        self.declare_parameter('depth_m', 2.5, ParameterDescriptor(
            description='Profundidad constante publicada, en metros (32FC1).'))
        self.declare_parameter('hfov_rad', 1.20, ParameterDescriptor(
            description='Campo de visión horizontal para calcular los intrínsecos, en rad.'))
        self._depth_m = float(self.get_parameter('depth_m').value)
        self._hfov = float(self.get_parameter('hfov_rad').value)
        if self._depth_m <= 0.0 or not 0.0 < self._hfov < np.pi:
            raise ValueError('depth_m debe ser > 0 y hfov_rad debe estar en (0, pi).')

        self._bridge = CvBridge()
        self._cached_shape = None
        self._cached_depth = None
        input_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        output_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self._depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', output_qos)
        self._info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', output_qos)
        self.create_subscription(Image, 'camera/image_raw', self._on_image, input_qos)
        self.get_logger().info(
            f'Profundidad sintética de {self._depth_m:.2f} m con campo de visión de '
            f'{self._hfov:.2f} rad.')

    def _depth_for(self, height, width):
        """Reutiliza la matriz de profundidad mientras no cambie el tamaño de la imagen."""
        if self._cached_shape != (height, width):
            self._cached_depth = np.full((height, width), self._depth_m, dtype=np.float32)
            self._cached_shape = (height, width)
        return self._cached_depth

    def _on_image(self, msg):
        if msg.width == 0 or msg.height == 0:
            self.get_logger().warning('Cuadro vacío recibido; se descarta.',
                                      throttle_duration_sec=LOG_THROTTLE_SEC)
            return
        try:
            depth_msg = self._bridge.cv2_to_imgmsg(
                self._depth_for(msg.height, msg.width), encoding='32FC1')
        except Exception as exc:  # noqa: B902 - una conversión fallida no debe tumbar el nodo
            self.get_logger().warning(f'No se pudo crear la profundidad: {exc}.',
                                      throttle_duration_sec=LOG_THROTTLE_SEC)
            return
        depth_msg.header = msg.header

        intr = intrinsics_from_hfov(msg.width, msg.height, self._hfov)
        info = CameraInfo()
        info.header = msg.header
        info.width = msg.width
        info.height = msg.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [intr.fx, 0.0, intr.cx, 0.0, intr.fy, intr.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [intr.fx, 0.0, intr.cx, 0.0, 0.0, intr.fy, intr.cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self._depth_pub.publish(depth_msg)
        self._info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SyntheticDepthNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
