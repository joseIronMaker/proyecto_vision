"""Localizador RGB-D: detección 2D + profundidad + intrínsecos -> Detection3DArray (FR-030)."""

from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import message_filters
from percepcion_nav.geometry import (center_along_ray, central_window, deproject,
                                     depth_scale_for_encoding, intrinsics_from_k, metric_size,
                                     robust_depth)
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose

# Constantes internas no ajustables (contracts/parameters.md).
LOG_THROTTLE_SEC = 5.0
UNMATCHED_CHECK_PERIOD_SEC = 2.0


def build_detection3d(detection2d, xyz, size_xy, size_z=0.0):
    """
    Construye una Detection3D en el marco óptico a partir de su Detection2D de origen.

    Conserva el header, la clase y el score; la posición va en results[0].pose.pose.position y
    en bbox.center.position, con orientación identidad, y bbox.size = (ancho, alto, size_z).
    size_z es 0 si la profundidad del objeto se desconoce.
    """
    x, y, z = (float(value) for value in xyz)
    detection = Detection3D()
    detection.header = detection2d.header
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = detection2d.results[0].hypothesis.class_id
    hypothesis.hypothesis.score = detection2d.results[0].hypothesis.score
    hypothesis.pose.pose.position = Point(x=x, y=y, z=z)
    hypothesis.pose.pose.orientation.w = 1.0
    detection.results.append(hypothesis)
    detection.bbox.center.position = Point(x=x, y=y, z=z)
    detection.bbox.center.orientation.w = 1.0
    detection.bbox.size.x = float(size_xy[0])
    detection.bbox.size.y = float(size_xy[1])
    detection.bbox.size.z = float(size_z)
    return detection


def build_detection3d_array(header, detections):
    """Agrupa detecciones 3D con el header de la Detection2DArray de origen."""
    msg = Detection3DArray()
    msg.header = header
    msg.detections = list(detections)
    return msg


class RgbdLocalizerNode(Node):
    """Estima la posición 3D de cada detección con la mediana de una ventana de profundidad."""

    def __init__(self):
        super().__init__('rgbd_localizer')
        self.declare_parameter('sync_slop_sec', 0.02, ParameterDescriptor(
            description='Diferencia máxima entre estampas de detección y profundidad, en s.'))
        self.declare_parameter('sync_queue_size', 30, ParameterDescriptor(
            description='Cola del ApproximateTimeSynchronizer.'))
        self.declare_parameter('depth_window_fraction', 0.30, ParameterDescriptor(
            description='Lado de la ventana central como fracción del bbox, en (0, 1].'))
        self.declare_parameter('min_valid_pixels', 5, ParameterDescriptor(
            description='Mínimo de píxeles de profundidad válidos para aceptar la mediana.'))
        self.declare_parameter('estimate_center', False, ParameterDescriptor(
            description='Estimar el centro del objeto suponiendo que su profundidad es igual a '
                        'su ancho métrico; si es false, se publica la superficie visible.'))
        self._slop = float(self.get_parameter('sync_slop_sec').value)
        queue_size = int(self.get_parameter('sync_queue_size').value)
        self._fraction = float(self.get_parameter('depth_window_fraction').value)
        self._min_valid = int(self.get_parameter('min_valid_pixels').value)
        self._estimate_center = bool(self.get_parameter('estimate_center').value)
        if not 0.0 < self._fraction <= 1.0:
            raise ValueError('depth_window_fraction debe estar en (0, 1].')

        self._bridge = CvBridge()
        self._intrinsics = None
        self._received = 0
        self._matched = 0

        detections_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=queue_size,
                                    reliability=ReliabilityPolicy.RELIABLE)
        depth_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=queue_size,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        info_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        output_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)

        self._publisher = self.create_publisher(Detection3DArray, 'detections_3d', output_qos)
        self.create_subscription(CameraInfo, 'camera/camera_info', self._on_camera_info,
                                 info_qos)
        detections_sub = message_filters.Subscriber(self, Detection2DArray, 'detections',
                                                    qos_profile=detections_qos)
        depth_sub = message_filters.Subscriber(self, Image, 'camera/depth/image_raw',
                                               qos_profile=depth_qos)
        detections_sub.registerCallback(self._count_detection)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [detections_sub, depth_sub], queue_size=queue_size, slop=self._slop)
        self._sync.registerCallback(self._on_pair)
        self.create_timer(UNMATCHED_CHECK_PERIOD_SEC, self._check_unmatched)

    def _on_camera_info(self, msg):
        intrinsics = intrinsics_from_k(msg.k)
        if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
            self.get_logger().warning('CameraInfo con fx o fy no positivos; se ignora.',
                                      throttle_duration_sec=LOG_THROTTLE_SEC)
            return
        self._intrinsics = intrinsics

    def _count_detection(self, _msg):
        self._received += 1

    def _check_unmatched(self):
        """Avisa si llegaron detecciones pero ninguna encontró profundidad dentro del margen."""
        if self._received > 0 and self._matched == 0:
            self.get_logger().warning(
                f'Llegaron {self._received} mensajes de detecciones sin profundidad a menos de '
                f'sync_slop_sec = {self._slop} s; no se procesan. ¿Se publica la profundidad?',
                throttle_duration_sec=LOG_THROTTLE_SEC)
        self._received = 0
        self._matched = 0

    def _on_pair(self, detections_msg, depth_msg):
        self._matched += 1
        if self._intrinsics is None:
            self.get_logger().warning(
                'Aún no llegan los intrínsecos de la cámara (camera_info); no se publica la '
                'detección 3D.', throttle_duration_sec=LOG_THROTTLE_SEC)
            return
        scale = depth_scale_for_encoding(depth_msg.encoding)
        if scale is None:
            self.get_logger().warning(
                f'Codificación de profundidad no soportada: "{depth_msg.encoding}". Se esperaba '
                '32FC1, 16UC1 o mono16.', throttle_duration_sec=LOG_THROTTLE_SEC)
            return
        try:
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: B902 - una conversión fallida no debe tumbar el nodo
            self.get_logger().warning(
                f'No se pudo convertir la profundidad: {exc}. Se descarta el par.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        height, width = depth.shape[:2]
        detections_3d = []
        for detection in detections_msg.detections:
            if not detection.results:
                continue
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            if not (0.0 <= cx < width and 0.0 <= cy < height):
                self.get_logger().warning(
                    'El centro de una detección cae fuera de la imagen de profundidad; se omite. '
                    '¿Profundidad no alineada al color?', throttle_duration_sec=LOG_THROTTLE_SEC)
                continue
            window = central_window(cx, cy, detection.bbox.size_x, detection.bbox.size_y,
                                    self._fraction, width, height)
            z = None if window is None else robust_depth(depth, window, scale, self._min_valid)
            if z is None:
                class_id = detection.results[0].hypothesis.class_id
                self.get_logger().warning(
                    f'Profundidad inválida en la ventana de "{class_id}" (ceros, NaN o pocos '
                    'píxeles válidos); se omite la detección.',
                    throttle_duration_sec=LOG_THROTTLE_SEC)
                continue
            xyz = deproject(cx, cy, z, self._intrinsics)
            size = metric_size(detection.bbox.size_x, detection.bbox.size_y, z, self._intrinsics)
            depth_extent = 0.0
            if self._estimate_center:
                # Supuesto de huella cuadrada: la profundidad del objeto es igual a su ancho.
                depth_extent = size[0]
                xyz = center_along_ray(xyz, depth_extent)
            detections_3d.append(build_detection3d(detection, xyz, size, depth_extent))

        self._publisher.publish(build_detection3d_array(detections_msg.header, detections_3d))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RgbdLocalizerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
