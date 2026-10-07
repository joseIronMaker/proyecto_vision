"""Nodo de visualización: dibuja las detecciones sobre el cuadro del que provienen."""

import zlib

import cv2
from cv_bridge import CvBridge
import message_filters
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

# Periodo mínimo entre advertencias repetidas (contracts/parameters.md, constantes internas).
LOG_THROTTLE_SEC = 5.0
WINDOW_NAME = 'percepcion_nav: detecciones'


def class_color(class_name):
    """Devuelve un color BGR estable para cada clase."""
    value = zlib.crc32(class_name.encode('utf-8'))
    return (64 + value % 192, 64 + (value >> 8) % 192, 64 + (value >> 16) % 192)


class VisualizerNode(Node):
    """Empareja imagen y detecciones por estampa exacta y publica la imagen anotada."""

    def __init__(self):
        super().__init__('visualizer')
        self.declare_parameter('show_window', False, ParameterDescriptor(
            description='Abrir una ventana local de OpenCV además del tópico.'))
        self.declare_parameter('sync_queue_size', 30, ParameterDescriptor(
            description='Cola del TimeSynchronizer exacto imagen-detecciones.'))
        self._show_window = bool(self.get_parameter('show_window').value)
        queue_size = int(self.get_parameter('sync_queue_size').value)

        self._bridge = CvBridge()
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=queue_size,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        detections_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=queue_size,
                                    reliability=ReliabilityPolicy.RELIABLE)
        output_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)

        self._publisher = self.create_publisher(Image, 'detections_image', output_qos)
        image_sub = message_filters.Subscriber(self, Image, 'camera/image_raw',
                                               qos_profile=image_qos)
        detections_sub = message_filters.Subscriber(self, Detection2DArray, 'detections',
                                                    qos_profile=detections_qos)
        # Estampa exacta: el detector copia el header de la imagen (FR-013, FR-020).
        self._sync = message_filters.TimeSynchronizer([image_sub, detections_sub], queue_size)
        self._sync.registerCallback(self._on_pair)

    def _on_pair(self, image_msg, detections_msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: B902 - conversión fallida: se descarta el par
            self.get_logger().warning(
                f'No se pudo convertir la imagen ({image_msg.encoding}): {exc}. Se descarta.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        for detection in detections_msg.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            half_w = detection.bbox.size_x / 2.0
            half_h = detection.bbox.size_y / 2.0
            top_left = (int(round(cx - half_w)), int(round(cy - half_h)))
            bottom_right = (int(round(cx + half_w)), int(round(cy + half_h)))
            color = class_color(hypothesis.class_id)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)
            label = f'{hypothesis.class_id} {hypothesis.score:.2f}'
            text_origin = (top_left[0], max(top_left[1] - 6, 12))
            cv2.putText(frame, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
                        cv2.LINE_AA)

        annotated = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated.header = image_msg.header
        self._publisher.publish(annotated)

        if self._show_window:
            cv2.imshow(WINDOW_NAME, frame)
            cv2.waitKey(1)

    def destroy_node(self):
        """Cierra la ventana local, si se abrió, antes de destruir el nodo."""
        if self._show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualizerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
