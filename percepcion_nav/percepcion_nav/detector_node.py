"""Nodo detector: ejecuta YOLO sobre cada cuadro y publica vision_msgs/Detection2DArray."""

import os
import time

from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Constantes internas no ajustables (contracts/parameters.md).
LOG_THROTTLE_SEC = 5.0
FPS_REPORT_PERIOD_SEC = 10.0


def parse_class_filter(text):
    """Convierte 'chair, person' en {'chair', 'person'}; vacío significa todas las clases."""
    return {name.strip() for name in str(text).split(',') if name.strip()}


def build_detection2d_array(header, boxes):
    """
    Construye un Detection2DArray a partir de cajas (cx, cy, w, h, clase, score) en píxeles.

    El header del arreglo y de cada detección es el de la imagen de origen (FR-013). Las cajas
    de tamaño no positivo se omiten (SC-004). Sin cajas, el arreglo sale vacío (FR-015).
    """
    msg = Detection2DArray()
    msg.header = header
    for cx, cy, width, height, class_name, score in boxes:
        if width <= 0 or height <= 0:
            continue
        detection = Detection2D()
        detection.header = header
        detection.bbox.center.position.x = float(cx)
        detection.bbox.center.position.y = float(cy)
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(class_name)
        hypothesis.hypothesis.score = float(score)
        detection.results.append(hypothesis)
        msg.detections.append(detection)
    return msg


class DetectorNode(Node):
    """Detecta objetos COCO con un modelo YOLO preentrenado."""

    def __init__(self):
        super().__init__('detector')
        self.declare_parameter('model_path', '', ParameterDescriptor(
            description='Ruta de los pesos de YOLO; si no existe, el nodo falla al arrancar.'))
        self.declare_parameter('conf_threshold', 0.40, ParameterDescriptor(
            description='Score mínimo para publicar una detección.'))
        self.declare_parameter('device', 'cpu', ParameterDescriptor(
            description='Dispositivo de inferencia: cpu o cuda:0.'))
        self.declare_parameter('imgsz', 640, ParameterDescriptor(
            description='Tamaño de entrada del modelo en píxeles.'))
        self.declare_parameter('class_filter', '', ParameterDescriptor(
            description='Clases COCO separadas por comas; vacío = todas.'))

        model_path = os.path.expanduser(str(self.get_parameter('model_path').value))
        self._conf = float(self.get_parameter('conf_threshold').value)
        self._device = str(self.get_parameter('device').value)
        self._imgsz = int(self.get_parameter('imgsz').value)
        self._classes = parse_class_filter(self.get_parameter('class_filter').value)

        # Falla rápida (FR-016): se verifica el archivo antes de importar ultralytics para que no
        # descargue pesos en silencio al directorio de trabajo.
        if not model_path or not os.path.isfile(model_path):
            self.get_logger().fatal(
                f'No existe el archivo de pesos "{model_path}". Descárgalo (ver README) o indica '
                'otro con el argumento de launch "model" o el parámetro "model_path".')
            raise SystemExit(1)
        try:
            from ultralytics import YOLO
            self._model = YOLO(model_path)
        except Exception as exc:  # noqa: B902 - cualquier falla de carga es fatal al arrancar
            self.get_logger().fatal(f'No se pudo cargar el modelo "{model_path}": {exc}')
            raise SystemExit(1)
        self._names = self._model.names

        self._bridge = CvBridge()
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        detections_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                    reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.VOLATILE)
        self._publisher = self.create_publisher(Detection2DArray, 'detections', detections_qos)
        self.create_subscription(Image, 'camera/image_raw', self._on_image, image_qos)

        self._frames = 0
        self._report_start = time.monotonic()
        classes = ', '.join(sorted(self._classes)) if self._classes else 'todas'
        self.get_logger().info(
            f'Modelo "{os.path.basename(model_path)}" cargado en {self._device}; '
            f'umbral {self._conf:.2f}; clases: {classes}.')

    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: B902 - conversión fallida: se descarta el cuadro
            self.get_logger().warning(
                f'No se pudo convertir la imagen ({msg.encoding}): {exc}. Se descarta.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        try:
            result = self._model.predict(frame, conf=self._conf, imgsz=self._imgsz,
                                         device=self._device, verbose=False)[0]
        except Exception as exc:  # noqa: B902 - una inferencia fallida no debe tumbar el nodo
            self.get_logger().warning(
                f'Falló la inferencia: {exc}. Se descarta el cuadro.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        boxes = []
        if result.boxes is not None and len(result.boxes) > 0:
            xywh = result.boxes.xywh.cpu().numpy()
            scores = result.boxes.conf.cpu().numpy()
            class_ids = result.boxes.cls.cpu().numpy().astype(int)
            for (cx, cy, width, height), score, class_id in zip(xywh, scores, class_ids):
                class_name = self._names[int(class_id)]
                if self._classes and class_name not in self._classes:
                    continue
                boxes.append((cx, cy, width, height, class_name, score))

        self._publisher.publish(build_detection2d_array(msg.header, boxes))
        self._report_rate()

    def _report_rate(self):
        """Registra periódicamente los cuadros por segundo procesados (evidencia de SC-003)."""
        self._frames += 1
        elapsed = time.monotonic() - self._report_start
        if elapsed >= FPS_REPORT_PERIOD_SEC:
            self.get_logger().info(f'Inferencia: {self._frames / elapsed:.1f} cuadros/s.')
            self._frames = 0
            self._report_start = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
