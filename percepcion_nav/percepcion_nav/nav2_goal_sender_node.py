"""Emisor de metas: detección 3D de la clase objetivo -> NavigateToPose en el mapa (FR-040)."""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from percepcion_nav.goal_planning import (compute_goal, plausible_jump, select_best,
                                          target_moved, yaw_to_quaternion)
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_geometry_msgs  # noqa: F401 - registra las transformaciones de PointStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from vision_msgs.msg import Detection3DArray

# Periodo mínimo entre registros repetidos (contracts/parameters.md, constantes internas).
LOG_THROTTLE_SEC = 5.0

STATE_IDLE = 'IDLE'
STATE_SENDING = 'SENDING'
STATE_ACTIVE = 'ACTIVE'

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class Nav2GoalSenderNode(Node):
    """Calcula una meta frente al objeto y la envía a Nav2, o solo la publica en dry_run."""

    def __init__(self):
        super().__init__('nav2_goal_sender')
        self.declare_parameter('target_class', 'chair', ParameterDescriptor(
            description='Clase objetivo; se usa la detección de mayor score.'))
        self.declare_parameter('map_frame', 'map', ParameterDescriptor(
            description='Marco de la meta.'))
        self.declare_parameter('robot_base_frame', 'base_link', ParameterDescriptor(
            description='Marco del robot para la línea robot-objeto.'))
        self.declare_parameter('standoff_distance', 1.0, ParameterDescriptor(
            description='Distancia de separación al objeto, en m.'))
        self.declare_parameter('arrival_tolerance', 0.30, ParameterDescriptor(
            description='No se envía meta si dist <= separación + este margen, en m.'))
        self.declare_parameter('goal_update_threshold', 0.50, ParameterDescriptor(
            description='Movimiento mínimo del objetivo para reemplazar una meta activa, en m.'))
        self.declare_parameter('max_target_jump', 1.5, ParameterDescriptor(
            description='Salto máximo creíble del objetivo respecto al último usado, en m.'))
        self.declare_parameter('tf_timeout_sec', 0.2, ParameterDescriptor(
            description='Tiempo límite de las consultas tf2, en s.'))
        self.declare_parameter('server_timeout_sec', 2.0, ParameterDescriptor(
            description='Espera máxima del servidor de acción, en s.'))
        self.declare_parameter('dry_run', True, ParameterDescriptor(
            description='Calcula y publica la vista previa sin enviar la meta.'))
        self.declare_parameter('action_name', 'navigate_to_pose', ParameterDescriptor(
            description='Nombre del servidor de acción NavigateToPose.'))

        self._target_class = str(self.get_parameter('target_class').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('robot_base_frame').value)
        self._standoff = float(self.get_parameter('standoff_distance').value)
        self._tolerance = float(self.get_parameter('arrival_tolerance').value)
        self._update_threshold = float(self.get_parameter('goal_update_threshold').value)
        self._max_jump = float(self.get_parameter('max_target_jump').value)
        self._tf_timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        self._server_timeout = float(self.get_parameter('server_timeout_sec').value)
        self._action_name = str(self.get_parameter('action_name').value)

        # El listener usa un grupo de callbacks reentrante y main() usa un ejecutor multihilo:
        # así una consulta con tiempo límite dentro de un callback recibe /tf mientras espera
        # (research R9).
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        preview_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        detections_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                    reliability=ReliabilityPolicy.RELIABLE)
        self._preview_pub = self.create_publisher(PoseStamped, 'goal_pose_preview', preview_qos)
        self.create_subscription(Detection3DArray, 'detections_3d', self._on_detections,
                                 detections_qos)
        self._client = ActionClient(self, NavigateToPose, self._action_name)

        self._state = STATE_IDLE
        # Instante del último resultado de navegación: las detecciones anteriores describen una
        # posición del robot que ya no existe y no deben provocar metas nuevas (FR-042).
        self._last_result_time = None
        self._goal_seq = 0
        self._active_seq = None
        self._last_target = None
        self.get_logger().info(
            f'Clase objetivo "{self._target_class}", separación {self._standoff:.2f} m, '
            f'tolerancia de llegada {self._tolerance:.2f} m, marcos {self._map_frame} / '
            f'{self._base_frame}.')

    def _on_detections(self, msg):
        candidates = [(d.results[0].hypothesis.class_id, d.results[0].hypothesis.score, d)
                      for d in msg.detections if d.results]
        best = select_best(candidates, self._target_class)
        if best is None:
            return

        point = PointStamped()
        point.header = best.header
        point.point = best.results[0].pose.pose.position
        try:
            target = self._tf_buffer.transform(point, self._map_frame, timeout=self._tf_timeout)
            robot = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time.from_msg(best.header.stamp),
                timeout=self._tf_timeout)
        except TransformException as exc:
            self.get_logger().warning(
                f'Transformación no disponible hacia "{self._map_frame}": {exc} No se calcula '
                'la meta.', throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        object_xy = (target.point.x, target.point.y)
        robot_xy = (robot.transform.translation.x, robot.transform.translation.y)
        if not plausible_jump(self._last_target, object_xy, self._max_jump):
            self.get_logger().warning(
                f'Objetivo estimado en ({object_xy[0]:.2f}, {object_xy[1]:.2f}), a más de '
                f'{self._max_jump:.1f} m del último ({self._last_target[0]:.2f}, '
                f'{self._last_target[1]:.2f}); el objeto es estático, se descarta como error de '
                'estimación.', throttle_duration_sec=LOG_THROTTLE_SEC)
            return
        self.get_logger().info(
            f'Objetivo "{self._target_class}" en {self._map_frame}: ({object_xy[0]:.2f}, '
            f'{object_xy[1]:.2f}); robot en ({robot_xy[0]:.2f}, {robot_xy[1]:.2f}).',
            throttle_duration_sec=LOG_THROTTLE_SEC)

        goal = compute_goal(robot_xy, object_xy, self._standoff, self._tolerance)
        if goal is None:
            self.get_logger().info(
                'El robot ya está a la distancia de separación más la tolerancia de llegada, o '
                'más cerca: no se envía meta.', throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        pose = PoseStamped()
        pose.header.stamp = best.header.stamp
        pose.header.frame_id = self._map_frame
        pose.pose.position.x, pose.pose.position.y = goal[0], goal[1]
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(goal[2])
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self._preview_pub.publish(pose)

        # Se lee en cada detección para poder cambiarlo con `ros2 param set` (FR-045).
        if self.get_parameter('dry_run').value:
            return
        self._send_goal(pose, object_xy, goal[2])

    def _send_goal(self, pose, object_xy, yaw):
        stamp = Time.from_msg(pose.header.stamp)
        if self._last_result_time is not None and stamp <= self._last_result_time:
            return  # observación anterior al último resultado: robot ya en otra pose
        replacing = self._state != STATE_IDLE
        if replacing and not target_moved(self._last_target, object_xy, self._update_threshold):
            return  # FR-044: ya hay una meta en curso hacia el mismo objetivo
        if not self._client.server_is_ready() and not self._client.wait_for_server(
                timeout_sec=self._server_timeout):
            self.get_logger().warning(
                f'El servidor de acción "{self._action_name}" no respondió en '
                f'{self._server_timeout:.1f} s; no se envía la meta.',
                throttle_duration_sec=LOG_THROTTLE_SEC)
            return

        self._goal_seq += 1
        seq = self._goal_seq
        self._active_seq = seq
        self._last_target = object_xy
        self._state = STATE_SENDING
        note = ' (reemplaza la anterior: el objetivo se movió)' if replacing else ''
        self.get_logger().info(
            f'Enviando meta #{seq} a ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}), '
            f'orientación {math.degrees(yaw):.0f}°{note}.')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self._client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda f, s=seq: self._on_goal_response(f, s))

    def _on_goal_response(self, future, seq):
        if seq != self._active_seq:
            return  # respuesta de una meta ya reemplazada
        try:
            handle = future.result()
        except Exception as exc:  # noqa: B902 - el envío fallido no debe tumbar el nodo
            self.get_logger().warning(f'Falló el envío de la meta #{seq}: {exc}.')
            self._state = STATE_IDLE
            return
        if not handle.accepted:
            self.get_logger().warning(f'Nav2 rechazó la meta #{seq} (¿navegación aún no activa?).',
                                      throttle_duration_sec=LOG_THROTTLE_SEC)
            self._state = STATE_IDLE
            return
        self._state = STATE_ACTIVE
        self.get_logger().info(f'Nav2 aceptó la meta #{seq}.')
        handle.get_result_async().add_done_callback(lambda f, s=seq: self._on_result(f, s))

    def _on_result(self, future, seq):
        if seq != self._active_seq:
            return  # resultado de una meta ya reemplazada
        status = future.result().status
        name = STATUS_NAMES.get(status, str(status))
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Meta #{seq} terminada: {name}.')
        else:
            self.get_logger().warning(f'Meta #{seq} terminada: {name}.')
        self._last_result_time = self.get_clock().now()
        self._state = STATE_IDLE

    def _on_feedback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Navegando: faltan {remaining:.2f} m.',
                               throttle_duration_sec=LOG_THROTTLE_SEC)

    def destroy_node(self):
        """Libera el cliente de acción antes de destruir el nodo."""
        self._client.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Nav2GoalSenderNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
