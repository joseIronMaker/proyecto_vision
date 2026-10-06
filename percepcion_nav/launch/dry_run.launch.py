"""
Sistema completo en dry_run, sin robot ni Gazebo (US3, US4, FR-051).

Video -> detector -> localizador RGB-D (con profundidad sintética) -> emisor de metas, con un
árbol de marcos estático. Los nombres de marco y la posición de la cámara son argumentos de
lanzamiento y alimentan también los parámetros de los nodos (constitución, principio I).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Rotación fija camera_link -> marco óptico (convención de REP 103, no es un valor ajustable).
OPTICAL_ROTATION = ['--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708']


def static_tf(name, parent, child, translation=None, rotation=None):
    """Crea un static_transform_publisher con argumentos con nombre."""
    arguments = []
    if translation is not None:
        arguments += ['--x', translation[0], '--y', translation[1], '--z', translation[2]]
    if rotation is not None:
        arguments += rotation
    arguments += ['--frame-id', parent, '--child-frame-id', child]
    return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                arguments=arguments, output='log')


def generate_launch_description():
    share = FindPackageShare('percepcion_nav')
    video = LaunchConfiguration('video')
    model = LaunchConfiguration('model')
    params_file = LaunchConfiguration('params_file')
    viewer = LaunchConfiguration('viewer')
    use_rviz = LaunchConfiguration('use_rviz')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    camera_link_frame = LaunchConfiguration('camera_link_frame')
    optical_frame = LaunchConfiguration('optical_frame')
    cam_xyz = (LaunchConfiguration('cam_x'), LaunchConfiguration('cam_y'),
               LaunchConfiguration('cam_z'))

    arguments = [
        DeclareLaunchArgument(
            'video', default_value=PathJoinSubstitution([share, 'media', 'sample.mp4']),
            description='Ruta del video, o índice de dispositivo ("0").'),
        DeclareLaunchArgument(
            'model', default_value=PathJoinSubstitution([share, 'models', 'yolo11n.pt']),
            description='Ruta de los pesos de YOLO.'),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([share, 'config', 'params.yaml']),
            description='Archivo de parámetros de los nodos.'),
        DeclareLaunchArgument('viewer', default_value='true',
                              description='Abrir rqt_image_view sobre /detections_image.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Abrir RViz con la vista previa de la meta.'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='Marco raíz del árbol estático.'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                              description='Marco de odometría estático.'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='Marco del robot.'),
        DeclareLaunchArgument('camera_link_frame', default_value='camera_link',
                              description='Marco del cuerpo de la cámara.'),
        DeclareLaunchArgument('optical_frame', default_value='camera_color_optical_frame',
                              description='Marco óptico de la cámara (REP 103).'),
        DeclareLaunchArgument('cam_x', default_value='0.10',
                              description='Posición x de camera_link en base_frame, en m.'),
        DeclareLaunchArgument('cam_y', default_value='0.0',
                              description='Posición y de camera_link en base_frame, en m.'),
        DeclareLaunchArgument('cam_z', default_value='0.30',
                              description='Posición z de camera_link en base_frame, en m.'),
    ]

    transforms = [
        static_tf('tf_map_odom', map_frame, odom_frame),
        static_tf('tf_odom_base', odom_frame, base_frame),
        static_tf('tf_base_camera', base_frame, camera_link_frame, translation=cam_xyz),
        static_tf('tf_camera_optical', camera_link_frame, optical_frame,
                  rotation=OPTICAL_ROTATION),
    ]

    nodes = [
        Node(package='percepcion_nav', executable='camera_node', name='camera',
             parameters=[params_file, {'source': video, 'frame_id': optical_frame}],
             output='screen', emulate_tty=True),
        Node(package='percepcion_nav', executable='detector_node', name='detector',
             parameters=[params_file, {'model_path': model}], output='screen',
             emulate_tty=True),
        Node(package='percepcion_nav', executable='visualizer_node', name='visualizer',
             parameters=[params_file], output='screen', emulate_tty=True),
        Node(package='percepcion_nav', executable='synthetic_depth_node',
             name='synthetic_depth', parameters=[params_file], output='screen',
             emulate_tty=True),
        Node(package='percepcion_nav', executable='rgbd_localizer_node', name='rgbd_localizer',
             parameters=[params_file], output='screen', emulate_tty=True),
        Node(package='percepcion_nav', executable='nav2_goal_sender_node',
             name='nav2_goal_sender',
             parameters=[params_file, {
                 'dry_run': ParameterValue(True, value_type=bool),
                 'map_frame': map_frame,
                 'robot_base_frame': base_frame,
             }],
             output='screen', emulate_tty=True),
        Node(package='rqt_image_view', executable='rqt_image_view', name='image_view',
             arguments=['/detections_image'], condition=IfCondition(viewer)),
        Node(package='rviz2', executable='rviz2', name='rviz',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'percepcion_nav.rviz'])],
             condition=IfCondition(use_rviz)),
    ]

    return LaunchDescription(arguments + transforms + nodes)
