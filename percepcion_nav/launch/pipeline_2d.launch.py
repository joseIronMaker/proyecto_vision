"""Pipeline 2D: cámara -> detector -> visualización (US1, FR-051)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('percepcion_nav')
    video = LaunchConfiguration('video')
    model = LaunchConfiguration('model')
    params_file = LaunchConfiguration('params_file')
    viewer = LaunchConfiguration('viewer')

    return LaunchDescription([
        DeclareLaunchArgument(
            'video', default_value=PathJoinSubstitution([share, 'media', 'sample.mp4']),
            description='Ruta del video, o índice de dispositivo ("0").'),
        DeclareLaunchArgument(
            'model', default_value=PathJoinSubstitution([share, 'models', 'yolo11n.pt']),
            description='Ruta de los pesos de YOLO.'),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([share, 'config', 'params.yaml']),
            description='Archivo de parámetros de los nodos.'),
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='Abrir rqt_image_view sobre /detections_image (SC-002).'),

        Node(package='percepcion_nav', executable='camera_node', name='camera',
             parameters=[params_file, {'source': video}], output='screen', emulate_tty=True),
        Node(package='percepcion_nav', executable='detector_node', name='detector',
             parameters=[params_file, {'model_path': model}], output='screen',
             emulate_tty=True),
        Node(package='percepcion_nav', executable='visualizer_node', name='visualizer',
             parameters=[params_file], output='screen', emulate_tty=True),
        Node(package='rqt_image_view', executable='rqt_image_view', name='image_view',
             arguments=['/detections_image'], condition=IfCondition(viewer)),
    ])
