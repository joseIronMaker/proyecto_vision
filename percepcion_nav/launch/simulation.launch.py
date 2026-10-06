"""
Simulación de extremo a extremo (US5, FR-051, FR-060–FR-066).

Gazebo Harmonic con el TurtleBot 4 mínimo de Nav2 y su pila de navegación, más la silla objetivo
y los nodos de percepción conectados a la cámara simulada solo por remapeos. No se lanzan el
nodo de cámara ni la profundidad sintética (FR-035, FR-063).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Tópicos de la cámara simulada del TurtleBot 4 (puentes de nav2_minimal_tb4_sim).
CAMERA_REMAPS = [
    ('/camera/image_raw', '/rgbd_camera/image'),
    ('/camera/depth/image_raw', '/rgbd_camera/depth_image'),
    ('/camera/camera_info', '/rgbd_camera/camera_info'),
]


def spawn_chair(context):
    """Inserta la silla: un modelo del paquete por nombre o un modelo de Fuel por URL (FR-067)."""
    chair_model = LaunchConfiguration('chair_model').perform(context)
    if chair_model.startswith('http'):
        source = chair_model
    else:
        source = os.path.join(get_package_share_directory('percepcion_nav'), 'models',
                              chair_model, 'model.sdf')
    arguments = ['-file', source, '-name', 'target_chair',
                 '-x', LaunchConfiguration('chair_x').perform(context),
                 '-y', LaunchConfiguration('chair_y').perform(context),
                 '-z', '0.0',
                 '-Y', LaunchConfiguration('chair_yaw').perform(context)]
    return [Node(package='ros_gz_sim', executable='create', name='spawn_chair',
                 arguments=arguments, output='screen')]


def generate_launch_description():
    share = FindPackageShare('percepcion_nav')
    nav2_share = FindPackageShare('nav2_bringup')
    model = LaunchConfiguration('model')
    params_file = LaunchConfiguration('params_file')
    sim_params = PathJoinSubstitution([share, 'config', 'params_sim.yaml'])

    arguments = [
        DeclareLaunchArgument('headless', default_value='True',
                              description='Sin la interfaz de Gazebo (render por software). '
                                          'True o False con mayúscula: Nav2 lo evalúa como '
                                          'expresión de Python.'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Abrir RViz de Nav2.'),
        DeclareLaunchArgument('viewer', default_value='true',
                              description='Abrir rqt_image_view sobre /detections_image.'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='Arrancar sin enviar metas; para navegar después: '
                                          'ros2 param set /nav2_goal_sender dry_run false.'),
        DeclareLaunchArgument('chair_model', default_value='freecad_chair',
                              description='Modelo del paquete o URL de Fuel (respaldo FR-067).'),
        DeclareLaunchArgument('chair_x', default_value='-5.0',
                              description='Posición x real de la silla en el mundo, en m.'),
        DeclareLaunchArgument('chair_y', default_value='0.0',
                              description='Posición y real de la silla en el mundo, en m.'),
        DeclareLaunchArgument('chair_yaw', default_value='3.1416',
                              description='Orientación de la silla en el mundo, en rad.'),
        DeclareLaunchArgument('chair_spawn_delay', default_value='10.0',
                              description='Espera antes de insertar la silla, en s.'),
        DeclareLaunchArgument(
            'model', default_value=PathJoinSubstitution([share, 'models', 'yolo11n.pt']),
            description='Ruta de los pesos de YOLO.'),
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([share, 'config', 'params.yaml']),
            description='Archivo de parámetros de los nodos.'),
    ]

    # Grupo con alcance propio: los argumentos que recibe el launch de Nav2 (entre ellos su
    # propio `params_file`) no deben sobrescribir los de este archivo.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'tb4_simulation_launch.py'])),
        launch_arguments={
            'params_file': PathJoinSubstitution([share, 'config', 'nav2_params_sim.yaml']),
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items())

    perception = [
        Node(package='percepcion_nav', executable='detector_node', name='detector',
             parameters=[params_file, sim_params, {'model_path': model}],
             remappings=CAMERA_REMAPS, output='screen', emulate_tty=True),
        Node(package='percepcion_nav', executable='visualizer_node', name='visualizer',
             parameters=[params_file, sim_params], remappings=CAMERA_REMAPS, output='screen',
             emulate_tty=True),
        Node(package='percepcion_nav', executable='rgbd_localizer_node', name='rgbd_localizer',
             parameters=[params_file, sim_params], remappings=CAMERA_REMAPS, output='screen',
             emulate_tty=True),
        Node(package='percepcion_nav', executable='nav2_goal_sender_node',
             name='nav2_goal_sender',
             parameters=[params_file, sim_params, {
                 'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}],
             output='screen', emulate_tty=True),
        Node(package='rqt_image_view', executable='rqt_image_view', name='image_view',
             arguments=['/detections_image'],
             condition=IfCondition(LaunchConfiguration('viewer'))),
    ]

    return LaunchDescription(arguments + [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([share, 'models'])),
        GroupAction([simulation], scoped=True),
        TimerAction(period=LaunchConfiguration('chair_spawn_delay'),
                    actions=[OpaqueFunction(function=spawn_chair)]),
    ] + perception)
