"""Instalación del paquete percepcion_nav (ament_python)."""

from glob import glob
import os

from setuptools import setup

package_name = 'percepcion_nav'


def files_under(dirname):
    """Lista (destino, archivos) para copiar recursivamente un directorio si existe."""
    entries = []
    if not os.path.isdir(dirname):
        return entries
    for root, _, files in os.walk(dirname):
        paths = [os.path.join(root, f) for f in files if f != '.gitkeep']
        if paths:
            entries.append((os.path.join('share', package_name, root), paths))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ] + files_under('models') + files_under('media'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Balbuena',
    maintainer_email='josebalbuena181096@gmail.com',
    description='Percepción para navegación: cámara, detector, RGB-D y metas de Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = percepcion_nav.camera_node:main',
            'detector_node = percepcion_nav.detector_node:main',
            'visualizer_node = percepcion_nav.visualizer_node:main',
            'rgbd_localizer_node = percepcion_nav.rgbd_localizer_node:main',
            'nav2_goal_sender_node = percepcion_nav.nav2_goal_sender_node:main',
            'synthetic_depth_node = percepcion_nav.synthetic_depth_node:main',
            'sim_panel = percepcion_nav.sim_panel:main',
        ],
    },
)
