from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xixi',
    maintainer_email='xixi@todo.todo',
    description='PX4 v1.16 D-task mission controller and dashboard.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vehicle_status_listener = '
            'uav_control.vehicle_status_listener:main',
            'mission_controller_node = '
            'uav_control.mission_controller_node:main',
            'mission_dashboard_node = '
            'uav_control.mission_dashboard_node:main',
            'car_start_gateway = uav_control.car_start_gateway:main',
            'car_udp_trigger_node = uav_control.car_udp_trigger_node:main',
            'car_link_bridge_node = '
            'uav_control.hardware.car_link_bridge_node:main',
            'car_link_simulator_node = '
            'uav_control.hardware.car_link_simulator_node:main',
            'payload_bridge_node = '
            'uav_control.hardware.payload_bridge_node:main',
            'payload_mock_node = '
            'uav_control.hardware.payload_mock_node:main',
            'system_health_node = '
            'uav_control.integration.system_health_node:main',
            'safety_gate_node = '
            'uav_control.integration.safety_gate_node:main',
            'mock_vision_v2_node = '
            'uav_control.mock_vision_v2_node:main',
        ],
    },
)
