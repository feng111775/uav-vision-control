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
        ('share/' + package_name + '/scripts', glob('scripts/*')),
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
            'sitl_result_recorder = '
            'uav_control.sitl_result_recorder:main',
            'sitl_preflight_gate = '
            'uav_control.sitl_preflight_gate:main',
            'sitl_mode_recovery = '
            'uav_control.sitl_mode_recovery:main',
            'd_task_mock_node = '
            'uav_control.d_task_mock_node:main',
            'readiness_gate = '
            'uav_control.readiness_gate_node:main',
            'stage_observer = '
            'uav_control.stage_observer:main',
            'odom_freeze_relay = '
            'uav_control.odom_freeze_relay:main',
        ],
    },
)
