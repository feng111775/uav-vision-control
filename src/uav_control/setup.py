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
    ],
    install_requires=['setuptools', 'matplotlib'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xixi',
    maintainer_email='xixi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vehicle_status_listener = '
            'uav_control.vehicle_status_listener:main',
            'offboard_control = uav_control.offboard_control:main',
            'vision_offboard_controller = '
            'uav_control.vision_offboard_controller:main',
            'mission_manager_node = '
            'uav_control.mission_manager_node:main',
            'fake_position_node = '
            'uav_control.fake_position_node:main',
            'trajectory_bridge_node = '
            'uav_control.trajectory_bridge_node:main',
            'mission_offboard_controller = '
            'uav_control.mission_offboard_controller:main',
            'px4_position_bridge_node = '
            'uav_control.px4_position_bridge_node:main',
            'mission_visualizer = '
            'uav_control.mission_visualizer:main',
            'animal_detector_sim_node = '
            'uav_control.animal_detector_sim_node:main',
            'animal_statistics_node = '
            'uav_control.animal_statistics_node:main',
            'flight_trajectory_logger = '
            'uav_control.flight_trajectory_logger:main',
        ],
    },
)
