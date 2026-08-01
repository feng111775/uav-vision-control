from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models/red_target_plate',
         glob('models/red_target_plate/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xixi',
    maintainer_email='xixi@todo.todo',
    description='H7Plus vision data bridge and test publisher.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'h7_bridge_node = uav_vision.h7_bridge_node:main',
            'vision_interface_node = uav_vision.vision_interface_node:main',
            'fake_h7_node = uav_vision.fake_h7_node:main',
            'target_filter_node = uav_vision.target_filter_node:main',
            'target_predictor_node = uav_vision.target_predictor_node:main',
            'landing_error_node = uav_vision.landing_error_node:main',
            'vision_dashboard_node = uav_vision.vision_dashboard_node:main',
            'd_task_sitl_scenario_node = '
            'uav_vision.d_task_sitl_scenario_node:main',
            'visual_servo_node = uav_vision.visual_servo_node:main',
            'camera_selector_node = '
            'uav_vision.camera_selector_node:main',
            'gazebo_red_target_detector_node = '
            'uav_vision.gazebo_red_target_detector_node:main',
        ],
    },
)
