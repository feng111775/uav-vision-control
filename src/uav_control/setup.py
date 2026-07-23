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
    ],
    install_requires=['setuptools'],
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
        ],
    },
)
