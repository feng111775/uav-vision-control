from glob import glob

from setuptools import find_packages, setup


package_name = 'pi_camera_vision'

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
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='xixi',
    maintainer_email='xixi@todo.todo',
    description='CSI/USB/file red-target vision with an H7-compatible topic.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pi_camera_vision_node = pi_camera_vision.vision_node:main',
            'offline_test = pi_camera_vision.offline_test:main',
        ],
    },
)
