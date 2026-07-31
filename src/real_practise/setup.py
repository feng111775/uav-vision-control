from setuptools import find_packages, setup

package_name = 'real_practise'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[
          ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
          ('share/' + package_name, ['package.xml']),
          ('share/' + package_name + '/config', ['config/first_flight_real.yaml']),
          ('share/' + package_name + '/launch', ['launch/first_flight_bench.launch.py', 'launch/first_flight_real.launch.py']),
      ], install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['first_flight_supervisor_node = real_practise.first_flight_supervisor_node:main']})
