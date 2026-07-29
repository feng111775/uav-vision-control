from glob import glob
import os

from setuptools import find_packages, setup

package_name = "ground_station"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xixi",
    maintainer_email="xixi@todo.todo",
    description="Offline read-only D-task ground station foundation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "health_check_node = ground_station.health_check_node:main",
        ],
    },
)
