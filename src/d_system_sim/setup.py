from glob import glob
import os

from setuptools import find_packages, setup

package_name = "d_system_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="xixi",
    maintainer_email="xixi@todo.todo",
    description="D-task joint simulation orchestration foundation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "system_check_node = d_system_sim.system_check_node:main",
        ],
    },
)
