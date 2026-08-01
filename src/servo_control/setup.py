from setuptools import find_packages, setup

package_name = "servo_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", ["config/servo_dry_run.yaml", "config/servo_real.yaml"]),
        (f"share/{package_name}/launch", ["launch/servo_control.launch.py"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={"console_scripts": ["servo_node = servo_control.servo_node:main"]},
)
