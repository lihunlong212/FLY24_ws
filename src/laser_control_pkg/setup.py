from setuptools import find_packages, setup

package_name = "laser_control_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/laser_control.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="orangepi",
    maintainer_email="orangepi@todo.todo",
    description="GPIO laser control for the plant-protection mission.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "laser_control_node = laser_control_pkg.laser_control_node:main",
        ],
    },
)
