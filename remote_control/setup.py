"""ROS 2 ament_python setup for the Robot320 remote controller."""

from glob import glob

from setuptools import find_packages, setup

package_name = "remote_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hongshi Agent Contributors",
    maintainer_email="hongshi-agent@example.com",
    description="Robot320 remote side: ROS 2 / FastDDS / UDP-JSON control clients",
    license="MIT",
    extras_require={"gui": ["PySide6>=6.5"], "test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "robot320_remote_cli = remote_control.cli:main",
            "robot320_remote_ros2 = remote_control.ros2_client:main",
            "robot320_remote_fastdds = remote_control.fastdds_client:main",
            "robot320_remote_gui = remote_control.gui:main",
        ],
    },
)
