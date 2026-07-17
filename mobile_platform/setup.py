"""ROS 2 ament_python setup for the Robot320 mobile platform."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mobile_platform"


def vendor_data_files():
    entries = []
    for directory, _, filenames in os.walk("vendor"):
        files = [os.path.join(directory, name) for name in filenames]
        if files:
            relative = os.path.relpath(directory, "vendor")
            destination = os.path.join("share", package_name, "vendor", relative)
            entries.append((destination, files))
    return entries


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
        *vendor_data_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hongshi Agent Contributors",
    maintainer_email="hongshi-agent@example.com",
    description="Robot320 mobile platform onboard side: CAN + ROS 2 + FastDDS bridge",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "robot320_onboard = mobile_platform.onboard_node:main",
            "robot320_ros2_bridge = mobile_platform.ros2_node:main",
            "robot320_fastdds_bridge = mobile_platform.fastdds_node:main",
            "robot320_fastdds_gateway = mobile_platform.fastdds_ros_gateway:main",
            "robot320_cli = mobile_platform.cli:main",
        ],
    },
)
