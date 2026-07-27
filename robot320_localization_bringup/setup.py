from glob import glob

from setuptools import find_packages, setup


package_name = "robot320_localization_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/behavior_trees", glob("behavior_trees/*")),
        (f"share/{package_name}/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hongshi Agent Contributors",
    maintainer_email="hongshi-agent@example.com",
    description="Robot320 MID-360 SLAM, localization, and Nav2 bringup",
    license="MIT",
    entry_points={
        "console_scripts": [
            "frontier_explorer = "
            "robot320_localization_bringup.frontier_explorer:main",
            "scan_restamper = "
            "robot320_localization_bringup.scan_restamper:main",
            "cmd_vel_relay = "
            "robot320_localization_bringup.cmd_vel_relay:main",
        ],
    },
)
