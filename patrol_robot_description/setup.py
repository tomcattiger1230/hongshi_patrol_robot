from glob import glob

from setuptools import find_packages, setup


package_name = "patrol_robot_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (
            f"share/{package_name}/isaac_sim",
            glob("isaac_sim/*.py") + glob("isaac_sim/*.npz"),
        ),
        (f"share/{package_name}/rviz", glob("rviz/*")),
        (f"share/{package_name}/scripts", glob("scripts/*")),
        (f"share/{package_name}/urdf", glob("urdf/*")),
        (f"share/{package_name}/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hongshi Agent Contributors",
    maintainer_email="hongshi-agent@example.com",
    description="Primitive roboQ-320 Ackermann model for Gazebo and Isaac Sim",
    license="MIT",
    entry_points={
        "console_scripts": [
            "patrol_demo_controller = "
            "patrol_robot_description.demo_controller:main",
        ],
    },
)
