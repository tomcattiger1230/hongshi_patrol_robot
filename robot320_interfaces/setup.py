from glob import glob

from setuptools import find_packages, setup


package_name = "robot320_interfaces"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/dds", glob("robot320_interfaces/dds/*.idl")),
        (f"share/{package_name}/scripts", glob("scripts/*.sh")),
    ],
    package_data={"robot320_interfaces.dds": ["*.idl"]},
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Hongshi Agent Contributors",
    maintainer_email="hongshi-agent@example.com",
    description="Robot320 JSON messages and ROS 2 compatible Fast DDS wire contract",
    license="MIT",
    extras_require={"test": ["pytest"]},
)
