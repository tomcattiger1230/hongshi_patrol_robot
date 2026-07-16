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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hongshi Agent Contributors",
    maintainer_email="hongshi-agent@example.com",
    description="NUC bringup for Robot320 MID-360s Cartographer localization",
    license="MIT",
)
