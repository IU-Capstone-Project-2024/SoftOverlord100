from setuptools import find_packages, setup
import os
from glob import glob


package_name = "urdf_dummy"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*")),
        ),
        (
            os.path.join("share", package_name, "description"),
            glob(os.path.join("description", "*urdf")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/simulator.rviz"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ephy",
    maintainer_email="d.kuznetsov@innopolis.university",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest", "unittest"],
    entry_points={
        "console_scripts": ["converter = urdf_dummy.converter:main"],
    },
)