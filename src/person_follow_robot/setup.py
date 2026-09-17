from glob import glob
import os
from setuptools import find_packages, setup

package_name = "person_follow_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="thadn",
    maintainer_email="thad91738@gmail.com",
    description="Bringup and safe distance controller for an enrolled-person-following BW-DR03 robot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follow_controller = person_follow_robot.follow_controller:main",
            "follow_calibrator = person_follow_robot.calibrate_distance:main",
        ],
    },
)
