import os
from glob import glob

from setuptools import find_packages, setup

package_name = "person_follow_nav"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="thaihoa",
    maintainer_email="thad91738@gmail.com",
    description="Person following with LiDAR obstacle avoidance for BW-DR03.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_tracker = person_follow_nav.target_tracker_node:main",
            "follow_planner = person_follow_nav.follow_planner_node:main",
            "calibrate_lidar = person_follow_nav.calibrate_lidar_node:main",
            "calibrate_center = person_follow_nav.calibrate_center_node:main",
        ],
    },
)
