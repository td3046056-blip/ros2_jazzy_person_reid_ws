from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "person_reid_tracker"


def recursive_files(base_dir):
    base = Path(base_dir)
    return [str(p) for p in base.rglob("*") if p.is_file()]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[
        "person_reid_tracker",
        "person_reid_tracker.*",
        "tracking_module",
        "tracking_module.*",
        "models",
        "models.*",
        "utils",
        "utils.*",
    ]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/models", glob("model_assets/*")),
        (f"share/{package_name}/data", recursive_files("data")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="student",
    maintainer_email="student@example.com",
    description="ROS2 Jazzy camera person target locking with YOLOv5, DeepSORT, and external ReID recovery.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_reid_node = person_reid_tracker.camera_reid_node:main",
            "camera_reid_demo = person_reid_tracker.camera_reid_demo:main",
        ],
    },
)
