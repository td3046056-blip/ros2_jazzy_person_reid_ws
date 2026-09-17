from glob import glob

from setuptools import find_packages, setup

package_name = "person_follow_identity"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="thadn",
    maintainer_email="thad91738@gmail.com",
    description="Strict enrolled-person identity lock for ROS2 Jazzy person following.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "identity_lock_node = person_follow_identity.node:main",
            "identity_lock_demo = person_follow_identity.demo:main",
        ],
    },
)
