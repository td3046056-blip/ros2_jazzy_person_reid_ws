from setuptools import find_packages, setup

package_name = "robot_web"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    # Trang web nam trong goi python (robot_web/static) de ca ban build thuong lan --symlink-install deu tim thay
    package_data={package_name: ["static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="thaihoa",
    maintainer_email="thad91738@gmail.com",
    description="Trang dieu khien robot bam nguoi qua trinh duyet.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_server = robot_web.web_node:main",
        ],
    },
)
