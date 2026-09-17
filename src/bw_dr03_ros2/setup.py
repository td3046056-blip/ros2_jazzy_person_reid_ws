from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bw_dr03_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thaihoa',
    maintainer_email='thaihoa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'raw_serial_node = bw_dr03_ros2.raw_serial_node:main',
            'decoded_serial_node = bw_dr03_ros2.decoded_serial_node:main',
        ],
    },
)
