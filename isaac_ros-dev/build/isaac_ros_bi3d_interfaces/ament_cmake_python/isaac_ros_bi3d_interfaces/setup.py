from setuptools import find_packages
from setuptools import setup

setup(
    name='isaac_ros_bi3d_interfaces',
    version='3.1.0',
    packages=find_packages(
        include=('isaac_ros_bi3d_interfaces', 'isaac_ros_bi3d_interfaces.*')),
)
