from setuptools import find_packages
from setuptools import setup

setup(
    name='isaac_ros_nitros_bridge_interfaces',
    version='3.1.0',
    packages=find_packages(
        include=('isaac_ros_nitros_bridge_interfaces', 'isaac_ros_nitros_bridge_interfaces.*')),
)