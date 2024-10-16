from setuptools import find_packages
from setuptools import setup

setup(
    name='autonav_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('autonav_interfaces', 'autonav_interfaces.*')),
)
