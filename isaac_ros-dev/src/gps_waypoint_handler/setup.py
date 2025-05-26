from setuptools import find_packages, setup

package_name = 'gps_waypoint_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vtuser',
    maintainer_email='sramey02@vt.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_conversions = gps_waypoint_handler.gps_conversions:main',
            'waypoint_commander = gps_waypoint_handler.waypoint_commander:main',
            'get_gps_positioning = gps_waypoint_handler.get_gps_positioning:main',
        ],
    },
)
