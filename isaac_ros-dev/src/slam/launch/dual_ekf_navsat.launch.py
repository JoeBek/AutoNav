# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    pkg_share = FindPackageShare(package='ekf_handler').find('ekf_handler')
    dual_ekf_config = os.path.join(pkg_share, 'config', 'slam.yaml')

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[dual_ekf_config, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/local_ekf")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[dual_ekf_config, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/global_ekf")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[dual_ekf_config, {'zero_altitude': True}, {"use_sim_time": False}],
                remappings=[
                    ('imu', 'sick_scansegment_xd/imu'),
                    ('gps/fix', 'gps_fix'), 
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global_ekf"),
                ],
            ),
        ]
    )