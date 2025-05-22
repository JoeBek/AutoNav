#!/bin/bash

NAV_PATH="$(dirname ${BASH_SOURCE[0]})/../src/slam/config/nav2_params.yaml"

ros2 launch nav2_bringup navigation_launch.py params_file:=$NAV_PATH use_sim_time:=false
