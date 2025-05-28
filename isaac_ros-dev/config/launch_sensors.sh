#!/bin/bash

CONFIG_DIR=$(dirname "${BASH_SOURCE[0]}")

trap 'echo "Cleaning up..."; pkill -P $$; exit' EXIT SIGINT SIGTERM

"./$CONFIG_DIR/run-lidar.sh" 2>&1 1>/dev/null & 
"./$CONFIG_DIR/run-zed.sh" 2>&1 1>/dev/null &  
ros2 run gps_handler gps_publisher 1>/dev/null &

echo "camera, lidar, gps started... maybe..."

wait
