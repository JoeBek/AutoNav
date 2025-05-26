#!/bin/bash
export RCUTILS_LOGGING_LEVEL_SEVERITY='ERROR'
ros2 launch zed_wrapper zed_camera.launch.py camera_model:='zed2i' 
