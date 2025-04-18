# ZED Autonav User manual


this document is intended as a knowledge base for anything ZED related for AutoNav 24/25.

---


# Fast Use Guide

To launch the zed node, you can run `zedup` from the ros2_ws directory. This command is an alias for the zed node launch file,
`ros2 launch zed_wrapper zed_camera.launch.py camera_model:='zed2i'`. Remember to source the ROS setup script in each terminal you open.

This node publishes rolling camera data to a variety of topics. The list can be found at:

[https://www.stereolabs.com/docs/ros2/zed-node]

this link also contains other very useful information on the zed node.


---


# features and where to find them

feature exploration with the zed is ongoing. 

Useful features will be documented here, along with links to further documentation

## ROS parameters

The zed camera has a suite of configuration parameters available for the ROS2 wrapper.

The link can be found [here](https://www.stereolabs.com/docs/ros2/020_zed-node)

```
gpu_id - gpu id for computation

camera_flip [bool] - if mounted upside down

pub_resolution ['NATIVE' | 'CUSTOM' | 'OPTIMIZED'] - custom for saving bandwidth. set general.pub_downscale_factor to reduce bandwidth.

pub_frame_rate [int] - set publishing frequency. 

```

Other Parameter topics:
- streaming
- camera image 
- ROI 
- depth 
- odometry and localization
- global localization 
- mapping
- object detection



---

# setup

On the team laptop, I am running cuda 12.6, the '560' nvidia gpu kernel driver (run `nvidia-smi` to see yours)

The ZED SDK at latest version

[https://www.stereolabs.com/docs/installation/linux]

ros2 humble

[https://docs.ros.org/en/humble/index.html]

zed ros2 wrapper

[https://github.com/stereolabs/zed-ros2-wrapper]


these should be all you need to run the zed node.



