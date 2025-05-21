#!/bin/bash

ros2 launch sick_scan_xd sick_multiscan.launch.py \
       	hostname:=192.168.0.1 \
       	udp_receiver_ip:=192.168.0.2 \
	publish_frame_id:="lidar_footprint" \
	tf_publish_rate:=0 \

