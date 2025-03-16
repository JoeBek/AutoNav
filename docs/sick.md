Lidar documentation


# setup and configuration

there are some setup commands needed for the lidar to work--
I have these set up to run using an entrypoint script 

run the lidar with:
```
ros2 launch sick_scan_xd sick_multiscan.launch.py hostname:=192.168.0.1 udp_receiver_id:=192.168.0.2
```

