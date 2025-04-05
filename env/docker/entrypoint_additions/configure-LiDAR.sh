#!/bin/bash

# configure networking for LiDAR
ip link set eno1 up
ip addr add 192.168.0.2/24 dev eno1 
tput setaf 5
echo "LiDAR configured... go get em tiger"
tput sgr0
