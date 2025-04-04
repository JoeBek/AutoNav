#!/bin/bash

# configure networking for LiDAR
ifconfig set eth0 up
ip addr add 192.168.0.2/24 dev eth0
tput setaf 5
echo "LiDAR configured... go get em tiger"
tput sgr0
