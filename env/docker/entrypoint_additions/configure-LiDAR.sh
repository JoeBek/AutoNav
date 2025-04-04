#!/bin/bash

# configure networking for LiDAR
ifconfig eth0 up
ifconfig eth0 192.168.0.2 netmask 255.255.255.0
tput 5
echo "LiDAR configured... go get em tiger"
tput sgr0
