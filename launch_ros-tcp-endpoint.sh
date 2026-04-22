#!/bin/bash

source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.0.13 -p ROS_TCP_PORT:=10000  