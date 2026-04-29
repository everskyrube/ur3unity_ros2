#!/bin/bash

source install/setup.bash

ROS_IP=0.0.0.0
echo "Binding ROS TCP endpoint to $ROS_IP:10000 (all interfaces)"

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:="$ROS_IP" -p ROS_TCP_PORT:=10000

# ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.0.13 -p ROS_TCP_PORT:=10000