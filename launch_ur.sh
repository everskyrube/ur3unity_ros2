#!/bin/bash

# Launch UR3 without gripper, and joint_state_publisher
source /opt/ros/humble/setup.bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur3