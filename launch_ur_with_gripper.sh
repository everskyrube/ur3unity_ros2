#!/bin/bash

# Launch UR3 without gripper, and joint_state_publisher
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_with_gripper_description view_ur3_with_gripper.launch.py