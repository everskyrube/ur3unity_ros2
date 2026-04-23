#!/bin/bash

# Launch UR3 without gripper, and joint_state_publisher
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur3_with_gripper_moveit_config servo_teleop.launch.py

# To skip rviz (e.g. headless test):                                                       
# ros2 launch ur3_with_gripper_moveit_config servo_teleop.launch.py launch_rviz:=false  