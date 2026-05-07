cmake_minimum_required(VERSION 3.8)
project(ur3_with_gripper_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config srdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/ps5_to_servo_teleop.py
    scripts/mode_widget.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
