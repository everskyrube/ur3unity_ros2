from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    xacro_path = PathJoinSubstitution([
        FindPackageShare("ur3_with_gripper_description"),
        "urdf",
        "ur3_with_gripper.urdf.xacro",
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        xacro_path,
        " ",
        "ur_type:=", ur_type,
    ])

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur3_with_gripper_description"),
        "rviz",
        "view_robot.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),
    ])
