"""PS5 DualSense → MoveIt Servo teleop stack for UR3 + Robotiq 2F-140 (mock hardware).

Pipeline:
  joy (device) → ps5_to_servo_teleop → /servo_node/delta_{twist,joint}_cmds
                                    ↓
                                moveit_servo → /forward_position_controller/commands
                                    ↓
                         ros2_control (mock_components/GenericSystem)
                                    ↓
                         /joint_states → robot_state_publisher → rviz
"""
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_yaml(pkg_share_dir: str, rel_path: str) -> dict:
    path = os.path.join(pkg_share_dir, rel_path)
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur3")
    launch_rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true")
    joy_device_arg = DeclareLaunchArgument("joy_device", default_value="/dev/input/js0")

    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    joy_device = LaunchConfiguration("joy_device")

    desc_pkg = FindPackageShare("ur3_with_gripper_description")
    moveit_pkg = FindPackageShare("ur3_with_gripper_moveit_config")

    # --- Robot description (URDF via xacro) ---
    urdf_xacro = PathJoinSubstitution(
        [desc_pkg, "urdf", "ur3_with_gripper.urdf.xacro"])
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            urdf_xacro, " ",
            "ur_type:=", ur_type, " ",
            "use_fake_hardware:=true", " ",
            "generate_ros2_control_tag:=true",
        ]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    # --- Semantic description (SRDF via xacro) ---
    srdf_xacro = PathJoinSubstitution(
        [moveit_pkg, "srdf", "ur3_with_gripper.srdf.xacro"])
    robot_description_semantic_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            srdf_xacro,
        ]),
        value_type=str,
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content}

    # --- MoveIt parameter yamls loaded eagerly (servo needs dict, not a file path) ---
    # Uses FindPackageShare().perform() at resolution time is too fragile; load from
    # the ament index instead.
    from ament_index_python.packages import get_package_share_directory
    moveit_share = get_package_share_directory("ur3_with_gripper_moveit_config")

    kinematics_yaml = _load_yaml(moveit_share, "config/kinematics.yaml")
    joint_limits_yaml = {"robot_description_planning":
                         _load_yaml(moveit_share, "config/joint_limits.yaml")}
    ompl_yaml = _load_yaml(moveit_share, "config/ompl_planning.yaml")
    servo_yaml = _load_yaml(moveit_share, "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    ros2_controllers_file = os.path.join(moveit_share, "config/ros2_controllers.yaml")
    ps5_teleop_file = os.path.join(moveit_share, "config/ps5_teleop.yaml")
    rviz_config = os.path.join(moveit_share, "rviz/servo_teleop.rviz")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # --- Core nodes ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, ros2_controllers_file],
    #     output="screen",
    # )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_file],
        remappings=[("/joint_states", "/arm/joint_states")],
        output="screen",
    )

    # Changed for /gripper/joint_states
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            robot_description,
            {"source_list": ["/arm/joint_states", "/gripper/joint_states"]}
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager",
                   "/controller_manager"],
        output="screen",
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager",
                   "/controller_manager"],
        output="screen",
    )

    # Make sure the position controller only starts after joint_state_broadcaster
    delay_forward_ctrl = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    # --- moveit_servo ---
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            ompl_yaml,
        ],
    )

    # --- Joystick + teleop bridge ---
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "device_name": "",
            "device_id": 0,
            "dev": joy_device,
            "deadzone": 0.05,
            "autorepeat_rate": 50.0,
        }],
        output="screen",
    )

    ps5_teleop = Node(
        package="ur3_with_gripper_moveit_config",
        executable="ps5_to_servo_teleop.py",
        name="ps5_to_servo_teleop",
        parameters=[ps5_teleop_file],
        output="screen",
    )

    # --- rviz ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[robot_description, robot_description_semantic,
                    robot_description_kinematics],
        output="log",
        condition=__import__("launch.conditions", fromlist=["IfCondition"]).IfCondition(launch_rviz),
    )

    # --- mode_widget ---
    mode_widget_node = Node(
        package='ur3_with_gripper_moveit_config',
        executable='mode_widget.py',
        name='mode_widget',
        output='screen'
    )


    return LaunchDescription([
        ur_type_arg,
        launch_rviz_arg,
        joy_device_arg,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_forward_ctrl,
        joint_state_publisher_node,
        servo_node,
        joy_node,
        ps5_teleop,
        rviz_node,
        mode_widget_node, 
    ])
