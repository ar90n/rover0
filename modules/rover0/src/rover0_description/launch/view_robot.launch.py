from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("prefix", default_value='""', description="Prefix to be added to the robot description")
    )

    prefix = LaunchConfiguration("prefix")

    # Convert the URDF/XACRO file to URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rover0_description"), "urdf", "rover0.urdf.xacro"]),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution([FindPackageShare("rover0_description"), "rviz", "rover0.rviz"])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", rviz_config_file],
    )
    joint_state_pub_once = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/joint_states",
            "sensor_msgs/JointState",
            "'{"
            'header: {stamp: {sec: 1, nanosec: 1}, frame_id: ""}, '
            'name: ["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"], '  # noqa: E501
            "position: [0.0, 0.0, 0.0, 0.0], "  # e.g., set to 90 degrees
            "velocity: [0.0, 0.0, 0.0, 0.0], "
            "effort: [0.0, 0.0, 0.0, 0.0]"
            "}'",
        ],
        shell=True,
    )

    nodes = [robot_state_publisher_node, rviz_node, joint_state_pub_once]
    return LaunchDescription(declared_arguments + nodes)
