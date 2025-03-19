import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch.actions import AppendEnvironmentVariable, GroupAction
from launch import LaunchDescription, LaunchDescriptionEntity, condition
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node, LoadComposableNodes
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    EqualsSubstitution,
    NotEqualsSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions.find_package import FindPackage
from launch.event_handlers import OnShutdown
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value='""', description="Prefix to be added to the robot description"),
        DeclareLaunchArgument("use_sim", default_value="False", description="Use gazebo sim"),
        DeclareLaunchArgument("log_level", default_value="info", description="log level"),
    ]

    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    log_level = LaunchConfiguration("log_level")

    use_sim_time = IfElseSubstitution(use_sim, if_value="True", else_value="False")

    robot_state_pub_node = Node(
        package="camera_ros",
        executable="camera_node",
        output="both",
        parameters=[],
    )

    nodes: list[LaunchDescriptionEntity] = [
        robot_state_pub_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
