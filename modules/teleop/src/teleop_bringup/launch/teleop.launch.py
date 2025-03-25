import os
import tempfile
from pathlib import Path
from urllib.parse import urljoin

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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    teleop_bringup_dir = get_package_share_directory("teleop_bringup")
    print(teleop_bringup_dir)

    declared_arguments = [
        DeclareLaunchArgument("prefix", default_value='""', description="Prefix to be added to the robot description"),
        DeclareLaunchArgument("use_sim", default_value="False", description="Use gazebo sim"),
        DeclareLaunchArgument("log_level", default_value="info", description="log level"),
        DeclareLaunchArgument(
            "camera_calibration_file",
            default_value=str(urljoin('file://', os.path.join(teleop_bringup_dir, "config", "ov5647_calibration.yaml"))),
            description="Full path to the camera calibration for cmeara",
        ),
        DeclareLaunchArgument(
            "camera_params_file",
            default_value=os.path.join(teleop_bringup_dir, "config", "camera_params.yaml"),
            description="Full path to the ROS params for cmeara",
        ),
    ]

    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    log_level = LaunchConfiguration("log_level")
    camera_params_file = LaunchConfiguration("camera_params_file")
    camera_calibration_file = LaunchConfiguration("camera_calibration_file")

    use_sim_time = IfElseSubstitution(use_sim, if_value="True", else_value="False")

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name='camera',
        output="both",
        parameters=[camera_params_file, {'camera_info_url': camera_calibration_file}],
    )

    #image_transport_node = Node(
    #    package='image_transport',
    #    executable='republish',
    #    name='video_republisher',
    #    parameters=[
    #        {'out.foxglove.encoding': 'h264_v4l2m2m'},
    #        {'out.foxglove.gop_size': 1},
    #        {'out.foxglove.bit_rate': 4000000},
    #        {'out.foxglove.pixel_format': 'yuv420p'},
    #        {'out_transport': 'foxglove'},
    #    ],
    #    arguments=['raw', 'foxglove'],
    #    remappings=[
    #        ('in', '/camera/image_raw'),
    #        ('out', '/camera/image_h264'),
    #    ],
    #)

    foxglove_bridge_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        ),
        launch_arguments={'port': '8765'}.items()
    )

    nodes: list[LaunchDescriptionEntity] = [
        camera_node,
        foxglove_bridge_node
    ]

    return LaunchDescription(declared_arguments + nodes)
