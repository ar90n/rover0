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
        DeclareLaunchArgument(
            "foxglove_bridge_port",
            default_value="8765",
            description="Port of foxglove bridge",
        ),
    ]

    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    log_level = LaunchConfiguration("log_level")
    camera_params_file = LaunchConfiguration("camera_params_file")
    camera_calibration_file = LaunchConfiguration("camera_calibration_file")
    foxglove_bridge_port = LaunchConfiguration("foxglove_bridge_port")

    use_sim_time = IfElseSubstitution(use_sim, if_value="True", else_value="False")

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[{
            "move": {
                "type": "topic",
                "interface_type": "geometry_msgs/msg/Twist",
                "topic_name": "cmd_vel_teleop",
                "deadman_buttons": [5],
                "axis_mappings":{
                    "linear-x": {
                        "axis": 1,
                        "scale": 1.0,
                        "offset": 0.0,
                    },
                    "linear-y": {
                        "axis": 0,
                        "scale": 1.0,
                        "offset": 0.0,
                    },
                    "angular-z": {
                        "axis": 2,
                        "scale": 1.0,
                        "offset": 0.0,
                    }
                },
            },
            'use_sim_time': use_sim_time
        }],
    )

    teleop_nodes = GroupAction(
        actions=[
            Node(
                name="teleop_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[{"autostart": "true"}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            LoadComposableNodes(
                target_container="teleop_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="camera_ros",
                        plugin="camera::CameraNode",
                        name="camera",
                        parameters=[camera_params_file, {'camera_info_url': camera_calibration_file, 'use_sim_time': use_sim_time}],
                    ),
                ],
            ),
            LoadComposableNodes(
                target_container="teleop_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="foxglove_bridge",
                        plugin="foxglove_bridge::FoxgloveBridge",
                        name="foxglove_bridge",
                        parameters=[{
                            'port': foxglove_bridge_port,
                            "address"                   :"0.0.0.0" ,
                            "tls"                       :False ,
                            "certfile"                  :"" ,
                            "keyfile"                   :"" ,
                            "topic_whitelist"           :['.*'] ,
                            "param_whitelist"           :['.*'] ,
                            "service_whitelist"         :['.*'] ,
                            "client_topic_whitelist"    :['.*'] ,
                            "min_qos_depth"             :1 ,
                            "max_qos_depth"             :10 ,
                            "num_threads"               :0 ,
                            "send_buffer_limit"         :10000000 ,
                            "use_sim_time"              :use_sim_time ,
                            "capabilities"              :["clientPublish","parameters","parametersSubscribe","services","connectionGraph","assets"],
                            "include_hidden"            :False ,
                            "asset_uri_allowlist"       :['^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']
                        }],
                    ),
                ],
            ),
        ]
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

    nodes: list[LaunchDescriptionEntity] = [
        joy_teleop_node,
        teleop_nodes,
    ]

    return LaunchDescription(declared_arguments + nodes)
