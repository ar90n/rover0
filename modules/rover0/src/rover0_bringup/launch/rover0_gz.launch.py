from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    world_file_path = PathJoinSubstitution([
        FindPackageShare('rover0_gz_description'),
        'world',
        'my_world.sdf'
    ])

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix to be added to the robot description'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_sdf_file',
            default_value=world_file_path,
            description='Path to the SDF world file'
        )
    )

    prefix = LaunchConfiguration('prefix')
    world_sdf_file = LaunchConfiguration('world_sdf_file')

    # Convert the URDF/XACRO file to URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover0_gz_description"), "urdf", "rover0_gz.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rover0_description"), "rviz", "rover0.rviz"]
    )
    ros_gz_bridge_config_file = PathJoinSubstitution(
        [FindPackageShare("rover0_gz_description"), "config", "rover0_bridge.yaml"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        arguments=['-d', rviz_config_file],
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': ['-r ', world_sdf_file],  'on_exit_shutdown': 'true'}.items()
    )
    spawn_entity = Node(
      package='ros_gz_sim',
      executable='create',
      arguments=[
          '-topic', '/robot_description',
          '-name', 'rover0',
          "-z", '0.01',
      ],
      output='screen'
    )
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config_file
        }],
        output='screen'
    )
    rover0_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover0_controller",  "-c", "/controller_manager"],
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
        ros_gz_bridge_node,
        gazebo,
        spawn_entity,
        rover0_controller_spawner
    ]
    return LaunchDescription(declared_arguments + nodes)
