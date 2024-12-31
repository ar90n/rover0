import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch.actions import AppendEnvironmentVariable
from launch import LaunchDescription, LaunchDescriptionEntity, condition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, IfElseSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions.find_package import FindPackage
from launch.event_handlers import OnShutdown

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir2 = get_package_share_directory('nav2_minimal_tb3_sim')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    declared_arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix to be added to the robot description'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps', 'tb3_sandbox.yaml'),
        ),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
            description='Full path to world model file to load',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='False',
            description='Use gazebo sim'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='Use rviz'
        ),
        DeclareLaunchArgument(
            'headless', default_value='False', description='Whether to execute gzclient)'
        )
    ]

    prefix = LaunchConfiguration('prefix')
    map_yaml_file = LaunchConfiguration('map')
    world = LaunchConfiguration('world')
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    package_name = IfElseSubstitution(use_sim, if_value="rover0_gz_description", else_value="rover0_robot_description")
    model_xacro_file_name = IfElseSubstitution(use_sim, if_value="rover0_gz.urdf.xacro", else_value="rover0_robot.urdf.xacro")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name), "urdf", model_xacro_file_name
                ],
            ),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}


    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        condition=UnlessCondition(use_sim)
    )
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "-c", "/controller_manager"],
        condition=UnlessCondition(use_sim)
    )

    rover0_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover0_controller",  "-c", "/controller_manager"],
    )

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        output="screen",
        parameters=[{
            "use_mag": False,
            "gain": 0.1,
        }],
        remappings=[
            ('/imu/data_raw', '/imu_sensor_broadcaster/imu'),
            ('/imu/data', '/imu/filtered')
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("rover0_description"), "rviz", "rover0.rviz"
        ]
    )
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )


    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover0_bringup"), "config", "rover0_controllers.yaml"
        ]
    )
    ros2_control_node = Node(
        condition=UnlessCondition(use_sim),
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both"
    )

    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
        condition=IfCondition(use_sim)
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(
            [use_sim, ' and not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    ros_gz_bridge_config_file = PathJoinSubstitution(
        [FindPackageShare("rover0_gz_description"), "config", "rover0_bridge.yaml"]
    )
    ros_gz_bridge_node = Node(
        condition=IfCondition(use_sim),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config_file,
            'expand_gz_topic_names': True,
            'use_sim_time': True,
        }],
        output='screen'
    )

    spawn_entity_node = Node(
      condition=IfCondition(use_sim),
      package='ros_gz_sim',
      executable='create',
      arguments=[
          '-topic', '/robot_description',
          '-name', 'rover0',
          '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
          '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']
      ],
      output='screen'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(bringup_dir2, 'models'))
    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(bringup_dir2)).parent.resolve()))

    nodes: list[LaunchDescriptionEntity]= [
        set_env_vars_resources,
        set_env_vars_resources2,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        rover0_controller_spawner,
        imu_filter_madgwick_node,
        rviz_node,
        ros2_control_node,
        world_sdf_xacro,
        gazebo_server,
        gazebo_client,
        remove_temp_sdf_file,
        ros_gz_bridge_node,
        spawn_entity_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
