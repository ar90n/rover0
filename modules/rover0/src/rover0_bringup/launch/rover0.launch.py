from launch import LaunchDescription, LaunchDescriptionEntity, condition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def get_robot_description(use_sim: bool):
    package_name = "rover0_gz_description" if use_sim else "rover0_robot_description"
    xacro_file_name = "rover0_gz.urdf.xacro" if use_sim else "rover0_robot.urdf.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name), "urdf", xacro_file_name
                ]
            ),
        ]
    )
    return {"robot_description": ParameterValue(robot_description_content, value_type=str)}

def generate_launch_description():
    default_world_file_path = PathJoinSubstitution([
        FindPackageShare('rover0_gz_description'),
        'world',
        'my_world.sdf'
    ])

    declared_arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix to be added to the robot description'
        ),
        DeclareLaunchArgument(
            'world_sdf_file',
            default_value=default_world_file_path,
            description='Path to the SDF world file'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use gazebo sim'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Use rviz'
        )
    ]

    prefix = LaunchConfiguration('prefix')
    world_sdf_file = LaunchConfiguration('world_sdf_file')
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')

    print(use_sim)

    robot_description = get_robot_description(use_sim)

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
    )
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "-c", "/controller_manager"],
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
            '/launch/gz_sim.launch.py'
        ]),
        condition=IfCondition(use_sim),
        launch_arguments={'gz_args': ['-r ', world_sdf_file],  'on_exit_shutdown': 'true'}.items()
    )

    ros_gz_bridge_config_file = PathJoinSubstitution(
        [FindPackageShare("rover0_gz_description"), "config", "rover0_bridge.yaml"]
    )
    ros_gz_bridge_node = Node(
        condition=IfCondition(use_sim),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config_file
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
          "-z", '0.01',
      ],
      output='screen'
    )
    nodes: list[LaunchDescriptionEntity]= [
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        rover0_controller_spawner,
        imu_filter_madgwick_node,
        rviz_node,
        ros2_control_node,
        gazebo,
        ros_gz_bridge_node,
        spawn_entity_node
    ]

    return LaunchDescription(declared_arguments + nodes)
