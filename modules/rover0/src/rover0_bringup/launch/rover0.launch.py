from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rover0_description"), "urdf", "rover0.urdf.xacro"
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover0_bringup"), "config", "rover0_controllers.yaml"
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("rover0_description"), "rviz", "rover0.rviz"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both"
    )

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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        rover0_controller_spawner,
        imu_filter_madgwick_node,
        #rviz_node
    ])