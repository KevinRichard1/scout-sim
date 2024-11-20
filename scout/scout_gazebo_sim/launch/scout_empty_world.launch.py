import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    robot_namespace = DeclareLaunchArgument('robot_namespace', default_value='/', description='Robot Namespace')
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(get_package_share_directory('scout_gazebo_sim'), 'worlds', 'weston_robot_empty.world'),
        description='Path to the world file for Gazebo simulation'
    )

    # Include the empty world launch file from gazebo_ros
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'empty_world.launch.py')
        ),
        launch_arguments={
            'world_name': world_name,
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'true',
            'headless': 'false',
            'debug': 'false'
        }.items()
    )

    # Include spawn_scout_v2 launch file from scout_gazebo_sim
    spawn_scout = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('scout_gazebo_sim'), 'launch', 'spawn_scout_v2.launch.py')
        )
    )

    # Start RViz with the configuration
    rviz_node = Node(
        package='rviz',
        executable='rviz',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('scout_description'), 'rviz', 'navigation.rviz')]
    )

    # Return the launch description
    return LaunchDescription([
        robot_namespace,
        world_name,
        gazebo_launch,
        spawn_scout,
        rviz_node
    ])