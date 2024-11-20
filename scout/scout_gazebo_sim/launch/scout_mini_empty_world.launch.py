import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as ROS2Node  # Correct import for ROS 2 Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace', default_value='/',
            description='Namespace for the robot'
        ),
        DeclareLaunchArgument(
            'world_name', default_value=os.path.join(get_package_share_directory('scout_gazebo_sim'), 'worlds', 'weston_robot_empty.world'),
            description='World to load in Gazebo'
        ),
        
        # Load the Gazebo world
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name'),
                'paused': 'false',
                'use_sim_time': 'true',
                'gui': 'true',
                'headless': 'false',
                'debug': 'false'
            }.items()
        ),
        
        # Spawn the Scout Mini robot
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('scout_gazebo_sim'), 'launch', 'spawn_scout_mini.launch.py')
        ),
        
        # Launch RViz with the given configuration
        ROS2Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('scout_description'), 'rviz', 'navigation.rviz')]
        ),
    ])