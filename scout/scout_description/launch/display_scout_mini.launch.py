import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command

def generate_launch_description():
    # Paths for URDF and RViz files
    urdf_file = PathJoinSubstitution([
        FindPackageShare('scout_description'),
        'urdf',
        'scout_mini.urdf'
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('scout_description'),
        'rviz',
        'scout_mini_model_display.rviz'
    ])


    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('model', default_value='scout_mini', description='Robot model'),
        DeclareLaunchArgument('gui', default_value='False', description='Launch with GUI'),

        # Robot description parameter (URDF file)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # Joint state publisher node
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # RViz node with model display
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_file],
            additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])