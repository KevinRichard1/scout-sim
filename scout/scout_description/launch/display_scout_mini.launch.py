import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare

def generate_launch_description():
    # Get the file paths for URDF and RViz configuration
    urdf_file = os.path.join(
        FindPackageShare('scout_description').find('scout_description'),
        'urdf', 'scout_mini.urdf'
    )

    rviz_config_file = os.path.join(
        FindPackageShare('scout_description').find('scout_description'),
        'rviz', 'scout_mini_model_display.rviz'
    )

    # Define launch arguments
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
            parameters=[{'robot_description': urdf_file}]
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
            arguments=['-d', rviz_config_file]
        ),
    ])