import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    DeclareLaunchArgument('robot_namespace', default_value='/')
    DeclareLaunchArgument('urdf_extras', default_value='empty.urdf')

    # Get paths for URDF and xacro
    xacro_file = os.path.join(
        FindPackageShare('scout_description').find('scout_description'),
        'urdf', 'scout_mini.urdf'
    )

    urdf_extras = os.path.join(
        FindPackageShare('scout_description').find('scout_description'),
        'urdf', 'empty.urdf'
    )

    # Process the xacro file
    robot_description = xacro.process_file(xacro_file, mappings={
        'robot_namespace': LaunchConfiguration('robot_namespace'),
        'urdf_extras': urdf_extras
    }).toxml()

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('robot_namespace', default_value='/'),
        DeclareLaunchArgument('urdf_extras', default_value=urdf_extras),

        # Set the robot description as a parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
    ])