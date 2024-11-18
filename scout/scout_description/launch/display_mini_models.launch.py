import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare
import launch_ros
import xacro


def generate_launch_description():
    # Path to the xacro file
    xacro_file = os.path.join(
        FindPackageShare('scout_description').find('scout_description'),
        'urdf', 'mini.xacro'
    )

    # Load the xacro file into a robot description parameter
    robot_description = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # Declare the argument 'model'
        DeclareLaunchArgument('model', default_value='robot'),

        # Robot description parameter (xacro processed)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint state publisher GUI node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz node with model display
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(
                FindPackageShare('scout_description').find('scout_description'),
                'rviz', 'model_display.rviz'
            )]
        ),
    ])
