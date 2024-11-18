import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.substitutions import FindExecutable
import launch

def generate_launch_description():
    # Gazebo launch file
    gazebo_launch_file = os.path.join(
        FindPackageShare('gazebo_ros').find('gazebo_ros'),
        'launch', 'empty_world.launch.py'  # ROS 2 Gazebo launch file path
    )

    # URDF file for the robot
    urdf_file = os.path.join(
        FindPackageShare('scout_description').find('scout_description'),
        'urdf', 'scout_mini.urdf'
    )

    return LaunchDescription([
        # Include Gazebo launch file
        IncludeLaunchDescription(
            gazebo_launch_file,
            launch_arguments={}.items()  # Add any arguments if needed
        ),

        # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40'],
            output='screen'
        ),

        # Spawn model in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',  # Use spawn_entity.py in ROS 2
            name='spawn_model',
            arguments=['-file', urdf_file, '-urdf', '-model', 'scout_description'],
            output='screen'
        ),

        # Fake joint calibration node
        Node(
            package='ros2topic',  # ROS 2 equivalent package for `rostopic`
            executable='pub',     # ROS 2 equivalent for `rostopic pub`
            name='fake_joint_calibration',
            arguments=['/calibrated', 'std_msgs/msg/Bool', 'true'],
            output='screen'
        ),
    ])