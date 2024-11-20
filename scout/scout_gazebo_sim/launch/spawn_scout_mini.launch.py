import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Paths for URDF and RViz files
    urdf_file = PathJoinSubstitution([
        FindPackageShare('scout_description'),
        'urdf',
        'scout_mini.urdf'
    ])

    return LaunchDescription([
        # Declare launch arguments for initial pose and other settings
        DeclareLaunchArgument('x', default_value='0.0', description='Initial x position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial y position'),
        DeclareLaunchArgument('z', default_value='0.0', description='Initial z position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial yaw angle'),
        DeclareLaunchArgument('robot_namespace', default_value='/'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='true'),
        DeclareLaunchArgument('config_file', default_value='config/scout_v2_control.yaml', description='Path to the parameter file'),

        # Spawn the Scout model in Gazebo
        Node(
            name='spawn_scout_model',
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
                '-unpause',
                '-urdf',
                '-param', 'robot_description',
                '-model', LaunchConfiguration('robot_namespace'),
                '-entity',
                'scout_mini',
            ]
        ),

        # Load joint controller configurations directly into the node
        Node(
            package='scout_gazebo_sim',
            executable='scout_skid_steer_controller',  # Use the compiled executable here
            name='load_joint_control_params',
            output='screen',
            parameters=[os.path.join(FindPackageShare('scout_gazebo_sim').find('scout_gazebo_sim'), 'config', 'scout_v2_control.yaml')]
        ),

        # Load controllers using controller manager
        Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            output='screen',
            arguments=[
                'scout_state_controller',
                'scout_motor_fr_controller',
                'scout_motor_fl_controller',
                'scout_motor_rl_controller',
                'scout_motor_rr_controller'
            ]
        ),

        # Start robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        # Start the Scout Skid Steer controller
        Node(
            package='scout_gazebo_sim',
            executable='scout_skid_steer_controller',
            name='scout_skid_steer_controller',
            output='screen',
            parameters=[{'robot_namespace': LaunchConfiguration('robot_namespace')}]
        ),
    ])
