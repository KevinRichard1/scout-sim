from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace', default_value='/',
            description='Namespace for the robot'
        ),
        DeclareLaunchArgument(
            'world_name', default_value=os.path.join(
                FindPackageShare('scout_gazebo_sim').find('scout_gazebo_sim'),
                'worlds/clearpath_playpen.world'
            ),
            description='World to load in Gazebo'
        ),
        
        # Load the robot description parameter (URDF or Xacro)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    os.path.join(
                        FindPackageShare('scout_description').find('scout_description'),
                        'urdf/scout_mini.xacro'
                    )
                ])
            }]
        ),
        
        # Start Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch', 'gzserver.launch.py'
                )
            ]),
            launch_arguments={
                'world': LaunchConfiguration('world_name'),
                'use_sim_time': 'true'
            }.items(),
        ),

        # Start Gazebo client (optional, for GUI visualization)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch', 'gzclient.launch.py'
                )
            ]),
        ),

        # Wait for Gazebo spawn service
        TimerAction(
            period=5.0,
            actions=[LogInfo(msg="Waiting for Gazebo spawn service...")]
        ),
        
        # Spawn the Scout robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=[
                '-file', os.path.join(
                    FindPackageShare('scout_description').find('scout_description'),
                    'urdf/scout_mini.xacro'
                ),
                '-entity', 'scout_mini',
                '-robot_namespace', LaunchConfiguration('robot_namespace'),
                '-x', '0.0', '-y', '0.0', '-z', '0.0'
            ],
            output='screen',
        ),
    ])
