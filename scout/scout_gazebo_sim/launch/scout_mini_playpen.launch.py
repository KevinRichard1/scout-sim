from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

scout_description_path = os.path.join(
    get_package_share_directory('scout_description'), 'share'
)
os.environ['GAZEBO_MODEL_PATH'] = f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{scout_description_path}"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

def generate_launch_description():
    world_file = get_package_share_directory('scout_gazebo_sim') + '/worlds/clearpath_playpen.world'

    urdf_file = PathJoinSubstitution([
        FindPackageShare('scout_description'),
        'urdf',
        'scout_mini.xacro'
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        ),


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
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
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
                '-file', urdf_file,
                '-entity', 'scout_mini',
                '-robot_namespace', LaunchConfiguration('robot_namespace'),
                '-x', '0.0', '-y', '0.0', '-z', '0.2'
            ],
            output='screen',
        ),
    ])