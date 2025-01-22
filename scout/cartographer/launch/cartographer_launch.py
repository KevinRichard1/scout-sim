from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
            arguments=[
                '-configuration_directory', '/path/to/your/config',
                '-configuration_basename', 'scout_mini_3d.lua'
            ],
            remappings=[
                ('/points', '/lidar/points') 
            ]
        ),

        # Start RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/path/to/your/cartographer.rviz']
        ),
    ])
