import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot')
    
    remappings = [
        ('/cmd_vel', '/cmd_vel'),  # Ensure cmd_vel is sent to robot
    ]

    a_star_navigator_node = Node(
        package='my_bot',
        executable='a_star_navigator.py',
        name='a_star_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_tolerance': 0.2,
            'lookahead_distance': 0.5,
            'max_linear_vel': 0.4,
            'max_angular_vel': 1.2
        }],
        remappings=remappings
    )

    return LaunchDescription([
        a_star_navigator_node
    ])

