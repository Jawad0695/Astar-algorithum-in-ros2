import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = get_package_share_directory('my_bot')
    
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # Launch slam_toolbox in mapping mode
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[('/scan', '/scan')] # Ensure it matches
    )

    # Static transform: base_footprint -> base_link (if not already published by robot_state_publisher)
    # The navigator now handles both, but having a clean TF tree is good.
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        slam_toolbox_node
    ])
