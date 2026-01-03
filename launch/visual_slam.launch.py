from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['--delete_db_on_start'],
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'use_sim_time': use_sim_time,
                'approx_sync': True,
                'wait_for_transform': 0.5,
                'map_always_update': True,
                'queue_size': 20,
                'Reg/Force3DoF': 'true',
                'Optimizer/Slam2D': 'true',
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false'
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/odom')                 # Odometry topic
            ]
        ),

        # RTAB-Map Visualization (Optional)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/odom')
            ]
        ),
    ])
