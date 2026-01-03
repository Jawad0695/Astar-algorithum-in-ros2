import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Helper to find the package share directory
    pkg_my_bot = FindPackageShare('my_bot')
    
    # Path to the ORB-SLAM configuration file
    config_file_path = PathJoinSubstitution([
        pkg_my_bot, 
        'config', 
        'orb_slam_webcam_params.yaml'
    ])
    
    # Declare the vocabulary path argument
    declare_vocab_path = DeclareLaunchArgument(
        'vocab_path',
        default_value='/home/jawad/bocbot_ws/src/ORB_SLAM3_ROS2/ORB_SLAM3/vocabulary/ORBvoc.txt', 
        description='Path to ORB-SLAM vocabulary file'
    )
    
    # Declare the camera device argument
    declare_video_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Webcam device path'
    )
    
    # Launch Webcam Driver Node (v4l2_camera)
    webcam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'time_per_frame': [1, 30],
            'camera_frame_id': 'camera_link'
        }]
    )
    
    # Launch ORB-SLAM Node
    # Many ROS2 ports of ORB_SLAM3 expect [voc_file] [settings_file] as positional arguments
    orb_slam_node = Node(
        package='orb_slam3_ros2',
        executable='orb_alt', # Keeping this for now, but see note below
        name='orb_slam3',
        output='screen',
        arguments=[
            LaunchConfiguration('vocab_path'),
            config_file_path
        ],
        parameters=[
            {'use_sim_time': False}
        ],
        remappings=[
            ('/camera/image_raw', '/image_raw')
        ]
    )

    return LaunchDescription([
        declare_vocab_path,
        declare_video_device,
        webcam_node,
        orb_slam_node
    ])
