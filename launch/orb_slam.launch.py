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
        'orb_slam_bot_params.yaml'
    ])
    
    declare_vocab_path = DeclareLaunchArgument(
        'vocab_path',
        default_value='/home/jawad/bocbot_ws/src/ORB_SLAM3_ROS2/ORB_SLAM3/Vocabulary/ORBvoc.txt', 
        description='Path to ORB-SLAM vocabulary file'
    )

    declare_use_pangolin = DeclareLaunchArgument(
        'use_pangolin',
        default_value='false',
        description='Whether to use Pangolin for visualization'
    )

    declare_sensor_type = DeclareLaunchArgument(
        'sensor_type',
        default_value='monocular',
        description='Sensor type: monocular or imu-monocular'
    )

    declare_executable = DeclareLaunchArgument(
        'executable',
        default_value='imu_mono_node_cpp',
        description='Executable to run: imu_mono_node_cpp or orb_alt'
    )
    
    # Launch ORB-SLAM Node
    orb_slam_node = Node(
        package='orb_slam3_ros2',
        executable=LaunchConfiguration('executable'),
        name='orb_slam3',
        output='screen',
        parameters=[
            {'voc_file': LaunchConfiguration('vocab_path')},
            {'settings_file': config_file_path},
            {'use_sim_time': True},
            {'sensor_type': LaunchConfiguration('sensor_type')},
            {'use_pangolin': LaunchConfiguration('use_pangolin')}
        ],
        remappings=[
            ('camera/camera/color/image_raw', '/camera/image_raw'),
            ('camera/camera/imu', '/imu')
        ]
    )

    return LaunchDescription([
        declare_vocab_path,
        declare_use_pangolin,
        declare_sensor_type,
        declare_executable,
        orb_slam_node
    ])
