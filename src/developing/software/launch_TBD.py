#!/usr/bin/env python3
"""
Launch file for the complete robot system
Starts all necessary ROS2 nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Launch argument declarations
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        
        # Enhanced Robot Controller Server
        Node(
            package='robot_controller',  # Replace with your package name
            executable='enhanced_server',
            name='enhanced_robot_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            respawn=True,
            respawn_delay=2.0
        ),
        
        # Ultrasonic Sensor Node
        Node(
            package='robot_controller',  # Replace with your package name
            executable='ultrasonic_sensors',
            name='ultrasonic_sensor_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            respawn=True,
            respawn_delay=2.0
        ),
        
        # Camera Node (using v4l2_camera or usb_cam)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [1920, 1080],
                'pixel_format': 'YUYV',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_frame_id': 'camera_optical_frame',
                'use_sim_time': use_sim_time
            }],
            remappings=[
                ('image_raw', '/image_raw')
            ]
        ),
        
        # Object Tracker Node (if you have one)
        # Uncomment and modify according to your tracker implementation
        # Node(
        #     package='object_tracker',
        #     executable='tracker_node',
        #     name='object_tracker',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'target_color': 'red',  # or whatever you're tracking
        #         'publish_rate': 10.0
        #     }],
        #     remappings=[
        #         ('image_raw', '/image_raw'),
        #         ('tracker_data', '/tracker_data')
        #     ]
        # ),
    ])

if __name__ == '__main__':
    generate_launch_description()