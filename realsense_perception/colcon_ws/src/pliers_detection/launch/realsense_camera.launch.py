from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            parameters=[{
                'enable_pointcloud': True,
                'align_depth': True,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
            }],
            output='screen',
        ),
    ])
