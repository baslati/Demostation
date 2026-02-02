from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            parameters=[{
                'enable_pointcloud': False,  # Not needed for YOLO
                'align_depth': True,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
            }],
            output='screen',
        ),
        Node(
            package='pliers_yolo_detection',
            executable='pliers_yolo_node.py',
            name='pliers_yolo_node',
            output='screen',
            parameters=[
                {'yolo_model': 'yolov8n.pt'},
                {'target_class_id': 76}  # Scissors as proxy for pliers
            ]
        )
    ])