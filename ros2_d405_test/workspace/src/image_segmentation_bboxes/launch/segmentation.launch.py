from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_segmentation_bboxes',
            executable='segmentation_node',
            name='segmentation_node',
            output='screen',
        ),
    ])