from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Realsense-Kamera starten
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                '/workspace/src/pliers_detection/launch', 'realsense_camera.launch.py')]),
        ),
        # Detection-Node
        Node(
            package='pliers_detection',
            executable='plier_detector.py',
            name='pliers_detector',
        ),
        # RViz für Visualisierung
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join('/workspace/src/pliers_detection/launch', 'pliers_detection.rviz')],
        ),
    ])