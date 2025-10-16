from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # Orbbec-Kamera starten (anpassen an dein Launch)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                '/workspace/src/OrbbecSDK_ROS2/orbbec_camera/launch', 'orbbec_camera.launch.py')]),
        ),
        # Dein Detection-Node
        Node(
            package='cube_detection',
            executable='cube_detector.py',
            name='cube_detector',
        ),
        # RViz für Visualisierung
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/path/to/config.rviz'],  # Erstelle eine RViz-Config
        ),
    ])