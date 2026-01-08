#!/usr/bin/env python3

import launch
import launch_ros.actions
import launch.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            parameters=[{
                'enable_pointcloud': True,
                'align_depth': True,
            }],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])

if __name__ == '__main__':
    launch.launch(generate_launch_description())