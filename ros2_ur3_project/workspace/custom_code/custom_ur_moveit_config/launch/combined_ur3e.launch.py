from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Pfade zu den Launch-Files
    moveit_launch = os.path.join(
        get_package_share_directory('custom_ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    )
    control_launch = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    )

    # Standard-Parameter
    ur_type = LaunchConfiguration('ur_type', default='ur3e')
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.122.20')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')
    description_package = LaunchConfiguration('description_package', default='custom_ur_description')
    moveit_config_package = LaunchConfiguration('moveit_config_package', default='custom_ur_moveit_config')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')


    # Spawner für die wichtigsten Controller (werden direkt nach Start aktiviert)
    from launch_ros.actions import Node as RosNode
    controller_spawners = [
        RosNode(
            package='controller_manager',
            executable='spawner',
            arguments=['scaled_joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    ]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'use_fake_hardware': use_fake_hardware,
                'description_package': description_package,
                'moveit_config_package': moveit_config_package,
                'launch_rviz': launch_rviz
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'use_fake_hardware': use_fake_hardware,
                'launch_rviz': 'false'
            }.items()
        ),
        *controller_spawners
    ])

# Hilfsfunktion für ROS2 package share directory
from ament_index_python.packages import get_package_share_directory
