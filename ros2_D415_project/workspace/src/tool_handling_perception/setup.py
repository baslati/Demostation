from setuptools import setup

package_name = 'tool_handling_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for tool handling perception',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_check = tool_handling_perception.01_camera_check:main',
            'realsense_launch = tool_handling_perception.02_realsense_launch:main',
            'yolo_detect = tool_handling_perception.03_yolo_detect:main',
            'pose_estimation = tool_handling_perception.04_pose_estimation:main',
        ],
    },
)