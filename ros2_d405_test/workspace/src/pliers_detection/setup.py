from setuptools import setup

package_name = 'pliers_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Package for pliers detection using YOLO and Realsense D405',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_training_images = pliers_detection.save_training_images:main',
            'yolo_detector = pliers_detection.yolo_detector:main',
        ],
    },
)
