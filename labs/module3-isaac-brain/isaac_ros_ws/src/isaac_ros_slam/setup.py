from setuptools import setup

package_name = 'isaac_ros_slam'

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
    maintainer='Humanoid Robotics Team',
    maintainer_email='humanoid@robotics.edu',
    description='Isaac ROS package for SLAM capabilities in humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_slam_node = isaac_ros_slam.isaac_slam_node:main',
            'isaac_mapping_node = isaac_ros_slam.isaac_mapping_node:main',
            'isaac_localization_node = isaac_ros_slam.isaac_localization_node:main',
        ],
    },
)