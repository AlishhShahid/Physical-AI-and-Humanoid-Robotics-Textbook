from setuptools import setup

package_name = 'isaac_ros_navigation'

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
    description='Isaac ROS package for navigation stack in humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_navigation_node = isaac_ros_navigation.isaac_navigation_node:main',
            'isaac_path_planner = isaac_ros_navigation.isaac_path_planner:main',
            'isaac_controller = isaac_ros_navigation.isaac_controller:main',
        ],
    },
)