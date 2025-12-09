from setuptools import setup

package_name = 'isaac_ros_perception'

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
    description='Isaac ROS package for perception modules in humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_object_detection = isaac_ros_perception.isaac_object_detection:main',
            'isaac_segmentation = isaac_ros_perception.isaac_segmentation:main',
            'isaac_feature_extraction = isaac_ros_perception.isaac_feature_extraction:main',
        ],
    },
)