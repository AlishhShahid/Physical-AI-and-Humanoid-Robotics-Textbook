from setuptools import setup

package_name = 'humanoid_control_pkg'

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
    description='ROS2 package for humanoid robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_controller = humanoid_control_pkg.humanoid_controller:main',
            'joint_state_publisher = humanoid_control_pkg.joint_state_publisher:main',
            'basic_communication = humanoid_control_pkg.basic_communication:main',
            'rclpy_examples = humanoid_control_pkg.rclpy_examples:main',
            'motor_driver = humanoid_control_pkg.motor_driver:main',
        ],
    },
)