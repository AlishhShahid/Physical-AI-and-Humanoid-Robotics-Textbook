from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_control_pkg',
            executable='humanoid_controller',
            name='humanoid_controller',
            output='screen',
        ),
    ])
