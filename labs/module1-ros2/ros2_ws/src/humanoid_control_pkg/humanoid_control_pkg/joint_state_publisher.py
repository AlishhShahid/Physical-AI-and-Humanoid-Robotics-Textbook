import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


class JointStatePublisher(Node):
    """
    Joint state publisher for the humanoid robot
    """
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)

        # Initialize joint positions
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

        self.get_logger().info('Joint State Publisher node initialized')

    def publish_joint_states(self):
        """Publish joint states with simulated movement"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        # Simulate some simple joint movement
        time = self.get_clock().now().nanoseconds / 1e9
        for i in range(len(self.joint_names)):
            self.joint_positions[i] = math.sin(time + i) * 0.1

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()