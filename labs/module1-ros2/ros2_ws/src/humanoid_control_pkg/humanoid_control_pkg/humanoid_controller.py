import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class HumanoidController(Node):
    """
    Basic humanoid robot controller node
    """
    def __init__(self):
        super().__init__('humanoid_controller')

        # Create publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Create subscriber for commands
        self.command_subscriber = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info('Humanoid Controller node initialized')

    def command_callback(self, msg):
        """Handle incoming robot commands"""
        self.get_logger().info(f'Received command: {msg.data}')
        # Process command and execute appropriate action

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.name = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]
        msg.position = [0.0] * len(msg.name)  # Default position
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    humanoid_controller = HumanoidController()

    try:
        rclpy.spin(humanoid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()