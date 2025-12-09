import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class BasicCommunicationNode(Node):
    """
    Basic communication node demonstrating ROS2 communication patterns
    """
    def __init__(self):
        super().__init__('basic_communication_node')

        # Create publishers
        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )

        self.joint_command_publisher = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )

        # Create subscribers
        self.command_subscriber = self.create_subscription(
            String,
            'commands',
            self.command_callback,
            10
        )

        self.sensor_subscriber = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Timer for publishing status
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Basic Communication Node initialized')

    def command_callback(self, msg):
        """Handle incoming commands"""
        self.get_logger().info(f'Received command: {msg.data}')
        # Process command and potentially send response

    def sensor_callback(self, msg):
        """Handle incoming sensor data"""
        self.get_logger().info(f'Received sensor data: {msg.data}')

    def publish_status(self):
        """Publish robot status"""
        msg = String()
        msg.data = f'Robot operational at {self.get_clock().now().seconds_nanoseconds()}'
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    communication_node = BasicCommunicationNode()

    try:
        rclpy.spin(communication_node)
    except KeyboardInterrupt:
        pass
    finally:
        communication_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()