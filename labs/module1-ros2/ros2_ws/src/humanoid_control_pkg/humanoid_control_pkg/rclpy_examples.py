import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time


class RclpyIntegrationNode(Node):
    """
    Example node demonstrating various rclpy integration patterns
    """
    def __init__(self):
        super().__init__('rclpy_integration_node')

        # QoS profile for different communication needs
        qos_profile = QoSProfile(depth=10)

        # Publishers
        self.status_publisher = self.create_publisher(String, 'robot_status', qos_profile)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', qos_profile)

        # Subscribers
        self.cmd_subscriber = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            qos_profile
        )

        # Service client example
        # self.service_client = self.create_client(SomeService, 'some_service')

        # Timer with different rates
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.joint_timer = self.create_timer(0.1, self.publish_joint_state)

        # Parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('control_rate', 50)  # Hz

        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.get_logger().info(f'Rclpy Integration Node initialized for {robot_name}')

    def command_callback(self, msg):
        """Handle commands with proper error handling"""
        try:
            self.get_logger().info(f'Received command: {msg.data}')
            # Process command
            response_msg = String()
            response_msg.data = f'Processed: {msg.data}'
            self.status_publisher.publish(response_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def publish_status(self):
        """Publish robot status"""
        msg = String()
        msg.data = f'Status: OK - {time.time()}'
        self.status_publisher.publish(msg)

    def publish_joint_state(self):
        """Publish joint state"""
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [0.0, 0.1, 0.2]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.joint_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    rclpy_node = RclpyIntegrationNode()

    try:
        rclpy.spin(rclpy_node)
    except KeyboardInterrupt:
        rclpy_node.get_logger().info('Interrupted, shutting down...')
    finally:
        rclpy_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()