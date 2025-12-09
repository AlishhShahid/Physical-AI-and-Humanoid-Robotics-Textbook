import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import threading


class SimulatedMotorDriver(Node):
    """
    Simulated motor driver for humanoid robot joints
    """
    def __init__(self):
        super().__init__('motor_driver')

        # Motor state variables
        self.joint_positions = {
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0
        }

        self.joint_velocities = {joint: 0.0 for joint in self.joint_positions}
        self.joint_efforts = {joint: 0.0 for joint in self.joint_positions}

        # Publishers
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Subscribers
        self.joint_command_subscriber = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )

        self.motor_command_subscriber = self.create_subscription(
            Twist,
            'motor_commands',
            self.motor_command_callback,
            10
        )

        # Timer for updating motor states
        self.motor_timer = self.create_timer(0.01, self.update_motor_states)  # 100Hz

        self.get_logger().info('Simulated Motor Driver initialized')

    def joint_command_callback(self, msg):
        """Handle joint position commands"""
        joint_names = list(self.joint_positions.keys())

        for i, value in enumerate(msg.data):
            if i < len(joint_names):
                joint_name = joint_names[i]
                # Update position with simple proportional control
                target_pos = value
                current_pos = self.joint_positions[joint_name]

                # Simple proportional control to move toward target
                error = target_pos - current_pos
                self.joint_velocities[joint_name] = error * 5.0  # 5 rad/s per rad error

                # Update position based on velocity
                dt = 0.01  # 10ms based on timer
                self.joint_positions[joint_name] += self.joint_velocities[joint_name] * dt

    def motor_command_callback(self, msg):
        """Handle motor commands for movement"""
        self.get_logger().info(f'Motor command received: linear={msg.linear}, angular={msg.angular}')

        # Convert twist command to joint movements
        # This is a simplified example - in reality, inverse kinematics would be used
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Example: Turn left/right by moving hip joints differentially
        if angular_z != 0:
            self.joint_positions['left_hip_joint'] += angular_z * 0.02
            self.joint_positions['right_hip_joint'] -= angular_z * 0.02

        # Example: Move forward by moving legs
        if linear_x != 0:
            self.joint_positions['left_knee_joint'] += linear_x * 0.01
            self.joint_positions['right_knee_joint'] += linear_x * 0.01

    def update_motor_states(self):
        """Publish updated joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names and current values
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = list(self.joint_velocities.values())
        msg.effort = list(self.joint_efforts.values())

        self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    motor_driver = SimulatedMotorDriver()

    try:
        rclpy.spin(motor_driver)
    except KeyboardInterrupt:
        pass
    finally:
        motor_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()