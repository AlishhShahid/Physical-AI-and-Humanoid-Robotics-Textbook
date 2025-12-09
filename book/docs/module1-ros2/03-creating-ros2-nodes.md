# Creating ROS 2 Nodes for Humanoid Control

## Introduction

In this chapter, we'll explore the creation of ROS 2 nodes that form the core of our humanoid robot's nervous system. We'll build upon the basic structure created in the previous chapter and implement specific functionality for controlling our humanoid robot.

## Understanding ROS 2 Node Architecture

A ROS 2 node is an executable that uses ROS 2 client library to communicate with other nodes. In our humanoid robot, nodes will handle:

- Joint state publishing
- Motor control
- Command processing
- Sensor data handling
- High-level control logic

## The Humanoid Controller Node

The humanoid controller node is the primary control node for our robot. Let's examine its structure:

```python
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
```

## Key Components Explained

### Publishers
Publishers send messages to topics. In our controller:
- `joint_state_publisher`: Publishes the current state of all joints
- `cmd_vel_publisher`: Publishes velocity commands

### Subscribers
Subscribers receive messages from topics:
- `command_subscriber`: Receives high-level commands for the robot

### Timers
Timers allow for periodic execution of functions:
- `self.timer`: Publishes joint states every 0.1 seconds

## The Motor Driver Node

The motor driver node simulates the interface with physical motors:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time


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

        # Publishers and Subscribers
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        self.joint_command_subscriber = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )

        # Timer for updating motor states
        self.motor_timer = self.create_timer(0.01, self.update_motor_states)

        self.get_logger().info('Simulated Motor Driver initialized')
```

## Creating Custom Messages

While we're using standard ROS 2 message types, you may need custom messages for specific applications. To create a custom message:

1. Create a `msg` directory in your package
2. Define your message in a `.msg` file
3. Update your `package.xml` with the message generation dependency
4. Use the message in your nodes

## Best Practices for Node Development

### 1. Error Handling
Always include proper error handling in your nodes:

```python
def command_callback(self, msg):
    try:
        self.get_logger().info(f'Received command: {msg.data}')
        # Process command
    except Exception as e:
        self.get_logger().error(f'Error processing command: {str(e)}')
```

### 2. Parameter Management
Use ROS 2 parameters for configurable values:

```python
def __init__(self):
    super().__init__('humanoid_controller')
    self.declare_parameter('control_rate', 10.0)
    self.rate = self.get_parameter('control_rate').value
```

### 3. Resource Management
Properly clean up resources when the node is destroyed:

```python
def destroy_node(self):
    # Clean up any resources
    super().destroy_node()
```

## Running and Testing Your Nodes

To run your nodes:

```bash
# Source your workspace
source ~/humanoid_ws/install/setup.bash

# Run the humanoid controller
ros2 run humanoid_control_pkg humanoid_controller

# In another terminal, run the motor driver
ros2 run humanoid_control_pkg motor_driver
```

To check that your nodes are running:

```bash
# List all running nodes
ros2 node list

# Check topics published by a node
ros2 node info /humanoid_controller
```

## Next Steps

In the next chapter, we'll explore how to create launch files to start multiple nodes simultaneously and how to configure your robot using parameters. This will allow us to easily start our entire humanoid robot system with a single command.