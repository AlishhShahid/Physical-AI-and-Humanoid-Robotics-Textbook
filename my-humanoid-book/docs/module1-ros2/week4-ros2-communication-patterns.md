---
sidebar_position: 2
---

# Week 4: ROS2 Communication Patterns

## Learning Objectives

By the end of this week, you will be able to:
- Implement different ROS2 communication patterns (topics, services, actions)
- Design robust communication architectures for humanoid robots
- Handle asynchronous communication and error scenarios
- Optimize communication for real-time robotic applications

## Introduction to ROS2 Communication Patterns

ROS2 provides three primary communication patterns for robotic applications:
1. **Topics**: Publish-subscribe pattern for continuous data streams
2. **Services**: Request-response pattern for synchronous operations
3. **Actions**: Goal-result-feedback pattern for long-running tasks

## Topic-Based Communication

Topics implement a publish-subscribe pattern that's ideal for continuous data streams like sensor readings, robot states, or control commands.

### Publisher Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
import math
import time

class HumanoidPublisherNode(Node):
    """
    Publisher node for humanoid robot state information
    """
    def __init__(self):
        super().__init__('humanoid_publisher')

        # Create publishers for different data types
        self.joint_state_pub = self.create_publisher(JointState, '/humanoid_robot/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/humanoid_robot/status', 10)

        # Timer for periodic publishing
        self.timer = self.create_timer(0.1, self.publish_robot_state)  # 10 Hz

        self.get_logger().info('Humanoid Publisher Node initialized')

    def publish_robot_state(self):
        """Publish current robot state"""
        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'

        # Define joint names (example for humanoid)
        joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        joint_state.name = joint_names

        # Generate example joint positions (in radians)
        positions = []
        for i, name in enumerate(joint_names):
            # Create oscillating joint positions for demonstration
            pos = math.sin(time.time() + i) * 0.5
            positions.append(pos)

        joint_state.position = positions

        # Publish joint states
        self.joint_state_pub.publish(joint_state)

        # Publish status message
        status_msg = String()
        status_msg.data = f'Publishing joint states - Time: {time.time()}'
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    publisher_node = HumanoidPublisherNode()

    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class HumanoidSubscriberNode(Node):
    """
    Subscriber node for humanoid robot commands and sensor data
    """
    def __init__(self):
        super().__init__('humanoid_subscriber')

        # Create subscribers for different topics
        self.joint_state_sub = self.create_subscription(
            JointState, '/humanoid_robot/joint_states', self.joint_state_callback, 10)

        self.status_sub = self.create_subscription(
            String, '/humanoid_robot/status', self.status_callback, 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/humanoid_robot/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('Humanoid Subscriber Node initialized')

    def joint_state_callback(self, msg):
        """Handle joint state messages"""
        self.get_logger().debug(f'Received joint states for {len(msg.name)} joints')

        # Process joint positions
        for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
            if abs(pos) > 1.5:  # Check for potentially unsafe positions
                self.get_logger().warn(f'Joint {name} position {pos} may be out of safe range')

    def status_callback(self, msg):
        """Handle status messages"""
        self.get_logger().info(f'Status: {msg.data}')

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.get_logger().info(f'Velocity command: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), '
                              f'angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = HumanoidSubscriberNode()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service-Based Communication

Services provide request-response communication suitable for operations that have a clear beginning and end.

### Service Implementation

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool, Trigger
from std_msgs.msg import Bool

class HumanoidServiceNode(Node):
    """
    Service node for humanoid robot operations
    """
    def __init__(self):
        super().__init__('humanoid_service')

        # Create services
        self.emergency_stop_srv = self.create_service(
            SetBool, '/humanoid_robot/emergency_stop', self.emergency_stop_callback)

        self.calibrate_srv = self.create_service(
            Trigger, '/humanoid_robot/calibrate', self.calibrate_callback)

        # Publisher for emergency stop status
        self.emergency_status_pub = self.create_publisher(Bool, '/humanoid_robot/emergency_status', 10)

        self.emergency_stopped = False

        self.get_logger().info('Humanoid Service Node initialized')

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service request"""
        self.get_logger().info(f'Emergency stop requested: {request.data}')

        self.emergency_stopped = request.data
        response.success = True

        if request.data:
            response.message = 'Emergency stop activated'
            self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
        else:
            response.message = 'Emergency stop deactivated'
            self.get_logger().info('Emergency stop deactivated')

        # Publish emergency status
        status_msg = Bool()
        status_msg.data = self.emergency_stopped
        self.emergency_status_pub.publish(status_msg)

        return response

    def calibrate_callback(self, request, response):
        """Handle calibration service request"""
        self.get_logger().info('Calibration requested')

        # Simulate calibration process
        import time
        self.get_logger().info('Starting calibration sequence...')

        # In a real implementation, this would calibrate all joints
        time.sleep(2)  # Simulate calibration time

        self.get_logger().info('Calibration complete')

        response.success = True
        response.message = 'Calibration completed successfully'

        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = HumanoidServiceNode()

    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        pass
    finally:
        service_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Action-Based Communication

Actions are perfect for long-running operations that require goal setting, feedback, and result reporting.

### Action Implementation

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from humanoid_robot_msgs.action import WalkToPose
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import threading
import time
import math

class HumanoidActionServer(Node):
    """
    Action server for humanoid robot walking to pose
    """
    def __init__(self):
        super().__init__('humanoid_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            WalkToPose,
            'walk_to_pose',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # Publisher for walking status
        self.walk_status_pub = self.create_publisher(Float64, '/humanoid_robot/walk_progress', 10)

        self.get_logger().info('Humanoid Action Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject goal request"""
        # Accept all goals for now
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the walking action"""
        self.get_logger().info('Executing walk to pose action...')

        # Get goal pose
        target_pose = goal_handle.request.target_pose
        self.get_logger().info(f'Walking to pose: ({target_pose.position.x}, {target_pose.position.y})')

        # Create result message
        result = WalkToPose.Result()
        result.success = False
        result.message = 'Walk failed'

        # Simulate walking progress
        feedback_msg = WalkToPose.Feedback()

        try:
            # Simulate walking to target (in a real implementation, this would control the robot)
            total_steps = 50
            for i in range(total_steps):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Walk cancelled'
                    return result

                # Simulate progress
                progress = float(i + 1) / total_steps
                feedback_msg.current_pose.position.x = target_pose.position.x * progress
                feedback_msg.current_pose.position.y = target_pose.position.y * progress
                feedback_msg.progress = progress * 100.0

                # Publish progress
                progress_msg = Float64()
                progress_msg.data = progress * 100.0
                self.walk_status_pub.publish(progress_msg)

                # Publish feedback
                goal_handle.publish_feedback(feedback_msg)

                # Sleep to simulate walking
                time.sleep(0.1)

            # If we reach here, the walk is complete
            goal_handle.succeed()
            result.success = True
            result.message = 'Successfully walked to target pose'

        except Exception as e:
            self.get_logger().error(f'Error during walk execution: {e}')
            goal_handle.abort()
            result.success = False
            result.message = f'Walk failed: {str(e)}'

        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = HumanoidActionServer()

    try:
        # Use multi-threaded executor to handle multiple goals
        executor = MultiThreadedExecutor()
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quality of Service (QoS) Considerations

For humanoid robotics applications, appropriate QoS settings are crucial:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# For sensor data (high frequency, best effort)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE
)

# For control commands (critical, reliable)
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

# For configuration parameters (persistent)
param_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)
```

## Practical Exercise: Communication Pattern Implementation

1. Implement a publisher for joint states with proper QoS settings
2. Create a subscriber that processes sensor data and publishes control commands
3. Develop a service for robot calibration
4. Implement an action for complex movement sequences
5. Test communication reliability under various network conditions

## Summary

This week, we've explored the three main ROS2 communication patterns essential for humanoid robotics:

- **Topics**: For continuous data streams like sensor readings and robot states
- **Services**: For synchronous operations like calibration and configuration
- **Actions**: For long-running tasks like navigation and complex movements

Proper use of these communication patterns, along with appropriate QoS settings, is critical for building robust and responsive humanoid robot systems.