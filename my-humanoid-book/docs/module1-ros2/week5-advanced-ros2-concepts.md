---
sidebar_position: 3
---

# Week 5: Advanced ROS2 Concepts for Humanoid Robotics

## Learning Objectives

By the end of this week, you will be able to:
- Implement advanced ROS2 features for complex humanoid systems
- Design and use custom message and service definitions
- Apply parameter management and dynamic reconfiguration
- Implement lifecycle nodes for robust robot state management
- Optimize performance and handle real-time constraints

## Custom Message and Service Definitions

For humanoid robotics applications, custom message types are essential to represent complex robot-specific data structures.

### Creating Custom Messages

First, let's define a custom message for humanoid joint states:

```c
# humanoid_robot_msgs/msg/HumanoidJointState.msg
# Custom message for humanoid robot joint states with additional safety information

std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
float64[] safety_limits
bool[] limits_exceeded
float64[] commanded_position
float64[] commanded_velocity
```

### Creating Custom Services

For humanoid-specific operations, we might need custom services:

```c
# humanoid_robot_msgs/srv/ConfigureWalk.srv
# Service to configure walking parameters for humanoid robot

# Request
float64 step_height
float64 step_length
float64 step_duration
float64 balance_threshold
string gait_type

# Response
bool success
string message
float64 actual_step_height
float64 actual_step_length
```

### Creating Custom Actions

For complex humanoid behaviors, custom actions provide the necessary flexibility:

```c
# humanoid_robot_msgs/action/Balance.action
# Action for humanoid balance control

# Goal
float64 target_com_x
float64 target_com_y
float64 max_time
bool enable_logging

# Result
bool success
string message
float64 final_com_x
float64 final_com_y
float64 balance_score

# Feedback
float64 current_com_x
float64 current_com_y
float64 balance_error
float64 progress_percentage
```

## Lifecycle Nodes for Robust State Management

Lifecycle nodes provide better state management for complex humanoid systems:

```python
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from std_msgs.msg import String

class HumanoidLifecycleNode(LifecycleNode):
    """
    Lifecycle node for humanoid robot with proper state management
    """
    def __init__(self):
        super().__init__('humanoid_lifecycle_node')
        self.get_logger().info('Humanoid Lifecycle Node initialized')

    def on_configure(self, state):
        """Configure the node"""
        self.get_logger().info(f'Configuring node, current state: {state}')

        # Create publishers (but they won't be active until activated)
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)

        # Initialize internal variables
        self.counter = 0

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """Activate the node"""
        self.get_logger().info(f'Activating node, current state: {state}')

        # Activate the publisher
        self.pub.on_activate()

        # Create timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Deactivate the node"""
        self.get_logger().info(f'Deactivating node, current state: {state}')

        # Deactivate publisher
        self.pub.on_deactivate()

        # Destroy timer
        self.timer.destroy()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Clean up the node"""
        self.get_logger().info(f'Cleaning up node, current state: {state}')

        # Destroy publisher
        self.destroy_publisher(self.pub)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        """Shutdown the node"""
        self.get_logger().info(f'Shutting down node, current state: {state}')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        """Handle errors"""
        self.get_logger().error(f'Error occurred, current state: {state}')
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        """Timer callback for periodic publishing"""
        msg = String()
        msg.data = f'Lifecycle message #{self.counter}'
        self.counter += 1

        # Publish only if the publisher is active
        if self.pub.is_activated:
            self.pub.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    # Create the lifecycle node
    node = HumanoidLifecycleNode()

    # Transition the node through its lifecycle
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameter Management and Dynamic Reconfiguration

Proper parameter management is crucial for humanoid robots that need to adapt to different conditions:

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float64

class HumanoidParameterNode(Node):
    """
    Node demonstrating advanced parameter management for humanoid robot
    """
    def __init__(self):
        super().__init__('humanoid_parameter_node')

        # Declare parameters with descriptors
        self.declare_parameter(
            'control_loop_rate',
            100,  # Default value: 100 Hz
            ParameterDescriptor(
                name='control_loop_rate',
                type=ParameterType.PARAMETER_INTEGER,
                description='Control loop rate in Hz',
                additional_constraints='Must be between 10 and 1000 Hz',
                integer_range=[ParameterDescriptor.INTEGER_RANGE().from_dict({
                    'from_value': 10,
                    'to_value': 1000,
                    'step': 1
                })],
                read_only=False
            )
        )

        self.declare_parameter(
            'safety_limits.max_joint_velocity',
            2.0,
            ParameterDescriptor(
                name='safety_limits.max_joint_velocity',
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum joint velocity limit in rad/s',
                floating_point_range=[ParameterDescriptor.FloatingPointRange().from_dict({
                    'from_value': 0.1,
                    'to_value': 10.0,
                    'step': 0.1
                })]
            )
        )

        self.declare_parameter(
            'walking_parameters.step_height',
            0.05,
            ParameterDescriptor(
                name='walking_parameters.step_height',
                type=ParameterType.PARAMETER_DOUBLE,
                description='Default step height in meters',
                floating_point_range=[ParameterDescriptor.FloatingPointRange().from_dict({
                    'from_value': 0.01,
                    'to_value': 0.2,
                    'step': 0.01
                })]
            )
        )

        # Create publisher for parameter changes
        self.param_change_pub = self.create_publisher(
            String, '/humanoid_robot/parameter_changes', 10)

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timer for periodic parameter usage
        self.timer = self.create_timer(0.1, self.use_parameters)

        self.get_logger().info('Humanoid Parameter Node initialized')

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')

            # Publish parameter change notification
            change_msg = String()
            change_msg.data = f'Parameter {param.name} changed to {param.value}'
            self.param_change_pub.publish(change_msg)

        return SetParametersResult(successful=True)

    def use_parameters(self):
        """Use parameters in robot control"""
        # Get current parameter values
        control_rate = self.get_parameter('control_loop_rate').value
        max_vel = self.get_parameter('safety_limits.max_joint_velocity').value
        step_height = self.get_parameter('walking_parameters.step_height').value

        # Use parameters in control logic
        self.get_logger().debug(f'Using parameters - Rate: {control_rate}Hz, Max Vel: {max_vel}rad/s, Step: {step_height}m')

def main(args=None):
    rclpy.init(args=args)
    param_node = HumanoidParameterNode()

    try:
        rclpy.spin(param_node)
    except KeyboardInterrupt:
        pass
    finally:
        param_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-Time Performance Considerations

For humanoid robots, real-time performance is critical. Here's how to optimize for real-time behavior:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from threading import Lock
import time
import numpy as np

class HumanoidRealtimeNode(Node):
    """
    Node optimized for real-time performance in humanoid robotics
    """
    def __init__(self):
        super().__init__('humanoid_realtime_node')

        # Use dedicated callback group for time-critical operations
        self.rt_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # High-frequency publishers with optimized QoS
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/humanoid_robot/joint_commands',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ),
            callback_group=self.rt_callback_group
        )

        # High-frequency subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid_robot/imu/data',
            self.imu_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            ),
            callback_group=self.rt_callback_group
        )

        # Real-time control loop
        self.control_timer = self.create_timer(
            0.001,  # 1000 Hz control loop
            self.realtime_control_loop,
            callback_group=self.rt_callback_group
        )

        # Data synchronization
        self.data_lock = Lock()
        self.imu_data = None
        self.last_update_time = self.get_clock().now()

        # Performance monitoring
        self.loop_times = []
        self.loop_count = 0

        self.get_logger().info('Humanoid Real-time Node initialized')

    def imu_callback(self, msg):
        """High-frequency IMU data callback"""
        with self.data_lock:
            self.imu_data = msg
            self.last_update_time = self.get_clock().now()

    def realtime_control_loop(self):
        """Time-critical control loop"""
        start_time = time.perf_counter()

        with self.data_lock:
            if self.imu_data is not None:
                # Perform time-critical control calculations
                self.perform_control_update(self.imu_data)

        # Monitor loop performance
        end_time = time.perf_counter()
        loop_time = end_time - start_time
        self.loop_times.append(loop_time)

        # Log performance if needed
        self.loop_count += 1
        if self.loop_count % 1000 == 0:  # Every 1000 iterations
            avg_time = np.mean(self.loop_times)
            max_time = np.max(self.loop_times)
            self.get_logger().info(f'Control loop - Avg: {avg_time*1000:.2f}ms, Max: {max_time*1000:.2f}ms')
            self.loop_times = []  # Reset for next measurement period

    def perform_control_update(self, imu_data):
        """Perform actual control calculations"""
        # Example: Balance control based on IMU data
        roll = self.quaternion_to_roll_pitch_yaw(imu_data.orientation)[0]
        pitch = self.quaternion_to_roll_pitch_yaw(imu_data.orientation)[1]

        # Simple PD controller for balance
        kp = 10.0  # Proportional gain
        kd = 1.0   # Derivative gain

        # Calculate control effort (simplified)
        roll_control = -kp * roll - kd * imu_data.angular_velocity.x
        pitch_control = -kp * pitch - kd * imu_data.angular_velocity.y

        # Generate joint commands based on control effort
        joint_commands = self.generate_joint_commands(roll_control, pitch_control)

        # Publish commands
        self.publish_joint_commands(joint_commands)

    def quaternion_to_roll_pitch_yaw(self, q):
        """Convert quaternion to roll, pitch, yaw"""
        import math
        # Simplified conversion (in practice, use tf_transformations or similar)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def generate_joint_commands(self, roll_control, pitch_control):
        """Generate joint commands from balance control"""
        # Simplified joint command generation
        # In practice, this would involve complex inverse kinematics
        commands = {
            'left_hip_roll': roll_control * 0.5,
            'right_hip_roll': -roll_control * 0.5,
            'left_ankle_pitch': pitch_control * 0.3,
            'right_ankle_pitch': pitch_control * 0.3
        }
        return commands

    def publish_joint_commands(self, commands):
        """Publish joint commands with proper header timing"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        for joint_name, position in commands.items():
            msg.name.append(joint_name)
            msg.position.append(position)

        self.joint_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # For real-time performance, consider using real-time scheduler
    # and appropriate QoS settings

    realtime_node = HumanoidRealtimeNode()

    try:
        rclpy.spin(realtime_node)
    except KeyboardInterrupt:
        pass
    finally:
        realtime_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Node Coordination and Synchronization

For complex humanoid systems, multiple nodes need to coordinate effectively:

```python
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Bool, Float64
import threading
import time

class HumanoidCoordinatorNode(Node):
    """
    Coordinator node for managing multiple humanoid robot subsystems
    """
    def __init__(self):
        super().__init__('humanoid_coordinator')

        # State tracking
        self.motor_ready = False
        self.sensors_ready = False
        self.control_enabled = False

        # Publishers for coordination
        self.enable_pub = self.create_publisher(Bool, '/humanoid_robot/enable_control', 10)
        self.sync_pub = self.create_publisher(TimeMsg, '/humanoid_robot/sync_time', 10)

        # Subscribers for system status
        self.motor_status_sub = self.create_subscription(
            Bool, '/humanoid_robot/motor_status', self.motor_status_callback, 10)

        self.sensor_status_sub = self.create_subscription(
            Bool, '/humanoid_robot/sensor_status', self.sensor_status_callback, 10)

        # Timer for coordination
        self.coordination_timer = self.create_timer(1.0, self.coordination_callback)

        # Thread for complex coordination tasks
        self.coordination_thread = threading.Thread(target=self.background_coordination)
        self.coordination_thread.daemon = True
        self.coordination_thread.start()

        self.get_logger().info('Humanoid Coordinator Node initialized')

    def motor_status_callback(self, msg):
        """Handle motor status updates"""
        self.motor_ready = msg.data
        self.get_logger().debug(f'Motor status: {self.motor_ready}')

    def sensor_status_callback(self, msg):
        """Handle sensor status updates"""
        self.sensors_ready = msg.data
        self.get_logger().debug(f'Sensor status: {self.sensors_ready}')

    def coordination_callback(self):
        """Periodic coordination tasks"""
        # Check overall system readiness
        system_ready = self.motor_ready and self.sensors_ready

        if system_ready and not self.control_enabled:
            # Enable control when all systems are ready
            self.enable_control()
        elif not system_ready and self.control_enabled:
            # Disable control if any system is not ready
            self.disable_control()

        # Publish synchronization time
        sync_time = self.get_clock().now().to_msg()
        self.sync_pub.publish(sync_time)

    def enable_control(self):
        """Enable robot control"""
        if not self.control_enabled:
            self.control_enabled = True
            enable_msg = Bool()
            enable_msg.data = True
            self.enable_pub.publish(enable_msg)
            self.get_logger().info('Control enabled - all systems ready')

    def disable_control(self):
        """Disable robot control"""
        if self.control_enabled:
            self.control_enabled = False
            enable_msg = Bool()
            enable_msg.data = False
            self.enable_pub.publish(enable_msg)
            self.get_logger().info('Control disabled - system not ready')

    def background_coordination(self):
        """Background coordination tasks"""
        while rclpy.ok():
            # Perform complex coordination tasks in background
            # This could include planning, optimization, etc.
            time.sleep(0.1)  # Prevent busy waiting

def main(args=None):
    rclpy.init(args=args)
    coordinator_node = HumanoidCoordinatorNode()

    try:
        rclpy.spin(coordinator_node)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Practical Exercise: Advanced ROS2 Implementation

1. Create custom message definitions for humanoid-specific data
2. Implement a lifecycle node for the main robot controller
3. Set up parameter management with validation
4. Optimize a control loop for real-time performance
5. Implement a coordinator node for multi-system management

## Summary



This week, we've covered advanced ROS2 concepts essential for complex humanoid robotics:



- **Custom messages, services, and actions** for robot-specific data

- **Lifecycle nodes** for robust state management

- **Parameter management** with validation and dynamic reconfiguration

- **Real-time performance optimization** for critical control loops

- **Multi-node coordination** for complex robot systems



These advanced concepts enable the development of sophisticated humanoid robot systems with proper state management, performance optimization, and robust communication patterns.



## Challenges and Extensions



- **Challenge 1:** Implement a ROS2 action server that allows you to command the robot to walk a certain distance or for a specific duration.

- **Challenge 2:** Create a more sophisticated real-time control loop that uses a PID controller to maintain the robot's balance.

- **Challenge 3:** Develop a launch file that starts all the nodes of your robot system in the correct order and with the correct parameters.
