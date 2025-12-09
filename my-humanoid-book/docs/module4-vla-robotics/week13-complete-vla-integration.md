---
sidebar_position: 2
---

# Week 13: Action Execution and Complete VLA Integration

## Learning Objectives

By the end of this week, you will be able to:
- Implement action execution systems for controlling humanoid robots
- Integrate vision, language, and action components into a complete system
- Test voice-commanded robot control in simulation environments
- Debug and optimize VLA system performance
- Evaluate the effectiveness of natural language robot control

## Action Execution and Motor Control

The action execution system is responsible for translating high-level action plans into low-level motor commands that control the humanoid robot's actuators and joints. This component bridges the gap between abstract planning and physical robot control.

### Action Execution Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from humanoid_robot_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time
import math

class ActionExecutionNode(Node):
    """
    Action Execution and Motor Control node for humanoid robot
    Executes planned actions by controlling robot motors and actuators
    """
    def __init__(self):
        super().__init__('action_execution')

        # Parameters
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('action_execution_timeout', 30.0)  # seconds

        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.action_execution_timeout = self.get_parameter('action_execution_timeout').value

        # Current robot state
        self.is_executing = False

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create subscribers
        self.action_plan_sub = self.create_subscription(
            String, '/humanoid_robot/action_plan', self.action_plan_callback, 10)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid_robot/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/humanoid_robot/execution_status', 10)

        self.get_logger().info('Action Execution node initialized')

    def action_plan_callback(self, msg: String):
        """Execute action plan received from LLM planner"""
        if self.is_executing:
            self.get_logger().warn('Already executing an action plan, skipping new plan')
            return

        try:
            import json
            action_plan = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Error parsing action plan: {e}')
            return

        self.get_logger().info(f'Executing action plan with {len(action_plan)} actions')

        self.is_executing = True
        success = True

        for i, action in enumerate(action_plan):
            self.get_logger().info(f'Executing action {i+1}/{len(action_plan)}: {action.get("action_type", "unknown")}')

            action_success = self.execute_single_action(action)
            if not action_success:
                self.get_logger().error(f'Action failed: {action}')
                success = False
                break

            # Small delay between actions
            time.sleep(0.1)

        self.is_executing = False

        # Publish execution status
        status_msg = String()
        status_msg.data = f"Plan execution {'succeeded' if success else 'failed'}"
        self.status_pub.publish(status_msg)

        if success:
            self.get_logger().info('Action plan completed successfully')
        else:
            self.get_logger().error('Action plan execution failed')

    def execute_single_action(self, action: dict) -> bool:
        """Execute a single action from the plan"""
        action_type = action.get('action_type', '').upper()

        if action_type == 'NAVIGATE_TO_LOCATION':
            return self.execute_navigation_to_location(action)
        elif action_type == 'MOVE_RELATIVE':
            return self.execute_move_relative(action)
        elif action_type == 'TURN':
            return self.execute_turn(action)
        elif action_type == 'GRAB_OBJECT':
            return self.execute_grab_object(action)
        elif action_type == 'RELEASE_OBJECT':
            return self.execute_release_object(action)
        elif action_type == 'SPEAK':
            return self.execute_speak(action)
        elif action_type == 'WAVE_ARM':
            return self.execute_wave_arm(action)
        elif action_type == 'DETECT_OBJECTS':
            return self.execute_detect_objects(action)
        elif action_type == 'WAIT':
            return self.execute_wait(action)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return True  # Don't fail on unknown actions

    def execute_navigation_to_location(self, action: dict) -> bool:
        """Execute navigation to a specific location"""
        x = action.get('x', 0.0)
        y = action.get('y', 0.0)

        self.get_logger().info(f'Navigating to location: ({x}, {y})')

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        # Send navigation goal
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.action_execution_timeout)

        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Navigation goal accepted, waiting for result...')
                get_result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=self.action_execution_timeout)

                if get_result_future.result() is not None:
                    result = get_result_future.result().result
                    self.get_logger().info(f'Navigation completed with result: {result}')
                    return True
                else:
                    self.get_logger().error('Navigation result future was not completed')
                    return False
            else:
                self.get_logger().error('Navigation goal was rejected')
                return False
        else:
            self.get_logger().error('Navigation goal future was not completed')
            return False

    def execute_move_relative(self, action: dict) -> bool:
        """Execute relative movement"""
        direction = action.get('direction', 'forward')
        distance = action.get('distance', 1.0)
        speed = action.get('speed', self.max_linear_velocity)

        self.get_logger().info(f'Moving {direction} {distance} meters')

        # Determine velocity based on direction
        vel_x = 0.0
        vel_y = 0.0

        if direction == 'forward':
            vel_x = min(speed, self.max_linear_velocity)
        elif direction == 'backward':
            vel_x = -min(speed, self.max_linear_velocity)
        elif direction == 'left':
            vel_y = min(speed, self.max_linear_velocity)
        elif direction == 'right':
            vel_y = -min(speed, self.max_linear_velocity)
        else:
            self.get_logger().warn(f'Unknown direction: {direction}, defaulting to forward')
            vel_x = min(speed, self.max_linear_velocity)

        # Calculate time to move the specified distance
        move_time = abs(distance) / speed if speed > 0 else 0.1

        # Execute movement
        cmd_vel = Twist()
        cmd_vel.linear.x = vel_x
        cmd_vel.linear.y = vel_y
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0  # No rotation during linear movement

        start_time = time.time()
        while time.time() - start_time < move_time and rclpy.ok():
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.05)  # 20 Hz control loop

        # Stop the robot
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        return True

    def execute_turn(self, action: dict) -> bool:
        """Execute turn action"""
        direction = action.get('direction', 'left')
        angle_deg = action.get('angle', 90)
        angular_speed = action.get('angular_speed', self.max_angular_velocity)

        self.get_logger().info(f'Turning {direction} by {angle_deg} degrees')

        # Convert angle to radians
        angle_rad = math.radians(abs(angle_deg))

        # Determine angular velocity based on direction
        if direction == 'left':
            angular_vel = min(angular_speed, self.max_angular_velocity)
        elif direction == 'right':
            angular_vel = -min(angular_speed, self.max_angular_velocity)
        else:
            self.get_logger().warn(f'Unknown direction: {direction}, defaulting to left')
            angular_vel = min(angular_speed, self.max_angular_velocity)

        # Calculate time to turn the specified angle
        turn_time = angle_rad / abs(angular_vel) if angular_vel != 0 else 0.1

        # Execute turn
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = angular_vel

        start_time = time.time()
        while time.time() - start_time < turn_time and rclpy.ok():
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.05)  # 20 Hz control loop

        # Stop the robot
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        return True
```

## Complete VLA Agent Integration

The VLA agent integrates all components into a cohesive system that can receive voice commands and execute them on the humanoid robot:

```python
class VLAAgentNode(Node):
    """
    Vision-Language-Action Agent node for humanoid robot
    Integrates voice recognition, LLM planning, and action execution
    """
    def __init__(self):
        super().__init__('vla_agent')

        # Parameters
        self.declare_parameter('enable_vision', True)
        self.declare_parameter('enable_language', True)
        self.declare_parameter('enable_action', True)
        self.declare_parameter('agent_frequency', 10.0)  # Hz

        self.enable_vision = self.get_parameter('enable_vision').value
        self.enable_language = self.get_parameter('enable_language').value
        self.enable_action = self.get_parameter('enable_action').value
        self.agent_frequency = self.get_parameter('agent_frequency').value

        # Agent state
        self.current_command = None
        self.current_plan = None
        self.is_executing = False
        self.agent_active = True

        # Create action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create subscribers
        self.voice_command_sub = self.create_subscription(
            String, '/humanoid_robot/voice_command', self.voice_command_callback, 10)

        self.interpreted_command_sub = self.create_subscription(
            String, '/humanoid_robot/interpreted_command', self.interpreted_command_callback, 10)

        self.action_plan_sub = self.create_subscription(
            String, '/humanoid_robot/action_plan', self.action_plan_callback, 10)

        self.status_sub = self.create_subscription(
            String, '/humanoid_robot/execution_status', self.execution_status_callback, 10)

        # Create publishers
        self.status_pub = self.create_publisher(String, '/humanoid_robot/vla_agent_status', 10)
        self.response_pub = self.create_publisher(String, '/humanoid_robot/vla_agent_response', 10)

        # Create timer for agent loop
        self.agent_timer = self.create_timer(1.0 / self.agent_frequency, self.agent_loop)

        self.get_logger().info('VLA Agent node initialized and ready')

    def voice_command_callback(self, msg: String):
        """Handle voice command input"""
        self.get_logger().info(f'Received voice command: {msg.data}')
        self.current_command = msg.data

        # Publish status update
        status_msg = String()
        status_msg.data = f'Received command: {msg.data}'
        self.status_pub.publish(status_msg)

    def interpreted_command_callback(self, msg: String):
        """Handle interpreted command from LLM planner"""
        try:
            import json
            command_data = json.loads(msg.data)
            self.get_logger().info(f'Received interpreted command: {command_data}')
        except json.JSONDecodeError:
            self.get_logger().info(f'Received interpreted command: {msg.data}')

    def action_plan_callback(self, msg: String):
        """Handle action plan from LLM planner"""
        try:
            import json
            plan_data = json.loads(msg.data)
            self.get_logger().info(f'Received action plan with {len(plan_data) if isinstance(plan_data, list) else "unknown"} actions')
            self.current_plan = plan_data
        except json.JSONDecodeError:
            self.get_logger().warn(f'Could not parse action plan as JSON: {msg.data}')

    def execution_status_callback(self, msg: String):
        """Handle execution status updates"""
        self.get_logger().info(f'Execution status: {msg.data}')

        # Publish response to user
        response_msg = String()
        response_msg.data = f'Command execution: {msg.data}'
        self.response_pub.publish(response_msg)

    def agent_loop(self):
        """Main agent loop that coordinates VLA components"""
        if not self.agent_active:
            return

        # Check if we have a plan to execute
        if self.current_plan and not self.is_executing:
            self.get_logger().info(f'Executing plan with {len(self.current_plan) if isinstance(self.current_plan, list) else 0} actions')
            self.execute_plan(self.current_plan)
            self.current_plan = None

    def execute_plan(self, plan: list):
        """Execute a plan of actions"""
        if not plan or not isinstance(plan, list):
            self.get_logger().warn('Invalid plan provided for execution')
            return

        self.is_executing = True
        self.get_logger().info(f'Starting execution of {len(plan)} actions')

        # In a real implementation, this would coordinate with the action execution system
        for i, action in enumerate(plan):
            self.get_logger().info(f'Executing action {i+1}/{len(plan)}: {action.get("action_type", "unknown")}')
            # In a real system, this would trigger actual robot actions
            time.sleep(0.2)

        self.is_executing = False
        self.get_logger().info('Plan execution completed')

        # Publish completion status
        status_msg = String()
        status_msg.data = f'Plan with {len(plan)} actions completed successfully'
        self.status_pub.publish(status_msg)
```

## VLA System Launch and Configuration

The complete VLA system is launched using a ROS2 launch file that brings up all components:

```python
# vla_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'openai_api_key',
            default_value='',
            description='OpenAI API key for LLM planning'
        ),

        # VLA Agent main node
        Node(
            package='vla_agent',
            executable='vla_agent',
            name='vla_agent',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'agent_name': 'humanoid_vla_agent'},
                {'enable_vision': True},
                {'enable_language': True},
                {'enable_action': True},
                {'agent_frequency': 10.0},
            ]
        ),

        # Voice command interface node
        Node(
            package='vla_agent',
            executable='voice_interface',
            name='voice_command_interface',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'sample_rate': 16000},
                {'recording_duration': 5.0},
                {'openai_api_key': LaunchConfiguration('openai_api_key')},
            ]
        ),

        # LLM planning node
        Node(
            package='vla_agent',
            executable='llm_planner',
            name='llm_planning',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'openai_model': 'gpt-4-turbo'},
                {'openai_api_key': LaunchConfiguration('openai_api_key')},
                {'max_retries': 3},
            ]
        ),

        # Action execution node
        Node(
            package='vla_agent',
            executable='action_executor',
            name='action_execution',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'max_linear_velocity': 0.5},
                {'max_angular_velocity': 1.0},
            ]
        )
    ])
```

## Testing the Complete VLA System

To test the complete VLA system, follow these steps:

1. **Launch the simulation environment** with the humanoid robot model
2. **Start the VLA system** using the launch file
3. **Issue voice commands** such as "move forward 2 meters" or "turn left"
4. **Observe robot behavior** and verify that actions are executed correctly
5. **Monitor system status** through the various ROS2 topics

### Example Voice Commands

The system can interpret and execute various types of commands:

- **Navigation**: "Go to the kitchen", "Move forward 2 meters"
- **Rotation**: "Turn left 90 degrees", "Face the door"
- **Manipulation**: "Pick up the red ball", "Wave your arm"
- **Complex**: "Go to the table and pick up the cup"

## Performance Optimization and Debugging

### Common Issues and Solutions

1. **Voice Recognition Issues**:
   - Poor audio quality: Check microphone settings and environment noise
   - API limits: Implement rate limiting and caching
   - Network issues: Configure local fallback recognition

2. **LLM Planning Issues**:
   - Incorrect action sequences: Refine system prompts
   - API costs: Implement caching for common commands
   - Response time: Optimize prompt structure

3. **Action Execution Issues**:
   - Robot not responding: Check ROS2 topic connections
   - Inaccurate movements: Calibrate motor control parameters
   - Timeout errors: Adjust action execution timeouts

### System Monitoring

Monitor the VLA system using ROS2 tools:

```bash
# Monitor topics
ros2 topic echo /humanoid_robot/voice_command
ros2 topic echo /humanoid_robot/action_plan
ros2 topic echo /humanoid_robot/execution_status

# Check system performance
ros2 run top top
ros2 run plotjuggler plotjuggler
```

## Practical Exercise: Complete VLA Integration

1. Launch the complete VLA system in Gazebo simulation
2. Issue various voice commands to the robot
3. Observe how the system processes commands and executes actions
4. Test error handling when commands fail or time out
5. Evaluate the naturalness and effectiveness of voice interaction

## Summary



This week, we've completed the Vision-Language-Action robotics system by implementing:



- Action execution and motor control systems

- Complete VLA agent that integrates all components

- Launch configuration for the full system

- Testing and debugging procedures for VLA systems



The VLA system represents a significant advancement in human-robot interaction, enabling natural language control of complex humanoid robots. This system forms the foundation for more advanced agentic behaviors and autonomous capabilities in future robotics applications.



With this, we've completed all four modules of our humanoid robotics textbook, covering from basic ROS2 communication to advanced AI-powered control systems. The robot now has:

- A nervous system (ROS2) for communication

- A digital twin (Gazebo/Unity) for simulation

- An AI brain (Isaac) for perception and navigation

- VLA capabilities for natural interaction



This comprehensive system provides a solid foundation for advanced humanoid robotics research and development.



## Challenges and Extensions



- **Challenge 1:** Implement a more sophisticated dialogue system that allows the robot to ask for clarification if a command is ambiguous.

- **Challenge 2:** Add memory to the VLA agent, so it can remember previous commands and objects it has seen.

- **Challenge 3:** Extend the action execution system to support more complex actions, such as opening doors or manipulating objects with two hands.
