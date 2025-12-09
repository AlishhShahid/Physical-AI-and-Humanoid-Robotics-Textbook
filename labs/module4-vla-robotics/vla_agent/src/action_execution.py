#!/usr/bin/env python3
"""
Action Execution and Motor Control Interface for VLA Robotics
Executes planned actions on the humanoid robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import JointState
from humanoid_robot_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math
from typing import Dict, List, Optional

class ActionExecutionNode(Node):
    """
    Action Execution and Motor Control node for humanoid robot
    Executes planned actions by controlling robot motors and actuators
    """
    def __init__(self):
        super().__init__('action_execution')

        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('cmd_vel_topic', '/humanoid_robot/cmd_vel')
        self.declare_parameter('joint_state_topic', '/humanoid_robot/joint_states')
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('action_execution_timeout', 30.0)  # seconds

        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.action_execution_timeout = self.get_parameter('action_execution_timeout').value

        # Current robot state
        self.current_pose = Pose()
        self.current_joint_states = JointState()
        self.is_executing = False

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create subscribers
        self.action_plan_sub = self.create_subscription(
            String, '/humanoid_robot/action_plan', self.action_plan_callback, 10)

        self.joint_state_sub = self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, '/humanoid_robot/execution_status', 10)

        # QoS profile for navigation
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.get_logger().info('Action Execution node initialized')

    def joint_state_callback(self, msg: JointState):
        """Update current joint states"""
        self.current_joint_states = msg

    def action_plan_callback(self, msg: String):
        """Execute action plan received from LLM planner"""
        if self.is_executing:
            self.get_logger().warn('Already executing an action plan, skipping new plan')
            return

        try:
            action_plan = eval(msg.data)  # Note: In production, use json.loads instead of eval
            if isinstance(action_plan, str):
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

    def execute_single_action(self, action: Dict) -> bool:
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

    def execute_navigation_to_location(self, action: Dict) -> bool:
        """Execute navigation to a specific location"""
        x = action.get('x', 0.0)
        y = action.get('y', 0.0)
        z = action.get('z', 0.0)  # For 3D navigation if needed

        self.get_logger().info(f'Navigating to location: ({x}, {y})')

        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
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

    def execute_move_relative(self, action: Dict) -> bool:
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

    def execute_turn(self, action: Dict) -> bool:
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

    def execute_grab_object(self, action: Dict) -> bool:
        """Execute object grabbing action"""
        object_type = action.get('object_type', 'object')
        arm = action.get('arm', 'right')  # 'left' or 'right'

        self.get_logger().info(f'Attempting to grab {object_type} with {arm} arm')

        # In a real implementation, this would control the robot's gripper
        # For simulation, we'll just log the action and simulate the movement
        self.simulate_arm_movement(arm, 'grab')

        time.sleep(1.0)  # Simulate grab time
        return True

    def execute_release_object(self, action: Dict) -> bool:
        """Execute object release action"""
        arm = action.get('arm', 'right')  # 'left' or 'right'

        self.get_logger().info(f'Releasing object with {arm} arm')

        # In a real implementation, this would control the robot's gripper
        # For simulation, we'll just log the action and simulate the movement
        self.simulate_arm_movement(arm, 'release')

        time.sleep(0.5)  # Simulate release time
        return True

    def simulate_arm_movement(self, arm: str, action: str):
        """Simulate arm movement for grab/release"""
        # In a real implementation, this would send joint commands to move the arm
        # For simulation, we'll just log what would happen
        self.get_logger().info(f'Simulating {action} action with {arm} arm')

    def execute_speak(self, action: Dict) -> bool:
        """Execute speech action"""
        message = action.get('message', 'Hello')

        self.get_logger().info(f'Speaking: {message}')

        # In a real implementation, this would use text-to-speech
        # For simulation, we'll just log the action
        return True

    def execute_wave_arm(self, action: Dict) -> bool:
        """Execute arm waving action"""
        arm = action.get('arm', 'right')  # 'left' or 'right'

        self.get_logger().info(f'Waving {arm} arm')

        # Simulate arm waving by moving joints
        self.simulate_arm_movement(arm, 'wave')

        time.sleep(1.0)  # Simulate wave time
        return True

    def execute_detect_objects(self, action: Dict) -> bool:
        """Execute object detection"""
        self.get_logger().info('Detecting objects in environment')

        # In a real implementation, this would trigger perception system
        # For simulation, we'll just log the action
        time.sleep(1.0)  # Simulate detection time
        return True

    def execute_wait(self, action: Dict) -> bool:
        """Execute wait action"""
        duration = action.get('duration', 1.0)

        self.get_logger().info(f'Waiting for {duration} seconds')

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            time.sleep(0.1)

        return True


def main(args=None):
    rclpy.init(args=args)
    execution_node = ActionExecutionNode()

    try:
        rclpy.spin(execution_node)
    except KeyboardInterrupt:
        pass
    finally:
        execution_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()