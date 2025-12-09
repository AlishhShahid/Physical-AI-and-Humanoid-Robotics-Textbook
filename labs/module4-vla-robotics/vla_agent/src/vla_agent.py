#!/usr/bin/env python3
"""
Vision-Language-Action (VLA) Agent for Humanoid Robotics
Main integration node that connects voice, language, and action systems
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist, Pose
from humanoid_robot_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import time
from typing import Dict, List, Optional
import json

class VLAAgentNode(Node):
    """
    Vision-Language-Action Agent node for humanoid robot
    Integrates voice recognition, LLM planning, and action execution
    """
    def __init__(self):
        super().__init__('vla_agent')

        # Parameters
        self.declare_parameter('agent_name', 'humanoid_vla_agent')
        self.declare_parameter('enable_vision', True)
        self.declare_parameter('enable_language', True)
        self.declare_parameter('enable_action', True)
        self.declare_parameter('agent_frequency', 10.0)  # Hz
        self.declare_parameter('command_timeout', 60.0)  # seconds

        self.agent_name = self.get_parameter('agent_name').value
        self.enable_vision = self.get_parameter('enable_vision').value
        self.enable_language = self.get_parameter('enable_language').value
        self.enable_action = self.get_parameter('enable_action').value
        self.agent_frequency = self.get_parameter('agent_frequency').value
        self.command_timeout = self.get_parameter('command_timeout').value

        # Agent state
        self.current_command = None
        self.current_plan = None
        self.is_executing = False
        self.agent_active = True

        # Create action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.voice_command_sub = self.create_subscription(
            String, '/humanoid_robot/voice_command', self.voice_command_callback, 10)

        self.interpreted_command_sub = self.create_subscription(
            String, '/humanoid_robot/interpreted_command', self.interpreted_command_callback, 10)

        self.action_plan_sub = self.create_subscription(
            String, '/humanoid_robot/action_plan', self.action_plan_callback, 10)

        self.status_sub = self.create_subscription(
            String, '/humanoid_robot/execution_status', self.execution_status_callback, 10)

        if self.enable_vision:
            self.camera_sub = self.create_subscription(
                Image, '/humanoid_robot/camera/image_raw', self.camera_callback, qos_profile)

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
            command_data = json.loads(msg.data)
            self.get_logger().info(f'Received interpreted command: {command_data}')
        except json.JSONDecodeError:
            self.get_logger().info(f'Received interpreted command: {msg.data}')

    def action_plan_callback(self, msg: String):
        """Handle action plan from LLM planner"""
        try:
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

    def camera_callback(self, msg: Image):
        """Handle camera input for vision processing"""
        if self.enable_vision:
            # In a real implementation, this would process the camera image
            # For now, just log that we received an image
            self.get_logger().debug(f'Received camera image: {msg.width}x{msg.height}')

    def agent_loop(self):
        """Main agent loop that coordinates VLA components"""
        if not self.agent_active:
            return

        # Check if we have a command to process
        if self.current_command and not self.is_executing:
            self.get_logger().info(f'Processing command: {self.current_command}')

            # Reset current command after processing
            self.current_command = None

        # Check if we have a plan to execute
        if self.current_plan and not self.is_executing:
            self.get_logger().info(f'Executing plan with {len(self.current_plan) if isinstance(self.current_plan, list) else 0} actions')
            self.execute_plan(self.current_plan)
            self.current_plan = None

    def execute_plan(self, plan: List[Dict]):
        """Execute a plan of actions"""
        if not plan or not isinstance(plan, list):
            self.get_logger().warn('Invalid plan provided for execution')
            return

        self.is_executing = True
        self.get_logger().info(f'Starting execution of {len(plan)} actions')

        # In a real implementation, this would coordinate with the action execution system
        # For now, we'll just log the plan
        for i, action in enumerate(plan):
            self.get_logger().info(f'Executing action {i+1}/{len(plan)}: {action.get("action_type", "unknown")}')
            # Simulate action execution time
            time.sleep(0.2)

        self.is_executing = False
        self.get_logger().info('Plan execution completed')

        # Publish completion status
        status_msg = String()
        status_msg.data = f'Plan with {len(plan)} actions completed successfully'
        self.status_pub.publish(status_msg)

    def start_agent(self):
        """Start the VLA agent"""
        self.agent_active = True
        self.get_logger().info('VLA Agent started')

    def stop_agent(self):
        """Stop the VLA agent"""
        self.agent_active = False
        self.get_logger().info('VLA Agent stopped')

    def destroy_node(self):
        """Clean up resources"""
        self.stop_agent()
        super().destroy_node()


class VLAMainNode(Node):
    """
    Main VLA node that manages all VLA components
    """
    def __init__(self):
        super().__init__('vla_main')

        # Create all VLA components
        self.voice_node = None
        self.planning_node = None
        self.execution_node = None
        self.agent_node = VLAAgentNode()

        # Start all components in separate threads
        self.voice_thread = None
        self.planning_thread = None
        self.execution_thread = None
        self.agent_thread = None

        self.get_logger().info('VLA Main node initialized')

    def start_all_components(self):
        """Start all VLA components"""
        self.get_logger().info('Starting all VLA components...')

        # Initialize nodes
        if self.agent_node.enable_language:
            from llm_planning import LLMPlanningNode
            self.planning_node = LLMPlanningNode()

        if self.agent_node.enable_action:
            from action_execution import ActionExecutionNode
            self.execution_node = ActionExecutionNode()

        # Start nodes in separate threads
        if self.planning_node:
            self.planning_thread = threading.Thread(target=self._run_planning_node, daemon=True)
            self.planning_thread.start()

        if self.execution_node:
            self.execution_thread = threading.Thread(target=self._run_execution_node, daemon=True)
            self.execution_thread.start()

        # Start main agent
        self.agent_thread = threading.Thread(target=self._run_agent_node, daemon=True)
        self.agent_thread.start()

        self.get_logger().info('All VLA components started')

    def _run_planning_node(self):
        """Run the planning node in a separate thread"""
        try:
            rclpy.spin(self.planning_node)
        except Exception as e:
            self.get_logger().error(f'Planning node error: {e}')

    def _run_execution_node(self):
        """Run the execution node in a separate thread"""
        try:
            rclpy.spin(self.execution_node)
        except Exception as e:
            self.get_logger().error(f'Execution node error: {e}')

    def _run_agent_node(self):
        """Run the agent node in a separate thread"""
        try:
            rclpy.spin(self.agent_node)
        except Exception as e:
            self.get_logger().error(f'Agent node error: {e}')

    def destroy_node(self):
        """Clean up all components"""
        if self.planning_node:
            self.planning_node.destroy_node()
        if self.execution_node:
            self.execution_node.destroy_node()
        if self.agent_node:
            self.agent_node.destroy_node()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Create and start the main VLA system
    vla_main = VLAMainNode()
    vla_main.start_all_components()

    try:
        # Keep the main thread alive
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        vla_main.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()