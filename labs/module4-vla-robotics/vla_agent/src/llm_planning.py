#!/usr/bin/env python3
"""
LLM-based Planning Module for VLA Robotics
Interprets voice commands and generates action sequences
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
from humanoid_robot_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, Point
import openai
from openai import OpenAI
import os
import json
import re
import time
from typing import List, Dict, Tuple

class LLMPlanningNode(Node):
    """
    LLM-based Planning node for humanoid robot
    Interprets voice commands and generates action sequences
    """
    def __init__(self):
        super().__init__('llm_planning')

        # Parameters
        self.declare_parameter('openai_model', 'gpt-4-turbo')
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('command_timeout', 30.0)

        self.openai_model = self.get_parameter('openai_model').value
        self.openai_api_key = self.get_parameter('openai_api_key').value
        self.max_retries = self.get_parameter('max_retries').value
        self.command_timeout = self.get_parameter('command_timeout').value

        # Set up OpenAI client
        if self.openai_api_key:
            self.client = OpenAI(api_key=self.openai_api_key)
        else:
            # Check for API key in environment variable
            api_key = os.getenv('OPENAI_API_KEY')
            if api_key:
                self.client = OpenAI(api_key=api_key)
            else:
                self.get_logger().warn('OpenAI API key not found. Planning will not work without it.')
                self.client = None

        # Action clients
        self.nav_client = None

        # Create subscribers
        self.voice_command_sub = self.create_subscription(
            String, '/humanoid_robot/interpreted_command', self.voice_command_callback, 10)

        # Create publishers
        self.action_plan_pub = self.create_publisher(String, '/humanoid_robot/action_plan', 10)
        self.status_pub = self.create_publisher(String, '/humanoid_robot/planning_status', 10)

        # Initialize action clients
        if self.client:
            self.get_logger().info('LLM Planning node initialized with OpenAI client')
        else:
            self.get_logger().warn('LLM Planning node initialized without OpenAI client - functionality limited')

    def voice_command_callback(self, msg):
        """Process voice command and generate action plan"""
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')

        if not self.client:
            self.get_logger().error('OpenAI client not available for planning')
            return

        try:
            # Generate action plan using LLM
            action_plan = self.generate_action_plan(command)

            if action_plan:
                # Publish action plan
                plan_msg = String()
                plan_msg.data = json.dumps(action_plan)
                self.action_plan_pub.publish(plan_msg)

                self.get_logger().info(f'Generated action plan: {action_plan}')

                # Execute the plan
                self.execute_action_plan(action_plan)
            else:
                self.get_logger().warn('No action plan generated for command')

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')

    def generate_action_plan(self, command: str) -> List[Dict]:
        """
        Generate action plan from voice command using LLM
        """
        if not self.client:
            return []

        # Define the context and possible actions for the humanoid robot
        system_prompt = """
        You are an action planning assistant for a humanoid robot. Your job is to interpret natural language commands and generate a sequence of actions for the robot to execute.

        Available actions:
        1. NAVIGATE_TO_LOCATION: Move the robot to a specific location (x, y coordinates)
        2. MOVE_RELATIVE: Move the robot relative to its current position (forward, backward, left, right, distance in meters)
        3. TURN: Rotate the robot (left, right, angle in degrees)
        4. GRAB_OBJECT: Attempt to grab an object at the current location
        5. RELEASE_OBJECT: Release a held object
        6. SPEAK: Make the robot speak a message
        7. WAVE_ARM: Wave the robot's arm
        8. DETECT_OBJECTS: Use perception to detect objects in the environment
        9. WAIT: Wait for a specified duration in seconds

        The output should be a JSON array of action objects, each with 'action_type' and relevant parameters.

        Examples:
        - "Move forward 2 meters" -> [{"action_type": "MOVE_RELATIVE", "direction": "forward", "distance": 2.0}]
        - "Go to the kitchen" -> [{"action_type": "NAVIGATE_TO_LOCATION", "x": 3.5, "y": -1.2}]
        - "Turn left and move forward" -> [{"action_type": "TURN", "direction": "left", "angle": 90}, {"action_type": "MOVE_RELATIVE", "direction": "forward", "distance": 1.0}]
        - "Pick up the red ball" -> [{"action_type": "DETECT_OBJECTS"}, {"action_type": "GRAB_OBJECT", "object_type": "red ball"}]

        Respond with only the JSON array, no other text.
        """

        user_prompt = f"Command: {command}"

        for attempt in range(self.max_retries):
            try:
                response = self.client.chat.completions.create(
                    model=self.openai_model,
                    messages=[
                        {"role": "system", "content": system_prompt},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=0.1,
                    max_tokens=500
                )

                response_text = response.choices[0].message.content.strip()

                # Extract JSON from response if it contains other text
                json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
                if json_match:
                    json_str = json_match.group()
                    action_plan = json.loads(json_str)
                    return action_plan
                else:
                    # Try to parse the entire response as JSON
                    try:
                        action_plan = json.loads(response_text)
                        return action_plan
                    except json.JSONDecodeError:
                        self.get_logger().warn(f'Could not parse LLM response as JSON: {response_text}')
                        return []

            except Exception as e:
                self.get_logger().warn(f'LLM request failed (attempt {attempt + 1}): {e}')
                if attempt == self.max_retries - 1:
                    # Final attempt failed
                    return []
                time.sleep(1)  # Wait before retry

        return []

    def execute_action_plan(self, action_plan: List[Dict]):
        """Execute the generated action plan"""
        for i, action in enumerate(action_plan):
            self.get_logger().info(f'Executing action {i+1}/{len(action_plan)}: {action["action_type"]}')

            success = False
            try:
                if action['action_type'] == 'NAVIGATE_TO_LOCATION':
                    success = self.execute_navigation(action)
                elif action['action_type'] == 'MOVE_RELATIVE':
                    success = self.execute_move_relative(action)
                elif action['action_type'] == 'TURN':
                    success = self.execute_turn(action)
                elif action['action_type'] == 'GRAB_OBJECT':
                    success = self.execute_grab_object(action)
                elif action['action_type'] == 'RELEASE_OBJECT':
                    success = self.execute_release_object(action)
                elif action['action_type'] == 'SPEAK':
                    success = self.execute_speak(action)
                elif action['action_type'] == 'WAVE_ARM':
                    success = self.execute_wave_arm(action)
                elif action['action_type'] == 'DETECT_OBJECTS':
                    success = self.execute_detect_objects(action)
                elif action['action_type'] == 'WAIT':
                    success = self.execute_wait(action)
                else:
                    self.get_logger().warn(f'Unknown action type: {action["action_type"]}')
                    success = True  # Consider unknown actions as successful to continue

            except Exception as e:
                self.get_logger().error(f'Error executing action {action["action_type"]}: {e}')
                success = False

            if not success:
                self.get_logger().error(f'Action failed: {action}')
                break

    def execute_navigation(self, action: Dict) -> bool:
        """Execute navigation to a specific location"""
        x = action.get('x', 0.0)
        y = action.get('y', 0.0)

        self.get_logger().info(f'Navigating to location: ({x}, {y})')

        # In a real implementation, this would call the navigation action
        # For simulation, we'll just log the action
        # nav_goal = NavigateToPose.Goal()
        # nav_goal.pose.position.x = x
        # nav_goal.pose.position.y = y
        # nav_goal.pose.orientation.z = 0.0  # Face forward
        # nav_goal.pose.orientation.w = 1.0

        # Send navigation goal and wait for result
        # result = self.nav_client.send_goal(nav_goal)

        # For now, simulate success
        time.sleep(1)  # Simulate navigation time
        return True

    def execute_move_relative(self, action: Dict) -> bool:
        """Execute relative movement"""
        direction = action.get('direction', 'forward')
        distance = action.get('distance', 1.0)

        self.get_logger().info(f'Moving {direction} {distance} meters')

        # In a real implementation, this would send velocity commands
        # For simulation, we'll just log the action
        time.sleep(0.5)  # Simulate movement time
        return True

    def execute_turn(self, action: Dict) -> bool:
        """Execute turn action"""
        direction = action.get('direction', 'left')
        angle = action.get('angle', 90)

        self.get_logger().info(f'Turning {direction} by {angle} degrees')

        # In a real implementation, this would send rotation commands
        # For simulation, we'll just log the action
        time.sleep(0.5)  # Simulate turn time
        return True

    def execute_grab_object(self, action: Dict) -> bool:
        """Execute object grabbing action"""
        object_type = action.get('object_type', 'object')

        self.get_logger().info(f'Attempting to grab {object_type}')

        # In a real implementation, this would coordinate with manipulation system
        # For simulation, we'll just log the action
        time.sleep(1)  # Simulate grab time
        return True

    def execute_release_object(self, action: Dict) -> bool:
        """Execute object release action"""
        self.get_logger().info('Releasing held object')

        # In a real implementation, this would coordinate with manipulation system
        # For simulation, we'll just log the action
        time.sleep(0.5)  # Simulate release time
        return True

    def execute_speak(self, action: Dict) -> bool:
        """Execute speech action"""
        message = action.get('message', 'Hello')

        self.get_logger().info(f'Speaking: {message}')

        # In a real implementation, this would use text-to-speech
        # For simulation, we'll just log the action
        return True

    def execute_wave_arm(self, action: Dict) -> bool:
        """Execute arm waving action"""
        self.get_logger().info('Waving arm')

        # In a real implementation, this would control arm joints
        # For simulation, we'll just log the action
        time.sleep(1)  # Simulate wave time
        return True

    def execute_detect_objects(self, action: Dict) -> bool:
        """Execute object detection"""
        self.get_logger().info('Detecting objects in environment')

        # In a real implementation, this would trigger perception system
        # For simulation, we'll just log the action
        time.sleep(1)  # Simulate detection time
        return True

    def execute_wait(self, action: Dict) -> bool:
        """Execute wait action"""
        duration = action.get('duration', 1.0)

        self.get_logger().info(f'Waiting for {duration} seconds')

        time.sleep(duration)
        return True


def main(args=None):
    rclpy.init(args=args)
    planning_node = LLMPlanningNode()

    try:
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()