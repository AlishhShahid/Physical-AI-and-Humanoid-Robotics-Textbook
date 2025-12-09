#!/usr/bin/env python3
"""
Tests for the Vision-Language-Action (VLA) agent
These tests verify that the VLA system works correctly for voice-commanded control
"""

import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# Add the vla_agent module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../vla_agent/src'))

# Import the VLA agent components
from vla_agent import VLAAgentNode, VLAMainNode
from llm_planning import LLMPlanningNode
from action_execution import ActionExecutionNode
from voice_command_interface import VoiceCommandInterface


class TestVLAAgent(unittest.TestCase):
    """Test cases for the VLA Agent"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        if not rclpy.ok():
            rclpy.init()

        self.node = VLAAgentNode()

    def tearDown(self):
        """Tear down test fixtures after each test method."""
        self.node.destroy_node()

    def test_agent_initialization(self):
        """Test that the VLA agent initializes correctly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.agent_name, 'humanoid_vla_agent')
        self.assertTrue(self.node.enable_vision)
        self.assertTrue(self.node.enable_language)
        self.assertTrue(self.node.enable_action)

    def test_voice_command_processing(self):
        """Test processing of voice commands"""
        # Create a mock message
        msg = String()
        msg.data = "move forward 2 meters"

        # Call the callback
        self.node.voice_command_callback(msg)

        # Check that the command was stored
        self.assertEqual(self.node.current_command, "move forward 2 meters")

    def test_action_plan_processing(self):
        """Test processing of action plans"""
        # Create a mock action plan
        plan = [
            {"action_type": "MOVE_RELATIVE", "direction": "forward", "distance": 2.0},
            {"action_type": "TURN", "direction": "left", "angle": 90}
        ]

        import json
        msg = String()
        msg.data = json.dumps(plan)

        # Call the callback
        self.node.action_plan_callback(msg)

        # Check that the plan was stored
        self.assertEqual(len(self.node.current_plan), 2)
        self.assertEqual(self.node.current_plan[0]["action_type"], "MOVE_RELATIVE")

    def test_execute_simple_plan(self):
        """Test execution of a simple action plan"""
        plan = [
            {"action_type": "WAIT", "duration": 0.1}  # Short wait for testing
        ]

        # Execute the plan
        self.node.execute_plan(plan)

        # Since the plan execution is synchronous in our implementation,
        # this test verifies that no exceptions are raised
        self.assertFalse(self.node.is_executing)  # Should be False after execution


class TestLLMPlanning(unittest.TestCase):
    """Test cases for the LLM Planning module"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        if not rclpy.ok():
            rclpy.init()

        # Create a node without OpenAI client for testing
        with patch('openai.OpenAI'):
            self.node = LLMPlanningNode()

    def tearDown(self):
        """Tear down test fixtures after each test method."""
        self.node.destroy_node()

    def test_generate_simple_plan(self):
        """Test generation of a simple action plan"""
        # Test with a mock OpenAI response
        test_response = '[{"action_type": "MOVE_RELATIVE", "direction": "forward", "distance": 1.0}]'

        # Mock the OpenAI client
        with patch.object(self.node, 'client') as mock_client:
            mock_completion = Mock()
            mock_completion.choices = [Mock()]
            mock_completion.choices[0].message.content = test_response
            mock_client.chat.completions.create.return_value = mock_completion

            # Generate plan
            result = self.node.generate_action_plan("move forward 1 meter")

            # Verify the result
            self.assertEqual(len(result), 1)
            self.assertEqual(result[0]["action_type"], "MOVE_RELATIVE")
            self.assertEqual(result[0]["direction"], "forward")
            self.assertEqual(result[0]["distance"], 1.0)

    def test_voice_command_processing(self):
        """Test processing of voice commands through the callback"""
        # Create a mock message
        msg = String()
        msg.data = "turn left"

        # Mock the generate_action_plan method
        with patch.object(self.node, 'generate_action_plan') as mock_gen_plan:
            mock_gen_plan.return_value = [
                {"action_type": "TURN", "direction": "left", "angle": 90}
            ]

            # Mock the execute_action_plan method
            with patch.object(self.node, 'execute_action_plan') as mock_exec_plan:
                # Call the callback
                self.node.voice_command_callback(msg)

                # Verify that the plan generation was called
                mock_gen_plan.assert_called_once_with("turn left")
                mock_exec_plan.assert_called_once()


class TestActionExecution(unittest.TestCase):
    """Test cases for the Action Execution module"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        if not rclpy.ok():
            rclpy.init()

        self.node = ActionExecutionNode()

    def tearDown(self):
        """Tear down test fixtures after each test method."""
        self.node.destroy_node()

    def test_execute_move_relative(self):
        """Test execution of relative movement action"""
        action = {
            "action_type": "MOVE_RELATIVE",
            "direction": "forward",
            "distance": 1.0,
            "speed": 0.5
        }

        # Execute the action
        result = self.node.execute_single_action(action)

        # Verify success
        self.assertTrue(result)

    def test_execute_turn(self):
        """Test execution of turn action"""
        action = {
            "action_type": "TURN",
            "direction": "left",
            "angle": 90,
            "angular_speed": 0.5
        }

        # Execute the action
        result = self.node.execute_single_action(action)

        # Verify success
        self.assertTrue(result)

    def test_execute_wait(self):
        """Test execution of wait action"""
        action = {
            "action_type": "WAIT",
            "duration": 0.1  # Short duration for testing
        }

        # Execute the action
        result = self.node.execute_single_action(action)

        # Verify success
        self.assertTrue(result)

    def test_execute_unknown_action(self):
        """Test execution of unknown action type"""
        action = {
            "action_type": "UNKNOWN_ACTION",
            "param": "value"
        }

        # Execute the action (should succeed without error)
        result = self.node.execute_single_action(action)

        # Verify success (unknown actions should not fail execution)
        self.assertTrue(result)


class TestVoiceCommandInterface(unittest.TestCase):
    """Test cases for the Voice Command Interface"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        if not rclpy.ok():
            rclpy.init()

        # Create node with mocked OpenAI client
        with patch('openai.OpenAI'):
            self.node = VoiceCommandInterface()

    def tearDown(self):
        """Tear down test fixtures after each test method."""
        self.node.destroy_node()

    def test_process_with_local_stt(self):
        """Test local speech-to-text processing"""
        # Test the local STT fallback method
        result = self.node.process_with_local_stt("dummy.wav")

        # Should return a placeholder string
        self.assertIsInstance(result, str)
        self.assertGreater(len(result), 0)


def suite():
    """Create a test suite combining all test cases"""
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestVLAAgent))
    suite.addTest(unittest.makeSuite(TestLLMPlanning))
    suite.addTest(unittest.makeSuite(TestActionExecution))
    suite.addTest(unittest.makeSuite(TestVoiceCommandInterface))
    return suite


if __name__ == '__main__':
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    suite = suite()
    result = runner.run(suite)

    # Exit with error code if tests failed
    sys.exit(0 if result.wasSuccessful() else 1)