#!/usr/bin/env python3

"""
Tests for the Gazebo simulation of the humanoid robot digital twin.
These tests verify that the simulation components work correctly.
"""

import unittest
import os
import sys
from unittest.mock import Mock, patch

# Add the project root to the path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

class TestGazeboSimulation(unittest.TestCase):
    """Test cases for Gazebo simulation components"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.world_file_path = "gazebo_sim/worlds/humanoid_world.world"
        self.model_file_path = "gazebo_sim/models/humanoid_robot/model.urdf"
        self.config_file_path = "gazebo_sim/models/humanoid_robot/model.config"

    def test_world_file_exists(self):
        """Test that the Gazebo world file exists"""
        self.assertTrue(os.path.exists(self.world_file_path),
                       f"World file does not exist: {self.world_file_path}")

    def test_model_file_exists(self):
        """Test that the robot model file exists"""
        self.assertTrue(os.path.exists(self.model_file_path),
                       f"Model file does not exist: {self.model_file_path}")

    def test_config_file_exists(self):
        """Test that the model config file exists"""
        self.assertTrue(os.path.exists(self.config_file_path),
                       f"Config file does not exist: {self.config_file_path}")

    def test_world_file_readable(self):
        """Test that the world file can be read"""
        try:
            with open(self.world_file_path, 'r') as f:
                content = f.read()
            self.assertIsNotNone(content)
            self.assertGreater(len(content), 0)
        except Exception as e:
            self.fail(f"Could not read world file: {e}")

    def test_model_file_readable(self):
        """Test that the model file can be read"""
        try:
            with open(self.model_file_path, 'r') as f:
                content = f.read()
            self.assertIsNotNone(content)
            self.assertGreater(len(content), 0)
        except Exception as e:
            self.fail(f"Could not read model file: {e}")

    def test_world_has_physics_config(self):
        """Test that the world file contains physics configuration"""
        with open(self.world_file_path, 'r') as f:
            content = f.read()

        self.assertIn('<physics', content, "Physics configuration not found in world file")
        self.assertIn('type="ode"', content, "ODE physics engine not specified")
        self.assertIn('<max_step_size>', content, "max_step_size not found in physics config")

    def test_model_has_transmissions(self):
        """Test that the model file contains transmission definitions"""
        with open(self.model_file_path, 'r') as f:
            content = f.read()

        self.assertIn('<transmission', content, "Transmission definitions not found in model")
        self.assertIn('hardwareInterface', content, "Hardware interfaces not found in transmissions")

    def test_model_has_gazebo_plugins(self):
        """Test that the model file contains Gazebo plugins"""
        with open(self.model_file_path, 'r') as f:
            content = f.read()

        self.assertIn('<gazebo>', content, "Gazebo plugins not found in model")
        self.assertIn('libgazebo_ros_control.so', content, "ROS control plugin not found")
        self.assertIn('libgazebo_ros_joint_state_publisher.so', content, "Joint state publisher plugin not found")

    def test_model_has_sensors(self):
        """Test that the model file contains sensor definitions"""
        with open(self.model_file_path, 'r') as f:
            content = f.read()

        self.assertIn('<sensor', content, "Sensor definitions not found in model")
        self.assertIn('type="imu"', content, "IMU sensor not found in model")

    def test_config_file_readable(self):
        """Test that the config file can be read"""
        try:
            with open(self.config_file_path, 'r') as f:
                content = f.read()
            self.assertIsNotNone(content)
            self.assertGreater(len(content), 0)
        except Exception as e:
            self.fail(f"Could not read config file: {e}")

    def test_config_has_model_name(self):
        """Test that the config file contains the model name"""
        with open(self.config_file_path, 'r') as f:
            content = f.read()

        self.assertIn('<name>humanoid_robot</name>', content, "Model name not found in config")

    def test_sensors_config_exists(self):
        """Test that the sensors configuration file exists"""
        sensors_config_path = "gazebo_sim/config/sensors.yaml"
        self.assertTrue(os.path.exists(sensors_config_path),
                       f"Sensors config file does not exist: {sensors_config_path}")

    def test_sensors_config_readable(self):
        """Test that the sensors config file can be read"""
        sensors_config_path = "gazebo_sim/config/sensors.yaml"
        try:
            with open(sensors_config_path, 'r') as f:
                content = f.read()
            self.assertIsNotNone(content)
            self.assertGreater(len(content), 0)
        except Exception as e:
            self.fail(f"Could not read sensors config file: {e}")

    def test_collision_detector_plugin_exists(self):
        """Test that the collision detector plugin exists"""
        plugin_path = "gazebo_sim/plugins/collision_detector.cpp"
        self.assertTrue(os.path.exists(plugin_path),
                       f"Collision detector plugin does not exist: {plugin_path}")

    def test_collision_detector_readable(self):
        """Test that the collision detector plugin can be read"""
        plugin_path = "gazebo_sim/plugins/collision_detector.cpp"
        try:
            with open(plugin_path, 'r') as f:
                content = f.read()
            self.assertIsNotNone(content)
            self.assertGreater(len(content), 0)
        except Exception as e:
            self.fail(f"Could not read collision detector plugin: {e}")

    def test_sensors_config_valid_yaml(self):
        """Test that the sensors config is valid YAML"""
        try:
            import yaml
            sensors_config_path = "gazebo_sim/config/sensors.yaml"
            with open(sensors_config_path, 'r') as f:
                data = yaml.safe_load(f)
            self.assertIsNotNone(data)
            self.assertIn('humanoid_robot', data)
        except ImportError:
            # If PyYAML is not available, just check file format
            with open(sensors_config_path, 'r') as f:
                content = f.read()
            self.assertIn('humanoid_robot:', content)
        except Exception as e:
            self.fail(f"Sensors config is not valid YAML: {e}")


class TestUnityIntegration(unittest.TestCase):
    """Test cases for Unity integration components"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.unity_readme_path = "unity_hri/README.md"
        self.unity_scene_path = "unity_hri/Assets/Scenes/MainScene.unity"
        self.ros_integration_path = "unity_hri/ROSIntegration.md"

    def test_unity_readme_exists(self):
        """Test that the Unity README file exists"""
        self.assertTrue(os.path.exists(self.unity_readme_path),
                       f"Unity README file does not exist: {self.unity_readme_path}")

    def test_unity_scene_exists(self):
        """Test that the Unity scene file exists"""
        self.assertTrue(os.path.exists(self.unity_scene_path),
                       f"Unity scene file does not exist: {self.unity_scene_path}")

    def test_ros_integration_doc_exists(self):
        """Test that the ROS integration document exists"""
        self.assertTrue(os.path.exists(self.ros_integration_path),
                       f"ROS integration document does not exist: {self.ros_integration_path}")

    def test_unity_readme_readable(self):
        """Test that the Unity README can be read"""
        try:
            with open(self.unity_readme_path, 'r') as f:
                content = f.read()
            self.assertIsNotNone(content)
            self.assertGreater(len(content), 0)
        except Exception as e:
            self.fail(f"Could not read Unity README: {e}")

    def test_unity_scene_readable(self):
        """Test that the Unity scene can be read"""
        try:
            with open(self.unity_scene_path, 'r') as f:
                content = f.read()
            self.assertIsNotNone(content)
            self.assertGreater(len(content), 0)
        except Exception as e:
            self.fail(f"Could not read Unity scene: {e}")

    def test_unity_scene_has_ros_component(self):
        """Test that the Unity scene has the ROS component"""
        with open(self.unity_scene_path, 'r') as f:
            content = f.read()

        self.assertIn('rosIP:', content, "ROS IP configuration not found in Unity scene")
        self.assertIn('rosPort:', content, "ROS port configuration not found in Unity scene")
        self.assertIn('robotTopic:', content, "Robot topic configuration not found in Unity scene")


def suite():
    """Create a test suite combining all test cases"""
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestGazeboSimulation))
    suite.addTest(unittest.makeSuite(TestUnityIntegration))
    return suite


if __name__ == '__main__':
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    suite = suite()
    result = runner.run(suite)

    # Exit with error code if tests failed
    sys.exit(0 if result.wasSuccessful() else 1)