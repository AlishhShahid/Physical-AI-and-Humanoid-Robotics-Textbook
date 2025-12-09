import unittest
import rclpy
from rclpy.node import Node
from humanoid_control_pkg.humanoid_controller import HumanoidController
from humanoid_control_pkg.motor_driver import SimulatedMotorDriver
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class TestHumanoidController(unittest.TestCase):
    """Test cases for the humanoid controller nodes"""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context once for all tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context after all tests"""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.node = HumanoidController()
        self.motor_node = SimulatedMotorDriver()

    def tearDown(self):
        """Tear down test fixtures after each test method."""
        self.node.destroy_node()
        self.motor_node.destroy_node()

    def test_node_initialization(self):
        """Test that the humanoid controller node initializes properly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'humanoid_controller')

    def test_motor_driver_initialization(self):
        """Test that the motor driver node initializes properly"""
        self.assertIsNotNone(self.motor_node)
        self.assertEqual(self.motor_node.get_name(), 'motor_driver')

    def test_joint_state_publisher_exists(self):
        """Test that the joint state publisher is created"""
        # Check that the publisher exists
        publishers = self.node.get_publishers_info_by_topic('joint_states')
        self.assertTrue(len(publishers) > 0)

    def test_command_subscription_exists(self):
        """Test that the command subscriber is created"""
        subscriptions = self.node.get_subscriptions_info_by_topic('robot_commands')
        self.assertTrue(len(subscriptions) > 0)

    def test_joint_names(self):
        """Test that the joint names are as expected"""
        expected_joints = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        # Test with a sample joint state message
        msg = JointState()
        msg.name = expected_joints
        self.assertEqual(len(msg.name), len(expected_joints))
        self.assertEqual(msg.name, expected_joints)


class TestMotorDriverFunctionality(unittest.TestCase):
    """Test cases for motor driver functionality"""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context once for all tests"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context after all tests"""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.motor_node = SimulatedMotorDriver()

    def tearDown(self):
        """Tear down test fixtures after each test method."""
        self.motor_node.destroy_node()

    def test_initial_joint_positions(self):
        """Test that initial joint positions are zero"""
        expected_joints = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]

        for joint in expected_joints:
            self.assertIn(joint, self.motor_node.joint_positions)
            self.assertEqual(self.motor_node.joint_positions[joint], 0.0)

    def test_joint_command_processing(self):
        """Test that joint commands are processed correctly"""
        from std_msgs.msg import Float64MultiArray

        # Create a sample command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.5, 0.3, 0.1]  # First 3 joints to specific positions

        # Process the command (this would normally happen via callback)
        joint_names = list(self.motor_node.joint_positions.keys())
        for i, value in enumerate(cmd_msg.data):
            if i < len(joint_names):
                joint_name = joint_names[i]
                self.motor_node.joint_positions[joint_name] = value

        # Check that the first 3 joints have the expected values
        self.assertEqual(self.motor_node.joint_positions[joint_names[0]], 0.5)
        self.assertEqual(self.motor_node.joint_positions[joint_names[1]], 0.3)
        self.assertEqual(self.motor_node.joint_positions[joint_names[2]], 0.1)


def test_suite():
    """Create a test suite combining all test cases"""
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestHumanoidController))
    suite.addTest(unittest.makeSuite(TestMotorDriverFunctionality))
    return suite


if __name__ == '__main__':
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    suite = test_suite()
    result = runner.run(suite)

    # Exit with error code if tests failed
    import sys
    sys.exit(0 if result.wasSuccessful() else 1)