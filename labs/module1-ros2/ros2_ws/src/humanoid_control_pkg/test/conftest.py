"""Pytest configuration for humanoid control package tests"""

import pytest
import rclpy


@pytest.fixture(scope="session")
def ros_context():
    """Initialize ROS context for all tests"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def humanoid_controller(ros_context):
    """Create a humanoid controller node for testing"""
    from humanoid_control_pkg.humanoid_controller import HumanoidController
    node = HumanoidController()
    yield node
    node.destroy_node()


@pytest.fixture
def motor_driver(ros_context):
    """Create a motor driver node for testing"""
    from humanoid_control_pkg.motor_driver import SimulatedMotorDriver
    node = SimulatedMotorDriver()
    yield node
    node.destroy_node()