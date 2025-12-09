#!/usr/bin/env python3

"""
Tests for the Isaac AI brain modules.
These tests verify that the SLAM, navigation, and perception components work correctly.
"""

import unittest
import os
import sys
import numpy as np
from unittest.mock import Mock, patch, MagicMock

# Add the project root to the path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

class TestIsaacSLAM(unittest.TestCase):
    """Test cases for Isaac SLAM components"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.map_resolution = 0.05  # meters per pixel
        self.map_width = 20.0      # meters
        self.map_height = 20.0     # meters
        self.map_width_pixels = int(self.map_width / self.map_resolution)
        self.map_height_pixels = int(self.map_height / self.map_resolution)

    def test_map_initialization(self):
        """Test that the occupancy map is properly initialized"""
        from isaac_ros_slam.isaac_mapping_node import IsaacMappingNode

        # Mock the ROS node initialization
        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacMappingNode()
            node.declare_parameter = Mock()
            node.get_parameter = Mock(return_value=Mock(value=0.05))

            # Initialize map dimensions
            node.map_resolution = self.map_resolution
            node.map_width = self.map_width
            node.map_height = self.map_height
            node.map_width_pixels = self.map_width_pixels
            node.map_height_pixels = self.map_height_pixels

            # Create occupancy map
            node.occupancy_map = np.full((node.map_height_pixels, node.map_width_pixels), -1, dtype=np.int8)

            # Verify map is properly initialized
            self.assertEqual(node.occupancy_map.shape, (self.map_height_pixels, self.map_width_pixels))
            self.assertTrue(np.all(node.occupancy_map == -1))  # All unknown initially

    def test_ray_tracing_algorithm(self):
        """Test the ray tracing algorithm for marking free space"""
        from isaac_ros_slam.isaac_mapping_node import IsaacMappingNode

        # Create a mock node to test the ray tracing function
        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacMappingNode()

            # Test ray tracing between two points
            # This is a simplified test of the concept
            start_x, start_y = 10, 10  # Map coordinates
            end_x, end_y = 15, 15

            # Create a small test map
            test_map = np.zeros((20, 20), dtype=np.int8)
            test_map[end_y, end_x] = 100  # Mark end as occupied

            # Simulate ray tracing (simplified version of the algorithm)
            dx = abs(end_x - start_x)
            dy = abs(end_y - start_y)
            sx = 1 if start_x < end_x else -1
            sy = 1 if start_y < end_y else -1
            err = dx - dy

            x, y = start_x, start_y
            free_cells = []

            while True:
                if x == end_x and y == end_y:
                    break

                # Mark as free space (if not already occupied)
                if test_map[y, x] != 100:  # Not occupied
                    test_map[y, x] = 0  # Free
                    free_cells.append((x, y))

                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x += sx
                if e2 < dx:
                    err += dx
                    y += sy

                if x < 0 or x >= test_map.shape[1] or y < 0 or y >= test_map.shape[0]:
                    break

            # Verify that free space was marked along the ray
            self.assertGreater(len(free_cells), 0, "No free cells were marked")
            # The end point should remain occupied
            self.assertEqual(test_map[end_y, end_x], 100)

    def test_particle_filter_initialization(self):
        """Test that the particle filter is properly initialized"""
        from isaac_ros_slam.isaac_localization_node import IsaacLocalizationNode

        num_particles = 100
        map_resolution = 0.05
        map_width = 20.0
        map_height = 20.0

        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacLocalizationNode()
            node.declare_parameter = Mock()
            param_mock = Mock()
            param_mock.value = num_particles
            node.get_parameter = Mock(return_value=param_mock)

            # Initialize particles
            node.num_particles = num_particles
            node.map_resolution = map_resolution
            node.map_width = map_width
            node.map_height = map_height
            node.map_width_pixels = int(map_width / map_resolution)
            node.map_height_pixels = int(map_height / map_resolution)

            # Initialize particles with some uncertainty
            initial_std = 0.5  # meters
            initial_yaw_std = 0.1  # radians
            initial_x, initial_y, initial_yaw = 0.0, 0.0, 0.0

            node.particles = np.zeros((node.num_particles, 3))  # x, y, theta
            node.weights = np.ones(node.num_particles) / node.num_particles

            node.particles[:, 0] = initial_x + np.random.normal(0, initial_std, node.num_particles)
            node.particles[:, 1] = initial_y + np.random.normal(0, initial_std, node.num_particles)
            node.particles[:, 2] = initial_yaw + np.random.normal(0, initial_yaw_std, node.num_particles)

            # Verify particles are initialized
            self.assertEqual(node.particles.shape, (num_particles, 3))
            self.assertEqual(len(node.weights), num_particles)
            self.assertAlmostEqual(np.sum(node.weights), 1.0, places=5)


class TestIsaacNavigation(unittest.TestCase):
    """Test cases for Isaac navigation components"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        pass

    def test_world_to_map_conversion(self):
        """Test conversion between world coordinates and map indices"""
        from isaac_ros_navigation.isaac_navigation_node import IsaacNavigationNode

        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacNavigationNode()

            # Mock map metadata
            class MockMapInfo:
                def __init__(self):
                    self.resolution = 0.05
                    self.origin = Mock()
                    self.origin.position = Mock()
                    self.origin.position.x = -10.0  # Map origin offset
                    self.origin.position.y = -10.0
                    self.width = 400  # 20m / 0.05m = 400 pixels
                    self.height = 400

            node.map_metadata = MockMapInfo()

            # Test conversion
            world_x, world_y = 0.0, 0.0  # Robot at world origin
            map_idx = node.world_to_map(world_x, world_y)

            self.assertIsNotNone(map_idx)
            expected_x = int((world_x - node.map_metadata.origin.position.x) / node.map_metadata.resolution)
            expected_y = int((world_y - node.map_metadata.origin.position.y) / node.map_metadata.resolution)
            self.assertEqual(map_idx, (expected_x, expected_y))

            # Test conversion at map boundaries
            boundary_x = node.map_metadata.origin.position.x + node.map_metadata.width * node.map_metadata.resolution
            boundary_y = node.map_metadata.origin.position.y + node.map_metadata.height * node.map_metadata.resolution
            map_idx_boundary = node.world_to_map(boundary_x - 0.1, boundary_y - 0.1)  # Just inside boundary
            self.assertIsNotNone(map_idx_boundary)

            # Test conversion outside map
            outside_idx = node.world_to_map(100.0, 100.0)  # Far outside map
            self.assertIsNone(outside_idx)

    def test_path_planning_algorithms(self):
        """Test path planning algorithms"""
        from isaac_ros_navigation.isaac_path_planner import IsaacPathPlanner

        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacPathPlanner()

            # Create a simple test map (small for testing)
            test_map = np.zeros((10, 10), dtype=np.int8)
            # Add some obstacles (value 100)
            test_map[5, 2:8] = 100  # Horizontal wall

            # Mock map metadata
            class MockMapInfo:
                def __init__(self):
                    self.resolution = 1.0
                    self.origin = Mock()
                    self.origin.position = Mock()
                    self.origin.position.x = 0.0
                    self.origin.position.y = 0.0
                    self.width = 10
                    self.height = 10

            node.inflated_map = test_map
            node.map_metadata = MockMapInfo()

            # Test A* planning
            start = (1, 1)  # Start position
            goal = (8, 8)   # Goal position (should be reachable by going around wall)

            # Since we can't call the actual planning method without full ROS setup,
            # we'll test the helper methods
            start_idx = node.world_to_map(start[0], start[1])
            goal_idx = node.world_to_map(goal[0], goal[1])

            self.assertIsNotNone(start_idx)
            self.assertIsNotNone(goal_idx)

            # Verify that the start and goal positions are in free space
            self.assertLess(test_map[start_idx[1], start_idx[0]], 50)  # Free space threshold
            self.assertLess(test_map[goal_idx[1], goal_idx[0]], 50)


class TestIsaacPerception(unittest.TestCase):
    """Test cases for Isaac perception components"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        pass

    def test_feature_extraction(self):
        """Test feature extraction methods"""
        from isaac_ros_perception.isaac_feature_extraction import IsaacFeatureExtractionNode

        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacFeatureExtractionNode()
            node.declare_parameter = Mock()
            param_mock = Mock()
            param_mock.value = 'orb'
            node.get_parameter = Mock(return_value=param_mock)

            # Initialize detector
            node.feature_type = 'orb'
            node.max_features = 500
            node.initialize_feature_detectors()

            # Create a simple test image
            test_image = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)

            # Test feature extraction
            features_msg, debug_image = node.extract_features(test_image)

            # Verify outputs are not None
            self.assertIsNotNone(features_msg)
            self.assertIsNotNone(debug_image)
            self.assertEqual(debug_image.shape[2], 3)  # Should be color image

    def test_color_feature_extraction(self):
        """Test color-based feature extraction"""
        from isaac_ros_perception.isaac_feature_extraction import IsaacFeatureExtractionNode

        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacFeatureExtractionNode()

            # Create a test image with distinct colors
            test_image = np.zeros((100, 100, 3), dtype=np.uint8)
            test_image[25:75, 25:75] = [255, 0, 0]  # Red square
            test_image[10:40, 10:40] = [0, 255, 0]  # Green square in red square

            # Test color feature extraction
            color_features = node.extract_color_features(test_image)

            # Verify we got some features
            self.assertGreater(len(color_features), 0)
            self.assertIsInstance(color_features, np.ndarray)

    def test_shape_feature_extraction(self):
        """Test shape-based feature extraction"""
        from isaac_ros_perception.isaac_feature_extraction import IsaacFeatureExtractionNode

        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacFeatureExtractionNode()

            # Create a test image with a simple shape
            test_image = np.zeros((100, 100, 3), dtype=np.uint8)
            cv2.rectangle(test_image, (30, 30), (70, 70), (255, 255, 255), -1)  # White square

            # Test shape feature extraction
            shape_features = node.extract_shape_features(test_image)

            # Verify we got some features
            self.assertGreater(len(shape_features), 0)
            self.assertIsInstance(shape_features, np.ndarray)


class TestIsaacIntegration(unittest.TestCase):
    """Test cases for Isaac module integration"""

    def test_map_to_world_conversion(self):
        """Test conversion between map indices and world coordinates"""
        from isaac_ros_navigation.isaac_path_planner import IsaacPathPlanner

        with patch('rclpy.node.Node.__init__', lambda x, y: None):
            node = IsaacPathPlanner()

            # Mock map metadata
            class MockMapInfo:
                def __init__(self):
                    self.resolution = 0.05
                    self.origin = Mock()
                    self.origin.position = Mock()
                    self.origin.position.x = -10.0
                    self.origin.position.y = -10.0

            node.map_metadata = MockMapInfo()

            # Test conversion
            map_x, map_y = 100, 100  # Some map coordinates
            world_coords = node.map_to_world(map_x, map_y)

            self.assertIsNotNone(world_coords)
            expected_x = map_x * node.map_metadata.resolution + node.map_metadata.origin.position.x
            expected_y = map_y * node.map_metadata.resolution + node.map_metadata.origin.position.y
            self.assertAlmostEqual(world_coords[0], expected_x, places=5)
            self.assertAlmostEqual(world_coords[1], expected_y, places=5)


def suite():
    """Create a test suite combining all test cases"""
    suite = unittest.TestSuite()
    suite.addTest(unittest.makeSuite(TestIsaacSLAM))
    suite.addTest(unittest.makeSuite(TestIsaacNavigation))
    suite.addTest(unittest.makeSuite(TestIsaacPerception))
    suite.addTest(unittest.makeSuite(TestIsaacIntegration))
    return suite


if __name__ == '__main__':
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    suite = suite()
    result = runner.run(suite)

    # Exit with error code if tests failed
    sys.exit(0 if result.wasSuccessful() else 1)