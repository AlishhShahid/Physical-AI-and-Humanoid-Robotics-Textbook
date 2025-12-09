import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from collections import deque
import tf2_ros
from geometry_msgs.msg import TransformStamped


class IsaacMappingNode(Node):
    """
    Isaac ROS Mapping node for humanoid robot
    Processes sensor data to create and update occupancy grid maps
    """
    def __init__(self):
        super().__init__('isaac_mapping_node')

        # Parameters
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_width', 20.0)      # meters
        self.declare_parameter('map_height', 20.0)     # meters
        self.declare_parameter('update_rate', 2.0)     # Hz
        self.declare_parameter('max_lidar_range', 10.0)  # meters
        self.declare_parameter('min_lidar_range', 0.1)   # meters
        self.declare_parameter('map_publish_rate', 0.5)  # Hz

        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.min_lidar_range = self.get_parameter('min_lidar_range').value
        self.map_publish_rate = self.get_parameter('map_publish_rate').value

        # Initialize map dimensions
        self.map_width_pixels = int(self.map_width / self.map_resolution)
        self.map_height_pixels = int(self.map_height / self.map_resolution)

        # Create occupancy grid map with probabilities (0-100, -1 unknown)
        self.occupancy_map = np.full((self.map_height_pixels, self.map_width_pixels), -1, dtype=np.int8)

        # Create costmap for navigation (with inflation)
        self.costmap = np.zeros((self.map_height_pixels, self.map_width_pixels), dtype=np.uint8)

        # Robot pose tracking
        self.robot_x = self.map_width_pixels // 2
        self.robot_y = self.map_height_pixels // 2
        self.robot_theta = 0.0

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/lidar/scan', self.lidar_callback, 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)

        # Create timers
        self.map_timer = self.create_timer(1.0 / self.update_rate, self.process_map_updates)
        self.publish_timer = self.create_timer(1.0 / self.map_publish_rate, self.publish_maps)

        # Data buffers
        self.scan_buffer = deque(maxlen=10)  # Store recent scans for consistency

        self.get_logger().info('Isaac Mapping node initialized')

    def lidar_callback(self, msg):
        """Process LIDAR scan data for mapping"""
        try:
            # Convert LIDAR scan to map coordinates
            angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
            ranges = np.array(msg.ranges)

            # Filter valid ranges
            valid_indices = (ranges >= self.min_lidar_range) & (ranges <= self.max_lidar_range) & ~np.isnan(ranges)
            valid_angles = angles[valid_indices]
            valid_ranges = ranges[valid_indices]

            # Convert to Cartesian coordinates in robot frame
            x_robot = valid_ranges * np.cos(valid_angles)
            y_robot = valid_ranges * np.sin(valid_angles)

            # Transform to map frame based on current robot pose
            cos_theta = np.cos(self.robot_theta)
            sin_theta = np.sin(self.robot_theta)

            x_map = self.robot_x + (x_robot * cos_theta - y_robot * sin_theta) / self.map_resolution
            y_map = self.robot_y + (x_robot * sin_theta + y_robot * cos_theta) / self.map_resolution

            # Update occupancy grid using ray casting
            self.update_occupancy_grid(x_map, y_map, valid_ranges)

            # Store scan for consistency checking
            self.scan_buffer.append({
                'x_map': x_map,
                'y_map': y_map,
                'ranges': valid_ranges,
                'timestamp': msg.header.stamp
            })

        except Exception as e:
            self.get_logger().error(f'Error processing LIDAR: {e}')

    def update_occupancy_grid(self, x_map, y_map, ranges):
        """Update occupancy grid using ray casting algorithm"""
        # Bounds checking
        valid_points = (x_map >= 0) & (x_map < self.map_width_pixels) & \
                      (y_map >= 0) & (y_map < self.map_height_pixels)

        x_map = x_map[valid_points].astype(int)
        y_map = y_map[valid_points].astype(int)

        # Mark occupied cells along the ray
        for i in range(len(x_map)):
            if 0 <= x_map[i] < self.map_width_pixels and 0 <= y_map[i] < self.map_height_pixels:
                # Mark as occupied (high probability)
                self.occupancy_map[y_map[i], x_map[i]] = 99  # Highly likely occupied

                # Perform ray tracing to mark free space along the beam
                self.ray_trace_free_space(self.robot_x, self.robot_y, x_map[i], y_map[i])

    def ray_trace_free_space(self, x0, y0, x1, y1):
        """Ray trace from robot position to detected obstacle to mark free space"""
        # Bresenham's line algorithm to mark free space
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            # Check bounds
            if not (0 <= x < self.map_width_pixels and 0 <= y < self.map_height_pixels):
                break

            # Don't mark occupied cells as free
            if self.occupancy_map[y, x] != 99:  # Not already marked as occupied
                # Mark as free space with high probability
                if self.occupancy_map[y, x] == -1:  # Unknown
                    self.occupancy_map[y, x] = 0  # Free
                elif self.occupancy_map[y, x] > 20:  # If it was marked occupied
                    self.occupancy_map[y, x] = max(0, self.occupancy_map[y, x] - 10)  # Reduce occupancy

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def process_map_updates(self):
        """Process map updates and apply map maintenance operations"""
        # Apply map smoothing/filtering
        self.smooth_map()

        # Update costmap with obstacle inflation
        self.update_costmap()

    def smooth_map(self):
        """Apply smoothing to reduce noise in the map"""
        # Create a copy of the current map
        smoothed_map = self.occupancy_map.copy()

        # Apply a simple smoothing filter (excluding unknown cells)
        for i in range(1, self.map_height_pixels - 1):
            for j in range(1, self.map_width_pixels - 1):
                if self.occupancy_map[i, j] != -1:  # Not unknown
                    # Get 3x3 neighborhood (excluding unknown cells)
                    neighborhood = self.occupancy_map[i-1:i+2, j-1:j+2]
                    valid_cells = neighborhood[neighborhood != -1]

                    if len(valid_cells) > 4:  # At least 5 valid cells in neighborhood
                        # Calculate average, excluding the center
                        avg = np.mean(valid_cells)
                        smoothed_map[i, j] = int(np.clip(avg, 0, 100))

        self.occupancy_map = smoothed_map

    def update_costmap(self):
        """Update costmap with obstacle inflation for navigation"""
        # Reset costmap
        self.costmap.fill(0)

        # Find occupied cells in the occupancy map
        occupied_cells = np.where(self.occupancy_map > 50)  # Threshold for occupancy

        # Inflate obstacles in costmap
        inflation_radius = int(0.3 / self.map_resolution)  # 30cm inflation

        for i, j in zip(occupied_cells[0], occupied_cells[1]):
            # Create inflation around occupied cells
            y_min = max(0, i - inflation_radius)
            y_max = min(self.map_height_pixels, i + inflation_radius + 1)
            x_min = max(0, j - inflation_radius)
            x_max = min(self.map_width_pixels, j + inflation_radius + 1)

            for y in range(y_min, y_max):
                for x in range(x_min, x_max):
                    # Calculate distance from obstacle
                    dist = np.sqrt((y - i)**2 + (x - j)**2)
                    if dist <= inflation_radius:
                        # Inverse relationship: closer = higher cost
                        cost = int(254 * (1.0 - dist / inflation_radius))
                        self.costmap[y, x] = max(self.costmap[y, x], cost)

    def publish_maps(self):
        """Publish the occupancy grid and costmap"""
        # Publish occupancy map
        if self.occupancy_map.size > 0:
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'map'

            # Set map metadata
            map_msg.info.resolution = self.map_resolution
            map_msg.info.width = self.map_width_pixels
            map_msg.info.height = self.map_height_pixels
            map_msg.info.origin.position.x = -self.map_width / 2.0
            map_msg.info.origin.position.y = -self.map_height / 2.0
            map_msg.info.origin.position.z = 0.0
            map_msg.info.origin.orientation.w = 1.0

            # Flatten the occupancy map for the message
            flat_map = self.occupancy_map.flatten()
            map_msg.data = [int(x) for x in flat_map]

            self.map_pub.publish(map_msg)

        # Publish costmap
        if self.costmap.size > 0:
            costmap_msg = OccupancyGrid()
            costmap_msg.header.stamp = self.get_clock().now().to_msg()
            costmap_msg.header.frame_id = 'map'

            # Set map metadata (same as occupancy map)
            costmap_msg.info.resolution = self.map_resolution
            costmap_msg.info.width = self.map_width_pixels
            costmap_msg.info.height = self.map_height_pixels
            costmap_msg.info.origin.position.x = -self.map_width / 2.0
            costmap_msg.info.origin.position.y = -self.map_height / 2.0
            costmap_msg.info.origin.position.z = 0.0
            costmap_msg.info.origin.orientation.w = 1.0

            # Flatten the costmap for the message
            flat_costmap = self.costmap.flatten()
            costmap_msg.data = [int(x) for x in flat_costmap]

            self.costmap_pub.publish(costmap_msg)

    def get_map(self):
        """Return the current map"""
        return self.occupancy_map.copy()

    def get_costmap(self):
        """Return the current costmap"""
        return self.costmap.copy()


def main(args=None):
    rclpy.init(args=args)

    isaac_mapping_node = IsaacMappingNode()

    try:
        rclpy.spin(isaac_mapping_node)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_mapping_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()