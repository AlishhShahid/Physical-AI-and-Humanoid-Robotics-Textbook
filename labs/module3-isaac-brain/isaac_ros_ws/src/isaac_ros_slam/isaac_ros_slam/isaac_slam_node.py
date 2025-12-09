import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, Imu, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import ApproximateTimeSynchronizer, Subscriber
import message_filters


class IsaacSlamNode(Node):
    """
    Isaac ROS SLAM node for humanoid robot
    Integrates visual, LIDAR, and IMU data for mapping and localization
    """
    def __init__(self):
        super().__init__('isaac_slam_node')

        # Parameters
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_width', 20.0)      # meters
        self.declare_parameter('map_height', 20.0)     # meters
        self.declare_parameter('update_rate', 1.0)     # Hz
        self.declare_parameter('max_range', 10.0)      # meters for LIDAR
        self.declare_parameter('min_range', 0.1)       # meters for LIDAR

        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value

        # Initialize map dimensions
        self.map_width_pixels = int(self.map_width / self.map_resolution)
        self.map_height_pixels = int(self.map_height / self.map_resolution)

        # Create occupancy grid map
        self.occupancy_map = np.zeros((self.map_height_pixels, self.map_width_pixels), dtype=np.int8)
        self.occupancy_map.fill(-1)  # Unknown

        # Robot pose tracking
        self.robot_pose = np.array([self.map_width_pixels // 2, self.map_height_pixels // 2, 0.0])  # x, y, theta
        self.odom_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers for sensor data
        self.image_sub = message_filters.Subscriber(
            self, Image, '/humanoid_robot/camera/image_raw', qos_profile=qos_profile)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/humanoid_robot/depth/image_raw', qos_profile=qos_profile)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/lidar/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/humanoid_robot/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/humanoid_robot/odom', self.odom_callback, 10)

        # Synchronize image and depth for visual SLAM
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_depth_callback)

        # Create publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.odom_pub = self.create_publisher(Odometry, '/humanoid_robot/slam_odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/humanoid_robot/slam_pose', 10)

        # Create timer for map publishing
        self.map_timer = self.create_timer(1.0 / self.update_rate, self.publish_map)

        self.get_logger().info('Isaac SLAM node initialized')

    def lidar_callback(self, msg):
        """Process LIDAR data for mapping"""
        # Process LIDAR scan to update map
        angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
        ranges = np.array(msg.ranges)

        # Filter valid ranges
        valid_indices = (ranges >= self.min_range) & (ranges <= self.max_range) & ~np.isnan(ranges)
        valid_angles = angles[valid_indices]
        valid_ranges = ranges[valid_indices]

        # Convert to Cartesian coordinates in robot frame
        x_robot = valid_ranges * np.cos(valid_angles)
        y_robot = valid_ranges * np.sin(valid_angles)

        # Transform to map frame based on current robot pose
        cos_theta = np.cos(self.robot_pose[2])
        sin_theta = np.sin(self.robot_pose[2])

        x_map = self.robot_pose[0] + (x_robot * cos_theta - y_robot * sin_theta) / self.map_resolution
        y_map = self.robot_pose[1] + (x_robot * sin_theta + y_robot * cos_theta) / self.map_resolution

        # Update occupancy grid
        x_map = x_map.astype(int)
        y_map = y_map.astype(int)

        # Bounds checking
        valid_points = (x_map >= 0) & (x_map < self.map_width_pixels) & \
                      (y_map >= 0) & (y_map < self.map_height_pixels)

        x_map = x_map[valid_points]
        y_map = y_map[valid_points]

        # Mark occupied cells (simplified approach)
        for i in range(len(x_map)):
            if 0 <= x_map[i] < self.map_width_pixels and 0 <= y_map[i] < self.map_height_pixels:
                self.occupancy_map[y_map[i], x_map[i]] = 100  # Occupied

    def image_depth_callback(self, image_msg, depth_msg):
        """Process synchronized image and depth for visual SLAM"""
        try:
            # Convert ROS images to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "32FC1")

            # Perform visual feature extraction (simplified)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect features using ORB (simplified approach)
            orb = cv2.ORB_create(nfeatures=500)
            keypoints, descriptors = orb.detectAndCompute(gray, None)

            # Use depth information to get 3D positions of features
            if keypoints and descriptors is not None:
                # Process visual features for mapping
                self.process_visual_features(keypoints, cv_depth)

        except Exception as e:
            self.get_logger().error(f'Error processing image/depth: {e}')

    def process_visual_features(self, keypoints, depth_image):
        """Process visual features for SLAM"""
        # Simplified feature processing
        # In a real implementation, this would track features over time
        # and use them for pose estimation and mapping
        pass

    def imu_callback(self, msg):
        """Process IMU data for state estimation"""
        # In a real implementation, this would integrate IMU data
        # for improved pose estimation
        pass

    def odom_callback(self, msg):
        """Process odometry data for SLAM initialization"""
        # Update our internal odometry estimate
        self.odom_pose[0] = msg.pose.pose.position.x
        self.odom_pose[1] = msg.pose.pose.position.y
        # Convert quaternion to yaw
        quat = msg.pose.pose.orientation
        self.odom_pose[2] = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                       1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

        # Update robot pose based on odometry (simplified)
        # In a real implementation, this would use sensor fusion
        self.robot_pose[0] = int((self.odom_pose[0] + self.map_width/2) / self.map_resolution)
        self.robot_pose[1] = int((self.odom_pose[1] + self.map_height/2) / self.map_resolution)
        self.robot_pose[2] = self.odom_pose[2]

    def publish_map(self):
        """Publish the occupancy grid map"""
        if self.occupancy_map.size == 0:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Set map metadata
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width_pixels
        msg.info.height = self.map_height_pixels
        msg.info.origin.position.x = -self.map_width / 2.0
        msg.info.origin.position.y = -self.map_height / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten the occupancy map for the message
        flat_map = self.occupancy_map.flatten()
        msg.data = [int(x) for x in flat_map]

        self.map_pub.publish(msg)

    def publish_odom(self):
        """Publish SLAM-based odometry"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'

        # Set pose
        msg.pose.pose.position.x = self.robot_pose[0] * self.map_resolution - self.map_width/2
        msg.pose.pose.position.y = self.robot_pose[1] * self.map_resolution - self.map_height/2
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        from math import sin, cos
        yaw = self.robot_pose[2]
        msg.pose.pose.orientation.w = cos(yaw/2)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sin(yaw/2)

        # Set twist (simplified)
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = 0.0

        self.odom_pub.publish(msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    isaac_slam_node = IsaacSlamNode()

    try:
        rclpy.spin(isaac_slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()