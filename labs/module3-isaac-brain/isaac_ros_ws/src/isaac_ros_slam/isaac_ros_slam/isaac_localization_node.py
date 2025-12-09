import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from tf2_ros import TransformBroadcaster
import tf2_ros
import numpy as np
import scipy.stats
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_geometry_msgs
from cv_bridge import CvBridge


class IsaacLocalizationNode(Node):
    """
    Isaac ROS Localization node for humanoid robot
    Performs Monte Carlo Localization (Particle Filter) for pose estimation
    """
    def __init__(self):
        super().__init__('isaac_localization_node')

        # Parameters
        self.declare_parameter('num_particles', 1000)
        self.declare_parameter('map_resolution', 0.05)  # meters per pixel
        self.declare_parameter('map_width', 20.0)      # meters
        self.declare_parameter('map_height', 20.0)     # meters
        self.declare_parameter('update_rate', 5.0)     # Hz
        self.declare_parameter('resample_threshold', 0.5)  # Effective particles ratio
        self.declare_parameter('initial_x', 0.0)       # meters
        self.declare_parameter('initial_y', 0.0)       # meters
        self.declare_parameter('initial_yaw', 0.0)     # radians

        self.num_particles = self.get_parameter('num_particles').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.update_rate = self.get_parameter('update_rate').value
        self.resample_threshold = self.get_parameter('resample_threshold').value
        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_yaw = self.get_parameter('initial_yaw').value

        # Initialize map dimensions
        self.map_width_pixels = int(self.map_width / self.map_resolution)
        self.map_height_pixels = int(self.map_height / self.map_resolution)

        # Initialize particle filter
        self.particles = np.zeros((self.num_particles, 3))  # x, y, theta
        self.weights = np.ones(self.num_particles) / self.num_particles

        # Initialize particles around initial pose with some uncertainty
        initial_std = 0.5  # meters
        initial_yaw_std = 0.1  # radians
        self.particles[:, 0] = initial_x + np.random.normal(0, initial_std, self.num_particles)
        self.particles[:, 1] = initial_y + np.random.normal(0, initial_std, self.num_particles)
        self.particles[:, 2] = initial_yaw + np.random.normal(0, initial_yaw_std, self.num_particles)

        # Store map and robot pose
        self.current_map = None
        self.robot_pose = np.array([initial_x, initial_y, initial_yaw])

        # Motion model noise
        self.motion_noise = {
            'translational': 0.1,  # meters
            'rotational': 0.05,    # radians
            'drift': 0.01          # systematic error
        }

        # Sensor model parameters
        self.sensor_params = {
            'max_range': 10.0,     # meters
            'sigma_hit': 0.2,      # standard deviation for hit model
            'lambda_short': 0.1,   # decay rate for short readings
            'z_hit': 0.8,          # probability of hit
            'z_short': 0.1,        # probability of short reading
            'z_max': 0.05,         # probability of max range
            'z_rand': 0.05         # probability of random measurement
        }

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

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/humanoid_robot/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/lidar/scan', self.lidar_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)

        # Create publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.particle_cloud_pub = self.create_publisher(OccupancyGrid, '/particlecloud', 10)

        # Create timer for localization updates
        self.localization_timer = self.create_timer(1.0 / self.update_rate, self.localization_update)

        self.get_logger().info(f'Isaac Localization node initialized with {self.num_particles} particles')

    def map_callback(self, msg):
        """Receive map from SLAM node"""
        # Store the map for use in localization
        self.current_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        self.get_logger().info('Received map for localization')

    def initial_pose_callback(self, msg):
        """Initialize particles based on initial pose estimate"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        quat = msg.pose.pose.orientation
        yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))

        # Reset particles around the initial pose with uncertainty
        initial_std = 0.5  # meters
        initial_yaw_std = 0.1  # radians
        self.particles[:, 0] = x + np.random.normal(0, initial_std, self.num_particles)
        self.particles[:, 1] = y + np.random.normal(0, initial_std, self.num_particles)
        self.particles[:, 2] = yaw + np.random.normal(0, initial_yaw_std, self.num_particles)

        # Reset weights
        self.weights.fill(1.0 / self.num_particles)

        self.get_logger().info(f'Reset particles around initial pose: ({x:.2f}, {y:.2f}, {yaw:.2f})')

    def odom_callback(self, msg):
        """Process odometry data for motion prediction"""
        # Extract pose from odometry
        dx = msg.pose.pose.position.x - self.robot_pose[0]
        dy = msg.pose.pose.position.y - self.robot_pose[1]

        # Convert quaternion to yaw
        quat = msg.pose.pose.orientation
        current_yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        dtheta = current_yaw - self.robot_pose[2]

        # Update robot pose
        self.robot_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, current_yaw])

        # Predict particle motion based on odometry
        self.predict_motion(dx, dy, dtheta)

    def predict_motion(self, dx, dy, dtheta):
        """Predict particle motion based on odometry"""
        # Add noise to motion
        dx_noisy = dx + np.random.normal(0, self.motion_noise['translational'], self.num_particles)
        dy_noisy = dy + np.random.normal(0, self.motion_noise['translational'], self.num_particles)
        dtheta_noisy = dtheta + np.random.normal(0, self.motion_noise['rotational'], self.num_particles)

        # Update particle positions
        cos_theta = np.cos(self.particles[:, 2])
        sin_theta = np.sin(self.particles[:, 2])

        self.particles[:, 0] += dx_noisy * cos_theta - dy_noisy * sin_theta
        self.particles[:, 1] += dx_noisy * sin_theta + dy_noisy * cos_theta
        self.particles[:, 2] += dtheta_noisy

        # Normalize angles to [-pi, pi]
        self.particles[:, 2] = (self.particles[:, 2] + np.pi) % (2 * np.pi) - np.pi

    def lidar_callback(self, msg):
        """Process LIDAR data for particle weighting"""
        if self.current_map is None:
            return

        # Convert LIDAR scan to usable format
        angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
        ranges = np.array(msg.ranges)

        # Update particle weights based on LIDAR likelihood
        self.update_particle_weights(angles, ranges)

        # Resample particles if needed
        self.resample_particles()

    def update_particle_weights(self, angles, ranges):
        """Update particle weights based on sensor likelihood"""
        # Only update weights for valid ranges
        valid_indices = (ranges > 0) & (ranges < self.sensor_params['max_range']) & ~np.isnan(ranges)
        valid_angles = angles[valid_indices]
        valid_ranges = ranges[valid_indices]

        if len(valid_ranges) == 0:
            return

        # For each particle, calculate likelihood of the measurement
        for i in range(self.num_particles):
            particle_x = self.particles[i, 0]
            particle_y = self.particles[i, 1]
            particle_theta = self.particles[i, 2]

            # Transform LIDAR beams to map frame for this particle
            beam_x = particle_x + valid_ranges * np.cos(valid_angles + particle_theta)
            beam_y = particle_y + valid_ranges * np.sin(valid_angles + particle_theta)

            # Convert to map coordinates
            map_x = ((beam_x - self.map_origin[0]) / self.map_resolution).astype(int)
            map_y = ((beam_y - self.map_origin[1]) / self.map_resolution).astype(int)

            # Check bounds
            valid_beam = (map_x >= 0) & (map_x < self.current_map.shape[1]) & \
                        (map_y >= 0) & (map_y < self.current_map.shape[0])

            if np.any(valid_beam):
                # Get occupancy values along the beams
                beam_occupancy = self.current_map[map_y[valid_beam], map_x[valid_beam]]

                # Calculate likelihood based on occupancy
                # Occupied cells (value 100) should match short ranges
                # Free cells (value 0) should match longer ranges
                likelihood = self.calculate_beam_likelihood(beam_occupancy, valid_ranges[valid_beam])
                self.weights[i] *= likelihood

        # Normalize weights
        total_weight = np.sum(self.weights)
        if total_weight > 0:
            self.weights /= total_weight
        else:
            # If all weights are zero, reset to uniform
            self.weights.fill(1.0 / self.num_particles)

    def calculate_beam_likelihood(self, occupancy_values, measured_ranges):
        """Calculate likelihood of measurements given occupancy"""
        # Simplified likelihood model
        # In a real implementation, this would use a more sophisticated beam-based model
        expected_ranges = np.where(occupancy_values == 100, 1.0, 10.0)  # Occupied vs free
        diff = np.abs(measured_ranges - expected_ranges)

        # Use Gaussian model for likelihood
        likelihood = np.mean(np.exp(-0.5 * (diff / 2.0)**2))  # sigma = 2.0
        return max(likelihood, 0.01)  # Minimum likelihood

    def resample_particles(self):
        """Resample particles based on their weights"""
        # Calculate effective number of particles
        effective_particles = 1.0 / np.sum(self.weights**2)

        if effective_particles < self.resample_threshold * self.num_particles:
            # Resample particles
            indices = np.random.choice(
                self.num_particles,
                size=self.num_particles,
                p=self.weights
            )

            self.particles = self.particles[indices]
            self.weights.fill(1.0 / self.num_particles)

    def localization_update(self):
        """Main localization update function"""
        if self.current_map is None:
            return

        # Calculate estimated pose as weighted average of particles
        estimated_x = np.average(self.particles[:, 0], weights=self.weights)
        estimated_y = np.average(self.particles[:, 1], weights=self.weights)
        estimated_theta = np.arctan2(
            np.average(np.sin(self.particles[:, 2]), weights=self.weights),
            np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        )

        # Update robot pose estimate
        self.robot_pose = np.array([estimated_x, estimated_y, estimated_theta])

        # Calculate covariance
        cov = np.cov(self.particles.T, aweights=self.weights)

        # Publish estimated pose
        self.publish_pose_estimate(estimated_x, estimated_y, estimated_theta, cov)

        # Publish particle cloud for visualization
        self.publish_particle_cloud()

    def publish_pose_estimate(self, x, y, theta, covariance):
        """Publish the estimated pose with covariance"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        from math import sin, cos
        msg.pose.pose.orientation.w = cos(theta/2)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sin(theta/2)

        # Set covariance (flatten 3x3 matrix to 6x6 with zeros for z, roll, pitch)
        cov_6x6 = np.zeros((6, 6))
        cov_6x6[0:2, 0:2] = covariance[0:2, 0:2]  # x, y
        cov_6x6[5, 5] = covariance[2, 2]  # yaw
        msg.pose.covariance = cov_6x6.flatten().tolist()

        self.pose_pub.publish(msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def publish_particle_cloud(self):
        """Publish particle cloud for visualization (simplified as occupancy grid)"""
        # This is a simplified representation - in reality, you'd publish as a PointCloud2
        # For now, we'll create a sparse occupancy grid showing particle positions

        # Create a sparse representation of particle positions
        particle_map = np.full((self.map_height_pixels, self.map_width_pixels), -1, dtype=np.int8)

        # Convert particle positions to map coordinates
        map_x = ((self.particles[:, 0] - self.map_origin[0]) / self.map_resolution).astype(int)
        map_y = ((self.particles[:, 1] - self.map_origin[1]) / self.map_resolution).astype(int)

        # Filter valid coordinates
        valid = (map_x >= 0) & (map_x < self.map_width_pixels) & \
                (map_y >= 0) & (map_y < self.map_height_pixels)

        valid_x = map_x[valid]
        valid_y = map_y[valid]

        # Mark particle positions (use weight to determine intensity)
        for i, (x, y) in enumerate(zip(valid_x, valid_y)):
            if i < len(self.weights):
                # Convert weight to occupancy value (0-100)
                occupancy = int(self.weights[valid][i] * 100 * self.num_particles)
                particle_map[y, x] = min(100, occupancy)

        # Publish as occupancy grid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width_pixels
        msg.info.height = self.map_height_pixels
        msg.info.origin.position.x = self.map_origin[0]
        msg.info.origin.position.y = self.map_origin[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        flat_map = particle_map.flatten()
        msg.data = [int(x) for x in flat_map]

        self.particle_cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    isaac_localization_node = IsaacLocalizationNode()

    try:
        rclpy.spin(isaac_localization_node)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_localization_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()