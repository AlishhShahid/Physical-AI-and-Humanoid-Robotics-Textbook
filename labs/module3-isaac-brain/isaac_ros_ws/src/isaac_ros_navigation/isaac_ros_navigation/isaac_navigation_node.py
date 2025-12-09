import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import math
from threading import Lock
from collections import deque


class IsaacNavigationNode(Node):
    """
    Isaac ROS Navigation node for humanoid robot
    Implements complete navigation stack with path planning and execution
    """
    def __init__(self):
        super().__init__('isaac_navigation_node')

        # Parameters
        self.declare_parameter('planner_frequency', 0.5)  # Hz
        self.declare_parameter('controller_frequency', 10.0)  # Hz
        self.declare_parameter('max_vel_x', 0.5)  # m/s
        self.declare_parameter('max_vel_theta', 1.0)  # rad/s
        self.declare_parameter('min_vel_x', 0.1)  # m/s
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        self.declare_parameter('yaw_goal_tolerance', 0.1)  # radians
        self.declare_parameter('obstacle_range', 2.0)  # meters
        self.declare_parameter('max_obstacle_height', 0.5)  # meters

        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.max_vel_theta = self.get_parameter('max_vel_theta').value
        self.min_vel_x = self.get_parameter('min_vel_x').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.yaw_goal_tolerance = self.get_parameter('yaw_goal_tolerance').value
        self.obstacle_range = self.get_parameter('obstacle_range').value
        self.max_obstacle_height = self.get_parameter('max_obstacle_height').value

        # Initialize navigation state
        self.current_pose = None
        self.current_goal = None
        self.current_path = None
        self.is_navigating = False
        self.navigation_cancelled = False

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/humanoid_robot/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/humanoid_robot/lidar/scan', self.laser_callback, 10)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid_robot/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/humanoid_robot/global_plan', 10)
        self.local_plan_pub = self.create_publisher(Path, '/humanoid_robot/local_plan', 10)

        # Create action server for navigation
        self.navigation_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_navigation,
            goal_callback=self.navigation_goal_callback,
            cancel_callback=self.navigation_cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # Navigation timers
        self.planner_timer = self.create_timer(
            1.0 / self.planner_frequency, self.plan_path, callback_group=ReentrantCallbackGroup())
        self.controller_timer = self.create_timer(
            1.0 / self.controller_frequency, self.follow_path, callback_group=ReentrantCallbackGroup())

        # Data storage
        self.map_data = None
        self.map_metadata = None
        self.obstacle_points = deque(maxlen=100)  # Store recent obstacle detections

        self.get_logger().info('Isaac Navigation node initialized')

    def navigation_goal_callback(self, goal_request):
        """Handle navigation goal requests"""
        self.get_logger().info(f'Received navigation goal: ({goal_request.pose.pose.position.x:.2f}, {goal_request.pose.pose.position.y:.2f})')
        return GoalResponse.ACCEPT

    def navigation_cancel_callback(self, goal_handle):
        """Handle navigation cancel requests"""
        self.get_logger().info('Received navigation cancel request')
        self.navigation_cancelled = True
        return CancelResponse.ACCEPT

    def map_callback(self, msg):
        """Receive map from SLAM"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_metadata = msg.info
        self.get_logger().info('Received map for navigation')

    def odom_callback(self, msg):
        """Receive odometry data"""
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Receive laser scan data for obstacle detection"""
        # Process laser scan to detect obstacles
        angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))])
        ranges = np.array(msg.ranges)

        # Filter valid ranges within obstacle range
        valid_indices = (ranges > 0) & (ranges <= self.obstacle_range) & ~np.isnan(ranges)
        valid_angles = angles[valid_indices]
        valid_ranges = ranges[valid_indices]

        # Convert to Cartesian coordinates relative to robot
        x_robot = valid_ranges * np.cos(valid_angles)
        y_robot = valid_ranges * np.sin(valid_angles)

        # Store obstacle points for path planning
        for x, y in zip(x_robot, y_robot):
            if math.sqrt(x**2 + y**2) < self.obstacle_range:
                self.obstacle_points.append((x, y))

    def execute_navigation(self, goal_handle):
        """Execute navigation action"""
        goal = goal_handle.request.pose

        self.get_logger().info(f'Executing navigation to goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')

        self.current_goal = goal
        self.is_navigating = True
        self.navigation_cancelled = False

        # Plan initial path
        success = self.plan_path()

        if not success:
            self.get_logger().error('Failed to plan initial path to goal')
            goal_handle.abort()
            self.is_navigating = False
            return

        # Execute navigation
        while self.is_navigating and not self.navigation_cancelled:
            # Check if goal is reached
            if self.is_goal_reached():
                self.get_logger().info('Goal reached successfully')
                self.is_navigating = False
                goal_handle.succeed()
                result = NavigateToPose.Result()
                result.result = True
                return result

            # Wait a bit before checking again
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # Check if navigation was cancelled
        if self.navigation_cancelled:
            self.get_logger().info('Navigation cancelled')
            goal_handle.canceled()
            self.is_navigating = False
            result = NavigateToPose.Result()
            result.result = False
            return result

        # If we get here, something went wrong
        self.get_logger().error('Navigation failed')
        goal_handle.abort()
        self.is_navigating = False
        result = NavigateToPose.Result()
        result.result = False
        return result

    def plan_path(self):
        """Plan path from current pose to goal using A* algorithm"""
        if self.current_pose is None or self.current_goal is None or self.map_data is None:
            return False

        # Convert current pose and goal to map coordinates
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        # Convert to map indices
        start_idx = self.world_to_map(start_x, start_y)
        goal_idx = self.world_to_map(goal_x, goal_y)

        if start_idx is None or goal_idx is None:
            self.get_logger().error('Start or goal position is outside map bounds')
            return False

        # Run A* path planning
        path_indices = self.a_star_plan(start_idx, goal_idx)

        if path_indices is None:
            self.get_logger().error('Failed to find path to goal')
            return False

        # Convert path indices back to world coordinates
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for idx in path_indices:
            world_pose = self.map_to_world(idx[0], idx[1])
            if world_pose:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.pose.position.x = world_pose[0]
                pose_stamped.pose.position.y = world_pose[1]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path.poses.append(pose_stamped)

        self.current_path = path
        self.path_pub.publish(path)

        return True

    def a_star_plan(self, start, goal):
        """A* path planning algorithm"""
        # Implement A* algorithm
        from queue import PriorityQueue

        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        def get_neighbors(node):
            neighbors = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = node[0] + dx, node[1] + dy
                    if 0 <= nx < self.map_metadata.width and 0 <= ny < self.map_metadata.height:
                        # Check if cell is not occupied
                        if self.map_data[ny, nx] < 50:  # Free space threshold
                            neighbors.append((nx, ny))
            return neighbors

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while not open_set.empty():
            current = open_set.get()[1]

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor in get_neighbors(current):
                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

        return None  # No path found

    def world_to_map(self, x, y):
        """Convert world coordinates to map indices"""
        if self.map_metadata is None:
            return None

        # Calculate map coordinates
        map_x = int((x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
        map_y = int((y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)

        # Check bounds
        if 0 <= map_x < self.map_metadata.width and 0 <= map_y < self.map_metadata.height:
            return (map_x, map_y)
        else:
            return None

    def map_to_world(self, map_x, map_y):
        """Convert map indices to world coordinates"""
        if self.map_metadata is None:
            return None

        world_x = map_x * self.map_metadata.resolution + self.map_metadata.origin.position.x
        world_y = map_y * self.map_metadata.resolution + self.map_metadata.origin.position.y

        return (world_x, world_y)

    def follow_path(self):
        """Follow the planned path using a local controller"""
        if not self.is_navigating or self.current_path is None or self.current_pose is None:
            return

        # Check for obstacles in the way
        if self.is_path_blocked():
            self.get_logger().info('Path is blocked by obstacles, replanning...')
            self.plan_path()
            return

        # Get the next waypoint from the path
        next_waypoint = self.get_next_waypoint()
        if next_waypoint is None:
            return

        # Calculate velocity commands to reach the waypoint
        cmd_vel = self.calculate_velocity_to_waypoint(next_waypoint)

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel)

        # Publish local plan for visualization
        self.publish_local_plan(next_waypoint)

    def is_path_blocked(self):
        """Check if the current path is blocked by obstacles"""
        if len(self.obstacle_points) == 0:
            return False

        # Check if any obstacles are on the current path
        if self.current_path is None:
            return False

        # Simple check: see if any recent obstacles are near the robot
        for obs_x, obs_y in list(self.obstacle_points)[-10:]:  # Check last 10 obstacles
            dist = math.sqrt(obs_x**2 + obs_y**2)  # Distance from robot
            if dist < 0.5:  # Obstacle is within 0.5m of robot
                return True

        return False

    def get_next_waypoint(self):
        """Get the next waypoint along the path"""
        if self.current_path is None or len(self.current_path.poses) == 0:
            return None

        # For simplicity, just return the first waypoint in the path
        # In a real implementation, we would select the closest waypoint ahead of the robot
        return self.current_path.poses[0]

    def calculate_velocity_to_waypoint(self, waypoint):
        """Calculate velocity commands to reach a waypoint"""
        cmd_vel = Twist()

        if self.current_pose is None:
            return cmd_vel

        # Get robot position and orientation
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Get waypoint position
        wp_x = waypoint.pose.position.x
        wp_y = waypoint.pose.position.y

        # Calculate distance and angle to waypoint
        dist_to_wp = math.sqrt((wp_x - robot_x)**2 + (wp_y - robot_y)**2)

        # Calculate angle to waypoint
        angle_to_wp = math.atan2(wp_y - robot_y, wp_x - robot_x)

        # Get robot's current orientation (simplified - assuming z-axis rotation)
        robot_yaw = math.atan2(2.0 * (self.current_pose.orientation.w * self.current_pose.orientation.z +
                                    self.current_pose.orientation.x * self.current_pose.orientation.y),
                              1.0 - 2.0 * (self.current_pose.orientation.y**2 +
                                          self.current_pose.orientation.z**2))

        # Calculate angle difference
        angle_diff = angle_to_wp - robot_yaw
        # Normalize angle to [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Simple proportional controller
        if dist_to_wp > self.goal_tolerance:
            # Move toward waypoint
            cmd_vel.linear.x = min(self.max_vel_x, max(self.min_vel_x, dist_to_wp * 0.5))
            cmd_vel.angular.z = max(-self.max_vel_theta, min(self.max_vel_theta, angle_diff * 1.0))
        else:
            # Close enough to waypoint, stop
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

        return cmd_vel

    def publish_local_plan(self, next_waypoint):
        """Publish local plan for visualization"""
        if next_waypoint is None:
            return

        local_path = Path()
        local_path.header.stamp = self.get_clock().now().to_msg()
        local_path.header.frame_id = 'map'

        # Add current pose and next waypoint to local plan
        current_pose_stamped = PoseStamped()
        current_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        current_pose_stamped.header.frame_id = 'map'
        current_pose_stamped.pose = self.current_pose
        local_path.poses.append(current_pose_stamped)

        local_path.poses.append(next_waypoint)

        self.local_plan_pub.publish(local_path)

    def is_goal_reached(self):
        """Check if the goal has been reached"""
        if self.current_pose is None or self.current_goal is None:
            return False

        # Calculate distance to goal
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)

        # Check position tolerance
        if dist_to_goal > self.goal_tolerance:
            return False

        # Check orientation tolerance (simplified)
        # In a real implementation, you'd check the goal's orientation as well
        return True


def main(args=None):
    rclpy.init(args=args)

    isaac_navigation_node = IsaacNavigationNode()

    # Use multi-threaded executor to handle action server callbacks
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(isaac_navigation_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        isaac_navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()