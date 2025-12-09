---
sidebar_position: 2
---

# Week 10: Navigation Stack and Path Planning

## Learning Objectives

By the end of this week, you will be able to:
- Implement a complete navigation stack for autonomous movement
- Understand and apply various path planning algorithms (A*, Dijkstra, RRT)
- Create trajectory following and obstacle avoidance systems
- Integrate navigation with perception and mapping systems
- Test navigation performance in simulation environments

## Navigation Stack Overview

The navigation stack is a crucial component of autonomous robotics that enables robots to move safely from one location to another. It consists of several interconnected modules:

1. **Global Planner**: Generates an optimal path from start to goal
2. **Local Planner**: Creates executable trajectories while avoiding obstacles
3. **Controller**: Sends commands to robot actuators to follow trajectories
4. **Costmap**: Represents obstacles and navigation costs in the environment

## Path Planning Algorithms

### A* Algorithm

A* is a popular graph traversal algorithm that finds the shortest path between nodes using a heuristic function. For robotics, it's particularly effective because:

- It guarantees optimal solutions if the heuristic is admissible
- It's efficient for grid-based environments
- It can incorporate various cost functions

```python
import heapq
import numpy as np

class AStarPlanner:
    """
    A* path planning implementation for humanoid robot navigation
    """
    def __init__(self, map_resolution=0.05):
        self.map_resolution = map_resolution
        self.motion = self.get_motion_model()

    def planning(self, start, goal, occupancy_map):
        """
        A* path planning
        :param start: Start position (x, y)
        :param goal: Goal position (x, y)
        :param occupancy_map: 2D array representing the map
        :return: Path as list of (x, y) coordinates
        """
        # Convert to map coordinates
        start_idx = (int(start[0] / self.map_resolution), int(start[1] / self.map_resolution))
        goal_idx = (int(goal[0] / self.map_resolution), int(goal[1] / self.map_resolution))

        # Initialize open and closed sets
        open_set = []
        closed_set = set()

        # Create start node
        start_node = Node(start_idx[0], start_idx[1], 0.0, self.heuristic(start_idx, goal_idx))
        heapq.heappush(open_set, (start_node.cost, start_node))

        while open_set:
            current = heapq.heappop(open_set)[1]
            closed_set.add((current.x, current.y))

            # Check if goal is reached
            if current.x == goal_idx[0] and current.y == goal_idx[1]:
                # Reconstruct path
                path = []
                node = current
                while node.parent is not None:
                    path.append((node.x * self.map_resolution, node.y * self.map_resolution))
                    node = node.parent
                path.append((start[0], start[1]))
                return path[::-1]

            # Expand neighbors
            for motion in self.motion:
                node = Node(current.x + motion[0], current.y + motion[1],
                           current.cost + motion[2], 0.0)
                node.parent = current

                # Check if node is in closed set
                if (node.x, node.y) in closed_set:
                    continue

                # Check map boundaries
                if not self.verify_node(node, occupancy_map):
                    continue

                # Check if already in open set
                node_found = False
                for _, n in open_set:
                    if n.x == node.x and n.y == node.y:
                        node_found = True
                        if n.cost > node.cost:
                            n.cost = node.cost
                            n.parent = current
                        break

                if not node_found:
                    node.cost += self.heuristic((node.x, node.y), goal_idx)
                    heapq.heappush(open_set, (node.cost, node))

        return None  # No path found

    def heuristic(self, n1, n2):
        """Calculate heuristic value"""
        w = 1.0  # Weight of heuristic
        d = w * np.sqrt((n1[0] - n2[0])**2 + (n1[1] - n2[1])**2)
        return d

    def verify_node(self, node, occupancy_map):
        """Verify if node is valid"""
        if node.x < 0 or node.y < 0:
            return False

        if node.x >= occupancy_map.shape[1] or node.y >= occupancy_map.shape[0]:
            return False

        # Check if node is in obstacle
        if occupancy_map[node.y, node.x] > 50:  # Threshold for occupied space
            return False

        return True

    def get_motion_model(self):
        """Define motion model for path planning"""
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, np.sqrt(2)],
                  [-1, 1, np.sqrt(2)],
                  [1, -1, np.sqrt(2)],
                  [1, 1, np.sqrt(2)]]
        return motion

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index
        self.parent = None
```

### Dijkstra's Algorithm

Dijkstra's algorithm is similar to A* but doesn't use a heuristic function, making it suitable for situations where you need to find paths to multiple destinations or when a good heuristic is not available.

### RRT (Rapidly-exploring Random Tree)

RRT is particularly useful for high-dimensional configuration spaces and complex environments:

```python
import random

class RRTPlanner:
    """
    RRT path planning implementation
    """
    def __init__(self, map_resolution=0.05, expand_dis=0.5, path_resolution=0.1):
        self.map_resolution = map_resolution
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution

    def planning(self, start, goal, occupancy_map, max_iter=1000):
        """
        RRT path planning
        """
        start_node = self.Node(start[0], start[1])
        goal_node = self.Node(goal[0], goal[1])

        self.node_list = [start_node]

        for i in range(max_iter):
            # Random sampling
            rnd_node = self.get_random_node(goal_node)

            # Find nearest node
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            # Expand tree
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, occupancy_map):
                self.node_list.append(new_node)

            # Check if goal is reached
            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y, goal_node):
                return self.generate_final_course(len(self.node_list) - 1)

        return None  # Cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        """Steer from one node to another"""
        new_node = self.Node(from_node.x, from_node.y)
        d, angle = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = int(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(angle)
            new_node.y += self.path_resolution * math.sin(angle)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
```

## Isaac Navigation Node Implementation

Let's examine the complete navigation implementation for our humanoid robot:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from action_msgs.msg import GoalStatus
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from humanoid_robot_msgs.action import NavigateToPose
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class IsaacNavigationNode(Node):
    """
    Isaac Navigation node for humanoid robot
    Implements path planning and execution for autonomous navigation
    """
    def __init__(self):
        super().__init__('isaac_navigation')

        # Parameters
        self.declare_parameter('planner_frequency', 5.0)  # Hz
        self.declare_parameter('controller_frequency', 10.0)  # Hz
        self.declare_parameter('planner_type', 'astar')  # astar, dijkstra, rrt
        self.declare_parameter('inflation_radius', 0.5)  # meters

        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.planner_type = self.get_parameter('planner_type').value
        self.inflation_radius = self.get_parameter('inflation_radius').value

        # Initialize path planner
        self.path_planner = IsaacPathPlanner(planner_type=self.planner_type)

        # Navigation state
        self.current_pose = None
        self.current_map = None
        self.current_goal = None
        self.current_path = None
        self.path_index = 0

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/humanoid_robot/pose', self.pose_callback, 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/humanoid_robot/map', self.map_callback, 10)

        # Create action server for navigation
        self.nav_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.execute_navigate_to_pose,
            goal_callback=self.nav_goal_callback,
            cancel_callback=self.nav_cancel_callback
        )

        # Create publishers
        self.path_pub = self.create_publisher(Path, '/humanoid_robot/plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/humanoid_robot/cmd_vel', 10)

        # Create timers
        self.planning_timer = self.create_timer(1.0 / self.planner_frequency, self.plan_path)
        self.controller_timer = self.create_timer(1.0 / self.controller_frequency, self.follow_path)

        self.get_logger().info('Isaac Navigation node initialized')

    def nav_goal_callback(self, goal_request):
        """Handle navigation goal request"""
        # Accept all goals for now
        return GoalResponse.ACCEPT

    def nav_cancel_callback(self, goal_handle):
        """Handle navigation cancel request"""
        return CancelResponse.ACCEPT

    def execute_navigate_to_pose(self, goal_handle):
        """Execute navigation to pose action"""
        goal = goal_handle.request.pose
        self.get_logger().info(f'Navigating to pose: ({goal.position.x}, {goal.position.y})')

        # Store goal and wait for path planning
        self.current_goal = goal

        # Wait for path to be planned
        while self.current_path is None and not goal_handle.is_cancel_requested:
            rclpy.spin_once(self, timeout_sec=0.1)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return NavigateToPose.Result()

        # Follow the path
        while self.path_index < len(self.current_path) and not goal_handle.is_cancel_requested:
            rclpy.spin_once(self, timeout_sec=0.1)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return NavigateToPose.Result()

        # Check if goal reached
        if self.path_index >= len(self.current_path):
            result = NavigateToPose.Result()
            result.result = True
            goal_handle.succeed()
            return result

        result = NavigateToPose.Result()
        result.result = False
        goal_handle.abort()
        return result

    def pose_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg.pose

    def map_callback(self, msg):
        """Update current map"""
        try:
            # Convert occupancy grid to numpy array
            self.current_map = np.array(msg.data).reshape(msg.info.height, msg.info.width).astype(np.int8)
            self.map_metadata = msg.info
        except Exception as e:
            self.get_logger().error(f'Error processing map: {e}')

    def plan_path(self):
        """Plan path from current pose to goal"""
        if self.current_pose is None or self.current_goal is None or self.current_map is None:
            return

        try:
            # Convert poses to map coordinates
            start = (self.current_pose.position.x, self.current_pose.position.y)
            goal = (self.current_goal.position.x, self.current_goal.position.y)

            # Plan path using selected algorithm
            path = self.path_planner.plan(start, goal, self.current_map)

            if path is not None:
                self.current_path = path
                self.path_index = 0

                # Publish path for visualization
                self.publish_path(path)

                self.get_logger().info(f'Path planned with {len(path)} waypoints')
            else:
                self.get_logger().warn('No path found to goal')

        except Exception as e:
            self.get_logger().error(f'Error planning path: {e}')

    def publish_path(self, path):
        """Publish path for visualization"""
        try:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'

            for pose in path:
                point = PoseStamped()
                point.pose.position.x = pose[0]
                point.pose.position.y = pose[1]
                point.pose.position.z = 0.0
                path_msg.poses.append(point)

            self.path_pub.publish(path_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing path: {e}')

    def follow_path(self):
        """Follow the planned path"""
        if self.current_path is None or self.path_index >= len(self.current_path):
            return

        try:
            # Get current target waypoint
            target = self.current_path[self.path_index]

            # Calculate distance to target
            if self.current_pose:
                dist = np.sqrt((target[0] - self.current_pose.position.x)**2 +
                              (target[1] - self.current_pose.position.y)**2)

                # Move to target if close enough
                if dist < 0.1:  # 10cm threshold
                    self.path_index += 1
                    if self.path_index >= len(self.current_path):
                        self.get_logger().info('Reached goal!')
                        return

                # Calculate velocity command to reach target
                cmd_vel = self.calculate_velocity_command(target)

                # Publish velocity command
                self.cmd_vel_pub.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f'Error following path: {e}')

    def calculate_velocity_command(self, target):
        """Calculate velocity command to reach target"""
        cmd = Twist()

        if self.current_pose:
            # Calculate direction to target
            dx = target[0] - self.current_pose.position.x
            dy = target[1] - self.current_pose.position.y
            distance = np.sqrt(dx**2 + dy**2)

            if distance > 0.05:  # Only move if not very close
                # Normalize direction
                dx /= distance
                dy /= distance

                # Set linear velocity proportional to distance (with maximum)
                cmd.linear.x = min(0.5, distance * 0.5)  # Max 0.5 m/s
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0

                # Set angular velocity to face target
                current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
                target_yaw = np.arctan2(dy, dx)
                angle_diff = target_yaw - current_yaw

                # Normalize angle difference
                while angle_diff > np.pi:
                    angle_diff -= 2 * np.pi
                while angle_diff < -np.pi:
                    angle_diff += 2 * np.pi

                cmd.angular.z = min(0.5, max(-0.5, angle_diff * 1.0))  # Max 0.5 rad/s

        return cmd

    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return np.arctan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    nav_node = IsaacNavigationNode()

    try:
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Costmap and Obstacle Avoidance

The navigation system uses costmaps to represent obstacles and navigation costs:

```python
class Costmap:
    """
    Costmap for navigation planning
    """
    def __init__(self, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.costmap = np.zeros((height, width), dtype=np.uint8)

        # Define cost values
        self.FREE_SPACE = 0      # Free space
        self.OBSTACLE = 254      # Obstacle
        self.INSCRIBED = 253     # Inscribed obstacle
        self.UNKNOWN = 255       # Unknown space

    def update_with_laser_scan(self, laser_scan, robot_pose):
        """Update costmap with laser scan data"""
        # Convert laser scan to map coordinates
        for i, range_val in enumerate(laser_scan.ranges):
            if laser_scan.range_min <= range_val <= laser_scan.range_max:
                angle = laser_scan.angle_min + i * laser_scan.angle_increment
                world_x = robot_pose.position.x + range_val * np.cos(angle + robot_pose.orientation.z)
                world_y = robot_pose.position.y + range_val * np.sin(angle + robot_pose.orientation.z)

                # Convert to map coordinates
                map_x = int((world_x - self.origin_x) / self.resolution)
                map_y = int((world_y - self.origin_y) / self.resolution)

                # Mark obstacle if within bounds
                if 0 <= map_x < self.width and 0 <= map_y < self.height:
                    self.costmap[map_y, map_x] = self.OBSTACLE

    def inflate_obstacles(self, inflation_radius):
        """Inflate obstacles to account for robot size"""
        inflated_costmap = self.costmap.copy()
        inflation_cells = int(inflation_radius / self.resolution)

        for y in range(self.height):
            for x in range(self.width):
                if self.costmap[y, x] >= 100:  # Obstacle threshold
                    # Inflate around obstacle
                    for iy in range(max(0, y - inflation_cells), min(self.height, y + inflation_cells + 1)):
                        for ix in range(max(0, x - inflation_cells), min(self.width, x + inflation_cells + 1)):
                            dist = np.sqrt((ix - x)**2 + (iy - y)**2)
                            if dist <= inflation_cells:
                                current_cost = inflated_costmap[iy, ix]
                                new_cost = min(254, int(254 * (1.0 - dist / inflation_cells)))
                                inflated_costmap[iy, ix] = max(current_cost, new_cost)

        self.costmap = inflated_costmap
```

## Practical Exercise: Navigation Testing

1. Set up a navigation goal in Gazebo simulation
2. Observe the path planning process in RViz
3. Test navigation in various environments (corridors, rooms, obstacle courses)
4. Analyze path efficiency and obstacle avoidance behavior

## Summary

This week, we've implemented a complete navigation stack for our humanoid robot using NVIDIA Isaac principles. We've covered:

- Path planning algorithms (A*, Dijkstra, RRT)
- Navigation stack architecture
- Trajectory following and control
- Costmap and obstacle inflation
- Action-based navigation interface

Next week, we'll complete the Isaac implementation by focusing on perception systems that enable the robot to understand its environment.