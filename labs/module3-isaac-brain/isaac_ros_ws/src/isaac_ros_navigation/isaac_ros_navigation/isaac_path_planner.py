import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from queue import PriorityQueue
from threading import Lock


class IsaacPathPlanner(Node):
    """
    Isaac ROS Path Planner for humanoid robot
    Implements global path planning algorithms (A*, Dijkstra, etc.)
    """
    def __init__(self):
        super().__init__('isaac_path_planner')

        # Parameters
        self.declare_parameter('planner_type', 'astar')  # astar, dijkstra, rrt
        self.declare_parameter('inflation_radius', 0.3)  # meters
        self.declare_parameter('cost_factor', 1.0)       # cost multiplier
        self.declare_parameter('use_gradients', True)    # use gradient-based planning
        self.declare_parameter('max_iterations', 10000)  # max iterations for planning
        self.declare_parameter('goal_tolerance', 0.5)    # meters

        self.planner_type = self.get_parameter('planner_type').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.cost_factor = self.get_parameter('cost_factor').value
        self.use_gradients = self.get_parameter('use_gradients').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # Initialize planner state
        self.map_data = None
        self.map_metadata = None
        self.inflated_map = None
        self.path_lock = Lock()

        # Create subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Create publishers
        self.path_pub = self.create_publisher(Path, '/humanoid_robot/global_plan', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/humanoid_robot/path_marker', 10)

        # Create service for path planning
        self.plan_path_service = self.create_service(
            # In a real implementation, this would be a custom service
            # For now, we'll use a timer to demonstrate planning
        )

        self.get_logger().info(f'Isaac Path Planner initialized with {self.planner_type} planner')

    def map_callback(self, msg):
        """Receive map from SLAM"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_metadata = msg.info

        # Inflate obstacles in the map
        self.inflated_map = self.inflate_obstacles(self.map_data, msg.info.resolution)

        self.get_logger().info('Received and processed map for path planning')

    def inflate_obstacles(self, original_map, resolution):
        """Inflate obstacles by inflation radius"""
        inflated_map = original_map.copy()
        inflation_cells = int(self.inflation_radius / resolution)

        if inflation_cells <= 0:
            return inflated_map

        # Create a kernel for obstacle inflation
        kernel_size = 2 * inflation_cells + 1
        kernel = np.ones((kernel_size, kernel_size))

        # Find occupied cells
        occupied_cells = np.where(original_map > 50)  # Threshold for occupancy

        # Inflate each occupied cell
        for i, j in zip(occupied_cells[0], occupied_cells[1]):
            # Calculate bounds for inflation
            y_min = max(0, i - inflation_cells)
            y_max = min(original_map.shape[0], i + inflation_cells + 1)
            x_min = max(0, j - inflation_cells)
            x_max = min(original_map.shape[1], j + inflation_cells + 1)

            # Mark area around occupied cell as occupied
            for y in range(y_min, y_max):
                for x in range(x_min, x_max):
                    dist = math.sqrt((y - i)**2 + (x - j)**2)
                    if dist <= inflation_cells:
                        inflated_map[y, x] = 100  # Occupied

        return inflated_map

    def plan_path(self, start, goal):
        """
        Plan path from start to goal
        :param start: (x, y) in world coordinates
        :param goal: (x, y) in world coordinates
        :return: Path as list of (x, y) coordinates or None if no path found
        """
        if self.inflated_map is None:
            self.get_logger().error('No map available for path planning')
            return None

        # Convert world coordinates to map indices
        start_idx = self.world_to_map(start[0], start[1])
        goal_idx = self.world_to_map(goal[0], goal[1])

        if start_idx is None or goal_idx is None:
            self.get_logger().error('Start or goal position is outside map bounds')
            return None

        # Check if start or goal is in occupied space
        if self.inflated_map[start_idx[1], start_idx[0]] > 50:
            self.get_logger().error('Start position is in occupied space')
            return None

        if self.inflated_map[goal_idx[1], goal_idx[0]] > 50:
            self.get_logger().error('Goal position is in occupied space')
            return None

        # Plan path based on selected algorithm
        if self.planner_type == 'astar':
            path_indices = self.a_star_plan(start_idx, goal_idx)
        elif self.planner_type == 'dijkstra':
            path_indices = self.dijkstra_plan(start_idx, goal_idx)
        elif self.planner_type == 'rrt':
            path_indices = self.rrt_plan(start_idx, goal_idx)
        else:
            self.get_logger().error(f'Unknown planner type: {self.planner_type}')
            return None

        if path_indices is None:
            self.get_logger().error('Failed to find path to goal')
            return None

        # Convert path indices back to world coordinates
        path_world = []
        for idx in path_indices:
            world_coords = self.map_to_world(idx[0], idx[1])
            if world_coords:
                path_world.append(world_coords)

        return path_world

    def a_star_plan(self, start, goal):
        """A* path planning algorithm"""
        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        def get_neighbors(node):
            neighbors = []
            # 8-connected neighborhood
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = node[0] + dx, node[1] + dy
                    if (0 <= nx < self.inflated_map.shape[1] and
                        0 <= ny < self.inflated_map.shape[0]):
                        # Check if cell is not occupied
                        if self.inflated_map[ny, nx] < 50:  # Free space threshold
                            # Add cost based on terrain
                            cost = 1.0 if dx == 0 or dy == 0 else 1.414  # 4-connected vs 8-connected
                            neighbors.append(((nx, ny), cost))
            return neighbors

        # Initialize data structures
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        visited_count = 0

        while not open_set.empty():
            if visited_count > self.max_iterations:
                self.get_logger().warn('A* reached maximum iterations')
                return None

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

            for neighbor, move_cost in get_neighbors(current):
                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    open_set.put((f_score[neighbor], neighbor))

            visited_count += 1

        return None  # No path found

    def dijkstra_plan(self, start, goal):
        """Dijkstra's path planning algorithm (A* with zero heuristic)"""
        def get_neighbors(node):
            neighbors = []
            # 4-connected neighborhood
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                nx, ny = node[0] + dx, node[1] + dy
                if (0 <= nx < self.inflated_map.shape[1] and
                    0 <= ny < self.inflated_map.shape[0]):
                    # Check if cell is not occupied
                    if self.inflated_map[ny, nx] < 50:  # Free space threshold
                        cost = 1.0 if abs(dx) + abs(dy) == 1 else 1.414  # 4-connected vs 8-connected
                        neighbors.append(((nx, ny), cost))
            return neighbors

        # Initialize data structures
        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        g_score = {start: 0}

        visited_count = 0

        while not open_set.empty():
            if visited_count > self.max_iterations:
                self.get_logger().warn('Dijkstra reached maximum iterations')
                return None

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

            for neighbor, move_cost in get_neighbors(current):
                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    open_set.put((tentative_g_score, neighbor))

            visited_count += 1

        return None  # No path found

    def rrt_plan(self, start, goal):
        """RRT (Rapidly-exploring Random Tree) path planning algorithm - simplified version"""
        # For simplicity, implement a basic RRT approach
        # In a real implementation, this would be more sophisticated

        # This is a simplified version - a full RRT implementation would be more complex
        # For now, we'll fall back to A* for demonstration
        self.get_logger().info('RRT planner not fully implemented, falling back to A*')
        return self.a_star_plan(start, goal)

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

    def smooth_path(self, path):
        """Apply path smoothing to reduce path length and improve smoothness"""
        if len(path) < 3:
            return path

        # Simple smoothing by removing unnecessary waypoints
        smoothed_path = [path[0]]

        i = 0
        while i < len(path) - 2:
            # Check if we can skip the next point and go directly to the one after
            if self.is_line_clear(path[i], path[i+2]):
                # Skip the middle point
                i += 2
            else:
                # Keep the next point
                smoothed_path.append(path[i+1])
                i += 1

        # Add the last point
        if len(path) > 1:
            smoothed_path.append(path[-1])

        return smoothed_path

    def is_line_clear(self, point1, point2):
        """Check if a straight line between two points is obstacle-free"""
        # Simplified implementation - in reality, this would use a line-drawing algorithm
        # to check each cell along the line for obstacles
        x1, y1 = point1
        x2, y2 = point2

        # Convert to map coordinates
        idx1 = self.world_to_map(x1, y1)
        idx2 = self.world_to_map(x2, y2)

        if idx1 is None or idx2 is None:
            return False

        # For simplicity, just check if both points are in free space
        # A full implementation would check the entire line
        return (self.inflated_map[idx1[1], idx1[0]] < 50 and
                self.inflated_map[idx2[1], idx2[0]] < 50)

    def create_path_message(self, path_world):
        """Create a Path message from world coordinates"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path_world:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 0.0
            # Set orientation to identity (will be computed during navigation)
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        return path_msg

    def create_path_marker(self, path_msg):
        """Create a marker for visualizing the path"""
        marker = Marker()
        marker.header = path_msg.header
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the scale of the marker
        marker.scale.x = 0.05  # Line width

        # Set the color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the points of the line strip
        for pose_stamped in path_msg.poses:
            point = Point()
            point.x = pose_stamped.pose.position.x
            point.y = pose_stamped.pose.position.y
            point.z = 0.0  # Keep path on ground plane
            marker.points.append(point)

        return marker


def main(args=None):
    rclpy.init(args=args)

    isaac_path_planner = IsaacPathPlanner()

    try:
        rclpy.spin(isaac_path_planner)
    except KeyboardInterrupt:
        pass
    finally:
        isaac_path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()