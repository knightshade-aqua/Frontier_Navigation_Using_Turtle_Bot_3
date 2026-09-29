#!/usr/bin/env python3

import math
import heapq

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.frontier_sub = self.create_subscription(PoseArray, '/detected_frontiers', self.frontier_callback, 10)

        self.goal_reached_sub = self.create_subscription(Empty,'/goal_reached',self.goal_reached_callback,10)

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.map = None

        self.robot_x = None
        self.robot_y = None

        self.goal_x = None
        self.goal_y = None

        # Run planner periodically

        self.timer = self.create_timer(2.0, self.plan)
        self.current_goal = None
        self.reached_goals = []
        self.plan_needed = True
        self.frontiers = []
        self.unreachable_goals = []
        self.get_logger().info("A* planner started")


    # Goal reached callback
    def goal_reached_callback(self, msg):

        self.get_logger().info("Goal reached signal received. Selecting new frontier.")

        if self.current_goal is not None:
            self.reached_goals.append(
                self.current_goal
            )

        # Forget the old goal
        self.current_goal = None
        self.plan_needed = True

        # Force a new planning cycle
        #self.plan()



    def frontier_callback(self, pose_msg):

        for pose in pose_msg.poses:
            x = pose.position.x
            y = pose.position.y
            self.frontiers.append((x, y))

    def map_callback(self, msg):
        self.map = msg



    def odom_callback(self, msg):

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y



    def world_to_grid(self, x, y):

        resolution = self.map.info.resolution

        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)

        return grid_x, grid_y



    def grid_to_world(self, gx, gy):

        resolution = self.map.info.resolution

        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        x = origin_x + (gx + 0.5) * resolution
        y = origin_y + (gy + 0.5) * resolution

        return x, y

 
    # Check for valid cell

    def is_free(self, x, y, inflation_radius=3):



        width = self.map.info.width
        height = self.map.info.height

        if x < 0 or x >= width:
            return False

        if y < 0 or y >= height:
            return False

        index = y * width + x

        occupancy = self.map.data[index]

        # Unknown
        if occupancy == -1:
            return False

        # Occupied
        if occupancy >= 50:
            return False

        for dx in range(-inflation_radius, inflation_radius + 1):
            for dy in range(-inflation_radius, inflation_radius + 1):
                nx, ny = x + dx, y + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                n_idx = ny * width + nx
                if self.map.data[n_idx] >= 50:
                    return False

        return True

  
    # A* Heuristic Euclidean distance
    def heuristic(self, a, b):

        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return math.sqrt(dx * dx + dy * dy)

    # A* Search

    def select_frontier(self, frontiers):

        best_frontier = None
        best_distance = float('inf')

        min_frontier_distance = 0.3

        for fx, fy in frontiers:
            already_reached = False
            for gx, gy in self.reached_goals + self.unreachable_goals:
                distance_to_old_goal = math.sqrt((fx - gx) ** 2 + (fy - gy) ** 2)

                if distance_to_old_goal < 0.25:
                    already_reached = True
                    break

            if already_reached:
                continue
            # Distance from robot to frontier
            distance = math.sqrt((fx - self.robot_x) ** 2 + (fy - self.robot_y) ** 2)

            if distance < min_frontier_distance:
                continue

            if distance < best_distance:
                best_distance = distance
                best_frontier = (fx, fy)

        return best_frontier

    def astar(self, start, goal):
        # Priority queue
        open_set = []

        heapq.heappush(open_set, (0.0, start))

        # Cost from start
        g_cost = {start: 0.0}

        # Parent dictionary
        came_from = {}

        # 8-connected neighborhood
        neighbors = [

            (-1,  0, 1.0),
            ( 1,  0, 1.0),
            ( 0, -1, 1.0),
            ( 0,  1, 1.0),

            (-1, -1, math.sqrt(2)),
            (-1,  1, math.sqrt(2)),
            ( 1, -1, math.sqrt(2)),
            ( 1,  1, math.sqrt(2)),
        ]

        visited = set()

        while open_set:

            _, current = heapq.heappop(open_set)

            if current in visited:
                continue

            visited.add(current)

            # Goal reached
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy, movement_cost in neighbors:

                nx = current[0] + dx
                ny = current[1] + dy

                neighbor = (nx, ny)

                # Check map
                if not self.is_free(nx, ny):
                    continue

                # Avoid cutting diagonally through obstacles
                if dx != 0 and dy != 0:

                    if not self.is_free(current[0] + dx, current[1]):
                        continue

                    if not self.is_free(current[0], current[1] + dy):
                        continue

                new_cost = (g_cost[current] + movement_cost)

                if (neighbor not in g_cost or new_cost < g_cost[neighbor]):

                    g_cost[neighbor] = new_cost

                    f_cost = (new_cost + self.heuristic(neighbor, goal))

                    heapq.heappush(open_set, (f_cost, neighbor))

                    came_from[neighbor] = current

        # No path
        return None

  
    # Recontruct Path
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


    def find_nearest_free_cell(self, goal, search_radius=5):

        gx, gy = goal

        # Goal itself is free
        if self.is_free(gx, gy):
            return goal

        best_cell = None
        best_distance = float('inf')

        for dx in range(-search_radius, search_radius + 1):

            for dy in range(-search_radius, search_radius + 1):

                nx = gx + dx
                ny = gy + dy

                if not self.is_free(nx, ny):
                    continue

                distance = math.sqrt(dx**2 + dy**2)

                if distance < best_distance:
                    best_distance = distance
                    best_cell = (nx, ny)

        return best_cell

  
    # Main planning function

    def plan(self):

        #self.get_logger().info(f" Frontiers : {self.frontiers}")


        # Check required data
        #if not self.plan_needed:
        #    return

        if self.map is None:
            self.get_logger().info("Waiting for /map...")
            return

        if self.robot_x is None or self.robot_y is None:
            self.get_logger().info("Waiting for /odom...")
            return

        if not self.frontiers:
            self.get_logger().warn("No frontiers available.")
            return


  
        # Check whether current goal has been reached

        #if self.current_goal is not None:

        #    distance_to_goal = math.sqrt(
        #        (self.robot_x - self.current_goal[0]) ** 2 +
        #        (self.robot_y - self.current_goal[1]) ** 2
        #    )

        #    if distance_to_goal < 0.10:

        #        self.get_logger().info(f"Reached frontier: {self.current_goal}")

        #        self.reached_goals.append(self.current_goal)

        #        self.current_goal = None


       
        # Select a new frontier if necessary
        if self.current_goal is None:

            goal_world = self.select_frontier(self.frontiers)
            self.get_logger().info(f"reached_goals so far: {len(self.reached_goals)}, unique frontiers: {len(set(self.frontiers))}")

            if goal_world is None:

                self.get_logger().warn("No unexplored frontier available.")

                return

            self.current_goal = goal_world

            self.get_logger().info(f"Selected new frontier: {self.current_goal}")


        start_temp = self.world_to_grid(self.robot_x, self.robot_y)
        goal_temp = self.world_to_grid(self.current_goal[0], self.current_goal[1])

        # Checking if frontier goal is a free cell
        start = self.find_nearest_free_cell(start_temp, search_radius=5)
        goal = self.find_nearest_free_cell(goal_temp, search_radius=5)

        if start is None:
            self.get_logger().warn(f"No free cell found near frontier " f"{self.current_goal}")
            return

        if goal is None:
            self.get_logger().warn(f"No free cell found near frontier " f"{self.current_goal}")
            self.unreachable_goals.append(self.current_goal)
            self.current_goal = None
            return


        self.get_logger().info(
            f"A* planning: "
            f"start={start}, goal={goal}"
        )

        # Validate start
        if not self.is_free(start[0], start[1]):
            self.get_logger().warn("Robot start cell is not free!")
            return

        # Validate goal
        if not self.is_free(goal[0], goal[1]):
            self.get_logger().warn("Goal cell is not free!")
            return

        # Run A*
        grid_path = self.astar(start, goal)

        if grid_path is None:
            self.get_logger().warn("A* could not find a path!")
            self.unreachable_goals.append(self.current_goal)
            self.current_goal = None
            return


        self.get_logger().info(f"Path found! {len(grid_path)} cells")



        # Convert grid path -> ROS Path
        ros_path = Path()
        ros_path.header.stamp = (self.get_clock().now().to_msg())
        ros_path.header.frame_id = (self.map.header.frame_id)


        for gx, gy in grid_path:
            x, y = self.grid_to_world(gx, gy)

            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            ros_path.poses.append(pose)

        self.get_logger().info(f"Publishing path with {len(ros_path.poses)} poses")



        # Publish path

        self.path_pub.publish(ros_path)

        self.get_logger().info("Path published successfully.")
        #self.plan_needed = False


def main(args=None):

    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()