import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, Pose
from collections import deque

class Frontier_Explorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.frontier_pub = self.create_publisher(PoseArray, '/detected_frontiers', 10)
        self.robot_x = None
        self.robot_y = None
        self.get_logger().info("Frontier Detector Node has started.")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def get_reachable_free_cells(self, data, width, height, start_gx, start_gy):
        if data[start_gy * width + start_gx] != 0:
            return set()

        visited = set()
        queue = deque([(start_gx, start_gy)])
        visited.add((start_gx, start_gy))

        while queue:
            cx, cy = queue.popleft()
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                if (nx, ny) in visited:
                    continue
                if data[ny * width + nx] == 0:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return visited

    def cluster_frontier_cells(self, frontier_cells):
        clusters = []

        visited = set()

        # Use 8-connected neighborhood for clustering
        cluster_neighbors = [
            (-1, -1),
            (-1,  0),
            (-1,  1),
            ( 0, -1),
            ( 0,  1),
            ( 1, -1),
            ( 1,  0),
            ( 1,  1)
        ]

        for cell in frontier_cells:

            if cell in visited:
                continue

            # New cluster
            cluster = []

            queue = [cell]

            visited.add(cell)

            while queue:

                current = queue.pop()

                cluster.append(current)

                cx, cy = current

                for dx, dy in cluster_neighbors:

                    nx = cx + dx
                    ny = cy + dy

                    neighbor = (nx, ny)

                    if neighbor in frontier_cells:

                        if neighbor not in visited:

                            visited.add(neighbor)

                            queue.append(neighbor)

            clusters.append(cluster)

        return clusters


    def get_cluster_representatives(self, cluster, max_cluster_span=5):
        cluster = sorted(cluster)

        representatives = []

        for i in range(0, len(cluster), max_cluster_span):
            sub_group = cluster[i:i + max_cluster_span]

            avg_x = sum(cell[0] for cell in sub_group) / len(sub_group)
            avg_y = sum(cell[1] for cell in sub_group) / len(sub_group)

            closest_cell = min(sub_group, key=lambda c: (c[0] - avg_x) ** 2 + (c[1] - avg_y) ** 2)

            representatives.append(closest_cell)

        return representatives

    

    def map_callback(self, msg: OccupancyGrid):
        if self.robot_x is None:
            return  # wait for odom

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        start_gx = int((self.robot_x - origin_x) / resolution)
        start_gy = int((self.robot_y - origin_y) / resolution)

        reachable = self.get_reachable_free_cells(data, width, height, start_gx, start_gy)

        neighbor_offsets = [-1, 1, -width, width]
        frontier_cells = set()

        for (x_grid, y_grid) in reachable:   # <-- only scan reachable cells now

            if x_grid == 0 or x_grid == width - 1 or y_grid == 0 or y_grid == height - 1:
                continue

            idx = y_grid * width + x_grid
            is_frontier = False
            for offset in neighbor_offsets:
                neighbor_idx = idx + offset
                if data[neighbor_idx] == -1:
                    is_frontier = True
                    break

            if is_frontier:
                frontier_cells.add((x_grid, y_grid))

        clusters = self.cluster_frontier_cells(frontier_cells)
            # ---------------------------------------------------------
        # Step 3: Generate representative point for each cluster
        # ---------------------------------------------------------

        frontier_poses = PoseArray()

        frontier_poses.header = msg.header

        for cluster in clusters:


            # Ignore very small clusters
            if len(cluster) < 3:
                continue

            # Calculate centroid
            for avg_x, avg_y in self.get_cluster_representatives(cluster, max_cluster_span=10):

                # Grid -> world coordinates
                world_x = (origin_x + (avg_x + 0.5) * resolution)

                world_y = (origin_y + (avg_y + 0.5) * resolution)

                pose = Pose()

                pose.position.x = world_x
                pose.position.y = world_y
                pose.position.z = 0.0

                pose.orientation.w = 1.0

                frontier_poses.poses.append(pose)

        # ---------------------------------------------------------
        # Step 4: Publish representative frontier points
        # ---------------------------------------------------------

        self.frontier_pub.publish(frontier_poses)

        self.get_logger().info(
            f"Raw frontier cells: "
            f"{len(frontier_cells)} | "
            f"Clusters: "
            f"{len(frontier_poses.poses)}",
            throttle_duration_sec=2.0
        )



            # If it's a frontier, convert to world coordinates and save it
            #if is_frontier:
                # Calculate center of the cell in world coordinates (meters)
            #    world_x = origin_x + (x_grid + 0.5) * resolution
            #    world_y = origin_y + (y_grid + 0.5) * resolution

            #    pose = Pose()
            #    pose.position.x = world_x
            #    pose.position.y = world_y
            #    pose.position.z = 0.0
            #    pose.orientation.w = 1.0  # Default neutral orientation
                
            #    frontier_poses.poses.append(pose)

        # Publish the raw frontier points
        #self.frontier_pub.publish(frontier_poses)
        #self.get_logger().info(f"Published {len(frontier_poses.poses)} frontier points.", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = Frontier_Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
