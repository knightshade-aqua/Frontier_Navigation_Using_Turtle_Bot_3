import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty

import numpy as np


def quat_to_yaw(q):
    return np.arctan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y**2 + q.z**2)
    )


def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


class PathController(Node):

    def __init__(self):

        super().__init__(
            'path_controller',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Must match the generator's latched QoS
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, path_qos)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Empty, '/goal_reached', 10)

        # Robot state
        self.x = None
        self.y = None
        self.theta = None

        # Path
        self.path = None
        self.last_index = 0

        # Controller parameters
        self.kx = 0.5
        self.ky = 1.2
        self.k_yaw = 1.2

        self.lookahead_distance = 0.25

        self.max_linear_velocity = 0.3
        self.max_angular_velocity = 1.0

        self.goal_tolerance = 0.10      
        self.yaw_tolerance = 0.05        

        self.get_logger().info("Path controller started.")

    # =============================================================
    # CALLBACKS
    # =============================================================

    def path_callback(self, msg):

        self.path = msg
        self.last_index = 0

        self.get_logger().info(f"Received new path with {len(msg.poses)} poses.")

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = quat_to_yaw(msg.pose.pose.orientation)

        self.follow_path()

    # =============================================================
    # FOLLOW PATH
    # =============================================================

    def follow_path(self):

        if self.path is None or len(self.path.poses) == 0:
            return

        # Final goal pose (position + orientation)
        goal_pose = self.path.poses[-1]
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        goal_yaw = quat_to_yaw(goal_pose.pose.orientation)

        goal_distance = np.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)
        at_final = (self.last_index == len(self.path.poses) - 1)

        
        if at_final and goal_distance < self.goal_tolerance:

            yaw_error = wrap_angle(goal_yaw - self.theta)

            if abs(yaw_error) > self.yaw_tolerance:
                w = float(np.clip(self.k_yaw * yaw_error,
                                  -self.max_angular_velocity,
                                  self.max_angular_velocity))
                self.publish_cmd(0.0, w)
                return

            self.get_logger().info("Goal reached (position and orientation).")
            self.stop_robot()
            self.goal_reached_pub.publish(Empty())
            self.path = None
            return

        # Normal path following
        target = self.get_lookahead_point()

        if target is None:
            self.stop_robot()
            return

        target_x, target_y = target

        dx = target_x - self.x
        dy = target_y - self.y
        distance = np.sqrt(dx**2 + dy**2)

        heading_error = wrap_angle(np.arctan2(dy, dx) - self.theta)

        w = self.ky * heading_error

        if abs(heading_error) < 0.4:
            v = self.kx * distance
        else:
            v = 0.0

        v = np.clip(v, 0.0, self.max_linear_velocity)
        w = np.clip(w, -self.max_angular_velocity, self.max_angular_velocity)

        self.publish_cmd(v, w)

        self.get_logger().info(
            f"Target=({target_x:.2f}, {target_y:.2f}) "
            f"error=({heading_error:.2f}) "
            f"v={v:.2f} w={w:.2f}",
            throttle_duration_sec=1.0
        )

    # =============================================================
    # LOOKAHEAD POINT
    # =============================================================

    def get_lookahead_point(self):

        if self.path is None:
            return None

        self.last_index = min(self.last_index, len(self.path.poses) - 1)

        for idx in range(self.last_index, len(self.path.poses)):

            p = self.path.poses[idx].pose.position
            distance = np.sqrt((p.x - self.x)**2 + (p.y - self.y)**2)

            if distance >= self.lookahead_distance:
                self.last_index = idx
                return p.x, p.y

        # No lookahead point left: aim at the final point
        last_pose = self.path.poses[-1]
        self.last_index = len(self.path.poses) - 1

        return last_pose.pose.position.x, last_pose.pose.position.y

    # =============================================================
    # COMMAND HELPERS
    # =============================================================

    def publish_cmd(self, v, w):

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):

        self.publish_cmd(0.0, 0.0)


def main(args=None):

    rclpy.init(args=args)

    node = PathController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
