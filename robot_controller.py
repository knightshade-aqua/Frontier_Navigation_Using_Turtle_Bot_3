import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Empty

import numpy as np


class PathController(Node):

    def __init__(self):

        super().__init__('path_controller')

        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.path_sub = self.create_subscription(Path,'/planned_path',self.path_callback,10)

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        self.cmd_pub = self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.goal_reached_pub = self.create_publisher(Empty, '/goal_reached', 10)

        # ---------------------------------------------------------
        # Robot state
        # ---------------------------------------------------------

        self.x = None
        self.y = None
        self.theta = None

        # ---------------------------------------------------------
        # Current path
        # ---------------------------------------------------------

        self.path = None

        # ---------------------------------------------------------
        # Controller parameters
        # ---------------------------------------------------------

        self.kx = 0.5
        self.ky = 1.2

        self.lookahead_distance = 0.25

        self.max_linear_velocity = 0.3
        self.max_angular_velocity = 1.0

        self.goal_tolerance = 0.10

        self.get_logger().info("Path controller started.")

    # =============================================================
    # PATH CALLBACK
    # =============================================================

    def path_callback(self, msg):

        self.path = msg
        self.last_index = 0

        self.get_logger().info(f"Received new path with " f"{len(msg.poses)} poses.")

    # =============================================================
    # ODOM CALLBACK
    # =============================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion -> yaw
        self.theta = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y**2 + q.z**2)
        )

        # Follow path
        self.follow_path()

    # =============================================================
    # FOLLOW PATH
    # =============================================================

    def follow_path(self):

        if self.path is None:
            return

        if len(self.path.poses) == 0:
            return

        # ---------------------------------------------------------
        # Find lookahead point
        # ---------------------------------------------------------

        target = self.get_lookahead_point()

        if target is None:
            self.stop_robot()
            return

        target_x, target_y = target

        # ---------------------------------------------------------
        # Position error in world frame
        # ---------------------------------------------------------

        dx = target_x - self.x
        dy = target_y - self.y

        distance = np.sqrt(dx**2 + dy**2)

        # ---------------------------------------------------------
        # Transform error into robot frame
        # ---------------------------------------------------------

        angle_to_target = np.arctan2(dy, dx)
        heading_error = angle_to_target - self.theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        w = self.ky * heading_error

        if abs(heading_error) < 0.4:   
            v = self.kx * distance
        else:
            v = 0.0
        v = np.clip(v, 0.0, self.max_linear_velocity)
        w = np.clip(w, -self.max_angular_velocity, self.max_angular_velocity)

        # ---------------------------------------------------------
        # Check final goal
        # ---------------------------------------------------------

        goal_pose = self.path.poses[-1]

        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        goal_distance = np.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)

        if goal_distance < self.goal_tolerance:

            self.get_logger().info("Goal reached.")

            self.stop_robot()

            msg_reached = Empty()
            self.goal_reached_pub.publish(msg_reached)

            # Clear path
            self.path = None

            return

        # ---------------------------------------------------------
        # Publish velocity
        # ---------------------------------------------------------

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(w)

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"Target=({target_x:.2f}, {target_y:.2f}) "
            f"error=({heading_error:.2f}) "
            f"v={v:.2f} w={w:.2f}",
            throttle_duration_sec=1.0
        )

    # =============================================================
    # FIND LOOKAHEAD POINT
    # =============================================================

    def get_lookahead_point(self):

        if self.path is None:
            return None

        if not hasattr(self, 'last_index'):
            self.last_index = 0

        self.last_index = min(self.last_index, len(self.path.poses) - 1)

        # Search path for a point at least lookahead_distance from robot

        for idx in range(self.last_index, len(self.path.poses)):

            pose_stamped = self.path.poses[idx]

            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            

            distance = np.sqrt(
                (px - self.x)**2 +
                (py - self.y)**2
            )

            if distance >= self.lookahead_distance:
                self.last_index = idx
                return px, py

        # If no lookahead point exists,
        # use final point

        last_pose = self.path.poses[-1]
        self.last_index = len(self.path.poses) - 1

        return (last_pose.pose.position.x, last_pose.pose.position.y)

    # =============================================================
    # STOP ROBOT
    # =============================================================

    def stop_robot(self):

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)


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