import sys
import math
import argparse

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.utilities import remove_ros_args

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathGenerator(Node):
    """
    Generates ONE path (point-to-point or circular) relative to the robot's
    starting pose and publishes it once on /planned_path (odom frame).

    Usage:
      ros2 run <pkg> path_generator             # point-to-point (default)
      ros2 run <pkg> path_generator circular    # circular
    """

    def __init__(self, path_type='point'):
        super().__init__(
            'path_generator',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.path_select = path_type
        self.frame_id = 'odom'
        self.published = False

        # Latched publisher: a controller started later still receives the path.
        # The controller's subscription must use the same QoS.
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, '/planned_path', path_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(
            f"Path generator started (path='{self.path_select}'), waiting for odometry ...")

    # =============================================================
    # HELPERS
    # =============================================================

    def make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # =============================================================
    # ODOM CALLBACK (used once, to get the start pose)
    # =============================================================

    def odom_callback(self, msg):
        if self.published:
            return

        x0 = msg.pose.pose.position.x
        y0 = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta0 = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        if self.path_select == 'point':
            poses = self.point_to_point(x0, y0, theta0)
        else:
            poses = self.circular_path(x0, y0, theta0)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        path.poses = poses

        self.path_pub.publish(path)
        self.published = True
        self.destroy_subscription(self.odom_sub)

        self.get_logger().info(
            f"Published '{self.path_select}' path with {len(poses)} poses.")

    # =============================================================
    # POINT TO POINT
    # =============================================================

    def point_to_point(self, x0, y0, theta0):
        x_d = x0 + 5.0
        y_d = y0 + 3.0
        theta_d = math.radians(30)          # final heading (odom frame)

        dx, dy = x_d - x0, y_d - y0
        dist = math.hypot(dx, dy)
        line_yaw = math.atan2(dy, dx)

        n = max(int(dist / 0.1), 1)         # waypoint every ~10 cm

        poses = []
        for i in range(n + 1):
            t = i / n
            yaw = theta_d if i == n else line_yaw
            poses.append(self.make_pose(x0 + t * dx, y0 + t * dy, yaw))
        return poses

    # =============================================================
    # CIRCULAR
    # =============================================================

    def circular_path(self, x0, y0, theta0):
        r = 2.0
        num_points = 72                     # 5 degrees apart

        # Circle passes through the start pose, tangent to the robot heading,
        # centre on the robot's left -> counter-clockwise.
        cx = x0 - r * math.sin(theta0)
        cy = y0 + r * math.cos(theta0)

        poses = []
        for i in range(num_points + 1):     # +1 closes the loop
            phi = theta0 - math.pi / 2.0 + 2.0 * math.pi * i / num_points
            poses.append(self.make_pose(
                cx + r * math.cos(phi),
                cy + r * math.sin(phi),
                phi + math.pi / 2.0))
        return poses


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('path_type', nargs='?', default='point',
                        choices=['point', 'circular'])
    opts = parser.parse_args(remove_ros_args(args=sys.argv)[1:])

    node = PathGenerator(opts.path_type)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
