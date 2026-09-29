import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from scipy.spatial.transform import Rotation as R
import numpy as np


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription  # prevent unused variable warning
        self.publisher 

    def listener_callback(self, msg):

        kx = 0.1
        ky = 0.1
        ktheta = 0.1

        x_pose = msg.pose.pose.position.x
        y_pose = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        theta = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y**2 + q.z**2))

        rot_mat = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta),  np.cos(theta)]])
        
        #New code
        x_d = 2.0
        y_d = 1.0
        theta_d = np.pi

        distance_error = np.sqrt((x_d - x_pose)**2 + (y_d - y_pose)**2)

        theta_path = np.arctan2(x_d - x_pose, y_d - y_pose)
        
        #quat = [x, y, z, w]
        #r = R.from_quat(quat)
        #rot_mat = r.as_matrix() 
        dist_diff = np.array([y_d - y_pose, x_d - x_pose])

        error = rot_mat.T @ dist_diff

        error_theta = np.arctan2(np.sin(theta_d - theta), np.cos(theta_d - theta))

        v = kx * error[0]
        w = ky * error[1] + ktheta * error_theta

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w

        self.publisher.publish(cmd)

        if distance_error < 0.05 and abs(theta_error) < 0.05:

            # Stop robot
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

            self.publisher.publish(cmd)

            print("Target reached!")

            return





        print(f"x = {x_pose:.3f}, y = {y_pose:.3f}, error = {error}, error_theta = {error_theta}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()