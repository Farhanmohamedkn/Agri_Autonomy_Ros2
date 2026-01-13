import random
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class EnuGpsSimulator(Node):

    def __init__(self):
        super().__init__('enu_gps_simulator')

        # Parameters
        self.declare_parameter('noise_std', 0.3)     # meters (1-sigma)
        self.declare_parameter('publish_rate', 5.0) # Hz

        self.noise_std = float(self.get_parameter('noise_std').value)
        self.rate = float(self.get_parameter('publish_rate').value)

        self.current_pose = None

        # Subscriber: simulated ground-truth pose (odom)
        self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )

        # Publisher: simulated GPS in map frame
        self.gps_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/gps/enu',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.rate,
            self.publish_gps
        )

        self.get_logger().info(
            f"ENU GPS simulator started "
            f"(noise={self.noise_std} m, rate={self.rate} Hz)"
        )

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def publish_gps(self):
        if self.current_pose is None:
            return

        # Add Gaussian noise to simulate GPS
        x = self.current_pose.pose.position.x + random.gauss(0.0, self.noise_std)
        y = self.current_pose.pose.position.y + random.gauss(0.0, self.noise_std)

        gps_msg = PoseWithCovarianceStamped()

        # 🔴 Use the same timestamp for consistency
        gps_msg.header.stamp = self.current_pose.header.stamp
        gps_msg.header.frame_id = 'map'

        gps_msg.pose.pose.position.x = x
        gps_msg.pose.pose.position.y = y
        gps_msg.pose.pose.position.z = 0.0

        # Orientation is unknown for GPS
        gps_msg.pose.pose.orientation.w = 1.0

        # Covariance matrix (6x6, row-major)
        pos_cov = self.noise_std ** 2
        very_large = 1e6

        gps_msg.pose.covariance = [
            pos_cov, 0.0,      0.0,      0.0,      0.0,      0.0,
            0.0,     pos_cov,  0.0,      0.0,      0.0,      0.0,
            0.0,     0.0,      very_large,0.0,      0.0,      0.0,
            0.0,     0.0,      0.0,      very_large,0.0,      0.0,
            0.0,     0.0,      0.0,      0.0,      very_large,0.0,
            0.0,     0.0,      0.0,      0.0,      0.0,      very_large
        ]

        self.gps_pub.publish(gps_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EnuGpsSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
