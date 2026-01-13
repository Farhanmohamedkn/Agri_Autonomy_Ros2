import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class PoseCovWrapper(Node):

    def __init__(self):
        super().__init__('pose_cov_wrapper')

        # Parameters (easy to tune later)
        self.declare_parameter('xy_covariance', 0.05)
        self.declare_parameter('yaw_covariance', 0.02)

        self.xy_cov = float(self.get_parameter('xy_covariance').value)
        self.yaw_cov = float(self.get_parameter('yaw_covariance').value)

        # Subscriber: Pose without covariance
        self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )

        # Publisher: Pose with covariance
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/robot/pose_cov',
            10
        )

        self.get_logger().info(
            f"Pose covariance wrapper started "
            f"(xy_cov={self.xy_cov}, yaw_cov={self.yaw_cov})"
        )

    def pose_callback(self, pose_msg: PoseStamped):
        pose_cov = PoseWithCovarianceStamped()

        # 🔴 CRITICAL: preserve header exactly
        pose_cov.header = pose_msg.header

        # Copy pose
        pose_cov.pose.pose = pose_msg.pose

        # Build covariance matrix (6x6, row-major)
        pose_cov.pose.covariance = [
            self.xy_cov, 0.0,         0.0,         0.0,         0.0,         0.0,
            0.0,         self.xy_cov, 0.0,         0.0,         0.0,         0.0,
            0.0,         0.0,         999.0,       0.0,         0.0,         0.0,
            0.0,         0.0,         0.0,         999.0,       0.0,         0.0,
            0.0,         0.0,         0.0,         0.0,         999.0,       0.0,
            0.0,         0.0,         0.0,         0.0,         0.0,         self.yaw_cov
        ]

        self.pose_cov_pub.publish(pose_cov)


def main(args=None):
    rclpy.init(args=args)
    node = PoseCovWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
