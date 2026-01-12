import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class PoseCovarianceWrapper(Node):

    def __init__(self):
        super().__init__('pose_cov_wrapper')

        # ---- Parameters (tunable later) ----
        self.declare_parameter('position_stddev', 0.2)   # meters
        self.declare_parameter('yaw_stddev', 0.1)        # radians

        self.pos_std = float(self.get_parameter('position_stddev').value)
        self.yaw_std = float(self.get_parameter('yaw_stddev').value)

        # Subscriber: raw pose (no covariance)
        self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )

        # Publisher: pose with covariance
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/robot/pose_cov',
            10
        )

        self.get_logger().info(
            f"Pose covariance wrapper started "
            f"(σ_xy={self.pos_std} m, σ_yaw={self.yaw_std} rad)"
        )

    def pose_callback(self, msg: PoseStamped):
        out = PoseWithCovarianceStamped()

        # Copy header & pose
        out.header = msg.header
        out.pose.pose = msg.pose

        # Build covariance matrix (6x6, row-major)
        cov = [0.0] * 36

        cov[0]  = self.pos_std ** 2    # x
        cov[7]  = self.pos_std ** 2    # y
        cov[14] = 1e6                  # z (unused in 2D)
        cov[21] = 1e6                  # roll
        cov[28] = 1e6                  # pitch
        cov[35] = self.yaw_std ** 2    # yaw

        out.pose.covariance = cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PoseCovarianceWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
