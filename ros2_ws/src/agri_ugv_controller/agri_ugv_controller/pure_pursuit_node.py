import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


def get_yaw_from_quaternion(q):
    """Extract yaw from quaternion assuming flat ground"""
    return math.atan2(
        2.0 * (q.w * q.z),
        1.0 - 2.0 * (q.z * q.z)
    )


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('cross_track_gain', 1.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.waypoint_tol = float(self.get_parameter('waypoint_tolerance').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.k_h = float(self.get_parameter('heading_gain').value)
        self.k_ct = float(self.get_parameter('cross_track_gain').value)

        # -------------------------
        # State
        # -------------------------
        self.current_pose = None
        self.path = None
        self.target_index = 0

        # -------------------------
        # Subscribers
        # -------------------------
        self.create_subscription(
            Path, '/mission/path', self.path_callback, 10
        )
        self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )

        # -------------------------
        # Publisher
        # -------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # -------------------------
        # Control loop
        # -------------------------
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("UGV Pure Pursuit Controller (EKF + CTE) started")

    def path_callback(self, msg: Path):
        # Accept path only once
        if self.path is None:
            self.path = msg.poses
            self.target_index = 0
            self.get_logger().info(
                f"Path received with {len(self.path)} waypoints"
            )

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.current_pose is None or self.path is None:
            return

        if self.target_index >= len(self.path):
            self.stop_robot()
            return

        target_pose = self.path[self.target_index].pose

        # -------------------------
        # Position error (map frame)
        # -------------------------
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        # Waypoint reached
        if distance < self.waypoint_tol:
            self.target_index += 1
            return

        # -------------------------
        # Heading
        # -------------------------
        target_heading = math.atan2(dy, dx)
        current_yaw = get_yaw_from_quaternion(
            self.current_pose.orientation
        )

        heading_error = target_heading - current_yaw
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        # -------------------------
        # Cross-track error (robot frame)
        # -------------------------
        cross_track_error = (
            -math.sin(current_yaw) * dx
            + math.cos(current_yaw) * dy
        )

        # -------------------------
        # Control law
        # -------------------------
        cmd = Twist()

        # Angular velocity: heading + lateral correction
        cmd.angular.z = (
            self.k_h * heading_error
            + self.k_ct * cross_track_error
        )
        cmd.angular.z = max(
            -self.max_w,
            min(self.max_w, cmd.angular.z)
        )

        # Linear velocity: slow down when turning hard
        cmd.linear.x = self.linear_speed * max(
            0.3, 1.0 - abs(cmd.angular.z) / self.max_w
        )

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
