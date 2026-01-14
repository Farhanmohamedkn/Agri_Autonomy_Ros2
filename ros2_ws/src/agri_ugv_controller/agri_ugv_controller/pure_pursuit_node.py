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


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.5)

        # More realistic for skid-steer than 1.0
        self.declare_parameter('max_angular_speed', 0.6)

        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('cross_track_gain', 1.0)

        # NEW: "soft" lookahead for CTE normalization (meters)
        # Larger -> less twitchy, smaller -> more aggressive
        self.declare_parameter('cte_lookahead', 1.0)

        # NEW: optional smoothing for angular command (0..1)
        # 0 = no smoothing, 0.7 = strong smoothing
        self.declare_parameter('angular_smoothing_alpha', 0.4)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.waypoint_tol = float(self.get_parameter('waypoint_tolerance').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.k_h = float(self.get_parameter('heading_gain').value)
        self.k_ct = float(self.get_parameter('cross_track_gain').value)

        self.cte_L = float(self.get_parameter('cte_lookahead').value)
        self.alpha = float(self.get_parameter('angular_smoothing_alpha').value)

        # -------------------------
        # State
        # -------------------------
        self.current_pose = None
        self.path = None
        self.target_index = 0

        # for smoothing
        self.prev_w = 0.0

        # -------------------------
        # Subscribers
        # -------------------------
        self.create_subscription(Path, '/mission/path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # -------------------------
        # Publisher
        # -------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # -------------------------
        # Control loop
        # -------------------------
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("UGV Pure Pursuit Controller (EKF + stabilized CTE) started")

    def path_callback(self, msg: Path):
        # Accept path only once
        if self.path is None:
            self.path = msg.poses
            self.target_index = 0
            self.get_logger().info(f"Path received with {len(self.path)} waypoints")

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
        current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)

        heading_error = wrap_to_pi(target_heading - current_yaw)

        # -------------------------
        # Cross-track error (robot frame)
        # -------------------------
        # Positive CTE means target is to robot's left in robot frame (depending on convention)
        cross_track_error = (
            -math.sin(current_yaw) * dx
            + math.cos(current_yaw) * dy
        )

        # -------------------------
        # Control law (stabilized)
        # -------------------------
        cmd = Twist()

        # Soft-saturate CTE into an "angle" so big CTE doesn't explode steering
        # atan2(cte, L) ~ cte/L for small errors, saturates toward +/- pi/2 for big errors
        cte_angle = math.atan2(cross_track_error, max(1e-3, self.cte_L))

        w_cmd = (self.k_h * heading_error) + (self.k_ct * cte_angle)

        # Clamp
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))

        # Optional smoothing (kills sign-flip chatter)
        # w = (1-alpha)*w_cmd + alpha*prev
        if 0.0 < self.alpha < 1.0:
            w_cmd = (1.0 - self.alpha) * w_cmd + self.alpha * self.prev_w
        self.prev_w = w_cmd

        cmd.angular.z = w_cmd

        # Linear speed: allow 0 when turning hard (prevents forward scrubbing + oscillation)
        turn_ratio = abs(cmd.angular.z) / max(1e-3, self.max_w)
        cmd.linear.x = self.linear_speed * max(0.0, 1.0 - 1.2 * turn_ratio)

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
