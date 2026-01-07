import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist


def get_yaw_from_quaternion(q):
    """
    Extract yaw from quaternion assuming flat ground
    """
    return math.atan2(
        2.0 * (q.w * q.z),
        1.0 - 2.0 * (q.z * q.z)
    )


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.waypoint_tol = self.get_parameter('waypoint_tolerance').value
        self.max_w = self.get_parameter('max_angular_speed').value

        # State
        self.current_pose = None
        self.path = None
        self.target_index = 0

        # Subscribers
        self.create_subscription(
            Path, '/mission/path', self.path_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/robot/pose', self.pose_callback, 10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("UGV Pure Pursuit Controller started")

    def path_callback(self, msg: Path):
        # Accept the path ONLY once
        if self.path is None:
            self.path = msg.poses
            self.target_index = 0
            self.get_logger().info(
                f"Path received with {len(self.path)} waypoints"
            )

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose

    def control_loop(self):
        if self.current_pose is None or self.path is None:
            return

        if self.target_index >= len(self.path):
            self.stop_robot()
            return

        target_pose = self.path[self.target_index].pose

        # Position error
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        # Waypoint reached
        if distance < self.waypoint_tol:
            self.target_index += 1
            return

        # Desired heading
        target_heading = math.atan2(dy, dx)

        # Current heading
        current_yaw = get_yaw_from_quaternion(
            self.current_pose.orientation
        )

        # Heading error
        heading_error = target_heading - current_yaw

        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2.0 * math.pi
        while heading_error < -math.pi:
            heading_error += 2.0 * math.pi

        cmd = Twist()

        # Angular velocity (P controller)
        cmd.angular.z = 1.5 * heading_error
        cmd.angular.z = max(
            -self.max_w,
            min(self.max_w, cmd.angular.z)
        )

        # Forward velocity (slow down when turning)
        cmd.linear.x = self.linear_speed * max(
            0.3, 1.0 - abs(heading_error)
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
