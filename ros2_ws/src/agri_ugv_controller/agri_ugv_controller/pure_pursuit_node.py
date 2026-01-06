import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
def get_yaw_from_quaternion(q):
        # Assuming flat ground (z-axis rotation only)
        return math.atan2(
            2.0 * (q.w * q.z),
            1.0 - 2.0 * (q.z * q.z)
        )

class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('linear_speed', 0.5)

        self.lookahead = self.get_parameter('lookahead_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value

        # State
        self.current_pose = None
        self.path = None
        self.target_index = 0

        # Subscribers
        self.create_subscription(Path, '/mission/path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/robot/pose', self.pose_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("UGV Pure Pursuit Controller started")
    

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.target_index = 0

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

        if distance < self.lookahead:
            self.target_index += 1
            return

        # Desired heading
        target_heading = math.atan2(dy, dx)

        # Current heading
        current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)

        # Heading error (normalized)
        heading_error = target_heading - current_yaw

        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Gains
        k_ang = 1.5

        cmd = Twist()

        # Angular velocity (rad/s)
        cmd.angular.z = k_ang * heading_error

        # Forward velocity (slow down when turning)
        cmd.linear.x = self.linear_speed * max(
            0.2, 1.0 - abs(heading_error)
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
