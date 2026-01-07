import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist


def yaw_to_quaternion(yaw):
    """
    Convert yaw angle (rad) to quaternion (z, w),
    assuming flat ground (roll = pitch = 0)
    """
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class FakeLocalizationNode(Node):

    def __init__(self):
        super().__init__('fake_localization')

        # Parameters
        self.declare_parameter('dt', 0.1)
        self.dt = self.get_parameter('dt').value

        # Robot state (map frame)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Latest commanded velocity
        self.cmd_vel = Twist()

        # Subscribers
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/pose',
            10
        )

        # Timer
        self.timer = self.create_timer(self.dt, self.update_pose)

        self.get_logger().info("Fake localization node started")

    def cmd_callback(self, msg: Twist):
        self.cmd_vel = msg

    def update_pose(self):
        # Extract velocities
        v = self.cmd_vel.linear.x
        w = self.cmd_vel.angular.z

        # Unicycle kinematic model
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw += w * self.dt

        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi

        # Build pose message
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        # Orientation from yaw
        qz, qw = yaw_to_quaternion(self.yaw)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
