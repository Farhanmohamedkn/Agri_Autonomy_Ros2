import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    """
    Convert yaw (rad) to quaternion (z, w), assuming roll=pitch=0.
    """
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class FakeLocalizationNode(Node):

    def __init__(self):
        super().__init__('fake_localization')

        # Parameters
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')

        self.dt = float(self.get_parameter('dt').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)

        # Robot state (in parent_frame)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Latest commanded velocity
        self.cmd_vel = Twist()

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/pose',
            10
        )

        # Timer
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f"Fake localization started (dt={self.dt}s, tf: {self.parent_frame}->{self.child_frame})"
        )

    def cmd_callback(self, msg: Twist):
        self.cmd_vel = msg

    def update(self):
        # Extract velocities
        v = float(self.cmd_vel.linear.x)
        w = float(self.cmd_vel.angular.z)

        # Unicycle kinematic update
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw += w * self.dt

        # Normalize yaw to [-pi, pi]
        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi

        now = self.get_clock().now().to_msg()
        qz, qw = yaw_to_quaternion(self.yaw)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.parent_frame

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        # Publish TF: parent_frame -> child_frame
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
