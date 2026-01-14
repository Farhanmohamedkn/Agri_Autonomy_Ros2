import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw


class WheelOdometryNode(Node):
    """
    Computes skid-steer wheel odometry from wheel joint positions in /joint_states.

    Assumptions:
      - 2D (x, y, yaw)
      - Left wheels share same rotation; right wheels share same rotation
      - Uses average of front+rear on each side if available
    """

    def __init__(self):
        super().__init__("wheel_odometry")

        # --- parameters
        self.declare_parameter("wheel_radius", 0.25)
        self.declare_parameter("track_width", 0.9)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)

        # Must match your URDF/joint publisher names
        self.declare_parameter("left_wheel_joints", [
            "front_left_wheel_joint",
            "rear_left_wheel_joint",
        ])
        self.declare_parameter("right_wheel_joints", [
            "front_right_wheel_joint",
            "rear_right_wheel_joint",
        ])

        self.r = float(self.get_parameter("wheel_radius").value)
        self.W = float(self.get_parameter("track_width").value)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.left_names = list(self.get_parameter("left_wheel_joints").value)
        self.right_names = list(self.get_parameter("right_wheel_joints").value)

        # --- state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left = None
        self.prev_right = None
        self.prev_stamp = None

        # --- ROS
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odometry", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, "/joint_states", self.js_cb, 50)

        self.get_logger().info(
            f"WheelOdometry started r={self.r}m W={self.W}m, tf={self.publish_tf}, "
            f"left={self.left_names}, right={self.right_names}"
        )

    def _avg_joint_position(self, msg: JointState, joint_names):
        # Build map name->position for quick lookup
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        vals = [name_to_pos[n] for n in joint_names if n in name_to_pos]
        if not vals:
            return None
        return sum(vals) / len(vals)

    def js_cb(self, msg: JointState):
        # Use message time if provided, else node time
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        left = self._avg_joint_position(msg, self.left_names)
        right = self._avg_joint_position(msg, self.right_names)
        if left is None or right is None:
            return

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.prev_stamp = stamp
            return

        # dt
        dt = (stamp.sec - self.prev_stamp.sec) + (stamp.nanosec - self.prev_stamp.nanosec) * 1e-9
        if dt <= 0.0:
            return

        # wheel angle deltas
        dth_l = left - self.prev_left
        dth_r = right - self.prev_right

        self.prev_left = left
        self.prev_right = right
        self.prev_stamp = stamp

        # distance traveled by each side
        dl = self.r * dth_l
        dr = self.r * dth_r

        # skid-steer / diff-drive style
        ds = 0.5 * (dr + dl)
        dyaw = (dr - dl) / self.W

        # integrate pose
        yaw_mid = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw += dyaw

        # normalize yaw
        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi

        vx = ds / dt
        wz = dyaw / dt

        # publish odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(self.yaw)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        # simple covariance (tune later)
        # Larger yaw cov because skid-steer slips
        odom.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  999.0,0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  999.0,0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  999.0,0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.10
            ]

        self.odom_pub.publish(odom)

        # publish TF odom->base_link (optional)
        if self.publish_tf and self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
