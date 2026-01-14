import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class SkidSteerWheelStatePublisher(Node):

    def __init__(self):
        super().__init__('skid_steer_wheel_state_publisher')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('wheel_radius', 0.25)
        self.declare_parameter('track_width', 0.9)

        self.r = self.get_parameter('wheel_radius').value
        self.W = self.get_parameter('track_width').value

        # -------------------------
        # State
        # -------------------------
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.last_time = self.get_clock().now()

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.cmd = Twist()

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info("Skid-steer wheel state publisher started")

    def cmd_vel_callback(self, msg: Twist):
        self.cmd = msg

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        v = self.cmd.linear.x
        w = self.cmd.angular.z

        # Skid-steer kinematics
        omega_left = (v - w * self.W / 2.0) / self.r
        omega_right = (v + w * self.W / 2.0) / self.r

        # Integrate wheel angles
        self.left_pos += omega_left * dt
        self.right_pos += omega_right * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [
            'front_left_wheel_joint',
            'rear_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]
        js.position = [
            self.left_pos,
            self.left_pos,
            self.right_pos,
            self.right_pos
        ]

        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = SkidSteerWheelStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
