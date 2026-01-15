import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from .coverage_planner import LawnMowerPlanner


class MissionPlannerNode(Node):

    def __init__(self):
        super().__init__('agri_mission_planner')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('field_length', 100.0)
        self.declare_parameter('field_width', 50.0)
        self.declare_parameter('row_spacing', 6.0)
        self.declare_parameter('start_delay', 5.0)
        self.declare_parameter('goal_tolerance', 0.8)

        field_length = self.get_parameter('field_length').value
        field_width = self.get_parameter('field_width').value
        row_spacing = self.get_parameter('row_spacing').value
        self.start_delay = self.get_parameter('start_delay').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # -------------------------
        # Robot pose (EKF)
        # -------------------------
        self.current_pose = None
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # -------------------------
        # Coverage planner
        # -------------------------
        planner = LawnMowerPlanner(field_length, field_width, row_spacing)
        self.waypoints = planner.generate_path()
        self.current_index = 0

        self.get_logger().info(
            f"Coverage mission with {len(self.waypoints)} waypoints"
        )

        # -------------------------
        # Path publisher (RViz)
        # -------------------------
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.path_pub = self.create_publisher(
            Path,
            '/mission/path',
            path_qos
        )

        self.path_repub_count = 0
        self.path_repub_timer = self.create_timer(
            0.5,
            self._republish_path
        )

        # -------------------------
        # Nav2 action client
        # -------------------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 action server available")

        # -------------------------
        # Delayed mission start
        # -------------------------
        self.start_timer = self.create_timer(
            self.start_delay,
            self.start_mission
        )

    # -------------------------
    # Odometry callback
    # -------------------------
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def distance_to_goal(self, x, y):
        if self.current_pose is None:
            return float('inf')

        dx = self.current_pose.position.x - x
        dy = self.current_pose.position.y - y
        return math.hypot(dx, dy)

    # -------------------------
    # Publish full coverage path
    # -------------------------
    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)

    def _republish_path(self):
        self.publish_path()
        self.path_repub_count += 1

        if self.path_repub_count == 1:
            self.get_logger().info("Published coverage path to /mission/path")

        if self.path_repub_count >= 4:
            self.path_repub_timer.cancel()

    # -------------------------
    # Start mission
    # -------------------------
    def start_mission(self):
        self.start_timer.cancel()
        self.send_next_goal()

    # -------------------------
    # Send Nav2 goal
    # -------------------------
    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ Coverage mission completed")
            return

        x, y = self.waypoints[self.current_index]

        if math.hypot(x, y) < self.goal_tolerance:
            self.get_logger().info(
                f"Skipping waypoint {self.current_index + 1} (too close)"
            )
            self.current_index += 1
            self.send_next_goal()
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Sending goal {self.current_index + 1}/{len(self.waypoints)} "
            f"→ x={x:.2f}, y={y:.2f}"
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # -------------------------
    # Goal response
    # -------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected by Nav2")
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    # -------------------------
    # Goal result (FIXED)
    # -------------------------
    def goal_result_callback(self, future):
        status = future.result().status
        x, y = self.waypoints[self.current_index]
        dist = self.distance_to_goal(x, y)

        if dist < self.goal_tolerance:
            self.get_logger().info(
                f"✅ Waypoint {self.current_index + 1} reached "
                f"(dist={dist:.2f} m, status={status})"
            )
        else:
            self.get_logger().warn(
                f"⚠️ Waypoint {self.current_index + 1} aborted "
                f"(dist={dist:.2f} m, status={status}) — skipping"
            )

        self.current_index += 1
        self.send_next_goal()

    # -------------------------
    # Feedback
    # -------------------------
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Distance remaining: {feedback.distance_remaining:.2f} m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
