import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from .coverage_planner import LawnMowerPlanner


class MissionPlannerNode(Node):

    def __init__(self):
        super().__init__('agri_mission_planner')

        # Field parameters (meters)
        self.declare_parameter('field_length', 80.0)
        self.declare_parameter('field_width', 40.0)
        self.declare_parameter('row_spacing', 5.0)

        field_length = self.get_parameter('field_length').value
        field_width = self.get_parameter('field_width').value
        row_spacing = self.get_parameter('row_spacing').value

        # Planner
        planner = LawnMowerPlanner(
            field_length,
            field_width,
            row_spacing
        )

        path_points = planner.generate_path()

        # Publisher
        self.path_pub = self.create_publisher(
            Path,
            '/mission/path',
            10
        )

        # Build Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        for x, y in path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # UGV stays on ground
            pose.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose)

        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info(
            f"UGV coverage path ready with {len(self.path_msg.poses)} poses"
        )

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
