from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="agri_wheel_odometry",
            executable="wheel_odometry_node",
            name="wheel_odometry",
            output="screen",
            parameters=[{
                # keep consistent with your existing skid params :contentReference[oaicite:7]{index=7}
                "wheel_radius": 0.25,
                "track_width": 0.9,
                "publish_tf": True,
                "odom_frame": "odom",
                "base_frame": "base_link",
            }]
        )
    ])
