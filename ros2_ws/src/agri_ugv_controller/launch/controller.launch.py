from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agri_ugv_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                # -------------------------
                # Speed limits
                # -------------------------
                'linear_speed': 0.5,          # m/s
                'max_angular_speed': 0.6,     # rad/s (skid-steer safe)

                # -------------------------
                # Path tracking
                # -------------------------
                'waypoint_tolerance': 0.5,    # m
                'heading_gain': 1.2,
                'cross_track_gain': 1.0,

                # -------------------------
                # Stability tuning
                # -------------------------
                'cte_lookahead': 1.0,         # m (try 1.5 if still twitchy)
                'angular_smoothing_alpha': 0.4
            }]
        )
    ])
