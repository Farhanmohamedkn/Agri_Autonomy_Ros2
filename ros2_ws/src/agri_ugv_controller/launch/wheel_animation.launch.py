from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agri_ugv_controller',
            executable='skid_steer_wheel_state_publisher',
            output='screen'
        )
    ])
