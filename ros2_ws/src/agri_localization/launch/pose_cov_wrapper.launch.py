from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='agri_localization',
            executable='pose_cov_wrapper_node',
            name='pose_cov_wrapper',
            output='screen'
        ),

    ])
