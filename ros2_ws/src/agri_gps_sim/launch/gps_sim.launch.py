from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='agri_gps_sim',
            executable='enu_gps_sim_node',
            name='gps_sim',
            output='screen'
        ),

    ])
