from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('agri_ugv_description')
    urdf = os.path.join(pkg_share, 'urdf', 'agri_ugv.urdf')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf],
            parameters=[{'use_sim_time': True}]
        )
    ])
