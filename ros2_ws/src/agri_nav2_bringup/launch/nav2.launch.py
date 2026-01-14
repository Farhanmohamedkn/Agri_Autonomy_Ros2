from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_params = os.path.join(
        get_package_share_directory('agri_nav2_bringup'),
        'config',
        'nav2_params.yaml'
    )

    return LaunchDescription([

        # Nav2 lifecycle manager + servers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
                'autostart': 'true'
            }.items()
        )
    ])
