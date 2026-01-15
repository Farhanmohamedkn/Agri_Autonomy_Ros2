from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    nav2_params = os.path.join(
        get_package_share_directory('agri_nav2_bringup'),
        'config',
        'nav2_params.yaml'
    )

    nav2_launch = IncludeLaunchDescription(
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

    return LaunchDescription([
        GroupAction([
            # Nav2 will publish to /cmd_vel_raw instead of /cmd_vel
            SetRemap(src='/cmd_vel', dst='/cmd_vel_raw'),

            # velocity_smoother will publish to /cmd_vel (robot listens here)
            SetRemap(src='/cmd_vel_smoothed', dst='/cmd_vel'),

            nav2_launch
        ])
    ])
