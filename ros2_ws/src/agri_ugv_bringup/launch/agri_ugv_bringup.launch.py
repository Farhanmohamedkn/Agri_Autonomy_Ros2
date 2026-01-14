from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------------------------
    # Launch arguments
    # -------------------------
    use_sim = LaunchConfiguration('use_sim')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation sources (fake localization + GPS sim)'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )

    # -------------------------
    # Package share directories
    # -------------------------
    fake_loc_pkg = get_package_share_directory('agri_fake_localization')
    gps_sim_pkg = get_package_share_directory('agri_gps_sim')
    localization_pkg = get_package_share_directory('agri_localization')
    mission_pkg = get_package_share_directory('agri_mission_planner')
    controller_pkg = get_package_share_directory('agri_ugv_controller')
    description_pkg = get_package_share_directory('agri_ugv_description')

    # -------------------------
    # Simulation localization stack
    # -------------------------
    fake_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fake_loc_pkg, 'launch', 'fake_localization.launch.py')
        ),
        condition=IfCondition(use_sim)
    )

    pose_cov_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'pose_cov_wrapper.launch.py')
        ),
        condition=IfCondition(use_sim)
    )

    gps_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gps_sim_pkg, 'launch', 'gps_sim.launch.py')
        ),
        condition=IfCondition(use_sim)
    )

    # -------------------------
    # EKF localization
    # -------------------------
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'ekf.launch.py')
        )
    )

    # -------------------------
    # Mission planner
    # -------------------------
    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mission_pkg, 'launch', 'mission.launch.py')
        )
    )

    # -------------------------
    # Controller
    # -------------------------
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'controller.launch.py')
        )
    )
    
    # -------------------------
    # robot state, joint state publisher and URDF stack
    # -------------------------
    robot_state_pub = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(description_pkg, 'launch', 'robot_state_publisher.launch.py')
    )
    )
    
    wheel_anim_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(controller_pkg, 'launch', 'wheel_animation.launch.py')
    )
    )



    # -------------------------
    # RViz (optional)
    # -------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(localization_pkg, 'rviz', 'agri_ugv.rviz')],
        condition=IfCondition(use_rviz)
    )


    # -------------------------
    # Final launch description
    # -------------------------
    return LaunchDescription([
        declare_use_sim,
        declare_use_rviz,

        robot_state_pub,
        

        fake_localization_launch,
        pose_cov_wrapper_launch,
        gps_sim_launch,

        ekf_launch,
        mission_launch,
        controller_launch,
        wheel_anim_launch,

        rviz_node,
    ])
