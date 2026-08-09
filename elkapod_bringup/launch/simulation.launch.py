import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_launch_path = os.path.join(
        get_package_share_directory("elkapod_gazebo"),
        "launch",
        "elkapod.gazebo.launch.py",
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("elkapod_bringup"),
                    "launch",
                    "rsp.launch.py",
                )
            ]
        ),
        launch_arguments={"sim_mode": "true"}.items(),
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("elkapod_bringup"),
                    "launch",
                    "controllers.launch.py",
                )
            ]
        ),
        launch_arguments={"sim_mode": "true"}.items(),
    )

    motion_manager_launch_path = os.path.join(
        get_package_share_directory("elkapod_motion_manager"),
        "launch",
        "elkapod_motion_manager.launch.py",
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
    )

    motion_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(motion_manager_launch_path),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    delayed_actions = TimerAction(
        period=10.0,
        actions=[
            motion_manager_launch,
        ],
    )

    return LaunchDescription(
        [
            # gazebo_launch,
            rsp,
            ros2_control_launch,
            delayed_actions,
        ]
    )
