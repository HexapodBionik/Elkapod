from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_mode = DeclareLaunchArgument(
        "sim_mode",
        default_value="false",
        description="Launch hardware or simulation system",
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("elkapod_description"),
            "config",
            "my_controllers.yaml",
        ]
    )

    motion_manager_config = PathJoinSubstitution(
        [
            FindPackageShare("elkapod_motion_manager"),
            "config",
            "elkapod_motion_manager.yaml",
        ]
    )

    sim_pid_config = PathJoinSubstitution(
        [
            FindPackageShare("elkapod_motion_manager"),
            "config",
            "sim_pid_config.yaml",
        ]
    )

    real_pid_config = PathJoinSubstitution(
        [
            FindPackageShare("elkapod_motion_manager"),
            "config",
            "real_pid_config.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        emulate_tty=True,
    )

    controllers = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["elkapod_ik_controller"],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["imu_broadcaster"],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
                emulate_tty=True,
            ),
        ],
    )

    gait_spawner_sim = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "elkapod_gait_controller",
            "--inactive",
            "--param-file",
            robot_controllers,
            "--param-file",
            motion_manager_config,
            "--param-file",
            sim_pid_config,
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("sim_mode")),
    )

    gait_spawner_real = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "elkapod_gait_controller",
            "--inactive",
            "--param-file",
            robot_controllers,
            "--param-file",
            motion_manager_config,
            "--param-file",
            real_pid_config,
        ],
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("sim_mode")),
    )

    controllers_second_wave = TimerAction(
        period=8.0,
        actions=[
            gait_spawner_sim,
            gait_spawner_real,
        ],
    )

    return LaunchDescription(
        [sim_mode, control_node, controllers, controllers_second_wave]
    )
