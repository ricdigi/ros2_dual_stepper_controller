from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch argument for RViz config
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("ros2_dual_stepper_controller"),
                "config",
                "rviz_config.rviz"
            ]),
            description="Full path to the RViz config file"
        ),
    ]

    rviz_config = LaunchConfiguration("rviz_config")

    # Teleoperation node
    teleop_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_keyboard",
        output="screen",
        remappings=[
            ("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")
        ]
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config]
    )

    return LaunchDescription(declared_arguments + [teleop_node, rviz_node])
