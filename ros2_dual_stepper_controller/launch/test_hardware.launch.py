from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically."),
    ]

    gui = LaunchConfiguration("gui")

    # URDF from Xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ros2_dual_stepper_controller"),
            "description", "urdf", "dual_stepper.urdf.xacro"
        ])
    ])
    robot_description = {"robot_description": robot_description_content}

    # Controllers config
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ros2_dual_stepper_controller"),
        "config", "diff_drive_controllers.yaml"
    ])

    # Core control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # State publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Diff drive controller (you named it dual_stepper_base_controller)
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dual_stepper_base_controller", "--controller-manager", "/controller_manager"],
    )

    # RViz launch (delayed after joint_state_broadcaster to avoid TF errors)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(gui),
    )

    delay_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    # Optional: force robot_controller to launch before joint_state_broadcaster
    delay_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription(declared_arguments + [
        robot_state_pub_node,
        control_node,
        robot_controller_spawner,
        delay_joint_state,
        delay_rviz,
    ])
