from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    dry_run = LaunchConfiguration("dry_run")
    start_inhibited = LaunchConfiguration("start_inhibited")
    device = LaunchConfiguration("device")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("k9_drive_pkg"), "urdf", "k9_drive_bench.urdf.xacro"]
    )
    controller_yaml = PathJoinSubstitution(
        [FindPackageShare("k9_drive_pkg"), "config", "drive_controller.yaml"]
    )

    robot_description_content = Command(
        [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " dry_run:=",
                dry_run,
                " start_inhibited:=",
                start_inhibited,
                " device:=",
                device,
        ]
    )
    # Xacro expands to XML text.  Explicitly type the launch substitution as a
    # string; otherwise launch_ros attempts to parse the XML as YAML.
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Current Jazzy ros2_control pattern: robot_state_publisher publishes the
    # robot description; controller_manager consumes it. Keep controller params
    # separate rather than passing robot_description directly to control_node.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[controller_yaml],
        remappings=[("robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            # Safety default: dry-run and inhibited unless the operator explicitly overrides both.
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="If true, exercise ros2_control without opening /dev/roboclaw.",
            ),
            DeclareLaunchArgument(
                "start_inhibited",
                default_value="true",
                description="Start the drive hardware software-inhibited even after controller activation.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/roboclaw",
                description="Exclusive RoboClaw serial device.",
            ),
            robot_state_publisher,
            control_node,
            joint_state_spawner,
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_spawner, on_exit=[drive_spawner])
            ),
        ]
    )
