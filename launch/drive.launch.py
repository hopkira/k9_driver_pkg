from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Jazzy pattern: controller_manager owns manager parameters only. Controller
    # parameters are supplied by the spawner with --param-file below.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[{"update_rate": 30}],
        remappings=[("robot_description", "/robot_description")],
        output="screen",
    )

    # Current Jazzy ros2_control demo pattern: load both controllers through the
    # spawner and explicitly give it the controller parameter file.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="k9_controller_spawner",
        arguments=[
            "joint_state_broadcaster",
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controller_yaml,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="If true, exercise ros2_control without opening /dev/roboclaw.",
            ),
            DeclareLaunchArgument(
                "start_inhibited",
                default_value="true",
                description="Start drive software-inhibited after hardware activation.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/roboclaw",
                description="Exclusive RoboClaw serial device.",
            ),
            LogInfo(
                msg=[
                    "K9 drive launch: dry_run=",
                    dry_run,
                    ", start_inhibited=",
                    start_inhibited,
                    ", device=",
                    device,
                ]
            ),
            robot_state_publisher,
            control_node,
            controller_spawner,
        ]
    )
