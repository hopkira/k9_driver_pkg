"""K9 RoboClaw controller bring-up; robot description normally owned by k9_system_pkg."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def create_drive_nodes(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('normal', 'commissioning'):
        raise RuntimeError('Invalid drive mode %r: choose normal or commissioning' % mode)

    dry_run = LaunchConfiguration('dry_run').perform(context)
    inhibited = LaunchConfiguration('start_inhibited').perform(context)
    device = LaunchConfiguration('device').perform(context)
    publish_description = LaunchConfiguration('publish_robot_description').perform(context).lower() in ('true', '1', 'yes', 'on')
    nodes = []

    if publish_description:
        # Standalone *bench* drive test only. k9_system_pkg publishes the full model otherwise.
        bench_xacro = PathJoinSubstitution([
            FindPackageShare('k9_drive_pkg'), 'urdf', 'k9_drive_bench.urdf.xacro'])
        description = Command([
            FindExecutable(name='xacro'), ' ', bench_xacro,
            ' dry_run:=', dry_run,
            ' start_inhibited:=', inhibited,
            ' device:=', device,
        ])
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': ParameterValue(description, value_type=str)}]))

    # On both hosts this manager consumes the FULL model provided by k9_system_pkg.
    nodes.append(Node(
        package='controller_manager', executable='ros2_control_node',
        name='controller_manager', output='screen',
        parameters=[{'update_rate': 30}],
        remappings=[('robot_description', '/robot_description')]))

    controller_yaml = PathJoinSubstitution([
        FindPackageShare('k9_drive_pkg'),
        'config',
        'drive_controller.yaml',
    ])

    # Start each controller through its own spawner.
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_yaml,
        ],
    ))

    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_yaml,
        ],
    ))
    return [LogInfo(msg=(
        f'K9 drive mode={mode}, dry_run={dry_run}, start_inhibited={inhibited}, '
        f'device={device}, local_bench_description={publish_description}'))] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='normal',
                              description='normal or commissioning; both run the same controllers'),
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='Do not open physical RoboClaw when true'),
        DeclareLaunchArgument('start_inhibited', default_value='true',
                              description='Keep physical drive inhibited on startup'),
        DeclareLaunchArgument('device', default_value='/dev/roboclaw'),
        DeclareLaunchArgument('publish_robot_description', default_value='false',
                              description='Bench-only fallback publisher; never enable under k9_system_pkg'),
        OpaqueFunction(function=create_drive_nodes),
    ])
