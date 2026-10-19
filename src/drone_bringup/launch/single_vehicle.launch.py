"""Launch description for a single PX4-connected vehicle."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Define vehicle and mission nodes with launch-time parameters."""
    namespace = LaunchConfiguration('namespace')
    px4_topic_prefix = LaunchConfiguration('px4_topic_prefix')

    launch_arguments = [
        DeclareLaunchArgument(
            'namespace', default_value='', description='Application ROS namespace'
        ),
        DeclareLaunchArgument(
            'px4_topic_prefix', default_value='/fmu', description='PX4 DDS topic prefix'
        ),
        DeclareLaunchArgument(
            'px4_system_id', default_value='1', description='PX4 MAVLink system ID'
        ),
        DeclareLaunchArgument(
            'enable_flight_commands',
            default_value='false',
            description='Explicitly enable vehicle-control flight actions',
        ),
        DeclareLaunchArgument('setpoint_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('telemetry_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('default_navigation_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('default_position_tolerance_m', default_value='0.5'),
        DeclareLaunchArgument('command_ack_timeout_sec', default_value='2.0'),
        DeclareLaunchArgument('transition_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument('takeoff_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('starting_position_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('action_server_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('child_result_timeout_sec', default_value='90.0'),
        DeclareLaunchArgument('default_waypoint_timeout_sec', default_value='30.0'),
    ]

    vehicle_control = Node(
        package='vehicle_control',
        executable='vehicle_control_node',
        name='vehicle_control',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'px4_topic_prefix': px4_topic_prefix,
                'px4_system_id': ParameterValue(
                    LaunchConfiguration('px4_system_id'), value_type=int
                ),
                'enable_flight_commands': ParameterValue(
                    LaunchConfiguration('enable_flight_commands'), value_type=bool
                ),
                'setpoint_rate_hz': ParameterValue(
                    LaunchConfiguration('setpoint_rate_hz'), value_type=float
                ),
                'telemetry_timeout_sec': ParameterValue(
                    LaunchConfiguration('telemetry_timeout_sec'), value_type=float
                ),
                'default_navigation_timeout_sec': ParameterValue(
                    LaunchConfiguration('default_navigation_timeout_sec'), value_type=float
                ),
                'default_position_tolerance_m': ParameterValue(
                    LaunchConfiguration('default_position_tolerance_m'), value_type=float
                ),
                'command_ack_timeout_sec': ParameterValue(
                    LaunchConfiguration('command_ack_timeout_sec'), value_type=float
                ),
                'transition_timeout_sec': ParameterValue(
                    LaunchConfiguration('transition_timeout_sec'), value_type=float
                ),
                'takeoff_timeout_sec': ParameterValue(
                    LaunchConfiguration('takeoff_timeout_sec'), value_type=float
                ),
            }
        ],
    )

    mission_execution = Node(
        package='mission_execution',
        executable='mission_execution_node',
        name='mission_execution',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'telemetry_timeout_sec': ParameterValue(
                    LaunchConfiguration('telemetry_timeout_sec'), value_type=float
                ),
                'starting_position_timeout_sec': ParameterValue(
                    LaunchConfiguration('starting_position_timeout_sec'), value_type=float
                ),
                'action_server_timeout_sec': ParameterValue(
                    LaunchConfiguration('action_server_timeout_sec'), value_type=float
                ),
                'child_result_timeout_sec': ParameterValue(
                    LaunchConfiguration('child_result_timeout_sec'), value_type=float
                ),
                'default_waypoint_timeout_sec': ParameterValue(
                    LaunchConfiguration('default_waypoint_timeout_sec'), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription([*launch_arguments, vehicle_control, mission_execution])
