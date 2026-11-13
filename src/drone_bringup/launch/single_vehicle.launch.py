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
    log_level = LaunchConfiguration('log_level')

    launch_arguments = [
        DeclareLaunchArgument(
            'namespace', default_value='', description='Application ROS namespace'
        ),
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='',
            description='Plan assignment ID; empty derives it from the namespace',
        ),
        DeclareLaunchArgument('maximum_plan_altitude_m', default_value='30.0'),
        DeclareLaunchArgument('maximum_plan_distance_m', default_value='100.0'),
        DeclareLaunchArgument(
            'px4_topic_prefix', default_value='/fmu', description='PX4 DDS topic prefix'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Runtime ROS log threshold for all application nodes',
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
        DeclareLaunchArgument('max_orbit_radius_m', default_value='20.0'),
        DeclareLaunchArgument('max_orbit_speed_mps', default_value='3.0'),
        DeclareLaunchArgument('max_orbit_center_distance_m', default_value='100.0'),
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
        arguments=['--ros-args', '--log-level', log_level],
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
                'max_orbit_radius_m': ParameterValue(
                    LaunchConfiguration('max_orbit_radius_m'), value_type=float
                ),
                'max_orbit_speed_mps': ParameterValue(
                    LaunchConfiguration('max_orbit_speed_mps'), value_type=float
                ),
                'max_orbit_center_distance_m': ParameterValue(
                    LaunchConfiguration('max_orbit_center_distance_m'), value_type=float
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

    mission_planner = Node(
        package='mission_planner',
        executable='mission_planner_node',
        name='mission_planner',
        namespace=namespace,
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {
                'maximum_altitude_m': ParameterValue(
                    LaunchConfiguration('maximum_plan_altitude_m'), value_type=float
                ),
                'maximum_horizontal_distance_m': ParameterValue(
                    LaunchConfiguration('maximum_plan_distance_m'), value_type=float
                ),
                'max_orbit_radius_m': ParameterValue(
                    LaunchConfiguration('max_orbit_radius_m'), value_type=float
                ),
                'max_orbit_speed_mps': ParameterValue(
                    LaunchConfiguration('max_orbit_speed_mps'), value_type=float
                ),
                'max_orbit_center_distance_m': ParameterValue(
                    LaunchConfiguration('max_orbit_center_distance_m'), value_type=float
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
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {
                'telemetry_timeout_sec': ParameterValue(
                    LaunchConfiguration('telemetry_timeout_sec'), value_type=float
                ),
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'maximum_plan_altitude_m': ParameterValue(
                    LaunchConfiguration('maximum_plan_altitude_m'), value_type=float
                ),
                'maximum_plan_distance_m': ParameterValue(
                    LaunchConfiguration('maximum_plan_distance_m'), value_type=float
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

    return LaunchDescription(
        [*launch_arguments, vehicle_control, mission_planner, mission_execution]
    )
