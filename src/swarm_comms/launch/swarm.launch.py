import os
import shutil
import subprocess

from launch import LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PX4_AIRFRAME_X500 = '4001'
PX4_SIM_MODEL = 'gz_x500'
FLIGHT_NODE = 'formation_drone'
MAX_VEHICLE_COUNT = 255
AGENT_STARTUP_DELAY_S = 2.0
GAZEBO_STARTUP_DELAY_S = 30.0
VEHICLE_STAGGER_S = 8.0
NODE_STARTUP_DELAY_S = 5.0


def _arguments() -> list[DeclareLaunchArgument]:
    return [
        DeclareLaunchArgument('count', default_value='3'),
        DeclareLaunchArgument('pad_spacing_m', default_value='5.0'),
        DeclareLaunchArgument('follow_distance_m', default_value='5.0'),
        DeclareLaunchArgument('radius_m', default_value='8.0'),
        DeclareLaunchArgument('altitude_m', default_value='4.0'),
        DeclareLaunchArgument('layer_m', default_value='2.0'),
        DeclareLaunchArgument('laps', default_value='1'),
        DeclareLaunchArgument('centre_north_m', default_value=''),
        DeclareLaunchArgument('centre_east_m', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value='default'),
    ]


def _launch_swarm(context: LaunchContext) -> list[Action]:
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    count = int(value('count'))
    if not 1 <= count <= MAX_VEHICLE_COUNT:
        raise RuntimeError(f'count must be between 1 and {MAX_VEHICLE_COUNT}')

    px4_binary = shutil.which('px4-gazebo')
    if px4_binary is None:
        raise RuntimeError('px4-gazebo is missing; run this launch inside the project container')

    pad_spacing_m = float(value('pad_spacing_m'))
    follow_distance_m = float(value('follow_distance_m'))
    radius_m = float(value('radius_m'))
    altitude_m = float(value('altitude_m'))
    layer_m = float(value('layer_m'))
    laps = int(value('laps'))
    headless = value('headless').lower() in ('true', '1', 'yes')
    world = value('world')
    centre_north_m = float(value('centre_north_m') or radius_m + pad_spacing_m)
    centre_east_m = float(value('centre_east_m') or (count - 1) * pad_spacing_m / 2.0)

    # PX4 treats PX4_GZ_STANDALONE as present-or-absent, so remove inherited values.
    os.environ.pop('PX4_GZ_STANDALONE', None)

    actions: list[Action] = [
        LogInfo(msg=(
            f'swarm_comms: {count} drone(s), leader/follower routes on /swarm/state, '
            'PX4 instances 0 onward')),
    ]

    for slot in range(count):
        instance = slot
        px4_namespace = f'px4_{instance}' if instance else ''
        pad_north_m = 0.0
        pad_east_m = slot * pad_spacing_m
        px4_environment = {
            'PX4_SYS_AUTOSTART': PX4_AIRFRAME_X500,
            'PX4_SIM_MODEL': PX4_SIM_MODEL,
            'PX4_GZ_MODEL_POSE': f'{pad_east_m},{pad_north_m}',
            'PX4_GZ_WORLD': world,
        }
        if headless:
            px4_environment['HEADLESS'] = '1'

        px4_process = ExecuteProcess(
            cmd=[px4_binary, '-i', str(instance), '-d'],
            name=f'px4_{instance}',
            additional_env=px4_environment,
            output='screen',
        )
        px4_start_delay_s = AGENT_STARTUP_DELAY_S
        if slot > 0:
            px4_start_delay_s += GAZEBO_STARTUP_DELAY_S + (slot - 1) * VEHICLE_STAGGER_S
        actions.append(TimerAction(period=px4_start_delay_s, actions=[px4_process]))

        flight_node = Node(
            package='swarm_comms',
            executable=FLIGHT_NODE,
            name=f'drone_{slot}',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'px4_ns': px4_namespace,
                'system_id': instance + 1,
                'slot': slot,
                'count': count,
                'home_north_m': pad_north_m,
                'home_east_m': pad_east_m,
                'centre_north_m': centre_north_m,
                'centre_east_m': centre_east_m,
                'radius_m': radius_m,
                'altitude_m': altitude_m,
                'layer_m': layer_m,
                'laps': laps,
                'follow_distance_m': follow_distance_m,
            }],
        )
        node_start_delay_s = (
            AGENT_STARTUP_DELAY_S
            + GAZEBO_STARTUP_DELAY_S
            + max(0, count - 1) * VEHICLE_STAGGER_S
            + NODE_STARTUP_DELAY_S
        )
        actions.append(TimerAction(period=node_start_delay_s, actions=[flight_node]))

    return actions


def _stop_gazebo(context: LaunchContext) -> list[Action]:
    try:
        subprocess.run(
            [
                'gz', 'service', '-s', '/server_control',
                '--reqtype', 'gz.msgs.ServerControl',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', 'stop: true',
            ],
            check=False,
            timeout=10,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.SubprocessError):
        pass
    return []


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        _arguments()
        + [
            RegisterEventHandler(
                OnShutdown(on_shutdown=[OpaqueFunction(function=_stop_gazebo)])
            ),
            ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                name='micro_xrce_agent',
                output='screen',
            ),
            OpaqueFunction(function=_launch_swarm),
        ]
    )
