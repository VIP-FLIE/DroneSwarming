# ===========================================================================
# swarm.launch.py -- start N PX4 vehicles and N copies of formation_drone.
#
#     ros2 launch swarm_comms swarm.launch.py
#     ros2 launch swarm_comms swarm.launch.py count:=5 follow_distance_m:=6.0
#     ros2 launch swarm_comms swarm.launch.py headless:=true count:=6
#
# TWO TERMINALS HAVE TO BE RUNNING FIRST, in this order:
#
#     MicroXRCEAgent udp4 -p $PX4_UXRCE_DDS_PORT   the PX4 to ROS 2 bridge
#     drone-gcs                                    the ground station link
#
# That variable is 8888 unless you are sharing a machine, where every person
# needs their own. scripts/native/env.sh sets it, along with the instance_base
# argument below. See docs/setup/LAB-SERVER.md.
#
# Neither is started here on purpose. The agent has to be up BEFORE any PX4
# starts, or that PX4's client gives up looking for it and none of its /fmu/
# topics ever appear.
#
# HOW THIS DIFFERS FROM swarm_naive/launch/swarm.launch.py
# --------------------------------------------------------
# Almost not at all, which is the interesting part. Same pad layout, same
# instance numbering, same spawn poses, same staggering. It runs a different
# node and adds ONE launch argument, follow_distance_m, which is declared once,
# read once and passed once. Everything else that differs between the two files
# is wording.
#
# That is the point. Everything which makes this swarm a swarm happens at RUN
# TIME, over /swarm/state, so none of it can be configured here. A launch file
# can say where the drones start. It cannot say what they do about each other.
#
# THE PAD LAYOUT
# --------------
# Drones are lined up along east, `pad_spacing_m` apart, at north 0. Drone 0's
# pad is the origin of the shared swarm frame, and drone 0 is the leader.
#
#     north
#       ^          (the ring the leader flies is out this way)
#       |
#       |     drone0   drone1   drone2
#       +-------o--------o--------o------> east
#            leader   follows0  follows1
#
# CLEANING UP
# -----------
# Ctrl-C stops PX4 and the nodes. Gazebo is started by the first PX4 as a
# detached background process and does not die with it, so this file asks it to
# stop on shutdown. If one survives, `pkill -f 'gz sim'`.
#
# Full guide: docs/swarm/SWARM.md
# ===========================================================================

import os
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# PX4's airframe id for the Gazebo x500 quadrotor, from
# ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500 in PX4 v1.16.1.
PX4_AIRFRAME_X500 = '4001'
PX4_SIM_MODEL = 'gz_x500'

FLIGHT_NODE = 'formation_drone'


def _arguments():
    """Everything you can change from the command line, with its default."""
    return [
        DeclareLaunchArgument(
            'count', default_value='3',
            description='How many drones. Slot 0 leads; every other slot '
                        'follows the one below it. Maximum 10, because '
                        'drone-gcs only heartbeats one block of ten instances, '
                        'instance_base + 1 to instance_base + 10.'),
        DeclareLaunchArgument(
            'pad_spacing_m', default_value='5.0',
            description='Gap between pads along east.'),
        DeclareLaunchArgument(
            'follow_distance_m', default_value='5.0',
            description='How far behind the drone ahead each follower sits. '
                        'The only parameter here that swarm_naive has no use '
                        'for, because it is the only one that describes a '
                        'relationship between two aircraft.'),
        DeclareLaunchArgument(
            'radius_m', default_value='8.0',
            description='Radius of the ring the LEADER flies. The followers '
                        'have no route; they go where the chain takes them.'),
        DeclareLaunchArgument(
            'altitude_m', default_value='4.0',
            description='Altitude of drone 0. Every other drone flies one '
                        'layer_m higher than the one before it.'),
        DeclareLaunchArgument(
            'layer_m', default_value='2.0',
            description='Vertical gap between slots. Still doing real work '
                        'here: the chase rule only constrains the drone '
                        'directly ahead of you, so nothing stops slot 3 from '
                        'flying through where slot 1 is.'),
        DeclareLaunchArgument(
            'laps', default_value='1',
            description='Laps of the ring before the leader calls it a day.'),
        DeclareLaunchArgument(
            'centre_north_m', default_value='',
            description='Centre of the ring, north, in the shared swarm frame. '
                        'Empty means radius_m + pad_spacing_m.'),
        DeclareLaunchArgument(
            'centre_east_m', default_value='',
            description='Centre of the ring, east. Empty means the middle of '
                        'the row of pads.'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true skips Gazebo rendering entirely. Worth it above '
                        'about four drones, and on any machine without a GPU.'),
        DeclareLaunchArgument(
            'world', default_value='default',
            description='Gazebo world, by file name. See docs/simulation/WORLDS.md.'),
        DeclareLaunchArgument(
            'px4_dir', default_value=os.environ.get(
                'PX4_DIR', os.path.expanduser('~/PX4-Autopilot')),
            description='PX4-Autopilot checkout to run.'),
        DeclareLaunchArgument(
            'instance_base', default_value=os.environ.get('PX4_INSTANCE_BASE', '0'),
            description='Added to every SITL instance number, so several '
                        'people can fly on one machine without colliding on '
                        "PX4's fixed per-instance UDP ports. 0 on your own "
                        'machine. On a shared one it comes from DRONE_SLOT, '
                        'see docs/setup/LAB-SERVER.md.'),
        DeclareLaunchArgument(
            'gazebo_boot_s', default_value='30.0',
            description='Head start for the first vehicle, which has to load '
                        'Gazebo before any other vehicle can join its world.'),
        DeclareLaunchArgument(
            'stagger_s', default_value='8.0',
            description='Gap between the remaining vehicles.'),
        DeclareLaunchArgument(
            'px4_output', default_value='log',
            description="Where PX4's own console goes. 'log' writes it to "
                        '~/.ros/log/<run>/launch.log and leaves the terminal '
                        'showing only your '
                        "nodes. 'screen' shows PX4 too."),
    ]


def _launch_swarm(context, *args, **kwargs):
    """Build the per-drone actions.

    An OpaqueFunction rather than a plain list because the number of drones is
    itself a launch argument, and a launch argument has no value until launch is
    running. `perform(context)` turns one into a Python value you can loop over.
    """
    def value(name):
        return LaunchConfiguration(name).perform(context)

    count = int(value('count'))
    pad_spacing_m = float(value('pad_spacing_m'))
    follow_distance_m = float(value('follow_distance_m'))
    radius_m = float(value('radius_m'))
    altitude_m = float(value('altitude_m'))
    layer_m = float(value('layer_m'))
    laps = int(value('laps'))
    headless = value('headless').lower() in ('true', '1', 'yes')
    world = value('world')
    px4_dir = os.path.expanduser(value('px4_dir'))
    instance_base = int(value('instance_base'))
    gazebo_boot_s = float(value('gazebo_boot_s'))
    stagger_s = float(value('stagger_s'))
    px4_output = value('px4_output')

    if not 1 <= count <= 10:
        raise RuntimeError(
            'count must be between 1 and 10. drone-gcs heartbeats a block of '
            'ten SITL instances, so an eleventh drone could never arm.')

    # Blocks are ten wide and start at a multiple of ten, so one person's
    # vehicles can never land on another person's ports. drone-gcs enforces the
    # same rule, and a mismatch between the two is a swarm that launches and
    # then sits on the pad, so it is checked in both places.
    if instance_base < 0 or instance_base > 90 or instance_base % 10:
        raise RuntimeError(
            f'instance_base must be 0, 10, 20 ... 90, got {instance_base}. '
            'It comes from PX4_INSTANCE_BASE, which scripts/native/env.sh '
            'derives from DRONE_SLOT. See docs/setup/LAB-SERVER.md.')

    px4_binary = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    if not os.path.isfile(px4_binary):
        raise RuntimeError(
            f'No PX4 binary at {px4_binary}. Run ./scripts/native/install.sh, '
            'or pass px4_dir:=/path/to/PX4-Autopilot.')

    # PX4 treats this variable as present-or-absent rather than 1-or-0, so even
    # PX4_GZ_STANDALONE=0 makes every vehicle sit waiting for a Gazebo server
    # that nothing started. Children inherit this process's environment.
    os.environ.pop('PX4_GZ_STANDALONE', None)

    centre_north_m = float(value('centre_north_m') or (radius_m + pad_spacing_m))
    centre_east_m = float(value('centre_east_m') or (count - 1) * pad_spacing_m / 2.0)

    actions = [
        LogInfo(msg=(
            f'swarm_comms: {count} drones on /swarm/state. Slot 0 leads a '
            f'{radius_m:.1f} m ring centred on north {centre_north_m:.1f} '
            f'east {centre_east_m:.1f}; the rest chase at '
            f'{follow_distance_m:.1f} m. Nobody moves until everybody is ready. '
            f'SITL instances {instance_base + 1}-{instance_base + count}, so '
            f'topics are under /px4_{instance_base + 1}/fmu/ upward. '
            'MicroXRCEAgent and drone-gcs must already be running.')),
    ]

    for slot in range(count):
        # SITL instances are numbered from 1. Instance 0 would publish on bare
        # /fmu/... with no namespace, which is right for a single vehicle and a
        # special case for every other line in this file.
        #
        # instance_base is 0 unless you are sharing the machine, in which case
        # it moves this whole swarm into your own block of instance numbers,
        # and with it every UDP port PX4 derives from the instance.
        instance = instance_base + slot + 1

        # The pad, in the shared swarm frame. These same two numbers go to
        # Gazebo as a spawn point and to the node as "where I am".
        pad_north_m = 0.0
        pad_east_m = slot * pad_spacing_m

        # PX4_GZ_MODEL_POSE is "x,y,z,roll,pitch,yaw" in GAZEBO'S world frame,
        # which is ENU: x is EAST, y is NORTH, z is UP. PX4's own frame is NED,
        # so east and north come out swapped relative to everything else here.
        # Verified in the simulator: "6,0" spawns 5.98 m east, "0,6" spawns
        # 5.99 m north.
        model_pose = f'{pad_east_m},{pad_north_m}'

        px4_env = {
            'PX4_SYS_AUTOSTART': PX4_AIRFRAME_X500,
            'PX4_SIM_MODEL': PX4_SIM_MODEL,
            'PX4_GZ_MODEL_POSE': model_pose,
            'PX4_GZ_WORLD': world,
        }
        if headless:
            px4_env['HEADLESS'] = '1'

        # -i <instance> is the whole of PX4's multi-vehicle support. From it PX4
        # derives the topic namespace (px4_<instance>), the MAVLink system id
        # (instance + 1), the GCS port (18570 + instance) and the Gazebo model
        # name (x500_<instance>). -d skips the interactive pxh> shell, which
        # there is no terminal for under ros2 launch.
        px4_process = ExecuteProcess(
            cmd=[px4_binary, '-i', str(instance), '-d'],
            name=f'px4_{instance}',
            cwd=px4_dir,
            additional_env=px4_env,
            output=px4_output,
        )

        start_at = 0.0 if slot == 0 else gazebo_boot_s + (slot - 1) * stagger_s
        actions.append(TimerAction(period=start_at, actions=[px4_process]))

        flight_node = Node(
            package='swarm_comms',
            executable=FLIGHT_NODE,
            name=f'drone_{slot}',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Which aircraft to talk to.
                'px4_ns': f'px4_{instance}',
                'system_id': instance + 1,
                # Which drone in the swarm this is. Slot 0 leads.
                'slot': slot,
                'count': count,
                # Where its pad is, in the shared swarm frame. The node needs
                # this to convert its own position into something the other
                # drones can use.
                'home_north_m': pad_north_m,
                'home_east_m': pad_east_m,
                # The leader's route. Passed to every drone even though only
                # slot 0 uses it, so that any drone could lead.
                'centre_north_m': centre_north_m,
                'centre_east_m': centre_east_m,
                'radius_m': radius_m,
                'altitude_m': altitude_m,
                'layer_m': layer_m,
                'laps': laps,
                # The formation.
                'follow_distance_m': follow_distance_m,
            }],
        )

        # Nodes start once every vehicle is up. They are patient either way, so
        # this is for readable output rather than correctness. It matters a
        # little more here than in swarm_naive: a node that starts late holds
        # the whole barrier, because nobody moves until everybody is ready.
        nodes_at = gazebo_boot_s + max(0, count - 1) * stagger_s + 5.0
        actions.append(TimerAction(period=nodes_at, actions=[flight_node]))

    return actions


def _stop_gazebo(context, *args, **kwargs):
    """Ask the Gazebo server to shut down when this launch ends.

    Gazebo is started by the first PX4 through its own startup script, as a
    detached background process, so it outlives the PX4 that started it.
    Leaving one running is worse than it sounds: the next launch finds a world
    already up, joins that one instead of starting a fresh one, and you get a
    field with six drones in it where half of them do not respond.

    This is the polite way to ask. It is addressed to the server itself over
    Gazebo's own service interface rather than matched on a process name, so it
    cannot take down somebody else's simulator. If it misses one anyway,
    `pkill -f 'gz sim'` is the blunt way.

    NOTE THE SHAPE OF THIS, because it is a trap in ROS 2 launch. The obvious
    way to write it is an ExecuteProcess in the OnShutdown handler. That does
    not work: by the time the handler fires, launch is already tearing down and
    will not start a new process, so the action is registered, never runs, and
    logs nothing about why. An OpaqueFunction runs its Python inline instead, so
    a blocking subprocess.run here actually completes before launch exits.

    The stop is asynchronous on Gazebo's side. The service replies immediately
    and the server takes a second or two to actually go, which is normal.
    """
    del context, args, kwargs
    try:
        subprocess.run(
            ['gz', 'service', '-s', '/server_control',
             '--reqtype', 'gz.msgs.ServerControl',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', 'stop: true'],
            check=False, timeout=10,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        # No gz on PATH, or it hung. Not worth failing a shutdown over, and
        # `pkill -f 'gz sim'` is always there.
        pass
    return []


def generate_launch_description():
    return LaunchDescription(
        _arguments() + [
            OpaqueFunction(function=_launch_swarm),
            RegisterEventHandler(
                OnShutdown(on_shutdown=[OpaqueFunction(function=_stop_gazebo)])),
        ])
