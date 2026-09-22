# ===========================================================================
# swarm.launch.py -- start N PX4 vehicles and N copies of waypoint_drone.
#
#     ros2 launch swarm_naive swarm.launch.py
#     ros2 launch swarm_naive swarm.launch.py count:=5 radius_m:=12.0
#     ros2 launch swarm_naive swarm.launch.py headless:=true count:=6
#
# TWO TERMINALS HAVE TO BE RUNNING FIRST, in this order:
#
#     MicroXRCEAgent udp4 -p 8888     the PX4 to ROS 2 bridge
#     drone-gcs                       the ground station link PX4 requires
#
# Neither is started here on purpose. The agent has to be up BEFORE any PX4
# starts, or that PX4's client gives up looking for it and none of its /fmu/
# topics ever appear. Owning them in this file would hide that ordering, and
# both are shared infrastructure that outlives any one flight. Watch drone-gcs
# while the swarm boots: it prints one line per vehicle, and if that count stops
# short of `count`, the vehicles it never names are the ones that will sit on
# their pads reporting "Arming denied".
#
# WHAT A SWARM NEEDS THAT ONE DRONE DOES NOT
# ------------------------------------------
# Every drone is a separate PX4 process, exactly as it is in the field where
# each airframe carries its own autopilot. Three things become per vehicle, and
# all three are decided here:
#
#   instance    ->  topic namespace     /px4_<instance>/fmu/...
#   instance    ->  MAVLink system id   instance + 1
#   pad offset  ->  where it spawns AND what the node is told about itself
#
# The last one is the interesting one. PX4 measures position from each
# vehicle's own pad, so a shared flight plan only works if every node knows
# where its pad sits in the shared frame. This file is the single place that
# knows both, which is what keeps the two from drifting apart.
#
# THE PAD LAYOUT
# --------------
# Drones are lined up along east, `pad_spacing_m` apart, at north 0. Drone 0's
# pad is the origin of the shared swarm frame, so its pad offset is (0, 0) and
# it is the one vehicle whose local frame and the swarm frame agree.
#
#     north
#       ^          (the ring the swarm flies is out this way)
#       |
#       |     drone0   drone1   drone2
#       +-------o--------o--------o------> east
#             (0,0)   (0,5)    (0,10)
#
# CLEANING UP
# -----------
# Ctrl-C stops PX4 and the nodes. Gazebo is started by the first PX4 through its
# own startup script, as a detached background process, so it does NOT die with
# it. This file asks it to stop on shutdown. If one survives anyway, the symptom
# is memorable: the next launch finds a world already running and spawns the new
# drones into the OLD one. Kill it by hand with
#
#     pkill -f 'gz sim'
#
# Full guide: docs/SWARM.md
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
# ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500 in PX4 v1.16.1. That
# file is also where `param set-default NAV_DLL_ACT 2` lives, which is why
# drone-gcs is not optional.
PX4_AIRFRAME_X500 = '4001'
PX4_SIM_MODEL = 'gz_x500'

# The node this launch file runs, one copy per vehicle.
FLIGHT_NODE = 'waypoint_drone'


def _arguments():
    """Everything you can change from the command line, with its default."""
    return [
        DeclareLaunchArgument(
            'count', default_value='3',
            description='How many drones. Each one is a separate PX4 process, '
                        'so this is also how many CPU cores you would like to '
                        'have. Maximum 10, because drone-gcs only heartbeats '
                        'SITL instances 0-9.'),
        DeclareLaunchArgument(
            'pad_spacing_m', default_value='5.0',
            description='Gap between pads along east. Wide enough that the '
                        'downwash and the position noise of one vehicle do not '
                        'reach the next.'),
        DeclareLaunchArgument(
            'radius_m', default_value='8.0',
            description='Radius of the ring the swarm flies.'),
        DeclareLaunchArgument(
            'altitude_m', default_value='4.0',
            description='Altitude of drone 0. Every other drone flies one '
                        'layer_m higher than the one before it.'),
        DeclareLaunchArgument(
            'layer_m', default_value='2.0',
            description='Vertical gap between slots. This is the ONLY thing '
                        'keeping these drones apart, because nothing in this '
                        'package knows where any other drone is. Set it to 0.0 '
                        'to see what that costs.'),
        DeclareLaunchArgument(
            'laps', default_value='1',
            description='Laps of the ring before coming home.'),
        DeclareLaunchArgument(
            'centre_north_m', default_value='',
            description='Centre of the ring, north, in the shared swarm frame. '
                        'Empty means radius_m + pad_spacing_m, which puts the '
                        'ring just north of the pads.'),
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
            description='Gazebo world, by file name. See docs/WORLDS.md.'),
        DeclareLaunchArgument(
            'px4_dir', default_value=os.environ.get(
                'PX4_DIR', os.path.expanduser('~/PX4-Autopilot')),
            description='PX4-Autopilot checkout to run.'),
        DeclareLaunchArgument(
            'gazebo_boot_s', default_value='30.0',
            description='Head start for the first vehicle, which has to load '
                        'Gazebo before any other vehicle can join its world. '
                        'Raise it on a slow machine or a heavy world.'),
        DeclareLaunchArgument(
            'stagger_s', default_value='8.0',
            description='Gap between the remaining vehicles. They only need to '
                        'not spawn on top of each other.'),
        DeclareLaunchArgument(
            'px4_output', default_value='log',
            description="Where PX4's own console goes. 'log' writes it to "
                        '~/.ros/log/<run>/launch.log and leaves the terminal '
                        'showing only your '
                        "nodes. 'screen' shows PX4 too, which is what you want "
                        'when a vehicle will not boot.'),
    ]


def _launch_swarm(context, *args, **kwargs):
    """Build the per-drone actions.

    An OpaqueFunction rather than a plain list because the number of drones is
    itself a launch argument, and a launch argument is a substitution that has
    no value until launch is running. `perform(context)` is what turns one into
    a Python value you can loop over.
    """
    def value(name):
        return LaunchConfiguration(name).perform(context)

    count = int(value('count'))
    pad_spacing_m = float(value('pad_spacing_m'))
    radius_m = float(value('radius_m'))
    altitude_m = float(value('altitude_m'))
    layer_m = float(value('layer_m'))
    laps = int(value('laps'))
    headless = value('headless').lower() in ('true', '1', 'yes')
    world = value('world')
    px4_dir = os.path.expanduser(value('px4_dir'))
    gazebo_boot_s = float(value('gazebo_boot_s'))
    stagger_s = float(value('stagger_s'))
    px4_output = value('px4_output')

    if not 1 <= count <= 10:
        raise RuntimeError(
            'count must be between 1 and 10. drone-gcs heartbeats SITL '
            'instances 0-9 only, so an eleventh drone could never arm.')

    px4_binary = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    if not os.path.isfile(px4_binary):
        raise RuntimeError(
            f'No PX4 binary at {px4_binary}. Run ./scripts/native/install.sh, '
            'or pass px4_dir:=/path/to/PX4-Autopilot.')

    # PX4 treats this variable as present-or-absent rather than 1-or-0, so even
    # PX4_GZ_STANDALONE=0 makes every vehicle sit waiting for a Gazebo server
    # that nothing started. Children inherit this process's environment, so
    # removing it here removes it for all of them.
    os.environ.pop('PX4_GZ_STANDALONE', None)

    # The ring sits just north of the row of pads, centred on it, unless you
    # said otherwise.
    centre_north_m = float(value('centre_north_m') or (radius_m + pad_spacing_m))
    centre_east_m = float(value('centre_east_m') or (count - 1) * pad_spacing_m / 2.0)

    actions = [
        LogInfo(msg=(
            f'swarm_naive: {count} drones, {radius_m:.1f} m ring centred on '
            f'north {centre_north_m:.1f} east {centre_east_m:.1f}, '
            f'altitudes {altitude_m:.1f} m to '
            f'{altitude_m + (count - 1) * layer_m:.1f} m. '
            'MicroXRCEAgent and drone-gcs must already be running.')),
    ]

    for slot in range(count):
        # SITL instances are numbered from 1, not 0. Instance 0 would publish on
        # bare /fmu/... with no namespace, which is right for a single vehicle
        # and a special case for every other line in this file. Starting at 1
        # makes all `count` drones identical in shape.
        instance = slot + 1

        # The pad, in the shared swarm frame. These same two numbers go to
        # Gazebo as a spawn point and to the node as "where I am", and they must
        # agree.
        pad_north_m = 0.0
        pad_east_m = slot * pad_spacing_m

        # PX4_GZ_MODEL_POSE is "x,y,z,roll,pitch,yaw" in GAZEBO'S world frame,
        # which is ENU: x is EAST, y is NORTH, z is UP. PX4's own frame is NED.
        # So east and north come out swapped relative to everything else here.
        #
        # Verified in the simulator rather than assumed: spawning at "6,0" put
        # the vehicle 5.98 m east, and "0,6" put it 5.99 m north, measured from
        # the difference in the reference latitude and longitude the two
        # estimators reported. The conversion is in PX4 v1.16.1,
        # src/modules/simulation/gz_bridge/GZBridge.cpp, "position ENU -> NED".
        model_pose = f'{pad_east_m},{pad_north_m}'

        px4_env = {
            'PX4_SYS_AUTOSTART': PX4_AIRFRAME_X500,
            'PX4_SIM_MODEL': PX4_SIM_MODEL,
            'PX4_GZ_MODEL_POSE': model_pose,
            'PX4_GZ_WORLD': world,
        }
        if headless:
            # PX4 reads this as present-or-absent too. Only the first vehicle
            # ever starts the renderer, but setting it on all of them keeps the
            # vehicles interchangeable.
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

        # The first vehicle loads Gazebo and the world, which is slow. The rest
        # only have to spawn a model into a world that already exists. PX4's own
        # startup script waits up to 30 s for the world before giving up
        # (px4-rc.gzsim), so these delays have that much slack built in on top.
        start_at = 0.0 if slot == 0 else gazebo_boot_s + (slot - 1) * stagger_s
        actions.append(TimerAction(period=start_at, actions=[px4_process]))

        flight_node = Node(
            package='swarm_naive',
            executable=FLIGHT_NODE,
            name=f'drone_{slot}',
            output='screen',
            emulate_tty=True,            # keeps rclcpp's log colours
            parameters=[{
                # Which aircraft to talk to.
                'px4_ns': f'px4_{instance}',
                'system_id': instance + 1,
                # Which drone in the swarm this is.
                'slot': slot,
                'count': count,
                # Where its pad is, in the shared swarm frame. Same numbers as
                # PX4_GZ_MODEL_POSE above, in the sane order.
                'home_north_m': pad_north_m,
                'home_east_m': pad_east_m,
                # The shared flight plan. Identical for every drone; only
                # `slot` makes them behave differently.
                'centre_north_m': centre_north_m,
                'centre_east_m': centre_east_m,
                'radius_m': radius_m,
                'altitude_m': altitude_m,
                'layer_m': layer_m,
                'laps': laps,
            }],
        )

        # Start the nodes once every vehicle is up. They are patient either way
        # (a node with no telemetry says so and waits), so this delay is for
        # readable output rather than correctness.
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
