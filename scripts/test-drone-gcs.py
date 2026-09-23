#!/usr/bin/env python3
"""Checks the parts of drone-gcs that flying a drone cannot check.

    ./scripts/test-drone-gcs.py

Runs in under a second, needs no simulator, no PX4 and no network. Both smoke
tests run it before they start a vehicle.

WHY THIS FILE EXISTS
--------------------
The smoke tests prove drone-gcs does its one job: a vehicle arms, which it
cannot do unless a ground station is satisfying PX4's datalink-loss failsafe.
That is the integration test, and it is the one that matters most.

It cannot see any of the following, because all of it happens in bookkeeping
that a single happy vehicle never exercises:

  * whether a message pymavlink could not decode is counted as a vehicle
  * whether the vehicle count is right
  * whether a lost link is noticed, and a recovered one
  * how long to wait before declaring a swarm short

THE REGRESSION THIS PINS
------------------------
drone-gcs used to count every distinct source system it heard. pymavlink
reports srcSystem 0 for any message whose id is missing from its bundled
dialect, because MAVLink_unknown never populates the frame header. PX4 v1.16.1
streams several such messages, so they all collapsed into a phantom "system 0"
and a three-drone swarm reported four vehicles linked.

That mattered because the docs tell you to check that count against the number
of drones you launched, and it was always one too high. Verified at the time by
reading the frames off the socket without pymavlink's decoder: they carry the
vehicle's real system id. See the header of .devcontainer/scripts/drone-gcs.

`test_phantom_vehicle` below is that bug, turned into a test.
"""
import contextlib
import importlib.machinery
import importlib.util
import io
import os
import sys
import time

# Captured before anything else can redirect it. drone-gcs prints as it runs,
# and that output is swallowed below so a run of this file reads as a list of
# results rather than a transcript of a ground station. It is kept, not thrown
# away: a failing test prints what the station said, which is usually the whole
# explanation.
_REAL_STDOUT = sys.stdout

try:
    from pymavlink import mavutil
except ImportError:
    print("SKIP  pymavlink is not installed, so drone-gcs cannot run either")
    sys.exit(0)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRONE_GCS = os.path.join(REPO_ROOT, ".devcontainer", "scripts", "drone-gcs")

# Loaded by path because drone-gcs has no .py extension: it is a command, and
# both environments put it on PATH under that name.
_loader = importlib.machinery.SourceFileLoader("drone_gcs", DRONE_GCS)
_spec = importlib.util.spec_from_loader("drone_gcs", _loader)
gcs = importlib.util.module_from_spec(_spec)
_loader.exec_module(gcs)

FAILURES = []


def check(label, got, want):
    if got == want:
        print(f"  PASS  {label}", file=_REAL_STDOUT)
    else:
        print(f"  FAIL  {label}: got {got!r}, want {want!r}", file=_REAL_STDOUT)
        FAILURES.append(label)


def run(test):
    """Run one test with drone-gcs's own chatter captured."""
    print(test.__doc__.splitlines()[0], file=_REAL_STDOUT)
    before = len(FAILURES)
    captured = io.StringIO()
    with contextlib.redirect_stdout(captured), contextlib.redirect_stderr(captured):
        test()
    if len(FAILURES) > before:
        print("        what drone-gcs printed during that test:", file=_REAL_STDOUT)
        for line in captured.getvalue().splitlines():
            print(f"          {line}", file=_REAL_STDOUT)


def vehicle_message(system_id):
    """A decodable message with a populated header, as a real vehicle sends.

    MAVLink_header's positional order is (msgId, incompat_flags, compat_flags,
    mlen, seq, srcSystem, srcComponent). Passing the system id positionally sets
    seq instead and every assertion below quietly tests the wrong thing, so
    these are keyword arguments on purpose.
    """
    message = mavutil.mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_PX4,
        0, 0, mavutil.mavlink.MAV_STATE_STANDBY, 3)
    message._header = mavutil.mavlink.MAVLink_header(
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        srcSystem=system_id, srcComponent=1)
    return message


def test_undecodable_messages_are_not_vehicles():
    """A message pymavlink cannot read tells you nothing about who sent it."""
    # 410 is EVENT, which PX4 streams and pymavlink 2.4.49 does not define.
    unknown = mavutil.mavlink.MAVLink_unknown(410, b"\x01\x02")
    check("its srcSystem really is a lie", unknown.get_srcSystem(), 0)
    check("UNKNOWN_410 rejected", gcs.is_from_a_vehicle(unknown), False)
    check("BAD_DATA rejected",
          gcs.is_from_a_vehicle(mavutil.mavlink.MAVLink_bad_data(b"\x01", "x")), False)
    check("a real heartbeat is accepted", gcs.is_from_a_vehicle(vehicle_message(2)), True)
    # Belt and braces: MAVLink reserves 0 for broadcast, so it is never a vehicle
    # even on a message that decoded cleanly.
    check("system 0 rejected even when decodable",
          gcs.is_from_a_vehicle(vehicle_message(0)), False)


def test_phantom_vehicle():
    """The actual bug: undecodable traffic must not inflate the count."""
    station = gcs.Station(expected=3)
    for message_id in (8, 290, 291, 380, 410, 411, 436):
        message = mavutil.mavlink.MAVLink_unknown(message_id, b"\x01")
        if gcs.is_from_a_vehicle(message):
            station.on_message(message)
        else:
            station.note_skipped_message(message)
    check("seven undecodable ids, zero vehicles", station.linked_count(), 0)

    for system_id in (2, 3, 4):
        station.on_message(vehicle_message(system_id))
    check("three real drones counted as three", station.linked_count(), 3)
    station.on_message(vehicle_message(2))
    check("a repeat message does not double count", station.linked_count(), 3)


def test_link_loss_and_recovery():
    """A vehicle that goes quiet is noticed, and so is one that comes back."""
    original = gcs.LINK_LOSS_S
    gcs.LINK_LOSS_S = 0.3
    try:
        station = gcs.Station(expected=3)
        for system_id in (2, 3, 4):
            station.on_message(vehicle_message(system_id))
        time.sleep(gcs.LINK_LOSS_S + 0.2)
        # Keep two alive so only system 3 is overdue.
        station.on_message(vehicle_message(2))
        station.on_message(vehicle_message(4))
        station.check_for_lost_links()
        check("the quiet one is dropped from the count", station.linked_count(), 2)
        station.check_for_lost_links()
        check("and is not reported twice", station.linked_count(), 2)
        station.on_message(vehicle_message(3))
        check("it is counted again when it returns", station.linked_count(), 3)
    finally:
        gcs.LINK_LOSS_S = original


def test_shortfall_deadline_scales():
    """A big swarm is legitimately slower to assemble than a small one.

    The launch files stagger PX4 instances on purpose, because the first vehicle
    has to load Gazebo before any other can join its world. A flat deadline told
    a ten-drone swarm it was short while it was still doing the right thing.
    """
    check("no --expect", gcs.Station(None).deadline_s(), gcs.SILENCE_WARNING_S)
    check("--expect 1", gcs.Station(1).deadline_s(), gcs.SILENCE_WARNING_S)
    check("--expect 3", gcs.Station(3).deadline_s(),
          gcs.SILENCE_WARNING_S + 2 * gcs.EXTRA_WAIT_PER_VEHICLE_S)
    check("--expect 10", gcs.Station(10).deadline_s(),
          gcs.SILENCE_WARNING_S + 9 * gcs.EXTRA_WAIT_PER_VEHICLE_S)
    check("a bigger swarm waits longer",
          gcs.Station(10).deadline_s() > gcs.Station(3).deadline_s(), True)


def test_expect_argument():
    """--expect has to stay inside the range of instances we heartbeat."""
    check("no argument means no expectation", gcs.parse_args([]).expect, None)
    check("--expect 3 parses", gcs.parse_args(["--expect", "3"]).expect, 3)
    for bad in ("0", "-1", str(gcs.SITL_INSTANCE_LIMIT + 1)):
        try:
            gcs.parse_args(["--expect", bad])
            check(f"--expect {bad} rejected", "accepted", "rejected")
        except SystemExit:
            check(f"--expect {bad} rejected", "rejected", "rejected")


def ports_for(instance_base):
    return sorted(port for _host, port in gcs.sitl_addresses(instance_base))


def test_instance_block_covers_every_vehicle_it_can_launch():
    """The block has to reach the tenth drone of a ten-drone swarm.

    The launch files number vehicles `instance_base + slot + 1` for slot
    0..count-1, so count:=10 produces instance base+10. A station covering ten
    ports starting at base+0 stopped one short: the last drone of a full swarm
    never got a heartbeat, so its datalink failsafe was never satisfied and it
    sat on the pad reporting "Arming denied" while the other nine flew.
    """
    ports = ports_for(0)
    check("slot 0 reaches instance 10", ports[-1], gcs.SITL_GCS_PORT_BASE + 10)
    check("ten swarm instances covered, plus instance 0", len(ports), 11)

    # Instance 0 is what a bare `drone-sim` runs, and it has to be heartbeated
    # like any other. PX4 will not send to a UDP peer it has not heard from
    # (_src_addr_initialized, mavlink_main.cpp:782), so a station that skips
    # 18570 leaves the single-vehicle case depending on MAV_BROADCAST, which is
    # a parameter rather than a promise. Dropping it once was a real
    # regression; this is that regression turned into a test.
    check("instance 0 is in slot 0's block",
          gcs.SITL_GCS_PORT_BASE in ports, True)
    check("instance 0 belongs to nobody else",
          gcs.SITL_GCS_PORT_BASE in set(ports_for(gcs.SITL_INSTANCE_BLOCK)), False)


def test_blocks_do_not_overlap():
    """Two people on one machine must never heartbeat the same vehicle."""
    slot0 = set(ports_for(0))
    slot1 = set(ports_for(gcs.SITL_INSTANCE_BLOCK))
    slot2 = set(ports_for(gcs.SITL_INSTANCE_BLOCK * 2))
    check("slot 0 and slot 1 share nothing", slot0 & slot1, set())
    check("slot 1 and slot 2 share nothing", slot1 & slot2, set())
    check("slot 1 starts where slot 0 ended",
          min(slot1) - max(slot0), 1)
    check("slot 1 is instances 11-20",
          (min(slot1), max(slot1)),
          (gcs.SITL_GCS_PORT_BASE + 11, gcs.SITL_GCS_PORT_BASE + 20))


def test_port_arguments():
    """--listen-port and --instance-base, and the environment behind them."""
    check("default listen port", gcs.parse_args([]).listen_port,
          gcs.GCS_LISTEN_PORT)
    check("default instance base", gcs.parse_args([]).instance_base, 0)
    check("--listen-port parses",
          gcs.parse_args(["--listen-port", "14551"]).listen_port, 14551)
    check("--instance-base parses",
          gcs.parse_args(["--instance-base", "10"]).instance_base, 10)

    # A base that is not a multiple of the block width produces overlapping
    # blocks, which is the exact bug the whole scheme exists to prevent.
    for bad in ("-10", "5", "95", "100"):
        try:
            gcs.parse_args(["--instance-base", bad])
            check(f"--instance-base {bad} rejected", "accepted", "rejected")
        except SystemExit:
            check(f"--instance-base {bad} rejected", "rejected", "rejected")


def test_environment_defaults():
    """env.sh sets these, so reading them has to work and has to be safe."""
    saved = {name: os.environ.get(name)
             for name in ("DRONE_GCS_PORT", "PX4_INSTANCE_BASE")}
    try:
        os.environ["DRONE_GCS_PORT"] = "14553"
        os.environ["PX4_INSTANCE_BASE"] = "30"
        args = gcs.parse_args([])
        check("listen port comes from DRONE_GCS_PORT", args.listen_port, 14553)
        check("instance base comes from PX4_INSTANCE_BASE",
              args.instance_base, 30)
        check("and the block follows it",
              ports_for(args.instance_base)[0], gcs.SITL_GCS_PORT_BASE + 31)

        # An explicit flag still wins, which is what makes the station
        # debuggable from a shell that has the wrong slot exported.
        check("a flag overrides the environment",
              gcs.parse_args(["--listen-port", "14550"]).listen_port, 14550)

        # A typo in ~/.bashrc must not stop the station starting. Falling back
        # to the single-user default is visible, because the startup banner
        # prints the ports it actually chose.
        os.environ["PX4_INSTANCE_BASE"] = "not-a-number"
        check("garbage in the environment falls back",
              gcs.parse_args([]).instance_base, 0)
        del os.environ["DRONE_GCS_PORT"]
        check("unset falls back", gcs.parse_args([]).listen_port,
              gcs.GCS_LISTEN_PORT)
    finally:
        for name, was in saved.items():
            if was is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = was


def main():
    print(f"Testing {os.path.relpath(DRONE_GCS, REPO_ROOT)}\n", file=_REAL_STDOUT)
    for test in (test_undecodable_messages_are_not_vehicles,
                 test_phantom_vehicle,
                 test_link_loss_and_recovery,
                 test_shortfall_deadline_scales,
                 test_expect_argument,
                 test_instance_block_covers_every_vehicle_it_can_launch,
                 test_blocks_do_not_overlap,
                 test_port_arguments,
                 test_environment_defaults):
        run(test)

    # Not an assertion. Which ids pymavlink lacks changes when pymavlink is
    # upgraded, and that is fine: the behaviour above is what must hold. This
    # only says how much undecodable traffic PX4 is currently producing.
    missing = [i for i in (8, 290, 291, 380, 410, 411, 436)
               if i not in mavutil.mavlink.mavlink_map]
    print(f"\n  note  this pymavlink cannot decode PX4 message ids "
          f"{missing or 'none'}", file=_REAL_STDOUT)

    if FAILURES:
        print(f"\nFAILED: {', '.join(FAILURES)}", file=_REAL_STDOUT)
        return 1
    print("\nAll drone-gcs checks passed.", file=_REAL_STDOUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
