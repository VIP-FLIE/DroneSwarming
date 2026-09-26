#!/usr/bin/env python3
"""Unit checks for the drone-gcs bookkeeping.

    ./scripts/test-drone-gcs.py

No simulator, no PX4, no network needed. Runs in under a second. Both smoke
tests run it before starting a vehicle.

Flying a vehicle can only prove that SOME ground station link works. This file
checks the parts one happy vehicle never exercises: undecodable MAVLink
traffic, vehicle counting, lost and recovered links, shortfall deadlines, and
the per-slot instance blocks.
"""
import contextlib
import importlib.machinery
import importlib.util
import io
import os
import sys
import time

# Captured before anything else can redirect stdout, so drone-gcs's own
# output can be swallowed below while test results still print.
_REAL_STDOUT = sys.stdout

try:
    from pymavlink import mavutil
except ImportError:
    print("SKIP  pymavlink is not installed, so drone-gcs cannot run either")
    sys.exit(0)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRONE_GCS = os.path.join(REPO_ROOT, ".devcontainer", "scripts", "drone-gcs")

# Loaded by path; drone-gcs has no .py extension because it is a command on
# PATH in both environments.
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
    """A decodable heartbeat with a populated header, as a real vehicle sends.

    MAVLink_header's positional order is (msgId, incompat_flags, compat_flags,
    mlen, seq, srcSystem, srcComponent), so the system id is a keyword here.
    """
    message = mavutil.mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_PX4,
        0, 0, mavutil.mavlink.MAV_STATE_STANDBY, 3)
    message._header = mavutil.mavlink.MAVLink_header(
        mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
        srcSystem=system_id, srcComponent=1)
    return message


def test_undecodable_traffic_is_not_a_vehicle():
    """Undecodable MAVLink traffic must not be counted as a vehicle."""
    # 410 is EVENT, which PX4 streams and pymavlink 2.4.49 does not define.
    unknown = mavutil.mavlink.MAVLink_unknown(410, b"\x01\x02")
    check("its srcSystem really is a lie", unknown.get_srcSystem(), 0)
    check("UNKNOWN_410 rejected", gcs.is_from_a_vehicle(unknown), False)
    check("BAD_DATA rejected",
          gcs.is_from_a_vehicle(mavutil.mavlink.MAVLink_bad_data(b"\x01", "x")), False)
    check("a real heartbeat is accepted", gcs.is_from_a_vehicle(vehicle_message(2)), True)
    # MAVLink reserves system 0 for broadcast, so it is never a vehicle even on
    # a message that decoded.
    check("system 0 rejected even when decodable",
          gcs.is_from_a_vehicle(vehicle_message(0)), False)

    # The regression: counting each distinct srcSystem let PX4's undecodable
    # messages collapse into a phantom "system 0", so three drones reported
    # four vehicles linked.
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
    """A vehicle that goes quiet is dropped from the count, and relinks."""
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
    """The shortfall deadline grows with the --expect count.

    The launch files stagger PX4 instances because the first vehicle must load
    Gazebo before the others can join its world. A flat deadline reported a
    ten-drone swarm as short while it was still assembling.
    """
    check("no --expect", gcs.Station(None).deadline_s(), gcs.SILENCE_WARNING_S)
    check("--expect 1", gcs.Station(1).deadline_s(), gcs.SILENCE_WARNING_S)
    check("--expect 3", gcs.Station(3).deadline_s(),
          gcs.SILENCE_WARNING_S + 2 * gcs.EXTRA_WAIT_PER_VEHICLE_S)
    check("--expect 10", gcs.Station(10).deadline_s(),
          gcs.SILENCE_WARNING_S + 9 * gcs.EXTRA_WAIT_PER_VEHICLE_S)
    check("a bigger swarm waits longer",
          gcs.Station(10).deadline_s() > gcs.Station(3).deadline_s(), True)


def test_argument_parsing():
    """The CLI arguments parse and reject out-of-range values."""
    check("no argument means no expectation", gcs.parse_args([]).expect, None)
    check("--expect 3 parses", gcs.parse_args(["--expect", "3"]).expect, 3)
    for bad in ("0", "-1", str(gcs.SITL_INSTANCE_LIMIT + 1)):
        try:
            gcs.parse_args(["--expect", bad])
            check(f"--expect {bad} rejected", "accepted", "rejected")
        except SystemExit:
            check(f"--expect {bad} rejected", "rejected", "rejected")

    check("default listen port", gcs.parse_args([]).listen_port,
          gcs.GCS_LISTEN_PORT)
    check("default instance base", gcs.parse_args([]).instance_base, 0)
    check("--listen-port parses",
          gcs.parse_args(["--listen-port", "14551"]).listen_port, 14551)
    check("--instance-base parses",
          gcs.parse_args(["--instance-base", "10"]).instance_base, 10)

    # A base that is not a multiple of the block width produces overlapping
    # blocks, which is the bug the whole block scheme exists to prevent.
    for bad in ("-10", "5", "95", "100"):
        try:
            gcs.parse_args(["--instance-base", bad])
            check(f"--instance-base {bad} rejected", "accepted", "rejected")
        except SystemExit:
            check(f"--instance-base {bad} rejected", "rejected", "rejected")


def ports_for(instance_base):
    return sorted(port for _host, port in gcs.sitl_addresses(instance_base))


def test_instance_blocks():
    """One station's instance block must cover its swarm and overlap no other."""
    ports = ports_for(0)
    check("slot 0 reaches instance 10", ports[-1], gcs.SITL_GCS_PORT_BASE + 10)
    check("ten swarm instances covered, plus instance 0", len(ports), 11)

    # Instance 0 is what a single-vehicle PX4 run without -i uses, and it has
    # to be heartbeated like any other. PX4 will not send to a UDP peer it has
    # not heard from (_src_addr_initialized, mavlink_main.cpp:782), so dropping
    # 18570 makes the single-vehicle case depend on MAV_BROADCAST, which is a
    # parameter rather than a promise. That was a real regression.
    check("instance 0 is in slot 0's block",
          gcs.SITL_GCS_PORT_BASE in ports, True)
    check("instance 0 belongs to nobody else",
          gcs.SITL_GCS_PORT_BASE in set(ports_for(gcs.SITL_INSTANCE_BLOCK)), False)

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


def test_environment_defaults():
    """env.sh variables feed the defaults; a flag still wins; garbage falls back."""
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

        check("a flag overrides the environment",
              gcs.parse_args(["--listen-port", "14550"]).listen_port, 14550)

        # A typo in ~/.bashrc must not stop the station starting.
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
    for test in (test_undecodable_traffic_is_not_a_vehicle,
                 test_link_loss_and_recovery,
                 test_shortfall_deadline_scales,
                 test_argument_parsing,
                 test_instance_blocks,
                 test_environment_defaults):
        run(test)

    # Not an assertion. Which ids pymavlink lacks changes when pymavlink is
    # upgraded; the behaviour above is what must hold.
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