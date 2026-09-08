"""Shared QoS profile for talking to PX4 over the uXRCE-DDS bridge.

This is the single most common reason a node "works" but receives nothing.
PX4 publishes its /fmu/out/ topics as BEST_EFFORT with TRANSIENT_LOCAL
durability. ROS 2's default subscriber profile is RELIABLE with VOLATILE
durability. Those are incompatible, so the subscription is created without
error, never matches the publisher, and the callback simply never fires.

`ros2 topic echo <topic>` has the same problem and will also sit silent
unless you pass --qos-reliability best_effort.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# vehicle_command is handled as a DDS replier (service) by PX4's uXRCE-DDS
# client, not as a regular subscriber. The replier uses RELIABLE + PERSISTENT
# QoS (see srv_base.cpp). BEST_EFFORT publishers are incompatible and the
# command is silently dropped.
PX4_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def topic_prefix(namespace: str) -> str:
    """Return the topic prefix for a PX4 instance.

    Instance 0 publishes on bare /fmu/... to match how a real vehicle behaves.
    Instance N above 0 is namespaced as /px4_N/fmu/... by the uXRCE-DDS client.
    """
    namespace = (namespace or '').strip('/')
    return f'/{namespace}' if namespace else ''
