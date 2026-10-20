"""QoS profiles and small ROS helpers shared by the M5 nodes.

Profile names follow assignment 4.1/5: R1 = reliable keep-last 1,
R10 = reliable depth 10, R100 = reliable depth 100, a trailing T means
transient-local (late joiners get the last sample).
"""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time


def _qos(depth, transient=False):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient
                    else DurabilityPolicy.VOLATILE))


R1, R10, R100, R1T = _qos(1), _qos(10), _qos(100), _qos(1, True)


def stamp_age(node, stamp):
    """Age in seconds of a header stamp on the node's clock.

    Returns None for an unset (zero) stamp: the caller must treat that as
    'no source time', never as fresh data from 1970.
    """
    if stamp.sec == 0 and stamp.nanosec == 0:
        return None
    return (node.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9


def declare(node, defaults):
    """Declare parameters from {name: default}; return {name: value}."""
    return {name: node.declare_parameter(name, default).value
            for name, default in defaults.items()}
