"""QoS profiles for PX4 uXRCE-DDS topics.

PX4's uXRCE-DDS client publishes /fmu/out/* with BEST_EFFORT reliability. A RELIABLE
subscriber will NOT match a BEST_EFFORT publisher, so the subscription silently never
fires -- the single most common "my ROS 2 node sees no PX4 data" cause.

Durability is the second trap and it varies between PX4 releases. Rather than guessing,
verify against the running stack once:

    ros2 topic info /fmu/out/vehicle_attitude --verbose

and set the ``px4_durability`` parameter to match ("volatile" or "transient_local").
A TRANSIENT_LOCAL subscriber will not match a VOLATILE publisher either.
"""
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)


def px4_sub_qos(durability='volatile', depth=5):
    """QoS for subscribing to /fmu/out/* topics."""
    d = (DurabilityPolicy.TRANSIENT_LOCAL if str(durability).lower() == 'transient_local'
         else DurabilityPolicy.VOLATILE)
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=d,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def px4_pub_qos(depth=10):
    """QoS for publishing to /fmu/in/* topics."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def sensor_qos(depth=5):
    """QoS for camera streams coming over ros_gz_bridge."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)
