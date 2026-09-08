"""Coordinate-frame conversions between ROS 2 (ENU/FLU) and PX4 (NED/FRD).

Frame definitions used throughout this package
----------------------------------------------
  World ENU  : +X East,     +Y North, +Z Up      (ROS 2 REP-103 world)
  World NED  : +X North,    +Y East,  +Z Down    (PX4 world)
  Body  FLU  : +X Forward,  +Y Left,  +Z Up      (ROS 2 REP-103 body)
  Body  FRD  : +X Forward,  +Y Right, +Z Down    (PX4 body)
  Camera RDF : +X right-in-image, +Y down-in-image, +Z along the optical axis

ENU<->NED and FLU<->FRD are both the same involutive operation: swap the first two
components and negate the third. Applying it twice is the identity, which is why one
helper covers both directions.
"""
import math
import numpy as np

# WORLD basis change ENU <-> NED: (x, y, z) -> (y, x, -z).  East<->North swap, Up->Down.
# This is a 180 deg rotation about (1,1,0)/sqrt(2); det = +1; it is its own inverse.
_SWAP_WORLD = np.array([[0.0, 1.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0]])

# BODY basis change FLU <-> FRD: (x, y, z) -> (x, -y, -z).  Forward kept, Left->Right,
# Up->Down. This is a 180 deg rotation about X; det = +1; also its own inverse.
# NOTE: this is NOT the same matrix as the world swap. Using one for the other is a
# classic and very hard-to-see bug -- both are involutive and both have det +1, so a
# round-trip test passes while real attitudes come out wrong.
_FLIP_BODY = np.array([[1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0],
                       [0.0, 0.0, -1.0]])

# Optical(RDF) axes expressed in the Gazebo camera LINK frame (X along the optical
# axis, Y left, Z up):  X_opt = -Y_link, Y_opt = -Z_link, Z_opt = +X_link.
R_OPTICAL_IN_LINK = np.array([[0.0, 0.0, 1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0]])


def camera_rotation_in_body(link_rpy=(0.0, math.pi / 2.0, 0.0)):
    """Rotation taking camera-optical(RDF) vectors into body(FLU) vectors.

    ``link_rpy`` is the roll/pitch/yaw of the camera LINK taken straight from the
    ``<pose>`` of the sensor in the vehicle SDF. The default is the nadir mount
    (pitch +90 deg) used by the x500_depth_down airframe.

    Do NOT hard-code the result. For the nadir mount this evaluates to

        [[ 0, -1,  0],
         [-1,  0,  0],
         [ 0,  0, -1]]

    i.e. image-right is body-RIGHT and image-down is body-BACKWARD -- a 90 deg roll
    about the optical axis relative to the "image-right = body-forward" arrangement
    one might naively assume. Getting this wrong rotates the whole velocity estimate
    by 90 deg, which reads as a plausible-looking but completely wrong odometry.
    """
    return euler_to_R_enu_flu(*link_rpy) @ R_OPTICAL_IN_LINK


def enu_to_ned(v):
    """ENU world vector -> NED world vector (involutive: also does NED -> ENU)."""
    return _SWAP_WORLD @ np.asarray(v, dtype=float)


def flu_to_frd(v):
    """FLU body vector -> FRD body vector (involutive: also does FRD -> FLU)."""
    return _FLIP_BODY @ np.asarray(v, dtype=float)


ned_to_enu = enu_to_ned          # same operation, named for readability at call sites
frd_to_flu = flu_to_frd


def normalize_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_enu_to_ned(yaw_enu):
    """ENU yaw (CCW from East) -> NED yaw (CW from North). Involutive."""
    return normalize_angle(math.pi / 2.0 - yaw_enu)


yaw_ned_to_enu = yaw_enu_to_ned


def quat_to_R(q):
    """PX4/Hamilton quaternion [w, x, y, z] -> 3x3 rotation matrix."""
    w, x, y, z = (float(c) for c in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    """3x3 rotation matrix -> Hamilton quaternion [w, x, y, z] (PX4 order)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def R_ned_frd_to_enu_flu(R_ned_frd):
    """Convert a NED<-FRD rotation into the equivalent ENU<-FLU rotation.

    v_ned = R @ v_frd, v_frd = D @ v_flu, v_enu = S @ v_ned  =>  R_enu_flu = S @ R @ D
    with S the world swap and D the body flip. The two matrices DIFFER; because both
    are involutive the same expression also converts back, so one function serves both
    directions.
    """
    return _SWAP_WORLD @ np.asarray(R_ned_frd, dtype=float) @ _FLIP_BODY


R_enu_flu_to_ned_frd = R_ned_frd_to_enu_flu


def euler_from_R_enu_flu(R):
    """Extract (roll, pitch, yaw) in the ENU/FLU convention from a rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.9999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                        # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def euler_to_R_enu_flu(roll, pitch, yaw):
    """(roll, pitch, yaw) -> ENU<-FLU rotation matrix, Z-Y-X intrinsic."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]) @
            np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]) @
            np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]))
