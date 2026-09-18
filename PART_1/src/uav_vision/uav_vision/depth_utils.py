"""Depth-image handling for the Gazebo/gz depth camera.

Two gz-specific details matter here:

1. Invalid pixels are +Inf (beyond the far clip, or nothing hit -- e.g. sky) and -Inf
   (nearer than the near clip). gz does not emit NaN. np.nanmedian does not ignore Inf,
   so a single sky pixel poisons a median unless Inf is converted to NaN first. Always
   call ``sanitize`` before any statistic.

2. The published value is Z-depth along the optical axis (``point.x`` = ``-viewSpacePos.z``
   in the gz Ogre2 depth shader), not Euclidean range. For a level nadir camera over flat
   ground that makes depth constant across the image and exactly equal to the AGL height,
   which is why height ends up being an absolute, drift-free measurement here.
"""
import numpy as np


def sanitize(depth, min_valid=0.25, max_valid=19.0):
    """Convert gz's +/-Inf and out-of-range values to NaN so nan-aware stats work."""
    d = np.asarray(depth, dtype=np.float32).copy()
    d[~np.isfinite(d)] = np.nan
    d[(d < min_valid) | (d > max_valid)] = np.nan
    return d


def agl_from_depth(depth, roll, pitch, centre_frac=0.5, min_valid_frac=0.15):
    """Height above ground from a nadir depth image.

    Uses a central window (edges are the first to leave the ground plane when the
    vehicle banks) and corrects the optical-axis depth for tilt:

        AGL = Z_axis * cos(tilt),   tilt = angle between the optical axis and vertical

    Returns NaN when too few pixels are valid, so callers can degrade rather than fly on
    a fabricated altitude.
    """
    d = sanitize(depth)
    h, w = d.shape
    ch, cw = int(h * centre_frac), int(w * centre_frac)
    r0, c0 = (h - ch) // 2, (w - cw) // 2
    patch = d[r0:r0 + ch, c0:c0 + cw]
    valid = np.isfinite(patch)
    if valid.mean() < min_valid_frac:
        return float('nan'), float(valid.mean())
    tilt = np.hypot(float(roll), float(pitch))
    return float(np.nanmedian(patch)) * float(np.cos(tilt)), float(valid.mean())


def depth_at_rgb_pixels(depth, pts_rgb, K_rgb, K_depth, min_valid=0.25, max_valid=19.0):
    """Look up depth for feature pixels detected in the RGB image.

    The OakD-Lite RGB and depth sensors have different resolutions and fields
    of view (1920x1080 @ ~1.204 rad vs 640x480 @ 1.274 rad), so RGB pixel (u,v) is not
    depth pixel (u,v). Because both sensors sit at the same pose in the model, the
    extrinsic between them is identity and the mapping is a pure intrinsic re-projection
    through normalised image coordinates:

        x = (u_rgb - cx_rgb)/fx_rgb            -> u_depth = x*fx_d + cx_d

    If a future model gives the two sensors different poses this must become a full
    reprojection using the depth value; assert the poses are equal in the SDF first.
    """
    d = np.asarray(depth, dtype=np.float32)
    h, w = d.shape
    x = (pts_rgb[:, 0] - K_rgb[0, 2]) / K_rgb[0, 0]
    y = (pts_rgb[:, 1] - K_rgb[1, 2]) / K_rgb[1, 1]
    u = np.rint(x * K_depth[0, 0] + K_depth[0, 2]).astype(int)
    v = np.rint(y * K_depth[1, 1] + K_depth[1, 2]).astype(int)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    z = np.full(len(pts_rgb), np.nan, dtype=float)
    zz = d[np.clip(v, 0, h - 1), np.clip(u, 0, w - 1)].astype(float)
    zz[~np.isfinite(zz)] = np.nan
    zz[(zz < min_valid) | (zz > max_valid)] = np.nan
    z[inside] = zz[inside]
    return z


def K_from_camera_info(msg):
    """3x3 intrinsic matrix from a sensor_msgs/CameraInfo."""
    k = np.asarray(msg.k, dtype=float).reshape(3, 3)
    if k[0, 0] <= 0.0:                      # some gz versions leave K empty; use P
        k = np.asarray(msg.p, dtype=float).reshape(3, 4)[:, :3]
    return k


def K_from_fov(width, height, hfov):
    """Fallback intrinsics from SDF geometry: fx = (width/2)/tan(hfov/2), square pixels."""
    fx = (width / 2.0) / np.tan(hfov / 2.0)
    return np.array([[fx, 0.0, width / 2.0 - 0.5],
                     [0.0, fx, height / 2.0 - 0.5],
                     [0.0, 0.0, 1.0]])
