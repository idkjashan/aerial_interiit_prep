"""Regression tests for the VO core. Run with: pytest -q src/uav_vision/test

These are the tests that caught the two real bugs during development:
  * the homography decomposition halving the translation, and
  * the FLU<->FRD body flip being confused with the ENU<->NED world swap.
Both pass a naive round-trip check, so keep the numeric assertions below.
"""
import numpy as np
import pytest

from uav_vision.frames import (enu_to_ned, flu_to_frd, quat_to_R, R_to_quat,
                               euler_to_R_enu_flu, euler_from_R_enu_flu,
                               R_ned_frd_to_enu_flu, R_enu_flu_to_ned_frd,
                               yaw_enu_to_ned, normalize_angle,
                               camera_rotation_in_body)
from uav_vision.vo_core import DownwardVO

K = np.array([[432.5, 0.0, 319.5], [0.0, 432.5, 239.5], [0.0, 0.0, 1.0]])


def test_world_and_body_swaps_are_different():
    """ENU<->NED swaps X/Y; FLU<->FRD does not. Conflating them is a silent bug."""
    assert np.allclose(enu_to_ned([0, 1, 0]), [1, 0, 0])     # North -> +X
    assert np.allclose(flu_to_frd([0, 1, 0]), [0, -1, 0])    # Left  -> -Y
    assert not np.allclose(enu_to_ned([0, 1, 0]), flu_to_frd([0, 1, 0]))


@pytest.mark.parametrize('seed', range(20))
def test_frame_change_roundtrip_and_yaw_consistency(seed):
    rng = np.random.default_rng(seed)
    r, p, y = rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(-np.pi, np.pi)
    R_enu = euler_to_R_enu_flu(r, p, y)
    R_ned = R_enu_flu_to_ned_frd(R_enu)
    assert np.allclose(R_ned_frd_to_enu_flu(R_ned), R_enu, atol=1e-12)
    assert abs(np.linalg.det(R_ned) - 1.0) < 1e-12
    yaw_ned = np.arctan2(R_ned[1, 0], R_ned[0, 0])
    assert abs(normalize_angle(yaw_ned - yaw_enu_to_ned(y))) < 1e-9


@pytest.mark.parametrize('seed', range(20))
def test_quat_and_euler_roundtrip(seed):
    rng = np.random.default_rng(seed)
    e = rng.uniform(-1.2, 1.2, 3)
    R = euler_to_R_enu_flu(*e)
    assert np.allclose(quat_to_R(R_to_quat(R)), R, atol=1e-9)
    assert np.allclose(euler_from_R_enu_flu(R), e, atol=1e-9)


def test_camera_rotation_matches_sdf_pose():
    """Nadir mount: image-right is body RIGHT and image-down is body BACKWARD.

    A 90 deg error here is invisible in a round-trip test but rotates the whole velocity
    estimate, so pin the actual axes.
    """
    R = camera_rotation_in_body((0.0, np.pi / 2.0, 0.0))
    assert abs(np.linalg.det(R) - 1.0) < 1e-12
    assert np.allclose(R @ [0, 0, 1], [0, 0, -1], atol=1e-9)    # optical axis -> down
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0], atol=1e-9)    # image right -> body right
    assert np.allclose(R @ [0, 1, 0], [-1, 0, 0], atol=1e-9)    # image down  -> body back


def test_homography_translation_is_exact_and_scale_invariant():
    """The bug this pins: projecting M onto SO(3) to recover R halves the translation.

    Supplying the known IMU rotation instead makes the translation exact, and invariant
    to the arbitrary scale AND sign that findHomography returns.
    """
    vo = DownwardVO(K)
    R21 = euler_to_R_enu_flu(np.deg2rad(1.0), np.deg2rad(-2.0), np.deg2rad(3.0))
    t_true = np.array([0.30, -0.45, 0.08])
    n, d = np.array([0.0, 0.0, 1.0]), 10.0
    H = K @ (R21 + np.outer(t_true, n) / d) @ np.linalg.inv(K)
    for scale in (1.0, 3.7, -2.2):
        t = vo._translation_from_homography(H * scale, R21, n, d)
        assert t is not None and np.allclose(t, t_true, atol=1e-9), f'scale={scale}'


def test_attitude_error_maps_to_position_error_as_d_tan_theta():
    """Sanity-check the dominant error term: ~d*tan(dtheta), i.e. 1 deg at 10 m ~ 17-22 cm."""
    vo = DownwardVO(K)
    R21 = np.eye(3)
    t_true = np.array([0.2, -0.1, 0.0])
    n, d = np.array([0.0, 0.0, 1.0]), 10.0
    H = K @ (R21 + np.outer(t_true, n) / d) @ np.linalg.inv(K)
    err = np.linalg.norm(
        vo._translation_from_homography(
            H, euler_to_R_enu_flu(np.deg2rad(1.0), 0, 0) @ R21, n, d) - t_true)
    assert 0.10 < err < 0.35


def test_depth_sanitize_handles_gz_infinities():
    """gz emits +Inf beyond the far clip and -Inf inside the near clip -- never NaN."""
    from uav_vision.depth_utils import sanitize, agl_from_depth
    d = np.full((60, 80), 10.0, np.float32)
    d[0, 0] = np.inf
    d[0, 1] = -np.inf
    d[0, 2] = 500.0
    s = sanitize(d)
    assert np.isnan(s[0, 0]) and np.isnan(s[0, 1]) and np.isnan(s[0, 2])
    assert np.isfinite(np.nanmedian(s)) and abs(np.nanmedian(s) - 10.0) < 1e-6
    agl, frac = agl_from_depth(d, 0.0, 0.0)
    assert abs(agl - 10.0) < 1e-3 and frac > 0.9
