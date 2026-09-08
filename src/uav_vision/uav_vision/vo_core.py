"""Downward-looking RGB-D visual odometry core -- pure NumPy/OpenCV, no ROS.

Kept ROS-free on purpose so it can be unit-tested and regression-benchmarked against a
synthetic ground-truth simulator (``tools/simworld.py``) without starting Gazebo, PX4 or
a ROS graph. ``vo_node.py`` is a thin ROS 2 wrapper around this class.

Why this formulation
--------------------
The camera stares straight down at flat ground, so the scene is a PLANE. That rules some
standard machinery out and makes other parts unusually easy:

* 5-point / essential-matrix VO is DEGENERATE on a planar scene -- do not use it.
* A plane induces an exact homography between any two views, so ``findHomography`` +
  RANSAC is both well-posed and very robust here.
* Depth gives metric scale directly, so there is no scale ambiguity to resolve.
* Z-depth from a level nadir camera IS the AGL altitude, which makes height an ABSOLUTE,
  drift-free measurement rather than an integrated one.

Two estimates are produced each frame:

  velocity  frame-to-frame sparse LK flow, gyro-de-rotated, scaled by depth.
  position  registration against a KEYFRAME rather than frame-to-frame integration.
            Dead-reckoned velocity drifts without bound; anchoring to a keyframe holds
            absolute position for as long as the same ground patch stays in view, which
            is exactly the 90 s station-keeping case. It also recovers position exactly
            after a vision dropout, because the pre-dropout keyframe is still valid.

Rotation is taken from the flight controller's attitude rather than recovered from the
homography. Projecting the homography onto SO(3) to extract R absorbs part of the
translation term and HALVES the recovered translation (see tests/test_vo_core.py).
"""
import numpy as np
import cv2

from .frames import camera_rotation_in_body, euler_to_R_enu_flu

# --- health states, published as the vision confidence signal -----------------
HEALTH_OK = 0
HEALTH_DEGRADED = 1
HEALTH_LOST = 2
_HEALTH_NAMES = {HEALTH_OK: 'OK', HEALTH_DEGRADED: 'DEGRADED', HEALTH_LOST: 'LOST'}


def health_name(h):
    return _HEALTH_NAMES.get(h, 'UNKNOWN')


class VOParams:
    """Tuning knobs. Mirrored 1:1 by config/vo_params.yaml."""

    max_corners = 500
    quality_level = 0.01
    min_distance = 12
    block_size = 7
    lk_win = 21
    lk_levels = 3
    fb_threshold = 1.0            # forward-backward consistency, pixels
    ransac_thresh = 2.0           # homography RANSAC reprojection threshold, pixels
    min_features = 60             # below this the frame is not trusted
    degraded_features = 120       # below this we report DEGRADED
    rekey_shift = 0.35            # re-key once the view has shifted 35% of frame width
    clahe_clip = 2.0              # photometric normalisation (lighting invariance)
    clahe_grid = 8
    max_depth = 19.0              # sensor far clip, metres
    min_depth = 0.25
    vel_lpf_alpha = 0.6           # velocity low-pass, 0..1 (1 = no filtering)


class DownwardVO:
    """Visual odometry for a nadir RGB-D camera over locally-flat ground."""

    def __init__(self, K, params=None, cam_link_rpy=(0.0, np.pi / 2.0, 0.0)):
        self.K = np.asarray(K, dtype=float)
        self.Kinv = np.linalg.inv(self.K)
        self.p = params or VOParams()
        # Camera-optical -> body(FLU). Derived from the SDF pose, never assumed.
        self.R_cam_in_body = camera_rotation_in_body(cam_link_rpy)
        self._clahe = cv2.createCLAHE(clipLimit=self.p.clahe_clip,
                                      tileGridSize=(self.p.clahe_grid, self.p.clahe_grid))
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.prev_img = None
        self.prev_pts = None
        self.kf_img = None
        self.kf_pts = None
        self.kf_pos = np.zeros(3)
        self.kf_rpy = (0.0, 0.0, 0.0)
        self.kf_depth = None
        self.pos = np.zeros(3)          # ENU, relative to the first frame
        self.vel = np.zeros(3)          # ENU
        self.health = HEALTH_LOST
        self.n_tracked = 0
        self.n_kf_inliers = 0
        self.kf_resets = 0
        self.consecutive_bad = 0

    # ---------------------------------------------------------------- helpers
    def _prep(self, img):
        """Grayscale + CLAHE. CLAHE is what makes tracking survive exposure changes."""
        if img.ndim == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img.dtype != np.uint8:
            img = cv2.convertScaleAbs(img)
        return self._clahe.apply(img)

    def _detect(self, img):
        p = cv2.goodFeaturesToTrack(
            img, maxCorners=self.p.max_corners, qualityLevel=self.p.quality_level,
            minDistance=self.p.min_distance, blockSize=self.p.block_size)
        return None if p is None else p.reshape(-1, 1, 2).astype(np.float32)

    def _track(self, img_a, img_b, pts_a):
        """Pyramidal LK with a forward-backward consistency gate."""
        if pts_a is None or len(pts_a) < 8:
            return None, None
        lk = dict(winSize=(self.p.lk_win, self.p.lk_win), maxLevel=self.p.lk_levels,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        pb, st_f, _ = cv2.calcOpticalFlowPyrLK(img_a, img_b, pts_a, None, **lk)
        if pb is None:
            return None, None
        pa_back, st_b, _ = cv2.calcOpticalFlowPyrLK(img_b, img_a, pb, None, **lk)
        if pa_back is None:
            return None, None
        a = pts_a.reshape(-1, 2)
        fb_err = np.linalg.norm(a - pa_back.reshape(-1, 2), axis=1)
        ok = (st_f.ravel() == 1) & (st_b.ravel() == 1) & (fb_err < self.p.fb_threshold)
        if ok.sum() < 8:
            return None, None
        return a[ok], pb.reshape(-1, 2)[ok]

    def _depth_at(self, depth, pts):
        h, w = depth.shape
        u = np.clip(np.rint(pts[:, 0]).astype(int), 0, w - 1)
        v = np.clip(np.rint(pts[:, 1]).astype(int), 0, h - 1)
        z = depth[v, u].astype(float)
        z[(z < self.p.min_depth) | (z > self.p.max_depth)] = np.nan
        return z

    # ------------------------------------------------------- velocity from flow
    def _velocity_from_flow(self, p0, p1, Z, omega_cam, dt):
        """Least-squares camera-frame ego-velocity from gyro-de-rotated sparse flow.

        Optical flow of a point at normalised coords (x, y) and depth Z, for a camera
        moving with linear velocity V and angular velocity w (both camera-frame):

            xdot = (-Vx + x*Vz)/Z  +  [ x*y*wx - (1 + x^2)*wy + y*wz ]
            ydot = (-Vy + y*Vz)/Z  +  [ (1 + y^2)*wx - x*y*wy - x*wz ]

        The bracketed rotational part depends only on the gyro, so subtract it and the
        remainder is linear in V. Rotational flow dominates at altitude -- a 0.1 rad/s
        body rate at 10 m looks like ~1 m/s of translation -- so de-rotation is not
        optional.
        """
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        x0 = (p0[:, 0] - cx) / fx
        y0 = (p0[:, 1] - cy) / fy
        xd = ((p1[:, 0] - cx) / fx - x0) / dt
        yd = ((p1[:, 1] - cy) / fy - y0) / dt
        wx, wy, wz = omega_cam
        xd = xd - (x0 * y0 * wx - (1.0 + x0 ** 2) * wy + y0 * wz)
        yd = yd - ((1.0 + y0 ** 2) * wx - x0 * y0 * wy - x0 * wz)

        good = np.isfinite(Z) & np.isfinite(xd) & np.isfinite(yd)
        if good.sum() < 8:
            return None, 0
        x0, y0, xd, yd, Z = x0[good], y0[good], xd[good], yd[good], Z[good]
        n = len(Z)
        A = np.zeros((2 * n, 3))
        b = np.empty(2 * n)
        A[0::2, 0] = -1.0 / Z
        A[0::2, 2] = x0 / Z
        b[0::2] = xd
        A[1::2, 1] = -1.0 / Z
        A[1::2, 2] = y0 / Z
        b[1::2] = yd
        V, *_ = np.linalg.lstsq(A, b, rcond=None)
        # one IRLS pass to shrug off tracking outliers the FB gate let through
        resid = np.abs(A @ V - b).reshape(-1, 2).max(axis=1)
        keep = resid < 3.0 * (np.median(resid) + 1e-9)
        if keep.sum() >= 8:
            m = np.repeat(keep, 2)
            V, *_ = np.linalg.lstsq(A[m], b[m], rcond=None)
        return V, int(keep.sum())

    # -------------------------------------------- translation from a homography
    def _translation_from_homography(self, H, R21, n_cam, d_cam):
        """Metric translation from a plane-induced homography, R21 supplied by the IMU.

        For X2 = R21 X1 + t and plane n^T X1 = d (camera-1 coords):
            H ~ K (R21 + t n^T / d) K^-1
        Normalise M = Kinv H K by its MIDDLE singular value (exactly 1 for the true
        matrix), then t = d (M - R21) n. Recovering R from H instead of trusting the
        IMU halves the translation -- covered by a regression test.
        """
        M = self.Kinv @ H @ self.K
        sv = np.linalg.svd(M, compute_uv=False)
        if not np.all(np.isfinite(sv)) or sv[1] < 1e-9:
            return None
        M = M / sv[1]
        if np.trace(M @ R21.T) < 0.0:        # H is defined up to sign as well as scale
            M = -M
        t = d_cam * ((M - R21) @ n_cam)
        return t if np.all(np.isfinite(t)) else None

    def _plane_normal_in_cam(self, rpy):
        """Ground-plane normal in camera coords, pointing away from the camera."""
        R_wc = euler_to_R_enu_flu(*rpy) @ self.R_cam_in_body
        n = R_wc.T @ np.array([0.0, 0.0, 1.0])       # world up, seen from the camera
        n = -n / np.linalg.norm(n)
        return n

    def _set_keyframe(self, img, rpy, alt):
        self.kf_img = img
        self.kf_pts = self._detect(img)
        self.kf_pos = self.pos.copy()
        self.kf_rpy = tuple(rpy)
        self.kf_depth = alt
        self.kf_resets += 1

    # -------------------------------------------------------------- main entry
    def step(self, img, depth, rpy, omega_body, dt, alt_agl):
        """Process one synchronised RGB-D frame.

        img        : mono8 or bgr8 image
        depth      : float32 depth image, metres, optical-axis Z, NaN where invalid
        rpy        : (roll, pitch, yaw) of the body in ENU/FLU, from the flight controller
        omega_body : body angular rate (rad/s) in FLU
        dt         : seconds since the previous frame
        alt_agl    : absolute height above ground, metres (from the depth image)
        """
        result = {'ok': False, 'mode': 'none', 'health': HEALTH_LOST,
                  'n_tracked': 0, 'n_kf_inliers': 0, 'quality': 0,
                  'pos': self.pos.copy(), 'vel': self.vel.copy(),
                  'kf_resets': self.kf_resets}

        g = self._prep(img)
        R_wb = euler_to_R_enu_flu(*rpy)
        R_wc = R_wb @ self.R_cam_in_body

        if self.prev_img is None or self.kf_img is None:
            self.prev_img, self.prev_pts = g, self._detect(g)
            self._set_keyframe(g, rpy, alt_agl)
            self.kf_resets = 0
            return result

        # ---- frame-to-frame: velocity -------------------------------------
        p0, p1 = self._track(self.prev_img, g, self.prev_pts)
        if p0 is None:
            self.consecutive_bad += 1
            self.health = HEALTH_LOST if self.consecutive_bad > 5 else HEALTH_DEGRADED
            self.prev_img, self.prev_pts = g, self._detect(g)
            result.update(mode='lost', health=self.health)
            return result

        self.n_tracked = len(p0)
        V_cam, _ = self._velocity_from_flow(
            p0, p1, self._depth_at(depth, p0),
            self.R_cam_in_body.T @ np.asarray(omega_body, dtype=float), dt)
        if V_cam is not None:
            v_new = R_wc @ V_cam
            a = self.p.vel_lpf_alpha
            self.vel = a * v_new + (1.0 - a) * self.vel

        # ---- keyframe registration: absolute, drift-free position ----------
        mode = 'dead_reckoning'
        kp0, kp1 = self._track(self.kf_img, g, self.kf_pts)
        if kp0 is not None and len(kp0) >= self.p.min_features:
            H, mask = cv2.findHomography(kp0, kp1, cv2.RANSAC,
                                         self.p.ransac_thresh, maxIters=2000)
            if H is not None and mask is not None and int(mask.sum()) >= self.p.min_features // 2:
                R_wc_kf = euler_to_R_enu_flu(*self.kf_rpy) @ self.R_cam_in_body
                R21 = R_wc.T @ R_wc_kf                      # keyframe-cam -> current-cam
                t21 = self._translation_from_homography(
                    H, R21, self._plane_normal_in_cam(self.kf_rpy), self.kf_depth)
                if t21 is not None:
                    delta_cam = -R21.T @ t21                # current cam in keyframe cam
                    self.pos = self.kf_pos + R_wc_kf @ delta_cam
                    self.n_kf_inliers = int(mask.sum())
                    mode = 'keyframe'
                    shift = np.linalg.norm(delta_cam[:2]) / max(alt_agl, 1e-3)
                    if shift > self.p.rekey_shift or self.n_kf_inliers < self.p.min_features:
                        self._set_keyframe(g, rpy, alt_agl)

        if mode != 'keyframe':                              # anchor lost -> integrate
            self.pos = self.pos + self.vel * dt
            self._set_keyframe(g, rpy, alt_agl)

        if np.isfinite(alt_agl):
            self.pos[2] = alt_agl                           # absolute height, no drift

        # ---- health / confidence ------------------------------------------
        self.consecutive_bad = 0
        if self.n_tracked < self.p.min_features or mode != 'keyframe':
            self.health = HEALTH_DEGRADED
        elif self.n_tracked < self.p.degraded_features:
            self.health = HEALTH_DEGRADED
        else:
            self.health = HEALTH_OK
        quality = int(np.clip(100.0 * self.n_tracked / self.p.degraded_features, 0, 100))
        if self.health == HEALTH_LOST:
            quality = 0

        self.prev_img = g
        self.prev_pts = (self._detect(g) if len(p1) < self.p.min_features
                         else p1.reshape(-1, 1, 2).astype(np.float32))

        result.update(ok=True, mode=mode, health=self.health, quality=quality,
                      n_tracked=self.n_tracked, n_kf_inliers=self.n_kf_inliers,
                      pos=self.pos.copy(), vel=self.vel.copy(),
                      kf_resets=self.kf_resets)
        return result
