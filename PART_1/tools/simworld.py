"""Synthetic downward-camera simulator over a textured ground plane.

Reproduces the geometry of the x500_depth_down airframe so the VO algorithm can be
validated numerically (ground truth known exactly) before touching Gazebo.

Frames
------
World : ENU  (+X East, +Y North, +Z Up), ground plane at Z = 0
Body  : FLU  (+X Fwd,  +Y Left, +Z Up)
Camera: RDF  (+X right-in-image, +Y down-in-image, +Z along optical axis)

With the body level and yaw = 0 the camera optical axis (+Z_cam) points at world -Z.
"""
import numpy as np, cv2

# camera(RDF) expressed in body(FLU) axes, for a perfectly downward-looking camera.
# columns = camera axes written in body coordinates
R_CAM_IN_BODY = np.array([[1.0, 0.0,  0.0],
                          [0.0,-1.0,  0.0],
                          [0.0, 0.0, -1.0]])


def euler_to_R(roll, pitch, yaw):
    """FLU body -> ENU world, Z-Y-X intrinsic (yaw, pitch, roll)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def make_K(width, height, hfov):
    fx = (width / 2.0) / np.tan(hfov / 2.0)
    fy = fx                                  # square pixels, as gz renders them
    return np.array([[fx, 0.0, width / 2.0 - 0.5],
                     [0.0, fy, height / 2.0 - 0.5],
                     [0.0, 0.0, 1.0]])


def make_ground(size_px=4000, extent_m=40.0, seed=0):
    """Procedural high-frequency ground texture (asphalt/gravel-like)."""
    rng = np.random.default_rng(seed)
    img = np.zeros((size_px, size_px), np.float32)
    for octave, amp in [(8, 0.5), (16, 0.28), (40, 0.18), (110, 0.12), (300, 0.10)]:
        n = rng.random((octave, octave)).astype(np.float32)
        img += amp * cv2.resize(n, (size_px, size_px), interpolation=cv2.INTER_CUBIC)
    # sprinkle discrete blobs -> strong, well-localised Shi-Tomasi corners
    for _ in range(9000):
        c = rng.integers(0, size_px, 2)
        r = int(rng.integers(2, 9))
        cv2.circle(img, (int(c[0]), int(c[1])), r, float(rng.random()), -1)
    img = cv2.GaussianBlur(img, (0, 0), 1.0)
    img -= img.min(); img /= max(img.max(), 1e-9)
    return (img * 235 + 10).astype(np.uint8), extent_m / size_px  # image, metres-per-texel


class DownCam:
    """Renders RGB + optical-axis depth for a downward camera over the ground plane."""

    def __init__(self, ground, m_per_px, width, height, hfov, near=0.2, far=19.1):
        self.g, self.s = ground, m_per_px
        self.W, self.H = width, height
        self.K = make_K(width, height, hfov)
        self.Kinv = np.linalg.inv(self.K)
        self.near, self.far = near, far
        u, v = np.meshgrid(np.arange(width, dtype=np.float32),
                           np.arange(height, dtype=np.float32))
        # unit-z rays in camera frame, precomputed
        self.rays = np.stack([(u - self.K[0, 2]) / self.K[0, 0],
                              (v - self.K[1, 2]) / self.K[1, 1],
                              np.ones_like(u)], -1)          # (H,W,3)

    def render(self, pos, rpy):
        """pos: (x,y,z) ENU metres. rpy: (roll,pitch,yaw) rad. -> (gray, depth32f)"""
        R_wc = euler_to_R(*rpy) @ R_CAM_IN_BODY
        d_w = self.rays @ R_wc.T                              # (H,W,3) ray dirs in world
        with np.errstate(divide='ignore', invalid='ignore'):
            t = -pos[2] / d_w[..., 2]                         # depth along optical axis
        t[~np.isfinite(t)] = np.nan
        t[t <= 0] = np.nan
        P = pos[None, None, :] + t[..., None] * d_w           # ground intersection (world)
        # world XY -> texture pixel. texture centred on world origin, +Y north = image up
        half = self.g.shape[0] / 2.0
        mx = (P[..., 0] / self.s + half).astype(np.float32)
        my = (half - P[..., 1] / self.s).astype(np.float32)
        img = cv2.remap(self.g, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        depth = t.astype(np.float32)
        depth[(depth < self.near) | (depth > self.far)] = np.nan   # gz clips like this
        return img, depth
