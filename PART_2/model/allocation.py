"""PX4 multicopter control allocation (v1.16), ported to numpy.

Mirrors, in the same order as the C++:
  ActuatorEffectivenessRotors::computeEffectivenessMatrix   -> effectiveness()
  ControlAllocator::update_effectiveness_matrix_if_needed   -> Allocator.set_effectiveness()
  ControlAllocationPseudoInverse (incl. normalization)      -> Allocator._update_mix()
  ControlAllocationSequentialDesaturation, MC_AIRMODE = 0   -> Allocator.allocate()

plus the rotor effectiveness scaling from PART_2/px4 (scale_column). The flight code is
the C++; this exists so the maths can be tested and used in quad_sim.py without PX4.
"""
import numpy as np

FLT_EPSILON = 1.1920929e-07
MINIMUM_YAW_MARGIN = 0.15
ROLL, PITCH, YAW, TX, TY, TZ = range(6)

# CA_ROTORn_PX, _PY, _KM from ROMFS/.../airframes/4001_gz_x500 (FRD, metres).
# CA_ROTORn_CT is not set there, so it is the default 6.5.
X500_ROTORS = [(0.13, 0.22, 0.05),
               (-0.13, -0.20, 0.05),
               (0.13, -0.22, -0.05),
               (-0.13, 0.20, -0.05)]


def effectiveness(rotors=X500_ROTORS, ct=6.5):
    """6 x n effectiveness matrix: rows roll, pitch, yaw torque, thrust x, y, z."""
    B = np.zeros((6, len(rotors)))
    axis = np.array([0.0, 0.0, -1.0])            # thrust points up (-z in FRD)
    for i, (px, py, km) in enumerate(rotors):
        position = np.array([px, py, 0.0])
        B[:3, i] = ct * np.cross(position, axis) - ct * km * axis
        B[3:, i] = ct * axis
    return B


def scale_column(B, motor_idx, scale):
    """The PART_2 change: multiply one motor's whole column by `scale`."""
    B = B.copy()
    B[:, motor_idx] *= scale
    return B


def allocation_scale(mix):
    """ControlAllocationPseudoInverse::updateControlAllocationMatrixScale (normalize_rpy on)."""
    s = np.ones(6)
    n_roll = np.count_nonzero(np.abs(mix[:, ROLL]) > 1e-3)
    n_pitch = np.count_nonzero(np.abs(mix[:, PITCH]) > 1e-3)
    roll = np.sqrt(np.sum(mix[:, ROLL] ** 2) / (n_roll / 2.0)) if n_roll else 1.0
    pitch = np.sqrt(np.sum(mix[:, PITCH] ** 2) / (n_pitch / 2.0)) if n_pitch else 1.0
    s[ROLL] = s[PITCH] = max(roll, pitch)
    s[YAW] = mix[:, YAW].max()
    for axis in (TZ, TY, TX):                    # z first; x/y fall back to the z scale
        norms = np.abs(mix[:, axis])
        n = np.count_nonzero(norms > FLT_EPSILON)
        s[axis] = norms.sum() / n if n else s[TZ]
    return s


def normalize(mix, s):
    """ControlAllocationPseudoInverse::normalizeControlAllocationMatrix."""
    mix = mix.copy()
    if s[ROLL] > FLT_EPSILON:
        mix[:, ROLL] /= s[ROLL]
        mix[:, PITCH] /= s[PITCH]
    if s[YAW] > FLT_EPSILON:
        mix[:, YAW] /= s[YAW]
    if s[TX] > FLT_EPSILON:
        mix[:, TX:] /= s[TX:]
    mix[np.abs(mix) < 1e-3] = 0.0
    return mix


class Allocator:
    """Sequential-desaturation allocator for motors in [0, 1] (MC_AIRMODE = 0)."""

    def __init__(self, B):
        self.had_actuator_failure = False
        self.scale = np.ones(6)
        self.set_effectiveness(B, update_normalization=True)

    def set_effectiveness(self, B, update_normalization):
        B = np.array(B, dtype=float)
        # rows where every entry is small are dropped (ControlAllocator does this too)
        B[np.all(np.abs(B) <= 0.05, axis=1)] = 0.0
        self.B = B
        mix = np.linalg.pinv(B)
        if update_normalization and not self.had_actuator_failure:
            self.scale = allocation_scale(mix)
        self.mix = normalize(mix, self.scale)
        self.n = B.shape[1]
        self.umin, self.umax = np.zeros(self.n), np.ones(self.n)

    def allocated(self, u):
        """Normalized wrench the allocator believes u produces (getAllocatedControl)."""
        return (self.B @ u) * self.scale

    # --- sequential desaturation -------------------------------------------
    def _gain(self, v, u, umin, umax):
        k_min = k_max = 0.0
        for i in range(self.n):
            if abs(v[i]) < 0.2:                   # don't desaturate with weak actuators
                continue
            for bound, over in ((umin[i], u[i] < umin[i]), (umax[i], u[i] > umax[i])):
                if over:
                    k = (bound - u[i]) / v[i]
                    k_min, k_max = min(k_min, k), max(k_max, k)
        return k_min + k_max

    def _desaturate(self, u, v, umin, umax, increase_only=False):
        g = self._gain(v, u, umin, umax)
        if increase_only and g < 0.0:
            return u
        u = u + g * v
        return u + 0.5 * self._gain(v, u, umin, umax) * v

    def allocate(self, c):
        """c = normalized [roll, pitch, yaw torque, thrust x, y, z] -> motor commands."""
        c = np.asarray(c, dtype=float)
        m = self.mix
        u = m[:, [ROLL, PITCH, TX, TY, TZ]] @ c[[ROLL, PITCH, TX, TY, TZ]]
        u = self._desaturate(u, m[:, TZ], self.umin, self.umax, increase_only=True)
        u = self._desaturate(u, m[:, ROLL], self.umin, self.umax)
        u = self._desaturate(u, m[:, PITCH], self.umin, self.umax)
        # yaw last, with a little headroom above max so some yaw survives at full thrust
        u = u + m[:, YAW] * c[YAW]
        umax_yaw = self.umax + (self.umax - self.umin) * MINIMUM_YAW_MARGIN
        u = self._desaturate(u, m[:, YAW], self.umin, umax_yaw)
        u = self._desaturate(u, m[:, TZ], self.umin, self.umax, increase_only=True)
        return np.clip(u, self.umin, self.umax)


def degraded_allocator(motor_idx, scale, B=None):
    """Allocator after CA_EFF_MOTOR/CA_EFF_SCALE were applied in flight.

    Normalization stays at the healthy value (setHadActuatorFailure(true) in the patch).
    """
    B = effectiveness() if B is None else B
    a = Allocator(B)
    a.had_actuator_failure = scale < 1.0
    a.set_effectiveness(scale_column(B, motor_idx, scale), update_normalization=True)
    return a
