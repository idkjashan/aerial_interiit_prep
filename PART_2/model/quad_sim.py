"""Small closed-loop model of the gz x500 + PX4 hold mode, used to predict Part 2 results.

Physics comes from the x500 SDF in PX4-gazebo-models (mass, inertia, rotor positions,
motorConstant, momentConstant, time constants). The controller is PX4's default cascade
(position P -> velocity PID -> attitude P with yaw weight -> rate PID with integrator
limits and allocator-saturation anti-windup), and the allocator is allocation.py.

It is a model: no sensor noise, no EKF, no hover-thrust estimator, no ground effect.
Use it for the shape of the response and to sanity check the Gazebo runs, not as data.

    python3 quad_sim.py                 # sweep 100/75/50/25/0 % and the built-in reference
    python3 quad_sim.py --level 0.5     # one level
"""
import argparse

import numpy as np

from allocation import Allocator, effectiveness, scale_column

G = 9.80665
MASS = 2.0 + 4 * 0.016077                     # base_link + rotors
J = np.diag([0.021667, 0.021667, 0.04])
KT, KM = 8.54858e-06, 0.016                   # motorConstant, momentConstant
TAU_UP, TAU_DOWN = 0.0125, 0.025
W_MIN, W_MAX = 150.0, 1000.0                  # SIM_GZ_EC_MIN / _MAX
# rotor positions in FRD (SDF is FLU: y flips) and spin: +1 = ccw from above
ROTOR_POS = np.array([[0.174, 0.174, 0.0], [-0.174, -0.174, 0.0],
                      [0.174, -0.174, 0.0], [-0.174, 0.174, 0.0]])
ROTOR_DIR = np.array([1.0, 1.0, -1.0, -1.0])

# PX4 defaults (mc_rate_control, mc_att_control, mc_pos_control)
RATE_P = np.array([0.15, 0.15, 0.2])
RATE_I = np.array([0.2, 0.2, 0.1])
RATE_D = np.array([0.003, 0.003, 0.0])
RATE_INT_LIM = np.array([0.3, 0.3, 0.3])
RATE_MAX = np.radians([220.0, 220.0, 200.0])
ATT_P = np.array([6.5, 6.5, 2.8])
YAW_WEIGHT = 0.4
XY_P, Z_P = 0.95, 1.0
XY_VEL_PID = (1.8, 0.4, 0.2)
Z_VEL_PID = (4.0, 2.0, 0.0)
TILT_MAX = np.radians(45.0)
THR_MIN, THR_MAX = 0.12, 1.0
HOVER_THRUST = (np.sqrt(MASS * G / (4 * KT)) - W_MIN) / (W_MAX - W_MIN)


# --- quaternion helpers, Hamilton [w, x, y, z], body FRD -> world NED ------------
def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qinv(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def qdcm(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def qfrom_dcm(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def qfrom_two_vectors(a, b):
    """Shortest rotation taking unit vector a to unit vector b."""
    c = np.cross(a, b)
    q = np.array([1.0 + np.dot(a, b), *c])
    n = np.linalg.norm(q)
    return q / n if n > 1e-9 else np.array([0.0, 1.0, 0.0, 0.0])


def euler(q):
    w, x, y, z = q
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


# --- controllers -------------------------------------------------------------
class Px4Hold:
    """Position hold -> attitude -> rate, as in PX4 (simplified where noted)."""

    def __init__(self, target):
        self.target = np.asarray(target, float)
        self.vel_int = np.zeros(3)
        self.rate_int = np.zeros(3)
        self.prev_rate = np.zeros(3)
        self.rate_d = np.zeros(3)
        self.prev_vel = np.zeros(3)
        self.yaw_sp = 0.0
        self.sat_pos = np.zeros(3, bool)
        self.sat_neg = np.zeros(3, bool)

    def position(self, p, v, dt):
        vel_sp = np.r_[XY_P * (self.target[:2] - p[:2]), Z_P * (self.target[2] - p[2])]
        vel_sp[2] = np.clip(vel_sp[2], -3.0, 1.5)
        err = vel_sp - v
        dv = (v - self.prev_vel) / dt
        self.prev_vel = v.copy()
        p_gain = np.array([XY_VEL_PID[0], XY_VEL_PID[0], Z_VEL_PID[0]])
        i_gain = np.array([XY_VEL_PID[1], XY_VEL_PID[1], Z_VEL_PID[1]])
        d_gain = np.array([XY_VEL_PID[2], XY_VEL_PID[2], Z_VEL_PID[2]])
        acc_sp = p_gain * err + self.vel_int - d_gain * dv
        self.vel_int = np.clip(self.vel_int + i_gain * err * dt, -G, G)
        body_z = np.array([-acc_sp[0], -acc_sp[1], G])
        body_z /= np.linalg.norm(body_z)
        tilt = np.arccos(np.clip(body_z[2], -1, 1))
        if tilt > TILT_MAX:                         # ControlMath::limitTilt
            xy = body_z[:2] / max(np.linalg.norm(body_z[:2]), 1e-9)
            body_z = np.r_[xy * np.sin(TILT_MAX), np.cos(TILT_MAX)]
        thrust_z = acc_sp[2] * HOVER_THRUST / G - HOVER_THRUST
        collective = min(thrust_z / body_z[2], -THR_MIN)
        thr = np.clip(-collective, THR_MIN, THR_MAX)
        # thrustToAttitude: body z axis points opposite the thrust
        z_b = body_z
        x_c = np.array([np.cos(self.yaw_sp), np.sin(self.yaw_sp), 0.0])
        y_b = np.cross(z_b, x_c)
        y_b /= np.linalg.norm(y_b)
        x_b = np.cross(y_b, z_b)
        return qfrom_dcm(np.column_stack([x_b, y_b, z_b])), thr

    def attitude(self, q, qd):
        """AttitudeControl::update with yaw weight."""
        e_z, e_z_d = qdcm(q)[:, 2], qdcm(qd)[:, 2]
        qd_red = qfrom_two_vectors(e_z, e_z_d)
        qd_red = qmul(qd_red, q)
        q_mix = qmul(qinv(qd_red), qd)
        q_mix *= np.sign(q_mix[0]) if q_mix[0] != 0 else 1.0
        q_mix[0], q_mix[3] = np.clip(q_mix[0], -1, 1), np.clip(q_mix[3], -1, 1)
        qd = qmul(qd_red, np.array([np.cos(YAW_WEIGHT * np.arccos(q_mix[0])), 0, 0,
                                    np.sin(YAW_WEIGHT * np.arcsin(q_mix[3]))]))
        qe = qmul(qinv(q), qd)
        eq = 2.0 * np.sign(qe[0] if qe[0] != 0 else 1.0) * qe[1:]
        gain = ATT_P.copy()
        gain[2] /= YAW_WEIGHT
        return np.clip(eq * gain, -RATE_MAX, RATE_MAX)

    def rate(self, rate, rate_sp, dt):
        """RateControl::update: PID, D on measurement (30 Hz LPF), limited integrator."""
        alpha = dt / (dt + 1.0 / (2 * np.pi * 30.0))
        self.rate_d += alpha * ((rate - self.prev_rate) / dt - self.rate_d)
        self.prev_rate = rate.copy()
        err = rate_sp - rate
        torque = RATE_P * err + self.rate_int - RATE_D * self.rate_d
        for i in range(3):
            if (self.sat_pos[i] and err[i] > 0) or (self.sat_neg[i] and err[i] < 0):
                continue
            i_factor = max(0.0, 1.0 - (err[i] / np.radians(400.0)) ** 2)
            self.rate_int[i] = np.clip(self.rate_int[i] + i_factor * RATE_I[i] * err[i] * dt,
                                       -RATE_INT_LIM[i], RATE_INT_LIM[i])
        return torque


# --- simulation --------------------------------------------------------------
def simulate(level=1.0, motor=0, case='scale', t_event=5.0, duration=25.0,
             altitude=20.0, detect_delay=0.02, dt=0.001, ctrl_dt=0.004):
    """Hover at `altitude`, then at t_event:

    case='scale'      our mechanism: scale `motor`'s column to `level`, motor untouched
    case='builtin'    PX4 CA_FAILURE_MODE=1: after `detect_delay` the motor is removed
                      from the allocation and its output set to NaN (stopped)
    case='stop_only'  motor stops but the allocator is not told (CA_FAILURE_MODE=0, or
                      newer PX4 'failure motor off' without detection)

    Returns a dict of time series."""
    B = effectiveness()
    alloc = Allocator(B)
    ctrl = Px4Hold([0.0, 0.0, -altitude])

    p, v = np.array([0.0, 0.0, -altitude]), np.zeros(3)
    q, w = np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3)
    omega = np.full(4, W_MIN + (W_MAX - W_MIN) * HOVER_THRUST)
    u = np.full(4, HOVER_THRUST)
    stopped = np.zeros(4, bool)
    scale, degraded = 1.0, False
    ctrl.vel_int[2] = 0.0

    log = {k: [] for k in ('t', 'scale', 'x', 'y', 'z', 'vz', 'roll', 'pitch', 'yaw',
                           'p', 'q', 'r', 'u', 'omega', 'unalloc')}
    n_ctrl = int(round(ctrl_dt / dt))
    unalloc = np.zeros(3)
    for k in range(int(duration / dt)):
        t = k * dt
        if not degraded and t >= t_event:
            degraded = True
            if case == 'scale':
                scale = level
                alloc.had_actuator_failure = scale < 1.0
                alloc.set_effectiveness(scale_column(B, motor, scale), update_normalization=True)
            elif case == 'stop_only':
                stopped[motor] = True
        if case == 'builtin' and degraded and not stopped[motor] and t >= t_event + detect_delay:
            stopped[motor] = True                  # NaN output once CA handles the failure
            alloc.had_actuator_failure = True
            scale = 0.0
            alloc.set_effectiveness(scale_column(B, motor, 0.0), update_normalization=False)

        if k % n_ctrl == 0:
            qd, thr = ctrl.position(p, v, ctrl_dt)
            rate_sp = ctrl.attitude(q, qd)
            torque = ctrl.rate(w, rate_sp, ctrl_dt)
            c = np.r_[torque, 0.0, 0.0, -thr]
            u = alloc.allocate(c)
            unalloc = (c - alloc.allocated(u))[:3]
            ctrl.sat_pos, ctrl.sat_neg = unalloc > 1e-6, unalloc < -1e-6

        # rotors
        w_cmd = np.where(stopped, 0.0, W_MIN + (W_MAX - W_MIN) * u)
        tau = np.where(w_cmd > omega, TAU_UP, TAU_DOWN)
        omega += (w_cmd - omega) * dt / tau
        thrust = KT * omega ** 2

        # rigid body
        R = qdcm(q)
        f_body = np.array([0.0, 0.0, -thrust.sum()])
        acc = R @ f_body / MASS + np.array([0.0, 0.0, G]) - 0.1 * v
        m_body = np.zeros(3)
        for i in range(4):
            m_body += np.cross(ROTOR_POS[i], [0.0, 0.0, -thrust[i]])
            m_body[2] += ROTOR_DIR[i] * KM * thrust[i]
        w_dot = np.linalg.solve(J, m_body - np.cross(w, J @ w))
        v = v + acc * dt
        p = p + v * dt
        w = w + w_dot * dt
        q = qmul(q, np.r_[1.0, 0.5 * w * dt])
        q /= np.linalg.norm(q)

        if k % 10 == 0:                            # log at 100 Hz
            roll, pitch, yaw = euler(q)
            for key, val in (('t', t), ('scale', scale), ('x', p[0]), ('y', p[1]),
                             ('z', p[2]), ('vz', v[2]), ('roll', roll), ('pitch', pitch),
                             ('yaw', yaw), ('p', w[0]), ('q', w[1]), ('r', w[2]),
                             ('u', np.where(stopped, np.nan, u)), ('omega', omega.copy()),
                             ('unalloc', unalloc.copy())):
                log[key].append(val)
        if p[2] >= 0.0:                            # hit the ground
            break
    out = {k: np.array(v) for k, v in log.items()}
    out['tilt'] = np.arccos(np.clip(np.cos(out['roll']) * np.cos(out['pitch']), -1, 1))
    return out


def summary(d, t_event, motor=0):
    """A few numbers per run; the same ones plot_log.py reports for real logs."""
    after = d['t'] >= t_event
    tilt = np.degrees(np.arccos(np.clip(np.cos(d['roll']) * np.cos(d['pitch']), -1, 1)))
    z0 = d['z'][~after][-1] if (~after).any() else d['z'][0]
    crashed = d['z'][-1] > -0.5
    last = d['t'] >= d['t'][-1] - 5.0
    u_end = d['u'][after & last][:, motor]            # NaN = stopped
    return {
        'alt_loss_m': max(0.0, float(np.max(d['z'][after] - z0))),
        'max_tilt_deg': float(tilt[after].max()),
        'max_yaw_rate_dps': float(np.degrees(np.abs(d['r'][after])).max()),
        'final_tilt_deg': float(tilt[last].mean()),
        'motor_u_mean': (float(np.nanmean(u_end)) if np.isfinite(u_end).mean() > 0.5 else float('nan')),
        'crashed': bool(crashed),
        't_ground_s': float(d['t'][-1] - t_event) if crashed else None,
    }


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--level', type=float, help='single effectiveness level, 0..1')
    ap.add_argument('--motor', type=int, default=1, help='motor number, 1-based')
    ap.add_argument('--duration', type=float, default=25.0)
    ap.add_argument('--plot', metavar='DIR', help='also write comparison figures to DIR')
    a = ap.parse_args()
    runs = ([(f'{a.level:.0%}', dict(level=a.level))] if a.level is not None else
            [(f'{k:.0%}', dict(level=k)) for k in (1.0, 0.75, 0.5, 0.25, 0.0)] +
            [('built-in handling', dict(case='builtin')),
             ('motor stop only', dict(case='stop_only'))])
    print(f'hover thrust {HOVER_THRUST:.3f}, motor {a.motor}, event at t=5 s')
    print(f'{"case":<22}{"alt loss m":>11}{"max tilt":>10}{"max yaw rate":>14}'
          f'{"final tilt":>12}{"motor u":>9}   outcome')
    results = []
    for name, kw in runs:
        d = simulate(motor=a.motor - 1, duration=a.duration, **kw)
        results.append((name, d))
        s = summary(d, 5.0, a.motor - 1)
        outcome = (f'hits ground after {s["t_ground_s"]:.1f} s' if s['crashed'] else
                   'holds' if s['final_tilt_deg'] < 5 and s['alt_loss_m'] < 2 else 'airborne, degraded')
        print(f'{name:<22}{s["alt_loss_m"]:>11.2f}{s["max_tilt_deg"]:>9.1f}°'
              f'{s["max_yaw_rate_dps"]:>12.0f}°/s{s["final_tilt_deg"]:>11.1f}°'
              f'{s["motor_u_mean"]:>9.2f}   {outcome}'.replace('      nan', '  stopped'))
    if a.plot:
        import os
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
        from plot_log import plot_compare
        os.makedirs(a.plot, exist_ok=True)
        m = a.motor - 1
        levels = [(n, d, 5.0, m) for n, d in results if n.endswith('%')]
        plot_compare(levels, os.path.join(a.plot, 'model_levels.png'), window=(-1.0, 20.0))
        loss = [(n, d, 5.0, m) for n, d in results if n in ('0%', 'built-in handling', 'motor stop only')]
        if loss:
            plot_compare(loss, os.path.join(a.plot, 'model_complete_loss.png'), window=(-0.5, 4.0))
        print(f'figures in {a.plot}')
