"""Reference hover trajectory + exact analytic derivatives for the validation harness.

Models a real 10 m station-keeping flight: gust-driven lateral drift at a few tenths of a
metre, slow altitude breathing, and a few degrees of attitude wobble. Ground truth is
analytic, so position/velocity error is exact rather than estimated.
"""
import numpy as np


def trajectory(t):
    """-> (position ENU [m], attitude (roll, pitch, yaw) [rad]) at time t."""
    x = 0.55*np.sin(0.13*2*np.pi*t) + 0.30*np.sin(0.041*2*np.pi*t + 1.1) + 0.012*t
    y = 0.45*np.cos(0.10*2*np.pi*t) + 0.25*np.sin(0.067*2*np.pi*t + 0.4) - 0.008*t
    z = 10.0 + 0.10*np.sin(0.21*2*np.pi*t)
    roll = np.deg2rad(3.5)*np.sin(0.31*2*np.pi*t)
    pitch = np.deg2rad(3.0)*np.cos(0.27*2*np.pi*t + 0.7)
    yaw = np.deg2rad(4.0)*np.sin(0.05*2*np.pi*t)
    return np.array([x, y, z]), np.array([roll, pitch, yaw])


def deriv(f, t, h=1e-4):
    """Central-difference velocity of the position component of ``f``."""
    a, _ = f(t - h)
    b, _ = f(t + h)
    return (b - a) / (2 * h)


def ang_rate(t, h=1e-4):
    """Central-difference body angular rate (rad/s)."""
    _, a = trajectory(t - h)
    _, b = trajectory(t + h)
    return (b - a) / (2 * h)
