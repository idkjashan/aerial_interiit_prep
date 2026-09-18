"""Checks on the allocation maths behind the PART_2 PX4 change.  Run: pytest -q PART_2/model"""
import numpy as np
import pytest

from allocation import (Allocator, degraded_allocator, effectiveness, scale_column,
                        ROLL, PITCH, YAW, TZ)

B = effectiveness()
HOVER = np.array([0, 0, 0, 0, 0, -0.73])      # normalized hover thrust of the gz x500


def test_healthy_hover_is_symmetric():
    u = Allocator(B).allocate(HOVER)
    assert np.allclose(u, 0.73)


@pytest.mark.parametrize('k', [0.9, 0.8, 0.75])
def test_allocator_compensates_a_rotor_it_believes_is_weak(k):
    """Unsaturated: rotor 1 is driven 1/k harder, the others are untouched."""
    u = degraded_allocator(0, k).allocate(HOVER)
    assert u[0] == pytest.approx(min(0.73 / k, 1.0))
    assert np.allclose(u[1:], 0.73)


def test_scaled_column_saturates_the_healthy_rotor_below_hover_thrust():
    """Below k = hover thrust the allocator can't keep its own torque balance at hover.

    It holds the motor at 100 % and gives up collective thrust (reduce-only desaturation).
    """
    a = degraded_allocator(0, 0.5)
    u = a.allocate(HOVER)
    assert u[0] == pytest.approx(1.0)
    believed = a.allocated(u)
    assert np.allclose(believed[:3], 0.0, atol=1e-6)     # torque it thinks it makes
    assert -believed[TZ] < 0.73                          # ...at the cost of thrust


def test_physical_disturbance_direction():
    """Real (unscaled) rotors: the overdriven front-right CCW rotor rolls left, pitches up, yaws right."""
    a = degraded_allocator(0, 0.5)
    physical = (B @ a.allocate(HOVER)) * a.scale
    assert physical[ROLL] < 0 and physical[PITCH] > 0 and physical[YAW] > 0


def test_zero_scale_removes_the_motor_exactly():
    a = degraded_allocator(0, 0.0)
    assert np.all(a.mix[0] == 0.0)
    for c in (HOVER, [0.2, -0.1, 0.3, 0, 0, -0.5]):
        assert a.allocate(c)[0] == 0.0


def test_zero_scale_matches_builtin_failure_handling():
    """CA_FAILURE_MODE=1 zeroes the column with normalization frozen - same allocator."""
    builtin = Allocator(B)
    builtin.had_actuator_failure = True
    zeroed = B.copy()
    zeroed[:, 0] = 0.0
    builtin.set_effectiveness(zeroed, update_normalization=False)
    ours = degraded_allocator(0, 0.0)
    assert np.allclose(builtin.mix, ours.mix)
    assert np.allclose(builtin.allocate(HOVER), ours.allocate(HOVER))


def test_three_rotor_hover_gives_up_yaw_and_thrust():
    a = degraded_allocator(0, 0.0)
    believed = a.allocated(a.allocate(HOVER))
    assert abs(believed[YAW]) > 0.3            # can't hold yaw on 3 rotors
    assert -believed[TZ] < 0.73                # and can't make hover thrust balanced


@pytest.mark.parametrize('k', [0.75, 0.5, 0.25])
def test_normalization_is_frozen_while_degraded(k):
    healthy = Allocator(B)
    assert np.allclose(degraded_allocator(0, k).scale, healthy.scale)
    # re-normalizing instead (what a plain CA_ROTOR0_CT change would do) moves every gain
    assert not np.allclose(Allocator(scale_column(B, 0, k)).scale, healthy.scale)


def test_equilibrium_torque_offsets_fit_in_rate_integrators():
    """Torque the rate integrators must hold so the allocator outputs the true hover trim.

    PX4's rate integrator limit is 0.3 (MC_RR/PR/YR_INT_LIM). Down to 50 % the offsets fit
    and the allocator reproduces the trim exactly. At 25 % the roll offset is past the
    limit and the yaw-last desaturation no longer reproduces the trim, so the model
    predicts a lasting attitude/thrust error there.
    """
    trim = np.full(4, 0.73)
    for k in (0.75, 0.5):
        a = degraded_allocator(0, k)
        v = a.allocated(trim)
        assert np.all(np.abs(v[:3]) < 0.3)
        assert np.allclose(a.allocate(v), trim, atol=1e-4)
    a = degraded_allocator(0, 0.25)
    v = a.allocated(trim)
    assert abs(v[ROLL]) > 0.3
    assert not np.allclose(a.allocate(v), trim, atol=1e-2)
