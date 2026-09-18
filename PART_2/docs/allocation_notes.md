# Allocation notes

The numbers behind the Part 2 README. Everything here can be reproduced with
`model/allocation.py` (a line-by-line port of PX4's multicopter allocation) and is pinned by
`model/test_allocation.py`.

## Effectiveness, pseudo-inverse, normalization

`ActuatorEffectivenessRotors::computeEffectivenessMatrix` builds, for each rotor at position
`r` with thrust axis `a = (0, 0, -1)`:

```
thrust = CT * a
moment = CT * (r x a) - CT * KM * a
```

With the x500 geometry (CT = 6.5):

```
            m1      m2      m3      m4
roll     -1.430   1.300   1.430  -1.300
pitch     0.845  -0.845   0.845  -0.845
yaw       0.325   0.325  -0.325  -0.325
thrust z -6.500  -6.500  -6.500  -6.500
```

`ControlAllocationPseudoInverse` takes the Moore-Penrose inverse and divides each column by
a scale so the rate controller's normalized requests map to sensible motor commands
(roll/pitch 0.418, yaw 0.806, thrust 0.0385). The resulting mix:

```
        roll    pitch    yaw   thrust
m1    -0.438   0.707   0.909   -1
m2     0.438  -0.707   1.000   -1
m3     0.438   0.707  -0.909   -1
m4    -0.438  -0.707  -1.000   -1
```

Two properties used in the README:

- Scaling column `i` by `k` scales row `i` of the pseudo-inverse by `1/k` (for a full-rank
  square block like the quad's roll/pitch/yaw/thrust). So the allocator asks `1/k` times
  more of that motor for the same request.
- At `k = 0` the column is zero, the minimum-norm solution gives that motor exactly 0, and the
  other three share the request in a least-squares sense. This is the same matrix the
  built-in `CA_FAILURE_MODE=1` handling produces.

The normalization scale is computed once from the healthy matrix and kept while a rotor is
degraded (`setHadActuatorFailure`). If it were recomputed on the scaled matrix, a 50 %
rotor would change the roll/pitch scale from 0.418 to 0.554, i.e. every motor's response to
a roll request would drop by about 25 % although only one rotor "changed".

## First instant after the change (hover request, thrust 0.729)

| scale | motor commands | what the allocator believes it produces (roll, pitch, yaw, thrust) | what the healthy rotors actually produce |
|---|---|---|---|
| 100 % | 0.73 0.73 0.73 0.73 | 0, 0, 0, 0.73 | 0, 0, 0, 0.73 |
| 75 % | 0.97 0.73 0.73 0.73 | 0, 0, 0, 0.73 | -0.15, +0.09, +0.06, 0.79 |
| 50 % | 1.00 0.50 0.50 0.50 | 0, 0, 0, **0.50** | -0.30, +0.18, +0.13, 0.63 |
| 25 % | 1.00 0.25 0.25 0.25 | 0, 0, 0, **0.25** | -0.45, +0.27, +0.20, 0.44 |
| 0 % | 0.00 0.14 0.88 1.00 | +0.06, -0.10, **-0.45**, 0.51 | same |

From 50 % down the allocator can't reach the requested thrust with the torque balance it
believes in, so the reduce-only thrust desaturation gives thrust away. The real motor 1
is then overdriven relative to the others: roll left, pitch up, yaw right, with less total
thrust than hover. At 0 % there's no mismatch any more (the allocator is right that motor 1
contributes nothing), but three rotors can't make zero yaw torque at hover thrust.

## Can the controller re-trim?

For 0 < k < 1 the physical hover trim (all motors 0.73) is still reachable if the rate
controller asks for the torque the allocator would believe that trim produces. That torque
has to come from the rate integrators, which PX4 limits to 0.3 (`MC_RR_INT_LIM`,
`MC_PR_INT_LIM`, `MC_YR_INT_LIM`).

| scale | torque request that yields the trim (roll, pitch, yaw) | allocator reproduces the trim? |
|---|---|---|
| 75 % | 0.11, -0.06, -0.05 | yes |
| 50 % | 0.22, -0.13, -0.10 | yes |
| 40 % | 0.26, -0.16, -0.12 | yes |
| 35 % | 0.28, -0.17, -0.12 | **no** |
| 25 % | **0.33**, -0.19, -0.14 | **no** |

At 35 % the torques still fit under 0.3 but the trim is lost anyway, because of the mixing
order. Sequential desaturation mixes roll, pitch and thrust first and yaw last. Motor 1's yaw
share is `1/k` times larger than normal, so before the (negative) yaw part is added, motor 1's
command is already above 1 and the thrust gets cut. Below 39 % the trim is out of
reach, and below about 31 % the roll torque needed is also beyond the integrator limit.

That is the boundary the closed-loop model shows: hold down to 40 %, lose altitude at
35 %. At 25 % the vehicle stays nearly level (max tilt 15° in the model) and simply cannot
make enough thrust, so it comes down in about 4 s instead of tumbling. At 0 % it
tumbles.

## Model

`model/quad_sim.py` couples the ported allocator with:

- rigid body with the x500 SDF mass (2.064 kg) and inertia, rotors at (±0.174, ±0.174) m,
  `motorConstant` 8.55e-6, `momentConstant` 0.016, first-order motor lag 12.5/25 ms,
  command 0..1 → 150..1000 rad/s (NaN → 0);
- PX4 default hold-mode cascade: position P (0.95 / 1.0) → velocity PID (1.8/0.4/0.2 xy,
  4/2/0 z) → thrust vector with 45° tilt limit → attitude P (6.5, 6.5, 2.8, yaw weight
  0.4) → rate PID (0.15/0.2/0.003 roll-pitch, 0.2/0.1 yaw) with the i-factor, the 0.3
  integrator limits and allocator-saturation anti-windup, all at 250 Hz.

Not modelled: sensors and EKF, the hover-thrust estimator, drag beyond a small linear term,
ground effect, rotor inertia coupling. Run `python3 model/quad_sim.py --plot docs/img` to
regenerate the table and the two figures.
