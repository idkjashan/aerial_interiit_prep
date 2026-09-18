# Offline validation

How the offline numbers in the README were produced and what they do and don't show.

## Method

Validating an estimator inside the simulator that is also under test makes it hard to tell
which one is wrong, so `tools/validate_vo.py` renders the camera itself:

- a procedurally textured ground plane at Z = 0 (`tools/simworld.py`);
- a pinhole camera with the depth sensor's intrinsics (640×480, HFOV 1.274 rad, fx 432.5);
- rays intersected with the plane and the texture sampled with `cv2.remap`; the depth image
  is the exact optical-axis Z, clipped like the gz sensor (0.2–19.1 m);
- an analytic 10 m hover trajectory (`tools/_traj.py`) with gust-like drift and a few degrees
  of attitude wobble, so position, velocity and body rates are known exactly.

The VO gets what it gets in flight: the image, the depth image, a noisy attitude (standing
in for EKF2's), a noisy gyro and the height from the depth image.

```bash
cd PART_1/tools
PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full
```

## Results

| Scenario | Max horizontal error | Mean | Velocity RMSE | p95 latency |
|---|---|---|---|---|
| Nominal 90 s hover | 0.234 m | 0.072 m | 0.050 m/s | 6.0 ms |
| Lighting ±45 % | 0.477 m | 0.251 m | 0.051 m/s | 7.8 ms |
| Motion blur σ 4.5–8 px | 0.221 m | 0.076 m | 0.065 m/s | 10.1 ms |
| Near-featureless ground, 5 % contrast | 0.234 m | 0.073 m | 0.051 m/s | 6.7 ms |
| 5 s total dropout | 0.228 m | 0.073 m | 0.051 m/s | 6.4 ms |

During the dropout health goes DEGRADED then LOST with quality 0; afterwards the error is
0.076 m against 0.074 m before, so position comes back without a jump.

Attitude sweep (30 s runs):

| attitude noise | 0.1° | 0.3° | 0.6° | 1.0° | 2.0° |
|---|---|---|---|---|---|
| max error | 0.078 m | 0.234 m | 0.467 m | 0.777 m | 1.545 m |

A constant bias of 1° or 2° gives 0.165 m and 0.173 m: it is the same in the keyframe and in
the current frame and mostly cancels. Noise doesn't, so EKF2's attitude noise is the budget
to watch (under ~1° RMS for the 1.5 m limit at 10 m).

What each result shows:

- The keyframe formulation doesn't drift over 90 s, with ×6 margin to the 1.5 m limit.
- The 60 ms budget is met on the CPU with ~10× to spare.
- The 21 px LK window copes with realistic motion blur.
- The keyframe survives a dropout, which is what makes the automatic recovery work.

**CLAHE.** With the first version of the re-keying logic, CLAHE brought the lighting case
from 0.97 m down to 0.39 m. After the re-keying was made stricter (re-key only after a real
translation and onto a well-tracked frame) that is no longer true on this synthetic test:
0.477 m with CLAHE, 0.392 m without. We kept CLAHE on because it is the configuration the
Gazebo hover was flown with and Gazebo's lighting is different from this synthetic model;
`clahe_clip` in `config/vo_params.yaml` is the knob if it needs revisiting.

## Two bugs this caught

Both pass a naive round-trip test, which is why `test/test_vo_core.py` checks numbers:

1. **Homography translation halved.** Recovering R by projecting `M = K⁻¹HK` onto SO(3)
   absorbs part of the `t·nᵀ/d` term; the recovered translation was exactly 0.5× the truth.
   Fixed by using the IMU rotation and solving only for `t`.
2. **FLU↔FRD mixed up with ENU↔NED.** The world swap is `(x, y, z) → (y, x, −z)`, the body
   flip is `(x, y, z) → (x, −y, −z)`. Both are their own inverse with det +1, so a
   round-trip passes while real attitudes come out wrong. Caught by a yaw-consistency check
   over random attitudes.

A third was found on paper: for the nadir mount, image-right is body right and image-down
is body backward, a 90° roll about the optical axis from the obvious guess.
`frames.camera_rotation_in_body()` derives it from the SDF pose and
`test_camera_rotation_matches_sdf_pose` pins the axes.

## Ground texture

At 10 m over the stock grey `ground_plane`, `goodFeaturesToTrack(500, 0.01, 12)` returns 0
corners. Over the generated texture it returns 500 (the cap), with or without CLAHE. The
texture has to be finer than the camera: plane size / pixels < altitude / fx, i.e. at least
4096 px for a 120 m plane at 10 m.

## What this does not show

- Render latency, DDS jitter, EKF2 fusion dynamics and the controller in the loop are not
  modelled; the Gazebo results in the README cover those.
- Simulated depth is near-perfect; real stereo at 10 m is much worse.
- The synthetic ground is uniformly textured. Real ground has bare patches, shadows and
  repeating patterns that can confuse the homography.
