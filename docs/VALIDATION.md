# Validation record

How the numbers in the README were produced, and what they do and do not prove.

## Method

`tools/validate_vo.py` avoids the chicken-and-egg problem of validating an estimator inside a
simulator that is itself the thing under test. It renders a downward camera **analytically**:

- A procedurally textured ground plane at Z = 0 (`tools/simworld.py`).
- A pinhole camera with the depth sensor's real intrinsics (640×480, HFOV 1.274 rad → fx 432.5).
- Rays are intersected with the plane and the texture sampled via `cv2.remap`; the depth image is
  the exact optical-axis Z of that intersection, clipped like the gz sensor (0.2–19.1 m).
- The vehicle follows an analytic 10 m hover trajectory (`tools/_traj.py`) with gust-driven drift
  and a few degrees of attitude wobble, so position, velocity and body rates are known in closed
  form rather than estimated.

The VO is then fed the same signals it will see in flight: the image, the depth image, a **noisy**
attitude (standing in for EKF2's), a **noisy** gyro, and an altitude derived from the depth image.

## What each result establishes

| Result | What it proves |
|---|---|
| Nominal max error 0.234 m over 90 s | The keyframe-anchored formulation does not drift; the 1.5 m gate has ×6.4 margin |
| p95 latency 5.9 ms | The 60 ms budget is met with an order of magnitude spare, on CPU only |
| Lighting 0.39 m *with* CLAHE vs 0.97 m without | CLAHE is load-bearing, not decoration |
| Blur to σ 8 px still 0.307 m | The 21 px LK window tolerates realistic motion blur |
| 5 s dropout → recovery to 0.076 m, no jump | The keyframe survives vision loss, which is what makes automatic recovery work |
| Attitude sweep 0.1°→2.0° | Identifies attitude noise as the dominant error term and gives the budget (< ~1° RMS) |

## Two bugs this harness caught

Both pass a naive round-trip test, which is why the numeric assertions in
`test/test_vo_core.py` matter:

1. **Homography translation halved.** Recovering R by projecting `M = K⁻¹HK` onto SO(3) absorbs
   part of the `t·nᵀ/d` term. Measured ratio was exactly 0.50 against ground truth. Fixed by
   supplying the known IMU rotation and solving only for `t`.
2. **FLU↔FRD confused with ENU↔NED.** The world swap is `(x,y,z)→(y,x,−z)`; the body flip is
   `(x,y,z)→(x,−y,−z)`. Both are involutive with det +1, so a round-trip test passes while real
   attitudes come out wrong. Caught by a yaw-consistency check across 500 random attitudes.

A third issue was found by derivation rather than test: the camera→body rotation for the nadir
mount puts **image-right at body-RIGHT and image-down at body-BACKWARD**, a 90° roll about the
optical axis away from the naive assumption. `frames.camera_rotation_in_body()` derives it from
the SDF pose; `test_camera_rotation_matches_sdf_pose` pins the axes.

## Ground-texture measurement

At 10 m over the stock grey `ground_plane`, `goodFeaturesToTrack(500, 0.01, 12)` returns
**0 corners**. Over the generated texture it returns **500** (saturating the cap), before and
after CLAHE. This is why plan §3 comes before everything else.

Texture resolution rule: texture GSD (`plane_size / pixels`) must be finer than camera GSD
(`altitude / fx`). At 10 m with fx 432.5 the camera samples 2.31 cm/px, so a 120 m plane needs
≥ 4096 px.

## What this does NOT prove

- Nothing here has flown in Gazebo yet. Render latency, DDS jitter, EKF2 fusion dynamics and
  controller coupling are all unmodelled.
- Simulated depth is near-perfect; real stereo at 10 m is much worse.
- The synthetic ground is uniformly textured. Real scenes have bald patches, shadows and
  repetitive structure that can alias the homography.

Treat these as a **lower bound on error** and a check that the maths and frames are right —
which is exactly what they were built for.

## Reproduce

```bash
python3 -m pytest src/uav_vision/test -q          # 45 passed
cd tools && PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full
```
