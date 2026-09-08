# Part 1 — Vision-Based Navigation System: Implementation Plan

**Audience:** the coding agent working on the Ubuntu 22.04 machine (`~/PX4-Autopilot`,
`~/px4_ros_ws`, ROS 2 Humble, Gazebo Harmonic 8.12, PX4 v1.16.0-rc1, branch `drdo_depth_down`).

**Status of this repo:** the VO algorithm, the frame maths and the ROS 2 nodes in
`src/uav_vision/` are **written and already validated offline** against a synthetic
ground-truth simulator. 45 unit tests pass. Your job is integration on real Gazebo/PX4,
not green-field authoring. Do not rewrite the core; extend and tune it.

---

## 0. Read this first — corrections to `SYSTEM_ENVIRONMENT.md`

`SYSTEM_ENVIRONMENT.md` is a good map but has errors that will cost you hours. Each of the
following was verified against PX4-Autopilot v1.16.0 source or measured locally.

| Claim in the doc | Reality | Consequence if believed |
|---|---|---|
| `EKF2_EV_POS_Y = 0.03`, `EKF2_EV_POS_Z = 0.242` | Should be **`-0.03`** and **`-0.242`**. The SDF pose is FLU; PX4 wants **FRD**, so Y and Z flip sign. | A 0.5 m lever-arm error injected into EKF2. |
| `COM_RCL_EXCEPT = 7` | Works, but the bit that matters is **bit 2 = OFFBOARD**, i.e. `4`. | Harmless, but understand why. |
| `EKF2_EV_CTRL = 11` (`pos + yaw`) | `11` **excludes velocity** (bit 2 = 4). The PS explicitly requires velocity. Use **`7`** (pos + velocity, magnetometer keeps heading) or `15` with a real quaternion. | Velocity never fused; you silently fail a scored requirement. |
| RGB HFOV `1.57 rad` | The stock OakD-Lite `IMX214` is **`1.204 rad`**. Your branch may differ. | Wrong `fx` → wrong metric scale. **Read `camera_info` at runtime; never hard-code.** |
| Airframe 4022 / `x500_depth_down` is standard | **Not upstream.** No 4022 and no `x500_depth_down` in v1.16.0 or `main`. It is local to your branch. Also `Tools/simulation/gz` is a **git submodule** (`PX4-gazebo-models`), which is why those paths 404 on GitHub. | You cannot look these up online — inspect the local files. |
| — (not mentioned) | **`COM_ARM_EKF_HGT/POS/VEL/YAW` do not exist in v1.16.** The pre-flight innovation gate is a hard-coded constant `kMinTestRatioPreflight = 0.5f` in `EKF2.cpp`. | You will waste time setting non-existent parameters. |
| — (not mentioned) | **`estimator_status` is NOT exposed over uXRCE-DDS by default** — only `estimator_status_flags` is. The innovation test ratios the PS asks you to log live in `estimator_status`. | Missing Phase-2 evidence unless you rebuild (Step 3.3). |
| — (not mentioned) | **Every stock Gazebo world has an untextured ground plane.** Measured: **0 corners** detected vs **500** with texture. | **The single biggest blocker.** VO cannot work at all. See Step 1. |

---

## 1. Architecture and the reasoning behind it

```
 /uav/rgb  ─┐
 /uav/depth ┼─► vo_node ──► /uav_visual_odometry/odom (ENU) ─► px4_odometry_bridge
 camera_info┘      ▲              /quality  /health   ─┐            │ NED/FRD
                   │                                   │            ▼
 /fmu/out/vehicle_attitude (attitude)                   │   /fmu/in/vehicle_visual_odometry
 /fmu/out/sensor_combined  (gyro)                       │            │
                                                        ▼            ▼
                                            vision_health_node ◄── EKF2
                                                        │      (estimator_status_flags,
                                                        ▼       vehicle_local_position)
                                                  /ready_to_arm
                                                        │
                                                        ▼
                                            offboard_mission_node ─► trajectory_setpoint
```

### Why a custom OpenCV node rather than RTAB-Map / VINS / ORB-SLAM3

- The camera stares at a **plane**. Essential-matrix (5-point) VO is **degenerate** on planar
  scenes; a plane-induced **homography** is exactly the right model. Most general VIO stacks
  are built for forward-facing, parallax-rich flight.
- We have **metric depth**, so there is no scale ambiguity to solve — the hard part those
  stacks exist for is already free.
- The mission is a 90 s stationary hover. Loop closure, mapping and place recognition are
  pure overhead against a 60 ms budget.
- The PS scores **confidence metrics, graceful degradation and EKF2 reset semantics**. Those
  need direct control of the `VehicleOdometry` fields — off-the-shelf packages do not expose them.
- Learned VO (DROID-SLAM, DPVO, RAFT, SuperPoint+LightGlue) is out: all are ≥ 40–100 ms on
  desktop-class GPUs *before* sharing an RTX 4060 Laptop with Gazebo's own rendering, and the
  monocular ones waste the depth channel we already have.

**Optional cross-check (cheap, good for the report):** `ros-humble-rtabmap-odom` is
apt-installable. Running `rgbd_odometry` alongside as a second opinion is a nice validation
figure, but do not put it in the control loop.

### The three estimates and why position does not drift

| Quantity | Source | Drift |
|---|---|---|
| **Height (z)** | Median depth over a central window × cos(tilt) | **None** — an absolute measurement. Z-depth from a level nadir camera *is* the AGL height. |
| **Velocity** | Sparse LK flow, gyro-de-rotated, scaled by depth | Bounded, noisy — never integrated for position. |
| **Position (x, y)** | **Homography registration against a keyframe**, not frame-to-frame integration | **None while the keyframe stays in view.** |

The keyframe anchor is the key design choice. Dead-reckoned velocity integrated over 90 s
drifts without bound and would fail the 1.5 m gate. Registering each frame against a keyframe
captured at hover start makes position *absolute*. It also gives **free recovery after vision
loss** — the pre-dropout keyframe is still valid when the image comes back (measured below).

Rotation comes from the **flight controller's attitude**, not from the homography. Projecting
`M = K⁻¹HK` onto SO(3) to extract R absorbs part of the translation term and **halves** it.
This is pinned by `test_homography_translation_is_exact_and_scale_invariant`.

### Measured performance (offline, exact ground truth)

Reproduce with `cd tools && PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full`

| Scenario | Max horiz error | Velocity RMSE | p95 latency | Verdict |
|---|---|---|---|---|
| **Nominal 90 s hover @10 m** | **0.234 m** | 0.050 m/s | **5.9 ms** | PASS ×6.4 vs the 1.5 m gate; PASS ×10 vs the 60 ms gate |
| Lighting ±45 % | 0.390 m | 0.051 | 6.7 ms | PASS (0.97 m without CLAHE — CLAHE is load-bearing) |
| Severe motion blur (σ 4.5–8 px) | 0.307 m | 0.065 | 8.8 ms | PASS |
| Near-featureless ground (5 % contrast) | 0.234 m | 0.051 | 6.8 ms | PASS |
| **5 s total vision loss** | recovers to 0.076 m (0.074 m before) | — | — | **Recovered with no jump**; health reported LOST, quality 0 throughout |

Attitude sensitivity — the dominant error term, ≈ `altitude × tan(attitude error)`:

| Attitude error | 0.1° | 0.3° | 0.6° | 1.0° | 2.0° |
|---|---|---|---|---|---|
| Max horiz error @10 m | 0.078 m | 0.234 m | 0.467 m | 0.777 m | **1.545 m (FAILS)** |

Attitude **bias** is nearly harmless (2° bias → 0.173 m) because it is common-mode between the
keyframe and the current frame and cancels in the relative rotation. Attitude **noise** does not
cancel. **Practical rule: keep EKF2 attitude noise under ~1° RMS.** If the hover gate is marginal,
tune the IMU/attitude side before touching the vision side.

---

## 2. Stage 0 — verify the environment before changing anything

```bash
ls ~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth_down/model.sdf   # must exist
cat  ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_down
ls ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

**Record these from the local `model.sdf` — do not assume:**
1. Sensor `<name>` tags and any `<topic>` overrides (`depth_camera` is usually literal and unscoped).
2. `<image><width>/<height>` and `<horizontal_fov>` for **both** sensors.
3. The camera link `<pose>` — its **RPY** feeds the `camera_link_rpy` parameter.
4. Whether the RGB and depth sensors share the same `<pose>`. `depth_utils.depth_at_rgb_pixels`
   assumes an identity extrinsic between them. If they differ, that function needs a full
   reprojection — flag it before flying.

**Deriving the camera→body rotation.** `frames.camera_rotation_in_body(link_rpy)` does this for
you. For the nadir mount (`0 1.5707 0`) it yields:

```
image-right → body RIGHT      image-down → body BACKWARD      optical axis → DOWN
```

That is a **90° roll about the optical axis** away from the "image-right = body-forward"
arrangement one would naively assume. Getting it wrong rotates the entire velocity estimate by
90°, producing plausible-looking but completely wrong odometry. Pinned by
`test_camera_rotation_matches_sdf_pose`.

---

## 3. Stage 1 — give the ground a texture (**do this first; nothing works without it**)

Measured on the stock grey `ground_plane`: `goodFeaturesToTrack` returns **0 corners**. With
texture: **500**. There is no tuning that rescues an untextured plane.

```bash
cd ~/aerial_interiit_prep/tools
python3 make_ground_texture.py --out ground_albedo.png --px 4096 --tiles 24

M=~/PX4-Autopilot/Tools/simulation/gz/models/textured_ground
mkdir -p $M/materials/textures
mv ground_albedo.png $M/materials/textures/
# wrap tools/textured_ground.sdf.snippet in <sdf version='1.9'>...</sdf> -> $M/model.sdf
# write a minimal $M/model.config alongside it
```

Then copy `worlds/default.sdf` to `worlds/vo_ground.sdf`, delete the `ground_plane` **visual**
(keep a collision plane) and add `<include><uri>model://textured_ground</uri></include>`.
Launch with `PX4_GZ_WORLD=vo_ground`.

Notes that cost real debugging time:
- `<albedo_map>` **must** use a `model://` URI; relative paths do not resolve (gz-sim #286).
- Use a thin `<box>`, **not** a `<plane>`. SDF has **no UV-tiling parameter** (confirmed: 
  `plane_shape.sdf` has only `<normal>`/`<size>`, and the `<pbr>` block has no repeat/scale),
  so one image would stretch across the whole surface. Tiling is baked into the PNG instead.
- **Texture GSD must be finer than camera GSD.** At 10 m with `fx ≈ 432` the camera samples
  2.3 cm/px; a 120 m plane therefore needs ≥ 4096 px (2.9 cm/texel). Coarser and features blur out.
- PX4's build already puts that models dir on `GZ_SIM_RESOURCE_PATH`.

**Alternative:** the stock `baylands` world has real photogrammetric texture (pulled from Fuel on
first run). Good as a second demo environment; the custom plane is more controllable.

**Gate:** fly manually to ~10 m, `ros2 run image_view image_view --ros-args -r image:=/uav/rgb`,
and confirm visible ground detail.

---

## 4. Stage 2 — PX4 parameters

```bash
# in the pxh> shell
param load /home/jashan/aerial_interiit_prep/tools/px4_gps_denied.params
param save
reboot          # EKF2_HGT_REF, EKF2_EV_DELAY and SYS_HAS_GPS are reboot-required
```

The file is annotated; the essentials:

| Parameter | Value | Why |
|---|---|---|
| `SYS_HAS_GPS` | `0` | **Reboot.** Sensor hub stops processing `sensor_gps`. |
| `EKF2_GPS_CTRL` | `0` | No GPS fusion at all (default is `7`). |
| `EKF2_HGT_REF` | `3` (Vision) | **Reboot.** Default is `1` (GPS); leaving it starves the height reference and looks exactly like "EKF2 is ignoring my vision". |
| `EKF2_EV_CTRL` | `7` | bit0 horiz pos + bit1 vert pos + bit2 **3D velocity**. |
| `EKF2_EV_DELAY` | `50.0` **ms** | **Reboot.** Set to your *measured* pipeline latency (Step 6.2). |
| `EKF2_EV_POS_X/Y/Z` | `0.12 / -0.03 / -0.242` | FRD, re-derive from your own SDF. |
| `COM_RCL_EXCEPT` | `4` | bit 2 exempts OFFBOARD from RC-loss failsafe. |
| `COM_POS_FS_EPH` / `COM_VEL_FS_EVH` | leave at `5.0` / `1.0` | The PS measures you against these — **do not relax them.** |

**GPS in Gazebo cannot be silenced at the source.** `GZBridge.cpp` subscribes to the model's
navsat sensor unconditionally, and there is no `PX4_SIM_GPS_DENIED` env var in v1.16. Disabling
*consumption* (`SYS_HAS_GPS=0` + `EKF2_GPS_CTRL=0`) is the correct and complete answer; say so
explicitly in the report, and evidence it with `cs_gps_hgt == false` in `estimator_status_flags`.

### 3.3 Expose `estimator_status` (needed for Phase-2 innovation evidence)

Add to `src/modules/uxrce_dds_client/dds_topics.yaml` under `publications:`

```yaml
  - topic: /fmu/out/estimator_status
    type: px4_msgs::msg::EstimatorStatus
```

then rebuild (`make px4_sitl`). CMake regenerates `dds_topics.h` from the YAML automatically —
no `make clean`. This gives you `pos_test_ratio`, `vel_test_ratio`, `hgt_test_ratio`, which the
PS asks you to keep below the failsafe thresholds. `vision_health_node` picks the topic up
automatically when present and warns when it is not.

---

## 5. Stage 3 — build and validate offline (no Gazebo needed)

```bash
cd ~/aerial_interiit_prep
colcon build --symlink-install --packages-select uav_vision
source install/setup.bash
python3 -m pytest src/uav_vision/test -q                       # expect 45 passed
cd tools && PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full
```

Run this **after every change to `vo_core.py`**. It is the cheapest possible regression signal
and it is where both real bugs were caught during development.

---

## 6. Stage 4 — bring up and verify each link

```bash
cd ~/aerial_interiit_prep/tools
./run_sim.sh --no-mission     # perception only, no arming
./check_stack.sh              # in another terminal
```

### 6.1 QoS — the #1 "no data" cause
PX4 publishes `/fmu/out/*` **BEST_EFFORT**. A RELIABLE subscriber never matches and the callback
silently never fires. Durability also varies between releases. Check once:

```bash
ros2 topic info /fmu/out/vehicle_attitude --verbose
```

and set the `px4_durability` parameter (`volatile` | `transient_local`) in `config/vo_params.yaml`
to match. Everything else in `qos.py` is already correct.

### 6.2 Measure the pipeline latency, then set `EKF2_EV_DELAY`
`latency_ms` in `/diagnostics` is the VO compute time only. End-to-end also includes Gazebo
render + bridge + DDS. Measure the total (camera stamp → EV publish) and put **that** number in
`EKF2_EV_DELAY` (ms, reboot-required). Expect 30–60 ms.

### 6.3 Expected healthy state
- `/uav_visual_odometry/odom` at ~30 Hz; `quality` ≥ 60; `health` = `OK`
- `/fmu/in/vehicle_visual_odometry` at 30 Hz
- `estimator_status_flags`: `cs_ev_pos` **true**, `cs_ev_vel` **true**, `cs_gps_hgt` **false**
- `vehicle_local_position`: `xy_valid`/`z_valid` true, `dead_reckoning` **false**, `eph` well under 5 m

---

## 7. Stage 5 — Phase 1: normal arming (20 % of Part 1)

PS requirements and where each is satisfied:

| Requirement | Implementation | Evidence to capture |
|---|---|---|
| Publish EV incl. velocity → `vehicle_visual_odometry` | `px4_odometry_bridge` | `ros2 topic hz /fmu/in/vehicle_visual_odometry` = 30 Hz |
| **Normal arming, no force** | `offboard_mission_node` waits on `/uav_vision_health/ready_to_arm`; arms with `VEHICLE_CMD_COMPONENT_ARM_DISARM param1=1.0` and **no** force flag (`param2=21196`) | PX4 console shows no "Force arming"; log the arm accept |
| Stable EKF2 EV fusion | `EKF2_EV_CTRL=7`, `EKF2_HGT_REF=3` | `cs_ev_pos`/`cs_ev_vel` true |
| Confidence metrics from feature count / tracking stability | `vo_core` quality 0–100 → `VehicleOdometry.quality` **and** variances | plot quality vs feature count |
| GPS disabled + EV params | `tools/px4_gps_denied.params` | `param show EKF2_GPS_CTRL` etc. |
| **20–30 Hz, correct FRD/NED** | bridge at 30 Hz; `frames.py`, 45 tests | `ros2 topic hz`; unit tests |

**On "MAVLink-based health monitoring":** this stack uses the modern **uXRCE-DDS** path
(`/fmu/in/vehicle_visual_odometry`), which is the direct successor to MAVLink
`VISION_POSITION_ESTIMATE`/`ODOMETRY` and is what the PS's own topic name refers to. State this
explicitly in the README. If a literal MAVLink demonstration is wanted, add a small MAVSDK
sender as a secondary artefact — do not migrate the control path.

---

## 8. Stage 6 — Phase 2: 90 s vision-only position hold (25 % of Part 1)

```bash
./run_sim.sh          # full mission
```

`offboard_mission_node` runs `WAIT_HEALTH → ARM → TAKEOFF(10 m) → HOLD(90 s) → LAND` and prints
the max hold radius and a PASS/FAIL against 1.5 m at the end.

| Requirement | How it is met |
|---|---|
| Hover inside 1.5 m for 90 s | Keyframe anchoring (offline: **0.234 m** max) |
| Handle EKF2 failsafe gracefully + auto-recover | `DEGRADED` state → zero-velocity setpoint (drops the position requirement OFFBOARD cannot satisfy) → returns to position hold when vision recovers. Never disarms, never fights PX4. |
| Log synchronised vision/control data incl. innovations | `run_sim.sh` records a rosbag of VO, health, EV out, EKF2 state |
| Scale-consistent velocity from known altitude | Depth-scaled flow; height is absolute, not integrated |
| Innovations below failsafe thresholds | `pos/vel/hgt_test_ratio` from `estimator_status` |

**If the hover gate is marginal:** attitude noise is almost certainly the cause (see the
sensitivity table). Check EKF2 attitude quality first. Secondary knobs: raise `min_features`,
lower `rekey_shift` (re-key less often — each re-key injects a small error), increase
`clahe_clip` under changing light.

---

## 9. Stage 7 — degradation demos (directly scored: "lighting, motion blur, tracking failures")

Each has an offline counterpart already measured, so you can show sim-vs-model agreement.

1. **Lighting** — animate a `<light>` in the world, or change camera exposure mid-flight.
2. **Motion blur** — command faster translations; the LK window handles it (measured to σ 8 px).
3. **Feature loss** — fly over a patch where you removed the texture, or publish black frames.
   Expect: `health` → `DEGRADED`/`LOST`, `quality` → 0, mission → `DEGRADED`, then clean recovery.
4. **Total dropout** — stop the bridge for 5 s, restart. Expect recovery with **no position jump**
   (the keyframe survives) and exactly one `reset_counter` increment.

---

## 10. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No callbacks on `/fmu/out/*` | QoS mismatch | Match `px4_durability` to `ros2 topic info -v` |
| `goodFeaturesToTrack` returns 0 | Untextured ground | Stage 1 |
| EV published but `cs_ev_pos` false | Rate < 5 Hz, or `EKF2_EV_CTRL=0`, or `pose_frame`/`velocity_frame` left 0 | Bridge sets frames explicitly; check `EKF2_EV_CTRL` |
| EV accepted then ignored after ~1 min | Height reference starved (`EKF2_HGT_REF` still `1`/GPS) | Set `3`, **reboot** |
| "Preflight Fail: height estimate not stable" | `hgt_test_ratio > 0.5` | Let EV settle before arming; check `EKF2_EV_DELAY` |
| "Preflight Fail: horizontal position unstable" | `pos_test_ratio > 0.5` | Usually a frame/sign error — re-run the unit tests |
| Arms, then drifts away smoothly | VO drifting and EKF2 trusting it (no ground truth to contradict it) | Confirm `mode == keyframe` in `/diagnostics`, not `dead_reckoning` |
| Position estimate rotated 90° | `camera_link_rpy` wrong | Stage 0 derivation |
| Velocity sign inverted | Camera→body rotation | `test_camera_rotation_matches_sdf_pose` |
| OFFBOARD rejected | Setpoints not streaming before the mode request, or local position invalid | Node sends >20 setpoints first; check `COM_RCL_EXCEPT=4` |
| Quaternion silently rejected | Not normalised to 1e-5, or zeros instead of NaN | Bridge normalises; sends NaN when `fuse_yaw=false` |
| Low RTF | 1080p RGB render | Drop the RGB `<image>` to 640×480 in the SDF |

---

## 11. Acceptance checklist

**Phase 1**
- [ ] `/fmu/in/vehicle_visual_odometry` at 20–30 Hz with finite position **and velocity**
- [ ] Armed **without** force; PX4 console clean
- [ ] `cs_ev_pos` && `cs_ev_vel` true, `cs_gps_hgt` false
- [ ] `quality` tracks feature count; variances grow when degraded
- [ ] `EKF2_GPS_CTRL=0`, `SYS_HAS_GPS=0`, `EKF2_EV_CTRL=7`, `EKF2_HGT_REF=3` captured
- [ ] 45 unit tests pass

**Phase 2**
- [ ] 90 s hover, max radius < 1.5 m, plotted
- [ ] `pos/vel/hgt_test_ratio` plotted below threshold throughout
- [ ] Induced degradation → graceful `DEGRADED` → automatic recovery, no disarm
- [ ] Vision pipeline latency < 60 ms, measured
- [ ] rosbag + plots + video

**Deliverable.** This repository IS the workspace: cloning it into `~/aerial_interiit_prep`
gives you a directory you can `colcon build` straight away. The PS submission format,
however, asks for a zip containing `PART_1/` and `PART_2/` folders -- so at submission time,
copy this tree into a `PART_1/` directory alongside `PART_2/`, keeping the README, this plan,
logs, flowcharts and the video with it.

---

## 12. Honest limitations to state in the report

1. **Flat-ground assumption.** The homography model requires a locally planar scene. Over
   significant relief the plane model breaks; the keyframe inlier count is the detector, and the
   fallback is dead-reckoned velocity.
2. **Attitude is taken from EKF2, which is IMU-driven.** This is standard VIO practice, but it
   means the system is vision-*aided*, not vision-*only*, in the strictest sense. Position and
   velocity are genuinely vision-derived; heading comes from the magnetometer unless `fuse_yaw`
   is enabled. Say so plainly.
3. **Simulated depth is near-perfect.** A real OakD-Lite at 10 m has far worse stereo depth
   (quadratic range error, dropouts on low-texture ground). Height would need a rangefinder or
   the plane-fit path on real hardware.
4. **Keyframe anchoring holds position, not a map.** Translate far enough and keyframes chain,
   which reintroduces slow drift. Fine for station-keeping; a real mission needs loop closure.
5. **Offline numbers are a lower bound on error**, not a prediction. Gazebo adds render latency,
   DDS jitter and controller coupling that the harness does not model.
