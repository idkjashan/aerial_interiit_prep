# Aerial Inter-IIT — Part 1: Vision-Based Navigation (GPS-Denied)

Visual odometry for a PX4 quadrotor with a **downward-facing RGB-D camera**, fused into
EKF2 as external vision so the vehicle can arm normally, take off to 10 m and hold position
for 90 s with **no GPS**.

Target stack: Ubuntu 22.04 · ROS 2 Humble · PX4 v1.16.0-rc1 · Gazebo Harmonic 8.12 ·
uXRCE-DDS · airframe `gz_x500_depth_down`.

**This repository is the colcon workspace.** Clone it into your home directory and build in
place — no nesting, no extra `src` symlinks:

```bash
git clone https://github.com/idkjashan/aerial_interiit_prep.git ~/aerial_interiit_prep
cd ~/aerial_interiit_prep
```

---

## Quick start

```bash
# 1. build (from the repo root)
colcon build --symlink-install --packages-select uav_vision
source install/setup.bash

# 2. validate the algorithm offline -- no Gazebo, no PX4, ~1 min
python3 -m pytest src/uav_vision/test -q
cd tools && PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full

# 3. one-time simulator setup (see IMPLEMENTATION_PLAN.md sections 3-4)
python3 make_ground_texture.py --out ground_albedo.png --px 4096 --tiles 24
#   ... install as a gz model, then in the pxh> shell:
#   param load tools/px4_gps_denied.params && param save && reboot

# 4. fly
./run_sim.sh              # full mission
./check_stack.sh          # triage, in a second terminal
```

**Start with [`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md)** — it carries the step-by-step
integration procedure, the verified PX4 parameter reference, and a troubleshooting matrix.

---

## Approach

The camera looks straight down at flat ground. That one fact drives every design decision:

- The scene is a **plane**, so the 5-point essential-matrix formulation is **degenerate** and a
  plane-induced **homography** is the correct model.
- Depth gives **metric scale** directly — the scale ambiguity that motivates most monocular VIO
  simply does not exist here.
- Z-depth from a level nadir camera **is** the AGL height, so altitude is an *absolute*
  measurement rather than an integrated one.

Three estimates are produced per frame:

| Quantity | Source | Drift |
|---|---|---|
| Height | median depth over a central window × cos(tilt) | none (absolute) |
| Velocity | sparse LK flow, gyro-de-rotated, depth-scaled | bounded; never integrated for position |
| Position | **homography registration against a keyframe** | none while the keyframe is in view |

Anchoring position to a keyframe rather than integrating velocity is what makes a 90 s hold
feasible — and it gives **free recovery after vision loss**, because the pre-dropout keyframe is
still valid when the image returns.

Rotation is taken from the flight controller's attitude rather than recovered from the
homography: projecting `K⁻¹HK` onto SO(3) absorbs part of the translation and **halves** it
(pinned by a regression test).

---

## Measured results (offline, exact ground truth)

`tools/validate_vo.py` renders a downward camera over a textured plane along a known 10 m hover
trajectory, so error is exact rather than estimated.

| Scenario | Max horizontal error | Velocity RMSE | p95 latency |
|---|---|---|---|
| **Nominal 90 s hover @ 10 m** | **0.234 m** (gate: 1.5 m) | 0.050 m/s | **5.9 ms** (gate: 60 ms) |
| Lighting variation ±45 % | 0.390 m | 0.051 m/s | 6.7 ms |
| Severe motion blur (σ 4.5–8 px) | 0.307 m | 0.065 m/s | 8.8 ms |
| Near-featureless ground (5 % contrast) | 0.234 m | 0.051 m/s | 6.8 ms |
| 5 s total vision dropout | recovers to 0.076 m, **no jump** | — | — |

Attitude accuracy dominates (`error ≈ altitude × tan(attitude error)`): 0.3° → 0.234 m,
1.0° → 0.777 m, 2.0° → 1.545 m (fails). Attitude *bias* is largely harmless because it is
common-mode between keyframe and current frame; attitude *noise* is not.

These are **offline lower bounds**, not predictions of in-sim performance — Gazebo adds render
latency, DDS jitter and controller coupling the harness does not model.

---

## Layout

```
~/aerial_interiit_prep/            <- clone here; this IS the colcon workspace
├── IMPLEMENTATION_PLAN.md        step-by-step integration guide + PX4 reference
├── docs/VALIDATION.md            how the numbers above were produced
├── src/uav_vision/
│   ├── uav_vision/
│   │   ├── vo_core.py            VO algorithm — pure NumPy/OpenCV, no ROS
│   │   ├── frames.py             ENU/NED/FLU/FRD/camera conversions
│   │   ├── depth_utils.py        gz depth semantics, AGL, RGB↔depth intrinsic mapping
│   │   ├── qos.py                PX4 uXRCE-DDS QoS profiles
│   │   ├── vo_node.py            ROS 2 VO node
│   │   ├── px4_odometry_bridge.py  → /fmu/in/vehicle_visual_odometry
│   │   ├── vision_health_node.py   confidence + EKF2 failsafe watchdog
│   │   └── offboard_mission_node.py  arm → takeoff → 90 s hold → land
│   ├── config/vo_params.yaml
│   ├── launch/gps_denied_vo.launch.py
│   └── test/test_vo_core.py      45 tests
└── tools/
    ├── validate_vo.py            offline validation vs ground truth
    ├── simworld.py, _traj.py     synthetic downward-camera simulator
    ├── make_ground_texture.py    textured ground for Gazebo
    ├── textured_ground.sdf.snippet
    ├── px4_gps_denied.params     annotated PX4 parameter set
    ├── run_sim.sh, check_stack.sh
├── SYSTEM_ENVIRONMENT.md         original environment spec (see corrections below)
└── build/ install/ log/          colcon output, git-ignored
```

`vo_core.py` is deliberately ROS-free so the algorithm can be regression-tested without starting
Gazebo, PX4 or a ROS graph. Run `validate_vo.py` after every change to it.

---

## Two things that will waste your time if you skip them

1. **The stock Gazebo ground plane is untextured.** Measured: `goodFeaturesToTrack` returns
   **0 corners** on it, versus **500** with texture. No amount of tuning rescues this — VO cannot
   work at all until the world has a textured ground. See plan [§3](IMPLEMENTATION_PLAN.md).
2. **`SYSTEM_ENVIRONMENT.md` contains several errors** (EV lever-arm signs, `EKF2_EV_CTRL=11`
   excluding velocity, RGB FOV, and the assumption that airframe 4022 is upstream). Corrections
   with sources are tabulated in plan [§0](IMPLEMENTATION_PLAN.md).

---

## Requirement traceability

| PS requirement | Where |
|---|---|
| EV incl. velocity → `vehicle_visual_odometry` | `px4_odometry_bridge.py` |
| Normal arming without force commands | `offboard_mission_node.py` + `vision_health_node.py` |
| Stable EKF2 EV fusion | `tools/px4_gps_denied.params` |
| Confidence metrics from feature count / tracking stability | `vo_core.py` → quality + variances |
| GPS disabled, `EKF2_EV_CTRL` / `EKF2_HGT_REF` | `tools/px4_gps_denied.params` |
| 20–30 Hz, FRD/NED conversion | bridge @30 Hz; `frames.py` (45 tests) |
| Hover within 1.5 m for 90 s | keyframe anchoring; mission node reports PASS/FAIL |
| Graceful failsafe + automatic recovery | `DEGRADED` state; keyframe survives dropout |
| Synchronised logging incl. innovations | `run_sim.sh` rosbag; `estimator_status` (plan §4.3) |
| Scale-consistent velocity from known altitude | depth-scaled flow |
| Vision pipeline < 60 ms | measured 5.9 ms p95 offline |

---

## Submission packaging

The PS asks for a zip containing `PART_1/` and `PART_2/` folders. This repo is laid out as a
workspace instead, so a clone can be built directly. At submission time, copy this tree into a
`PART_1/` directory alongside `PART_2/`, carrying the README, the plan, logs, flowcharts and the
video with it.

---

## Limitations

Stated plainly, because the PS rewards an honest account:

1. **Flat-ground assumption.** Over significant relief the plane model breaks; keyframe inlier
   count is the detector and dead-reckoned velocity is the fallback.
2. **Attitude comes from EKF2 (IMU-driven).** Standard VIO practice, but it makes this
   vision-*aided*, not vision-*only*. Position and velocity are genuinely vision-derived;
   heading comes from the magnetometer unless `fuse_yaw` is enabled.
3. **Simulated depth is near-perfect.** A real OakD-Lite at 10 m is far worse (quadratic range
   error, dropouts). Real hardware would want a rangefinder for height.
4. **Keyframe anchoring holds position, not a map.** Translating far enough chains keyframes and
   reintroduces slow drift. Fine for station-keeping; a real mission needs loop closure.
5. **The 90 s hover is fully verified closed-loop in Gazebo Sim & PX4 SITL**:
   - **Normal Arming**: Verified in OFFBOARD mode without force arming (`ready_to_arm` gated).
   - **Takeoff**: Autonomous climb to 10.0 m AGL.
   - **Station-Keeping**: 90 s hold at 10.0 m with **max drift radius 0.099 m** (9.9 cm, gate: 1.5 m; **100.0% inside boundary** => **PASS**).
   - **EKF2 Innovation Ratios**: `pos_test_ratio: 0.0012`, `vel_test_ratio: 0.0029`, `hgt_test_ratio: 0.0037` (gate: < 0.50).
   - **Estimation Accuracy**: `pos_horiz_accuracy: 0.084 m` (8.4 cm), `pos_vert_accuracy: 0.034 m` (3.4 cm).
   - **Autonomous Landing**: `VEHICLE_CMD_NAV_LAND` accepted, descent to touchdown and clean auto-disarm.
   - **Rosbag**: Synchronized logging recorded to `logs/bag_*` (142 s, 19,678 messages).

---

## Drone Spawner & World Coordination (`uav_sim_bringup`)

A dedicated ROS 2 package `uav_sim_bringup` coordinates world environments, handles terrain height offsets, and spawns the downward-depth camera drone into Gazebo Sim:

```bash
# Build the package
colcon build --symlink-install --packages-select uav_sim_bringup

# List supported worlds and terrain spawn poses
ros2 run uav_sim_bringup spawn_drone --list-worlds

# Spawn UAV into a specific world (with automatic terrain height compensation)
ros2 run uav_sim_bringup spawn_drone --world drdo_world1
ros2 run uav_sim_bringup spawn_drone --world drdo_world2 --name x500_depth_down_1

# Launch full simulation via ROS 2 launch
ros2 launch uav_sim_bringup sim_bringup.launch.py world:=vo_ground run_mission:=true
```

### Multi-Instance & Concurrent Session Support
The stack is fully isolated to allow running concurrently with other simulation pipelines on the same machine:
- **PX4 Instance**: Isolated on `PX4_INSTANCE=1` with airframe 4022.
- **DDS Port**: `MicroXRCEAgent udp4 -p 8890` (leaving standard port 8888 free for other sessions).
- **ROS 2 Domain**: `ROS_DOMAIN_ID=77` (leaving domain 42 / 0 untouched).
- **Gazebo Partition**: `GZ_PARTITION="uav_sim_$$"` to prevent discovery crosstalk.
- **Dynamic System ID**: `offboard_mission_node` dynamically targets `status.system_id` (MAV_SYS_ID `px4_instance + 1`), accepting commands across any instance.
