# Part 1 — Vision-based navigation without GPS

Visual odometry for a PX4 quadrotor with a downward-facing RGB-D camera, fused into EKF2 as
external vision. The vehicle arms normally (no force), takes off to 10 m and holds position
for 90 s with GPS disabled.

Stack: Ubuntu 22.04 · ROS 2 Humble · PX4 v1.16 line SITL · Gazebo Harmonic · uXRCE-DDS ·
airframe `gz_x500_depth_down` (OakD-Lite pointing down).

## Result in Gazebo

| | |
|---|---|
| Arming | normal arm in OFFBOARD, gated on vision health, no force flag |
| Takeoff / hold | 10.0 m, 90 s |
| Max distance from the hold point | **0.099 m** (limit 1.5 m), inside the circle 100 % of the time |
| EKF2 innovation test ratios | pos 0.0012, vel 0.0029, hgt 0.0037 (failsafe/pre-arm gate 0.5) |
| EKF2 accuracy | horizontal 0.084 m, vertical 0.034 m |
| Landing | `NAV_LAND` accepted, touchdown, auto-disarm |
| Log | rosbag of VO, health, EV output and EKF2 state, 142 s |

Logs and plots are in `logs/`.

## How it works

The camera looks straight down at flat ground, and most of the design follows from that:

- The scene is a plane, so the 5-point essential-matrix method is degenerate and a
  plane-induced **homography** is the right model.
- The depth image gives **metric scale** directly. There is no monocular scale problem.
- The optical-axis depth of a level nadir camera **is** the height above ground, so altitude
  is measured, not integrated.

Each frame gives three estimates:

| | from | drift |
|---|---|---|
| height | median depth in a central window × cos(tilt) | none, it's a measurement |
| velocity | sparse LK flow, de-rotated with the gyro, scaled by depth | bounded, never integrated for position |
| position | homography registration against a **keyframe** | none while the keyframe is in view |

Registering against a keyframe instead of integrating velocity is what makes a 90 s hold
possible. It also handles vision loss well: the keyframe from before the dropout is still
valid when the image comes back, so position recovers without a jump.

Rotation comes from the flight controller's attitude. Extracting it from the homography
instead absorbed part of the translation and halved it. A regression test pins this.

The ROS 2 side is four nodes (`src/uav_vision`):

```
camera + attitude ─► vo_node ─► px4_odometry_bridge ─► /fmu/in/vehicle_visual_odometry ─► EKF2
                        │                                                                   │
                        └── quality / health ─► vision_health_node ◄── EKF2 status ─────────┘
                                                        │ ready_to_arm
                                                        ▼
                                              offboard_mission_node ─► setpoints / commands
```

- `vo_node`: runs the VO (`vo_core.py`, no ROS inside) and publishes ENU odometry, a 0–100
  quality from feature count and tracking, and OK / DEGRADED / LOST.
- `px4_odometry_bridge`: ENU/FLU → NED/FRD at 30 Hz, explicit pose and velocity frames,
  variances that grow as quality drops, NaN quaternion so yaw stays with the magnetometer,
  `reset_counter` only on a real re-anchor.
- `vision_health_node`: combines VO quality with what EKF2 reports (`eph/evh`, fusion flags,
  innovation ratios) into `ready_to_arm`, and logs failsafe entry and recovery.
- `offboard_mission_node`: WAIT_HEALTH → ARM → TAKEOFF → HOLD 90 s → LAND. If vision is lost
  it switches to a zero-velocity setpoint and returns to position hold once vision has been
  good for a second. It never disarms or fights a PX4 failsafe.

## Layout

```
PART_1/                        this folder is the colcon workspace
├── src/
│   ├── uav_vision/            VO core, the four nodes, launch file, params, tests
│   └── uav_sim_bringup/       optional: one launch file for gz + PX4 + agent + bridge + stack
├── px4/
│   ├── gps_denied.params      annotated PX4 parameters
│   └── textured_ground.sdf.snippet
├── tools/
│   ├── run_sim.sh             start everything in order and record a rosbag
│   ├── check_stack.sh         is every link alive?
│   ├── validate_vo.py         offline check against exact ground truth
│   ├── simworld.py, _traj.py  synthetic downward camera for validate_vo.py
│   └── make_ground_texture.py
├── docs/
│   ├── setup.md               step-by-step setup, PX4 parameters, troubleshooting
│   └── validation.md          how the offline numbers were produced
└── logs/                      rosbag, plots, video
```

## Running it

```bash
cd PART_1
colcon build --symlink-install
source install/setup.bash

# offline checks, no simulator needed
python3 -m pytest src/uav_vision/test -q
cd tools && PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full

# one-time: textured ground world and PX4 parameters -> docs/setup.md, sections 3 and 4

# fly
./run_sim.sh               # full mission, logs in PART_1/logs/
./check_stack.sh           # second terminal
```

`run_sim.sh` uses PX4 instance 1, DDS port 8890, `ROS_DOMAIN_ID=77` and its own Gazebo
partition so it can run next to another simulation. Paths come from `PX4_DIR` (default
`~/PX4-Autopilot`) and `PX4_WS` (default `~/px4_ros_ws`).

Alternatively, with `uav_sim_bringup` built:

```bash
ros2 launch uav_sim_bringup sim_bringup.launch.py world:=vo_ground
```

## PS requirements

| Requirement | Where |
|---|---|
| EV with velocity → `vehicle_visual_odometry` | `px4_odometry_bridge.py` (uXRCE-DDS, the ROS 2 successor of MAVLink `ODOMETRY`) |
| Normal arming, no force | `vision_health_node.py` gates, `offboard_mission_node.py` arms with no force flag |
| Stable EKF2 EV fusion | `px4/gps_denied.params`, `cs_ev_pos` / `cs_ev_vel` in `estimator_status_flags` |
| Confidence from feature count / tracking | `vo_core.py` quality → message `quality` and variances |
| GPS off, `EKF2_EV_CTRL`, `EKF2_HGT_REF` | `px4/gps_denied.params` |
| 20–30 Hz, FRD/NED | bridge at 30 Hz, `frames.py` (covered by the tests) |
| Hover within 1.5 m for 90 s | keyframe anchoring; the mission node reports PASS/FAIL |
| Failsafe handling and recovery | DEGRADED path in the mission node; keyframe survives a dropout |
| Synchronised logs incl. innovations | rosbag from `run_sim.sh`, `estimator_status` (setup.md §4.3) |
| Scale-consistent velocity from known height | depth-scaled flow, measured height |
| Vision pipeline < 60 ms | 6 ms p95 offline |

## Offline validation

`tools/validate_vo.py` renders the downward camera over a textured plane along a known 10 m
hover path, so the error is exact. These are lower bounds: Gazebo adds render latency, DDS
jitter and the controller in the loop.

| Scenario | Max horizontal error | Velocity RMSE | p95 latency |
|---|---|---|---|
| Nominal 90 s hover at 10 m | 0.234 m | 0.050 m/s | 6.0 ms |
| Lighting ±45 % | 0.477 m | 0.051 m/s | 7.8 ms |
| Motion blur (σ 4.5–8 px) | 0.221 m | 0.065 m/s | 10.1 ms |
| Near-featureless ground (5 % contrast) | 0.234 m | 0.051 m/s | 6.7 ms |
| 5 s total vision dropout | 0.074 m before, 0.076 m after, no jump | – | – |

Attitude noise dominates: error ≈ altitude × tan(attitude error), 0.3° → 0.23 m,
1° → 0.78 m, 2° → 1.55 m. A constant attitude bias mostly cancels between keyframe and
current frame (2° bias → 0.17 m). More in `docs/validation.md`.

## Limitations

1. Flat ground is assumed. Over real relief the homography breaks down; the keyframe inlier
   count detects it and the fallback is integrated velocity.
2. Attitude comes from EKF2, i.e. from the IMU. That is normal for VIO but makes this
   vision-aided rather than vision-only; position and velocity are from vision, heading from
   the magnetometer.
3. Simulated depth is almost perfect. A real OakD-Lite at 10 m is much noisier and would want
   a rangefinder for height.
4. The keyframe holds a position, it doesn't build a map. Flying far chains keyframes and
   slow drift comes back.
