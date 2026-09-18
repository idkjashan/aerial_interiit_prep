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

Logs and plots are in `logs/`, the screen recording in `video/`.

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
│   ├── uav_vision/            VO core, the four nodes, launch file, params, RViz config, tests
│   └── uav_sim_bringup/       optional: one launch file for gz + PX4 + agent + bridge + stack
├── px4/                       files added to PX4-Autopilot, at their PX4 paths
│   ├── ROMFS/.../4022_gz_x500_depth_down      airframe (GPS-denied defaults)
│   ├── Tools/simulation/gz/models/            x500_depth_down, textured_ground
│   ├── Tools/simulation/gz/worlds/vo_ground.sdf
│   ├── src/modules/uxrce_dds_client/dds_topics.yaml   adds estimator_status
│   ├── gps_denied.params      the same parameters, annotated
│   └── textured_ground.sdf.snippet
├── tools/
│   ├── run_sim.sh             automatic run: start everything, fly the 90 s hover, record a rosbag
│   ├── launch_sim.sh          manual mode: start everything, vehicle waits on the ground
│   ├── manual_control.py      arm / takeoff / goto / land, and keyboard flying
│   ├── monitor.py             camera view with VO health and flight state
│   ├── vision_cut.sh          pause / resume the camera stream (vision loss demo)
│   ├── teleop_bridge.py       /cmd_vel -> PX4 velocity setpoints, if manual_control isn't running
│   ├── view_camera.py, view_rviz.sh   lighter camera view, RViz
│   ├── plot_bag.py            hold distance, innovation ratios and VO health from a rosbag
│   ├── check_stack.sh         is every link alive?
│   ├── validate_vo.py         offline check against exact ground truth
│   ├── simworld.py, _traj.py  synthetic downward camera for validate_vo.py
│   └── make_ground_texture.py
├── docs/
│   ├── setup.md               step-by-step setup, PX4 parameters, troubleshooting
│   └── validation.md          how the offline numbers were produced
├── logs/                      rosbag and plots
└── video/part1_demo.webm
```

## Running it

Needs ROS 2 Humble, `px4_msgs` (release/1.16) built in `~/px4_ros_ws`,
Micro-XRCE-DDS-Agent and PX4-Autopilot built for SITL in `~/PX4-Autopilot` (or set
`PX4_DIR`). Copy the files under `px4/` into the PX4 tree first (they are at the same
paths), then `make px4_sitl` so PX4 picks up the new airframe.

```bash
cd PART_1
source ~/px4_ros_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash

# offline checks, no simulator needed
python3 -m pytest src/uav_vision/test -q
cd tools && PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full && cd ..
```

Both launch scripts use PX4 instance 1, DDS port 8890, `ROS_DOMAIN_ID=77` and their own
Gazebo partition, so they can run next to another simulation.

### Automatic run (the 90 s hover)

```bash
./tools/run_sim.sh               # arm -> 10 m -> 90 s hold -> land, rosbag in logs/
./tools/check_stack.sh           # second terminal: topic rates, EKF2 flags, health
python3 tools/plot_bag.py        # plots from the latest bag, into logs/
```

The mission node prints the maximum distance from the hold point and PASS/FAIL against
1.5 m when the hold ends.

### Manual mode

```bash
./tools/launch_sim.sh            # Gazebo window; --headless to skip it
python3 tools/monitor.py         # second terminal: camera + VO health + position
python3 tools/manual_control.py takeoff 10
python3 tools/manual_control.py goto 2.0 1.0 10.0     # ENU: 2 m east, 1 m north, 10 m up
python3 tools/manual_control.py land
```

`takeoff`, `hover` and `goto` keep streaming setpoints until Ctrl-C, so stop one before
starting the next, or send new goals to the running one on `/goal_pose`.

`manual_control.py` without arguments is interactive: `a` arm, `t` take off to 10 m,
`h` hold, `L` land, `d` disarm, `i` `,` `j` `l` move, `w` `s` up/down, `u` `o` yaw, `k`
stop, space for status. It also takes `geometry_msgs/PoseStamped` goals on `/goal_pose`
and velocity commands on `/cmd_vel`, so `ros2 run teleop_twist_keyboard
teleop_twist_keyboard` works too. RViz: `./tools/view_rviz.sh`.

Take off with `manual_control.py`, not with the QGroundControl takeoff slider. QGC's takeoff
goes through PX4's Takeoff mode, and in this GPS-denied setup we saw the vehicle yaw hard on
liftoff that way. `manual_control.py` climbs in OFFBOARD and keeps the current heading.

### Vision loss and recovery

```bash
./tools/vision_cut.sh test 5     # pause the camera bridge for 5 s, then resume
```

VO health goes LOST within half a second and EKF2 loses its horizontal aiding. The
controller switches to a zero-velocity setpoint. Without a horizontal position PX4 can't
stay in OFFBOARD, and its failsafe drops to Altitude mode: it holds height on the barometer
instead of landing (the airframe sets both the offboard-loss and the position-loss action to
Altitude mode, `COM_OBL_RC_ACT=1`, `COM_POSCTL_NAVL=0`).
When the images come back, the keyframe from before the cut is still valid, so position
comes back without a jump. The controller then asks for OFFBOARD again and the hold resumes.

### Other

With `uav_sim_bringup` built, `ros2 launch uav_sim_bringup sim_bringup.launch.py
world:=vo_ground` starts the whole stack from one launch file.

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
| Failsafe handling and recovery | DEGRADED path in the mission node, `tools/vision_cut.sh`; keyframe survives a dropout |
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
