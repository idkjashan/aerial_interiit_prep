# Setup and integration notes

How to go from a fresh clone to the 90 s vision-only hover in Gazebo, plus the things that
cost us time along the way.

Environment we used: Ubuntu 22.04, ROS 2 Humble, PX4-Autopilot (v1.16 line, with a local
`x500_depth_down` airframe, 4022), Gazebo Harmonic 8, Micro-XRCE-DDS-Agent 2.4, `px4_msgs`
from `release/1.16` in `~/px4_ros_ws`.

---

## 0. Traps

| What you might assume | What is actually the case |
|---|---|
| The SDF camera pose `[0.12, 0.03, 0.242]` goes straight into `EKF2_EV_POS_*` | The SDF pose is FLU, PX4 wants FRD: `EKF2_EV_POS_Y = -0.03`, `EKF2_EV_POS_Z = -0.242`. |
| `EKF2_EV_CTRL = 11` fuses vision | 11 is pos + yaw and **leaves out velocity**. Use 7 (pos + vel; magnetometer keeps heading). |
| RGB HFOV is 1.57 rad | The stock OakD-Lite IMX214 is 1.204 rad. Read `camera_info` at runtime; the node only falls back to the SDF FOV if it never arrives. |
| `COM_ARM_EKF_*` controls the pre-arm innovation gate | Those parameters don't exist in v1.16. The gate is a constant, `kMinTestRatioPreflight = 0.5`, in `EKF2.cpp`. |
| `estimator_status` is on the DDS bridge | Only `estimator_status_flags` is by default. The innovation test ratios need step 4.3. |
| Airframe 4022 / `x500_depth_down` is upstream | It isn't; it lives in our PX4 branch. `Tools/simulation/gz` is a submodule (PX4-gazebo-models), so those paths 404 on GitHub. |
| The stock world is fine for VO | Every stock world has an untextured ground plane. `goodFeaturesToTrack` finds **0 corners** on it, 500 with a texture. Nothing works until step 3 is done. |

---

## 1. Architecture

```
 /uav/rgb  ─┐
 /uav/depth ┼─► vo_node ──► /uav_visual_odometry/odom (ENU) ─► px4_odometry_bridge
 camera_info┘      ▲              /quality  /health   ─┐            │ NED/FRD
                   │                                   │            ▼
 /fmu/out/vehicle_attitude                             │   /fmu/in/vehicle_visual_odometry
 /fmu/out/sensor_combined (gyro)                       │            │
                                                       ▼            ▼
                                           vision_health_node ◄── EKF2
                                                       │      (estimator_status_flags,
                                                       ▼       vehicle_local_position)
                                                 /ready_to_arm
                                                       │
                                                       ▼
                                           offboard_mission_node ─► trajectory_setpoint
```

Why a small custom VO node instead of RTAB-Map, VINS or ORB-SLAM3:

- The camera looks at a plane. Essential-matrix VO is degenerate there; a homography is the
  exact model.
- We have metric depth, so there is no scale problem to solve.
- The task is a 90 s hover. Mapping, loop closure and place recognition would only add
  latency.
- The PS scores confidence metrics and EKF2 reset behaviour, which means we need direct
  control over the `VehicleOdometry` fields.
- Learned VO (DROID-SLAM, DPVO, SuperPoint+LightGlue) needs 40–100 ms per frame on a laptop
  GPU that is also rendering Gazebo.

`rtabmap_odom` is apt-installable and makes a good cross-check for the report, but it stays
out of the control loop.

---

## 2. Check the PX4 side first

```bash
ls ~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth_down/model.sdf
cat ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_down
ls ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

From the model SDF, note the sensor names and topics, image sizes and FOVs of both cameras,
and the camera link `<pose>`. The pose RPY is the `camera_link_rpy` parameter. The RGB and
depth sensors must share one pose, because `depth_utils.depth_at_rgb_pixels` assumes an
identity extrinsic between them.

For the nadir mount (`0 1.5707 0`), `frames.camera_rotation_in_body()` gives image-right =
body right and image-down = body backward, a 90° roll about the optical axis from what you
would guess. Get it wrong and the velocity estimate comes out rotated by 90°.
`test_camera_rotation_matches_sdf_pose` pins it.

---

## 3. Give the ground a texture

The finished files are in `PART_1/px4/` (model `textured_ground`, world `vo_ground.sdf`,
airframe and `x500_depth_down`). Copy them into the PX4 tree at the same paths and run
`make px4_sitl`, since PX4 only picks up a new airframe when it is rebuilt
(`tools/launch_sim.sh` copies them if they are missing). The rest of this section is how
they were made.

```bash
cd PART_1/tools
python3 make_ground_texture.py --out ground_albedo.png --px 4096 --tiles 24

M=~/PX4-Autopilot/Tools/simulation/gz/models/textured_ground
mkdir -p $M/materials/textures
mv ground_albedo.png $M/materials/textures/
# wrap PART_1/px4/textured_ground.sdf.snippet in <sdf version='1.9'>...</sdf> -> $M/model.sdf
# and add a minimal model.config
```

Copy `worlds/default.sdf` to `worlds/vo_ground.sdf`, remove the ground plane's visual (keep
a collision plane) and add `<include><uri>model://textured_ground</uri></include>`. Run with
`PX4_GZ_WORLD=vo_ground`.

- `<albedo_map>` needs a `model://` URI; relative paths don't resolve (gz-sim #286).
- Use a thin `<box>`, not a `<plane>`. SDF has no UV tiling option, so the tiling is baked
  into the PNG.
- The texture has to be finer than what the camera resolves: at 10 m with fx ≈ 432 the
  camera sees 2.3 cm per pixel, so a 120 m plane needs at least 4096 px (2.9 cm per texel).

`baylands` has real photogrammetry texture and works as a second environment.

Check: fly to ~10 m manually and look at `/uav/rgb`
(`ros2 run image_view image_view --ros-args -r image:=/uav/rgb`).

---

## 4. PX4 parameters

`PART_1/px4/gps_denied.params` is annotated. In the `pxh>` shell:

```
param load <path>/PART_1/px4/gps_denied.params
param save
reboot
```

`tools/run_sim.sh` deletes the saved parameter file on every start so that the airframe
defaults apply. The airframe `px4/ROMFS/.../4022_gz_x500_depth_down` sets the same values,
so nothing has to be loaded by hand.

| Parameter | Value | Why |
|---|---|---|
| `SYS_HAS_GPS` | 0 | reboot; the sensor hub stops using `sensor_gps` |
| `EKF2_GPS_CTRL` | 0 | no GPS fusion (default 7) |
| `EKF2_HGT_REF` | 3 (vision) | reboot; left at GPS it starves the height reference and EKF2 looks like it ignores vision |
| `EKF2_EV_CTRL` | 7 | horizontal pos + vertical pos + 3D velocity |
| `EKF2_EV_DELAY` | 50 ms | reboot; set to the measured pipeline latency (step 6) |
| `EKF2_EV_POS_X/Y/Z` | 0.12 / -0.03 / -0.242 | FRD lever arm, from the SDF |
| `COM_RCL_EXCEPT` | 4 | bit 2: OFFBOARD exempt from RC loss |
| `COM_POS_FS_EPH` / `COM_VEL_FS_EVH` | 5.0 / 1.0 (default) | these are the failsafe limits we are judged against, so they stay |

GPS can't be switched off at the source in Gazebo: `GZBridge` always subscribes to the
navsat sensor and v1.16 has no GPS-denied switch. Turning off its consumption
(`SYS_HAS_GPS=0`, `EKF2_GPS_CTRL=0`) is the correct way. `cs_gps_hgt == false` in
`estimator_status_flags` shows it.

### 4.3 Exposing `estimator_status`

Add to `src/modules/uxrce_dds_client/dds_topics.yaml` under `publications:` and rebuild with
`make px4_sitl`:

```yaml
  - topic: /fmu/out/estimator_status
    type: px4_msgs::msg::EstimatorStatus
```

That gives `pos_test_ratio`, `vel_test_ratio` and `hgt_test_ratio`. `vision_health_node`
subscribes to it automatically when it exists.

---

## 5. Build and check offline

```bash
cd PART_1
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/uav_vision/test -q                     # 45 passed
cd tools && PYTHONPATH=../src/uav_vision python3 validate_vo.py --suite full
```

`validate_vo.py` is the cheapest regression check there is. Run it after every change to
`vo_core.py`; both real bugs in the core were caught there (see `validation.md`).

---

## 6. Bring-up

```bash
cd PART_1/tools
./run_sim.sh --no-mission     # everything except arming
./check_stack.sh              # second terminal
```

- **QoS.** PX4 publishes `/fmu/out/*` best-effort; a reliable subscriber never gets
  anything. Durability also has to match: check `ros2 topic info /fmu/out/vehicle_attitude
  --verbose` and set `px4_durability` in `config/vo_params.yaml`.
- **Latency.** `latency_ms` in `/diagnostics` is only the VO compute time. Measure camera
  stamp → EV publish end to end and put that in `EKF2_EV_DELAY` (expect 30–60 ms).
- **Healthy state:** `/uav_visual_odometry/odom` at ~30 Hz, quality ≥ 60, health OK;
  `/fmu/in/vehicle_visual_odometry` at 30 Hz; `cs_ev_pos` and `cs_ev_vel` true,
  `cs_gps_hgt` false; `xy_valid`/`z_valid` true, `dead_reckoning` false, `eph` well under 5 m.

---

## 7. Flying

```bash
./run_sim.sh          # WAIT_HEALTH -> ARM -> TAKEOFF 10 m -> HOLD 90 s -> LAND
```

The mission node prints the maximum hold radius and PASS/FAIL against 1.5 m at the end.
If the hold is marginal, attitude noise is almost always the reason
(position error ≈ altitude × tan(attitude error), see `validation.md`), so look at EKF2's
attitude before touching the vision side. After that: raise `min_features`, lower
`rekey_shift`, raise `clahe_clip` if the light changes.

## 8. Degradation demos

1. Lighting: animate a `<light>` or change the camera exposure in flight.
2. Motion blur: fly faster translations.
3. Feature loss: fly over an untextured patch or publish black frames. Health goes
   DEGRADED/LOST, quality 0, the mission goes to its zero-velocity hold, then recovers.
4. Dropout: stop the camera bridge for 5 s. Position comes back without a jump (the keyframe
   survives) and `reset_counter` increments once.

---

## 9. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| No callbacks on `/fmu/out/*` | QoS mismatch | match `px4_durability` to `ros2 topic info -v` |
| 0 features | untextured ground | step 3 |
| EV published, `cs_ev_pos` false | rate < 5 Hz, `EKF2_EV_CTRL=0`, or frames left at 0 | the bridge sets frames; check the parameter |
| EV accepted, then ignored after a minute | `EKF2_HGT_REF` still GPS | set 3 and reboot |
| "height estimate not stable" at arming | `hgt_test_ratio > 0.5` | let EV settle; check `EKF2_EV_DELAY` |
| "horizontal position unstable" | `pos_test_ratio > 0.5` | usually a frame/sign error, run the tests |
| Drifts away smoothly after arming | VO in `dead_reckoning` instead of `keyframe` | check `/diagnostics` |
| Position rotated 90° | wrong `camera_link_rpy` | step 2 |
| OFFBOARD rejected | setpoints not streaming first, or no valid local position | the node streams > 20 setpoints first; check `COM_RCL_EXCEPT` |
| Low real-time factor | 1080p RGB rendering | drop the RGB image to 640×480 in the SDF |
