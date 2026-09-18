# Inter-IIT Tech Meet 15.0 Prepathon — Aerial Robotics

GPS-denied navigation from a downward camera, and rotor effectiveness inside PX4's control
allocation. Both parts run on PX4 SITL in Gazebo Harmonic (Ubuntu 22.04, ROS 2 Humble).

| | |
|---|---|
| [PART_1](PART_1/) | Visual odometry from a downward RGB-D camera, fused into EKF2 as external vision. Normal arming, 10 m takeoff and a 90 s hold with GPS off (max 0.1 m from the hold point in Gazebo), plus manual flying and a vision-loss/recovery demo. |
| [PART_2](PART_2/) | A runtime scale on one rotor's column of PX4's effectiveness matrix. The x500 at 20 m holds down to 50 %, comes down upright at 25 % and tumbles at 0 %, which we compare with PX4's own motor-failure handling. |

Each part has its own README with the approach, setup, how to run it, results, logs and a
video. The submission zip is the two folders as they are.

## Setup in short

- PX4-Autopilot (v1.16 line) in `~/PX4-Autopilot` (or `PX4_DIR`), Gazebo Harmonic,
  Micro-XRCE-DDS-Agent, and `px4_msgs` (release/1.16) built in `~/px4_ros_ws`.
- Part 1: copy `PART_1/px4/*` into the PX4 tree (same paths), then build the ROS 2 workspace
  in `PART_1/`.
- Part 2: `git am -3 PART_2/px4/patches/*.patch` in the PX4 tree.
- `make px4_sitl`, then follow the part's README.

## Checks that don't need a simulator

```bash
pip install numpy opencv-python pytest matplotlib pyulog pymavlink
python3 -m pytest PART_1/src/uav_vision/test PART_2/model PART_2/tools -q
```
