# Inter-IIT Tech Meet 15.0 Prepathon — Aerial Robotics

GPS-denied navigation from a downward camera, and rotor effectiveness inside PX4's control
allocation. Both parts run on PX4 SITL in Gazebo Harmonic.

| | |
|---|---|
| [PART_1](PART_1/) | Visual odometry from a downward RGB-D camera, fused into EKF2 as external vision. Normal arming, 10 m takeoff and a 90 s hover with GPS turned off. |
| [PART_2](PART_2/) | A runtime scale on one rotor's column of PX4's effectiveness matrix, the x500's behaviour at 20 m from 100 % down to 0 %, and a comparison with PX4's own motor-failure handling. |

Each folder has its own README with the approach, setup and results. The submission zip is
these two folders as they are.

## Requirements

- Ubuntu 22.04, Gazebo Harmonic, PX4-Autopilot (v1.16 line) built for SITL
- Part 1: ROS 2 Humble, `px4_msgs` (release/1.16), Micro-XRCE-DDS-Agent, Python OpenCV + NumPy
- Part 2: Python with `pymavlink`, `pyulog`, `matplotlib`, `numpy`; PX4 patched with
  `PART_2/px4/patches`

## Checks that don't need a simulator

```bash
pip install numpy opencv-python pytest matplotlib pyulog pymavlink
python3 -m pytest PART_1/src/uav_vision/test PART_2/model PART_2/tools -q
```
