# Aerial Inter-IIT Preparation Workspace

This repository is the dedicated preparation and development workspace for solving the **GPS-Denied Drone Navigation & Visual Odometry Control** problem statement (Inter-IIT Tech Meet / Aerial Robotics).

---

## 🎯 Problem Statement Overview
Control an aerial vehicle (multirotor drone) autonomously in environments where **GPS signals are completely denied / absent**. 
State estimation is achieved using **onboard Visual Odometry (VO / VIO)**, fusing optical sensors (RGB-D camera / downward optical sensors) directly with PX4's Extended Kalman Filter (EKF2) via Micro-XRCE-DDS and executing offboard trajectory setpoint tracking.

---

## 📋 System Environment Specification
An exhaustive, version-locked specification of the development & simulation machine has been generated in:
👉 **[`SYSTEM_ENVIRONMENT.md`](SYSTEM_ENVIRONMENT.md)**

This file includes:
- **OS & Hardware:** Ubuntu 22.04.5 LTS, NVIDIA RTX 4060 Laptop GPU (Driver 570.172.08)
- **Robotics Middleware:** ROS 2 Humble Hawksbill, eProsima Fast-DDS 2.6.10
- **Flight Controller:** PX4 Autopilot `v1.16.0-rc1` (Branch `drdo_depth_down`, Airframe ID 4022 `gz_x500_depth_down`)
- **DDS Bridge:** Micro-XRCE-DDS-Agent `v2.4.2` (`/usr/local/bin/MicroXRCEAgent`)
- **Simulator:** Gazebo Sim Harmonic `8.12.0` with `ros_gz_bridge`
- **Sensors:** OakD-Lite Downward RGB-D (IMX214 1080p @ 30Hz + StereoOV7251 Depth @ 30Hz)
- **Mathematical Framework:** Rigid ENU $\iff$ NED $\iff$ FRD coordinate transformations
- **PX4 EKF2 Parameters:** Exact bitmasks and configuration values for GPS-denied EV fusion
- **PX4 uORB/ROS 2 Topics:** Topics, message layouts, and QoS profiles

---

## 📦 Part 1 — Implemented and validated

The Part 1 solution (GPS-denied visual odometry → EKF2 → 90 s vision-only position hold) is
implemented in **[`PART_1/`](PART_1/)**:

- **[`PART_1/IMPLEMENTATION_PLAN.md`](PART_1/IMPLEMENTATION_PLAN.md)** — step-by-step integration
  guide for the Ubuntu machine, a verified PX4 v1.16 parameter reference, and a troubleshooting
  matrix. **Start here.**
- **[`PART_1/README.md`](PART_1/README.md)** — approach, measured results, requirement traceability.
- **[`PART_1/docs/VALIDATION.md`](PART_1/docs/VALIDATION.md)** — how the numbers were produced.
- `PART_1/src/uav_vision/` — four ROS 2 nodes plus a ROS-free VO core (45 unit tests).
- `PART_1/tools/` — offline validation harness, ground-texture generator, PX4 params, bringup.

Measured offline against exact ground truth: **0.234 m max horizontal error over a 90 s hover
at 10 m** (gate: 1.5 m) at **5.9 ms p95 latency** (gate: 60 ms), with graceful degradation and
automatic recovery from total vision loss.

> ⚠️ Two findings that override this file:
> 1. **Every stock Gazebo world has an untextured ground plane** — measured **0** trackable
>    corners versus **500** with texture. VO cannot work until this is fixed.
> 2. **Several values in `SYSTEM_ENVIRONMENT.md` below are wrong** (external-vision lever-arm
>    signs, `EKF2_EV_CTRL=11` silently excluding velocity, RGB FOV, and the assumption that
>    airframe 4022 is upstream). Corrections with sources are in
>    [`PART_1/IMPLEMENTATION_PLAN.md` §0](PART_1/IMPLEMENTATION_PLAN.md).

---

## 🤖 Workflow with WSL High-Reasoning AI Model
1. **Pass [`SYSTEM_ENVIRONMENT.md`](SYSTEM_ENVIRONMENT.md)** to your high-reasoning model in WSL.
2. Have the model draft the implementation plan, node architecture, and ROS 2 packages adhering strictly to the documented topic names, message structures, and frame conventions.
3. Bring the generated ROS 2 packages and nodes into this workspace (`src/`).
4. Build and simulate directly on this native Linux machine with Gazebo Harmonic and PX4 SITL.

---

## 🚀 Quick Bringup Workflow

### 1. Source Environment
```bash
source /opt/ros/humble/setup.bash
source /home/jashan/px4_ros_ws/install/setup.bash
```

### 2. Run DDS Bridge
```bash
MicroXRCEAgent udp4 -p 8888
```

### 3. Run PX4 SITL in Gazebo Harmonic
```bash
cd /home/jashan/PX4-Autopilot
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=gz_x500_depth_down PX4_GZ_WORLD=default ./build/px4_sitl_default/bin/px4 -d
```

### 4. Run ROS-Gazebo Parameter Bridge
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  --ros-args \
  -r /world/default/clock:=/clock \
  -r /world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image:=/uav/rgb \
  -r /world/default/model/x500_depth_down_0/link/camera_link/sensor/IMX214/camera_info:=/uav/camera_info \
  -r /depth_camera:=/uav/depth \
  -r /depth_camera/points:=/uav/points
```

---

## 📂 Repository Structure (Planned)
```
aerial_interiit_prep/
├── README.md                  # This file
├── SYSTEM_ENVIRONMENT.md      # Ground-truth environment & API spec
├── .gitignore                 # Git ignore rules for ROS 2 & Python
└── src/                       # Custom ROS 2 packages to be implemented
    ├── uav_visual_odometry/   # VO feature tracker & depth estimator
    ├── uav_px4_bridge/        # Odometry translation & EKF2 interface
    └── uav_offboard_control/  # Mission & offboard trajectory setpoints
```
