# System Environment & Robotics Stack Specification

Snapshot of the Ubuntu machine the project runs on (moved here from the repo root).

**Some values below are wrong** (EV lever-arm signs, `EKF2_EV_CTRL = 11`, RGB HFOV, "4022 is
upstream"). The corrected values are in `PART_1/docs/setup.md` section 0 and in
`PART_1/px4/gps_denied.params`; trust those.

---

## 1. Host System & Hardware Architecture

| Component | Specification | Details / Environment Flags |
| :--- | :--- | :--- |
| **Operating System** | Ubuntu 22.04.5 LTS (Jammy Jellyfish) | Linux Kernel `6.8.0-138-generic` x86_64 |
| **Username / Host** | `jashan` / `LOQ` | Workspace root: `/home/jashan/aerial_interiit_prep` |
| **CPU** | Intel x86_64 Multi-core | High-concurrency SITL & compilation support |
| **Dedicated GPU** | NVIDIA GeForce RTX 4060 Laptop GPU | 8188 MiB GDDR6 VRAM |
| **NVIDIA Driver** | `570.172.08` | CUDA 12.8 driver capability |
| **Gazebo GPU Offload** | Hardware accelerated via PRIME | Variables required before launching Gazebo: |

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
```

---

## 2. Core Robotics Stack & Versions

### 2.1 ROS 2
- **Distribution:** ROS 2 Humble Hawksbill
- **Install Path:** `/opt/ros/humble`
- **Environment:** `ROS_DISTRO=humble`, `ROS_VERSION=2`
- **Default RMW (DDS):** `rmw_fastrtps_cpp` (eProsima Fast-DDS `2.6.10`)
- **Key Installed Packages:**
  - `ros-humble-cv-bridge` (v3.2.1)
  - `ros-humble-image-transport` (v3.1.11)
  - `ros-humble-navigation2` / `nav2-*` (v1.1.18)
  - `ros-humble-tf2-ros`, `ros-humble-tf2-geometry-msgs` (v0.25.12)
  - `ros-humble-ros-gzharmonic` / `ros-humble-ros-gzharmonic-bridge` (v0.244.12)

### 2.2 Build Toolchain & Compilers
- **Build System:** `colcon` (`colcon-core` with `ament_cmake` and `ament_python`)
- **C/C++ Compiler:** GCC/G++ `11.4.0` (`-std=c++17` / `-std=c++20`)
- **CMake:** Version `3.22.1`
- **Python:** Python `3.10.12` (`/usr/bin/python3`)
  - `opencv-python` / `cv2`: `4.12.0`
  - `numpy`: `1.26.4`
  - `scipy`: `1.15.3`

### 2.3 Simulator: Gazebo Sim (Harmonic)
- **Engine:** Gazebo Sim version `8.12.0` (Gazebo Harmonic release)
- **CLI Executable:** `/usr/bin/gz` (`gz sim`)
- **Bridge Package:** `ros_gz_bridge` (`ros2 run ros_gz_bridge parameter_bridge ...`)
- **Simulation Models & Worlds Path:**
  ```bash
  export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}
  ```

---

## 3. PX4 Autopilot & DDS Communication Bridge

### 3.1 PX4-Autopilot Repository
- **Local Directory:** `/home/jashan/PX4-Autopilot`
- **Version / Tag:** `v1.16.0-rc1-661-gbedaca02bc` (Release 1.16 series)
- **Active Working Branch:** `drdo_depth_down`
- **Key Airframe Definitions:**
  - Standard x500 Quadrotor: `gz_x500` (Airframe 4001)
  - Downward Depth Quadrotor: `gz_x500_depth_down` (Airframe 4022)
  - Vision Quadrotor: `x500_vision` (Airframe with front mono camera)
  - Optical Flow Quadrotor: `x500_flow`

### 3.2 Micro-XRCE-DDS-Agent
- **Binary Path:** `/usr/local/bin/MicroXRCEAgent`
- **Source Repository:** `/home/jashan/Micro-XRCE-DDS-Agent` (Tag `v2.4.2`, commit `57d0862`)
- **Protocol:** UDP over IPv4
- **Default Port:** `8888`
- **Execution Command:**
  ```bash
  MicroXRCEAgent udp4 -p 8888
  ```
- *Note:* PX4 SITL automatically establishes a client connection to `127.0.0.1:8888` upon startup.

### 3.3 ROS 2 Message Definitions (`px4_msgs`)
- **Workspace Path:** `/home/jashan/px4_ros_ws`
- **Branch:** `release/1.16` (commit `392e831`, v1.16.2 definition)
- **Source Script:**
  ```bash
  source /home/jashan/px4_ros_ws/install/setup.bash
  ```
- **QoS Requirements for PX4 Topics:**
  All PX4 uXRCE-DDS pub/sub interactions in ROS 2 **must** use best-effort QoS:
  ```python
  from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

  px4_qos = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT,
      durability=DurabilityPolicy.TRANSIENT_LOCAL,
      history=HistoryPolicy.KEEP_LAST,
      depth=1,
  )
  ```

---

## 4. Drone Models & Camera Specifications

### 4.1 Downward Depth Airframe: `gz_x500_depth_down` (ID 4022)
SDF definition: `/home/jashan/PX4-Autopilot/Tools/simulation/gz/models/x500_depth_down/model.sdf`

Equipped with an **OakD-Lite** sensor mounted at relative pose `[0.12, 0.03, 0.242, 0, 1.5708, 0]` (pitch down 90°):

#### 1. IMX214 (RGB Sensor)
- **Type:** 2D Color Camera
- **Frame ID:** `camera_link`
- **Resolution:** 1920 x 1080
- **Horizontal FOV:** 1.57 rad (~90°)
- **Update Rate:** 30 Hz
- **Clip Planes:** Near: 0.1 m, Far: 100.0 m
- **Gazebo Topic:** `/world/<world_name>/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image`
- **Camera Info:** `/world/<world_name>/model/x500_depth_down_0/link/camera_link/sensor/IMX214/camera_info`

#### 2. StereoOV7251 (Depth Sensor)
- **Type:** Depth Camera
- **Frame ID:** `camera_link`
- **Resolution:** 640 x 480
- **Image Format:** `R_FLOAT32` (32-bit floating point depth in meters)
- **Horizontal FOV:** 1.274 rad (~73°)
- **Update Rate:** 30 Hz
- **Clip / Depth Range:** Near: 0.2 m, Far: 19.1 m
- **Gazebo Depth Topic:** `/depth_camera`
- **Gazebo PointCloud Topic:** `/depth_camera/points`

### 4.2 Standard Gazebo-to-ROS 2 Bridge Configuration
```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/$W/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /world/$W/model/x500_depth_down_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /world/$W/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  --ros-args \
  -r /world/$W/clock:=/clock \
  -r /world/$W/model/x500_depth_down_0/link/camera_link/sensor/IMX214/image:=/uav/rgb \
  -r /world/$W/model/x500_depth_down_0/link/camera_link/sensor/IMX214/camera_info:=/uav/camera_info \
  -r /depth_camera:=/uav/depth \
  -r /depth_camera/points:=/uav/points
```

---

## 5. Coordinate Frames & Mathematical Conversions

A common failure in drone robotics is coordinate frame mismatch. The system strictly distinguishes between three standards:

```
       PX4 World (NED)                          ROS 2 World (ENU)
         +X (North)                                +Y (North)
             ▲                                         ▲
             │                                         │
             │                                         │
             └────────► +Y (East)                      └────────► +X (East)
            ╱                                         ╱
           ▼                                         ▲
         +Z (Down)                                 +Z (Up)
```

### 5.1 Summary of Reference Frames
1. **PX4 World (NED):** $+X$ North, $+Y$ East, $+Z$ Down. Heading: $0^\circ$ at North, clockwise positive.
2. **PX4 Body (FRD):** $+X$ Forward, $+Y$ Right, $+Z$ Down.
3. **ROS 2 World (ENU):** $+X$ East, $+Y$ North, $+Z$ Up. Heading: $0^\circ$ at East, counter-clockwise positive.
4. **ROS 2 Body (FLU):** $+X$ Forward, $+Y$ Left, $+Z$ Up.
5. **Camera Optical (RDF):** $+X$ Right, $+Y$ Down, $+Z$ Forward along optical axis.

### 5.2 Position & Velocity Conversions
$$\begin{bmatrix} X_{\text{ENU}} \\ Y_{\text{ENU}} \\ Z_{\text{ENU}} \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix} \begin{bmatrix} X_{\text{NED}} \\ Y_{\text{NED}} \\ Z_{\text{NED}} \end{bmatrix} \iff \begin{aligned} X_{\text{ENU}} &= Y_{\text{NED}} \\ Y_{\text{ENU}} &= X_{\text{NED}} \\ Z_{\text{ENU}} &= -Z_{\text{NED}} \end{aligned}$$

$$\begin{bmatrix} X_{\text{NED}} \\ Y_{\text{NED}} \\ Z_{\text{NED}} \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix} \begin{bmatrix} X_{\text{ENU}} \\ Y_{\text{ENU}} \\ Z_{\text{ENU}} \end{bmatrix} \iff \begin{aligned} X_{\text{NED}} &= Y_{\text{ENU}} \\ Y_{\text{NED}} &= X_{\text{ENU}} \\ Z_{\text{NED}} &= -Z_{\text{ENU}} \end{aligned}$$

### 5.3 Yaw Angle Conversions
$$\psi_{\text{ENU}} = \text{wrap}\left(\frac{\pi}{2} - \psi_{\text{NED}}\right)$$
$$\psi_{\text{NED}} = \text{wrap}\left(\frac{\pi}{2} - \psi_{\text{ENU}}\right)$$

```python
import math

def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def ned_to_enu(x_ned: float, y_ned: float, z_ned: float):
    return (float(y_ned), float(x_ned), float(-z_ned))

def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
    return (float(y_enu), float(x_enu), float(-z_enu))

def yaw_ned_to_enu(yaw_ned: float) -> float:
    return normalize_angle(math.pi / 2.0 - yaw_ned)

def yaw_enu_to_ned(yaw_enu: float) -> float:
    return normalize_angle(math.pi / 2.0 - yaw_enu)
```

---

## 6. GPS-Denied Flight & Visual Odometry Configuration

### 6.1 PX4 EKF2 Parameter Set for GPS-Denied Visual Navigation
To run PX4 without GPS using External Vision (EV / Visual Odometry), configure the following parameters in PX4 (either via `init.d-posix` airframe script, QGroundControl, or MAVLink shell):

| Parameter | Recommended Value | Meaning |
| :--- | :--- | :--- |
| `EKF2_GPS_CTRL` | `0` | **Disable GPS fusion** entirely in EKF2 |
| `SYS_HAS_GPS` | `0` | Notify system that no GPS sensor is attached |
| `EKF2_EV_CTRL` | `11` or `15` | **EV bitmask**: bit 0 (horizontal pos: 1) + bit 1 (vertical pos: 2) + bit 3 (yaw: 8) = **11**. Add bit 2 (3D velocity: 4) = **15**. |
| `EKF2_HGT_REF` | `3` | Height sensor source: **3 = EV (Vision)** (alternative: `1` Range finder / Lidar, `0` Barometer) |
| `COM_ARM_WO_GPS` | `1` | Allow arming in manual/offboard modes without GPS fix |
| `EKF2_EV_DELAY` | `10.0` to `20.0` | EV measurement latency buffer in milliseconds |
| `EKF2_EV_POS_X` | `0.12` | Sensor X offset from drone CG (meters) |
| `EKF2_EV_POS_Y` | `0.03` | Sensor Y offset from drone CG (meters) |
| `EKF2_EV_POS_Z` | `0.242` | Sensor Z offset from drone CG (meters) |
| `NAV_DLL_ACT` | `0` | Disable data-link loss failsafe during testing |
| `COM_RCL_EXCEPT` | `7` | RC loss exception bitmask (7 allows offboard flight without RC) |

### 6.2 Odometry Input Topic: `/fmu/in/vehicle_visual_odometry`
In PX4 v1.16, visual odometry is fed through topic `/fmu/in/vehicle_visual_odometry` with message type `px4_msgs/msg/VehicleOdometry`.

Key fields in `px4_msgs/msg/VehicleOdometry`:
```python
from px4_msgs.msg import VehicleOdometry

odom_msg = VehicleOdometry()
odom_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # microseconds
odom_msg.timestamp_sample = odom_msg.timestamp

# 1 = POSE_FRAME_NED (world-fixed NED)
# 2 = POSE_FRAME_FRD (body-relative heading)
odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED

# Position in NED meters: [North, East, Down] (down is negative altitude!)
odom_msg.position = [float(pos_north), float(pos_east), float(pos_down)]

# Orientation quaternion: [w, x, y, z] from FRD body frame to NED world
odom_msg.q = [float(qw), float(qx), float(qy), float(qz)]

# 1 = VELOCITY_FRAME_NED, 3 = VELOCITY_FRAME_BODY_FRD
odom_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
odom_msg.velocity = [float(vx_ned), float(vy_ned), float(vz_ned)]
odom_msg.angular_velocity = [float(roll_rate), float(pitch_rate), float(yaw_rate)]

# Covariances & Quality
odom_msg.position_variance = [0.01, 0.01, 0.01]
odom_msg.orientation_variance = [0.005, 0.005, 0.005]
odom_msg.velocity_variance = [0.02, 0.02, 0.02]
odom_msg.reset_counter = 0
odom_msg.quality = 100 # (0-100%)
```

---

## 7. Drone Offboard Control Protocol (ROS 2 / PX4 v1.16)

### 7.1 Topic Interaction Catalog

| Topic Name | Type | Direction | Frequency | Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | ROS 2 $\to$ PX4 | $\ge 10\text{ Hz}$ | Heartbeat enabling active control dimensions |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | ROS 2 $\to$ PX4 | $\ge 10\text{ Hz}$ | Target position / velocity in NED |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | ROS 2 $\to$ PX4 | Event-based | Arming (`CMD 400`) & Offboard switch (`CMD 176`) |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` | ROS 2 $\to$ PX4 | $20\text{–}30\text{ Hz}$ | External Vision pose estimate fed to EKF2 |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | PX4 $\to$ ROS 2 | $50\text{ Hz}$ | EKF2 fused state estimate (NED) |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | PX4 $\to$ ROS 2 | $5\text{ Hz}$ | Arming and navigation state flags |

### 7.2 Safe Handshake Sequence
1. Start streaming `/fmu/in/offboard_control_mode` (`position: True`) and `/fmu/in/trajectory_setpoint` at $10\text{–}20\text{ Hz}$.
2. After at least 10–20 setpoints have been sent, send `VehicleCommand`:
   - `command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE` (176)
   - `param1 = 1.0` (custom mode)
   - `param2 = 6.0` (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
3. Send Arm Command:
   - `command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM` (400)
   - `param1 = 1.0` (Arm)

---

## 8. Simulation Bringup Quickstart Workflow

Run the following commands in separate terminals or as a coordinated shell script:

```bash
# Terminal 1: Start MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: Start Gazebo Sim & PX4 SITL (using downward depth airframe 4022)
cd ~/PX4-Autopilot
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=gz_x500_depth_down PX4_GZ_WORLD=default ./build/px4_sitl_default/bin/px4 -d

# Terminal 3: Start ROS-Gazebo Bridge
source /opt/ros/humble/setup.bash
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

# Terminal 4: Visual Odometry Node & Offboard Controller (your implementation in this workspace)
source /opt/ros/humble/setup.bash
source /home/jashan/px4_ros_ws/install/setup.bash
source ~/aerial_interiit_prep/install/setup.bash
```
