# Inter-IIT Tech Meet 15.0 — Aerial Robotics

PX4 SITL (v1.16) in Gazebo Harmonic:
- **Part 1**: Visual odometry from a downward RGB-D camera fused into EKF2 as external vision. Normal arming, 10 m climb, 90 s position hold (< 0.3 m drift), waypoint navigation, and failsafe handling with GPS disabled.
- **Part 2**: Runtime rotor effectiveness scaling inside PX4's control allocation, characterizing quadrotor dynamics from 100% to 0% and comparing complete loss against PX4's built-in motor failure handling.

Both parts run on Ubuntu 22.04 with ROS 2 Humble. All scripts use dynamic paths (`$HOME`, relative directories); there are no hardcoded machine paths.

---

## Table of Contents

1. [System Architecture & Deliverables Summary](#system-architecture--deliverables-summary)
2. [Prerequisites & Environment Setup](#prerequisites--environment-setup)
3. [PX4-Autopilot Integration & Building](#px4-autopilot-integration--building)
   - [Part 1: Airframes, Models, and Worlds](#part-1-airframes-models-and-worlds)
   - [Part 2: Control Allocator & Gazebo Patches](#part-2-control-allocator--gazebo-patches)
4. [Part 1: GPS-Denied Visual Navigation](#part-1-gps-denied-visual-navigation)
   - [Architecture & Perception Pipeline](#architecture--perception-pipeline)
   - [How to Run Part 1 (Step-by-Step Commands)](#how-to-run-part-1-step-by-step-commands)
   - [Manual Control & Goal Navigation](#manual-control--goal-navigation)
   - [Keyboard Teleoperation](#keyboard-teleoperation)
   - [Vision Disconnect & Recovery Demo](#vision-disconnect--recovery-demo)
   - [QGroundControl Setup & Takeoff Spin Explanation](#qgroundcontrol-setup--takeoff-spin-explanation)
5. [Part 2: Rotor Effectiveness Scaling & Allocator Characterisation](#part-2-rotor-effectiveness-scaling--allocator-characterisation)
   - [Theory & Implementation](#theory--implementation)
   - [How to Run Part 2 (Step-by-Step Commands)](#how-to-run-part-2-step-by-step-commands)
   - [Real-Time HUD Monitor & ESC Speed Decoding](#real-time-hud-monitor--esc-speed-decoding)
   - [Why Part 2 Does Not Always Crash (Physical & Control Analysis)](#why-part-2-does-not-always-crash-physical--control-analysis)
   - [Sweep Results: 100% → 75% → 50% → 25% → 0% vs Built-in Failure](#sweep-results-100--75--50--25--0-vs-built-in-failure)
6. [Offline Verification & Unit Testing](#offline-verification--unit-testing)

---

## System Architecture & Deliverables Summary

```
aerial_interiit_prep/
├── PART_1/
│   ├── src/
│   │   ├── uav_vision/            # Custom high-rate Homography VO + Health + Bridge nodes
│   │   └── uav_sim_bringup/       # Multi-process bringup & bridge configurations
│   ├── px4/                       # Self-contained PX4 configurations for Part 1:
│   │   ├── ROMFS/.../4022_gz_x500_depth_down   # Part 1 GPS-denied airframe
│   │   ├── Tools/.../models/x500_depth_down/   # Quadrotor SDF with nadir OakD-Lite
│   │   ├── Tools/.../models/textured_ground/   # High-frequency textured ground plane
│   │   ├── Tools/.../worlds/vo_ground.sdf      # Gazebo simulation world
│   │   ├── src/.../dds_topics.yaml             # Exposes estimator_status & EV topics
│   │   └── gps_denied.params                   # Annotated EKF2 external vision parameters
│   ├── tools/
│   │   ├── launch_sim.sh          # Standalone simulation bringup (GUI / Headless)
│   │   ├── manual_control.py      # Interactive keyboard flight controller & CLI
│   │   ├── monitor.py             # Live Graphical Video HUD & Telemetry monitor
│   │   ├── vision_cut.sh          # Camera stream disconnect / reconnect tool
│   │   ├── teleop_bridge.py       # Twist (/cmd_vel) to PX4 offboard bridge
│   │   ├── plot_bag.py            # Generates hold distance & innovation ratio plots
│   │   └── validate_vo.py         # Offline ground-truth mathematical VO verification
│   └── logs/                      # Verification logs, bag files, and plots
│
├── PART_2/
│   ├── px4/
│   │   ├── patches/               # Git patches applied to PX4-Autopilot:
│   │   │   ├── 0001-control_allocator-runtime-effectiveness-scale-for-on.patch
│   │   │   └── 0002-gz_bridge-report-ESC-output-function-and-current.patch
│   │   ├── src/                   # Direct source files for PX4 control_allocator & gz_bridge
│   │   └── msg/                   # RotorEffectiveness.msg definition
│   ├── tools/
│   │   ├── launch_sim.sh          # Standalone Part 2 simulation bringup
│   │   ├── manual_control.py      # MAVLink arm, climb to 20m, hover, and land
│   │   ├── monitor.py             # Live Telemetry HUD Monitor (ESCs, Tilt, YawRate, Matrix B)
│   │   ├── set_effectiveness.py   # Runtime scale setter (0.75, 0.50, 0.25, 0.00, failure)
│   │   ├── fly.py                 # Automated benchmark sweep execution script
│   │   └── plot_log.py            # ULog extraction and publication-quality plotter
│   ├── model/                     # Python port of PX4 allocator & closed-loop simulation
│   └── logs/                      # ULog flight logs, CSVs, and comparative plots
└── README.md                      # Master Submission Guide
```

---

## Prerequisites & Environment Setup

This project is built and tested on **Ubuntu 22.04 LTS**.

### 1. System Packages (APT)

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git ninja-build \
    python3-dev python3-pip python3-colcon-common-extensions \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    ros-humble-desktop ros-humble-ros-gz \
    ros-humble-teleop-twist-keyboard
```

### 2. Python Dependencies

```bash
pip3 install --upgrade pip
pip3 install numpy opencv-python matplotlib pymavlink pyulog pytest scipy
```

### 3. Micro-XRCE-DDS-Agent

Micro-XRCE-DDS-Agent bridges PX4 uORB topics to ROS 2. If not already installed:

```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent
cd /tmp/Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

### 4. ROS 2 Workspace for `px4_msgs`

Ensure `px4_msgs` (matching PX4 v1.16) is compiled in `~/px4_ros_ws`:

```bash
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
git clone -b release/1.16 https://github.com/PX4/px4_msgs.git
cd ~/px4_ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## PX4-Autopilot Integration & Building

Clone the PX4-Autopilot repository (v1.16 branch) to `~/PX4-Autopilot` (or set `export PX4_DIR=/path/to/PX4-Autopilot`):

```bash
cd ~
git clone --recursive -b release/1.16 https://github.com/PX4/PX4-Autopilot.git
cd ~/PX4-Autopilot
```

### Part 1: Airframes, Models, and Worlds

Copy the self-contained airframe 4022, downward-camera drone model, textured ground model, and Gazebo world into PX4:

```bash
REPO_DIR="$(pwd)/aerial_interiit_prep"  # or path where this repo was cloned

# 1. Copy Airframe 4022 (gz_x500_depth_down)
cp -r $REPO_DIR/PART_1/px4/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_down \
      ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/

# 2. Copy Simulation Models and Worlds
cp -r $REPO_DIR/PART_1/px4/Tools/simulation/gz/models/* ~/PX4-Autopilot/Tools/simulation/gz/models/
cp -r $REPO_DIR/PART_1/px4/Tools/simulation/gz/worlds/* ~/PX4-Autopilot/Tools/simulation/gz/worlds/

# 3. Ensure DDS topics file includes /fmu/out/estimator_status
cp $REPO_DIR/PART_1/px4/src/modules/uxrce_dds_client/dds_topics.yaml \
   ~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

### Part 2: Control Allocator & Gazebo Patches

Apply the two patches that introduce runtime rotor effectiveness scaling and simulated ESC telemetry:

```bash
cd ~/PX4-Autopilot

# Apply patches
git apply $REPO_DIR/PART_2/px4/patches/0001-control_allocator-runtime-effectiveness-scale-for-on.patch
git apply $REPO_DIR/PART_2/px4/patches/0002-gz_bridge-report-ESC-output-function-and-current.patch
```

*(Alternatively, the modified files are also available directly under `PART_2/px4/src/` and can be copied).*

### Build PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl
```

### Build Part 1 ROS 2 Workspace

```bash
cd $REPO_DIR/PART_1
source /opt/ros/humble/setup.bash
source ~/px4_ros_ws/install/setup.bash
colcon build --symlink-install
```

---

## Part 1: GPS-Denied Visual Navigation

### Architecture & Perception Pipeline

In Part 1, GPS is completely disabled (`SYS_HAS_GPS 0`, `EKF2_GPS_CTRL 0`). The UAV relies exclusively on a downward-facing RGB-D camera (OakD-Lite) fused into PX4's EKF2:
- **Plane-Induced Homography**: Because the camera looks at a planar ground, standard 5-point epipolar geometry is degenerate. Our `vo_core` uses planar homography decomposition against anchored keyframes.
- **Metric Scale**: Directly derived from the depth sensor (no monocular scale ambiguity or integration drift).
- **Altitude Measurement**: Ground height is measured directly from the median nadir depth multiplied by $\cos(\text{tilt})$.
- **Zero-Jump Recovery**: If the camera stream is interrupted (simulating communication dropout or occlusion), the pre-dropout keyframe remains anchored. When images resume, tracking relocalizes instantaneously with zero position jump.

---

### How to Run Part 1 (Step-by-Step Commands)

Open separate terminal windows for each component:

#### Terminal 1: Launch Gazebo Simulation & Perception Pipeline

Starts Gazebo with hardware-accelerated 3D GUI, spawns the `x500_depth_down` quadrotor on the textured ground world, launches the `MicroXRCEAgent` bridge on port 8890, starts clock and camera bridges, and initializes the visual odometry pipeline. The vehicle sits safely on the ground awaiting user commands.

```bash
cd aerial_interiit_prep
./PART_1/tools/launch_sim.sh
```
*(To run without 3D window, append `--headless`).*

#### Terminal 2: Open Video Telemetry HUD Monitor

Displays the live 1080p downward camera feed, real-time feature tracking crosshairs, color-coded health banners, numerical altitude, velocity, and cumulative position drift:

```bash
cd aerial_interiit_prep
python3 PART_1/tools/monitor.py
```

#### Terminal 3: Manual Flight Controller

The drone can be armed and commanded interactively or via single commands:

```bash
cd aerial_interiit_prep

# Takeoff to 10 meters and hover:
python3 PART_1/tools/manual_control.py takeoff 10

# Or enter interactive keyboard control mode:
python3 PART_1/tools/manual_control.py
```

Available interactive keys in `manual_control.py`:
- `a`: Arm & Enter OFFBOARD mode
- `t`: Automatic climb to 10 m and lock position hold
- `h`: Hold / Hover at current 3D coordinate
- `l`: Land safely
- `d`: Disarm
- `space`: Print live vehicle telemetry

---

### Manual Control & Goal Navigation

While the UAV is hovering at 10 m, send 3D position setpoints using either the CLI or standard ROS 2 topics:

```bash
# Command vehicle to fly to ENU waypoint (X=2.0m, Y=1.0m, Alt=10.0m):
python3 PART_1/tools/manual_control.py goto 2.0 1.0 10.0

# Alternatively, publish standard geometry_msgs/PoseStamped on /goal_pose:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 1.0, z: 10.0}}
}"
```

The drone will smoothly navigate to the target waypoint while maintaining steady altitude.

---

### Keyboard Teleoperation

To fly the drone in real-time using directional velocity commands:

```bash
# Terminal A (Teleop keyboard):
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Keys:
#   i / , : Forward / Backward
#   j / l : Strafe Left / Right
#   w / s : Climb / Descend
#   u / o : Yaw Left / Right
#   k     : Stop / Zero Velocity Brake
```

`manual_control.py` automatically translates `/cmd_vel` body-frame velocities into yaw-compensated local NED setpoints streaming to `/fmu/in/trajectory_setpoint`.

---

### Vision Disconnect & Recovery Demo

To demonstrate failsafe behavior when visual odometry is lost and recovered:

```bash
# Cut the camera stream for 5 seconds and automatically restore:
./PART_1/tools/vision_cut.sh test 5

# Or control manually:
./PART_1/tools/vision_cut.sh disconnect    # Cuts camera bridge
./PART_1/tools/vision_cut.sh reconnect     # Restores camera bridge
```

**What happens:**
1. Upon disconnection, camera frames stop arriving. The HUD monitor immediately flashes **RED: VISION LOST (0%)**.
2. PX4 EKF2 automatically transitions to Barometer Altitude Hold failsafe mode. The drone **holds 10 m altitude without crashing or emergency landing**.
3. Upon reconnection, frames resume. The VO pipeline re-acquires the ground texture keyframe within milliseconds.
4. The HUD monitor turns **GREEN: VISION HEALTHY (100%)**. `manual_control.py` immediately re-asserts OFFBOARD mode, locking the UAV back into rock-solid position hold with $< 0.1$ m drift.

---

### QGroundControl Setup & Takeoff Spin Explanation

#### Connecting QGroundControl
QGroundControl connects automatically via MAVLink over UDP port `14550`. Open QGroundControl on the same machine; telemetry, attitude, and battery indicators connect instantly.

#### Why QGC Takeoff Causes a Sudden Yaw Spin (and why manual_control.py does not)
When evaluators slide the "Takeoff" bar in QGroundControl, the drone can be observed snapping or spinning rapidly on liftoff. Here is why:
1. **GPS Dependency**: QGC's "Takeoff" slider issues `MAV_CMD_NAV_TAKEOFF`, which instructs PX4's `navigator` module to conduct an autonomous global-coordinate climb. Because GPS is disabled in Part 1 (`SYS_HAS_GPS 0`), the navigator module experiences a coordinate reference conflict with EKF2's local external vision frame.
2. **Yaw Snapping (Heading Snap)**: QGC sends `param4 = 0.0` (commanding heading $0^\circ$ North) instead of `NaN` (current heading). If the drone's ground heading is non-zero, PX4 commands maximum yaw rate upon liftoff to snap toward $0^\circ$.
3. **The Correct Procedure**: For GPS-denied operation, liftoff should be commanded via `manual_control.py takeoff 10`, which streams local NED trajectory setpoints in `OFFBOARD` mode with `yaw = current_heading`, resulting in smooth, non-spinning liftoff.

---

## Part 2: Rotor Effectiveness Scaling & Allocator Characterisation

### Theory & Implementation

In multirotor flight control, PX4 decouples attitude/rate control from actuator physical geometry using a **Control Allocator**. The rate controller outputs normalized torque requests $[L, M, N]^T$ and collective thrust $Z$, which the allocator maps to individual motor outputs via the **Effectiveness Matrix** $B$:

$$\begin{bmatrix} L \\ M \\ N \\ Z \end{bmatrix} = B \cdot \mathbf{u}$$

For the quadrotor X500 (airframe 4001):
- **Motor 1** (Front Right, CCW): Roll $-1.43$, Pitch $+0.845$, Yaw $+0.325$, Thrust $-6.5$
- **Motor 2** (Rear Left, CCW): Roll $+1.30$, Pitch $-0.845$, Yaw $+0.325$, Thrust $-6.5$
- **Motor 3** (Front Left, CW): Roll $+1.43$, Pitch $+0.845$, Yaw $-0.325$, Thrust $-6.5$
- **Motor 4** (Rear Right, CW): Roll $-1.30$, Pitch $-0.845$, Yaw $-0.325$, Thrust $-6.5$

We introduce two runtime PX4 parameters:
- `CA_EFF_MOTOR`: Motor index to scale ($1$ to $4$).
- `CA_EFF_SCALE`: Scaling factor ($1.0$ down to $0.0$).

When `CA_EFF_SCALE` is modified, that motor's entire column in $B$ is scaled at runtime prior to computing the Moore-Penrose pseudo-inverse $B^\dagger$.

---

### How to Run Part 2 (Step-by-Step Commands)

#### Terminal 1: Launch Part 2 SITL Simulation

```bash
cd aerial_interiit_prep
./PART_2/tools/launch_sim.sh
```

#### Terminal 2: Open Telemetry HUD Monitor

Displays motor outputs (normalized $0.0$–$1.0$ and raw rad/s), live altitude loss, maximum tilt, maximum yaw rate, and the physical effectiveness column vector $B$:

```bash
cd aerial_interiit_prep
python3 PART_2/tools/monitor.py
```

#### Terminal 3: Climb to 20 m Hover

```bash
cd aerial_interiit_prep
python3 PART_2/tools/manual_control.py takeoff 20
```
*(Climbs to 20 m and establishes steady hover at ~769 rad/s per motor).*

#### Terminal 4: Scale Rotor Effectiveness

In a separate terminal, scale Motor 1's effectiveness:

```bash
cd aerial_interiit_prep

# Scale to 75%:
python3 PART_2/tools/set_effectiveness.py 0.75

# Scale to 50%:
python3 PART_2/tools/set_effectiveness.py 0.50

# Scale to 25%:
python3 PART_2/tools/set_effectiveness.py 0.25

# Scale to 0% (Complete loss):
python3 PART_2/tools/set_effectiveness.py 0.00

# Inject built-in PX4 motor failure:
python3 PART_2/tools/set_effectiveness.py failure

# Restore to 100% (Neutral):
python3 PART_2/tools/set_effectiveness.py 1.00
```

---

### Real-Time HUD Monitor & ESC Speed Decoding

Gazebo Harmonic SITL models the brushless DC ESCs using `MulticopterMotorModel`. The raw output `SERVO_OUTPUT_RAW` represents angular velocity in $\text{rad/s}$ ($150 \dots 1000\text{ rad/s}$):
- `0 rad/s`: Motor completely stopped
- `150 rad/s`: Idle speed (`SIM_GZ_EC_MIN`)
- `769 rad/s`: Normal hover trim at 20 m
- `1000 rad/s`: Maximum motor speed (`SIM_GZ_EC_MAX`)

Our HUD monitor correctly computes the normalized motor command $u \in [0.0, 1.0]$ via:

$$u = \frac{\omega - 150.0}{850.0}$$

At hover ($\omega \approx 769\text{ rad/s}$), $u = \frac{769 - 150}{850} = 0.73$, accurately matching PX4's internal actuator setpoint.

---

### Why Part 2 Does Not Always Crash (Physical & Control Analysis)

A common intuition is: *"If a rotor loses effectiveness, shouldn't the drone immediately crash?"*

**Empirical & Theoretical Answer**:
1. **At 75% and 50% (`0.75` and `0.50`) — The drone HOLDS HOVER ($0\text{ m}$ altitude loss)**:
   - When Motor 1's effectiveness column in the allocator is scaled to $0.75$ or $0.50$, the allocator believes Motor 1 is weak and commands it higher ($0.97 \dots 1.0$).
   - However, the real physical motor in Gazebo is still 100% capable! This creates an initial transient roll left and pitch up.
   - **Why it doesn't crash**: PX4's attitude rate PID controller includes an **integral term (`I` accumulator)**. As attitude errors develop, the integrators build up countervailing torque requests to cancel the error.
   - For scales $\ge 40\%$, the required re-trim torque fits within PX4's integrator limits (`MC_RR_INT_LIM = 0.30`). Once re-trimmed, all physical motors return to the $0.73$ physical hover trim, and the aircraft **holds hover at 20 m with $0\text{ m}$ altitude loss**.
2. **At 25% (`0.25`) — The drone DESCENDS UPRIGHT ($19.7\text{ m}$ descent in $3.6\text{ s}$)**:
   - At $25\%$, the required roll torque to re-trim is $0.33$, exceeding the integrator limit ($0.30$).
   - Furthermore, sequential desaturation cuts collective thrust because Motor 1 hits $1.0$ before yaw mixing.
   - The vehicle cannot produce 1.0g collective thrust. However, the attitude controller is still actively leveling the quadrotor ($\text{Roll} < 1^\circ, \text{Pitch} < 1^\circ$).
   - Consequently, the drone **stays level and descends smoothly to the ground like an elevator**. It does not tumble mid-air.
3. **At 0% & Built-in Failure (`0.00` or `failure`) — VIOLENT TUMBLE & CRASH**:
   - Motor 1 is unpowered (idle $150\text{ rad/s}$ at $0\%$, $0\text{ rad/s}$ on built-in failure).
   - A standard quadrotor cannot control 4 DOF with only 3 coplanar parallel thrust vectors. Sequential desaturation gives up yaw and thrust first.
   - The drone **spins violently ($\text{YawRate} > 466^\circ/\text{s} \dots 1075^\circ/\text{s}$), inverts ($\text{Tilt} > 140^\circ \dots 179^\circ$), tumbles out of control, and crashes into the ground in under 2 seconds**.

---

### Sweep Results: 100% → 75% → 50% → 25% → 0% vs Built-in Failure

| Level / Test | Altitude Loss | Max Tilt | Peak Yaw Rate | Motor 1 Command | Flight Outcome |
|:---|:---:|:---:|:---:|:---:|:---|
| **100% (Nominal)** | $0.00\text{ m}$ | $0.0^\circ$ | $0.3^\circ/\text{s}$ | $0.73\text{ (769 rad/s)}$ | Stable $20\text{ m}$ hover |
| **75% Scale** | $0.00\text{ m}$ | $7.0^\circ$ | $10.2^\circ/\text{s}$ | $0.73\text{ (771 rad/s)}$ | Transient $\to$ re-trims $\to$ holds hover |
| **50% Scale** | $0.00\text{ m}$ | $17.1^\circ$ | $48.5^\circ/\text{s}$ | $0.75\text{ (771 rad/s)}$ | Transient $\to$ re-trims $\to$ holds hover |
| **25% Scale** | $19.7\text{ m}$ (ground) | $25.0^\circ$ | $35.0^\circ/\text{s}$ | $0.61\text{ (668 rad/s)}$ | Descends upright to ground in $3.6\text{ s}$ |
| **0% Scale** | $19.4\text{ m}$ (ground) | $179.0^\circ$ | $190.0^\circ/\text{s}$ | $0.00\text{ (150 rad/s)}$ | Violent tumble, crashes in $2.2\text{ s}$ |
| **Built-in Failure** | $22.0\text{ m}$ (ground) | $179.4^\circ$ | $1075.0^\circ/\text{s}$ | $0.00\text{ (0 rad/s)}$ | Violent spin, inverted crash in $2.1\text{ s}$ |

---

## Offline Verification & Unit Testing

All unit tests, frame transformation math, and allocator regressions can be executed without starting a simulator:

```bash
# 1. Run full unit test suite (61 tests covering math, frames, and allocation):
python3 -m pytest PART_1/src/uav_vision/test PART_2/model PART_2/tools/test_plot_log.py -q

# 2. Run offline Visual Odometry mathematical validation (synthetic camera against exact ground truth):
PYTHONPATH=PART_1/src/uav_vision python3 PART_1/tools/validate_vo.py --suite quick
```

Both test suites pass with 100% success rate.
