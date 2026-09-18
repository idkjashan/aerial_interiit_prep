# Handover: integration and testing on the Ubuntu machine

Written 2026-09-18 at the end of a session in WSL, where there is no ROS, PX4 or Gazebo.
Nothing in this commit has been flown. Your job is to build it, fly it, collect the logs
and finish the result sections, not to redesign it.

Read `PART_2/README.md` and `PART_1/README.md` first; this file only covers what you need
to know to continue.

---

## 1. Machine facts

(from `.agent/environment.md`; the PX4 version was worked out during this session)

- `~/PX4-Autopilot`, branch `drdo_depth_down`, `v1.16.0-rc1-661-gbedaca02bc`. That is
  upstream `main` from about 2025-08-14 (`c2e2d8231e`) plus a few local commits (airframe
  4022, model `x500_depth_down`). It is **before** upstream `e59afce5db` (2025-09-30), which
  changed how `failure motor off` works. That matters for Part 2's reference run.
- `~/px4_ros_ws`: `px4_msgs` release/1.16. ROS 2 Humble, Gazebo Harmonic 8.12,
  MicroXRCEAgent 2.4.2.
- Part 1 runs on PX4 instance 1 (DDS 8890, `ROS_DOMAIN_ID=77`); Part 2 uses instance 0
  (MAVLink 14540). They don't collide.

---

## 2. What changed in the repo

**Layout.** The repo root is no longer the colcon workspace; `PART_1/` is.

```bash
cd ~/aerial_interiit_prep
git pull
rm -rf build install log       # old workspace output at the root
cd PART_1 && colcon build --symlink-install && source install/setup.bash
```

- `tools/px4_gps_denied.params` → `PART_1/px4/gps_denied.params`
- `IMPLEMENTATION_PLAN.md` → rewritten as `PART_1/docs/setup.md`
- `docs/VALIDATION.md` → `PART_1/docs/validation.md`
- `SYSTEM_ENVIRONMENT.md` → `.agent/environment.md`
- `run_sim.sh` / `check_stack.sh`: no hard-coded `/home/jashan` paths; they use `PX4_DIR`,
  `PX4_WS` (defaults `~/PX4-Autopilot`, `~/px4_ros_ws`) and find `PART_1/` themselves.
  Logs now go to `PART_1/logs/`.
- `uav_sim_bringup`: the `training_pool` / `uav_guided_ugv` paths are gone. Worlds and
  models are searched in PX4's gz folders and then on `GZ_SIM_RESOURCE_PATH`. For the DRDO
  worlds, export `GZ_SIM_RESOURCE_PATH=~/training_pool/src/drdo_gz_worlds/models:~/training_pool/src/drdo_gz_worlds/worlds`
  before launching. The unused `config/sim_config.yaml` and the unused
  `takeoff_altitude`/`hold_seconds` launch args were removed (they were never passed on).

**Part 1 code changes (made without a simulator, please re-verify):**

1. `vo_core.py`: `consecutive_dr` initialised in `reset()`; an unreachable branch removed;
   `step()` takes an optional `depth_lookup` instead of `vo_node` monkey-patching
   `_depth_at` every frame. `vo_node.py` passes it. Offline results are unchanged.
2. `offboard_mission_node.py`: **bug fix.** DEGRADED → HOLD waited `degraded_grace_s`
   measured from when vision was *lost*, so after any dropout longer than 1 s it resumed
   position hold on the first good frame. Now vision has to be good for 1 s continuously.
3. Launch file: plain `IfCondition` import. `vo_node` docstring: correct topic list.
4. Docstring and comment tone only, elsewhere.

---

## 3. What is verified and what isn't

Verified in WSL:

- Part 1: 45/45 unit tests; `validate_vo.py --suite full` (numbers in
  `PART_1/docs/validation.md`; the nominal hover is unchanged at 0.234 m, 6 ms).
- Found while checking: with the re-keying logic from your last commit, CLAHE no longer
  helps on the synthetic lighting test (0.477 m with, 0.392 m without). The docs say so
  honestly. CLAHE stays on because the Gazebo hover was flown with it.
- Part 2:
  - `ControlAllocator.cpp` with the patch compiles cleanly (clang via zig,
    `-Wall -Wextra -Werror -Wdouble-promotion -Wshadow`, real PX4 headers, generated
    uORB + parameter headers, stubbed Kconfig).
  - `CA_EFF_*` pass PX4's `module.yaml` → param generator; `RotorEffectiveness.msg` passes
    the uORB generator; `dds_topics.yaml` parses.
  - Both patches apply cleanly to `c2e2d8231e`; 0001 applies to v1.16.0 with `git apply -C1`.
  - `model/test_allocation.py` (13) and `tools/test_plot_log.py` (3, on synthetic .ulg files)
    pass.
  - `tools/fly.py` was run through every branch (sweep, crash-stop, finish-and-land,
    reference with and without effect) against a MAVLink mock of PX4 (not committed).

An independent review of the patch and tools (same session) found and we fixed: new
allocator objects after a `CA_METHOD`/`CA_AIRFRAME` change never got a normalization if the
scale was < 1 (now the scale resets to 1 on any re-creation, which also covers boot); a NaN
scale would have NaN'd every motor (now treated as 1); the motor-range check used a stale
motor count; `fly.py` left the scale / an injected motor-off active after Ctrl-C or a crash
(now restored on every exit); `plot_log.py` timed the built-in removal from a 5 Hz topic
(now from the full-rate motor output). Documented, not changed: a scale change made while
the built-in handling has a motor removed waits for the next parameter update.

Not verified:

- Anything in Gazebo.
- `GZMixingInterfaceESC.cpp` (patch 0002) was **not** compiled; it needs gz headers. It is
  three lines; if it fails, check `_mixing_output.outputFunction(i)` / `maxValue(i)`.
- The Part 1 ROS nodes after the edits above: only byte-compiled and linted.

---

## 4. Tasks, in order

### Part 1

1. Rebuild (section 2), run `PART_1/tools/run_sim.sh`, confirm the 90 s hold still passes.
2. Exercise the fixed DEGRADED path once: during the hold, stop the camera bridge for about
   5 s and restart it. Expect: health LOST → mission DEGRADED (zero-velocity hold) → back to
   HOLD about 1 s after vision returns, no position jump. Keep that bag, it's the
   "failsafe + recovery" evidence the PS asks for.
3. Copy the PX4-side files that only exist on this machine into `PART_1/px4/` (the PS wants
   every added or modified file):
   - `ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_down`
   - `Tools/simulation/gz/models/x500_depth_down/` (model.sdf, model.config)
   - `Tools/simulation/gz/worlds/vo_ground.sdf`
   - `Tools/simulation/gz/models/textured_ground/` without the large PNG (regenerated by
     `tools/make_ground_texture.py`)
   - the `dds_topics.yaml` change for `estimator_status`, if it was made
4. `run_sim.sh` deletes the saved parameters on each start, so what flew is whatever
   airframe 4022 sets. Compare it with `PART_1/px4/gps_denied.params` and make the file
   match what actually flew; fix `PART_1/docs/setup.md` §4 if values differ.
5. Put the good bag(s) in `PART_1/logs/` (the old ones are in `~/aerial_interiit_prep/logs/`,
   which used to be gitignored). There is no plotting script for Part 1 yet. Write a small
   `PART_1/tools/plot_bag.py` (rosbag2_py) for: horizontal distance from the hold point vs
   time with the 1.5 m line, `pos/vel/hgt_test_ratio` vs time with the 0.5 line, VO quality
   and health. Save the PNGs in `PART_1/logs/`.
6. Screen recording of the flight.

### Part 2

1. Patch and build PX4:
   ```bash
   cd ~/PX4-Autopilot && git checkout -b part2-rotor-effectiveness
   git am -3 ~/aerial_interiit_prep/PART_2/px4/patches/*.patch
   make px4_sitl
   ```
2. Quick check in `pxh>` (`PART_2/tools/run_sitl.sh --gui`, then in the PX4 shell):
   ```
   control_allocator status              # effectiveness matrix, column 1 = motor 1
   param set CA_EFF_MOTOR 1
   param set CA_EFF_SCALE 0.5            # console: "Motor 1 effectiveness 50%"
   control_allocator status              # column 1 halved, "Motor 1 effectiveness scaled to 0.50"
   listener rotor_effectiveness
   param set CA_EFF_SCALE 1
   ```
3. **Stock reference, worth recording.** Build with patch 0001 only (or build before
   applying 0002), run `python3 PART_2/tools/fly.py reference`. Expected: the injection is
   accepted and nothing happens, and `plot_log.py` labels it "allocator did not react". This
   backs up README section 3. Then build with 0002 and run the reference again. Expected on the
   PX4 console: `Removing motor from allocation (0x1)`, then "Motor failure detected". The
   vehicle spins up and tumbles.
4. Sweep: `python3 PART_2/tools/fly.py sweep`. When it stops on the ground, restart PX4 and
   run the remaining levels it prints (at least `--levels 0` on its own).
5. `plot_log.py` on every .ulg, and `--compare` of the reference against the 0 % run. Put
   everything in `PART_2/logs/` as described in `PART_2/logs/README.md`.
6. Fill the Gazebo table in `PART_2/README.md` §5 and add the Gazebo figures. Compare with the
   model table right below it and say where Gazebo differs; don't bend the text to match the
   model. The model predicts:

   | level | model |
   |---|---|
   | 75 %, 50 % | hold after a transient (max tilt 5° / 10°) |
   | 40 % | holds; 35 % and below: can't make hover thrust |
   | 25 % | stays upright, reaches the ground in ~4 s |
   | 0 % | tumbles, ground in ~2.3 s; built-in handling within ~0.1 s of that |

   If Gazebo holds at 25 %, the hover-thrust estimator or airmode may be why; check
   `MPC_THR_HOVER` / `hover_thrust_estimate` and `MC_AIRMODE` in the log.
7. Video: `run_sitl.sh --gui`, with the PX4 console (it prints every effectiveness change) and
   `fly.py`'s status line visible next to Gazebo.
8. Copy the modified files from your patched PX4 tree back over `PART_2/px4/msg` and
   `PART_2/px4/src`, and regenerate the patches with `git format-patch` from your branch,
   so the submitted files match your exact PX4.
9. Optional: copy `RotorEffectiveness.msg` into `~/px4_ros_ws/src/px4_msgs/msg/`, rebuild,
   and `ros2 topic echo /fmu/out/rotor_effectiveness`.

---

## 5. Things to keep in mind

- **Parameter update rate.** The allocator reads parameters at most once per second
  (`SubscriptionInterval` in `ControlAllocator`). `fly.py` sets `CA_EFF_MOTOR` before takeoff
  and changes levels seconds apart, so that's fine; don't set motor and scale back to back
  mid-flight.
- **Reboot resets the scale.** `CA_EFF_SCALE` is reset to 1 when the allocator starts. That's
  on purpose, not a bug.
- **Logging.** `run_sitl.sh` sets `SDLOG_PROFILE=147` (SITL default + high rate) so motor
  commands and attitude are logged at full rate. PX4 logs from boot, one file per PX4 run.
- **Crashes.** After a tumble PX4 may stay armed upside down (flight termination is off in
  SITL). Just restart PX4 for the next run.
- **Model.** `PART_2/model/quad_sim.py` is a simplified model. Keep it labelled as a
  prediction in the docs.

## 6. How the repo should read

The user wants the repo to read like their team wrote it: plain comments and docs, no
assistant-style phrasing, no emojis. Agent-only material (like this file) stays in `.agent/`.
The submission zip is `PART_1/` and `PART_2/`.

## 7. Branch and remote

Everything above is on `main` (pushed 2026-09-18), so a plain `git pull` gets it. Check
with the user before pushing your own results back.
