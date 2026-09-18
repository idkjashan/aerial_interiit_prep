# Logs

- `bag_<date>/`: rosbag recorded by `tools/run_sim.sh` (VO odometry, quality, health, AGL,
  diagnostics, EV sent to PX4, EKF2 local position, status and estimator flags)
- `hold_distance.png`, `innovations.png`, `vision_health.png`: from
  `python3 tools/plot_bag.py <bag>`

The plots are in git. The bag folders and the per-component text logs that `run_sim.sh`
writes here (`px4.log`, `vision.log`, ...) are left out of git; the bags go into the
submission zip.
