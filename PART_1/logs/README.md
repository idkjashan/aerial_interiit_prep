# Logs

- `bag_<date>/`: rosbag recorded by `tools/run_sim.sh` (VO odometry, quality, health, AGL,
  diagnostics, EV sent to PX4, EKF2 local position, status and estimator flags)
- `hold_distance.png`, `innovations.png`, `vision_health.png`: from
  `python3 tools/plot_bag.py <bag>`

The plots, rosbags, and per-component text logs (`px4.log`, `vision.log`, etc.) are tracked for verification and submission.
