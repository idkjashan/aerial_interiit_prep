# Logs

- `bag_<date>/` — rosbag recorded by `tools/run_sim.sh` (VO odometry, quality, health, AGL,
  diagnostics, EV sent to PX4, EKF2 local position, status and estimator flags)
- plots of the 90 s hold: hold radius, EKF2 innovation test ratios, VO quality
- the screen recording of the flight

`run_sim.sh` also writes one text log per component here (`px4.log`, `vision.log`, ...);
those are not committed.
