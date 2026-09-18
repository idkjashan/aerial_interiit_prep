# Logs

Put the Gazebo runs here, one folder per run:

```
logs/
├── reference/          fly.py reference
│   ├── <px4 log>.ulg
│   ├── reference_<time>.csv
│   └── results/        python3 ../../tools/plot_log.py <px4 log>.ulg --out results
├── sweep/              fly.py sweep (100 -> 75 -> 50 -> 25 ...)
├── zero/               fly.py sweep --levels 0
└── compare.png         plot_log.py reference/*.ulg zero/*.ulg --compare --labels "built-in" "0 %"
```

PX4 writes the .ulg to `PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/`; copy the
one for each run in here. The screen recording for the video goes next to it.
