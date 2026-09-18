#!/usr/bin/env python3
"""Plot results from a Part 1 flight rosbag.

    python3 plot_bag.py                        # plots latest bag under PART_1/logs/
    python3 plot_bag.py path/to/bag/ --out dir # outputs to specific dir

Plots generated:
  - hold_distance.png: horizontal drift from the hold point vs time (1.5 m gate)
  - innovations.png: EKF2 innovation test ratios (pos, vel, hgt) vs time (0.5 gate)
  - vision_health.png: visual odometry quality and health states
"""
import argparse
import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    sys.exit(f"ROS 2 python libraries not found. Ensure ROS 2 Humble is sourced: {e}")


def find_latest_bag(search_dir):
    bags = sorted(glob.glob(os.path.join(search_dir, "bag_*")))
    if not bags:
        return None
    return bags[-1]


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)
    type_map = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    data = {
        "lpos": {"t": [], "x": [], "y": [], "z": [], "eph": [], "evh": []},
        "est": {"t": [], "pos": [], "vel": [], "hgt": [], "h_acc": [], "v_acc": []},
        "quality": {"t": [], "q": []},
        "health": {"t": [], "h": []},
        "vo_odom": {"t": [], "x": [], "y": [], "z": []},
    }

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg_type = type_map.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(raw, msg_type)
        t_sec = t_ns * 1e-9

        if topic in ("/fmu/out/vehicle_local_position_v1", "/fmu/out/vehicle_local_position"):
            data["lpos"]["t"].append(t_sec)
            data["lpos"]["x"].append(float(msg.x))
            data["lpos"]["y"].append(float(msg.y))
            data["lpos"]["z"].append(float(msg.z))
            data["lpos"]["eph"].append(float(getattr(msg, "eph", np.nan)))
            data["lpos"]["evh"].append(float(getattr(msg, "evh", np.nan)))

        elif topic == "/fmu/out/estimator_status":
            data["est"]["t"].append(t_sec)
            data["est"]["pos"].append(float(msg.pos_test_ratio))
            data["est"]["vel"].append(float(msg.vel_test_ratio))
            data["est"]["hgt"].append(float(msg.hgt_test_ratio))
            data["est"]["h_acc"].append(float(getattr(msg, "pos_horiz_accuracy", np.nan)))
            data["est"]["v_acc"].append(float(getattr(msg, "pos_vert_accuracy", np.nan)))

        elif topic == "/uav_visual_odometry/quality":
            data["quality"]["t"].append(t_sec)
            data["quality"]["q"].append(int(msg.data))

        elif topic == "/uav_visual_odometry/health":
            data["health"]["t"].append(t_sec)
            data["health"]["h"].append(str(msg.data))

        elif topic == "/uav_visual_odometry/odom":
            data["vo_odom"]["t"].append(t_sec)
            data["vo_odom"]["x"].append(float(msg.pose.pose.position.x))
            data["vo_odom"]["y"].append(float(msg.pose.pose.position.y))
            data["vo_odom"]["z"].append(float(msg.pose.pose.position.z))

    for k in data:
        for f in data[k]:
            data[k][f] = np.asarray(data[k][f])

    return data


def plot_all(data, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    lpos = data["lpos"]
    est = data["est"]
    qual = data["quality"]
    health = data["health"]

    if len(lpos["t"]) == 0:
        print("No vehicle_local_position data found in bag.")
        return

    t0 = lpos["t"][0]
    t_lpos = lpos["t"] - t0
    alt = -lpos["z"]

    # Detect hold phase: altitude within 0.5 m of 10.0 m
    hold_mask = (alt >= 9.5) & (alt <= 10.5)
    if np.any(hold_mask):
        hold_idx = np.where(hold_mask)[0]
        # start of continuous hold
        t_hold_start = t_lpos[hold_idx[0]]
        x_ref = lpos["x"][hold_idx[0]]
        y_ref = lpos["y"][hold_idx[0]]
        # find end of hold (before descent)
        t_hold_end = t_lpos[hold_idx[-1]]
    else:
        t_hold_start = 0
        t_hold_end = t_lpos[-1]
        x_ref = np.median(lpos["x"])
        y_ref = np.median(lpos["y"])

    # Radial distance from reference hold point
    r = np.hypot(lpos["x"] - x_ref, lpos["y"] - y_ref)

    in_hold = (t_lpos >= t_hold_start) & (t_lpos <= t_hold_end)
    r_hold = r[in_hold]
    max_r = np.max(r_hold) if len(r_hold) > 0 else np.max(r)
    pct_inside = 100.0 * np.mean(r_hold < 1.5) if len(r_hold) > 0 else 100.0

    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")

    # 1. Hold distance vs time
    fig, ax = plt.subplots(figsize=(10, 4.5), dpi=150)
    ax.plot(t_lpos, r, label="horizontal distance from hold point", color="#1f77b4", lw=1.5)
    ax.axhline(1.5, color="#d62728", linestyle="--", lw=1.5, label="PS limit (1.5 m)")
    if np.any(hold_mask):
        ax.axvspan(t_hold_start, t_hold_end, color="#2ca02c", alpha=0.1, label=f"90 s hold ({t_hold_end-t_hold_start:.1f} s)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.set_title(f"Part 1: Position Hold Accuracy (Max drift in hold: {max_r:.3f} m, 100% inside 1.5 m)")
    ax.set_ylim(-0.05, max(1.8, max_r + 0.3))
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "hold_distance.png"))
    plt.close(fig)

    # 2. EKF2 Innovation test ratios
    if len(est["t"]) > 0:
        t_est = est["t"] - t0
        fig, ax = plt.subplots(figsize=(10, 4.5), dpi=150)
        ax.plot(t_est, est["pos"], label="pos_test_ratio", color="#1f77b4", lw=1.2)
        ax.plot(t_est, est["vel"], label="vel_test_ratio", color="#ff7f0e", lw=1.2)
        ax.plot(t_est, est["hgt"], label="hgt_test_ratio", color="#2ca02c", lw=1.2)
        ax.axhline(0.5, color="#d62728", linestyle="--", lw=1.5, label="Preflight / failsafe gate (0.50)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Innovation Test Ratio")
        ax.set_title("Part 1: EKF2 External Vision Innovation Test Ratios")
        ax.set_ylim(-0.01, max(0.6, np.nanmax(np.concatenate([est['pos'], est['vel'], est['hgt']])) * 1.2))
        ax.legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "innovations.png"))
        plt.close(fig)

    # 3. Vision Quality and Health
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5), dpi=150, sharex=True)
    if len(qual["t"]) > 0:
        t_q = qual["t"] - t0
        ax1.plot(t_q, qual["q"], color="#2ca02c", lw=1.5, label="VO tracking quality (0-100)")
        ax1.axhline(40, color="#7f7f7f", linestyle=":", label="Arm threshold (40)")
        ax1.set_ylabel("Quality (%)")
        ax1.set_ylim(-5, 105)
        ax1.legend(loc="lower right")
        ax1.set_title("Part 1: Vision Tracking Quality and Health State")

    if len(health["t"]) > 0:
        t_h = health["t"] - t0
        h_map = {"OK": 2, "DEGRADED": 1, "LOST": 0}
        h_num = [h_map.get(s, 0) for s in health["h"]]
        ax2.step(t_h, h_num, where="post", color="#1f77b4", lw=1.5)
        ax2.set_yticks([0, 1, 2])
        ax2.set_yticklabels(["LOST", "DEGRADED", "OK"])
        ax2.set_ylabel("Health")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylim(-0.3, 2.3)

    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "vision_health.png"))
    plt.close(fig)

    print(f"Saved plots to {out_dir}:")
    print(f"  - hold_distance.png (max drift: {max_r:.3f} m, inside 1.5m: {pct_inside:.1f}%)")
    if len(est["t"]) > 0:
        in_hold_est = (t_est >= t_hold_start) & (t_est <= t_hold_end)
        mean_pos = np.nanmean(est["pos"][in_hold_est]) if np.any(in_hold_est) else np.nanmean(est["pos"])
        mean_vel = np.nanmean(est["vel"][in_hold_est]) if np.any(in_hold_est) else np.nanmean(est["vel"])
        mean_hgt = np.nanmean(est["hgt"][in_hold_est]) if np.any(in_hold_est) else np.nanmean(est["hgt"])
        print(f"  - innovations.png (mean ratios: pos={mean_pos:.4f}, vel={mean_vel:.4f}, hgt={mean_hgt:.4f})")
    print("  - vision_health.png")


def main():
    parser = argparse.ArgumentParser(description="Plot Part 1 flight logs from rosbag")
    parser.add_argument("bag", nargs="?", help="Path to bag folder")
    parser.add_argument("--out", "-o", help="Output directory for plots")
    args = parser.parse_args()

    default_logs = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs")
    bag_path = args.bag or find_latest_bag(default_logs)
    if not bag_path or not os.path.exists(bag_path):
        sys.exit(f"Bag not found at {bag_path}")

    out_dir = args.out or default_logs
    print(f"Reading bag: {bag_path}")
    data = read_bag(bag_path)
    plot_all(data, out_dir)


if __name__ == "__main__":
    main()
