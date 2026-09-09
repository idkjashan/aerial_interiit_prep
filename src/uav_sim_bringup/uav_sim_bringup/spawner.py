#!/usr/bin/env python3
"""UAV spawner CLI and ROS 2 node.

Spawns the downward RGB-D camera UAV (x500_depth_down) into an active Gazebo Sim
world with world-specific terrain compensation or user-specified coordinates.
"""
import argparse
import os
import subprocess
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from uav_sim_bringup.world_config import WORLDS, get_uav_pose, get_world_config


def find_model_sdf(model_name: str = 'x500_depth_down') -> Optional[str]:
    """Find the SDF file for the UAV model."""
    search_paths = [
        f"/home/jashan/aerial_interiit_prep/models/{model_name}/model.sdf",
        f"/home/jashan/PX4-Autopilot/Tools/simulation/gz/models/{model_name}/model.sdf",
        f"/home/jashan/training_pool/src/drdo_gz_worlds/models/{model_name}/model.sdf",
        f"/home/jashan/uav_guided_ugv/install/drdo_gz_worlds/share/drdo_gz_worlds/models/{model_name}/model.sdf",
    ]
    for p in search_paths:
        if os.path.isfile(p):
            return p
    return None


def is_world_running(world_name: str, timeout_s: float = 3.0) -> bool:
    """Check if Gazebo is running and broadcasting the requested world."""
    cmd = ["gz", "service", "-i", "--service", f"/world/{world_name}/scene/info"]
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2.0)
            if "Service providers" in res.stdout:
                return True
        except (subprocess.SubprocessError, FileNotFoundError):
            pass
        time.sleep(0.5)
    return False


def spawn_uav(
    world_name: str,
    model_name: str = 'x500_depth_down',
    instance_name: str = 'x500_depth_down_0',
    pose: Optional[Tuple[float, float, float, float, float, float]] = None,
    allow_renaming: bool = False,
) -> bool:
    """Spawn UAV into the specified Gazebo world via gz service."""
    if pose is None:
        pose = get_uav_pose(world_name)

    x, y, z, r, p, yaw = pose

    model_sdf_path = find_model_sdf(model_name)
    if not model_sdf_path:
        print(f"[ERROR] Could not locate SDF for model '{model_name}'.", file=sys.stderr)
        return False

    print(f"[INFO] Spawning '{instance_name}' ({model_name}) into world '{world_name}'...")
    print(f"[INFO] Target pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, R={r:.4f}, P={p:.4f}, Y={yaw:.4f}")
    print(f"[INFO] Model SDF: {model_sdf_path}")

    # Wrap model SDF with pose
    sdf_str = (
        f'<sdf version="1.9">'
        f'<include>'
        f'<uri>file://{model_sdf_path}</uri>'
        f'<pose>{x} {y} {z} {r} {p} {yaw}</pose>'
        f'</include>'
        f'</sdf>'
    )

    req_str = f'name: "{instance_name}", allow_renaming: {"true" if allow_renaming else "false"}, sdf: \'{sdf_str}\''
    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req", req_str,
    ]

    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10.0)
        if res.returncode == 0 and ("data: true" in res.stdout or "data: True" in res.stdout):
            print(f"[SUCCESS] Successfully spawned '{instance_name}' in '{world_name}'.")
            return True
        else:
            print(f"[ERROR] Spawn service failed: {res.stdout} {res.stderr}", file=sys.stderr)
            return False
    except Exception as e:
        print(f"[ERROR] Subprocess error during spawn: {e}", file=sys.stderr)
        return False


def delete_uav(world_name: str, instance_name: str = 'x500_depth_down_0') -> bool:
    """Remove UAV instance from world if already present."""
    cmd = [
        "gz", "service",
        "-s", f"/world/{world_name}/remove",
        "--reqtype", "gz.msgs.Entity",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", f'name: "{instance_name}", type: MODEL',
    ]
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5.0)
        return res.returncode == 0 and "data: true" in res.stdout
    except Exception:
        return False


class SpawnerNode(Node):
    """ROS 2 wrapper node for UAV spawning."""

    def __init__(self):
        super().__init__('uav_spawner_node')
        self.declare_parameters('', [
            ('world', 'vo_ground'),
            ('model', 'x500_depth_down'),
            ('name', 'x500_depth_down_0'),
            ('x', float('nan')),
            ('y', float('nan')),
            ('z', float('nan')),
            ('roll', float('nan')),
            ('pitch', float('nan')),
            ('yaw', float('nan')),
            ('wait_for_world', True),
        ])
        self.spawn()

    def spawn(self):
        g = lambda n: self.get_parameter(n).value
        world = str(g('world'))
        model = str(g('model'))
        name = str(g('name'))
        wait = bool(g('wait_for_world'))

        if wait:
            self.get_logger().info(f"Waiting for Gazebo world '{world}'...")
            if not is_world_running(world, timeout_s=30.0):
                self.get_logger().error(f"Timed out waiting for world '{world}'!")
                return

        # Check if custom pose supplied
        vals = [g('x'), g('y'), g('z'), g('roll'), g('pitch'), g('yaw')]
        import math
        if not any(math.isnan(v) for v in vals):
            pose = tuple(float(v) for v in vals)
        else:
            pose = get_uav_pose(world)

        success = spawn_uav(world, model_name=model, instance_name=name, pose=pose)
        if success:
            self.get_logger().info(f"Spawned UAV '{name}' successfully.")
        else:
            self.get_logger().error(f"Failed to spawn UAV '{name}'.")


def main(args=None):
    parser = argparse.ArgumentParser(description='Spawn UAV into Gazebo Sim world.')
    parser.add_argument('--world', '-w', default='vo_ground', help='Target world name (default: vo_ground)')
    parser.add_argument('--model', '-m', default='x500_depth_down', help='Model name')
    parser.add_argument('--name', '-n', default='x500_depth_down_0', help='Instance name')
    parser.add_argument('--x', type=float, default=None, help='Spawn X (meters)')
    parser.add_argument('--y', type=float, default=None, help='Spawn Y (meters)')
    parser.add_argument('--z', type=float, default=None, help='Spawn Z (meters)')
    parser.add_argument('--roll', type=float, default=None, help='Spawn Roll (rad)')
    parser.add_argument('--pitch', type=float, default=None, help='Spawn Pitch (rad)')
    parser.add_argument('--yaw', type=float, default=None, help='Spawn Yaw (rad)')
    parser.add_argument('--replace', action='store_true', help='Delete existing model first')
    parser.add_argument('--list-worlds', action='store_true', help='List supported worlds')

    cli_args, ros_args = parser.parse_known_args()

    if cli_args.list_worlds:
        print("\nRegistered Worlds & Terrain Poses:")
        for w_name, cfg in WORLDS.items():
            p = cfg.uav_pose
            print(f"  - {w_name:20s}: x={p[0]:8.3f}, y={p[1]:8.3f}, z={p[2]:8.3f} | {cfg.description}")
        sys.exit(0)

    # If launched as a standalone CLI script
    if len(ros_args) <= 1 or not any('__node' in a for a in ros_args):
        if cli_args.replace:
            print(f"[INFO] Removing previous instance of '{cli_args.name}' if present...")
            delete_uav(cli_args.world, cli_args.name)

        pose = None
        if all(v is not None for v in [cli_args.x, cli_args.y, cli_args.z, cli_args.roll, cli_args.pitch, cli_args.yaw]):
            pose = (cli_args.x, cli_args.y, cli_args.z, cli_args.roll, cli_args.pitch, cli_args.yaw)

        ok = spawn_uav(cli_args.world, cli_args.model, cli_args.name, pose=pose)
        sys.exit(0 if ok else 1)

    # If launched via ros2 run / launch
    rclpy.init(args=args)
    node = SpawnerNode()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
