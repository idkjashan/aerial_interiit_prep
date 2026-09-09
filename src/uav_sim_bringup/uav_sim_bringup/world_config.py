"""World configurations and terrain spawn poses for UAV and UGV simulation.

Coordinates are verified against the DRDO Inter-IIT competition definitions and
custom benchmark worlds.
"""
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


@dataclass(frozen=True)
class WorldConfig:
    name: str
    description: str
    sdf_filename: str
    uav_pose: Tuple[float, float, float, float, float, float]  # x, y, z, roll, pitch, yaw
    ugv_pose: Optional[Tuple[float, float, float, float, float, float]] = None
    default_alt_target: float = 10.0


# Registry of known worlds
WORLDS: Dict[str, WorldConfig] = {
    'vo_ground': WorldConfig(
        name='vo_ground',
        description='Flat high-frequency textured ground (PS GPS-denied VO benchmark)',
        sdf_filename='vo_ground.sdf',
        uav_pose=(0.0, 0.0, 0.25, 0.0, 0.0, 0.0),
        default_alt_target=10.0,
    ),
    'drdo_world1': WorldConfig(
        name='drdo_world1',
        description='DRDO terrain world 1: hilly switchback road',
        sdf_filename='drdo_world1.sdf',
        uav_pose=(-10.226, 311.831, 22.863, 0.011338, 0.135709, -2.161422),
        ugv_pose=(-12.220319, 308.976703, 22.295580, 0.011338, 0.135709, -2.161422),
        default_alt_target=10.0,
    ),
    'drdo_world1_overlay': WorldConfig(
        name='drdo_world1_overlay',
        description='DRDO terrain world 1 with obstacle overlay',
        sdf_filename='drdo_world1_overlay.sdf',
        uav_pose=(-10.226, 311.831, 22.863, 0.011338, 0.135709, -2.161422),
        ugv_pose=(-12.220319, 308.976703, 22.295580, 0.011338, 0.135709, -2.161422),
        default_alt_target=10.0,
    ),
    'drdo_world2': WorldConfig(
        name='drdo_world2',
        description='DRDO terrain world 2: curved valley canyon',
        sdf_filename='drdo_world2.sdf',
        uav_pose=(103.776917, -101.472992, 17.318562, -0.054656, 0.032451, 2.460081),
        ugv_pose=(104.742386, -101.9010777, 15.730011, -0.054656, 0.032451, 2.460081),
        default_alt_target=10.0,
    ),
    'drdo_world2_overlay': WorldConfig(
        name='drdo_world2_overlay',
        description='DRDO terrain world 2 with obstacle overlay',
        sdf_filename='drdo_world2_overlay.sdf',
        uav_pose=(103.776917, -101.472992, 17.318562, -0.054656, 0.032451, 2.460081),
        ugv_pose=(104.742386, -101.9010777, 15.730011, -0.054656, 0.032451, 2.460081),
        default_alt_target=10.0,
    ),
    'drdo_world3': WorldConfig(
        name='drdo_world3',
        description='DRDO terrain world 3: high altitude ridge',
        sdf_filename='drdo_world3.sdf',
        uav_pose=(108.849, -265.663, 49.4752, 0.045161, 0.003268, 1.588),
        ugv_pose=(109.076, -262.736, 49.2026, 0.005846, -0.047033, 1.56611),
        default_alt_target=10.0,
    ),
    'drdo_world3_overlay': WorldConfig(
        name='drdo_world3_overlay',
        description='DRDO terrain world 3 with obstacle overlay',
        sdf_filename='drdo_world3_overlay.sdf',
        uav_pose=(108.849, -265.663, 49.4752, 0.045161, 0.003268, 1.588),
        ugv_pose=(109.076, -262.736, 49.2026, 0.005846, -0.047033, 1.56611),
        default_alt_target=10.0,
    ),
    'baylands': WorldConfig(
        name='baylands',
        description='Gazebo baylands coastal landscape',
        sdf_filename='baylands.sdf',
        uav_pose=(0.0, 0.0, 2.5, 0.0, 0.0, 0.0),
        default_alt_target=10.0,
    ),
    'default': WorldConfig(
        name='default',
        description='Gazebo empty ground plane',
        sdf_filename='default.sdf',
        uav_pose=(0.0, 0.0, 0.25, 0.0, 0.0, 0.0),
        default_alt_target=10.0,
    ),
}


def get_world_config(world_name: str) -> WorldConfig:
    """Retrieve world configuration, falling back to default if unknown."""
    if world_name in WORLDS:
        return WORLDS[world_name]
    # Check if suffix or prefix matches
    cleaned = world_name.replace('.sdf', '')
    if cleaned in WORLDS:
        return WORLDS[cleaned]
    # Fallback to vo_ground
    return WORLDS['vo_ground']


def get_uav_pose(world_name: str) -> Tuple[float, float, float, float, float, float]:
    """Return UAV spawn pose (x, y, z, roll, pitch, yaw) for world."""
    return get_world_config(world_name).uav_pose


def get_uav_pose_str(world_name: str) -> str:
    """Return comma-separated pose string for PX4_GZ_MODEL_POSE env var."""
    pose = get_uav_pose(world_name)
    return f"{pose[0]},{pose[1]},{pose[2]},{pose[3]},{pose[4]},{pose[5]}"


def get_ugv_pose(world_name: str) -> Optional[Tuple[float, float, float, float, float, float]]:
    """Return UGV spawn pose if defined for the world."""
    return get_world_config(world_name).ugv_pose
