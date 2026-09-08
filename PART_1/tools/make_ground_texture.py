#!/usr/bin/env python3
"""Generate a tileable, high-frequency ground texture for the Gazebo world.

WHY THIS EXISTS: every stock PX4/Gazebo world uses an untextured ground_plane (a flat
solid colour). A downward camera over it produces an image with ZERO corners, so
goodFeaturesToTrack returns nothing and the entire VO pipeline is dead on arrival. This
is the single biggest blocker for Part 1 and it is not obvious until you fly.

SDF has no UV-tiling parameter (confirmed: sdf/1.9/plane_shape.sdf exposes only <normal>
and <size>, and the <pbr> material block has no repeat/scale element), so a single
albedo_map gets stretched once across the whole plane. The practical fix used here is to
BAKE the tiling into one large image sized to the actual flight area.

    python3 make_ground_texture.py --out ground_albedo.png --px 4096 --tiles 24
"""
import argparse
import numpy as np
import cv2


def make(px=4096, tiles=24, seed=7):
    rng = np.random.default_rng(seed)
    cell = max(-(-px // tiles), 64)          # ceil, so tiles*cell >= px exactly
    t = np.zeros((cell, cell), np.float32)
    for octave, amp in ((4, 0.5), (8, 0.3), (16, 0.2), (32, 0.14), (64, 0.1)):
        n = rng.random((octave, octave)).astype(np.float32)
        n = np.vstack([n, n[:1]]); n = np.hstack([n, n[:, :1]])   # wrap for seamlessness
        t += amp * cv2.resize(n, (cell, cell), interpolation=cv2.INTER_CUBIC)
    for _ in range(cell * 3):                       # gravel: strong, localised corners
        c = rng.integers(0, cell, 2)
        cv2.circle(t, (int(c[0]), int(c[1])), int(rng.integers(1, 5)), float(rng.random()), -1)
    t -= t.min(); t /= max(t.max(), 1e-9)
    img = np.tile(t, (tiles, tiles))[:px, :px]
    img = cv2.GaussianBlur(img, (0, 0), 0.6)
    rgb = np.stack([img * 0.62, img * 0.60, img * 0.55], -1)      # desaturated asphalt
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='ground_albedo.png')
    ap.add_argument('--px', type=int, default=4096)
    ap.add_argument('--tiles', type=int, default=24)
    a = ap.parse_args()
    cv2.imwrite(a.out, make(a.px, a.tiles))
    print(f'wrote {a.out} ({a.px}x{a.px}, {a.tiles} tiles)')
    print('Rule of thumb: texture GSD (plane_size/px) must be FINER than the camera GSD\n'
          '  (altitude/fx). At 10 m with fx=432 the camera sees 2.3 cm/px, so a 120 m\n'
          '  plane needs >= 4096 px (2.9 cm/texel). Coarser and features blur out.')
