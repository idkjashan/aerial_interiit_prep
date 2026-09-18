#!/usr/bin/env python3
"""Offline validation of the VO core against synthetic ground truth (no ROS, no Gazebo).

Run it before flying and after any change to vo_core.py. It renders a downward camera over a
textured plane along a known 10 m hover trajectory, so position error is known exactly,
and reports it against the PS Phase-2 gate (inside 1.5 m for 90 s).

    python3 validate_vo.py                # nominal 90 s run
    python3 validate_vo.py --suite full   # + lighting / blur / low texture / dropout

Requires only numpy + opencv-python. Add src/uav_vision to PYTHONPATH:
    PYTHONPATH=../src/uav_vision python3 validate_vo.py
"""
import argparse, time
import numpy as np
import cv2

import simworld as S
from _traj import trajectory, deriv, ang_rate
from uav_vision.vo_core import DownwardVO, VOParams, health_name
from uav_vision.frames import camera_rotation_in_body

S.R_CAM_IN_BODY = camera_rotation_in_body()      # keep renderer and VO in one convention


def run(duration=90., rate=30., W=640, H=480, hfov=1.274, att_noise_deg=0.3,
        att_bias_deg=0.0, gyro_noise=0.003, depth_noise=0.02, blur=0.0, light=False,
        dropout=None, texture_gain=1.0, seed=1, label='', recovery=False):
    rng = np.random.default_rng(seed)
    ground, mpp = S.make_ground(4000, 40.0, seed=7)
    if texture_gain != 1.0:
        ground = np.clip(128 + (ground.astype(np.float32) - 128) * texture_gain,
                         0, 255).astype(np.uint8)
    cam = S.DownCam(ground, mpp, W, H, hfov)
    vo, dt, n = DownwardVO(cam.K, VOParams()), 1. / rate, int(duration * rate)
    bias = np.deg2rad(att_bias_deg) * np.array([1., -0.6, 0.3])
    rec = []
    for i in range(n):
        t = i * dt
        p, rpy = trajectory(t)
        img, depth = cam.render(p, rpy)
        if light:
            img = np.clip(img.astype(np.float32) * (1 + 0.45 * np.sin(0.11 * 2 * np.pi * t))
                          + 40 * np.sin(0.07 * 2 * np.pi * t), 0, 255).astype(np.uint8)
        if blur > 0:
            img = cv2.GaussianBlur(img, (0, 0), blur * (1 + 0.8 * abs(np.sin(np.pi * t))))
        drop = dropout is not None and dropout[0] <= t < dropout[1]
        if drop:
            img = np.full_like(img, 12); depth = np.full_like(depth, np.nan)
        if depth_noise > 0:
            depth = depth + rng.normal(0, depth_noise, depth.shape).astype(np.float32)
        rpy_m = rpy + bias + np.deg2rad(att_noise_deg) * rng.standard_normal(3)
        w_m = ang_rate(t) + gyro_noise * rng.standard_normal(3)
        dm = np.nanmedian(depth)
        alt = float(dm) * np.cos(np.hypot(*rpy_m[:2])) if np.isfinite(dm) else float('nan')
        t0 = time.perf_counter()
        r = vo.step(img, depth, rpy_m, w_m, dt, alt)
        r.update(lat=(time.perf_counter() - t0) * 1e3, t=t, gt=p,
                 gtv=deriv(trajectory, t), drop=drop)
        rec.append(r)

    good = [r for r in rec if r['ok'] and not r['drop']]
    if len(good) < 10:
        print(f'  [{label:<30}] FAILED ({len(good)} usable frames)'); return None
    gt = np.array([r['gt'] for r in good]); es = np.array([r['pos'] for r in good])
    gv = np.array([r['gtv'] for r in good]); ev = np.array([r['vel'] for r in good])
    off = gt[0] - es[0]; err = es + off - gt
    eh = np.linalg.norm(err[:, :2], axis=1)
    lat = np.array([r['lat'] for r in rec])
    kf = sum(1 for r in good if r['mode'] == 'keyframe') / len(good) * 100
    print(f'  [{label:<30}] horiz mean={eh.mean():5.3f} p95={np.percentile(eh,95):5.3f} '
          f'MAX={eh.max():5.3f} m | vel RMSE={np.sqrt(((ev-gv)**2).sum(1).mean()):5.3f} m/s | '
          f'z={np.abs(err[:,2]).mean():5.3f} m | feat={np.mean([r["n_tracked"] for r in good]):4.0f} '
          f'| kf={kf:3.0f}% | lat p95={np.percentile(lat,95):4.1f} ms')
    if recovery and dropout:
        dur = [r for r in rec if dropout[0] <= r['t'] < dropout[1]]
        post = [r for r in rec if dropout[1] <= r['t'] < dropout[1] + 2.0 and r['ok']]
        pre = [r for r in rec if r['t'] < dropout[0] and r['ok']]
        e_post = np.array([np.linalg.norm((r['pos'] + off - r['gt'])[:2]) for r in post])
        e_pre = np.array([np.linalg.norm((r['pos'] + off - r['gt'])[:2]) for r in pre])
        print(f'        during loss: health={sorted({health_name(r["health"]) for r in dur})} '
              f'quality={sorted({r["quality"] for r in dur})}')
        print(f'        recovery: {e_pre.mean():.3f} m before -> {e_post.mean():.3f} m after '
              f'(max {e_post.max():.3f}) => '
              f'{"RECOVERED, no jump" if e_post.max() < max(0.5, 3*e_pre.mean()) else "JUMPED"}')
    return eh, lat


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', choices=['quick', 'full'], default='quick')
    a = ap.parse_args()
    print('=' * 132)
    print('NOMINAL: 90 s hover @ 10 m, 30 Hz, depth-camera intrinsics (640x480, hfov 1.274)')
    print('=' * 132)
    out = run(90., label='nominal 90 s')
    if out:
        eh, lat = out
        ok = eh.max() < 1.5
        print(f'        PS Phase-2 gate  (inside 1.5 m for 90 s): max {eh.max():.3f} m  '
              f'=> {"PASS" if ok else "FAIL"} (margin x{1.5/max(eh.max(),1e-9):.1f})')
        print(f'        PS latency gate  (< 60 ms):                p95 {np.percentile(lat,95):.1f} ms '
              f'=> {"PASS" if np.percentile(lat,95) < 60 else "FAIL"}')
    if a.suite == 'full':
        print(); print('=' * 132); print('DEGRADATIONS named in the PS'); print('=' * 132)
        run(45., light=True, label='lighting variation +-45%')
        run(45., blur=4.5, label='severe motion blur')
        run(45., texture_gain=0.05, label='near-featureless ground 5%')
        run(60., dropout=(20., 25.), label='5 s TOTAL vision dropout', recovery=True)
        print(); print('=' * 132)
        print('ATTITUDE SENSITIVITY (dominant error term: position err ~ altitude * tan(err))')
        print('=' * 132)
        for d in (0.1, 0.3, 0.6, 1.0, 2.0):
            run(30., att_noise_deg=d, label=f'attitude noise {d} deg rms')
        for b in (1.0, 2.0):
            run(30., att_noise_deg=0.2, att_bias_deg=b, label=f'attitude BIAS {b} deg')
