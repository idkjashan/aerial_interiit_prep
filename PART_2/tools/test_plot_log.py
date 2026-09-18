"""plot_log.py against small synthetic .ulg files.  Run: pytest -q PART_2/tools"""
import math
import struct

import numpy as np

import plot_log

CODES = {'uint64_t': 'Q', 'uint32_t': 'I', 'uint16_t': 'H', 'uint8_t': 'B', 'float': 'f', 'bool': '?'}


def write_ulog(path, topics):
    """Minimal ULog writer: topics = {name: ([(type, count, field)], [row, ...])}."""
    with open(path, 'wb') as f:
        def msg(kind, data):
            f.write(struct.pack('<HB', len(data), ord(kind)) + data)
        f.write(b'ULog\x01\x12\x35\x01' + struct.pack('<Q', 0))
        msg('B', bytes(40))
        for name, (fields, _) in topics.items():
            spec = ''.join(f'{t}[{n}] {fld};' if n else f'{t} {fld};' for t, n, fld in fields)
            msg('F', f'{name}:{spec}'.encode())
        for i, name in enumerate(topics):
            msg('A', struct.pack('<BH', 0, i) + name.encode())
        for i, (fields, rows) in enumerate(topics.values()):
            fmt = '<' + ''.join(f'{n or 1}{CODES[t]}' for t, n, _ in fields)
            for row in rows:
                msg('D', struct.pack('<H', i) + struct.pack(fmt, *row))


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def flight(path, kind):
    """20 s at 20 m; at t=8 s either a 50 % step that settles, or an injection that tumbles."""
    t = np.arange(0.0, 20.0, 0.01)
    after = np.clip(t - 8.0, 0, None)
    if kind == 'scale50':
        roll = np.radians(8.0) * np.exp(-after / 0.7) * (t >= 8)
        z = np.full_like(t, -20.0)
        yaw_rate = np.radians(15.0) * np.exp(-after / 0.7) * (t >= 8)
        u0 = np.where(t >= 8, 0.73 + 0.27 * np.exp(-after / 0.5), 0.73)
    else:
        roll = np.minimum(np.radians(200.0) * after, np.radians(175.0))
        z = np.minimum(-20.0 + 0.5 * 9.81 * after ** 2, 0.0)
        yaw_rate = np.radians(170.0) * np.minimum(after, 1.0)
        u0 = np.where(t >= 8.03, np.nan, 0.73)
    us = lambda x: int(1e6 + x * 1e6)          # noqa: E731  (boot at 1 s)
    att = [(us(ti), *quat(r, 0.0, 0.0)) for ti, r in zip(t, roll)]
    lpos = [(us(ti), 0.0, 0.0, zi, 0.0, 0.0, 0.0) for ti, zi in zip(t, z)]
    angv = [(us(ti), 0.0, 0.0, r) for ti, r in zip(t, yaw_rate)]
    mot = [(us(ti), u, 0.73, 0.73, 0.73, *[math.nan] * 8) for ti, u in zip(t, u0)]
    col = [-1.43, 0.845, 0.325, 0.0, 0.0, -6.5]
    if kind == 'scale50':
        eff = [(us(ti), 1, 1.0 if ti < 8 else 0.5, *[c * (1.0 if ti < 8 else 0.5) for c in col])
               for ti in np.arange(0, 20, 1.0)] + [(us(8.0), 1, 0.5, *[c * 0.5 for c in col])]
        eff.sort()
        cmd, cas = [], [(us(0.0), 0)]
    else:
        eff = [(us(ti), 0, 1.0, *[0.0] * 6) for ti in np.arange(0, 20, 1.0)]
        cmd = [(us(8.0), 420, 101.0, 1.0, 1.0)]
        cas = [(us(0.0), 0), (us(8.2), 1)]           # 5 Hz status lags the removal
    write_ulog(path, {
        'vehicle_attitude': ([('uint64_t', 0, 'timestamp'), ('float', 4, 'q')], att),
        'vehicle_local_position': ([('uint64_t', 0, 'timestamp')] +
                                   [('float', 0, f) for f in ('x', 'y', 'z', 'vx', 'vy', 'vz')], lpos),
        'vehicle_angular_velocity': ([('uint64_t', 0, 'timestamp'), ('float', 3, 'xyz')], angv),
        'actuator_motors': ([('uint64_t', 0, 'timestamp'), ('float', 12, 'control')], mot),
        'rotor_effectiveness': ([('uint64_t', 0, 'timestamp'), ('uint8_t', 0, 'motor'),
                                 ('float', 0, 'scale'), ('float', 6, 'effectiveness')], eff),
        'vehicle_command': ([('uint64_t', 0, 'timestamp'), ('uint32_t', 0, 'command'),
                             ('float', 0, 'param1'), ('float', 0, 'param2'), ('float', 0, 'param3')], cmd),
        'control_allocator_status': ([('uint64_t', 0, 'timestamp'),
                                      ('uint16_t', 0, 'handled_motor_failure_mask')], cas),
    })


def test_scale_step_is_found_and_holds(tmp_path):
    path = tmp_path / 'scale50.ulg'
    flight(path, 'scale50')
    d = plot_log.load_ulog(str(path))
    events = plot_log.find_events(d)
    assert len(events) == 1
    te, label, scale, motor = events[0]
    assert abs(te - 9.0) < 1e-6 and scale == 0.5 and motor == 0 and '100% -> 50%' in label
    m = plot_log.segment_metrics(d, te, d['t'][-1], motor)
    assert m['outcome'] == 'holds'
    assert 7.0 < m['max_tilt_deg'] < 9.0
    rows, _ = plot_log.analyse(d, str(tmp_path / 'out'), 'scale50')
    assert (tmp_path / 'out' / 'event_01.png').exists()
    assert 'allocator column' in (tmp_path / 'out' / 'summary.md').read_text()


def test_injection_and_builtin_removal_become_one_event(tmp_path):
    path = tmp_path / 'ref.ulg'
    flight(path, 'builtin')
    d = plot_log.load_ulog(str(path))
    events = plot_log.find_events(d)
    assert len(events) == 1
    te, label, scale, motor = events[0]
    assert 'removed from allocation after 0.03 s' in label and scale == 0.0
    m = plot_log.segment_metrics(d, te, d['t'][-1], motor)
    assert m['outcome'].startswith('tumbled') and 1.5 < m['t_ground_s'] < 2.5
    assert np.isnan(m['motor_u_end'])                 # stopped motor logs NaN


def test_compare_figure(tmp_path):
    a, b = tmp_path / 'ref.ulg', tmp_path / 's.ulg'
    flight(a, 'builtin')
    flight(b, 'scale50')
    runs = []
    for p in (a, b):
        d = plot_log.load_ulog(str(p))
        e = plot_log.find_events(d)[0]
        runs.append((p.name, d, e[0], e[3]))
    plot_log.plot_compare(runs, str(tmp_path / 'compare.png'))
    assert (tmp_path / 'compare.png').stat().st_size > 10000
