#!/usr/bin/env python3
"""Figures and a summary table from a Part 2 PX4 log.

    python3 plot_log.py flight.ulg                        # -> results/<log>/ (pngs + summary.md)
    python3 plot_log.py ref.ulg zero.ulg --compare --labels "built-in" "0 %"

Events are found in the log itself: every change of rotor_effectiveness.scale, every
'failure' injection (vehicle_command 420) and the moment the allocator removed a motor on
its own (control_allocator_status.handled_motor_failure_mask). Each event gets a figure and
a row in the table, measured from the event until the next one.
"""
import argparse
import os

import numpy as np

TOPICS = ['rotor_effectiveness', 'vehicle_local_position', 'vehicle_attitude',
          'vehicle_angular_velocity', 'actuator_motors', 'control_allocator_status',
          'vehicle_command']
VEHICLE_CMD_INJECT_FAILURE = 420


def load_ulog(path, rate=50.0):
    """Read the topics we need and resample them onto one 50 Hz time base (s since boot)."""
    from pyulog import ULog
    ulog = ULog(path, TOPICS)
    topics = {}
    for d in ulog.data_list:
        topics.setdefault(d.name, d.data)             # first instance (multi_id 0)
    att = topics['vehicle_attitude']
    t = np.arange(att['timestamp'][0], att['timestamp'][-1], 1e6 / rate) / 1e6

    def hold(topic, field, default=np.nan):
        """Zero-order hold: the latest sample at or before each t."""
        d = topics.get(topic)
        if d is None or field not in d:
            return np.full(len(t), default, dtype=float)
        i = np.searchsorted(d['timestamp'] / 1e6, t, side='right') - 1
        v = np.asarray(d[field], dtype=float)[np.clip(i, 0, None)]
        return np.where(i >= 0, v, default)

    qw, qx, qy, qz = (hold('vehicle_attitude', f'q[{i}]') for i in range(4))
    out = {
        't': t,
        'scale': hold('rotor_effectiveness', 'scale'),
        'column': np.column_stack([hold('rotor_effectiveness', f'effectiveness[{i}]') for i in range(6)]),
        'x': hold('vehicle_local_position', 'x'),
        'y': hold('vehicle_local_position', 'y'),
        'z': hold('vehicle_local_position', 'z'),
        'vz': hold('vehicle_local_position', 'vz'),
        'roll': np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)),
        'pitch': np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1)),
        'yaw': np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)),
        'p': hold('vehicle_angular_velocity', 'xyz[0]'),
        'q': hold('vehicle_angular_velocity', 'xyz[1]'),
        'r': hold('vehicle_angular_velocity', 'xyz[2]'),
        'u': np.column_stack([hold('actuator_motors', f'control[{i}]') for i in range(4)]),
    }
    out['tilt'] = np.arccos(np.clip(1 - 2 * (qx ** 2 + qy ** 2), -1, 1))

    # event times come from the raw samples, not the 50 Hz grid
    eff = topics.get('rotor_effectiveness')
    out['scale_steps'] = []
    if eff is not None:
        ts, sc, mo = eff['timestamp'] / 1e6, eff['scale'].astype(float), eff['motor'].astype(int)
        for i in np.flatnonzero(np.abs(np.diff(sc)) > 1e-3) + 1:
            out['scale_steps'].append((float(ts[i]), float(sc[i - 1]), float(sc[i]), int(mo[i])))
    cas = topics.get('control_allocator_status')
    mot = topics.get('actuator_motors')
    out['removals'] = []
    if cas is not None:
        mask = cas['handled_motor_failure_mask'].astype(int)
        for i in np.flatnonzero((mask[1:] != 0) & (mask[:-1] == 0)) + 1:
            tr, m = float(cas['timestamp'][i] / 1e6), int(np.log2(mask[i] & -mask[i]))
            # control_allocator_status is only logged at 5 Hz; the motor output turns NaN at
            # the moment of removal and is logged at full rate, so take the time from there
            if mot is not None and f'control[{m}]' in mot:
                ts, c = mot['timestamp'] / 1e6, mot[f'control[{m}]']
                k = np.flatnonzero((ts > tr - 0.25) & (ts <= tr) & np.isnan(c))
                if len(k):
                    tr = float(ts[k[0]])
            out['removals'].append((tr, m))
    cmd = topics.get('vehicle_command')
    if cmd is not None:
        inj = (cmd['command'] == VEHICLE_CMD_INJECT_FAILURE) & (np.rint(cmd['param2']) == 1)
        out['inject_t'] = cmd['timestamp'][inj] / 1e6
        out['inject_motor'] = np.rint(cmd['param3'][inj]).astype(int)
    else:
        out['inject_t'], out['inject_motor'] = np.array([]), np.array([], int)
    return out


def find_events(d):
    """[(t, label, scale, motor index)] sorted by time.

    scale is the new effectiveness (0 for a built-in removal) or None for an injection
    that the allocator never acted on. A removal within 2 s of an injection is folded
    into the injection event as its detection latency.
    """
    events = []
    for ts, old, new, motor in d['scale_steps']:
        events.append((ts, f'motor {motor} effectiveness {old:.0%} -> {new:.0%}', new, motor - 1))
    removals = list(d['removals'])
    for ti, m in zip(d['inject_t'], d['inject_motor']):
        label, scale = f'failure motor off -i {m}', None
        for tr, mr in list(removals):
            if mr == m - 1 and 0 <= tr - ti < 2.0:
                label += f', removed from allocation after {tr - ti:.2f} s'
                scale = 0.0
                removals.remove((tr, mr))
        if scale is None:
            label += ', allocator did not react'
        events.append((float(ti), label, scale, int(m) - 1))
    for tr, mr in removals:
        events.append((tr, f'built-in handling removed motor {mr + 1}', 0.0, mr))
    return sorted(events, key=lambda e: e[0])


def segment_metrics(d, t0, t1, motor=0):
    """What happened between t0 and t1, relative to the state at t0."""
    w = (d['t'] >= t0) & (d['t'] < t1)
    if not w.any():
        return None
    k0 = max(np.searchsorted(d['t'], t0) - 1, 0)
    z0 = d['z'][k0]
    tw = d['t'][w]
    tilt = np.degrees(d['tilt'][w])
    last = tw >= tw[-1] - 3.0
    ground = np.flatnonzero(d['z'][w] > -1.0) if z0 < -5.0 else np.array([], int)
    u = d['u'][w][:, motor]
    m = {
        'duration_s': float(tw[-1] - t0),
        'alt_loss_m': float(max(np.nanmax(d['z'][w] - z0), 0.0)),
        'max_tilt_deg': float(np.nanmax(tilt)),
        'max_yaw_rate_dps': float(np.nanmax(np.degrees(np.abs(d['r'][w])))),
        'end_tilt_deg': float(np.nanmean(tilt[last])),
        'end_alt_err_m': float(np.nanmean(d['z'][w][last]) - z0),
        'motor_u_end': float(np.nanmean(u[last])) if np.isfinite(u[last]).any() else float('nan'),
        'motor_sat_pct': float(100 * np.mean(u[np.isfinite(u)] >= 0.99)) if np.isfinite(u).any() else 0.0,
        't_ground_s': float(tw[ground[0]] - t0) if len(ground) else None,
    }
    if m['t_ground_s'] is not None:
        how = 'tumbled' if m['max_tilt_deg'] > 60 else 'came down upright'
        m['outcome'] = f'{how}, ground after {m["t_ground_s"]:.1f} s'
    elif m['end_tilt_deg'] < 5 and abs(m['end_alt_err_m']) < 2 and m['max_tilt_deg'] < 45:
        m['outcome'] = 'holds'
    else:
        m['outcome'] = 'airborne, not settled'
    return m


# --- figures -----------------------------------------------------------------
def _axes(n):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(n, 1, figsize=(10, 2.1 * n), sharex=True)
    return plt, fig, ax


def plot_window(d, t0, t1, title, path, events=()):
    plt, fig, ax = _axes(5)
    w = (d['t'] >= t0) & (d['t'] <= t1)
    t = d['t'][w]
    ax[0].step(t, 100 * d['scale'][w], where='post', color='k')
    ax[0].set_ylabel('effectiveness %')
    ax[0].set_ylim(-5, 105)
    ax[1].plot(t, -d['z'][w])
    ax[1].set_ylabel('altitude m')
    ax[2].plot(t, np.degrees(d['roll'][w]), label='roll')
    ax[2].plot(t, np.degrees(d['pitch'][w]), label='pitch')
    ax[2].plot(t, np.degrees(d['tilt'][w]), 'k:', label='tilt')
    ax[2].set_ylabel('deg')
    ax[2].legend(loc='upper left', fontsize=8, ncol=3)
    ax[3].plot(t, np.degrees(d['r'][w]))
    ax[3].set_ylabel('yaw rate deg/s')
    for i in range(4):
        ax[4].plot(t, d['u'][w][:, i], label=f'motor {i + 1}')
    ax[4].set_ylabel('motor cmd')
    ax[4].set_ylim(-0.05, 1.05)
    ax[4].legend(loc='upper left', fontsize=8, ncol=4)
    ax[4].set_xlabel('time since boot, s')
    for te, *_ in events:
        if t0 <= te <= t1:
            for a in ax:
                a.axvline(te, color='r', lw=0.8, alpha=0.6)
    for a in ax:
        a.grid(alpha=0.3)
    ax[0].set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_compare(runs, path, window=(-2.0, 15.0)):
    """runs: [(label, d, t_event, motor index)] overlaid with t = 0 at each event."""
    plt, fig, ax = _axes(4)
    for label, d, te, motor in runs:
        w = (d['t'] >= te + window[0]) & (d['t'] <= te + window[1])
        t = d['t'][w] - te
        k0 = max(np.searchsorted(d['t'], te) - 1, 0)
        ax[0].plot(t, -(d['z'][w] - d['z'][k0]), label=label)
        ax[1].plot(t, np.degrees(d['tilt'][w]), label=label)
        ax[2].plot(t, np.degrees(d['r'][w]), label=label)
        ax[3].plot(t, np.nan_to_num(d['u'][w][:, motor], nan=-0.02), label=label)
    for a, yl in zip(ax, ('altitude change m', 'tilt deg', 'yaw rate deg/s', 'affected motor cmd')):
        a.set_ylabel(yl)
        a.axvline(0, color='r', lw=0.8)
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    ax[-1].set_xlabel('time since event, s   (motor cmd -0.02 = stopped / NaN)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def summary_table(rows):
    head = ('| t (s) | event | alt loss (m) | max tilt (°) | max yaw rate (°/s) | '
            'motor cmd at end | motor at 100 % (% of time) | outcome |\n'
            '|---|---|---|---|---|---|---|---|\n')
    body = ''
    for te, label, m in rows:
        body += (f'| {te:.1f} | {label} | {m["alt_loss_m"]:.1f} | {m["max_tilt_deg"]:.0f} | '
                 f'{m["max_yaw_rate_dps"]:.0f} | {m["motor_u_end"]:.2f} | {m["motor_sat_pct"]:.0f} | '
                 f'{m["outcome"]} |\n')
    return head + body


def analyse(d, out_dir, name='log'):
    os.makedirs(out_dir, exist_ok=True)
    events = find_events(d)
    rows = []
    for i, (te, label, scale, motor) in enumerate(events):
        t1 = events[i + 1][0] if i + 1 < len(events) else d['t'][-1]
        m = segment_metrics(d, te, t1, max(motor, 0))
        if m is None:
            continue
        rows.append((te, label, m))
        plot_window(d, te - 3.0, min(t1, te + 30.0), label, os.path.join(out_dir, f'event_{i + 1:02d}.png'), events)
    plot_window(d, d['t'][0], d['t'][-1], f'{name}: whole flight', os.path.join(out_dir, 'overview.png'), events)
    table = summary_table(rows)
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write(f'# {name}\n\n{table}')
        for te, label, _ in rows:
            k = np.searchsorted(d['t'], te + 0.5)
            col = d['column'][min(k, len(d['t']) - 1)]
            if np.isfinite(col).all():
                f.write(f'\n- {te:.1f} s, {label}: allocator column (roll, pitch, yaw, Tx, Ty, Tz) = '
                        f'{np.array2string(col, precision=3, suppress_small=True)}')
        f.write('\n')
    return rows, table


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('logs', nargs='+', help='.ulg files')
    ap.add_argument('--out', default='results', help='output folder')
    ap.add_argument('--compare', action='store_true',
                    help='overlay the first event of each log instead of per-log analysis')
    ap.add_argument('--labels', nargs='+', help='legend labels for --compare')
    a = ap.parse_args()
    if a.compare:
        runs = []
        for i, path in enumerate(a.logs):
            d = load_ulog(path)
            ev = find_events(d)
            # complete loss: an injection, or the effectiveness going to 0
            loss = [e for e in ev if e[2] is None or e[2] == 0.0] or ev
            if not loss:
                print(f'{path}: no events found')
                continue
            label = a.labels[i] if a.labels and i < len(a.labels) else os.path.basename(path)
            runs.append((label, d, loss[0][0], max(loss[0][3], 0)))
        os.makedirs(a.out, exist_ok=True)
        plot_compare(runs, os.path.join(a.out, 'compare.png'))
        print(f'wrote {os.path.join(a.out, "compare.png")}')
        return
    for path in a.logs:
        name = os.path.splitext(os.path.basename(path))[0]
        rows, table = analyse(load_ulog(path), os.path.join(a.out, name), name)
        print(f'\n{name}\n{table}')


if __name__ == '__main__':
    main()
