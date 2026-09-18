#!/usr/bin/env python3
"""Fly the Part 2 experiments against PX4 SITL over MAVLink.

    python3 fly.py reference                  # Phase 1: hover at 20 m, then 'failure motor off'
    python3 fly.py sweep                      # Phase 2: 100 -> 75 -> 50 -> 25 -> 0 %
    python3 fly.py sweep --levels 0.5 0.25    # only some levels
    python3 fly.py sweep --staircase          # step straight down, no return to 100 % between

In a sweep every level starts from a settled 20 m hover: after each level the scale goes
back to 100 % and the script waits for the vehicle to settle before the next one. That is
a fixed test schedule, not a reaction to what the vehicle does - nothing here tries to
detect or recover from the degradation. If the vehicle ends up on the ground the run stops
and prints the levels that are left; restart PX4 and run those on their own.

The PX4 log (.ulg) is the main record. This script also prints a status line twice a
second and writes the MAVLink telemetry plus every transition to a CSV.
"""
import argparse
import csv
import math
import os
import struct
import sys
import time

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402

mav = mavutil.mavlink

# not in every pymavlink dialect, values from MAVLink common.xml
MAV_CMD_INJECT_FAILURE = 420
FAILURE_UNIT_SYSTEM_MOTOR = 101
FAILURE_TYPE_OK = 0
FAILURE_TYPE_OFF = 1


class Vehicle:
    def __init__(self, url, csv_path):
        print(f'connecting to {url} ...')
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        hb = self.m.wait_heartbeat(timeout=60)
        if hb is None:
            sys.exit('no heartbeat - is PX4 running? (tools/run_sitl.sh)')
        print(f'heartbeat from system {self.m.target_system}')
        self.last = {}          # latest message of each type
        self.acks = {}          # command id -> MAV_RESULT
        self.params = {}        # param name -> PARAM_VALUE
        self.event = 'start'
        self.scale = float('nan')
        self._last_hb = self._last_print = 0.0
        self._csv_file = open(csv_path, 'w', newline='')
        self.csv = csv.writer(self._csv_file)
        self.csv.writerow(['t_boot_s', 'event', 'scale', 'alt_m', 'vz_mps', 'roll_deg',
                           'pitch_deg', 'yaw_deg', 'p_dps', 'q_dps', 'r_dps',
                           'esc1', 'esc2', 'esc3', 'esc4', 'mode', 'armed'])
        for msg_id, hz in ((mav.MAVLINK_MSG_ID_ATTITUDE, 50), (mav.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20),
                           (mav.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20), (mav.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 5)):
            self.command(mav.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz, wait_ack=False)

    # --- telemetry -----------------------------------------------------------
    def pump(self, duration=0.0):
        """Handle incoming messages for `duration` seconds (at least one pass)."""
        end = time.monotonic() + duration
        while True:
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self._handle(msg)
            now = time.monotonic()
            if now - self._last_hb > 1.0:
                self.m.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self._last_hb = now
            if now - self._last_print > 0.5:
                self._print_status()
                self._last_print = now
            if now >= end:
                return

    def _handle(self, msg):
        t = msg.get_type()
        if t == 'BAD_DATA' or (t == 'HEARTBEAT' and msg.get_srcComponent() != 1):
            return                                   # only the autopilot's heartbeat
        self.last[t] = msg
        if t == 'COMMAND_ACK':
            self.acks[msg.command] = msg.result
        elif t == 'PARAM_VALUE':
            self.params[msg.param_id] = msg
        elif t == 'STATUSTEXT':
            print(f'\n  PX4: {msg.text}')
        elif t == 'ATTITUDE':
            self._log_row()

    def _log_row(self):
        a, p = self.last.get('ATTITUDE'), self.last.get('LOCAL_POSITION_NED')
        s = self.last.get('SERVO_OUTPUT_RAW')
        esc = [getattr(s, f'servo{i}_raw', '') for i in range(1, 5)] if s else [''] * 4
        self.csv.writerow([f'{a.time_boot_ms / 1000:.3f}', self.event, self.scale,
                           f'{-p.z:.3f}' if p else '', f'{p.vz:.3f}' if p else '',
                           *(f'{math.degrees(v):.2f}' for v in (a.roll, a.pitch, a.yaw,
                                                                a.rollspeed, a.pitchspeed, a.yawspeed)),
                           *esc, self.mode, int(self.armed)])

    def _print_status(self):
        a, p = self.last.get('ATTITUDE'), self.last.get('LOCAL_POSITION_NED')
        if not a or not p:
            return
        s = self.last.get('SERVO_OUTPUT_RAW')
        esc = ' '.join(f'{getattr(s, f"servo{i}_raw"):4d}' for i in range(1, 5)) if s else '-'
        eff = f'{self.scale:4.0%}' if not math.isnan(self.scale) else '  - '
        sys.stdout.write(
            f'\r[{a.time_boot_ms / 1000:7.1f}s] eff {eff} | alt {-p.z:5.1f} m vz {p.vz:+5.1f} | '
            f'roll {math.degrees(a.roll):+6.1f} pitch {math.degrees(a.pitch):+6.1f} '
            f'yawrate {math.degrees(a.yawspeed):+6.0f} deg/s | esc {esc} | {self.mode:<12}')
        sys.stdout.flush()

    @property
    def alt(self):
        p = self.last.get('LOCAL_POSITION_NED')
        return -p.z if p else float('nan')

    @property
    def armed(self):
        hb = self.last.get('HEARTBEAT')
        return bool(hb and hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def mode(self):
        hb = self.last.get('HEARTBEAT')
        return mavutil.mode_string_v10(hb) if hb else '?'

    @property
    def on_ground(self):
        # only asked after takeoff, so a low altitude also means it came down
        ext = self.last.get('EXTENDED_SYS_STATE')
        landed = ext is not None and ext.landed_state == mav.MAV_LANDED_STATE_ON_GROUND
        return not self.armed or landed or self.alt < 1.0

    def tilt_deg(self):
        a = self.last.get('ATTITUDE')
        return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(a.roll) * math.cos(a.pitch))))) if a else 90.0

    # --- commands and parameters -------------------------------------------
    def command(self, cmd, *params, wait_ack=True, timeout=3.0):
        p = list(params) + [0.0] * (7 - len(params))
        self.acks.pop(cmd, None)
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
        if not wait_ack:
            return True
        end = time.monotonic() + timeout
        while time.monotonic() < end and cmd not in self.acks:
            self.pump(0.05)
        return self.acks.get(cmd) == mav.MAV_RESULT_ACCEPTED

    def set_param(self, name, value, is_int=False):
        """PARAM_SET with PX4's bytewise encoding for integers; returns the value read back."""
        if is_int:
            raw, ptype = struct.unpack('<f', struct.pack('<i', int(value)))[0], mav.MAV_PARAM_TYPE_INT32
        else:
            raw, ptype = float(value), mav.MAV_PARAM_TYPE_REAL32
        for _ in range(3):
            self.params.pop(name, None)
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                      name.encode(), raw, ptype)
            end = time.monotonic() + 2.0
            while time.monotonic() < end and name not in self.params:
                self.pump(0.05)
            msg = self.params.get(name)
            if msg is not None:
                if msg.param_type == mav.MAV_PARAM_TYPE_INT32:
                    return struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                return msg.param_value
        sys.exit(f'\nno reply setting {name} - does this PX4 build have the Part 2 patch?')

    def mark(self, event, scale=None):
        """Record a transition in the CSV and on the console."""
        self.event = event
        if scale is not None:
            self.scale = scale
        a = self.last.get('ATTITUDE')
        stamp = f'{a.time_boot_ms / 1000:.2f}s' if a else '?'
        print(f'\n>>> [{stamp}] {event}')
        if a:
            self._log_row()

    # --- flight phases ---------------------------------------------------------
    def wait_until(self, cond, timeout, what):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump(0.1)
            if cond():
                return True
        print(f'\n!!! timed out waiting for {what}')
        return False

    def wait_settled(self, alt, hold=5.0, timeout=60.0):
        """Altitude within 0.5 m and tilt under 5 deg for `hold` seconds."""
        since = None
        end = time.monotonic() + timeout
        while time.monotonic() < end and not self.on_ground:
            self.pump(0.1)
            ok = abs(self.alt - alt) < 0.5 and self.tilt_deg() < 5.0
            since = (since or time.monotonic()) if ok else None
            if since and time.monotonic() - since >= hold:
                return True
        return False

    def takeoff(self, alt):
        self.set_param('MIS_TAKEOFF_ALT', alt)
        self.set_param('NAV_DLL_ACT', 0, is_int=True)   # no RTL when this script disconnects
        print('arming (retries until the estimator is ready) ...')
        end = time.monotonic() + 90
        while not self.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1):
            if time.monotonic() > end:
                sys.exit('could not arm')
            self.pump(2.0)
        if not self.command(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan):
            sys.exit('takeoff command rejected')
        self.mark(f'takeoff to {alt:.0f} m')
        if not self.wait_until(lambda: self.alt > alt - 0.5, 60, 'climb'):
            sys.exit('takeoff failed')
        if not self.wait_settled(alt):
            sys.exit('did not settle at takeoff altitude')
        self.mark(f'hover {alt:.0f} m settled')

    def land(self):
        if not self.on_ground:
            self.mark('land')
            self.command(mav.MAV_CMD_NAV_LAND)
            self.wait_until(lambda: self.on_ground, 90, 'landing')

    def observe(self, seconds):
        """Fly on for `seconds`; returns False if the vehicle is on the ground by then."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pump(0.1)
            if self.on_ground:
                self.mark('vehicle on the ground')
                return False
        return True

    def restore(self, motor):
        """On the way out, whatever happened: scale back to 100 %, clear an injected failure.

        Both would otherwise stick for the rest of the PX4 session (an injected motor-off even
        across disarm), and the next run would start degraded.
        """
        try:
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component,
                                      b'CA_EFF_SCALE', 1.0, mav.MAV_PARAM_TYPE_REAL32)
            self.command(MAV_CMD_INJECT_FAILURE, FAILURE_UNIT_SYSTEM_MOTOR, FAILURE_TYPE_OK, motor,
                         wait_ack=False)
            self.pump(1.0)
        except Exception as e:                      # never mask the original error
            print(f'\ncould not restore CA_EFF_SCALE / motor state: {e}')

    def close(self):
        print()
        self._csv_file.close()


def run_reference(v, a):
    # PX4's own handling: detection (FD_ACT_EN) -> remove motor from allocation -> stop it
    v.set_param('CA_EFF_SCALE', 1.0)                    # start from a neutral allocator
    v.set_param('CA_EFF_MOTOR', 0, is_int=True)
    v.set_param('CA_FAILURE_MODE', 1, is_int=True)
    v.set_param('COM_ACT_FAIL_ACT', 0, is_int=True)     # warn only, no mode switch
    if v.set_param('SYS_FAILURE_EN', 1, is_int=True) != 1:
        sys.exit('SYS_FAILURE_EN could not be set')
    v.takeoff(a.alt)
    v.observe(a.baseline)
    v.mark(f'inject: failure motor off -i {a.motor}')
    if not v.command(MAV_CMD_INJECT_FAILURE, FAILURE_UNIT_SYSTEM_MOTOR, FAILURE_TYPE_OFF, a.motor):
        print('\n!!! no ACK for the injection (SYS_FAILURE_EN must be 1); watching anyway')
    if v.observe(a.dwell):
        v.mark(f'inject: failure motor ok -i {a.motor}')
        v.command(MAV_CMD_INJECT_FAILURE, FAILURE_UNIT_SYSTEM_MOTOR, FAILURE_TYPE_OK, a.motor)
        v.observe(5)
        v.land()


def run_sweep(v, a):
    levels = [float(x) for x in a.levels]
    if any(not 0.0 <= x <= 1.0 for x in levels):
        sys.exit('levels must be within 0..1')
    v.set_param('CA_EFF_SCALE', 1.0)
    v.set_param('CA_EFF_MOTOR', a.motor, is_int=True)
    v.takeoff(a.alt)
    v.mark(f'motor {a.motor} at 100 %', 1.0)
    v.observe(a.baseline)
    for i, level in enumerate(levels):
        got = v.set_param('CA_EFF_SCALE', level)
        v.mark(f'motor {a.motor} effectiveness -> {level:.0%} (PX4 reads back {got:.2f})', level)
        flying = v.observe(a.dwell)
        if flying and not a.staircase and level < 1.0:
            v.set_param('CA_EFF_SCALE', 1.0)
            v.mark(f'motor {a.motor} back to 100 %', 1.0)
            flying = v.wait_settled(a.alt, timeout=a.settle)
            if not flying and not v.on_ground:
                print('\n!!! did not settle back at the hover altitude, stopping here')
                break
        if not flying:
            left = levels[i + 1:]
            if left:
                print(f'\nnot flying any more; levels not flown: {left}. Restart PX4 and run:'
                      f'\n  python3 fly.py sweep --levels {" ".join(f"{x:g}" for x in left)}')
            return
    if a.staircase:
        v.set_param('CA_EFF_SCALE', 1.0)
        v.mark(f'motor {a.motor} back to 100 %', 1.0)
    v.observe(3)
    v.land()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', choices=['reference', 'sweep'])
    ap.add_argument('--url', default='udpin:0.0.0.0:14540',
                    help='MAVLink endpoint (PX4 SITL instance N sends to 14540+N)')
    ap.add_argument('--motor', type=int, default=1, help='motor number, 1-based (default 1)')
    ap.add_argument('--levels', nargs='+', default=['1.0', '0.75', '0.5', '0.25', '0.0'],
                    help='effectiveness levels 0..1, in order')
    ap.add_argument('--alt', type=float, default=20.0, help='hover altitude, m')
    ap.add_argument('--baseline', type=float, default=10.0, help='seconds of normal hover first')
    ap.add_argument('--dwell', type=float, default=20.0, help='seconds at each level / after injection')
    ap.add_argument('--settle', type=float, default=45.0, help='max seconds to settle between levels')
    ap.add_argument('--staircase', action='store_true', help='do not return to 100 %% between levels')
    ap.add_argument('--csv', help='telemetry CSV (default: <mode>_<time>.csv)')
    a = ap.parse_args()
    v = Vehicle(a.url, a.csv or f'{a.mode}_{time.strftime("%Y%m%d_%H%M%S")}.csv')
    try:
        (run_reference if a.mode == 'reference' else run_sweep)(v, a)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        v.restore(a.motor)
        v.close()
    print('done. The PX4 log is in PX4-Autopilot/build/px4_sitl_default/rootfs/log/<date>/')


if __name__ == '__main__':
    main()
