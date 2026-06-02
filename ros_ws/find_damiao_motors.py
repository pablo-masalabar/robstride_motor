#!/usr/bin/env python3
"""Scan a SocketCAN bus for Damiao DM-J43xx motors.

Probes each motor ID (0–15) by reading the SW_VER register.
Uses DamiaoCANComms + DamiaoMotorBase — the same classes as the motor node.

master_id=None (promiscuous mode) is used so motors with non-default MST_ID
register values are detected regardless of what arbitration ID they reply on.
The actual MST_ID is captured from the reply frame and reported.

Usage:
    python3 find_damiao_motors.py can0
    python3 find_damiao_motors.py can0 --timeout 0.05
"""

import argparse
import threading
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/damiao_p'))

from damiao_p.comms import DamiaoCANComms
from damiao_p.motor_base import DamiaoMotorBase, RegAddr

DAMIAO_ID_MIN = 0
DAMIAO_ID_MAX = 15

_CTRL_MODE_NAMES = {1: 'MIT', 2: 'POS_VEL', 3: 'VELOCITY', 4: 'FORCE_POS'}

_GR_TO_MODEL = [
    (8.0, 15.0, 'J4310-2EC (10:1)'),
]


def _guess_model(gr: float) -> str:
    for lo, hi, name in _GR_TO_MODEL:
        if lo <= gr <= hi:
            return name
    return f'unknown (GR={gr:.1f})'


def _probe_mst_id(comms: DamiaoCANComms, motor_id: int, timeout: float):
    """
    Send a SW_VER read and capture the arbitration_id of the reply.
    Returns the MST_ID (arbitration_id of the reply frame) or None on timeout.
    This runs before DamiaoMotorBase takes over, so we can report the real MST_ID.
    """
    import struct, can as _can
    event   = threading.Event()
    mst_id  = [None]

    def listener(msg):
        if msg.is_extended_id:
            return
        d = msg.data
        if len(d) < 8:
            return
        if d[2] != 0x33:
            return
        if d[3] != (int(RegAddr.SW_VER) & 0xFF):
            return
        if (d[0] | (d[1] << 8)) != motor_id:
            return
        mst_id[0] = msg.arbitration_id
        event.set()

    comms._dispatcher.register(listener)
    request = bytes([motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, int(RegAddr.SW_VER) & 0xFF])
    comms.send(0x7FF, request)
    event.wait(timeout=timeout)
    return mst_id[0]


def scan(channel: str, timeout: float) -> list:
    # master_id=None → promiscuous mode: dispatcher accepts replies from ANY
    # arbitration ID so motors with non-default MST_ID registers are detected.
    comms = DamiaoCANComms(
        channel=channel,
        bustype='socketcan',
        bitrate=1_000_000,
        master_id=None,
        rx_timeout=timeout,
    )
    comms.start_listener()

    found = []
    try:
        for motor_id in range(DAMIAO_ID_MIN, DAMIAO_ID_MAX + 1):
            print(f'\rProbing ID {motor_id:2d}/{DAMIAO_ID_MAX} …', end='', flush=True)

            # First pass: lightweight probe that also captures the real MST_ID
            actual_mst_id = _probe_mst_id(comms, motor_id, timeout)
            if actual_mst_id is None:
                continue

            # Motor found — create a proper instance for param reads
            motor = DamiaoMotorBase(
                motor_id=motor_id,
                master_id=actual_mst_id,
                comms=comms,
                rx_timeout=timeout,
            )

            gr_val   = motor.read_param_float(RegAddr.GR)
            imax     = motor.read_param_float(RegAddr.IMAX)
            vbus     = motor.read_param_float(RegAddr.VBUS)
            sw_ver   = motor.read_param_uint(RegAddr.SW_VER)
            ctrl_raw = motor.read_param_uint(RegAddr.CTRL_MODE)

            model    = _guess_model(gr_val) if gr_val is not None else 'unknown'
            gr_str   = f'{gr_val:.1f}:1'   if gr_val  is not None else 'n/a'
            imax_str = f'{imax:.1f} A'      if imax    is not None else 'n/a'
            vbus_str = f'{vbus:.2f} V'      if vbus    is not None else 'n/a'
            fw_str   = str(sw_ver)          if sw_ver  is not None else 'n/a'
            mode_str = _CTRL_MODE_NAMES.get(ctrl_raw, str(ctrl_raw)) if ctrl_raw is not None else 'n/a'

            print(
                f'\r  [FOUND] motor_id={motor_id}  mst_id=0x{actual_mst_id:02X}  '
                f'model≈{model}  fw={fw_str}  GR={gr_str}  '
                f'Imax={imax_str}  Vbus={vbus_str}  mode={mode_str}'
            )
            found.append({'motor_id': motor_id, 'mst_id': actual_mst_id})

    finally:
        comms.stop_listener()
        comms.close()

    return found


def main():
    parser = argparse.ArgumentParser(
        description='Scan a SocketCAN bus for Damiao DM-J43xx motors'
    )
    parser.add_argument('channel',
                        help='SocketCAN interface name, e.g. can0')
    parser.add_argument('--timeout', type=float, default=0.05,
                        help='Per-motor response timeout in seconds (default: 0.05)')
    args = parser.parse_args()

    print(f'Scanning {args.channel}  IDs {DAMIAO_ID_MIN}–{DAMIAO_ID_MAX}  '
          f'timeout={args.timeout * 1000:.0f} ms per motor  (promiscuous — any MST_ID)')
    print()

    found = scan(args.channel, args.timeout)

    print(f'\nDone — {len(found)} motor(s) found:')
    for m in found:
        print(f'  motor_id={m["motor_id"]}  mst_id=0x{m["mst_id"]:02X}')


if __name__ == '__main__':
    main()
