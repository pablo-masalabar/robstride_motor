#!/usr/bin/env python3
"""Scan a SocketCAN bus for RobStride motors.

Usage:
    python3 find_motors.py can0
    python3 find_motors.py can0 --min 0 --max 127 --timeout 0.05
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/robstride_p'))

from robstride_p.comms import CANComms
from robstride_p.motor_base import RobStrideMotorBase, ParamIndex

# Infer model from factory-default LIMIT_CUR.
# RS01 and RS02 share the same current limit — only V_MAX differs (needs motion to measure).
# Values are approximate; unreliable if the user has already changed LIMIT_CUR.
_CURRENT_TO_MODEL = [
    (15,  'RS05'),   # factory ~11 A
    (35,  'RS01/RS02'),  # factory ~23 A
    (60,  'RS03'),   # factory ~43 A
    (float('inf'), 'RS04'),  # factory ~90 A
]


def _guess_model(current_limit_a: float) -> str:
    for threshold, model in _CURRENT_TO_MODEL:
        if current_limit_a <= threshold:
            return model
    return 'unknown'


def scan(channel: str, id_min: int, id_max: int, timeout: float) -> list:
    comms = CANComms(channel=channel, bitrate=1_000_000)
    comms.start_listener()
    found = []

    try:
        for motor_id in range(id_min, id_max + 1):
            print(f'\rScanning ID {motor_id:3d}/{id_max} …', end='', flush=True)

            motor = RobStrideMotorBase(motor_id=motor_id, comms=comms, rx_timeout=timeout)

            # GET_DEVICE_ID (type-0) is the lightest possible query
            device_id = motor.get_device_id()

            if device_id is not None:
                mech_pos    = motor.read_param_float(ParamIndex.MECH_POS)
                limit_cur   = motor.read_param_float(ParamIndex.LIMIT_CUR)
                pos_str     = f'{mech_pos:.4f} rad' if mech_pos is not None else 'n/a'
                model_guess = _guess_model(limit_cur) if limit_cur is not None else 'unknown'
                cur_str     = f'{limit_cur:.1f} A' if limit_cur is not None else 'n/a'
                print(f'\r  [FOUND] motor_id={motor_id:3d}  '
                      f'model≈{model_guess}  limit_cur={cur_str}  '
                      f'mech_pos={pos_str}  device_id={device_id.hex()}')
                found.append(motor_id)

    finally:
        comms.stop_listener()
        comms.close()

    return found


def main():
    parser = argparse.ArgumentParser(
        description='Scan a SocketCAN bus for RobStride motors'
    )
    parser.add_argument('channel',
                        help='SocketCAN interface name, e.g. can0')
    parser.add_argument('--min', type=int, default=0,
                        help='First motor ID to scan (default: 0)')
    parser.add_argument('--max', type=int, default=127,
                        help='Last motor ID to scan (default: 127)')
    parser.add_argument('--timeout', type=float, default=0.05,
                        help='Per-motor response timeout in seconds (default: 0.05)')
    args = parser.parse_args()

    print(f'Scanning {args.channel}  IDs {args.min}–{args.max}  '
          f'timeout={args.timeout * 1000:.0f} ms per motor')
    print()

    found = scan(args.channel, args.min, args.max, args.timeout)

    print(f'\nDone — {len(found)} motor(s) found: {found}')


if __name__ == '__main__':
    main()
