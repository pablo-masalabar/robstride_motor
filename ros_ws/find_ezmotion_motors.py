#!/usr/bin/env python3
"""Scan a SocketCAN bus for EZmotion PCN/SCN C2 series motors (CANopen DS402).

Probes each CANopen Node-ID (1–127) by sending an SDO upload request for
Device Type (object 0x1000) — a mandatory object in every CANopen device.
If the node responds, additional DS402 registers are read and reported.

Usage:
    python3 find_ezmotion_motors.py can0
    python3 find_ezmotion_motors.py can0 --min 1 --max 32 --timeout 0.05
"""

import argparse
import math
import struct
import sys
import os
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/ezmotion_p'))

from ezmotion_p.comms import EZMotionCANComms
from ezmotion_p.motor_base import (
    EZMotionMotorBase, OD, OperationMode, DriveState,
    _decode_drive_state, _counts_to_rad, COUNTS_PER_REV,
)

_SDO_READ_REQ = 0x40

_OP_MODE_NAMES = {m.value: m.name for m in OperationMode}


def _probe_node(comms: EZMotionCANComms, node_id: int, timeout: float):
    """Send SDO read for Device Type (0x1000:00) and wait for response.

    Returns raw 8-byte SDO frame or None on timeout.
    Registers a temporary dispatcher entry for the SDO response COB-ID.
    """
    sdo_rx_id = 0x580 + node_id
    event     = threading.Event()
    result    = [None]

    def _cb(msg):
        if len(msg.data) >= 8:
            result[0] = bytes(msg.data)
            event.set()

    comms._dispatcher.register([sdo_rx_id], _cb)

    req = bytes([_SDO_READ_REQ, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])
    comms.send(0x600 + node_id, req)
    event.wait(timeout=timeout)
    return result[0]


def scan(channel: str, node_min: int, node_max: int, timeout: float) -> list:
    comms = EZMotionCANComms(channel=channel, bitrate=1_000_000)
    comms.start_listener()
    found = []

    try:
        for node_id in range(node_min, node_max + 1):
            print(f'\rProbing Node-ID {node_id:3d}/{node_max} …', end='', flush=True)

            raw = _probe_node(comms, node_id, timeout)
            if raw is None:
                continue

            device_type = struct.unpack_from('<I', raw, 4)[0]

            # Motor found — create instance for detailed register reads.
            # EZMotionMotorBase.add_motor_callback overwrites the probe
            # listener with its own, which is fine since the probe is done.
            motor = EZMotionMotorBase(node_id=node_id, comms=comms, rx_timeout=timeout)

            statusword  = motor.read_sdo_u16(OD.STATUS_WORD,      0x00)
            op_mode     = motor.read_sdo_u8 (OD.MODES_OF_OP_DISP, 0x00)
            pos_counts  = motor.read_sdo_s32(OD.POS_ACTUAL,       0x00)
            temp        = motor.read_sdo_u16(OD.TEMPERATURE,       0x00)
            stored_nid  = motor.read_sdo_u8 (OD.CAN_NODE_ID,       0x00)

            drive_state  = _decode_drive_state(statusword) if statusword is not None else DriveState.UNKNOWN
            pos_rad      = _counts_to_rad(pos_counts)      if pos_counts  is not None else None

            sw_str      = f'0x{statusword:04X}'                     if statusword is not None else 'n/a'
            state_str   = drive_state.name
            mode_str    = _OP_MODE_NAMES.get(op_mode, str(op_mode)) if op_mode    is not None else 'n/a'
            pos_str     = f'{pos_rad:.4f} rad'                      if pos_rad    is not None else 'n/a'
            temp_str    = f'{temp} °C'                              if temp        is not None else 'n/a'
            nid_str     = str(stored_nid)                           if stored_nid is not None else 'n/a'

            print(
                f'\r  [FOUND] node_id={node_id}  '
                f'device_type=0x{device_type:08X}  '
                f'state={state_str}  sw={sw_str}  mode={mode_str}  '
                f'pos={pos_str}  temp={temp_str}  stored_node_id={nid_str}'
            )
            found.append(node_id)

    finally:
        comms.stop_listener()
        comms.close()

    return found


def main():
    parser = argparse.ArgumentParser(
        description='Scan a SocketCAN bus for EZmotion PCN/SCN C2 series motors'
    )
    parser.add_argument('channel',
                        help='SocketCAN interface name, e.g. can0')
    parser.add_argument('--min', type=int, default=1,
                        help='First Node-ID to probe (default: 1)')
    parser.add_argument('--max', type=int, default=127,
                        help='Last Node-ID to probe (default: 127)')
    parser.add_argument('--timeout', type=float, default=0.05,
                        help='Per-node SDO response timeout in seconds (default: 0.05)')
    args = parser.parse_args()

    print(f'Scanning {args.channel}  Node-IDs {args.min}–{args.max}  '
          f'timeout={args.timeout * 1000:.0f} ms per node')
    print()

    found = scan(args.channel, args.min, args.max, args.timeout)

    print(f'\nDone — {len(found)} motor(s) found: {found}')


if __name__ == '__main__':
    main()
