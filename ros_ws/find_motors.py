#!/usr/bin/env python3

import argparse
import time
import can


READ_PARAM_CAN_ID = 0x7FF
READ_CMD = 0x33

# Register 0x0E = firmware version, read-only.
# Reading it is a safe way to check if a motor exists.
RID_SW_VERSION = 0x0E


def make_read_param_msg(target_id: int, rid: int) -> can.Message:
    """
    Damiao read parameter frame:
    CAN ID: 0x7FF
    Data:
      D0 = CANID_L
      D1 = CANID_H
      D2 = 0x33
      D3 = RID
    """
    data = [
        target_id & 0xFF,
        (target_id >> 8) & 0xFF,
        READ_CMD,
        rid & 0xFF,
    ]

    return can.Message(
        arbitration_id=READ_PARAM_CAN_ID,
        data=data,
        is_extended_id=False,
    )


def parse_response(msg: can.Message, rid: int):
    """
    Expected response:
    CAN ID: MST_ID
    Data:
      D0 = CANID_L
      D1 = CANID_H
      D2 = 0x33
      D3 = RID
      D4-D7 = data
    """
    if msg.is_extended_id:
        return None

    if len(msg.data) < 4:
        return None

    if msg.data[2] != READ_CMD:
        return None

    if msg.data[3] != rid:
        return None

    motor_id = msg.data[0] | (msg.data[1] << 8)

    value = None
    if len(msg.data) >= 8:
        value = int.from_bytes(bytes(msg.data[4:8]), byteorder="little", signed=False)

    return motor_id, value, msg.arbitration_id


def scan_motor_ids(interface: str, start_id: int, end_id: int, timeout: float):
    detected = {}

    with can.interface.Bus(channel=interface, bustype="socketcan") as bus:
        print(f"Scanning Damiao motors on {interface} from ID {start_id} to {end_id}...")

        for motor_id in range(start_id, end_id + 1):
            msg = make_read_param_msg(motor_id, RID_SW_VERSION)

            try:
                bus.send(msg)
            except can.CanError as e:
                print(f"Failed to send CAN frame for ID {motor_id}: {e}")
                continue

            start_time = time.time()

            while time.time() - start_time < timeout:
                rx = bus.recv(timeout=timeout)

                if rx is None:
                    break

                parsed = parse_response(rx, RID_SW_VERSION)
                if parsed is None:
                    continue

                detected_id, fw_version, feedback_can_id = parsed

                if detected_id == motor_id:
                    detected[detected_id] = {
                        "firmware_version": fw_version,
                        "feedback_can_id": feedback_can_id,
                    }
                    print(
                        f"Motor detected: ID={detected_id} "
                        f"(0x{detected_id:X}), feedback CAN ID=0x{feedback_can_id:X}, "
                        f"firmware/register value={fw_version}"
                    )
                    break

        if not detected:
            print("No motor detected.")
        else:
            print("\nDetected motor IDs:")
            for motor_id in sorted(detected.keys()):
                print(f"  ID {motor_id} / 0x{motor_id:X}")


def main():
    parser = argparse.ArgumentParser(description="Find Damiao DM-J4310 motor CAN ID.")
    parser.add_argument("--interface", "-i", default="can0", help="SocketCAN interface, default: can0")
    parser.add_argument("--start-id", type=int, default=0, help="Start CAN ID, default: 0")
    parser.add_argument("--end-id", type=int, default=15, help="End CAN ID, default: 15")
    parser.add_argument("--timeout", type=float, default=0.05, help="Response timeout per ID in seconds")

    args = parser.parse_args()

    if args.start_id < 0 or args.end_id > 0x7FF or args.start_id > args.end_id:
        raise ValueError("Invalid ID range. Use 0 to 0x7FF.")

    scan_motor_ids(
        interface=args.interface,
        start_id=args.start_id,
        end_id=args.end_id,
        timeout=args.timeout,
    )


if __name__ == "__main__":
    main()
