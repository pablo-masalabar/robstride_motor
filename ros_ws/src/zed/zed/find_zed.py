#!/usr/bin/env python3

import pyzed.sl as sl


def main():
    devices = sl.Camera.get_device_list()

    if not devices:
        print("No ZED devices found.")
        return

    print(f"Found {len(devices)} ZED device(s):\n")
    for i, dev in enumerate(devices):
        print(f"  [{i}] {dev.camera_model}")
        print(f"       serial  : {dev.serial_number}")
        print(f"       state   : {dev.camera_state}")


if __name__ == "__main__":
    main()
