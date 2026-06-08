#!/usr/bin/env python3

import pyrealsense2 as rs


def main():
    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("No RealSense devices found.")
        return

    print(f"Found {len(devices)} RealSense device(s):\n")
    for i, dev in enumerate(devices):
        name   = dev.get_info(rs.camera_info.name)
        serial = dev.get_info(rs.camera_info.serial_number)
        fw     = dev.get_info(rs.camera_info.firmware_version)
        usb    = dev.get_info(rs.camera_info.usb_type_descriptor)
        print(f"  [{i}] {name}")
        print(f"       serial  : {serial}")
        print(f"       firmware: {fw}")
        print(f"       usb     : {usb}")


if __name__ == "__main__":
    main()
