#!/usr/bin/env python3

import time
import can
import argparse
from src.can_utils import discover_node_ids
from src.configure import calibrate_motor

def main():
    parser = argparse.ArgumentParser(description = "Calibrate a specific ODrive node on the CAN bus.")
    parser.add_argument("-id", type = int, required = True, help = "Specify the node id to calibrate (e.g. -id 6)")
    args = parser.parse_args()

    # Get CAN bus object
    try:
        bus = can.interface.Bus("can0", bustype="socketcan")
    except Exception as e:
        print(f"[ERROR] Unable to open CAN bus: {e}")
        return

    # Discover available nodes on the bus.
    nodes = discover_node_ids(bus)
    if not nodes:
        print("[ERROR] No ODrives detected on the CAN bus.")
        bus.shutdown()
        return

    # Validate the provided node id.
    if args.id not in nodes:
        print(f"[ERROR] Specified node {args.id} not found on the bus. Available nodes: {nodes}")
        bus.shutdown()
        return

    # Prompt for user confirmation before calibrating.
    input(f"Ensure it is safe to proceed with calibration of node {args.id}. Press Enter to continue...")

    print(f"[INFO] Starting calibration for node {args.id}...")
    if calibrate_motor(bus, args.id):
        print(f"[INFO] Calibration successful for node {args.id}.")
    else:
        print(f"[ERROR] Calibration failed for node {args.id}.")

    bus.shutdown()

if __name__ == "__main__":
    main()