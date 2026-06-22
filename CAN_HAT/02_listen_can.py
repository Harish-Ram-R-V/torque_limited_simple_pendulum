#!/usr/bin/env python3
"""
STEP 2: Listen for any CAN frames on the bus.

Power ON your AK70-10 motor FIRST, then run this.
If the motor is healthy and on the bus, you may see its status frames.

Usage:
    python3 02_listen_can.py

Press Ctrl+C to stop.
"""

import can
import time


def listen():
    print("=" * 50)
    print("CAN Bus Listener")
    print("=" * 50)
    print("Listening on can0 for 10 seconds...")
    print("Power ON the AK70-10 motor now if not already.")
    print("Press Ctrl+C to stop early.\n")

    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
    except Exception as e:
        print(f"[FAIL] Cannot open can0: {e}")
        print("Make sure you ran 01_check_can.py first.")
        return

    count = 0
    start = time.time()

    try:
        while time.time() - start < 10:
            msg = bus.recv(timeout=1.0)
            if msg:
                count += 1
                data_hex = ' '.join(f'{b:02X}' for b in msg.data)
                print(f"[Frame #{count}] ID=0x{msg.arbitration_id:03X}  "
                      f"DLC={msg.dlc}  Data: {data_hex}")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        bus.shutdown()

    print(f"\nTotal frames received: {count}")
    if count == 0:
        print("\n[WARNING] No frames received!")
        print("Possible reasons:")
        print("  1. Motor not powered on")
        print("  2. CAN-H / CAN-L wires swapped or not connected")
        print("  3. Missing 120R termination (check jumper on HAT)")
        print("  4. Wrong oscillator frequency in config.txt (try 12000000)")
    else:
        print("\n[OK] Motor is talking on the CAN bus!")
        print("Proceed to 03_motor_enable.py")


if __name__ == "__main__":
    listen()
