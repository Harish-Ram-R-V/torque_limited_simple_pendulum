#!/usr/bin/env python3
"""
STEP 1: Check if CAN interface is working.
Run this FIRST before any motor test.

Usage:
    python3 01_check_can.py
"""

import subprocess
import sys


def check_can_interface():
    print("=" * 50)
    print("CAN Interface Check")
    print("=" * 50)

    # Check if can0 exists
    result = subprocess.run(["ip", "link", "show", "can0"],
                            capture_output=True, text=True)

    if result.returncode != 0:
        print("[FAIL] can0 not found!")
        print()
        print("Fix: Add these lines to /boot/firmware/config.txt and reboot:")
        print("  dtparam=spi=on")
        print("  dtoverlay=spi1-3cs")
        print("  dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23")
        sys.exit(1)

    print("[OK] can0 found")

    # Check if it's UP
    if "UP" in result.stdout:
        print("[OK] can0 is already UP")
    else:
        print("[INFO] can0 is DOWN — bringing it up at 1 Mbps...")
        up_result = subprocess.run(
            ["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"],
            capture_output=True, text=True
        )
        if up_result.returncode != 0:
            print(f"[FAIL] Could not bring up can0: {up_result.stderr}")
            sys.exit(1)
        print("[OK] can0 is now UP at 1 Mbps")

    print()
    print("Current can0 status:")
    subprocess.run(["ip", "-details", "link", "show", "can0"])
    print()
    print("All good! Proceed to 02_listen_can.py")


if __name__ == "__main__":
    check_can_interface()
