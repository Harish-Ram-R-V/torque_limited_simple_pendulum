#!/usr/bin/env python3
"""
STEP 4: Gentle position hold test with live feedback.

This holds the motor at its current zero position with a soft spring
(low Kp, moderate Kd). The motor will resist being moved by hand.

SAFE: Very low Kp=10, moderate Kd=1. Max torque is soft.
      The motor will NOT suddenly spin. It will just hold position.

Run this ONLY after 03_motor_enable.py worked successfully.

Usage:
    python3 04_position_hold.py

Press Ctrl+C to stop and disable motor safely.
"""

import can
import time
import signal
import sys

# ─── CONFIG ──────────────────────────────────────────────────────────────────
MOTOR_ID  = 0x01
CAN_IFACE = 'can0'
KP        = 10.0      # Spring stiffness — keep low for safety
KD        = 1.0       # Damping
TARGET_POS = 0.0      # Hold at zero position (rad)
TARGET_VEL = 0.0
TORQUE_FF  = 0.0      # No feedforward
LOOP_HZ    = 100      # Control loop frequency
# ─────────────────────────────────────────────────────────────────────────────

AK70_10_PARAMS = {
    "P_MIN": -12.5, "P_MAX": 12.5,
    "V_MIN": -50.0, "V_MAX": 50.0,
    "KP_MIN": 0.0,  "KP_MAX": 500.0,
    "KD_MIN": 0.0,  "KD_MAX": 5.0,
    "T_MIN": -24.8, "T_MAX": 24.8,
}

CMD_ENTER_MIT = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
CMD_EXIT_MIT  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


def uint_to_float(x_int, x_min, x_max, bits):
    return x_int * (x_max - x_min) / ((1 << bits) - 1) + x_min


def pack_command(pos, vel, kp, kd, t_ff, p=AK70_10_PARAMS):
    p_i  = float_to_uint(pos, p["P_MIN"],  p["P_MAX"],  16)
    v_i  = float_to_uint(vel, p["V_MIN"],  p["V_MAX"],  12)
    kp_i = float_to_uint(kp,  p["KP_MIN"], p["KP_MAX"], 12)
    kd_i = float_to_uint(kd,  p["KD_MIN"], p["KD_MAX"], 12)
    t_i  = float_to_uint(t_ff, p["T_MIN"], p["T_MAX"],  12)

    data = bytearray(8)
    data[0] = p_i >> 8
    data[1] = p_i & 0xFF
    data[2] = v_i >> 4
    data[3] = ((v_i & 0xF) << 4) | (kp_i >> 8)
    data[4] = kp_i & 0xFF
    data[5] = kd_i >> 4
    data[6] = ((kd_i & 0xF) << 4) | (t_i >> 8)
    data[7] = t_i & 0xFF
    return bytes(data)


def unpack_reply(data, p=AK70_10_PARAMS):
    if len(data) < 6:
        return None, None, None
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0xF) << 8) | data[5]
    pos    = uint_to_float(p_int, p["P_MIN"], p["P_MAX"], 16)
    vel    = uint_to_float(v_int, p["V_MIN"], p["V_MAX"], 12)
    torque = uint_to_float(t_int, p["T_MIN"], p["T_MAX"], 12)
    return pos, vel, torque


bus = None


def cleanup(sig=None, frame=None):
    """Safely exit motor mode on Ctrl+C or error."""
    global bus
    print("\n\nShutting down motor safely...")
    if bus:
        exit_msg = can.Message(arbitration_id=MOTOR_ID,
                               data=CMD_EXIT_MIT,
                               is_extended_id=False)
        try:
            bus.send(exit_msg)
            time.sleep(0.05)
        except Exception:
            pass
        bus.shutdown()
    print("Motor disabled. Bye!")
    sys.exit(0)


signal.signal(signal.SIGINT, cleanup)


def main():
    global bus

    print("=" * 55)
    print("AK70-10 Position Hold Test")
    print("=" * 55)
    print(f"Target position : {TARGET_POS} rad ({TARGET_POS*57.296:.1f} deg)")
    print(f"Kp={KP}  Kd={KD}  (low and safe)")
    print("Press Ctrl+C to stop and disable motor.\n")

    bus = can.interface.Bus(channel=CAN_IFACE, bustype='socketcan')

    # Enter MIT mode
    enter_msg = can.Message(arbitration_id=MOTOR_ID,
                            data=CMD_ENTER_MIT,
                            is_extended_id=False)
    bus.send(enter_msg)
    time.sleep(0.1)

    reply = bus.recv(timeout=0.5)
    if not reply:
        print("[FAIL] Motor did not respond to enter-MIT-mode command.")
        print("Check wiring and motor power.")
        bus.shutdown()
        return

    print("[OK] Motor in MIT mode. Starting control loop...\n")
    print(f"{'Time(s)':>8} | {'Pos(rad)':>10} | {'Vel(rad/s)':>10} | {'Torque(Nm)':>10}")
    print("-" * 48)

    dt = 1.0 / LOOP_HZ
    t_start = time.time()

    while True:
        t_now = time.time() - t_start

        # Send position hold command
        cmd = pack_command(TARGET_POS, TARGET_VEL, KP, KD, TORQUE_FF)
        msg = can.Message(arbitration_id=MOTOR_ID, data=cmd, is_extended_id=False)
        bus.send(msg)

        # Read feedback
        reply = bus.recv(timeout=dt)
        if reply:
            pos, vel, torque = unpack_reply(reply.data)
            if pos is not None:
                # Print at ~10 Hz to avoid flood
                if int(t_now * 10) % 1 == 0:
                    print(f"{t_now:>8.2f} | {pos:>10.4f} | {vel:>10.4f} | {torque:>10.4f}",
                          end='\r')

        time.sleep(max(0, dt - 0.001))


if __name__ == "__main__":
    main()
