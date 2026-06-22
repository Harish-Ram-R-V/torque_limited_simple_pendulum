#!/usr/bin/env python3
"""
STEP 3: Enable AK70-10 in MIT mode and read encoder feedback.

This sends the MIT "enter motor mode" command, then sends a zero-torque
command and reads back position, velocity, and torque.

SAFE: torque feedforward = 0, Kp = 0, Kd = 0.
      The motor will just sit there and report its state.

Usage:
    python3 03_motor_enable.py

Set MOTOR_ID to match the CAN ID on the label of your motor (usually 1).
"""

import can
import time
import struct

# ─── CONFIG ──────────────────────────────────────────────────────────────────
MOTOR_ID   = 0x01       # CAN ID of your AK70-10 (check label on motor body)
CAN_IFACE  = 'can0'
# ─────────────────────────────────────────────────────────────────────────────

# AK70-10 MIT mode parameter limits
AK70_10_PARAMS = {
    "P_MIN"  : -12.5,    # rad
    "P_MAX"  :  12.5,
    "V_MIN"  : -50.0,    # rad/s
    "V_MAX"  :  50.0,
    "KP_MIN" :   0.0,
    "KP_MAX" : 500.0,
    "KD_MIN" :   0.0,
    "KD_MAX" :   5.0,
    "T_MIN"  : -24.8,   # Nm
    "T_MAX"  :  24.8,
}

# Special command bytes (MIT protocol)
CMD_ENTER_MIT = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
CMD_EXIT_MIT  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
CMD_ZERO_POS  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])


def float_to_uint(x, x_min, x_max, bits):
    """Pack a float into an unsigned int with given bit width."""
    span = x_max - x_min
    x = max(x_min, min(x_max, x))   # clamp
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x_int, x_min, x_max, bits):
    """Unpack an unsigned int back to float."""
    span = x_max - x_min
    return x_int * span / ((1 << bits) - 1) + x_min


def pack_mit_command(pos, vel, kp, kd, torque_ff, params):
    """
    Pack MIT mode command into 8 bytes.
    Bit layout (MIT Mini-Cheetah protocol):
      Byte 0-1:        position  (16-bit, split 8+8 but packed as below)
      Actually the standard packing is:
        p  = 16 bits
        v  = 12 bits
        kp = 12 bits
        kd = 12 bits
        t  = 12 bits
      = 64 bits total = 8 bytes
    """
    p  = float_to_uint(pos,     params["P_MIN"],  params["P_MAX"],  16)
    v  = float_to_uint(vel,     params["V_MIN"],  params["V_MAX"],  12)
    kp_ = float_to_uint(kp,    params["KP_MIN"], params["KP_MAX"], 12)
    kd_ = float_to_uint(kd,    params["KD_MIN"], params["KD_MAX"], 12)
    t  = float_to_uint(torque_ff, params["T_MIN"], params["T_MAX"], 12)

    data = bytearray(8)
    data[0] = p >> 8
    data[1] = p & 0xFF
    data[2] = v >> 4
    data[3] = ((v & 0xF) << 4) | (kp_ >> 8)
    data[4] = kp_ & 0xFF
    data[5] = kd_ >> 4
    data[6] = ((kd_ & 0xF) << 4) | (t >> 8)
    data[7] = t & 0xFF
    return bytes(data)


def unpack_mit_reply(data, params):
    """
    Decode the 6-byte feedback from motor.
    Reply format:
      Byte 0: motor ID
      Byte 1-2: position (16 bits)
      Byte 2-3: velocity (12 bits, split across bytes 2-3)
      Byte 4-5: torque   (12 bits)
    Actually standard layout:
      [0]    = CAN ID
      [1][2] = position 16 bits
      [3][4] bits 7-4 = velocity high 4, bits 3-0 = velocity low 4
            (velocity is 12 bits across bytes 3 and 4 upper nibble)
      etc.
    """
    if len(data) < 6:
        return None, None, None

    motor_id = data[0]
    p_int  = (data[1] << 8) | data[2]
    v_int  = (data[3] << 4) | (data[4] >> 4)
    t_int  = ((data[4] & 0xF) << 8) | data[5]

    pos = uint_to_float(p_int, params["P_MIN"], params["P_MAX"], 16)
    vel = uint_to_float(v_int, params["V_MIN"], params["V_MAX"], 12)
    torque = uint_to_float(t_int, params["T_MIN"], params["T_MAX"], 12)

    return pos, vel, torque


def send_and_receive(bus, motor_id, data, timeout=0.05):
    """Send a CAN frame and wait for reply from the motor."""
    msg = can.Message(arbitration_id=motor_id,
                      data=data,
                      is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"[FAIL] Send error: {e}")
        return None

    # Wait for reply
    deadline = time.time() + timeout
    while time.time() < deadline:
        reply = bus.recv(timeout=0.01)
        if reply and reply.arbitration_id == motor_id:
            return reply.data
    return None


def main():
    print("=" * 55)
    print("AK70-10 MIT Mode Test (SAFE - zero torque)")
    print("=" * 55)
    print(f"Motor CAN ID : 0x{MOTOR_ID:02X}")
    print(f"Interface    : {CAN_IFACE}")
    print()

    # Open CAN bus
    try:
        bus = can.interface.Bus(channel=CAN_IFACE, bustype='socketcan')
        print("[OK] CAN bus opened")
    except Exception as e:
        print(f"[FAIL] Cannot open {CAN_IFACE}: {e}")
        print("Run 01_check_can.py first.")
        return

    try:
        # --- Enter MIT mode ---
        print("\n[1] Sending ENTER MIT MODE command...")
        reply = send_and_receive(bus, MOTOR_ID, CMD_ENTER_MIT)
        if reply:
            print(f"    Got reply: {' '.join(f'{b:02X}' for b in reply)}")
        else:
            print("    No reply received.")
            print("    Check: motor powered? CAN wired correctly? Correct ID?")
            return

        time.sleep(0.1)

        # --- Send zero command and read back state ---
        print("\n[2] Sending ZERO TORQUE command (safe, Kp=0, Kd=0)...")
        cmd = pack_mit_command(pos=0.0, vel=0.0, kp=0.0, kd=0.0,
                               torque_ff=0.0, params=AK70_10_PARAMS)
        print(f"    Sent bytes: {' '.join(f'{b:02X}' for b in cmd)}")

        reply = send_and_receive(bus, MOTOR_ID, cmd)
        if reply:
            raw_hex = ' '.join(f'{b:02X}' for b in reply)
            pos, vel, torque = unpack_mit_reply(reply, AK70_10_PARAMS)
            print(f"\n    Raw reply : {raw_hex}")
            print(f"    Position  : {pos:.4f} rad  ({pos * 57.296:.2f} deg)")
            print(f"    Velocity  : {vel:.4f} rad/s")
            print(f"    Torque    : {torque:.4f} Nm")
            print("\n[OK] Motor is responding correctly!")
        else:
            print("    No reply to command — motor may not be in MIT mode yet.")

        time.sleep(0.5)

        # --- Poll 5 times to confirm encoder is live ---
        print("\n[3] Reading encoder 5 times (1 second)...")
        for i in range(5):
            reply = send_and_receive(bus, MOTOR_ID, cmd)
            if reply:
                pos, vel, torque = unpack_mit_reply(reply, AK70_10_PARAMS)
                print(f"    [{i+1}] pos={pos:.4f} rad  vel={vel:.4f} rad/s  "
                      f"torque={torque:.4f} Nm")
            else:
                print(f"    [{i+1}] No reply")
            time.sleep(0.2)

    finally:
        # Always exit motor mode before closing
        print("\n[4] Sending EXIT MIT MODE command (disabling motor)...")
        send_and_receive(bus, MOTOR_ID, CMD_EXIT_MIT)
        print("[OK] Motor disabled safely.")
        bus.shutdown()
        print("[OK] CAN bus closed.")


if __name__ == "__main__":
    main()
