import can
import time
import math
import subprocess

# ── AK70-10 MIT parameters (manual section 5.3, page 63) ──────────────
MOTOR_ID = 1
P_MIN,  P_MAX  = -12.5,  12.5
V_MIN,  V_MAX  = -50.0,  50.0
T_MIN,  T_MAX  = -25.0,  25.0
KP_MIN, KP_MAX =   0.0, 500.0
KD_MIN, KD_MAX =   0.0,   5.0

# ── Conversion helpers ──────────────────────────────────────────────────
def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))

def uint_to_float(x_int, x_min, x_max, bits):
    return x_int * (x_max - x_min) / ((1 << bits) - 1) + x_min

# ── Pack MIT command (manual page 60-61) ───────────────────────────────
def pack_mit(p_des, v_des, kp, kd, t_ff):
    p_int  = float_to_uint(p_des, P_MIN,  P_MAX,  16)
    v_int  = float_to_uint(v_des, V_MIN,  V_MAX,  12)
    kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12)
    t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12)
    return [
        (p_int >> 8) & 0xFF,
         p_int & 0xFF,
        (v_int >> 4) & 0xFF,
        ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),
         kp_int & 0xFF,
        (kd_int >> 4) & 0xFF,
        ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
         t_int & 0xFF
    ]

# ── Decode motor reply (manual page 61, 67-68) ─────────────────────────
def decode_reply(data):
    motor_id = data[0]
    p_int    = (data[1] << 8) | data[2]
    v_int    = (data[3] << 4) | (data[4] >> 4)
    i_int    = ((data[4] & 0xF) << 8) | data[5]
    temp     = data[6] - 40
    error    = data[7]
    pos    = uint_to_float(p_int, P_MIN, P_MAX, 16)
    vel    = uint_to_float(v_int, V_MIN, V_MAX, 12)
    torque = uint_to_float(i_int, T_MIN, T_MAX, 12)
    return motor_id, pos, vel, torque, temp, error

ERROR_CODES = {
    0: "No fault",
    1: "Motor over-temperature",
    2: "Over-current",
    3: "Over-voltage",
    4: "Under-voltage",
    5: "Encoder fault",
    6: "MOSFET over-temperature",
    7: "Motor stall"
}

# ── Bring can0 up cleanly ───────────────────────────────────────────────
def reset_can():
    """Bring can0 down and up cleanly. Must be run before opening bus."""
    subprocess.run(["sudo","ip","link","set","can0","down"],
                   capture_output=True)
    time.sleep(0.3)
    r = subprocess.run(
        ["sudo","ip","link","set","can0","up",
         "type","can","bitrate","1000000",
         "restart-ms","100"],       # ← auto-restart on bus-off
        capture_output=True
    )
    time.sleep(0.3)
    return r.returncode == 0

# ── CAN send ────────────────────────────────────────────────────────────
def send_raw(bus, data):
    """
    Send one standard CAN frame.
    timeout=0.2 → waits up to 200ms if TX buffer is full
    instead of crashing immediately.
    Returns True on success, False on failure.
    """
    try:
        bus.send(
            can.Message(
                arbitration_id=MOTOR_ID,
                data=data,
                is_extended_id=False
            ),
            timeout=0.2
        )
        return True
    except can.CanOperationError as e:
        print(f"\n  [TX FAIL] {e}")
        return False

# ── CAN receive ─────────────────────────────────────────────────────────
def get_reply(bus, timeout=0.1):
    """
    Wait for a genuine motor reply frame.
    Filters out our own echoed packets (data[0]==0xFF).
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        msg = bus.recv(timeout=remaining)
        if msg is None:
            break
        if (len(msg.data) == 8
                and not msg.is_extended_id
                and msg.data[0] == MOTOR_ID
                and msg.data[0] != 0xFF):
            return decode_reply(msg.data)
    return None

# ── Motor special commands ──────────────────────────────────────────────
def enable_motor(bus):
    send_raw(bus, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC])
    time.sleep(0.1)
    return get_reply(bus, timeout=1.0)

def disable_motor(bus):
    # Send disable 3 times to make sure it lands
    for _ in range(3):
        send_raw(bus, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD])
        time.sleep(0.05)

def set_zero(bus):
    send_raw(bus, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE])
    time.sleep(0.1)

# ── Position move ───────────────────────────────────────────────────────
def move_to(bus, target_rad, kp=30.0, kd=1.0,
            speed=2.0, threshold=0.05, timeout=20.0):
    """
    Send MIT position commands at 20Hz (every 50ms) until
    motor reaches target_rad within threshold.

    Sending at 20Hz (not 50Hz) gives the motor enough time
    to ACK each frame before the next one arrives — prevents
    TX buffer overflow.
    """
    target_deg = math.degrees(target_rad)
    print(f"\n  Moving to {target_deg:.2f}° ({target_rad:.4f} rad)")
    print(f"  kp={kp}  kd={kd}  speed={speed} rad/s")
    print(f"\n  {'Pos':>10}  {'Vel':>8}  {'Torq':>8}  "
          f"{'Temp':>5}  {'Remaining':>10}")
    print(f"  {'-'*55}")

    reached  = False
    start    = time.time()
    cmd      = pack_mit(target_rad, speed, kp, kd, 0.0)

    while time.time() - start < timeout:

        # Send command
        ok = send_raw(bus, cmd)
        if not ok:
            # Single send failed — wait a bit and try again
            # Do NOT bring interface down — that kills everything
            time.sleep(0.1)
            continue

        # Read reply
        reply = get_reply(bus, timeout=0.05)
        if reply:
            mid, pos, vel, torq, temp, err = reply
            dist = abs(pos - target_rad)
            elapsed = time.time() - start

            print(f"  {math.degrees(pos):>9.2f}°  "
                  f"{vel:>8.3f}  "
                  f"{torq:>8.3f}  "
                  f"{temp:>4}°C  "
                  f"{math.degrees(dist):>9.2f}°",
                  end='\r')

            if err != 0:
                print(f"\n  [MOTOR ERROR] {ERROR_CODES.get(err, err)}")

            # Reached when close enough AND nearly stopped
            if dist < threshold and abs(vel) < 0.5:
                print(f"\n\n  [REACHED] {math.degrees(pos):.2f}° "
                      f"after {elapsed:.1f}s")
                reached = True
                break

        # 50ms between sends = 20Hz
        # This is the key fix — gives motor time to ACK
        time.sleep(0.05)

    if not reached:
        print(f"\n\n  [TIMEOUT] Target not reached in {timeout}s")

    return reached

# ── Position hold ───────────────────────────────────────────────────────
def hold_position(bus, pos_rad, kp=30.0, kd=1.0, duration=2.0):
    """Hold motor at pos_rad for duration seconds at 20Hz."""
    print(f"  Holding at {math.degrees(pos_rad):.2f}° "
          f"for {duration}s...")
    cmd   = pack_mit(pos_rad, 0.0, kp, kd, 0.0)
    start = time.time()
    while time.time() - start < duration:
        send_raw(bus, cmd)
        reply = get_reply(bus, timeout=0.05)
        if reply:
            _, pos, _, torq, _, _ = reply
            print(f"  Hold: {math.degrees(pos):.2f}°  "
                  f"torque={torq:.3f}N·m", end='\r')
        time.sleep(0.05)
    print()

# ── Input helpers ───────────────────────────────────────────────────────
def get_float(prompt, min_val, max_val):
    while True:
        try:
            val = float(input(prompt))
            if min_val <= val <= max_val:
                return val
            print(f"  Out of range: {min_val} to {max_val}")
        except ValueError:
            print("  Enter a number.")

def get_yes_no(prompt):
    while True:
        ans = input(prompt).strip().lower()
        if ans in ('y','yes'): return True
        if ans in ('n','no'):  return False
        print("  Enter y or n")

# ── Main ────────────────────────────────────────────────────────────────
def main():
    print("="*55)
    print("  AK70-10 MIT Position Controller")
    print(f"  Position : ±12.5 rad  (±{math.degrees(12.5):.0f}°)")
    print(f"  Speed    : ±50.0 rad/s")
    print(f"  Torque   : ±25.0 N·m")
    print("="*55)

    # Reset CAN interface cleanly before opening
    print("\n  Resetting can0...")
    if not reset_can():
        print("  [WARN] reset_can failed — continuing anyway")
    print("  can0 ready")

    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')
    except Exception as e:
        print(f"[FAIL] Cannot open can0: {e}")
        return

    # Always disable motor on exit, even on crash
    try:
        # Enable
        print("\n[1] Enabling motor...")
        reply = enable_motor(bus)
        if reply is None:
            print("[FAIL] No reply. Check wiring and MIT mode.")
            return

        mid, pos, vel, torq, temp, err = reply
        print(f"[OK]  Motor online")
        print(f"      ID={mid}  pos={math.degrees(pos):.2f}°  "
              f"temp={temp}°C  status={ERROR_CODES.get(err,err)}")

        # Zero
        print(f"\n[2] Current position: {math.degrees(pos):.2f}°")
        if get_yes_no("  Set this as zero? (y/n): "):
            set_zero(bus)
            print("  Zero set.")
            current_pos = 0.0
        else:
            current_pos = pos
            print(f"  Keeping existing zero.")

        # Main loop
        print("\n" + "="*55)
        print("  Enter positions. Type 'quit' to exit.")
        print("  Examples: 1.5708  pi/2  -3.14  0")
        print("="*55)

        while True:
            print(f"\n  Current: ~{math.degrees(current_pos):.2f}°")
            print(f"  Range  : {P_MIN} to {P_MAX} rad "
                  f"({math.degrees(P_MIN):.0f}° to "
                  f"{math.degrees(P_MAX):.0f}°)")

            raw = input("\n  Target (rad) or 'quit': ").strip()
            if raw.lower() in ('quit','q','exit'):
                break

            # Allow math expressions like pi/2
            try:
                target = float(eval(raw,
                    {"__builtins__": {}},
                    {"pi": math.pi, "sqrt": math.sqrt,
                     "sin": math.sin, "cos": math.cos}))
            except Exception:
                print("  Invalid. Try: 1.5708  or  pi/2")
                continue

            if not (P_MIN <= target <= P_MAX):
                print(f"  Out of range: {P_MIN} to {P_MAX} rad")
                continue

            dist = abs(target - current_pos)
            print(f"\n  Target : {target:.4f} rad = "
                  f"{math.degrees(target):.2f}°")
            print(f"  Travel : {math.degrees(dist):.2f}°")

            if dist < 0.05:
                print("  Already at target.")
                if not get_yes_no(
                        "  Move to another position? (y/n): "):
                    break
                continue

            # Speed selection
            if get_yes_no("  Custom speed? (y/n, default=2.0): "):
                speed = get_float(
                    "  Speed (0.1 to 10.0 rad/s): ", 0.1, 10.0)
            else:
                speed = 2.0

            # Move
            reached = move_to(
                bus, target,
                kp=30.0, kd=1.0,
                speed=speed,
                threshold=0.05,
                timeout=20.0
            )

            if reached:
                hold_position(bus, target,
                              kp=30.0, kd=1.0, duration=2.0)
                current_pos = target

            if not get_yes_no(
                    "\n  Move to another position? (y/n): "):
                break

    finally:
        # This block ALWAYS runs — even on Ctrl+C or crash
        print("\n[3] Disabling motor...")
        disable_motor(bus)
        bus.shutdown()
        print("[DONE] Motor disabled safely.")

if __name__ == "__main__":
    main()
