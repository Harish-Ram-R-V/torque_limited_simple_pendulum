import can
import time
import math
import matplotlib
matplotlib.use('Agg')            # Agg = no display needed, saves to file
import matplotlib.pyplot as plt
from collections import deque
import threading
import subprocess

# ── AK70-10 MIT parameters ─────────────────────────────────────────────
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
    0: "No fault",       1: "Motor over-temp",
    2: "Over-current",   3: "Over-voltage",
    4: "Under-voltage",  5: "Encoder fault",
    6: "FET over-temp",  7: "Motor stall"
}

# ── CAN setup ───────────────────────────────────────────────────────────
def reset_can():
    subprocess.run(["sudo","ip","link","set","can0","down"],
                   capture_output=True)
    time.sleep(0.3)
    subprocess.run(["sudo","ip","link","set","can0","up",
                    "type","can","bitrate","1000000",
                    "restart-ms","100"],
                   capture_output=True)
    time.sleep(0.2)
    subprocess.run(["sudo","ip","link","set","can0",
                    "txqueuelen","1000"],
                   capture_output=True)
    time.sleep(0.1)

def send_raw(bus, data):
    try:
        bus.send(can.Message(
            arbitration_id=MOTOR_ID,
            data=data,
            is_extended_id=False
        ), timeout=0.2)
        return True
    except Exception:
        return False

def get_reply(bus, timeout=0.05):
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

def enable_motor(bus):
    send_raw(bus, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC])
    time.sleep(0.1)
    return get_reply(bus, timeout=1.0)

def disable_motor(bus):
    for _ in range(3):
        send_raw(bus, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD])
        time.sleep(0.05)

# ── Shared state ────────────────────────────────────────────────────────
# No maxlen here — we keep ALL data for the final plot
time_data          = []
vel_actual_data    = []
vel_target_data    = []

current_target_vel = 0.0
motor_running      = True
kd_gain            = 2.0
data_lock          = threading.Lock()

# ── Motor control thread ────────────────────────────────────────────────
def motor_thread_func(bus):
    """
    Runs continuously in background.
    Sends MIT velocity commands at 20Hz.
    Stores every reply into shared lists for plotting later.
    Prints live telemetry to terminal so you can see
    what is happening without a plot window.
    """
    global motor_running
    start_time = time.time()
    last_print = time.time()

    while motor_running:
        with data_lock:
            target = current_target_vel
            kd     = kd_gain

        # Send velocity command
        # kp=0 → pure velocity mode
        cmd = pack_mit(0.0, target, 0.0, kd, 0.0)
        send_raw(bus, cmd)

        # Read motor reply
        reply = get_reply(bus, timeout=0.02)
        if reply:
            _, pos, vel, torq, temp, err = reply
            elapsed = time.time() - start_time

            # Store for plot
            with data_lock:
                time_data.append(elapsed)
                vel_actual_data.append(vel)
                vel_target_data.append(target)

            # Print to terminal every 200ms so it doesn't spam
            now = time.time()
            if now - last_print >= 0.2:
                error = vel - target
                print(f"  t={elapsed:6.2f}s  "
                      f"target={target:+7.3f}  "
                      f"actual={vel:+7.3f}  "
                      f"error={error:+7.3f}  "
                      f"torq={torq:+6.3f}  "
                      f"temp={temp}°C",
                      end='\r')
                last_print = now

            if err != 0:
                print(f"\n  [MOTOR ERROR] "
                      f"{ERROR_CODES.get(err,err)}")

        # 20Hz send rate — safe for MCP2515 at 1Mbps
        time.sleep(0.05)

# ── Save plot to PNG ────────────────────────────────────────────────────
def save_plot(filename='velocity_plot.png'):
    """
    Called once at the end after motor stops.
    Reads all collected data and saves a clean PNG plot.
    No display needed — works perfectly over SSH.
    """
    with data_lock:
        t   = list(time_data)
        va  = list(vel_actual_data)
        vt  = list(vel_target_data)

    if len(t) < 2:
        print("  Not enough data to plot.")
        return

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.patch.set_facecolor('#1e1e1e')

    # ── Top plot: velocity vs time ──────────────────────────────────
    ax1 = axes[0]
    ax1.set_facecolor('#2d2d2d')
    ax1.plot(t, vt, color='#FF4444', linewidth=1.5,
             linestyle='--', label='Target velocity')
    ax1.plot(t, va, color='#FFD700', linewidth=2,
             label='Actual velocity')
    ax1.axhline(y=0, color='#666666', linewidth=0.8)
    ax1.set_ylabel('Velocity (rad/s)', color='white', fontsize=12)
    ax1.set_title('AK70-10 — Velocity vs Time',
                  color='white', fontsize=14, fontweight='bold')
    ax1.legend(facecolor='#3d3d3d', labelcolor='white', fontsize=11)
    ax1.tick_params(colors='white')
    ax1.grid(True, alpha=0.3, color='#666666')
    for spine in ax1.spines.values():
        spine.set_color('#666666')

    # ── Bottom plot: tracking error vs time ─────────────────────────
    # error = actual - target
    # shows how well the motor followed the command
    error_data = [a - tgt for a, tgt in zip(va, vt)]

    ax2 = axes[1]
    ax2.set_facecolor('#2d2d2d')
    ax2.plot(t, error_data, color='#00BFFF',
             linewidth=1.5, label='Tracking error')
    ax2.axhline(y=0, color='#FF4444',
                linewidth=1.0, linestyle='--')
    ax2.fill_between(t, error_data, 0,
                     alpha=0.2, color='#00BFFF')
    ax2.set_xlabel('Time (s)', color='white', fontsize=12)
    ax2.set_ylabel('Error (rad/s)', color='white', fontsize=12)
    ax2.set_title('Tracking Error (actual - target)',
                  color='white', fontsize=12)
    ax2.legend(facecolor='#3d3d3d', labelcolor='white', fontsize=11)
    ax2.tick_params(colors='white')
    ax2.grid(True, alpha=0.3, color='#666666')
    for spine in ax2.spines.values():
        spine.set_color('#666666')

    plt.tight_layout()
    plt.savefig(filename, dpi=150,
                bbox_inches='tight',
                facecolor='#1e1e1e')
    plt.close()
    print(f"\n  Plot saved to {filename}")
    print(f"  Copy to your laptop with:")
    print(f"  scp l006@172.16.9.183:~/{filename} .")

# ── Main ────────────────────────────────────────────────────────────────
def main():
    global motor_running, current_target_vel, kd_gain

    print("="*55)
    print("  AK70-10 Velocity Controller — Plot saved on exit")
    print("="*55)

    # Reset CAN
    print("\n  Resetting can0...")
    reset_can()
    print("  can0 ready")

    # Open bus
    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan')
    except Exception as e:
        print(f"[FAIL] Cannot open can0: {e}")
        return

    try:
        # Enable motor
        print("\n  Enabling motor...")
        reply = enable_motor(bus)
        if reply is None:
            print("[FAIL] No motor reply.")
            return

        _, pos, _, _, temp, err = reply
        print(f"  Motor online — "
              f"pos={math.degrees(pos):.2f}°  "
              f"temp={temp}°C  "
              f"status={ERROR_CODES.get(err,err)}")

        # Get kd
        print("\n  Velocity gain kd "
              "(recommended: 2.0, range: 0.1–5.0)")
        while True:
            try:
                kd_gain = float(input("  Enter kd: "))
                if 0.1 <= kd_gain <= 5.0:
                    break
                print("  Enter 0.1 to 5.0")
            except ValueError:
                print("  Enter a number")

        # Start motor thread
        m_thread = threading.Thread(
            target=motor_thread_func,
            args=(bus,),
            daemon=True
        )
        m_thread.start()

        # ── Main input loop ─────────────────────────────────────────
        print("\n" + "="*55)
        print("  Motor running. Type velocity and press Enter.")
        print("  Type 'quit' to stop and save plot.")
        print("  Range: -50.0 to +50.0 rad/s")
        print("="*55 + "\n")

        while True:
            try:
                raw = input("  Target (rad/s): ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if raw.lower() in ('quit','q','exit',''):
                break

            try:
                val = float(eval(raw,
                    {"__builtins__": {}},
                    {"pi": math.pi, "sqrt": math.sqrt}))
            except Exception:
                print("  Invalid. Try: 5.0  -3.14  pi/2")
                continue

            if not (V_MIN <= val <= V_MAX):
                print(f"  Out of range: {V_MIN} to {V_MAX}")
                continue

            with data_lock:
                current_target_vel = val
            print(f"  → Target set to {val:.3f} rad/s")

    finally:
        # Stop motor thread
        motor_running = False
        time.sleep(0.3)

        # Disable motor
        print("\n\n  Disabling motor...")
        with data_lock:
            current_target_vel = 0.0
        disable_motor(bus)
        bus.shutdown()
        print("  Motor disabled.")

        # Save plot
        print("  Saving plot...")
        save_plot('velocity_plot.png')

if __name__ == "__main__":
    main()
