import can
import time
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
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

# ── CAN helpers ─────────────────────────────────────────────────────────
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

def set_zero(bus):
    send_raw(bus, [0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE])
    time.sleep(0.1)

# ── Quintic trajectory ──────────────────────────────────────────────────
def quintic_coeffs(theta0, thetaf, T):
    """
    Compute 5th order polynomial coefficients.

    Boundary conditions (6 equations, 6 unknowns):
        θ(0)  = theta0   θ(T)  = thetaf
        θ̇(0)  = 0        θ̇(T)  = 0
        θ̈(0)  = 0        θ̈(T)  = 0

    Standard solution (from robotics textbooks):
        a0 = theta0
        a1 = 0
        a2 = 0
        a3 = 10(thetaf-theta0) / T³
        a4 = -15(thetaf-theta0) / T⁴
        a5 = 6(thetaf-theta0) / T⁵
    """
    d  = thetaf - theta0
    a0 =  theta0
    a1 =  0.0
    a2 =  0.0
    a3 =  10 * d / T**3
    a4 = -15 * d / T**4
    a5 =   6 * d / T**5
    return a0, a1, a2, a3, a4, a5

def quintic_eval(coeffs, t):
    """
    Evaluate position, velocity, acceleration at time t.

    θ(t)  = a0 + a1t + a2t² + a3t³ + a4t⁴ + a5t⁵
    θ̇(t)  = a1 + 2a2t + 3a3t² + 4a4t³ + 5a5t⁴
    θ̈(t)  = 2a2 + 6a3t + 12a4t² + 20a5t³
    """
    a0, a1, a2, a3, a4, a5 = coeffs

    pos = (a0
           + a1*t
           + a2*t**2
           + a3*t**3
           + a4*t**4
           + a5*t**5)

    vel = (a1
           + 2*a2*t
           + 3*a3*t**2
           + 4*a4*t**3
           + 5*a5*t**4)

    acc = (2*a2
           + 6*a3*t
           + 12*a4*t**2
           + 20*a5*t**3)

    return pos, vel, acc

def generate_trajectory(theta0, thetaf, T, dt=0.02):
    """
    Generate a list of (time, position, velocity, acceleration)
    tuples spaced dt seconds apart.

    dt=0.02 = 20ms = 50Hz send rate.
    This matches our CAN send rate perfectly.
    """
    coeffs  = quintic_coeffs(theta0, thetaf, T)
    steps   = int(T / dt) + 1
    traj    = []

    for i in range(steps):
        t           = i * dt
        t           = min(t, T)   # clamp to T exactly
        pos, vel, acc = quintic_eval(coeffs, t)
        traj.append((t, pos, vel, acc))

    return traj

# ── Plot results ────────────────────────────────────────────────────────
def save_plot(planned, actual_t, actual_pos,
              actual_vel, theta0_deg, thetaf_deg, T,
              filename='quintic_result.png'):
    """
    Four-panel plot:
    1. Position: planned trajectory vs actual motor position
    2. Position error: how far motor deviated from plan
    3. Velocity: planned vs actual
    4. Acceleration: planned only (motor doesn't report accel)

    This is exactly what your professor wants to see —
    does the motor follow the quintic curve?
    """

    # Planned trajectory
    t_plan   = [x[0] for x in planned]
    p_plan   = [math.degrees(x[1]) for x in planned]
    v_plan   = [math.degrees(x[2]) for x in planned]
    a_plan   = [math.degrees(x[3]) for x in planned]

    # Actual from motor (convert rad to deg)
    p_actual = [math.degrees(p) for p in actual_pos]
    v_actual = [math.degrees(v) for v in actual_vel]

    # Position error
    # Interpolate planned position at actual time stamps
    p_plan_interp = np.interp(actual_t, t_plan, p_plan)
    p_error       = [a - pl for a, pl
                     in zip(p_actual, p_plan_interp)]

    fig, axes = plt.subplots(4, 1, figsize=(12, 14))
    fig.suptitle(
        f'Quintic Trajectory Tracking\n'
        f'{theta0_deg:.1f}° → {thetaf_deg:.1f}°  |  T={T}s',
        fontsize=14, fontweight='bold'
    )

    # ── Panel 1: Position ──────────────────────────────────────
    ax = axes[0]
    ax.plot(t_plan, p_plan,
            color='blue', linewidth=2,
            linestyle='--', label='Planned θ(t)')
    ax.plot(actual_t, p_actual,
            color='orange', linewidth=1.5,
            label='Actual motor position')
    ax.axhline(theta0_deg, color='gray',
               linewidth=0.8, linestyle=':')
    ax.axhline(thetaf_deg, color='gray',
               linewidth=0.8, linestyle=':')
    ax.set_ylabel('Position (deg)', fontsize=11)
    ax.set_title('Position: Planned vs Actual')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # ── Panel 2: Position error ────────────────────────────────
    ax = axes[1]
    ax.plot(actual_t, p_error,
            color='red', linewidth=1.5,
            label='Tracking error (actual - planned)')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.fill_between(actual_t, p_error, 0,
                    alpha=0.2, color='red')
    ax.set_ylabel('Error (deg)', fontsize=11)
    ax.set_title('Position Tracking Error')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # ── Panel 3: Velocity ──────────────────────────────────────
    ax = axes[2]
    ax.plot(t_plan, v_plan,
            color='green', linewidth=2,
            linestyle='--', label='Planned θ̇(t)')
    ax.plot(actual_t, v_actual,
            color='lime', linewidth=1.5,
            label='Actual velocity')
    ax.axhline(0, color='gray', linewidth=0.8)
    ax.set_ylabel('Velocity (deg/s)', fontsize=11)
    ax.set_title('Velocity: Planned vs Actual')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # ── Panel 4: Planned acceleration ─────────────────────────
    ax = axes[3]
    ax.plot(t_plan, a_plan,
            color='purple', linewidth=2,
            label='Planned θ̈(t)')
    ax.axhline(0, color='gray', linewidth=0.8)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Acceleration (deg/s²)', fontsize=11)
    ax.set_title('Planned Acceleration Profile')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"\n  Plot saved → {filename}")
    print(f"  Copy to laptop:")
    print(f"  scp l006@172.16.9.183:~/{filename} .")

    # Print tracking statistics
    if p_error:
        print(f"\n  Tracking statistics:")
        print(f"  Max error  : {max(abs(e) for e in p_error):.3f}°")
        print(f"  Mean error : "
              f"{sum(abs(e) for e in p_error)/len(p_error):.3f}°")
        print(f"  Final error: {p_error[-1]:.3f}°")

# ── Input helpers ───────────────────────────────────────────────────────
def get_float(prompt, min_val, max_val):
    while True:
        try:
            val = float(eval(input(prompt).strip(),
                {"__builtins__": {}},
                {"pi": math.pi, "sqrt": math.sqrt}))
            if min_val <= val <= max_val:
                return val
            print(f"  Out of range: {min_val} to {max_val}")
        except Exception:
            print("  Enter a number.")

def get_yes_no(prompt):
    while True:
        ans = input(prompt).strip().lower()
        if ans in ('y','yes'): return True
        if ans in ('n','no'):  return False
        print("  Enter y or n")

# ── Execute trajectory on motor ─────────────────────────────────────────
def execute_trajectory(bus, traj, kp=40.0, kd=1.5):
    """
    Send each trajectory setpoint to the motor at exactly dt intervals.

    For each step:
      - p_des = planned position at this time  (from quintic)
      - v_des = planned velocity at this time  (feedforward)
      - kp    = position stiffness
      - kd    = damping
      - t_ff  = 0

    Using v_des from the trajectory (velocity feedforward)
    is the key improvement over basic position control.
    It tells the motor how fast it should be moving right now,
    not just where it should be. This greatly improves tracking.

    Returns lists of actual time, position, velocity recorded.
    """
    actual_t   = []
    actual_pos = []
    actual_vel = []

    print(f"\n  Executing {len(traj)} steps "
          f"over {traj[-1][0]:.2f}s")
    print(f"  kp={kp}  kd={kd}")
    print(f"\n  {'Step':>5}  {'t(s)':>6}  "
          f"{'Plan(°)':>9}  {'Act(°)':>9}  "
          f"{'Err(°)':>8}  {'Vel':>8}")
    print(f"  {'-'*60}")

    start_wall = time.time()

    for i, (t_plan, p_plan, v_plan, a_plan) in enumerate(traj):

        # Wall clock time this step should fire
        step_deadline = start_wall + t_plan

        # Send MIT command with velocity feedforward
        cmd = pack_mit(p_plan, v_plan, kp, kd, 0.0)
        send_raw(bus, cmd)

        # Read motor reply
        reply = get_reply(bus, timeout=0.015)
        wall_now = time.time()

        if reply:
            _, pos, vel, torq, temp, err = reply
            elapsed = wall_now - start_wall

            actual_t.append(elapsed)
            actual_pos.append(pos)
            actual_vel.append(vel)

            # Print every 10th step to avoid terminal spam
            if i % 10 == 0:
                error_deg = math.degrees(pos) - math.degrees(p_plan)
                print(f"  {i:>5}  {elapsed:>6.3f}  "
                      f"{math.degrees(p_plan):>+9.2f}  "
                      f"{math.degrees(pos):>+9.2f}  "
                      f"{error_deg:>+8.3f}  "
                      f"{math.degrees(vel):>+8.1f}",
                      end='\r')

            if err != 0:
                print(f"\n  [MOTOR ERROR] "
                      f"{ERROR_CODES.get(err,err)}")
                break

        # Precise timing — sleep until exactly the next step time
        # This keeps the trajectory on schedule regardless of
        # how long send/receive took
        sleep_remaining = step_deadline - time.time()
        if sleep_remaining > 0:
            time.sleep(sleep_remaining)

    print(f"\n  Trajectory complete.")
    return actual_t, actual_pos, actual_vel

# ── Main ────────────────────────────────────────────────────────────────
def main():
    print("="*55)
    print("  AK70-10 Quintic Trajectory Controller")
    print("="*55)
    print()
    print("  Quintic (5th order) polynomial trajectory.")
    print("  Boundary conditions:")
    print("    Start: pos=θ₀,  vel=0,  acc=0")
    print("    End:   pos=θ_f, vel=0,  acc=0")
    print("  Motor starts and stops with zero velocity")
    print("  and zero acceleration — perfectly smooth.")
    print("="*55)

    # Reset CAN
    print("\n  Resetting can0...")
    reset_can()
    print("  can0 ready")

    # Open bus
    try:
        bus = can.interface.Bus(channel='can0',
                                interface='socketcan')
    except Exception as e:
        print(f"[FAIL] Cannot open can0: {e}")
        return

    try:
        # Enable motor
        print("\n[1] Enabling motor...")
        reply = enable_motor(bus)
        if reply is None:
            print("[FAIL] No motor reply.")
            return

        _, pos, vel, torq, temp, err = reply
        print(f"[OK]  Motor online")
        print(f"      pos={math.degrees(pos):.2f}°  "
              f"temp={temp}°C  "
              f"status={ERROR_CODES.get(err,err)}")

        # Set zero
        print(f"\n[2] Setting current position as zero...")
        set_zero(bus)
        print(f"  Zero set. Motor is now at 0°.")

        # Main loop — run multiple trajectories
        run_number = 1

        while True:
            print(f"\n{'='*55}")
            print(f"  Trajectory #{run_number}")
            print(f"{'='*55}")

            # Get parameters from user
            print("\n  Enter trajectory parameters.")
            print("  Range: position ±716° (±12.5 rad)")
            print("  Examples: 90, -90, 180, 45, -180")

            theta0_deg = get_float(
                "\n  Start position (degrees, ±716°): ",
                -716.0, 716.0)

            thetaf_deg = get_float(
                "  End   position (degrees, ±716°): ",
                -716.0, 716.0)

            T = get_float(
                "  Total time T (seconds, 1.0 to 20.0): ",
                1.0, 20.0)

            # Convert to radians for motor
            theta0 = math.radians(theta0_deg)
            thetaf = math.radians(thetaf_deg)

            # Validate — check if positions are within motor range
            if abs(theta0) > P_MAX or abs(thetaf) > P_MAX:
                print("  Position out of motor range "
                      "(±12.5 rad = ±716°)")
                continue

            # Generate trajectory
            print(f"\n  Generating quintic trajectory...")
            traj = generate_trajectory(theta0, thetaf, T, dt=0.02)
            print(f"  {len(traj)} setpoints generated")
            print(f"  Send rate: 50Hz (every 20ms)")

            # Show planned peak velocity
            _, _, v_peak, _ = max(
                traj, key=lambda x: abs(x[2]))
            print(f"  Peak planned velocity: "
                  f"{math.degrees(v_peak):.1f}°/s  "
                  f"({v_peak:.3f} rad/s)")

            # Confirm before executing
            print(f"\n  Ready to move motor:")
            print(f"  {theta0_deg:.1f}° → {thetaf_deg:.1f}°"
                  f"  in {T:.1f}s")
            if not get_yes_no("  Execute? (y/n): "):
                continue

            # Move to start position first if needed
            cur_reply = get_reply(bus, timeout=0.1)
            if cur_reply:
                cur_pos = cur_reply[1]
                if abs(cur_pos - theta0) > 0.1:
                    print(f"\n  Moving to start position "
                          f"{theta0_deg:.1f}° first...")
                    # Simple position move to start
                    start_traj = generate_trajectory(
                        cur_pos, theta0, 2.0, dt=0.02)
                    execute_trajectory(
                        bus, start_traj, kp=40.0, kd=1.5)
                    time.sleep(0.5)

            # Execute main trajectory
            print(f"\n[3] Executing quintic trajectory...")
            actual_t, actual_pos, actual_vel = execute_trajectory(
                bus, traj, kp=40.0, kd=1.5)

            # Hold at final position for 1 second
            print(f"\n  Holding at {thetaf_deg:.1f}°...")
            hold_cmd = pack_mit(thetaf, 0.0, 40.0, 1.5, 0.0)
            hold_start = time.time()
            while time.time() - hold_start < 1.0:
                send_raw(bus, hold_cmd)
                time.sleep(0.05)

            # Save plot
            filename = f'quintic_result_{run_number}.png'
            print(f"\n[4] Saving plot...")
            save_plot(traj, actual_t, actual_pos, actual_vel,
                      theta0_deg, thetaf_deg, T, filename)

            run_number += 1

            if not get_yes_no(
                    "\n  Run another trajectory? (y/n): "):
                break

    finally:
        print("\n[5] Disabling motor...")
        disable_motor(bus)
        bus.shutdown()
        print("  Motor disabled safely.")

if __name__ == "__main__":
    main()
