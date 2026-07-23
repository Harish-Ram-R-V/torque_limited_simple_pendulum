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

# ── Quintic core ────────────────────────────────────────────────────────
def quintic_coeffs_general(theta0, thetaf, v0, vf, T):
    """
    General quintic with specified boundary velocities.
    This is the key difference from the single-segment version.

    Boundary conditions:
        θ(0)  = theta0    θ(T)  = thetaf
        θ̇(0)  = v0        θ̇(T)  = vf
        θ̈(0)  = 0         θ̈(T)  = 0

    v0 and vf can be nonzero — this is what allows
    smooth chaining through waypoints without stopping.

    Solving the 6x6 linear system gives:
    """
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5

    a0 = theta0
    a1 = v0
    a2 = 0.0
    a3 = (20*thetaf - 20*theta0 - (8*vf + 12*v0)*T) / (2*T3)
    a4 = (30*theta0 - 30*thetaf + (14*vf + 16*v0)*T) / (2*T4)
    a5 = (12*thetaf - 12*theta0 - (6*vf + 6*v0)*T)  / (2*T5)

    return a0, a1, a2, a3, a4, a5

def quintic_eval(coeffs, t):
    a0, a1, a2, a3, a4, a5 = coeffs
    pos = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
    vel = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
    acc = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
    return pos, vel, acc

# ── Waypoint velocity computation ───────────────────────────────────────
def compute_waypoint_velocities(angles, times):
    """
    Compute velocity at each waypoint using finite differences.

    For intermediate waypoints:
        v_i = (θ_{i+1} - θ_{i-1}) / (t_{i+1} - t_{i-1})

    This is the slope of the chord connecting neighbouring
    waypoints — ensures the trajectory passes through each
    point smoothly without stopping.

    Start and end velocities are always zero
    (motor starts still and ends still).

    Example:
        waypoints: [0°, 90°, 45°, 135°]
        times:     [0s, 2s,  4s,  6s  ]

        v at 90°  = (45° - 0°)  / (4s - 0s) = 11.25 °/s
        v at 45°  = (135° - 90°)/ (6s - 2s) = 11.25 °/s
    """
    n  = len(angles)
    vd = [0.0] * n          # start with all zeros

    # Intermediate points only
    for i in range(1, n - 1):
        dt_total = times[i+1] - times[i-1]
        if dt_total > 0:
            vd[i] = (angles[i+1] - angles[i-1]) / dt_total
        # vd[0] stays 0 (start), vd[-1] stays 0 (end)

    return vd

def generate_multipoint_trajectory(angles_rad, times, dt=0.02):
    """
    Generate a complete multi-segment quintic trajectory.

    angles_rad : list of waypoint positions in radians
    times      : list of timestamps for each waypoint
    dt         : time step in seconds (0.02 = 50Hz)

    Returns a single flat list of (t, pos, vel, acc) tuples
    covering the entire trajectory from first to last waypoint.
    """
    n = len(angles_rad)
    assert n >= 2, "Need at least 2 waypoints"
    assert len(times) == n, "Need one time per waypoint"

    # Compute velocity at each waypoint
    vd = compute_waypoint_velocities(angles_rad, times)

    full_traj = []    # will hold ALL steps across ALL segments

    # Build one quintic segment between each pair of waypoints
    for seg in range(n - 1):
        theta0 = angles_rad[seg]
        thetaf = angles_rad[seg + 1]
        v0     = vd[seg]
        vf     = vd[seg + 1]
        t0     = times[seg]
        tf     = times[seg + 1]
        T      = tf - t0

        coeffs = quintic_coeffs_general(theta0, thetaf, v0, vf, T)

        # How many steps in this segment
        steps  = int(T / dt)

        for i in range(steps):
            t_local  = i * dt
            t_global = t0 + t_local   # time from start of whole traj
            pos, vel, acc = quintic_eval(coeffs, t_local)
            full_traj.append((t_global, pos, vel, acc))

    # Add the very last point exactly
    last_coeffs = quintic_coeffs_general(
        angles_rad[-2], angles_rad[-1],
        vd[-2], vd[-1],
        times[-1] - times[-2]
    )
    T_last = times[-1] - times[-2]
    pos, vel, acc = quintic_eval(last_coeffs, T_last)
    full_traj.append((times[-1], pos, vel, acc))

    return full_traj, vd

# ── Execute on motor ────────────────────────────────────────────────────
def execute_trajectory(bus, traj, kp=40.0, kd=1.5):
    actual_t   = []
    actual_pos = []
    actual_vel = []

    total_time = traj[-1][0] - traj[0][0]
    print(f"\n  Executing {len(traj)} steps "
          f"over {total_time:.2f}s at 50Hz")
    print(f"  kp={kp}  kd={kd}  (velocity feedforward ON)")
    print(f"\n  {'t(s)':>6}  {'Plan(°)':>9}  "
          f"{'Act(°)':>9}  {'Err(°)':>8}")
    print(f"  {'-'*45}")

    start_wall = time.time()

    for i, (t_plan, p_plan, v_plan, a_plan) in enumerate(traj):
        step_deadline = start_wall + t_plan

        # Send position + velocity feedforward
        cmd = pack_mit(p_plan, v_plan, kp, kd, 0.0)
        send_raw(bus, cmd)

        reply = get_reply(bus, timeout=0.015)
        wall_now = time.time()

        if reply:
            _, pos, vel, torq, temp, err = reply
            elapsed = wall_now - start_wall

            actual_t.append(elapsed)
            actual_pos.append(pos)
            actual_vel.append(vel)

            if i % 25 == 0:
                err_deg = (math.degrees(pos)
                           - math.degrees(p_plan))
                print(f"  {elapsed:>6.3f}  "
                      f"{math.degrees(p_plan):>+9.2f}  "
                      f"{math.degrees(pos):>+9.2f}  "
                      f"{err_deg:>+8.3f}",
                      end='\r')

            if err != 0:
                print(f"\n  [ERROR] "
                      f"{ERROR_CODES.get(err,err)}")
                break

        # Precise timing
        sleep_t = step_deadline - time.time()
        if sleep_t > 0:
            time.sleep(sleep_t)

    print(f"\n  Done.")
    return actual_t, actual_pos, actual_vel

# ── Plot ────────────────────────────────────────────────────────────────
def save_plot(traj, actual_t, actual_pos, actual_vel,
              waypoint_angles_deg, waypoint_times,
              filename='multipoint_result.png'):

    t_plan = [x[0] for x in traj]
    p_plan = [math.degrees(x[1]) for x in traj]
    v_plan = [math.degrees(x[2]) for x in traj]
    a_plan = [math.degrees(x[3]) for x in traj]

    p_actual = [math.degrees(p) for p in actual_pos]
    v_actual = [math.degrees(v) for v in actual_vel]

    p_plan_interp = np.interp(actual_t, t_plan, p_plan)
    p_error       = [a - pl for a, pl
                     in zip(p_actual, p_plan_interp)]

    fig, axes = plt.subplots(4, 1, figsize=(13, 14))
    fig.suptitle(
        'Multi-point Quintic Trajectory Tracking',
        fontsize=14, fontweight='bold'
    )

    # Panel 1: Position
    ax = axes[0]
    ax.plot(t_plan, p_plan,
            color='blue', linewidth=2,
            linestyle='--', label='Planned θ(t)')
    ax.plot(actual_t, p_actual,
            color='orange', linewidth=1.5,
            label='Actual motor position')
    # Mark waypoints
    ax.scatter(waypoint_times, waypoint_angles_deg,
               color='red', s=80, zorder=5,
               label='Waypoints')
    for i, (wt, wa) in enumerate(
            zip(waypoint_times, waypoint_angles_deg)):
        ax.annotate(f'P{i}\n{wa:.0f}°',
                    (wt, wa),
                    textcoords='offset points',
                    xytext=(5, 8), fontsize=8)
    ax.set_ylabel('Position (deg)', fontsize=11)
    ax.set_title('Position: Planned vs Actual '
                 '(red dots = waypoints)')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)
    # Vertical lines at waypoint times
    for wt in waypoint_times[1:-1]:
        ax.axvline(x=wt, color='red',
                   linewidth=0.8, linestyle=':',
                   alpha=0.5)

    # Panel 2: Error
    ax = axes[1]
    ax.plot(actual_t, p_error,
            color='red', linewidth=1.5,
            label='Tracking error')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.fill_between(actual_t, p_error, 0,
                    alpha=0.2, color='red')
    for wt in waypoint_times[1:-1]:
        ax.axvline(x=wt, color='red',
                   linewidth=0.8, linestyle=':',
                   alpha=0.5)
    ax.set_ylabel('Error (deg)', fontsize=11)
    ax.set_title('Tracking Error at Waypoint Transitions '
                 '(should be small)')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # Panel 3: Velocity
    ax = axes[2]
    ax.plot(t_plan, v_plan,
            color='green', linewidth=2,
            linestyle='--', label='Planned θ̇(t)')
    ax.plot(actual_t, v_actual,
            color='lime', linewidth=1.5,
            label='Actual velocity')
    ax.axhline(0, color='gray', linewidth=0.8)
    for wt in waypoint_times[1:-1]:
        ax.axvline(x=wt, color='red',
                   linewidth=0.8, linestyle=':',
                   alpha=0.5)
    ax.set_ylabel('Velocity (deg/s)', fontsize=11)
    ax.set_title('Velocity — should be continuous '
                 'at waypoints (no sudden jumps)')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # Panel 4: Acceleration
    ax = axes[3]
    ax.plot(t_plan, a_plan,
            color='purple', linewidth=2,
            label='Planned θ̈(t)')
    ax.axhline(0, color='gray', linewidth=0.8)
    for wt in waypoint_times[1:-1]:
        ax.axvline(x=wt, color='red',
                   linewidth=0.8, linestyle=':',
                   alpha=0.5)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Acceleration (deg/s²)', fontsize=11)
    ax.set_title('Acceleration Profile')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"\n  Plot saved → {filename}")
    print(f"  scp l006@172.16.9.183:~/{filename} .")

    if p_error:
        print(f"\n  Tracking statistics:")
        print(f"  Max error  : "
              f"{max(abs(e) for e in p_error):.3f}°")
        print(f"  Mean error : "
              f"{sum(abs(e) for e in p_error)"
              f"/len(p_error):.3f}°")
        print(f"  Final error: {p_error[-1]:.3f}°")

# ── Input helpers ───────────────────────────────────────────────────────
def get_float(prompt, min_val, max_val):
    while True:
        try:
            val = float(eval(input(prompt).strip(),
                {"__builtins__": {}},
                {"pi": math.pi}))
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

# ── Main ────────────────────────────────────────────────────────────────
def main():
    print("="*55)
    print("  AK70-10 Multi-point Quintic Trajectory")
    print("="*55)
    print()
    print("  Chains multiple quintic segments together.")
    print("  Motor passes through waypoints WITHOUT stopping.")
    print("  Velocity at each waypoint computed by:")
    print("  v_i = (θ_{i+1} - θ_{i-1}) / (t_{i+1} - t_{i-1})")
    print()
    print("  Boundary conditions per segment:")
    print("    Start: pos=θ_i,  vel=v_i,  acc=0")
    print("    End:   pos=θ_j,  vel=v_j,  acc=0")
    print("="*55)

    print("\n  Resetting can0...")
    reset_can()
    print("  can0 ready")

    try:
        bus = can.interface.Bus(channel='can0',
                                interface='socketcan')
    except Exception as e:
        print(f"[FAIL] Cannot open can0: {e}")
        return

    try:
        print("\n[1] Enabling motor...")
        reply = enable_motor(bus)
        if reply is None:
            print("[FAIL] No motor reply.")
            return

        _, pos, vel, torq, temp, err = reply
        print(f"[OK]  Motor online — "
              f"pos={math.degrees(pos):.2f}°  "
              f"temp={temp}°C")

        print("\n[2] Setting zero...")
        set_zero(bus)
        print("  Zero set.")

        run_number = 1

        while True:
            print(f"\n{'='*55}")
            print(f"  Multi-point Trajectory #{run_number}")
            print(f"{'='*55}")

            # Get waypoints
            print("\n  How many waypoints? (including start and end)")
            print("  Minimum 3 — example: 0° → 90° → 45° → 135°")
            while True:
                try:
                    n_waypoints = int(input("  Number of waypoints"
                                            " (3 to 8): "))
                    if 3 <= n_waypoints <= 8:
                        break
                    print("  Enter 3 to 8")
                except ValueError:
                    print("  Enter an integer")

            angles_deg = []
            times      = [0.0]

            print(f"\n  Enter {n_waypoints} waypoints.")
            print("  Range: ±716° (±12.5 rad)")

            for i in range(n_waypoints):
                if i == 0:
                    label = "Start"
                elif i == n_waypoints - 1:
                    label = "End  "
                else:
                    label = f"P{i}   "

                ang = get_float(
                    f"  {label} angle (deg): ",
                    -716.0, 716.0)
                angles_deg.append(ang)

                if i > 0:
                    t = get_float(
                        f"  Time to reach this point "
                        f"(seconds from start, >"
                        f"{times[-1]:.1f}): ",
                        times[-1] + 0.5,
                        times[-1] + 30.0)
                    times.append(t)

            # Convert to radians
            angles_rad = [math.radians(a) for a in angles_deg]

            # Check motor range
            if any(abs(a) > P_MAX for a in angles_rad):
                print("  A waypoint exceeds motor range ±12.5 rad")
                continue

            # Generate trajectory
            print(f"\n  Generating trajectory...")
            traj, vd = generate_multipoint_trajectory(
                angles_rad, times, dt=0.02)

            total_t = times[-1]
            print(f"  {len(traj)} setpoints over {total_t:.1f}s")
            print(f"\n  Waypoint summary:")
            print(f"  {'Point':>6}  {'Angle':>8}  "
                  f"{'Time':>6}  {'Velocity at point':>18}")
            print(f"  {'-'*50}")
            for i in range(n_waypoints):
                label = ("START" if i == 0
                         else "END" if i == n_waypoints-1
                         else f"P{i}")
                print(f"  {label:>6}  "
                      f"{angles_deg[i]:>+7.1f}°  "
                      f"{times[i]:>5.1f}s  "
                      f"{math.degrees(vd[i]):>+10.2f} °/s"
                      f"  {'← stops' if i in (0,n_waypoints-1) else '← passes through'}")

            if not get_yes_no(
                    "\n  Execute on motor? (y/n): "):
                continue

            # Move to start position first
            cur_reply = get_reply(bus, timeout=0.1)
            if cur_reply:
                cur_pos = cur_reply[1]
                if abs(cur_pos - angles_rad[0]) > 0.05:
                    print(f"\n  Moving to start "
                          f"{angles_deg[0]:.1f}° first...")
                    from_traj, _ = generate_multipoint_trajectory(
                        [cur_pos, angles_rad[0]],
                        [0.0, 2.0], dt=0.02)
                    execute_trajectory(
                        bus, from_traj, kp=40.0, kd=1.5)
                    time.sleep(0.5)

            # Execute
            print(f"\n[3] Executing multi-point trajectory...")
            actual_t, actual_pos, actual_vel = \
                execute_trajectory(bus, traj,
                                   kp=40.0, kd=1.5)

            # Hold at end
            end_cmd = pack_mit(
                angles_rad[-1], 0.0, 40.0, 1.5, 0.0)
            hold_start = time.time()
            while time.time() - hold_start < 1.0:
                send_raw(bus, end_cmd)
                time.sleep(0.05)

            # Save plot
            filename = (f'multipoint_result'
                        f'_{run_number}.png')
            print(f"\n[4] Saving plot...")
            save_plot(traj, actual_t, actual_pos,
                      actual_vel, angles_deg, times,
                      filename)

            run_number += 1

            if not get_yes_no(
                    "\n  Run another trajectory? (y/n): "):
                break

    finally:
        print("\n[5] Disabling motor...")
        disable_motor(bus)
        bus.shutdown()
        print("  Done.")

if __name__ == "__main__":
    main()
