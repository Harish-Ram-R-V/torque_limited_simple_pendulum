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
P_MIN,  P_MAX  = -12.5,  12.5    # rad
V_MIN,  V_MAX  = -50.0,  50.0    # rad/s
T_MIN,  T_MAX  = -25.0,  25.0    # N·m
KP_MIN, KP_MAX =   0.0, 500.0
KD_MIN, KD_MAX =   0.0,   5.0

# ── Conversion helpers ──────────────────────────────────────────────────
def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))

def uint_to_float(x_int, x_min, x_max, bits):
    return x_int * (x_max - x_min) / ((1 << bits) - 1) + x_min

def pack_mit(p_des, v_des, kp, kd, t_ff):
    """
    Pack 5 control values into 8-byte MIT CAN frame.

    Variables:
      p_des  = desired position   (radians)
      v_des  = desired velocity   (rad/s)   ← this is v_cmd after correction
      kp     = position stiffness (0-500)
      kd     = velocity damping   (0-5)
      t_ff   = feedforward torque (N·m)

    Motor computes internally:
      τ = kp*(p_des - p_actual) + kd*(v_des - v_actual) + t_ff
    """
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
    """
    Decode 8-byte motor reply into physical values.

    Variables returned:
      motor_id = which motor replied (should be 1)
      pos      = actual position (radians)   ← what motor reports it is
      vel      = actual velocity (rad/s)     ← from encoder differentiation
      torque   = actual torque   (N·m)       ← measured current × kt
      temp     = driver temperature (°C)
      error    = fault code (0=healthy)
    """
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
    subprocess.run(["sudo","ip","link","set","can1","down"],
                   capture_output=True)
    time.sleep(0.3)
    subprocess.run(["sudo","ip","link","set","can1","up",
                    "type","can","bitrate","1000000",
                    "restart-ms","100"],
                   capture_output=True)
    time.sleep(0.2)
    subprocess.run(["sudo","ip","link","set","can1",
                    "txqueuelen","1000"],
                   capture_output=True)
    time.sleep(0.1)

def send_raw(bus, data):
    try:
        bus.send(can.Message(
            arbitration_id=MOTOR_ID,
            data=data,
            is_extended_id=False), timeout=0.2)
        return True
    except Exception:
        return False

def get_reply(bus, timeout=0.05):
    deadline = time.time() + timeout
    while time.time() < deadline:
        remaining = deadline - time.time()
        if remaining <= 0: break
        msg = bus.recv(timeout=remaining)
        if msg is None: break
        if (len(msg.data) == 8
                and not msg.is_extended_id
                and msg.data[0] == MOTOR_ID
                and msg.data[0] != 0xFF):
            return decode_reply(msg.data)
    return None

def enable_motor(bus):
    send_raw(bus, [0xFF]*7 + [0xFC])
    time.sleep(0.1)
    return get_reply(bus, timeout=1.0)

def disable_motor(bus):
    for _ in range(3):
        send_raw(bus, [0xFF]*7 + [0xFD])
        time.sleep(0.05)

def set_zero(bus):
    send_raw(bus, [0xFF]*7 + [0xFE])
    time.sleep(0.1)

# ── Quintic polynomial ──────────────────────────────────────────────────
def quintic_coeffs(theta0, thetaf, T):
    """
    Compute coefficients for:
      θ(t) = a0 + a1t + a2t² + a3t³ + a4t⁴ + a5t⁵

    Variables:
      theta0  = start angle (rad)
      thetaf  = end angle (rad)
      T       = total time (seconds)
      d       = displacement = thetaf - theta0 (rad)

    Boundary conditions solved:
      θ(0)=theta0, θ̇(0)=0, θ̈(0)=0
      θ(T)=thetaf, θ̇(T)=0, θ̈(T)=0
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

    Variables:
      coeffs  = (a0,a1,a2,a3,a4,a5)
      t       = time within segment (0 to T)

    Returns:
      pos     = planned position at t (rad)   → sent as p_plan
      vel     = planned velocity at t (rad/s) → sent as v_plan (feedforward)
      acc     = planned acceleration (rad/s²) → plotted only
    """
    a0, a1, a2, a3, a4, a5 = coeffs
    pos = a0+a1*t+a2*t**2+a3*t**3+a4*t**4+a5*t**5
    vel = a1+2*a2*t+3*a3*t**2+4*a4*t**3+5*a5*t**4
    acc = 2*a2+6*a3*t+12*a4*t**2+20*a5*t**3
    return pos, vel, acc

def generate_trajectory(theta0, thetaf, T, dt=0.02):
    """
    Generate list of setpoints spaced dt seconds apart.

    Variables:
      theta0  = start angle (rad)
      thetaf  = end angle (rad)
      T       = total duration (seconds)
      dt      = time step (seconds), default 0.02 = 50Hz
      coeffs  = quintic coefficients for this move
      steps   = total number of frames = int(T/dt)+1
      traj    = output list of (t, pos, vel, acc) tuples
                one tuple per CAN frame to be sent
    """
    coeffs = quintic_coeffs(theta0, thetaf, T)
    steps  = int(T / dt) + 1
    traj   = []
    for i in range(steps):
        t             = min(i * dt, T)
        pos, vel, acc = quintic_eval(coeffs, t)
        traj.append((t, pos, vel, acc))
    return traj

# ── Execute trajectory WITH velocity feedback correction ────────────────
def execute_trajectory(bus, traj, kp=40.0, kd=1.5,
                       k_vel=2.0):
    """
    Send trajectory to motor with velocity feedback correction.

    THE KEY IMPROVEMENT — velocity command with error correction:

      pos_error = p_plan - p_actual        (how far behind motor is)
      v_cmd     = v_plan + k_vel * pos_error  (corrected velocity)

    This means:
      Motor lagging (pos_error > 0):
        v_cmd > v_plan → tell motor to move faster → catches up

      Motor ahead (pos_error < 0):
        v_cmd < v_plan → tell motor to slow down → stays on curve

      Motor on track (pos_error = 0):
        v_cmd = v_plan → pure feedforward, no correction needed

    Variables:
      bus        = CAN bus object (open socket to can1)
      traj       = list of (t, p_plan, v_plan, a_plan) tuples
      kp         = position stiffness in MIT controller (0-500)
      kd         = velocity damping in MIT controller (0-5)
      k_vel      = velocity correction gain (our new gain)
                   how aggressively to correct velocity when lagging
                   too high → oscillation
                   too low  → slow correction, still lags
                   good starting value: 2.0 rad/s per rad of error
                   = 2.0 means: 1 rad of lag → add 2 rad/s to velocity

      start_wall = wall clock time execution began
      t_plan     = planned time for current step (s)
      p_plan     = planned position at current step (rad)
      v_plan     = planned velocity at current step (rad/s)
      a_plan     = planned acceleration (rad/s²) — not sent

      step_deadline = exact wall clock time this step fires
      pos_actual    = what motor actually reports as position (rad)
      vel_actual    = what motor actually reports as velocity (rad/s)

      pos_error  = p_plan - pos_actual
                   positive = motor is behind planned position
                   negative = motor is ahead of planned position

      v_cmd      = v_plan + k_vel * pos_error
                   the corrected velocity command sent to motor
                   clamped to V_MIN..V_MAX before sending

      actual_t   = list of actual timestamps (for plotting)
      actual_pos = list of actual positions  (for plotting)
      actual_vel = list of actual velocities (for plotting)
      errors     = list of position errors   (for plotting)
      v_cmds     = list of corrected velocities sent (for plotting)

      sleep_t    = time to sleep until next step fires
                   = step_deadline - time.time()
    """
    actual_t   = []
    actual_pos = []
    actual_vel = []
    errors     = []     # pos_error at each step
    v_cmds     = []     # v_cmd at each step (after correction)

    # Track last known actual position for error computation
    # when motor does not reply in time
    last_actual_pos = traj[0][1]  # start at first planned position
    last_actual_vel = 0.0

    start_wall = time.time()

    print(f"\n  {'Step':>5}  {'t':>6}  {'p_plan':>8}  "
          f"{'p_actual':>9}  {'error':>7}  "
          f"{'v_plan':>8}  {'v_cmd':>8}")
    print(f"  {'-'*65}")

    for i, (t_plan, p_plan, v_plan, a_plan) in enumerate(traj):

        # ── When should this step fire? ──────────────────────
        step_deadline = start_wall + t_plan

        # ── Compute position error ───────────────────────────
        # pos_error: how far behind the motor is right now
        # positive = motor is lagging (behind planned position)
        # negative = motor is ahead of planned position
        pos_error = p_plan - last_actual_pos

        # ── Velocity correction ──────────────────────────────
        # v_cmd = feedforward + proportional correction
        # v_plan    = what velocity the quintic says we need
        # k_vel     = how aggressively to correct for lag
        # pos_error = how much lag there currently is
        v_cmd = v_plan + k_vel * pos_error

        # Clamp v_cmd to motor velocity limits
        # Cannot command faster than motor can go
        v_cmd = max(V_MIN, min(V_MAX, v_cmd))

        # ── Send command ─────────────────────────────────────
        # p_plan: where to be
        # v_cmd:  corrected velocity (feedforward + correction)
        # kp, kd: gains for motor's internal PD controller
        # 0.0:    no additional feedforward torque
        cmd = pack_mit(p_plan, v_cmd, kp, kd, 0.0)
        send_raw(bus, cmd)

        # ── Read motor reply ─────────────────────────────────
        reply = get_reply(bus, timeout=0.015)
        wall_now = time.time()

        if reply:
            _, pos_actual, vel_actual, torq, temp, err = reply
            elapsed = wall_now - start_wall

            # Update last known position for next step's error
            last_actual_pos = pos_actual
            last_actual_vel = vel_actual

            actual_t.append(elapsed)
            actual_pos.append(pos_actual)
            actual_vel.append(vel_actual)
            errors.append(pos_error)
            v_cmds.append(v_cmd)

            # Print every 10th step
            if i % 10 == 0:
                print(f"  {i:>5}  {elapsed:>6.3f}  "
                      f"{math.degrees(p_plan):>+8.2f}°  "
                      f"{math.degrees(pos_actual):>+9.2f}°  "
                      f"{math.degrees(pos_error):>+7.2f}°  "
                      f"{math.degrees(v_plan):>+8.1f}°/s  "
                      f"{math.degrees(v_cmd):>+8.1f}°/s",
                      end='\r')

            if err != 0:
                print(f"\n  [ERROR] {ERROR_CODES.get(err,err)}")

        # ── Precise timing ───────────────────────────────────
        # Sleep exactly until this step's deadline
        # Keeps send rate at exactly 50Hz
        sleep_t = step_deadline - time.time()
        if sleep_t > 0:
            time.sleep(sleep_t)

    print(f"\n  Trajectory done. {len(actual_t)} replies received.")
    return actual_t, actual_pos, actual_vel, errors, v_cmds

# ── Hold at position ────────────────────────────────────────────────────
def hold_position(bus, pos_rad, kp=40.0, kd=1.5, duration=1.0):
    """
    Hold motor at pos_rad for duration seconds.

    Variables:
      pos_rad  = target hold position (rad)
      kp, kd   = gains for position hold
      duration = how long to hold (seconds)
      cmd      = MIT command with v_des=0 (not moving, just holding)
    """
    cmd   = pack_mit(pos_rad, 0.0, kp, kd, 0.0)
    start = time.time()
    while time.time() - start < duration:
        send_raw(bus, cmd)
        get_reply(bus, timeout=0.02)
        time.sleep(0.02)

# ── Save plot ───────────────────────────────────────────────────────────
def save_plot(traj, actual_t, actual_pos, actual_vel,
              errors, v_cmds,
              theta0_deg, thetaf_deg, T, filename,
              k_vel):
    """
    Five-panel plot showing the effect of velocity correction.

    Variables:
      traj       = planned trajectory list
      actual_t   = timestamps of actual motor replies
      actual_pos = actual positions from motor
      actual_vel = actual velocities from motor
      errors     = pos_error = p_plan - p_actual at each step
      v_cmds     = corrected velocity commands sent
      theta0_deg = start angle in degrees (title)
      thetaf_deg = end angle in degrees (title)
      T          = trajectory duration (title)
      filename   = output PNG filename
      k_vel      = velocity correction gain (shown in title)

    Panel 1: position planned vs actual
    Panel 2: position error (should be smaller with correction)
    Panel 3: velocity — planned, feedforward, and corrected command
    Panel 4: the correction term k_vel × pos_error separately
    Panel 5: acceleration profile (planned only)
    """
    t_plan = [x[0] for x in traj]
    p_plan = [math.degrees(x[1]) for x in traj]
    v_plan = [math.degrees(x[2]) for x in traj]
    a_plan = [math.degrees(x[3]) for x in traj]

    p_actual   = [math.degrees(p) for p in actual_pos]
    v_actual   = [math.degrees(v) for v in actual_vel]
    errors_deg = [math.degrees(e) for e in errors]
    v_cmds_deg = [math.degrees(v) for v in v_cmds]

    # Interpolate planned at actual timestamps for error
    p_plan_interp = np.interp(actual_t, t_plan, p_plan)
    p_error_deg   = [a - pl for a, pl
                     in zip(p_actual, p_plan_interp)]

    # The correction term alone (k_vel × error)
    correction_deg = [math.degrees(k_vel * e) for e in errors]

    fig, axes = plt.subplots(5, 1, figsize=(13, 18))
    fig.suptitle(
        f'Quintic Trajectory with Velocity Feedback Correction\n'
        f'{theta0_deg:.1f}° → {thetaf_deg:.1f}°  '
        f'T={T}s  k_vel={k_vel}',
        fontsize=13, fontweight='bold')

    # Panel 1 — Position
    ax = axes[0]
    ax.plot(t_plan, p_plan, 'b--', linewidth=2,
            label='Planned θ(t)')
    ax.plot(actual_t, p_actual, color='orange',
            linewidth=1.5, label='Actual motor position')
    ax.set_ylabel('Position (deg)', fontsize=11)
    ax.set_title('Position: Planned vs Actual '
                 '(should be closer than before)')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # Panel 2 — Position error
    ax = axes[1]
    ax.plot(actual_t, p_error_deg, color='red',
            linewidth=1.5, label='Error (actual-planned)')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.fill_between(actual_t, p_error_deg, 0,
                    alpha=0.2, color='red')
    ax.set_ylabel('Error (deg)', fontsize=11)
    ax.set_title('Position Tracking Error '
                 '(should be smaller with velocity correction)')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # Panel 3 — Velocity comparison
    ax = axes[2]
    ax.plot(t_plan, v_plan, 'g--', linewidth=2,
            label='Planned v_plan (feedforward only)')
    ax.plot(actual_t, v_cmds_deg, color='blue',
            linewidth=1.5,
            label=f'v_cmd = v_plan + {k_vel}×error (sent)')
    ax.plot(actual_t, v_actual, color='lime',
            linewidth=1.0, alpha=0.7,
            label='Actual velocity (from motor)')
    ax.set_ylabel('Velocity (deg/s)', fontsize=11)
    ax.set_title('Velocity: Feedforward vs Corrected Command vs Actual')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # Panel 4 — Correction term alone
    ax = axes[3]
    ax.plot(actual_t, correction_deg, color='purple',
            linewidth=1.5,
            label=f'Correction = {k_vel} × pos_error')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.fill_between(actual_t, correction_deg, 0,
                    alpha=0.2, color='purple')
    ax.set_ylabel('Correction (deg/s)', fontsize=11)
    ax.set_title('Velocity Correction Term = k_vel × (p_plan - p_actual)\n'
                 'Positive = motor lagging, adding speed to catch up')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    # Panel 5 — Acceleration
    ax = axes[4]
    ax.plot(t_plan, a_plan, color='purple',
            linewidth=2, label='Planned θ̈(t)')
    ax.axhline(0, color='gray', linewidth=0.8)
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_ylabel('Acceleration (deg/s²)', fontsize=11)
    ax.set_title('Planned Acceleration Profile '
                 '(high values = harder for motor to track)')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.4)

    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close()

    # Stats
    if p_error_deg:
        max_e = max(abs(e) for e in p_error_deg)
        rms_e = math.sqrt(sum(e**2 for e in p_error_deg)
                          / len(p_error_deg))
        print(f"\n  Tracking stats:")
        print(f"  Max error : {max_e:.3f}°")
        print(f"  RMS error : {rms_e:.3f}°")
        print(f"  Final err : {p_error_deg[-1]:.3f}°")

    print(f"\n  Plot saved → {filename}")
    print(f"  scp l006@172.16.9.183:~/{filename} .")

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

# ── Main ────────────────────────────────────────────────────────────────
def main():
    print("="*55)
    print("  Quintic Trajectory + Velocity Feedback Correction")
    print("="*55)
    print()
    print("  Velocity command = v_feedforward + k_vel × pos_error")
    print("  k_vel: how aggressively to correct position lag")
    print("  Start with k_vel=2.0, increase if still lagging")

    print("\n  Resetting can1...")
    reset_can()
    print("  can1 ready")

    try:
        bus = can.interface.Bus(channel='can1',
                                interface='socketcan')
    except Exception as e:
        print(f"[FAIL] {e}")
        return

    try:
        print("\n[1] Enabling motor...")
        reply = enable_motor(bus)
        if reply is None:
            print("[FAIL] No reply.")
            return

        _, pos, vel, torq, temp, err = reply
        print(f"[OK] Motor online — "
              f"pos={math.degrees(pos):.2f}°  "
              f"temp={temp}°C")

        print("\n[2] Setting zero...")
        set_zero(bus)
        print("  Zero set.")

        # Get velocity correction gain
        print("\n[3] Velocity correction gain k_vel")
        print("  k_vel = 0.0  → pure feedforward (original)")
        print("  k_vel = 2.0  → moderate correction (recommended)")
        print("  k_vel = 5.0  → aggressive correction")
        print("  Too high → oscillation")
        k_vel = get_float("  Enter k_vel (0.0 to 10.0): ",
                          0.0, 10.0)

        # Position control gains
        print("\n[4] Position gains")
        kp = get_float("  kp (10-100, recommended 60): ",
                       10.0, 100.0)
        kd = get_float("  kd (0.5-5.0, recommended 2.0): ",
                       0.5, 5.0)

        run_number = 1

        while True:
            print(f"\n{'='*55}")
            print(f"  Trajectory #{run_number}  "
                  f"(k_vel={k_vel}  kp={kp}  kd={kd})")
            print(f"{'='*55}")

            # Get target
            theta0_deg = get_float(
                "\n  Start (deg, ±716°): ", -716.0, 716.0)
            thetaf_deg = get_float(
                "  End   (deg, ±716°): ", -716.0, 716.0)
            T = get_float(
                "  Time T (1.0-20.0 s): ", 1.0, 20.0)

            # Warn if T is too small for the distance
            dist_deg = abs(thetaf_deg - theta0_deg)
            min_T_recommended = dist_deg / 60.0
            if T < min_T_recommended:
                print(f"\n  [WARN] T={T}s may be too fast "
                      f"for {dist_deg:.0f}°")
                print(f"  Recommended minimum: "
                      f"T>{min_T_recommended:.1f}s")
                print(f"  Motor may not track — increase T "
                      f"or expect large errors")

            theta0 = math.radians(theta0_deg)
            thetaf = math.radians(thetaf_deg)

            if abs(theta0) > 12.5 or abs(thetaf) > 12.5:
                print("  Out of motor range ±12.5 rad")
                continue

            # Generate trajectory
            traj = generate_trajectory(theta0, thetaf, T)
            print(f"\n  {len(traj)} steps over {T}s at 50Hz")

            # Show peak velocity and acceleration required
            _, _, v_peak, _ = max(traj, key=lambda x: abs(x[2]))
            _, _, _, a_peak = max(traj, key=lambda x: abs(x[3]))
            print(f"  Peak velocity needed   : "
                  f"{math.degrees(v_peak):.1f} °/s")
            print(f"  Peak acceleration needed: "
                  f"{math.degrees(a_peak):.1f} °/s²")
            if abs(a_peak) > math.radians(200):
                print(f"  [WARN] High acceleration — "
                      f"expect tracking error")

            if not get_yes_no("\n  Execute? (y/n): "):
                continue

            # Move to start if needed
            cur_reply = get_reply(bus, timeout=0.1)
            if cur_reply:
                cur_pos = cur_reply[1]
                if abs(cur_pos - theta0) > 0.05:
                    print(f"\n  Moving to start "
                          f"{theta0_deg:.1f}°...")
                    start_traj = generate_trajectory(
                        cur_pos, theta0, 2.0)
                    execute_trajectory(
                        bus, start_traj,
                        kp=kp, kd=kd, k_vel=k_vel)
                    time.sleep(0.5)

            # Execute with velocity correction
            print(f"\n  Running with k_vel={k_vel}...")
            actual_t, actual_pos, actual_vel, \
                errors, v_cmds = execute_trajectory(
                    bus, traj, kp=kp, kd=kd, k_vel=k_vel)

            # Hold at end
            hold_position(bus, thetaf, kp=kp, kd=kd,
                          duration=1.0)

            # Save plot
            filename = f'quintic_corrected_{run_number}.png'
            save_plot(traj, actual_t, actual_pos, actual_vel,
                      errors, v_cmds,
                      theta0_deg, thetaf_deg, T,
                      filename, k_vel)

            run_number += 1

            if not get_yes_no(
                    "\n  Run another? (y/n): "):
                break

    finally:
        print("\n  Disabling motor...")
        disable_motor(bus)
        bus.shutdown()
        print("  Done.")

if __name__ == "__main__":
    main()
