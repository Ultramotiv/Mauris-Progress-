import math
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- CARTESIAN IMPEDANCE PARAMETERS ---
# FT sensor forces drive TCP motion in world coordinates.
# Inverse kinematics computes joint angles so ALL joints move naturally.
#
# Impedance per Cartesian DOF:  M * acc + B * vel + K * disp = F_scaled
# Translation: [x, y, z] in mm    |  Rotation: [rx, ry, rz] in deg

M_cart = [5.0, 5.0, 5.0, 2.0, 2.0, 2.0]      # Virtual mass  [x, y, z, rx, ry, rz]
B_cart = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]  #[20.0, 20.0, 20.0, 10.0, 10.0, 10.0] # Damping     these will give much more sensetive movement ->>> [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
K_cart = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]      # Spring stiffness (0 = free, no return to home)

# Force scaling: how responsive TCP motion is to applied forces
# 1 N steady push → 6 mm/s  (force_to_translation / B_cart = 120/20 = 6)
# Time constant τ = M/B = 5/20 = 0.25s (smooth ramp-up, no jerk)
force_to_translation = 120.0   # Scales N  → mm/s²  (higher = more responsive)
force_to_rotation    = 6.0     # Scales Nm → deg/s²

# Deadband and thresholds
force_deadband  = 0.5          # N/Nm — noise rejection
force_threshold = 1.0          # N/Nm — below this, only damping (robot decelerates)

# Velocity limits (safety)
max_linear_vel  = 200.0         # mm/s  — max TCP translation speed
max_angular_vel = 26.0         # deg/s — max TCP rotation speed

# IK safety: max joint change per control cycle (rejects discontinuous IK solutions)
max_joint_step = 15.0           # degrees

# Joint command smoothing (EMA low-pass filter)
# Prevents IK jitter — especially on wrist joints (J4, J5, J6)
# filtered = alpha * new_ik + (1 - alpha) * previous
# Lower alpha = smoother but slower response
joint_filter_alpha = [0.4, 0.4, 0.4, 0.3, 0.15, 0.3]  # per-joint (J5 extra smooth)

# Control loop timing
dt = 0.008

# Gravity compensation
gravity_compensation_samples = 200
baseline_forces = None
startup_delay = 1.0

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# --- COORDINATE TRANSFORMATION FUNCTIONS ---
def euler_to_rotation_matrix(rx, ry, rz):
    """Convert Euler angles (degrees) to rotation matrix (ZYX convention)"""
    rx_rad = math.radians(rx)
    ry_rad = math.radians(ry) 
    rz_rad = math.radians(rz)
    
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(rx_rad), -math.sin(rx_rad)],
        [0, math.sin(rx_rad), math.cos(rx_rad)]
    ])
    
    Ry = np.array([
        [math.cos(ry_rad), 0, math.sin(ry_rad)],
        [0, 1, 0],
        [-math.sin(ry_rad), 0, math.cos(ry_rad)]
    ])
    
    Rz = np.array([
        [math.cos(rz_rad), -math.sin(rz_rad), 0],
        [math.sin(rz_rad), math.cos(rz_rad), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx

# Fixed mounting rotation: FT sensor frame → TCP frame
# Physical mounting:  FT +X → TCP +X,  FT +Y → TCP +Y,  FT +Z → TCP +Z
# (FT sensor axes aligned with TCP axes — no axis flip)
R_FT_TO_TCP = np.array([
    [+1,  0,  0],
    [ 0, +1,  0],
    [ 0,  0, +1]
], dtype=float)

def ft_to_world(ft_forces, tcp_orientation, verbose=False):
    """
    DIRECT mapping: FT sensor → World coordinates (single combined rotation).

    The FT sensor is rigidly mounted on the end-effector with a known fixed
    orientation (R_FT_TO_TCP).  The end-effector orientation in the world
    frame is given by the TCP Euler angles (rx, ry, rz).

    Combined rotation:  R_ft_to_world = R_tcp_to_world @ R_ft_to_tcp
    World forces = R_ft_to_world @ raw_FT_vector

    This guarantees that regardless of TCP pose, a push in a given world
    direction ALWAYS produces the same world-frame force vector.

    Args:
        ft_forces: [Fx, Fy, Fz, Mx, My, Mz] raw from FT sensor
        tcp_orientation: [rx, ry, rz] current TCP Euler angles in degrees
        verbose: print mapping details

    Returns:
        world_forces: [Fx, Fy, Fz, Mx, My, Mz] in world frame
    """
    rx, ry, rz = tcp_orientation

    # TCP-to-World rotation from current end-effector orientation
    R_tcp_to_world = euler_to_rotation_matrix(rx, ry, rz)

    # Single combined rotation:  FT sensor → World
    R_ft_to_world = R_tcp_to_world @ R_FT_TO_TCP

    # Raw FT vectors
    ft_force_vec  = np.array([ft_forces[0], ft_forces[1], ft_forces[2]], dtype=float)
    ft_moment_vec = np.array([ft_forces[3], ft_forces[4], ft_forces[5]], dtype=float)

    # One-step transform to world
    world_force_vec  = R_ft_to_world @ ft_force_vec
    world_moment_vec = R_ft_to_world @ ft_moment_vec

    world_forces = [
        world_force_vec[0],   # Fx_world
        world_force_vec[1],   # Fy_world
        world_force_vec[2],   # Fz_world
        world_moment_vec[0],  # Mx_world
        world_moment_vec[1],  # My_world
        world_moment_vec[2]   # Mz_world
    ]

    if verbose:
        print("\n=== FT → WORLD (direct mapping) ===")
        print(f"TCP orientation: rx={rx:.1f}° ry={ry:.1f}° rz={rz:.1f}°")
        ft_labels = ['FT_Fx', 'FT_Fy', 'FT_Fz', 'FT_Mx', 'FT_My', 'FT_Mz']
        w_labels  = ['W_Fx',  'W_Fy',  'W_Fz',  'W_Mx',  'W_My',  'W_Mz']
        active_ft = [f"{ft_labels[i]}={ft_forces[i]:.2f}" for i in range(6)
                     if abs(ft_forces[i]) > 0.5]
        active_w  = [f"{w_labels[i]}={world_forces[i]:.2f}" for i in range(6)
                     if abs(world_forces[i]) > 0.5]
        if active_ft:
            print(f"  FT raw:   {', '.join(active_ft)}")
        if active_w:
            print(f"  World:    {', '.join(active_w)}")
        # Show which world axis each FT axis currently maps to (first column of R)
        axes = ['X', 'Y', 'Z']
        for col in range(3):
            dominant = np.argmax(np.abs(R_ft_to_world[:, col]))
            sign = '+' if R_ft_to_world[dominant, col] > 0 else '-'
            print(f"  FT {axes[col]} → World {sign}{axes[dominant]} "
                  f"(cos={R_ft_to_world[dominant, col]:.3f})")
        print("====================================\n")

    return world_forces

# --- FT SENSOR INITIALIZATION ---
def init_ft_sensor():
    company = 24
    device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0)
    time.sleep(0.5)
    robot.FT_Activate(1)
    time.sleep(1.0)  # longer settle after activation
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    # First zero — coarse
    robot.FT_SetZero(0)
    time.sleep(0.3)
    robot.FT_SetZero(1)
    time.sleep(1.0)
    # Second zero — fine (sensor has fully settled now)
    robot.FT_SetZero(0)
    time.sleep(0.3)
    robot.FT_SetZero(1)
    time.sleep(1.0)
    print("FT Sensor initialized and double-zeroed.")

# --- GRAVITY COMPENSATION CALIBRATION ---
def calibrate_baseline_forces():
    """Capture baseline forces with proper settling, discard, and validation"""
    global baseline_forces
    print("Calibrating baseline forces (gravity compensation)...")

    # Phase 1: Re-zero the sensor hardware right before calibration
    robot.FT_SetZero(0)
    time.sleep(0.3)
    robot.FT_SetZero(1)
    time.sleep(1.5)  # let sensor fully settle after zero

    # Phase 2: Discard first 50 readings (transient noise after zero)
    discard_count = 50
    print(f"  Discarding first {discard_count} readings...")
    for _ in range(discard_count):
        robot.FT_GetForceTorqueRCS()
        time.sleep(0.01)

    # Phase 3: Collect samples
    print(f"  Collecting {gravity_compensation_samples} samples...")
    force_samples = []
    for i in range(gravity_compensation_samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:  # No error
            forces = [ft_data[1][j] for j in range(6)]
            force_samples.append(forces)
        time.sleep(0.01)

    if not force_samples:
        print("WARNING: Could not capture any baseline forces!")
        baseline_forces = [0.0] * 6
        return

    # Phase 4: Compute mean baseline
    baseline_forces = [sum(f[i] for f in force_samples) / len(force_samples) for i in range(6)]

    # Phase 5: Compute std dev to check noise level
    std_dev = []
    for i in range(6):
        variance = sum((f[i] - baseline_forces[i]) ** 2 for f in force_samples) / len(force_samples)
        std_dev.append(math.sqrt(variance))

    labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
    print(f"  Samples collected: {len(force_samples)}")
    print(f"  Baseline mean:   {', '.join(f'{labels[i]}={baseline_forces[i]:+.3f}' for i in range(6))}")
    print(f"  Baseline stddev: {', '.join(f'{labels[i]}={std_dev[i]:.3f}' for i in range(6))}")

    # Warn if any baseline is suspiciously large (should be near zero after FT_SetZero)
    print("  Gravity compensation calibrated.")

# --- SIGNAL HANDLER (flag-based, avoids crashing mid-RPC) ---
shutdown_requested = False

def shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True
    print("\n[Ctrl+C] Shutdown requested — finishing current cycle...")

signal.signal(signal.SIGINT, shutdown)

# --- SETUP ---
init_ft_sensor()

# Get initial TCP position and calibrate
error, tcp_pos = robot.GetActualTCPPose()
if error != 0:
    print("Error getting TCP position. Exiting.")
    sys.exit(1)

calibrate_baseline_forces()

# Convert TCP pose to joint positions for initial setup
error, initial_joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting initial joint positions. Exiting.")
    sys.exit(1)

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling control...")
time.sleep(startup_delay)
print("\n" + "="*60)
print("CARTESIAN IMPEDANCE CONTROL — ALL JOINTS FREE (IK-based)")
print("="*60)
print(f"Cartesian impedance:  M={M_cart}  B={B_cart}  K={K_cart}")
print(f"Force scaling: translation={force_to_translation}, rotation={force_to_rotation}")
print(f"Max velocity: linear={max_linear_vel} mm/s, angular={max_angular_vel} deg/s")
print(f"IK safety: max joint step per cycle = {max_joint_step}°")
print("FT Sensor → World (direct rotation) → desired TCP pose → IK → ServoJ")
print(f"FT mounting: R_FT_TO_TCP = I (identity — axes aligned)")
print("  Combined: R_ft_to_world = R_tcp_to_world @ R_ft_to_tcp")
print("  Push FT in any direction → always maps to correct world direction")
print("Press Ctrl+C to stop.")
print("="*60 + "\n")

# --- MAIN CONTROL LOOP ---
def control_loop():
    global baseline_forces

    loop_count = 0

    # Cartesian velocity state [vx, vy, vz, vrx, vry, vrz]
    cart_velocity = np.zeros(6)

    # Record home TCP pose for workspace bounds
    error, home_tcp_list = robot.GetActualTCPPose()
    if error != 0:
        print("Error getting home TCP pose. Exiting control loop.")
        return
    home_tcp = np.array(home_tcp_list, dtype=float)

    # IMPORTANT: Track desired TCP ourselves (open-loop integration).
    # This avoids the feedback oscillation caused by reading lagging actual pose.
    desired_tcp = home_tcp.copy()

    # Also read initial joints for IK reference
    error, prev_joints = robot.GetActualJointPosDegree()
    if error != 0:
        print("Error getting initial joints. Exiting.")
        return

    # Joint command filter state (EMA)
    filtered_joints = list(prev_joints)  # start from current position

    print(f"Home TCP: x={home_tcp[0]:.1f} y={home_tcp[1]:.1f} z={home_tcp[2]:.1f} "
          f"rx={home_tcp[3]:.1f} ry={home_tcp[4]:.1f} rz={home_tcp[5]:.1f}")
    print(f"Joint filter alphas: {joint_filter_alpha}  (J5={joint_filter_alpha[4]} — extra smooth)")

    while not shutdown_requested:
        loop_count += 1
        verbose = (loop_count % 125 == 0)

        loop_start = time.time()

        # --- 1. Read actual TCP (orientation only) and joints (for IK reference) ---
        error, actual_tcp_list = robot.GetActualTCPPose()
        if error != 0:
            time.sleep(dt)
            continue
        actual_tcp = np.array(actual_tcp_list, dtype=float)
        # Use actual orientation for force transform (always up-to-date)
        tcp_orientation = list(actual_tcp[3:6])

        error, current_joints = robot.GetActualJointPosDegree()
        if error != 0:
            time.sleep(dt)
            continue

        # --- 2. Read and process FT sensor ---
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt)
            continue

        raw_forces = [ft_data[1][i] for i in range(6)]

        if baseline_forces is not None:
            comp_forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            comp_forces = list(raw_forces)

        for i in range(6):
            if abs(comp_forces[i]) < force_deadband:
                comp_forces[i] = 0.0

        # --- 3. Transform forces: FT sensor → World ---
        world_forces = ft_to_world(comp_forces, tcp_orientation, verbose=verbose)
        world_f = np.array(world_forces, dtype=float)

        # --- 4. Cartesian impedance control ---
        F_scaled = np.zeros(6)
        F_scaled[0:3] = world_f[0:3] * force_to_translation
        F_scaled[3:6] = world_f[3:6] * force_to_rotation

        lin_force_mag = np.linalg.norm(world_f[0:3])
        ang_force_mag = np.linalg.norm(world_f[3:6])

        for i in range(6):
            # Spring force (toward home)
            disp_from_home = desired_tcp[i] - home_tcp[i]
            spring_force = -K_cart[i] * disp_from_home

            # Damping (opposes velocity)
            damping_force = -B_cart[i] * cart_velocity[i]

            # External force (only if above threshold)
            if (i < 3 and lin_force_mag > force_threshold) or \
               (i >= 3 and ang_force_mag > force_threshold):
                ext_force = F_scaled[i]
            else:
                ext_force = 0.0

            total_force = ext_force + spring_force + damping_force

            if M_cart[i] > 0:
                acc = total_force / M_cart[i]
            else:
                acc = 0.0

            cart_velocity[i] += acc * dt

        # Clamp velocities
        for i in range(3):
            cart_velocity[i] = np.clip(cart_velocity[i], -max_linear_vel, max_linear_vel)
        for i in range(3, 6):
            cart_velocity[i] = np.clip(cart_velocity[i], -max_angular_vel, max_angular_vel)

        # --- 5. Integrate desired TCP (open-loop — no feedback oscillation) ---
        desired_tcp += cart_velocity * dt

        # Keep orientation from desired_tcp for smooth trajectory
        # but update it slowly from actual to avoid drift
        for i in range(3, 6):
            desired_tcp[i] = actual_tcp[i]  # orientation tracks actual (no integration drift)

        # --- 6. Inverse kinematics ---
        ik_result = robot.GetInverseKin(0, list(desired_tcp), -1)

        ik_ok = False
        if isinstance(ik_result, (list, tuple)) and len(ik_result) >= 2 and ik_result[0] == 0:
            ik_ok = True
            target_joints = list(ik_result[1][:6])
        else:
            ik_err = ik_result if isinstance(ik_result, int) else ik_result[0]
            if verbose:
                print(f"IK FAILED (err {ik_err}) for TCP "
                      f"[{', '.join(f'{x:.1f}' for x in desired_tcp)}]")
            # Snap desired_tcp back to actual to resync
            desired_tcp[:3] = actual_tcp[:3]
            cart_velocity *= 0.3
            filtered_joints = list(current_joints)  # reset filter state
            robot.ServoJ(current_joints, [0]*6, dt)
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            continue

        # --- 7. Safety: reject large joint jumps ---
        safe = True
        for j in range(6):
            if abs(target_joints[j] - current_joints[j]) > max_joint_step:
                if verbose:
                    print(f"  IK jump J{j+1}: {current_joints[j]:.1f}→{target_joints[j]:.1f}° REJECTED")
                safe = False
                break

        if not safe:
            desired_tcp[:3] = actual_tcp[:3]
            cart_velocity *= 0.3
            filtered_joints = list(current_joints)  # reset filter state
            robot.ServoJ(current_joints, [0]*6, dt)
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)
            continue

        # --- 8. Apply joint EMA filter (anti-jitter, especially J5) ---
        for j in range(6):
            alpha = joint_filter_alpha[j]
            filtered_joints[j] = alpha * target_joints[j] + (1.0 - alpha) * filtered_joints[j]

        # --- 9. Send filtered joint command ---
        err = robot.ServoJ(filtered_joints, [0]*6, dt)
        prev_joints = list(filtered_joints)  # remember for next cycle

        # --- 10. Verbose output ---
        if verbose:
            print(f"\n--- Loop {loop_count} (t={loop_count*dt:.1f}s) ---")
            print(f"Desired TCP: [{', '.join(f'{x:.1f}' for x in desired_tcp)}]")
            print(f"Actual  TCP: [{', '.join(f'{x:.1f}' for x in actual_tcp)}]")
            tracking_err = np.linalg.norm(desired_tcp[:3] - actual_tcp[:3])
            print(f"Tracking error: {tracking_err:.1f} mm")

            labels_f = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
            active = [(labels_f[i], world_f[i]) for i in range(6) if abs(world_f[i]) > 0.5]
            if active:
                print(f"World forces: {', '.join(f'{n}={v:.2f}' for n, v in active)}")

            labels_v = ['vx', 'vy', 'vz', 'vrx', 'vry', 'vrz']
            active_v = [(labels_v[i], cart_velocity[i]) for i in range(6)
                        if abs(cart_velocity[i]) > 0.1]
            if active_v:
                print(f"Cart vel: {', '.join(f'{n}={v:.1f}' for n, v in active_v)}")

            tcp_disp = desired_tcp[:3] - home_tcp[:3]
            active_d = [(labels_f[i], tcp_disp[i]) for i in range(3) if abs(tcp_disp[i]) > 0.5]
            if active_d:
                print(f"Disp from home: {', '.join(f'{n}={d:.1f}mm' for n, d in active_d)}")

            # Show raw IK vs filtered (highlights J5 smoothing)
            print(f"IK raw joints:  [{', '.join(f'{j:.2f}' for j in target_joints)}]")
            print(f"Filtered joints:[{', '.join(f'{j:.2f}' for j in filtered_joints)}]")
            j5_diff = abs(target_joints[4] - filtered_joints[4])
            if j5_diff > 0.05:
                print(f"  J5 filter absorbing {j5_diff:.2f}° of IK jitter")

            joint_delta = [filtered_joints[i] - current_joints[i] for i in range(6)]
            active_j = [(f'J{i+1}', joint_delta[i]) for i in range(6)
                        if abs(joint_delta[i]) > 0.01]
            if active_j:
                print(f"Joint deltas: {', '.join(f'{n}={d:+.2f}°' for n, d in active_j)}")

            if err != 0:
                print(f"ServoJ error: {err}")

        # --- 11. Timing ---
        elapsed = time.time() - loop_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

    # --- GRACEFUL SHUTDOWN (runs after loop exits) ---
    print("\nStopping servo mode...")
    time.sleep(0.05)  # let any in-flight RPC finish
    try:
        robot.ServoMoveEnd()
        print("ServoMoveEnd OK.")
    except Exception as e:
        print(f"ServoMoveEnd error (ignored): {e}")
    print("Control loop exited cleanly.")

control_loop()



