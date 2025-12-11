# FAIRINO COBOT — PURE Y-AXIS ADMITTANCE WITH 1KG RESISTANCE
# Velocity increases smoothly with applied force → feels PERFECT
# Robot applies 1kg (10N) opposing force for resistance effect
# absolutely no vibrations | Push harder = move faster
# Only in Y movement | MAX VELOCITY: 60deg/s

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# ============================================================================
# GLOBAL VARIABLES & TUNING (Y-AXIS VERSION)
# ============================================================================
robot = None
running = True
fixed_tcp_ref = None

# TUNING — Optimized for pure Y-axis admittance
FORCE_TO_MOTION_SCALE = 6.0          # How much Y movement per Newton
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]   # Virtual mass (light = responsive)
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]   # Damping per joint
IK_TO_SERVO_RATIO = 2
IK_UPDATE_RATE = 0.0025
SERVO_UPDATE_RATE = 0.008
FORCE_THRESHOLD = 0.8
FORCE_FILTER_ALPHA = 0.28
force_thresholds = [2.0, 2.0, 2.5, 1.0, 1.0, 1.0]  # Deadzones per axis
joint_velocity = [0.0] * 6
desired_joint_pos = [0.0] * 6
filtered_desired_joints = None
filtered_fy_world = 0.0
baseline_forces = [0.0] * 6

# NEW: Safe joint velocity limit
MAX_JOINT_VELOCITY = 60.0  # deg/s (safe operating speed)

# ============================================================================
# HELPERS
# ============================================================================
def euler_to_rotation_matrix(rx, ry, rz):
    rx = math.radians(rx); ry = math.radians(ry); rz = math.radians(rz)
    c, s = math.cos, math.sin
    Rx = np.array([[1,0,0], [0,c(rx),-s(rx)], [0,s(rx),c(rx)]])
    Ry = np.array([[c(ry),0,s(ry)], [0,1,0], [-s(ry),0,c(ry)]])
    Rz = np.array([[c(rz),-s(rz),0], [s(rz),c(rz),0], [0,0,1]])
    return Rz @ Ry @ Rx

def transform_force_to_world(ft_forces, orientation):
    R = euler_to_rotation_matrix(*orientation)
    tcp_force = np.array([ft_forces[0], ft_forces[1], -ft_forces[2]])
    world_force = R @ tcp_force
    return world_force[1]

def ema(new, old, alpha):
    return alpha * new + (1 - alpha) * old

# ============================================================================
# FT SENSOR
# ============================================================================
def init_ft_sensor():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor ready")

def calibrate_baseline(samples=150):
    print("Calibrating baseline... (keep tool still)")
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        global baseline_forces
        baseline_forces = np.mean(forces, axis=0).tolist()
        print(f"Baseline: {[f'{x:+.3f}' for x in baseline_forces]}")

# ============================================================================
# RESISTANCE FUNCTION — Applies 1kg (10N) opposing force
# ============================================================================
# def apply_resistance_force(current_fy_world):
#     """
#     When external force is detected on Y-axis, apply 1kg (10N) resistance
#     in the opposite direction.
    
#     Args:
#         current_fy_world: Current force detected in world Y-axis (Newtons)
    
#     Returns:
#         resistance_motion: Motion in -Y direction (mm) to create resistance
#     """
#     if abs(current_fy_world) < FORCE_THRESHOLD:
#         return 0.0
    
#     # 1kg = 10N resistance in opposite direction
#     RESISTANCE_FORCE = 30.0  # Newtons
    
#     # Determine direction: if positive force detected, push back negatively
#     resistance_direction = -1.0 if current_fy_world > 0 else 1.0
    
#     # Calculate resistance motion: 10N * 6mm/N = 60mm in opposite direction
#     resistance_motion = resistance_direction * RESISTANCE_FORCE * FORCE_TO_MOTION_SCALE
    
#     return resistance_motion

def apply_resistance_force(current_fy_world):
    """
    When external force is detected on Y-axis, apply constant resistance
    ALWAYS in +Y direction (regardless of push/pull direction).
    
    Args:
        current_fy_world: Current force detected in world Y-axis (Newtons)
    
    Returns:
        resistance_motion: Motion in +Y direction (mm) to create resistance
    """
    if abs(current_fy_world) < FORCE_THRESHOLD:
        return 0.0
    
    # 3kg = 30N resistance ALWAYS in +Y direction
    RESISTANCE_FORCE = 10.0  # Newtons
    
    # Always apply force in POSITIVE Y direction
    resistance_direction = 1.0
    
    # Calculate resistance motion: 30N * 6mm/N = 180mm in +Y direction
    resistance_motion = resistance_direction * RESISTANCE_FORCE * FORCE_TO_MOTION_SCALE
    
    return resistance_motion


# ============================================================================
# MAIN LOOP — PURE Y-AXIS ADMITTANCE WITH 1KG RESISTANCE
# ============================================================================
def control_loop():
    global running, filtered_fy_world, desired_joint_pos, joint_velocity
    global filtered_desired_joints, fixed_tcp_ref
    global previous_tcp_y, previous_time

    print("\n" + "="*70)
    print(" PERFECT Y-AXIS ADMITTANCE CONTROL WITH 1KG RESISTANCE")
    print(" Push forward/backward on tool → robot resists with 1kg force")
    print(" Gentle touch → slow | Strong push → faster (safe limit: 60°/s)")
    print("="*70)

    err, tcp = robot.GetActualTCPPose()
    if err != 0: return
    fixed_tcp_ref = tcp.copy()

    if robot.ServoMoveStart() != 0: return

    j = robot.GetActualJointPosDegree(flag=0)
    if j[0] == 0:
        desired_joint_pos[:] = j[1][:6]
    filtered_desired_joints = desired_joint_pos[:]

    acc_joints = None
    ik_count = servo_count = 0

    try:
        while running:
            t0 = time.time()

            err, current_tcp = robot.GetActualTCPPose()
            if err != 0: 
                time.sleep(IK_UPDATE_RATE); continue

            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] != 0: 
                time.sleep(IK_UPDATE_RATE); continue

            raw = ft[1][:6]
            compensated = [raw[i] - baseline_forces[i] for i in range(6)]

            # Deadzone
            for i in range(6):
                if abs(compensated[i]) < force_thresholds[i]:
                    compensated[i] = 0.0

            # Transform force to world Y
            fy_world = transform_force_to_world(compensated, current_tcp[3:6])
            filtered_fy_world = ema(fy_world, filtered_fy_world, FORCE_FILTER_ALPHA)

            active_force = filtered_fy_world if abs(filtered_fy_world) > FORCE_THRESHOLD else 0.0

            # Calculate resistance motion (1kg = 10N opposing force)
            resistance_motion = apply_resistance_force(filtered_fy_world)

            # Motion in Y direction (positive force → move +Y)
            delta_y = active_force * FORCE_TO_MOTION_SCALE
            
            # Apply resistance: subtract the opposing motion
            delta_y += resistance_motion

            target_y = current_tcp[1] + delta_y

            # Lock X, Z, and orientation — only Y changes
            target_tcp = [fixed_tcp_ref[0], 
                          target_y, 
                          fixed_tcp_ref[2],
                          fixed_tcp_ref[3], 
                          fixed_tcp_ref[4], 
                          fixed_tcp_ref[5]]

            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0: 
                time.sleep(IK_UPDATE_RATE); continue

            tj = np.array(ik[1][:6])
            acc_joints = tj if acc_joints is None else acc_joints + tj
            ik_count += 1

            if ik_count >= IK_TO_SERVO_RATIO:
                avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()

                # Apply admittance control with SAFE 60°/s joint velocity limit
                for j in range(6):
                    err = avg_joints[j] - desired_joint_pos[j]
                    f = err * 3.9
                    acc = (f - B[j] * joint_velocity[j]) / M[j]
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    # CLAMP to safe 60°/s maximum
                    joint_velocity[j] = np.clip(joint_velocity[j], -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE

                # Output smoothing
                alpha = 0.32
                if filtered_desired_joints is None:
                    filtered_desired_joints = desired_joint_pos[:]
                else:
                    for j in range(6):
                        filtered_desired_joints[j] = alpha * desired_joint_pos[j] + (1 - alpha) * filtered_desired_joints[j]

                robot.ServoJ(filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

                if servo_count % 20 == 0:
                    max_jv = max(abs(v) for v in joint_velocity)
                    resistance = apply_resistance_force(filtered_fy_world)
                    print(f"Fy={filtered_fy_world:+6.2f}N → Y={current_tcp[1]:8.2f}mm | "
                          f"Resistance={resistance:+6.2f}mm | MaxJointVel={max_jv:5.1f}°/s")

                acc_joints = None
                ik_count = 0
                servo_count += 1

            sleep_t = IK_UPDATE_RATE - (time.time() - t0)
            if sleep_t > 0: 
                time.sleep(sleep_t)

    except Exception as e:
        print("Error:", e)
    finally:
        robot.ServoMoveEnd()

# ============================================================================
# SHUTDOWN
# ============================================================================
def shutdown(sig, frame):
    global running
    print("\nStopping...")
    running = False
    time.sleep(0.5)
    err, tcp = robot.GetActualTCPPose()
    if err == 0 and fixed_tcp_ref is not None:
        dy = tcp[1] - fixed_tcp_ref[1]
        print(f"Final ΔY = {dy:+.2f} mm")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

if __name__ == "__main__":
    robot = Robot.RPC('192.168.58.2')
    print("Connected to Fairino Cobot")
    init_ft_sensor()
    calibrate_baseline()
    print("\nSTARTING PURE Y-AXIS ADMITTANCE WITH 1KG RESISTANCE")
    print("Push/pull the tool along Y → robot resists with 1kg force!")
    try:
        control_loop()
    except KeyboardInterrupt:
        shutdown(None, None)