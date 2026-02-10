# FAIRINO COBOT — PURE X+Y ADMITTANCE CONTROL
# Velocity increases smoothly with applied force in X or Y → feels PERFECT
# Absolutely no vibrations | Push harder = move faster
# Only in X and Y movement | MAX JOINT VELOCITY: 60 deg/s

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# ============================================================================
# GLOBAL VARIABLES & TUNING (X+Y VERSION)
# ============================================================================
robot = None
running = True
fixed_tcp_ref = None

# TUNING — Optimized for X+Y admittance
FORCE_TO_MOTION_SCALE = 6.0          # mm per Newton in X/Y
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]   # Virtual mass
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
filtered_fx_world = 0.0
filtered_fy_world = 0.0
baseline_forces = [0.0] * 6

MAX_JOINT_VELOCITY = 40.0  # deg/s

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

def transform_forces_to_world(ft_forces, orientation):
    R = euler_to_rotation_matrix(*orientation)
    tcp_force = np.array([ft_forces[0], ft_forces[1], -ft_forces[2]])
    world_force = R @ tcp_force
    return world_force[0], world_force[1]  # Fx, Fy in world frame

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
# MAIN LOOP — X+Y ADMITTANCE (NO Z)
# ============================================================================
def control_loop():
    global running, filtered_fx_world, filtered_fy_world
    global desired_joint_pos, joint_velocity, filtered_desired_joints, fixed_tcp_ref

    print("\n" + "="*70)
    print(" X+Y ADMITTANCE CONTROL — NO Z MOVEMENT")
    print(" Push in X/Y → robot follows smoothly in world X/Y")
    print(" Z and orientation locked | Max joint speed: 60°/s")
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

            # Apply deadzone
            for i in range(6):
                if abs(compensated[i]) < force_thresholds[i]:
                    compensated[i] = 0.0

            # Get world-frame X and Y forces
            fx_world, fy_world = transform_forces_to_world(compensated, current_tcp[3:6])
            filtered_fx_world = ema(fx_world, filtered_fx_world, FORCE_FILTER_ALPHA)
            filtered_fy_world = ema(fy_world, filtered_fy_world, FORCE_FILTER_ALPHA)

            # Apply force threshold
            active_fx = filtered_fx_world if abs(filtered_fx_world) > FORCE_THRESHOLD else 0.0
            active_fy = filtered_fy_world if abs(filtered_fy_world) > FORCE_THRESHOLD else 0.0

            # Compute target position: only X and Y change
            delta_x = active_fx * FORCE_TO_MOTION_SCALE
            delta_y = active_fy * FORCE_TO_MOTION_SCALE

            target_x = current_tcp[0] + delta_x
            target_y = current_tcp[1] + delta_y

            target_tcp = [
                target_x,
                target_y,
                fixed_tcp_ref[2],  # Fixed Z
                fixed_tcp_ref[3],
                fixed_tcp_ref[4],
                fixed_tcp_ref[5]   # Fixed orientation
            ]

            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0:
                time.sleep(IK_UPDATE_RATE); continue

            tj = np.array(ik[1][:6])
            acc_joints = tj if acc_joints is None else acc_joints + tj
            ik_count += 1

            if ik_count >= IK_TO_SERVO_RATIO:
                avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()

                # Admittance control with joint velocity limiting
                for j in range(6):
                    err_joint = avg_joints[j] - desired_joint_pos[j]
                    f = err_joint * 3.9
                    acc = (f - B[j] * joint_velocity[j]) / M[j]
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    joint_velocity[j] = np.clip(joint_velocity[j], -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE

                # Smooth output
                alpha = 0.32
                if filtered_desired_joints is None:
                    filtered_desired_joints = desired_joint_pos[:]
                else:
                    for j in range(6):
                        filtered_desired_joints[j] = (
                            alpha * desired_joint_pos[j] + (1 - alpha) * filtered_desired_joints[j]
                        )

                robot.ServoJ(filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

                if servo_count % 20 == 0:
                    max_jv = max(abs(v) for v in joint_velocity)
                    print(
                        f"Fx={filtered_fx_world:+6.2f}N Fy={filtered_fy_world:+6.2f}N Current TCP position is :->  "
                        f"X={current_tcp[0]:8.2f}mm Y={current_tcp[1]:8.2f}mm | "
                        f"MaxVel={max_jv:5.1f}°/s (LIMIT: {MAX_JOINT_VELOCITY}°/s)"
                    )

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
        dx = tcp[0] - fixed_tcp_ref[0]
        dy = tcp[1] - fixed_tcp_ref[1]
        print(f"Final ΔX = {dx:+.2f} mm, ΔY = {dy:+.2f} mm")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# ============================================================================
# MAIN
# ============================================================================
if __name__ == "__main__":
    robot = Robot.RPC('192.168.58.2')
    print("Connected to Fairino Cobot")
    init_ft_sensor()
    calibrate_baseline()
    print("\nSTARTING X+Y ADMITTANCE CONTROL (NO Z)")
    print("Push in any horizontal direction → robot follows naturally!")
    try:
        control_loop()
    except KeyboardInterrupt:
        shutdown(None, None)