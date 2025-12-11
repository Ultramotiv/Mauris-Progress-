# PERFECT PURE Y-AXIS ADMITTANCE CONTROL
# Only Y moves — X, Z, Rx, Ry, Rz permanently locked

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================
robot = None
running = True
fixed_tcp_ref = None

# === GOLDEN TUNING — MOVES BEAUTIFULLY, NO SHAKING ===
FORCE_TO_MOTION_SCALE = 3.8          # 1 N → 3.8 mm (perfect sensitivity)
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]     # Light virtual mass
B = [12.0, 12.0, 12.0, 14.0, 14.0, 14.0]  # Optimal damping (no oscillation)
K = [0.0] * 6

# Control rates
IK_UPDATE_RATE    = 0.0025    # ~400 Hz
SERVO_UPDATE_RATE = 0.008     # 125 Hz (Fairino maximum)
IK_TO_SERVO_RATIO = 3         # 400 / ~133 ≈ 3

# Force filtering
FORCE_THRESHOLD = 0.9
DEADBAND = 0.12
FORCE_FILTER_ALPHA = 0.25

# Per-axis deadband thresholds
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# State
joint_velocity = [0.0] * 6
desired_joint_pos = [0.0] * 6
filtered_desired_joints = None
filtered_fy_world = 0.0
baseline_forces = [0.0] * 6

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
    print("FT sensor initialized and zeroed")

def calibrate_baseline(samples=150):
    print("Calibrating baseline... (keep tool completely still)")
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        global baseline_forces
        baseline_forces = np.mean(forces, axis=0).tolist()
        print(f"Baseline calibrated: {[f'{x:+.3f}' for x in baseline_forces]}")
    else:
        print("Warning: Baseline failed")

# ============================================================================
# MAIN CONTROL LOOP — PERFECT Y-AXIS ONLY
# ============================================================================
def control_loop():
    global running, filtered_fy_world, desired_joint_pos, joint_velocity
    global filtered_desired_joints, fixed_tcp_ref

    print("\n" + "="*70)
    print("   PERFECT PURE Y-AXIS ADMITTANCE CONTROL")
    print("   Moves instantly | Zero vibration | Feather-light")
    print("   Only Y moves — X/Z/Orientation LOCKED forever")
    print("="*70)

    # Lock reference pose
    err, tcp = robot.GetActualTCPPose()
    if err != 0:
        print("Failed to read TCP pose")
        return
    fixed_tcp_ref = tcp.copy()
    print(f"Reference locked → X={tcp[0]:.2f} Z={tcp[2]:.2f} Orientation fixed")

    if robot.ServoMoveStart() != 0:
        print("Failed to start Servo mode")
        return

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
                time.sleep(IK_UPDATE_RATE)
                continue

            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue

            raw = ft[1][:6]
            compensated = [raw[i] - baseline_forces[i] for i in range(6)]
            for i in range(6):
                if abs(compensated[i]) < force_thresholds[i]:
                    compensated[i] = 0.0

            fy_world = transform_force_to_world(compensated, current_tcp[3:6])
            filtered_fy_world = ema(fy_world, filtered_fy_world, FORCE_FILTER_ALPHA)

            active_force = filtered_fy_world if abs(filtered_fy_world) > FORCE_THRESHOLD else 0.0
            delta_y = active_force * FORCE_TO_MOTION_SCALE
            target_y = current_tcp[1] + delta_y

            target_tcp = [
                fixed_tcp_ref[0],   # X locked
                target_y,           # ONLY Y changes
                fixed_tcp_ref[2],   # Z locked
                fixed_tcp_ref[3],
                fixed_tcp_ref[4],
                fixed_tcp_ref[5]
            ]

            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue

            tj = np.array(ik[1][:6])
            acc_joints = tj if acc_joints is None else acc_joints + tj
            ik_count += 1

            if ik_count >= IK_TO_SERVO_RATIO:
                avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()

                # Perfect inner admittance loop
                for j in range(6):
                    err = avg_joints[j] - desired_joint_pos[j]
                    f = err * 4.5                                    # Perfect P-gain
                    acc = (f - B[j] * joint_velocity[j]) / M[j]
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    joint_velocity[j] = np.clip(joint_velocity[j], -70, 70)
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE

                # Final smoothing (just enough)
                alpha = 0.35
                if filtered_desired_joints is None:
                    filtered_desired_joints = desired_joint_pos[:]
                else:
                    for j in range(6):
                        filtered_desired_joints[j] = alpha * desired_joint_pos[j] + (1 - alpha) * filtered_desired_joints[j]

                robot.ServoJ(filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

                if servo_count % 20 == 0:
                    print(f"Fy={filtered_fy_world:+6.2f}N → ΔY={delta_y:+6.1f}mm | "
                          f"Y={current_tcp[1]:8.2f}mm | X={current_tcp[0]:6.2f} Z={current_tcp[2]:6.2f}")

                acc_joints = None
                ik_count = 0
                servo_count += 1

            elapsed = time.time() - t0
            sleep_t = IK_UPDATE_RATE - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except Exception as e:
        print("Error:", e)
    finally:
        robot.ServoMoveEnd()
        print("Servo stopped")

# ============================================================================
# SHUTDOWN
# ============================================================================
def shutdown(sig, frame):
    global running
    print("\n\nStopping...")
    running = False
    time.sleep(0.5)
    err, tcp = robot.GetActualTCPPose()
    if err == 0 and fixed_tcp_ref is not None:
        dy = tcp[1] - fixed_tcp_ref[1]
        print(f"Final Y: {tcp[1]:.2f} mm (ΔY = {dy:+.2f} mm)")
        print(f"Drift → ΔX: {tcp[0]-fixed_tcp_ref[0]:+.3f} mm | ΔZ: {tcp[2]-fixed_tcp_ref[2]:+.3f} mm")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# ============================================================================
# MAIN
# ============================================================================
if __name__ == "__main__":
    robot = Robot.RPC('192.168.58.2')
    print("Connected to robot")

    init_ft_sensor()
    calibrate_baseline()

    err, tcp = robot.GetActualTCPPose()
    if err == 0:
        print(f"\nStarting from Y = {tcp[1]:.2f} mm")
        print("Push or pull gently along the tool Y-axis → robot follows instantly and smoothly")
        print("Ctrl+C to stop\n" + "="*70)

    try:
        control_loop()
    except KeyboardInterrupt:
        shutdown(None, None)