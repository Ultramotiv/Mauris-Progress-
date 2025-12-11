# This code was developed on 6th NOV 2025
# Updated: 7th NOV 2025 – Added +Y range limit (Start to Start+Y_MOVEMENT_LIMIT)
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")
error, version = robot.GetSDKVersion()
print(f"SDK version: {version}")

# --- COORDINATE TRANSFORMATION FUNCTIONS ---
def euler_to_rotation_matrix(rx, ry, rz):
    rx_rad = math.radians(rx); ry_rad = math.radians(ry); rz_rad = math.radians(rz)
    Rx = np.array([[1,0,0],
                   [0,math.cos(rx_rad),-math.sin(rx_rad)],
                   [0,math.sin(rx_rad),math.cos(rx_rad)]])
    Ry = np.array([[math.cos(ry_rad),0,math.sin(ry_rad)],
                   [0,1,0],
                   [-math.sin(ry_rad),0,math.cos(ry_rad)]])
    Rz = np.array([[math.cos(rz_rad),-math.sin(rz_rad),0],
                   [math.sin(rz_rad),math.cos(rz_rad),0],
                   [0,0,1]])
    return Rz @ Ry @ Rx

def transform_forces_to_tcp_and_world(ft_forces, tcp_orientation, verbose=False):
    rx, ry, rz = tcp_orientation
    R = euler_to_rotation_matrix(rx, ry, rz)
    tcp_force_vector = np.array([+ft_forces[0], +ft_forces[1], -ft_forces[2]])
    tcp_moment_vector = np.array([+ft_forces[3], -ft_forces[4], -ft_forces[5]])
    tcp_forces = list(tcp_force_vector) + list(tcp_moment_vector)
    world_force_vector = R @ tcp_force_vector
    world_moment_vector = R @ tcp_moment_vector
    world_forces = list(world_force_vector) + list(world_moment_vector)
    return tcp_forces, world_forces

# --- FT SENSOR INITIALIZATION ---
def init_ft_sensor():
    company = 24; device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0); time.sleep(0.5)
    robot.FT_Activate(1); time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0); robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0); time.sleep(0.5)
    robot.FT_SetZero(1); time.sleep(0.5)
    print("FT Sensor initialized and zeroed.")

def calibrate_baseline_forces(samples=100):
    print("Calibrating baseline forces (gravity compensation)...")
    force_samples = []
    for _ in range(samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:
            force_samples.append(ft_data[1][:6])
        time.sleep(0.01)
    if force_samples:
        baseline = [sum(col)/len(force_samples) for col in zip(*force_samples)]
        print(f"Baseline forces captured: {[f'{v:.2f}' for v in baseline]}")
        return baseline
    print("Warning: Could not capture baseline forces!")
    return [0.0]*6

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    print("\nShutting down gracefully...")
    sys.exit(0)
signal.signal(signal.SIGINT, shutdown)

# ============================================================================
# MAIN PROGRAM
# ============================================================================
current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:", current_pose)

init_ft_sensor()

# --- GET INITIAL TCP (THIS IS THE START OF THE RANGE) ---
tcp_result = robot.GetActualTCPPose(flag=1)
error_code = tcp_result[0]
current_tcp = tcp_result[1] if len(tcp_result) > 1 else None
if error_code != 0 or current_tcp is None or len(current_tcp) < 6:
    print(f"\nFailed to get current TCP position (error code: {error_code})")
    sys.exit(1)

y_start = current_tcp[1]  # This is the START Y
print(f"\nCurrent TCP Position [X, Y, Z, Rx, Ry, Rz]: {current_tcp}")

# --- DEFINE +Y LIMIT (CHANGE THIS VALUE) ---
Y_MOVEMENT_LIMIT = 300.0   # mm  ← YOUR DESIRED +Y TRAVEL
Y_END = y_start + Y_MOVEMENT_LIMIT  # This is the END Y

print(f"\nY-AXIS MOVEMENT RANGE:")
print(f"   START Y = {y_start:.2f} mm")
print(f"   END Y   = {Y_END:.2f} mm")
print(f"   MAX +Y  = +{Y_MOVEMENT_LIMIT:.1f} mm")
print("="*60)

baseline_forces = calibrate_baseline_forces()

# --- MOTION PARAMETERS ---
LINEAR_VEL = 5.0
FORCE_THRESHOLD = 2.0
DEADBAND = 0.5
FORCE_TO_VELOCITY_SCALE = 10.0
UPDATE_RATE = 0.008
M = [0.5, 0.5, 0.3, 0.5, 0.5, 0.5]  # Lower mass = more responsive
B = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5]  # Lower damping = less resistance
K = [0.0]*6  # Keep zero for free motion
# M = [3.0, 2.0, 2.0, 2.0, 2.0, 3.0] #[3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
# B = [150.0, 150.0, 150.0, 5.0, 5.0, 1.0] #[2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
# K = [0.0]*6
rtn = robot.ForceAndJointImpedanceStartStop(1, 0, M, K, B, 100, 100)
print(f"ForceAndJointImpedanceStartStop rtn is {rtn}")

print("\n" + "="*60)
print("FORCE-CONTROLLED Y-MOTION (RANGE LIMITED)")
print("="*60)
print(f"Allowed Y range: {y_start:.1f} to {Y_END:.1f} mm")
print("Apply force in +Y to move forward (cannot go beyond limit)")
print("Press Ctrl+C to stop")
print("="*60 + "\n")

# --- MAIN CONTROL LOOP ---
try:
    while True:
        error, current_tcp_pose = robot.GetActualTCPPose()
        if error != 0:
            print(f"TCP pose read error: {error}")
            time.sleep(UPDATE_RATE)
            continue

        tcp_orientation = current_tcp_pose[3:6]
        y_current_live = current_tcp_pose[1]

        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(UPDATE_RATE)
            continue

        raw_forces = ft_data[1][:6]
        compensated_forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        for i in range(6):
            if abs(compensated_forces[i]) < DEADBAND:
                compensated_forces[i] = 0.0

        tcp_forces, world_forces = transform_forces_to_tcp_and_world(compensated_forces, tcp_orientation)
        fy_world = world_forces[1]

        if abs(fy_world) > FORCE_THRESHOLD:
            y_movement = fy_world * FORCE_TO_VELOCITY_SCALE
            y_target_raw = y_current_live + y_movement

            # --- CLAMP TO ALLOWED RANGE ---
            y_target = np.clip(y_target_raw, y_start, Y_END)

            if y_target != y_target_raw:
                if y_target_raw > Y_END:
                    print(f"LIMIT REACHED: Y clamped to {Y_END:.2f} mm (end of range)")
                elif y_target_raw < y_start:
                    print(f"LIMIT REACHED: Y clamped to {y_start:.2f} mm (start of range)")

            target_desc_pos = [
                current_tcp_pose[0],
                y_target,  # Clamped Y
                current_tcp_pose[2],
                current_tcp_pose[3],
                current_tcp_pose[4],
                current_tcp_pose[5]
            ]

            print(f"Fy={fy_world: .2f}N → ΔY={y_movement: .2f}mm → Y_target={y_target:.2f}mm "
                  f"[Range: {y_start:.1f} to {Y_END:.1f}]")

            ret_linear = robot.MoveL(
                desc_pos=target_desc_pos,
                tool=0, user=0, joint_pos=[0.0]*7,
                vel=LINEAR_VEL, acc=0.0, ovl=100.0,
                blendR=-1.0, exaxis_pos=[0.0]*4,
                search=0, offset_flag=0, offset_pos=[0.0]*6
            )
            if ret_linear != 0:
                print(f"MoveL failed with error code: {ret_linear}")
        else:
            print(f"Holding position (Fy={fy_world:.2f}N < {FORCE_THRESHOLD}N)")

        time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    print("\n\nMotion control stopped by user")
    print("="*60)
    final_tcp_result = robot.GetActualTCPPose(flag=1)
    if final_tcp_result[0] == 0 and len(final_tcp_result) > 1:
        final_tcp = final_tcp_result[1]
        print(f"\nFinal TCP Position [X, Y, Z, Rx, Ry, Rz]: {final_tcp}")
        total_movement = final_tcp[1] - y_start
        print(f"\nTotal Y movement: {total_movement:.2f} mm")
        print(f" Initial Y: {y_start:.2f} mm")
        print(f" Final Y:   {final_tcp[1]:.2f} mm")
        print(f" Allowed range was: {y_start:.1f} to {Y_END:.1f} mm")
    print("="*60)