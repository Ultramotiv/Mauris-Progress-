# This code was developed on 6th NOV 2025
# Updated: 10th NOV 2025 – Z-AXIS + CORRECT FORCE DIRECTION
# PUSH DOWN → Robot goes DOWN | PULL UP → Robot goes UP

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# ----------------------------------------------------------------------
# CONNECT TO ROBOT
# ----------------------------------------------------------------------
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")
error, version = robot.GetSDKVersion()
print(f"SDK version: {version}")

# ----------------------------------------------------------------------
# COORDINATE TRANSFORMATION
# ----------------------------------------------------------------------
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

# ----------------------------------------------------------------------
# FT SENSOR
# ----------------------------------------------------------------------
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

# ----------------------------------------------------------------------
# SIGNAL HANDLER (Ctrl+C)
# ----------------------------------------------------------------------
def shutdown(sig, frame):
    print("\nShutting down gracefully...")
    sys.exit(0)
signal.signal(signal.SIGINT, shutdown)

# ----------------------------------------------------------------------
# MAIN – INITIAL SETUP
# ----------------------------------------------------------------------
current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1-J6]:", current_pose)
init_ft_sensor()

# ---- Get start TCP (lower Z bound) ----
tcp_res = robot.GetActualTCPPose(flag=1)
if tcp_res[0] != 0 or len(tcp_res) < 2:
    print(f"Failed to read TCP (error {tcp_res[0]})")
    sys.exit(1)
start_tcp = tcp_res[1]
z_start = start_tcp[2]
print(f"\nCurrent TCP Position [X, Y, Z, Rx, Ry, Rz]: {start_tcp}")

# ---- USER-DEFINED +Z LIMIT ----
print("\n" + "="*60)
print("SETUP: Define Z-Axis Movement Range")
print("="*60)
while True:
    try:
        Z_MOVEMENT_LIMIT = float(input(f"\nEnter maximum +Z movement (mm) [default: 100]: ") or "100")
        if Z_MOVEMENT_LIMIT <= 0:
            print("Error: Must be positive!")
            continue
        if Z_MOVEMENT_LIMIT > 200:
            confirm = input("Large movement! Continue? (yes/no): ").strip().lower()
            if confirm not in ['yes', 'y']:
                continue
        break
    except ValueError:
        print("Invalid input!")

Z_END = z_start + Z_MOVEMENT_LIMIT

print("\n" + "="*60)
print("CONFIGURED Z RANGE:")
print(f" START Z = {z_start:.2f} mm")
print(f" END Z   = {Z_END:.2f} mm")
print(f" MAX +Z  = +{Z_MOVEMENT_LIMIT:.1f} mm")
print("="*60)

# ---- USER CONFIRMATION ----
print("\n" + "="*60)
print("CONFIRMATION REQUIRED")
print("="*60)
print("The robot will:")
print(" • PUSH DOWN on tool → Robot moves DOWN")
print(" • PULL UP on tool   → Robot moves UP")
print(f" • Stay within Z: {z_start:.1f} → {Z_END:.1f} mm")
print(" • Soft impedance mode active")
print("\nCtrl+C to stop")
print("="*60)

while True:
    ans = input("\nType 'ok' or 'yes' to start: ").strip().lower()
    if ans in ['ok', 'yes', 'y']:
        print("\nStarting Z-force control...")
        break
    elif ans in ['no', 'n']:
        print("Cancelled.")
        sys.exit(0)

baseline_forces = calibrate_baseline_forces()

# Motion parameters
LINEAR_VEL = 10.0
FORCE_THRESHOLD = 2.0
DEADBAND = 0.5
FORCE_TO_VELOCITY_SCALE = 10.0
UPDATE_RATE = 0.008

# Impedance
M = [0.5, 0.5, 0.3, 0.5, 0.5, 0.5]
B = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5]
K = [0.0]*6

print("\nStarting impedance...")
rtn = robot.ForceAndJointImpedanceStartStop(1, 0, M, K, B, 100, 100)
if rtn != 0:
    if input("Continue anyway? (yes/no): ").lower() not in ['yes', 'y']:
        sys.exit(1)

print("\n" + "="*60)
print("Z-FORCE CONTROL ACTIVE – NATURAL DIRECTION")
print("="*60)
print("PUSH DOWN → Robot DOWN")
print("PULL UP   → Robot UP")
#print("Range:", z_start:.1f, "→", Z_END:.1f, "mm")
print("Ctrl+C to stop")
print("="*60 + "\n")
time.sleep(1)

# ----------------------------------------------------------------------
# MAIN LOOP – CORRECTED FORCE DIRECTION
# ----------------------------------------------------------------------
try:
    while True:
        err, cur = robot.GetActualTCPPose()
        if err != 0: time.sleep(UPDATE_RATE); continue
        z_live = cur[2]
        orientation = cur[3:6]

        ft = robot.FT_GetForceTorqueRCS()
        if ft[0] != 0: time.sleep(UPDATE_RATE); continue

        raw = ft[1][:6]
        comp = [raw[i] - baseline_forces[i] for i in range(6)]
        for i in range(6):
            if abs(comp[i]) < DEADBAND: comp[i] = 0.0

        _, world = transform_forces_to_tcp_and_world(comp, orientation)
        fz_world = world[2]  # Fz in world (positive = upward force on sensor)

        # Determine direction
        near_start = abs(z_live - z_start) < 1.0
        near_end   = abs(z_live - Z_END) < 1.0
        if near_end:
            direction = "RETURN"
        elif near_start:
            direction = "FORWARD"
        else:
            direction = "FORWARD" if z_live < (z_start + Z_END)/2 else "RETURN"

        if abs(fz_world) > FORCE_THRESHOLD:
            # KEY FIX: NEGATE fz_world so PUSH DOWN (fz_world < 0) → move DOWN
            dz = -fz_world * FORCE_TO_VELOCITY_SCALE
            z_target_raw = z_live + dz

            # One-direction clamp
            if direction == "FORWARD":
                z_target = max(z_target_raw, z_live)
                z_target = min(z_target, Z_END)
                action = "DOWN"
            else:
                z_target = min(z_target_raw, z_live)
                z_target = max(z_target, z_start)
                action = "UP"

            if z_target != z_target_raw:
                print(f"{action} CLAMP → Z={z_target:.2f} mm")

            target_pose = [cur[0], cur[1], z_target, cur[3], cur[4], cur[5]]

            print(f"Fz={fz_world:+.2f}N → ΔZ={dz:+.2f}mm → Z={z_target:.2f}mm [{action}]")

            ret = robot.MoveL(
                desc_pos=target_pose, tool=0, user=0, joint_pos=[0]*7,
                vel=LINEAR_VEL, acc=0.0, ovl=100.0,
                blendR=-1.0, exaxis_pos=[0]*4,
                search=0, offset_flag=0, offset_pos=[0]*6
            )
            if ret != 0:
                print(f"MoveL error {ret}")
        else:
            print(f"Holding (Fz={fz_world:.2f}N < {FORCE_THRESHOLD}N)")
            time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    print("\n\nStopped by user")
except Exception as e:
    print(f"\nError: {e}")
    import traceback; traceback.print_exc()
finally:
    print("\n" + "="*60)
    print("SHUTTING DOWN")
    robot.ForceAndJointImpedanceStartStop(0, 0, M, K, B, 100, 100)
    final = robot.GetActualTCPPose(flag=1)
    if final[0] == 0 and len(final) > 1:
        fz = final[1][2]
        print(f"Final Z: {fz:.2f} mm (Δ {fz - z_start:+.2f} mm)")
    print("Shutdown complete")
    print("="*60)