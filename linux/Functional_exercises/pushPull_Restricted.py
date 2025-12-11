# This code was developed on 6th NOV 2025
# Updated: 7th NOV 2025 – Added user confirmation before starting
# initially the code will ask you to confirm its the information is correct or not!


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
    tcp_force_vector   = np.array([+ft_forces[0], +ft_forces[1], -ft_forces[2]])
    tcp_moment_vector  = np.array([+ft_forces[3], -ft_forces[4], -ft_forces[5]])
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

# ---- Get start TCP (lower bound) ----
tcp_res = robot.GetActualTCPPose(flag=1)
if tcp_res[0] != 0 or len(tcp_res) < 2:
    print(f"Failed to read TCP (error {tcp_res[0]})")
    sys.exit(1)
start_tcp = tcp_res[1]
y_start   = start_tcp[1]

print(f"\nCurrent TCP Position [X, Y, Z, Rx, Ry, Rz]: {start_tcp}")

# ---- USER-DEFINED +Y LIMIT ----
print("\n" + "="*60)
print("SETUP: Define Movement Range")
print("="*60)

# Option 1: Ask user for movement limit
while True:
    try:
        Y_MOVEMENT_LIMIT = float(input(f"\nEnter maximum +Y movement from current position (mm) [default: 300]: ") or "300")
        if Y_MOVEMENT_LIMIT <= 0:
            print("Error: Movement limit must be positive!")
            continue
        if Y_MOVEMENT_LIMIT > 500:
            print("Warning: Large movement distance! Please confirm.")
            confirm = input("Continue with this value? (yes/no): ").strip().lower()
            if confirm not in ['yes', 'y']:
                continue
        break
    except ValueError:
        print("Invalid input! Please enter a number.")

Y_END = y_start + Y_MOVEMENT_LIMIT

# Display the configured range
print("\n" + "="*60)
print("CONFIGURED MOVEMENT RANGE:")
print("="*60)
print(f"   START Y = {y_start:.2f} mm  (current position)")
print(f"   END   Y = {Y_END:.2f} mm")
print(f"   MAX +Y  = +{Y_MOVEMENT_LIMIT:.1f} mm")
print("="*60)

# Show motion parameters
print("\nMOTION PARAMETERS:")
print(f"   Velocity:        10.0% of max speed")
print(f"   Force Threshold: 2.0 N")
print(f"   Update Rate:     125 Hz (8ms)")
print("="*60)

# ---- USER CONFIRMATION ----
print("\n" + "="*60)
print("⚠️  CONFIRMATION REQUIRED")
print("="*60)
print("The robot will:")
print("  • Move in +Y direction when forward force is applied")
print("  • Move in -Y direction when backward force is applied")
print(f"  • Stay within Y range: {y_start:.1f} to {Y_END:.1f} mm")
print("  • Operate with impedance control (compliant mode)")
print("\nYou can stop at any time with Ctrl+C")
print("="*60)

while True:
    user_input = input("\nType 'ok' or 'yes' to start force control: ").strip().lower()
    if user_input in ['ok', 'yes', 'y']:
        print("\n✓ Starting force control...")
        break
    elif user_input in ['no', 'n', 'cancel', 'exit']:
        print("\n✗ Operation cancelled by user.")
        sys.exit(0)
    else:
        print("Invalid input. Please type 'ok', 'yes', or 'no'.")

# ---- Calibrate baseline forces AFTER confirmation ----
baseline_forces = calibrate_baseline_forces()

# ---- Motion parameters ----
LINEAR_VEL              = 10.0
FORCE_THRESHOLD         = 2.0
DEADBAND                = 0.5
FORCE_TO_VELOCITY_SCALE = 10.0
UPDATE_RATE             = 0.008

# ---- Custom joint impedance (M, B, K) ----
M = [0.5, 0.5, 0.3, 0.5, 0.5, 0.5]
B = [1.0, 1.0, 1.0, 1.5, 1.5, 1.5]
K = [0.0]*6

# ---- START IMPEDANCE (once) ----
print("\nStarting impedance control...")
rtn = robot.ForceAndJointImpedanceStartStop(1, 0, M, K, B, 100, 100)
print(f"ForceAndJointImpedanceStartStop (START) rtn = {rtn}")

if rtn != 0:
    print(f"⚠️  Warning: Impedance control returned error code {rtn}")
    proceed = input("Continue anyway? (yes/no): ").strip().lower()
    if proceed not in ['yes', 'y']:
        print("Exiting...")
        sys.exit(1)

print("\n" + "="*60)
print("✓ FORCE CONTROL ACTIVE")
print("="*60)
print(f"Allowed Y range: {y_start:.1f} to {Y_END:.1f} mm")
print("→ Forward: +Y force → move toward END")
print("→ Return : –Y force → move toward START")
print("   (Opposite force is ignored)")
print("\n⚠️  Press Ctrl+C to stop safely")
print("="*60 + "\n")

# Small delay before starting
time.sleep(1)

# ----------------------------------------------------------------------
# MAIN CONTROL LOOP – ONE-DIRECTION LOGIC
# ----------------------------------------------------------------------
try:
    while True:
        err, cur = robot.GetActualTCPPose()
        if err != 0:
            print(f"TCP read error {err}")
            time.sleep(UPDATE_RATE); continue

        y_live = cur[1]
        orientation = cur[3:6]

        ft = robot.FT_GetForceTorqueRCS()
        if ft[0] != 0:
            print(f"FT error {ft[0]}")
            time.sleep(UPDATE_RATE); continue
        raw = ft[1][:6]

        comp = [raw[i] - baseline_forces[i] for i in range(6)]
        for i in range(6):
            if abs(comp[i]) < DEADBAND: comp[i] = 0.0

        _, world = transform_forces_to_tcp_and_world(comp, orientation)
        fy_world = world[1]

        # --- Determine current travel direction ---
        near_start = abs(y_live - y_start) < 1.0
        near_end   = abs(y_live - Y_END)   < 1.0

        if near_end:
            direction = "RETURN"
        elif near_start:
            direction = "FORWARD"
        else:
            direction = "FORWARD" if y_live < (y_start + Y_END)/2 else "RETURN"

        if abs(fy_world) > FORCE_THRESHOLD:
            dy = fy_world * FORCE_TO_VELOCITY_SCALE
            y_target_raw = y_live + dy

            # ---- ONE-DIRECTION CLAMP ----
            if direction == "FORWARD":
                y_target = max(y_target_raw, y_live)
                y_target = min(y_target, Y_END)
                action = "FORWARD"
            else:  # RETURN
                y_target = min(y_target_raw, y_live)
                y_target = max(y_target, y_start)
                action = "RETURN"

            if y_target != y_target_raw:
                print(f"{action} CLAMP → Y={y_target:.2f} mm (ignored opposite force)")

            target_pose = [cur[0], y_target, cur[2], cur[3], cur[4], cur[5]]

            print(f"Fy={fy_world: .2f}N → ΔY={dy: .2f}mm → Y={y_target:.2f}mm "
                  f"[{action}] [Range {y_start:.1f}-{Y_END:.1f}]")

            ret = robot.MoveL(
                desc_pos=target_pose, tool=0, user=0, joint_pos=[0]*7,
                vel=LINEAR_VEL, acc=0.0, ovl=100.0,
                blendR=-1.0, exaxis_pos=[0]*4,
                search=0, offset_flag=0, offset_pos=[0]*6
            )
            if ret != 0:
                print(f"MoveL error {ret}")
        else:
            print(f"Holding (Fy={fy_world:.2f} N < {FORCE_THRESHOLD} N)")

        time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    print("\n\n✓ Motion stopped by user (Ctrl+C)")
except Exception as e:
    print(f"\n✗ Unexpected error: {e}")
    import traceback
    traceback.print_exc()
finally:
    # ------------------------------------------------------------------
    # ALWAYS STOP IMPEDANCE
    # ------------------------------------------------------------------
    print("\n" + "="*60)
    print("SHUTTING DOWN SAFELY")
    print("="*60)
    print("Stopping joint impedance control...")
    rtn_stop = robot.ForceAndJointImpedanceStartStop(0, 0, M, K, B, 100, 100)
    print(f"ForceAndJointImpedanceStartStop (STOP) rtn = {rtn_stop}")

    final = robot.GetActualTCPPose(flag=1)
    if final[0] == 0 and len(final) > 1:
        f_tcp = final[1]
        print(f"\nFinal TCP Position: {f_tcp}")
        print(f"\nMOVEMENT SUMMARY:")
        print(f"   Initial Y: {y_start:.2f} mm")
        print(f"   Final Y:   {f_tcp[1]:.2f} mm")
        print(f"   Travel:    {f_tcp[1]-y_start:+.2f} mm")
        print(f"   Range:     {y_start:.1f} to {Y_END:.1f} mm")
    print("="*60)
    print("✓ Shutdown complete")