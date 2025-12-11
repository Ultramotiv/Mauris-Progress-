import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- PARAMETERS ---
force_to_deg = 5
dt = 0.008
MAX_VEL_DEG_S = 5.0
MAX_DELTA_PER_LOOP = MAX_VEL_DEG_S * dt  # 0.04 degrees per 8ms

# Gravity compensation
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 2.0

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# --- FT SENSOR INITIALIZATION ---
def init_ft_sensor():
    company = 24
    device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0)
    time.sleep(0.5)
    robot.FT_Activate(1)
    time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0)
    time.sleep(0.5)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT Sensor initialized and zeroed.")

# --- GRAVITY COMPENSATION ---
def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating baseline forces (gravity compensation)...")
    force_samples = []
    for _ in range(gravity_compensation_samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:
            forces = [
                ft_data[1][0],   # Fx
                -ft_data[1][1],  # Fy (inverted)
                ft_data[1][2],   # Fz
                ft_data[1][3],   # Mx
                ft_data[1][4],   # My
                ft_data[1][5]    # Mz
            ]
            force_samples.append(forces)
        time.sleep(0.01)
    if force_samples:
        baseline_forces = [sum(col) / len(force_samples) for col in zip(*force_samples)]
        print(f"Baseline forces: {[f'{f:.2f}' for f in baseline_forces]}")
    else:
        print("Warning: No baseline captured.")
        baseline_forces = [0.0] * 6

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nEmergency stop. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# =============================================================================
# --- INITIAL SETUP ---
# =============================================================================
init_ft_sensor()

print("Getting current joint positions...")
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print(f"Error getting joint positions: {error}. Exiting.")
    sys.exit(1)

print(f"Initial Joints → J1={joint_pos[0]:.2f}° J2={joint_pos[1]:.2f}° J3={joint_pos[2]:.2f}° J4={joint_pos[3]:.2f}° J5={joint_pos[4]:.2f}° J6={joint_pos[5]:.2f}°")

# Calibrate gravity
calibrate_baseline_forces()

# =============================================================================
# --- LOCK ALL JOINTS EXCEPT JOINT 3 (index 2) ---
# =============================================================================
fixed_joints = joint_pos.copy()           # Start with current pose
current_j3 = joint_pos[2]                 # JOINT 3 will move (index 2)
target_j3 = current_j3                    # Will be updated after user input
last_desired_j3 = current_j3              # For velocity limiting

print(f"\nJOINT 3 (index 2) will move. All others LOCKED.")
print(f"Current Joint 3: {current_j3:.2f}°")

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds...")
time.sleep(startup_delay)

# =============================================================================
# --- USER INPUT ---
# =============================================================================
try:
    user_input = float(input("\nEnter target change for Joint 3 (degrees, e.g., 20): "))
except:
    print("Invalid input. Exiting.")
    robot.ServoMoveEnd()
    sys.exit(1)

# Calculate target: current - user_input (negative = move backward)
target_j3 = current_j3 - user_input
print(f"\nMoving Joint 3 from {current_j3:.2f}° → {target_j3:.2f}°")
print(f"Speed: {MAX_VEL_DEG_S}°/s (very slow)")
print("Force + Joint data will be logged live...")
print("Press Ctrl+C to stop.\n")

# =============================================================================
# --- MAIN CONTROL LOOP: Move JOINT 3 only ---
# =============================================================================
def control_loop():
    global last_desired_j3, current_j3

    print("Starting motion...\n")

    while True:
        # --- 1. Read F/T sensor ---
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT error: {ft_data[0]}")
            time.sleep(dt)
            continue

        # --- 2. Process forces ---
        raw_forces = [
            ft_data[1][0], -ft_data[1][1], ft_data[1][2],
            ft_data[1][3], ft_data[1][4], ft_data[1][5]
        ]
        forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        for i in range(6):
            if abs(forces[i]) < 0.5:
                forces[i] = 0.0

        # --- 3. Log forces ---
        force_str = " | ".join([
            f"{name}={f:+.2f}" for name, f in zip(
                ["Fx", "Fy", "Fz", "Mx", "My", "Mz"], forces
            ) if abs(f) > 0.3
        ])

        # --- 4. Read current joint positions ---
        _, actual_pos = robot.GetActualJointPosDegree()
        current_j3 = actual_pos[2]  # JOINT 3 (index 2)

        # --- 5. Print ALL joints + forces ---
        joint_str = (f"J1={actual_pos[0]:6.2f}° J2={actual_pos[1]:6.2f}° "
                     f"J3={actual_pos[2]:6.2f}° J4={actual_pos[3]:6.2f}° "
                     f"J5={actual_pos[4]:6.2f}° J6={actual_pos[5]:6.2f}°")
        if force_str:
            print(f"{joint_str} | {force_str}")
        else:
            print(joint_str)

        # --- 6. Check if target reached ---
        error_j3 = target_j3 - current_j3
        if abs(error_j3) < 0.1:
            print(f"\nTARGET REACHED: Joint 3 = {current_j3:.2f}°")
            break

        # --- 7. Velocity-limited motion ---
        delta = np.clip(error_j3, -MAX_DELTA_PER_LOOP, MAX_DELTA_PER_LOOP)
        desired_j3 = last_desired_j3 + delta
        last_desired_j3 = desired_j3

        # --- 8. Send command: ONLY JOINT 3 MOVES ---
        desired_pos = fixed_joints.copy()
        desired_pos[2] = desired_j3  # JOINT 3 (index 2)
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[100]*6)
        if err != 0:
            print(f"ServoJ error: {err}")

        time.sleep(dt)

    # --- Motion complete: keep logging ---
    print("\nMotion finished. Holding position. Still logging joints + forces...")
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:
            raw = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
            forces = [raw[i] - baseline_forces[i] for i in range(6)]
            for i in range(6):
                if abs(forces[i]) < 0.5: forces[i] = 0.0
            active = [f"{n}={f:+.2f}" for n, f in zip(["Fx","Fy","Fz","Mx","My","Mz"], forces) if abs(f)>0.3]
            _, pos = robot.GetActualJointPosDegree()
            joint_str = (f"J1={pos[0]:6.2f}° J2={pos[1]:6.2f}° "
                         f"J3={pos[2]:6.2f}° J4={pos[3]:6.2f}° "
                         f"J5={pos[4]:6.2f}° J6={pos[5]:6.2f}°")
            if active:
                print(f"{joint_str} | {' | '.join(active)}")
            else:
                print(joint_str)
        time.sleep(0.1)

# --- RUN ---
control_loop()