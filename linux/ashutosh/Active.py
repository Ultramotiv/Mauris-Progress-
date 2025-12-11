import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- PARAMETERS PER JOINT ---
# M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
# B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
# K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]

M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Spring stiffness per joint

force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Scaling: degrees per Newton
force_to_deg = 5

# Control loop timing
dt = 0.008

# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 2.0  # Wait 2 seconds before enabling control

# --- SAFETY LIMITS FOR EACH JOINT (degrees) ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),      # Joint 1: min, max
    2: (-179.0, -35.0),    # Joint 2: min, max
    3: (60.0, 144.0),      # Joint 3: min, max
    4: (-258.0, 80.0),     # Joint 4: min, max
    5: (-170.0, 12.0),     # Joint 5: min, max
    6: (-170.0, 170.0),    # Joint 6: min, max
}

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

# --- GRAVITY COMPENSATION CALIBRATION ---
def calibrate_baseline_forces():
    """Capture baseline forces for gravity compensation"""
    global baseline_forces
    print("Calibrating baseline forces (gravity compensation)...")
    
    force_samples = []
    for i in range(gravity_compensation_samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:  # No error
            forces = [
                ft_data[1][0],   # Fx
                -ft_data[1][1],  # Fy (inverted)
                ft_data[1][2],   # Fz
                ft_data[1][3],   # Mx
                ft_data[1][4],   # My
                ft_data[1][5]    # Mz
            ]
            force_samples.append(forces)
        time.sleep(0.01)  # 10ms between samples
    
    # Calculate average baseline forces
    if force_samples:
        baseline_forces = [sum(forces[i] for forces in force_samples) / len(force_samples) 
                          for i in range(6)]
        print(f"Baseline forces captured: {[f'{f:.2f}' for f in baseline_forces]}")
        print("Gravity compensation calibrated.")
    else:
        print("Warning: Could not capture baseline forces!")
        baseline_forces = [0.0] * 6

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nServo stopped. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# --- SETUP ---
init_ft_sensor()

# --- SAFETY LIMIT CHECK FUNCTION ---
def is_within_safety_limits(joint_idx, angle):
    """Check if the given angle for a joint is within its safety limits."""
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True  # No limit specified

# Get initial position and calibrate
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

# --- SAFETY CHECK FOR ALL JOINTS BEFORE CONTINUING ---
safety_ok = True
for j in range(6):  # Indices 0-5 for joints 1-6
    joint_num = j + 1
    if not is_within_safety_limits(joint_num, joint_pos[j]):
        print(f"Initial joint angle out of safety range for Joint {joint_num}: {joint_pos[j]:.2f}° (Safety range: {JOINT_SAFETY_LIMITS[joint_num][0]}° to {JOINT_SAFETY_LIMITS[joint_num][1]}°)")
        safety_ok = False
if not safety_ok:
    print("One or more joints are out of safety range. Please manually move the robot to a safe position and restart.")
    sys.exit(1)

# Calibrate baseline forces at startup position
calibrate_baseline_forces()

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
print("Current joint angles:")
for i, angle in enumerate(joint_pos):
    print(f"  {joint_names[i]}: {angle:.2f}°")

# --- USER INPUT FOR MODE ---
mode = None
while mode not in ["sitting", "standing"]:
    mode = input("Select mode (sitting/standing): ").strip().lower()
    if mode not in ["sitting", "standing"]:
        print("Invalid input. Please enter 'sitting' or 'standing'.")

if mode == "sitting":
    print("Mode: Sitting - Joints 1, 2, 3, 4 are free, rest fixed.")
    free_joints = [1, 2, 3, 4]
else:
    print("Mode: Standing - Only Joint 2 is free, rest fixed.")
    free_joints = [2]

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling control...")
time.sleep(startup_delay)

print("Admittance control started:")
print(f"Mode: {mode.capitalize()}")
print("Press Ctrl+C to stop.")

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces
    
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(dt)
            continue
            
        # Get raw forces
        raw_forces = [
            ft_data[1][0],   # Fx
            -ft_data[1][1],  # Fy (inverted)
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]
        
        # Apply gravity compensation
        if baseline_forces is not None:
            forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            forces = raw_forces
        
        # Apply deadband to reduce noise
        deadband = 0.5
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0
        
        if mode == "sitting":
            # Joint 1 (index 0) reacts to +Fy
            j1 = 0
            fy_force = forces[1]
            if abs(fy_force) < 1.0:
                home_pos[j1] = desired_pos[j1]
                spring_force = -K[j1] * (desired_pos[j1] - home_pos[j1]) / force_to_deg
                acc = (spring_force - B[j1] * velocity[j1]) / M[j1]
            else:
                acc = (-fy_force - B[j1] * velocity[j1]) / M[j1]
            velocity[j1] += acc * dt
            desired_pos[j1] += velocity[j1] * dt * force_to_deg
            
            # Joint 2 (index 1) reacts to +Fx (inverted)
            j2 = 1
            fx_force = forces[0]
            if abs(fx_force) < 1.0:
                home_pos[j2] = desired_pos[j2]
                spring_force = -K[j2] * (desired_pos[j2] - home_pos[j2]) / force_to_deg
                acc = (spring_force - B[j2] * velocity[j2]) / M[j2]
            else:
                acc = (-fx_force - B[j2] * velocity[j2]) / M[j2]  # Inverted for +Fx -> -joint2
            velocity[j2] += acc * dt
            desired_pos[j2] += velocity[j2] * dt * force_to_deg
            
            # Joint 3 (index 2) reacts to +Fz
            j3 = 2
            fz_force = forces[2]
            if abs(fz_force) < 1.0:
                home_pos[j3] = desired_pos[j3]
                spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
                acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
            else:
                acc = (fz_force - B[j3] * velocity[j3]) / M[j3]
            velocity[j3] += acc * dt
            desired_pos[j3] += velocity[j3] * dt * force_to_deg
            
            # Joint 4 (index 3) reacts to +My (clockwise, so negative acceleration)
            j4 = 3
            my_force = forces[4]
            if abs(my_force) < 1.0:
                home_pos[j4] = desired_pos[j4]
                spring_force = -K[j4] * (desired_pos[j4] - home_pos[j4]) / force_to_deg
                acc = (spring_force - B[j4] * velocity[j4]) / M[j4]
            else:
                acc = (my_force - B[j4] * velocity[j4]) / M[j4]  # Negative for clockwise
            velocity[j4] += acc * dt
            desired_pos[j4] += velocity[j4] * dt * force_to_deg
        else:
            # Only Joint 2 (index 1) free, reacts to +Fx (inverted)
            j3 = 2
            fz_force = forces[2]
            if abs(fz_force) < 1.0:
                home_pos[j3] = desired_pos[j3]
                spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
                acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
            else:
                acc = (fz_force - B[j3] * velocity[j3]) / M[j3]
            velocity[j3] += acc * dt
            desired_pos[j3] += velocity[j3] * dt * force_to_deg
        
        # Lock all other joints
        for lock_j in range(6):
            if lock_j + 1 not in free_joints:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # --- SAFETY CHECK FOR ALL JOINTS ---
        safety_ok = True
        for j in range(6):  # Indices 0-5 for joints 1-6
            joint_num = j + 1
            if not is_within_safety_limits(joint_num, desired_pos[j]):
                print(f"Invalid joint angle for Joint {joint_num}: {desired_pos[j]:.2f}° (Safety range: {JOINT_SAFETY_LIMITS[joint_num][0]}° to {JOINT_SAFETY_LIMITS[joint_num][1]}°)")
                safety_ok = False
        if not safety_ok:
            time.sleep(dt)
            continue
        
        # Print joint angles at each movement step
        joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
        print("Current joint angles:")
        for idx, angle in enumerate(desired_pos):
            print(f"  {joint_names[idx]}: {angle:.2f}°")
        
        # Only send command if all joints are within their safety limits
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
        time.sleep(dt)

control_loop()

