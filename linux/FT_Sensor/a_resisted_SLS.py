# DONE
#This code will be used for shoulder Flexion Resistance based on the applied force 
# Where if patient stops the efforts the robot stops going forward 
# The code can be used for both standing and sitting
 
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- RESISTANCE SETTING (0-100%) ---
# 0% = No resistance (current behavior) 
# 10% = 10% of applied force becomes resistance
# 100% = 100% of applied force becomes resistance (robot locks)
RESISTANCE_PERCENTAGE = 100  # Set your desired resistance level here (0-100)

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]

print(f"Resistance set to: {RESISTANCE_PERCENTAGE}% of applied force")

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
    2: (-179.0, -35.0),   # Joint 2: min, max
    3: (60.0, 144.0),      # Joint 3: min, max
    4: (-258.0, 80.0),    # Joint 4: min, max
    5: (-170.0, 12.0),    # Joint 5: min, max
    6: (-170.0, 170.0),   # Joint 6: min, max
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

# --- RESISTANCE FORCE CALCULATION ---
def apply_resistance_force(applied_force, resistance_percent):
    """
    Apply resistance as a percentage of the applied force
    
    HOW IT WORKS:
    - Resistance is calculated as a percentage of the applied force
    - The effective force is reduced by exactly this percentage
    - Formula: effective_force = applied_force × (1 - resistance_percentage/100)
    
    EXAMPLES with 10% resistance:
    - Patient applies +10N → Effective force = 10N × 0.9 = 9N (1N resistance opposing)
    - Patient applies -5N  → Effective force = -5N × 0.9 = -4.5N (0.5N resistance opposing)
    - Patient applies +2N  → Effective force = 2N × 0.9 = 1.8N (0.2N resistance opposing)
    
    This ensures resistance is ALWAYS exactly the specified percentage of applied force.
    """
    if applied_force == 0:
        return 0.0
    
    # Calculate resistance factor (0-1 range)
    # 10% resistance → resistance_factor = 0.1
    # 50% resistance → resistance_factor = 0.5
    resistance_factor = resistance_percent / 100.0
    
    # The effective force is the applied force reduced by the resistance percentage
    # This guarantees that resistance is exactly the specified percentage
    effective_force = applied_force * (1.0 - resistance_factor)
    
    return effective_force

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
    print("Mode: Sitting - Joints 1, 2, 3 are free, rest fixed.")
    free_joints = [1, 2, 3]
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
print(f"Resistance: {RESISTANCE_PERCENTAGE}% of applied force")
print("Press Ctrl+C to stop.")
print("\n--- FORCE MONITORING ACTIVE ---")
print("Format: [Joint] Raw Force → Effective Force (after resistance)")

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
        
        # Apply resistance to ALL forces for monitoring (even if not all are used)
        forces_with_resistance = [apply_resistance_force(force, RESISTANCE_PERCENTAGE) for force in forces]
        
        # Print current force readings (gravity compensated)
        print(f"\n=== FORCE MONITORING (Gravity Compensated) ===")
        print(f"Raw Forces - Fx: {forces[0]:.2f}N, Fy: {forces[1]:.2f}N, Fz: {forces[2]:.2f}N")
        print(f"Modified Forces - Fx: {forces_with_resistance[0]:.2f}N, Fy: {forces_with_resistance[1]:.2f}N, Fz: {forces_with_resistance[2]:.2f}N")
        print(f"Raw Moments - Mx: {forces[3]:.2f}Nm, My: {forces[4]:.2f}Nm, Mz: {forces[5]:.2f}Nm")
        print(f"Modified Moments - Mx: {forces_with_resistance[3]:.2f}Nm, My: {forces_with_resistance[4]:.2f}Nm, Mz: {forces_with_resistance[5]:.2f}Nm")
        
        if mode == "sitting":
            # Joints 1, 2, 3 free
            # Use pre-calculated resistance values
            j1 = 1
            fz_force_raw = forces[2]
            fz_force_effective = forces_with_resistance[2]
            
            # Print individual joint force comparison for Joint 1
            if abs(fz_force_raw) > 0.1:  # Only print if there's significant force
                resistance_amount = fz_force_raw - fz_force_effective
                print(f"[Joint 1] Raw Fz: {fz_force_raw:.2f}N → Effective: {fz_force_effective:.2f}N (Resistance: {resistance_amount:.2f}N)")
            
            if abs(fz_force_effective) < 1.0:
                home_pos[j1] = desired_pos[j1]
                spring_force = -K[j1] * (desired_pos[j1] - home_pos[j1]) / force_to_deg
                acc = (spring_force - B[j1] * velocity[j1]) / M[j1]
            else:
                acc = (fz_force_effective - B[j1] * velocity[j1]) / M[j1]
            velocity[j1] += acc * dt
            desired_pos[j1] += velocity[j1] * dt * force_to_deg
            
            # Joint 2 (index 2) reacts to Fx
            j2 = 2
            fx_force_raw = forces[0]
            fx_force_effective = forces_with_resistance[0]
            
            # Print individual joint force comparison for Joint 2
            if abs(fx_force_raw) > 0.1:  # Only print if there's significant force
                resistance_amount = fx_force_raw - fx_force_effective
                print(f"[Joint 2] Raw Fx: {fx_force_raw:.2f}N → Effective: {fx_force_effective:.2f}N (Resistance: {resistance_amount:.2f}N)")
            
            if abs(fx_force_effective) < 1.0:
                home_pos[j2] = desired_pos[j2]
                spring_force = -K[j2] * (desired_pos[j2] - home_pos[j2]) / force_to_deg
                acc = (spring_force - B[j2] * velocity[j2]) / M[j2]
            else:
                acc = ((-fx_force_effective) - B[j2] * velocity[j2]) / M[j2]
            velocity[j2] += acc * dt
            desired_pos[j2] += velocity[j2] * dt * force_to_deg
            
            # Joint 3 (index 3) reacts to Fx (same as Joint 2)
            j3 = 3
            fx_force_raw = forces[0]
            fx_force_effective = forces_with_resistance[0]
            
            # Print individual joint force comparison for Joint 3
            if abs(fx_force_raw) > 0.1:  # Only print if there's significant force
                resistance_amount = fx_force_raw - fx_force_effective
                print(f"[Joint 3] Raw Fx: {fx_force_raw:.2f}N → Effective: {fx_force_effective:.2f}N (Resistance: {resistance_amount:.2f}N)")
            
            if abs(fx_force_effective) < 1.0:
                home_pos[j3] = desired_pos[j3]
                spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
                acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
            else:
                acc = ((-fx_force_effective) - B[j3] * velocity[j3]) / M[j3]
            velocity[j3] += acc * dt
            desired_pos[j3] += velocity[j3] * dt * force_to_deg
        else:
            # Only Joint 2 (index 2) free, reacts to Fz
            j = 2
            fz_force_raw = forces[2]
            fz_force_effective = forces_with_resistance[2]
            
            # Print individual joint force comparison for Joint 2 in standing mode
            if abs(fz_force_raw) > 0.1:  # Only print if there's significant force
                resistance_amount = fz_force_raw - fz_force_effective
                print(f"[Joint 2 Standing] Raw Fz: {fz_force_raw:.2f}N → Effective: {fz_force_effective:.2f}N (Resistance: {resistance_amount:.2f}N)")
            
            if abs(fz_force_effective) < 1.0:
                home_pos[j] = desired_pos[j]
                spring_force = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
                acc = (spring_force - B[j] * velocity[j]) / M[j]
            else:
                acc = (fz_force_effective - B[j] * velocity[j]) / M[j]
            velocity[j] += acc * dt
            delta = velocity[j] * dt * force_to_deg
            desired_pos[j] += delta
        
        # Lock all other joints
        for lock_j in range(6):
            if lock_j not in free_joints:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # --- SAFETY CHECK FOR ALL JOINTS ---
        safety_ok = True
        for j in range(6):  # Indices 0-5 for joints 1-6
            joint_num = j + 1
            if not is_within_safety_limits(joint_num, desired_pos[j]):
                print(f"SAFETY WARNING: Invalid joint angle for Joint {joint_num}: {desired_pos[j]:.2f}° (Safety range: {JOINT_SAFETY_LIMITS[joint_num][0]}° to {JOINT_SAFETY_LIMITS[joint_num][1]}°)")
                safety_ok = False
        if not safety_ok:
            time.sleep(dt)
            continue
        
        # Print joint angles at each movement step
        print(f"\n--- JOINT POSITIONS ---")
        for idx, angle in enumerate(desired_pos):
            joint_name = joint_names[idx]
            if (idx + 1) in free_joints:
                print(f"  {joint_name}: {angle:.2f}° (ACTIVE)")
            else:
                print(f"  {joint_name}: {angle:.2f}° (LOCKED)")
        
        # Only send command if all joints are within their safety limits
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print(f"ServoJ error: {err}")
        time.sleep(dt)

control_loop()

