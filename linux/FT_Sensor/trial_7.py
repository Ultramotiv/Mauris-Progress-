import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
B = [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
K = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Scaling: degrees per Newton
force_to_deg = 5

# Control loop timing
dt = 0.008

# Force mappings (kept for future use, but not used in this version)
simple_force_joint_map = {
    0: 1,  # Fx -> Joint 1
    1: 0,  # Fy -> Joint 0
    2: 2,  # Fz -> Joint 2
}

combination_mappings = {
    3: [0, 2],  # Joint 3: Fx + Fz
    4: [1, 2],  # Joint 4: Fy + Fz  
    5: [1, 0]   # Joint 5: Fy + Fx
}

combination_threshold = 1.5
single_force_threshold = 1.0

# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 2.0  # Wait 2 seconds before enabling control

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

# Get initial position and calibrate
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

# Calibrate baseline forces at startup position
calibrate_baseline_forces()

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

# --- START SERVO MODE (required for real-time, but we won't send commands) ---
# if robot.ServoMoveStart() != 0:
#     print("Failed to start servo mode.")
#     sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling monitoring...")
time.sleep(startup_delay)

print("FORCE MONITORING ACTIVE - NO ROBOT MOVEMENT")
print("Only printing significant forces (>0.3N after deadband)")
print("Press Ctrl+C to stop.")

# --- MAIN LOOP: ONLY READ AND PRINT FORCES ---
def control_loop():
    global baseline_forces
    
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
        deadband = 0.5  # Forces below this are ignored
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0
        
        # Print only when significant forces are detected
        force_names = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
        active_forces = []
        for i, force in enumerate(forces):
            if abs(force) > 0.3:  # Only show meaningful forces
                active_forces.append(f"{force_names[i]}={force:+.2f}N")
        
        if active_forces:
            print(" | ".join(active_forces))
        
        # --- SERVOJ COMMAND IS COMMENTED OUT ---
        # err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        # if err != 0:
        #     print("ServoJ error:", err)
            
        time.sleep(dt)

control_loop()