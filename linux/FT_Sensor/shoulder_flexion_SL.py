#This code will be used for shoulder Flexion PURE Active assistance, 
#Where if patient stopts the efforts the robot stops going forward 
#The code can be used only for standing and Lying-down as joint 2 is the only active rest are all fixed

import Robot
import time
import signal
import sys
import numpy as np

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Scaling: degrees per Newton
force_to_deg = 5

# Control loop timing
dt = 0.008

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

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling control...")
time.sleep(startup_delay)

print("Admittance control started - ONLY JOINT 2 ACTIVE:")
print("Joint 2: Force Z → Motion")
print("All other joints fixed.")
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
        
        # Print active forces (debug)
        if abs(forces[2]) > 0.3:  # Only care about Fz now
            print(f"Active force: Fz={forces[2]:.2f}N")
        
        # --- ONLY JOINT 2 REACTS TO Fz ---
        fz_force = forces[2]
        j = 2  # Joint index
        
        if abs(fz_force) < 1.0:
            # No external force → hold position
            home_pos[j] = desired_pos[j]
            spring_force = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
            acc = (spring_force - B[j] * velocity[j]) / M[j]
        else:
            # Apply Fz → motion
            acc = (fz_force - B[j] * velocity[j]) / M[j]
        
        velocity[j] += acc * dt
        delta = velocity[j] * dt * force_to_deg
        desired_pos[j] += delta
        
        # Keep all other joints fixed at home
        for lock_j in range(6):
            if lock_j != j:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
            
        time.sleep(dt)

control_loop()
