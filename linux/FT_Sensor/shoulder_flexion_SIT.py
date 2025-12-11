#This code will be used for shoulder Flexion PURE Active assistance, 
#Where if patient stopts the efforts the robot stops going forward 
#The code can be used only for sitting position, as joint 1,2,3 are open and rest are closed
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]  # Mass (inertia-like term)
B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0] # Damping
K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0] # Stiffness
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Scaling: degrees per Newton
force_to_deg = 5

# Control loop timing
dt = 0.008

# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 1.0  # Wait 1 seconds before enabling control

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
                ft_data[1][2],  # Fz (SIGN INVERTED HERE)
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

print("Admittance control started - JOINT 1, JOINT 2 & JOINT 3 ACTIVE:")
print("Joint 1: reacts to Fz")
print("Joint 2: reacts to Fx")
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
            ft_data[1][2],  # Fz (SIGN INVERTED HERE)
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
        
        # Debug print
        if abs(forces[2]) > 0.3 or abs(forces[0]) > 0.3:
            print(f"Forces: Fx={forces[0]:.2f}, Fz={forces[2]:.2f}")
        
        # --- JOINT 1 REACTS TO Fz ---
        j1 = 1
        fz_force = forces[2]
        if abs(fz_force) < 1.0:
            home_pos[j1] = desired_pos[j1]
            spring_force = -K[j1] * (desired_pos[j1] - home_pos[j1]) / force_to_deg
            acc = (spring_force - B[j1] * velocity[j1]) / M[j1]
        else:
            acc = (fz_force - B[j1] * velocity[j1]) / M[j1]  # SIGN FIXED
        
        velocity[j1] += acc * dt
        desired_pos[j1] += velocity[j1] * dt * force_to_deg
        
        # --- JOINT 2 REACTS TO Fx ---
        j2 = 2
        fx_force = forces[0]
        if abs(fx_force) < 1.0:
            home_pos[j2] = desired_pos[j2]
            spring_force = -K[j2] * (desired_pos[j2] - home_pos[j2]) / force_to_deg
            acc = (spring_force - B[j2] * velocity[j2]) / M[j2]
        else:
            acc = ((-fx_force) - B[j2] * velocity[j2]) / M[j2]
        
        velocity[j2] += acc * dt
        desired_pos[j2] += velocity[j2] * dt * force_to_deg

        # .... Joint 3
        j3 = 3
        fx_force = forces[0]
        if abs(fx_force) < 1.0:
            home_pos[j3] = desired_pos[j3]
            spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
            acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
        else:
            acc = ((-fx_force) - B[j3] * velocity[j3]) / M[j3]
        
        velocity[j3] += acc * dt
        desired_pos[j3] += velocity[j3] * dt * force_to_deg
        
        # --- LOCK ALL OTHER JOINTS ---
        for lock_j in range(6):
            if lock_j not in [j1, j2, j3]:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
            
        time.sleep(dt)

control_loop()
