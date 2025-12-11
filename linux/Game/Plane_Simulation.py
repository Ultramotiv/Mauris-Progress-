# "joint 1,2,3 are open and rest are closed"Exactly opposite: 1,2,3 locked, 4 and 5 open"
# Fx -> Joint 4
# Fy -> Joint 5
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- PARAMETERS PER JOINT ---
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Stiffness
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]
# ============================== SAFETY LIMITS ==============================
J4_MIN, J4_MAX = -175.0, -105.0           # Joint 4 range
J5_MIN, J5_MAX =   60.0,  120.0           # Joint 5 range
MAX_SPEED_DEG_S = 60.0                    # Hard speed limit
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
            ft_data[1][2],  # Fz 
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

        j3 = 3
        fz_force = -forces[0]
        if abs(fz_force) < 1.0:
            home_pos[j3] = desired_pos[j3]
            spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
            acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
        else:
            acc = (fz_force - B[j3] * velocity[j3]) / M[j3]  # SIGN FIXED
        
        velocity[j3] += acc * dt
        desired_pos[j3] += velocity[j3] * dt * force_to_deg

        j4 = 4
        fz_force = -forces[1]
        if abs(fz_force) < 1.0:
            home_pos[j4] = desired_pos[j4]
            spring_force = -K[j4] * (desired_pos[j4] - home_pos[j4]) / force_to_deg
            acc = (spring_force - B[j4] * velocity[j4]) / M[j4]
        else:
            acc = (fz_force - B[j4] * velocity[j4]) / M[j4]  # SIGN FIXED
        
        velocity[j4] += acc * dt
        desired_pos[j4] += velocity[j4] * dt * force_to_deg
        
        # --- LOCK ALL OTHER JOINTS ---
        for lock_j in range(6):
            if lock_j not in [j4, j3]:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
            
        time.sleep(dt)

control_loop()
