#Impedence

import Robot
import time
import signal
import sys

# --- PARAMETERS PER JOINT ---
# M = [5.0, 5.0, 5.0, 10.0, 10.0, 10.0]  # Fixed syntax error: 10,0 -> 10.0
# B = [40.0, 40.0, 40.0, 80.0, 80.0, 80.0]  # Slightly higher damping
# K = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # Moderate spring values
# force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]  # Force thresholds
M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Scaling: degrees per Newton
force_to_deg = 10  # 1N --> 10degree move for robot

# Control loop timing
dt = 0.008  # frequency laptop processor timing

# Updated Force axis to joint mapping based on your requirements
# (0: Fx, 1: Fy, 2: Fz, 3: Mx, 4: My, 5: Mz)
force_joint_map = {
    0: 1,  # Fx (Force in X) -> Joint 1
    1: 0,  # Fy (Force in Y) -> Joint 0  
    2: 2,  # Fz (Force in Z) -> Joint 2
    3: 5,  # Mx (Torque in X) -> Joint 3
    4: 3,  # My (Torque in Y) -> Joint 4
    5: 4   # Mz (Torque in Z) -> Joint 5 (NOW ENABLED!)
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

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nServo stopped. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# --- GRAVITY COMPENSATION ---
gravity_compensation_samples = 100
baseline_forces = None

def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating baseline forces (gravity compensation)...")
    force_samples = []
    for i in range(gravity_compensation_samples):
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
        baseline_forces = [sum(forces[i] for forces in force_samples) / len(force_samples) for i in range(6)]
        print(f"Baseline forces captured: {[f'{f:.2f}' for f in baseline_forces]}")
    else:
        print("Warning: Could not capture baseline forces!")
        baseline_forces = [0.0] * 6

# --- SETUP ---
init_ft_sensor()
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

calibrate_baseline_forces()

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print("Admittance control started. ALL joints active. Press Ctrl+C to stop.")

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces
    
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(dt)
            continue
            
        raw_forces = [
            ft_data[1][0],   # Fx
            -ft_data[1][1],  # Fy (inverted)
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]
        
        # Gravity compensation
        if baseline_forces is not None:
            forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            forces = raw_forces
        
        # Update each joint sequentially (thread-safe)
        joint_names = ["Joint 0", "Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5"]
        for axis, joint in force_joint_map.items():
            axis_force = forces[axis]
            threshold = force_thresholds[joint]
            # Reverse movement for joint 0 and joint 1
            if joint in [0, 1]:
                axis_force = -axis_force
            direction = "positive" if axis_force > 0 else ("negative" if axis_force < 0 else "zero")
            moved = False
            if abs(axis_force) < threshold:
                home_pos[joint] = desired_pos[joint]
                spring_force = -K[joint] * (desired_pos[joint] - home_pos[joint]) / force_to_deg  # This becomes 0
                acc = (spring_force - B[joint] * velocity[joint]) / M[joint]
            else:
                acc = (axis_force - B[joint] * velocity[joint]) / M[joint]
                moved = True
            
            # Update velocity and position
            velocity[joint] += acc * dt
            delta = velocity[joint] * dt * force_to_deg
            desired_pos[joint] += delta
            if moved:
                print(f"[ACTIVE] {joint_names[joint]} | Axis {axis} | Force: {axis_force:.2f}N | Direction: {direction}")
        
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
            
        time.sleep(dt)

control_loop()

