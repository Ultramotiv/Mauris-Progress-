import Robot
import time
import signal
import sys

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 5.0, 10.0, 10.0, 10.0]  # Fixed syntax error: 10,0 -> 10.0
B = [40.0, 40.0, 40.0, 80.0, 80.0, 80.0]  # Slightly higher damping
K = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # Moderate spring values
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]  # Force thresholds

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
    3: 3,  # Mx (Torque in X) -> Joint 3
    4: 4,  # My (Torque in Y) -> Joint 4
    5: 5   # Mz (Torque in Z) -> Joint 5 (NOW ENABLED!)
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

# --- SETUP ---
init_ft_sensor()
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

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
    global desired_pos, velocity, home_pos
    
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(dt)
            continue
            
        forces = [
            ft_data[1][0],   # Fx
            -ft_data[1][1],  # Fy (inverted)
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]
        
        # Update each joint sequentially (thread-safe)
        for axis, joint in force_joint_map.items():
            axis_force = forces[axis]
            threshold = force_thresholds[joint]
            
            if abs(axis_force) < threshold:
                # Set home position to current position when no significant force
                home_pos[joint] = desired_pos[joint]
                spring_force = -K[joint] * (desired_pos[joint] - home_pos[joint]) / force_to_deg  # This becomes 0
                acc = (spring_force - B[joint] * velocity[joint]) / M[joint]
            else:
                # Apply force-based acceleration
                acc = (axis_force - B[joint] * velocity[joint]) / M[joint]
            
            # Update velocity and position
            velocity[joint] += acc * dt
            delta = velocity[joint] * dt * force_to_deg
            desired_pos[joint] += delta
        
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
            
        time.sleep(dt)

control_loop()