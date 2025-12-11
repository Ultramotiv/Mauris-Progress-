import Robot
import time
import signal
import sys

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 5.0, 10.0, 10.0, 10.0]  # Fixed syntax error: 10,0 -> 10.0
B = [10.0, 10.0, 10.0, 20.0, 20.0, 20.0]  # REDUCED damping for less resistance
K = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]  # Moderate spring values
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]  # Force thresholds

# Scaling: degrees per Newton
force_to_deg = 7.70  # 1N --> 7.70degree move for robot

# Control loop timing
dt = 0.008  # frequency laptop processor timing

# Simple mappings for joints 0, 1, 2 (unchanged)
simple_force_joint_map = {
    0: 1,  # Fx (Force in X) -> Joint 1
    1: 0,  # Fy (Force in Y) -> Joint 0
    2: 2,  # Fz (Force in Z) -> Joint 2
}

# Combination mappings for joints 3, 4, 5
# Each joint responds to combination of two force axes
combination_mappings = {
    3: [0, 2],  # Joint 3: Force in X AND Force in Z
    4: [1, 2],  # Joint 4: Force in Y AND Force in Z  
    5: [1, 0]   # Joint 5: Force in Y AND Force in X
}

# Minimum force threshold for combination detection
combination_threshold = 0.5  # Both forces must exceed this to activate combination
single_force_threshold = 1.0  # Single forces must exceed this for joints 0,1,2

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

print("Admittance control started - COMBINATION MAPPINGS ONLY:")
print("Joint 0,1,2: DISABLED (for testing)")
print("Joint 3: Force X + Z (>0.5N each) | Joint 4: Force Y + Z (>0.5N each) | Joint 5: Force Y + X (>0.5N each)")
print("Press Ctrl+C to stop.")

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
            ft_data[1][0],   # Fx (axis 0)
            -ft_data[1][1],  # Fy (axis 1) - inverted
            ft_data[1][2],   # Fz (axis 2)
            ft_data[1][3],   # Mx (axis 3)
            ft_data[1][4],   # My (axis 4)
            ft_data[1][5]    # Mz (axis 5)
        ]
        
        # Print force information when forces are detected
        force_names = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
        active_forces = []
        for i, force in enumerate(forces):
            if abs(force) > 0.3:  # Print if force exceeds 0.3N (lower than thresholds for visibility)
                active_forces.append(f"{force_names[i]}={force:.2f}N")
        
        # Check for combination forces
        combination_detected = ""
        if abs(forces[0]) > 0.3 and abs(forces[2]) > 0.3:  # X and Z
            combination_detected += "X and Z detected! "
        if abs(forces[1]) > 0.3 and abs(forces[2]) > 0.3:  # Y and Z
            combination_detected += "Y and Z detected! "
        if abs(forces[1]) > 0.3 and abs(forces[0]) > 0.3:  # Y and X
            combination_detected += "Y and X detected! "
        
        # Print simple force detection
        if active_forces:
            print(f"Forces: {', '.join(active_forces)}")
            if combination_detected:
                print(f"COMBINATION: {combination_detected}")
            else:
                # Print single axis detection
                if abs(forces[0]) > 0.3:
                    print("Single Force: X direction")
                if abs(forces[1]) > 0.3:
                    print("Single Force: Y direction")
                if abs(forces[2]) > 0.3:
                    print("Single Force: Z direction")
        
        # Process simple mappings for joints 0, 1, 2 (COMMENTED OUT FOR TESTING)
        # for axis, joint in simple_force_joint_map.items():
        #     axis_force = forces[axis]
        #     
        #     # Use single force threshold for joints 0, 1, 2
        #     if abs(axis_force) < single_force_threshold:
        #         # Set home position to current position when no significant force
        #         home_pos[joint] = desired_pos[joint]
        #         spring_force = -K[joint] * (desired_pos[joint] - home_pos[joint]) / force_to_deg
        #         acc = (spring_force - B[joint] * velocity[joint]) / M[joint]
        #     else:
        #         # Apply force-based acceleration
        #         acc = (axis_force - B[joint] * velocity[joint]) / M[joint]
        #     
        #     # Update velocity and position
        #     velocity[joint] += acc * dt
        #     delta = velocity[joint] * dt * force_to_deg
        #     desired_pos[joint] += delta
        
        # Process combination mappings for joints 3, 4, 5
        for joint, axes in combination_mappings.items():
            axis1, axis2 = axes
            force1 = forces[axis1]
            force2 = forces[axis2]
            
            # Check if both forces exceed minimum threshold
            if abs(force1) > combination_threshold and abs(force2) > combination_threshold:
                # Combine forces - use the average of both forces
                combined_force = (force1 + force2) / 2.0
                
                # Apply combined force-based acceleration directly (no additional threshold check)
                acc = (combined_force - B[joint] * velocity[joint]) / M[joint]
                
                # Update velocity and position
                velocity[joint] += acc * dt
                delta = velocity[joint] * dt * force_to_deg
                desired_pos[joint] += delta
            else:
                # If combination conditions not met, apply spring return to home
                home_pos[joint] = desired_pos[joint]
                spring_force = -K[joint] * (desired_pos[joint] - home_pos[joint]) / force_to_deg
                acc = (spring_force - B[joint] * velocity[joint]) / M[joint]
                
                velocity[joint] += acc * dt
                delta = velocity[joint] * dt * force_to_deg
                desired_pos[joint] += delta
        
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
            
        time.sleep(dt)

control_loop()