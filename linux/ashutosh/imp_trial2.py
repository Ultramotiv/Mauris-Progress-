# in this code all joints are free except joint 2 which is always free
# Joint 1,3,4,5 with fault tolerance and assistance(spring-back) outside tolerance
# Gradual resistance and strong spring-back for other joints if joint 1,3,4,5 move beyond 15 degrees   

import Robot
import time
import signal
import sys
import math

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Scaling: degrees per Newton
force_to_deg = 10

# Control loop timing
dt = 0.008

# Updated Force axis to joint mapping
force_joint_map = {
    0: 1,  # Fx -> Joint 1
    1: 0,  # Fy -> Joint 0  
    2: 2,  # Fz -> Joint 2
    3: 5,  # Mx -> Joint 5
    4: 3,  # My -> Joint 3
    5: 4   # Mz -> Joint 4
}

# --- TRAJECTORY PARAMETERS ---
# Joint 2 trajectory limits (in degrees)
JOINT2_LOWER_LIMIT = 144.0
JOINT2_UPPER_LIMIT = 20.0
FAULT_TOLERANCE = 15.0  # ±15 degrees tolerance

# Trajectory parameters
TRAJECTORY_SPEED = 5.0  # degrees per second
trajectory_direction = -1  # Start moving from upper to lower (-1), then reverse (+1)

# Assist-as-needed parameters (based on paper)
K_ASSIST = 50.0  # Assistance stiffness when outside fault-tolerant region
K_FIELD_GRADIENT = 30.0  # Stiffness field gradient
K_INNER = 0.0  # Stiffness inside fault-tolerant region (free movement)

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

# --- TRAJECTORY FUNCTIONS ---
def get_desired_joint2_position(elapsed_time):
    """Calculate the desired position for joint 2 based on elapsed time"""
    global trajectory_direction
    
    # Calculate total range and time for one complete cycle
    total_range = abs(JOINT2_LOWER_LIMIT - JOINT2_UPPER_LIMIT)
    cycle_time = (2 * total_range) / TRAJECTORY_SPEED
    
    # Get position within current cycle
    cycle_position = (elapsed_time % cycle_time)
    half_cycle = cycle_time / 2
    
    if cycle_position <= half_cycle:
        # First half: move from upper to lower
        progress = cycle_position / half_cycle
        desired_pos = JOINT2_UPPER_LIMIT + progress * (JOINT2_LOWER_LIMIT - JOINT2_UPPER_LIMIT)
    else:
        # Second half: move from lower to upper
        progress = (cycle_position - half_cycle) / half_cycle
        desired_pos = JOINT2_LOWER_LIMIT + progress * (JOINT2_UPPER_LIMIT - JOINT2_LOWER_LIMIT)
    
    return desired_pos

def get_nearest_trajectory_position(current_pos):
    """Find the nearest position on the trajectory"""
    # Clamp to trajectory limits
    if current_pos < JOINT2_UPPER_LIMIT:
        return JOINT2_UPPER_LIMIT
    elif current_pos > JOINT2_LOWER_LIMIT:
        return JOINT2_LOWER_LIMIT
    else:
        return current_pos

def calculate_assistance_force(current_pos, desired_pos):
    """Calculate assistance force based on deviation from fault-tolerant region"""
    deviation = abs(current_pos - desired_pos)
    
    if deviation <= FAULT_TOLERANCE:
        # Inside fault-tolerant region - no assistance needed
        return 0.0
    else:
        # Outside fault-tolerant region - apply assistance
        excess_deviation = deviation - FAULT_TOLERANCE
        
        # Calculate stiffness based on position (stiffness field concept from paper)
        if excess_deviation <= FAULT_TOLERANCE:
            # Gradual increase in stiffness
            stiffness = K_ASSIST + (excess_deviation / FAULT_TOLERANCE) * K_FIELD_GRADIENT
        else:
            # Maximum stiffness for large deviations
            stiffness = K_ASSIST + K_FIELD_GRADIENT
        
        # Calculate assistance force
        assistance_force = stiffness * excess_deviation / force_to_deg
        
        # Determine direction (towards desired position)
        if current_pos > desired_pos:
            return -assistance_force
        else:
            return assistance_force

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
# Register initial joint positions
initial_joint_pos = None

init_ft_sensor()
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

calibrate_baseline_forces()

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6
initial_joint_pos = joint_pos.copy()

# Initialize trajectory
start_time = time.time()
print(f"Initial Joint 2 position: {joint_pos[2]:.2f} degrees")
print(f"Joint 2 trajectory: {JOINT2_UPPER_LIMIT}° to {JOINT2_LOWER_LIMIT}°")
print(f"Fault tolerance: ±{FAULT_TOLERANCE}°")

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print("Joint 2 trajectory control with fault tolerance started. Press Ctrl+C to stop.")

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces, initial_joint_pos
    
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time
        
        # Get current joint positions
        error, current_joint_pos = robot.GetActualJointPosDegree()
        if error != 0:
            print(f"Error getting joint positions: {error}")
            time.sleep(dt)
            continue
        
        # Get FT sensor data
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
        
        # --- JOINT 2: Always free, admittance control only ---
        # Directly set desired position to current position (no fault tolerance, no assistance)
        fz = forces[2]
        if abs(fz) > force_thresholds[2]:
            direction = 'positive' if fz > 0 else 'negative'
            print(f"[FORCE] Joint 2 moved | Force: {fz:.2f}N | Direction: {direction} | Joint: 2 | Position: {current_joint_pos[2]:.2f}°")
        # Joint 2 is always free: update desired position by admittance control only
        acc2 = (fz - B[2] * velocity[2]) / M[2]
        velocity[2] += acc2 * dt
        delta2 = velocity[2] * dt * force_to_deg
        desired_pos[2] += delta2
        
        # --- OTHER JOINTS: Always apply half force within tolerance, strong spring-back beyond ---
        for axis, joint in force_joint_map.items():
            if joint == 2:
                continue  # skip joint 2
            axis_force = forces[axis]
            threshold = force_thresholds[joint]
            # Reverse movement for joint 0 and joint 1
            if joint in [0, 1]:
                axis_force = -axis_force
            deviation = current_joint_pos[joint] - initial_joint_pos[joint]
            abs_deviation = abs(deviation)
            if abs_deviation <= FAULT_TOLERANCE:
                # Within tolerance: always apply half of the force
                resist_force = axis_force * 0.5
                total_force = resist_force
                if abs(axis_force) > threshold:
                    direction = 'positive' if axis_force > 0 else 'negative'
                    print(f"[RESIST] Joint {joint} | Force: {axis_force:.2f}N | Resistance: {resist_force:.2f}N | Deviation: {deviation:.2f}° | Direction: {direction} | Joint: {joint}")
            else:
                # Beyond tolerance: apply strong spring-back force
                excess_deviation = abs_deviation - FAULT_TOLERANCE
                spring_stiffness = K_ASSIST + min(excess_deviation / FAULT_TOLERANCE, 1.0) * K_FIELD_GRADIENT * 2.0
                spring_force = -spring_stiffness * deviation / force_to_deg
                total_force = spring_force + axis_force * 0.2
                direction = 'positive' if total_force > 0 else 'negative'
                print(f"[SPRING-BACK] Joint {joint} | Spring: {spring_force:.2f}N | Force: {axis_force:.2f}N | Deviation: {deviation:.2f}° | Direction: {direction} | Joint: {joint}")
            # Update velocity and desired position
            acc = (total_force - B[joint] * velocity[joint]) / M[joint]
            velocity[joint] += acc * dt
            delta = velocity[joint] * dt * force_to_deg
            desired_pos[joint] += delta
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
        time.sleep(dt)

control_loop()

