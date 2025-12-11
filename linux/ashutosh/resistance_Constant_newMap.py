#{Constant} Resisted code with NEW mapping 
# so the resistance is constant no matter what  

import math
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
kg = 5
Resistance_KG = 0.509 #used this value to get 5N as a result #5*9.81 = 49.05 N 
calculated_resistance = Resistance_KG * 9.81
ORIGINAL_M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]    # Mass/inertia per joint [J1, J2, J3, J4, J5, J6]
ORIGINAL_B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]    # Damping per joint [J1, J2, J3, J4, J5, J6]
ORIGINAL_K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # Spring stiffness per joint [J1, J2, J3, J4, J5, J6]
# ORIGINAL_M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
# ORIGINAL_B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
# ORIGINAL_K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
# Create working copies that are used throughout the code
# These ensure original parameters are never modified
M = ORIGINAL_M.copy()
B = ORIGINAL_B.copy()
K = ORIGINAL_K.copy()

# Make parameters immutable by converting to tuples (optional extra protection)
M = tuple(M)
B = tuple(B)
K = tuple(K)

force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Control loop timing
dt = 0.008

single_force_threshold = 0.5
# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 1.0  # Wait 1 seconds before enabling control

# --- SAFETY LIMITS FOR EACH JOINT (degrees) ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),      # Joint 1: min, max
    2: (-179.0, -35.0),   # Joint 2: min, max
    3: (60.0, 144.0),      # Joint 3: min, max
    4: (-258.0, 80.0),    # Joint 4: min, max
    5: (-170.0, 12.0),    # Joint 5: min, max
    6: (-170.0, 170.0),   # Joint 6: min, max
}

# Function to verify parameters haven't been modified
def verify_parameters():
    """Verify that M, B, K parameters haven't been accidentally modified"""
    global M, B, K, ORIGINAL_M, ORIGINAL_B, ORIGINAL_K
    
    # Convert back to lists for comparison if they're tuples
    current_M = list(M) if isinstance(M, tuple) else M
    current_B = list(B) if isinstance(B, tuple) else B
    current_K = list(K) if isinstance(K, tuple) else K
    
    if current_M != ORIGINAL_M:
        print(f"WARNING: M parameters have been modified!")
        print(f"Original: {ORIGINAL_M}")
        print(f"Current:  {current_M}")
        # Restore original values
        M = tuple(ORIGINAL_M)
        
    if current_B != ORIGINAL_B:
        print(f"WARNING: B parameters have been modified!")
        print(f"Original: {ORIGINAL_B}")
        print(f"Current:  {current_B}")
        # Restore original values
        B = tuple(ORIGINAL_B)
        
    if current_K != ORIGINAL_K:
        print(f"WARNING: K parameters have been modified!")
        print(f"Original: {ORIGINAL_K}")
        print(f"Current:  {current_K}")
        # Restore original values
        K = tuple(ORIGINAL_K)

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# --- COORDINATE TRANSFORMATION FUNCTIONS ---
def euler_to_rotation_matrix(rx, ry, rz):
    """Convert Euler angles (degrees) to rotation matrix"""
    rx_rad = math.radians(rx)
    ry_rad = math.radians(ry) 
    rz_rad = math.radians(rz)
    
    # Rotation matrices for each axis
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(rx_rad), -math.sin(rx_rad)],
        [0, math.sin(rx_rad), math.cos(rx_rad)]
    ])
    
    Ry = np.array([
        [math.cos(ry_rad), 0, math.sin(ry_rad)],
        [0, 1, 0],
        [-math.sin(ry_rad), 0, math.cos(ry_rad)]
    ])
    
    Rz = np.array([
        [math.cos(rz_rad), -math.sin(rz_rad), 0],
        [math.sin(rz_rad), math.cos(rz_rad), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix (ZYX order)
    R = Rz @ Ry @ Rx
    return R

def transform_forces_to_tcp_and_world(ft_forces, tcp_orientation, verbose=False):
    """
    Transform FT sensor forces to TCP frame and then to world coordinate frame
    
    FT sensor to TCP mappings:
    FT sensor -Z → TCP -Z
    FT sensor +Y → TCP -Y  
    FT sensor +X → TCP +X
    
    Args:
        ft_forces: [Fx_ft, Fy_ft, Fz_ft, Mx_ft, My_ft, Mz_ft] from FT sensor
        tcp_orientation: [rx, ry, rz] in degrees
        verbose: Print force mapping details
    
    Returns:
        tcp_forces: Forces in TCP frame
        world_forces: Forces in world frame
    """
    rx, ry, rz = tcp_orientation
    R = euler_to_rotation_matrix(rx, ry, rz)

    # Step 1: Convert FT sensor readings to TCP frame
    # FT sensor coordinate mapping to TCP coordinates
    tcp_force_vector = np.array([
        +ft_forces[0],   # FT +X → TCP +X
        -ft_forces[1],   # FT +Y → TCP -Y (flip sign)
        -ft_forces[2]    # FT -Z → TCP -Z (flip sign)
    ])

    tcp_moment_vector = np.array([
        +ft_forces[3],   # Mx: FT +X → TCP +X
        -ft_forces[4],   # My: FT +Y → TCP -Y (flip sign)
        -ft_forces[5]    # Mz: FT -Z → TCP -Z (flip sign)
    ])

    tcp_forces = [
        tcp_force_vector[0],  # Fx_tcp
        tcp_force_vector[1],  # Fy_tcp
        tcp_force_vector[2],  # Fz_tcp
        tcp_moment_vector[0], # Mx_tcp
        tcp_moment_vector[1], # My_tcp
        tcp_moment_vector[2]  # Mz_tcp
    ]

    # Step 2: Transform TCP frame forces to world frame using rotation matrix
    world_force_vector = R @ tcp_force_vector
    world_moment_vector = R @ tcp_moment_vector

    world_forces = [
        world_force_vector[0],  # Fx_world
        world_force_vector[1],  # Fy_world
        world_force_vector[2],  # Fz_world
        world_moment_vector[0], # Mx_world
        world_moment_vector[1], # My_world
        world_moment_vector[2]  # Mz_world
    ]

    if verbose:
        print("\n=== FORCE TRANSFORMATION DETAILS ===")
        print("FT Sensor to TCP mapping:")
        ft_axes = ['FT_X', 'FT_Y', 'FT_Z', 'FT_Mx', 'FT_My', 'FT_Mz']
        tcp_axes = ['TCP_X', 'TCP_-Y', 'TCP_-Z', 'TCP_Mx', 'TCP_-My', 'TCP_-Mz']
        
        for i in range(6):
            if abs(ft_forces[i]) > 0.5:  # Only show significant forces
                print(f"  {ft_axes[i]}={ft_forces[i]:.2f} → {tcp_axes[i]}={tcp_forces[i]:.2f}")
        
        print(f"TCP Orientation: [rx={rx:.1f}°, ry={ry:.1f}°, rz={rz:.1f}°]")
        print("TCP to World transformation:")
        world_axes = ['World_X', 'World_Y', 'World_Z', 'World_Mx', 'World_My', 'World_Mz']
        for i in range(6):
            if abs(world_forces[i]) > 0.5:  # Only show significant forces
                print(f"  TCP_{['X','Y','Z','Mx','My','Mz'][i]}={tcp_forces[i]:.2f} → {world_axes[i]}={world_forces[i]:.2f}")
        print("=====================================\n")

    return tcp_forces, world_forces

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
                ft_data[1][1],   # Fy
                ft_data[1][2],   # Fz
                ft_data[1][3],   # Mx
                ft_data[1][4],   # My
                ft_data[1][5]    # Mz
            ]
            force_samples.append(forces)
        time.sleep(0.01)
    
    if force_samples:
        baseline_forces = [sum(f[i] for f in force_samples)/len(force_samples) for i in range(6)]
        print(f"Baseline forces captured: {[f'{f:.2f}' for f in baseline_forces]}")
        print("Gravity compensation calibrated.")
    else:
        print("Warning: Could not capture baseline forces!")
        baseline_forces = [0.0]*6

# --- RESISTANCE FORCE CALCULATION ---
def apply_resistance_force(joint2_force):
    """
    Apply resistance force in opposite direction to the applied force
    If force is positive, resistance opposes it (reduces the effective force)
    If force is negative, resistance still opposes it (reduces the magnitude)
    """
    if abs(joint2_force) < 0.01:  # Essentially zero force
        return 0.0
    
    resistance_newton = Resistance_KG * 9.81 # or else try just applying kg 
    
    # Apply resistance in opposite direction to the applied force
    if joint2_force > 0:
        # Positive force: subtract resistance (oppose the positive force)
        effective_force = joint2_force - resistance_newton
    else:
        # Negative force: add resistance (oppose the negative force)  
        effective_force = joint2_force + resistance_newton
    
    # Ensure the effective force doesn't change sign due to resistance
    # (resistance should reduce magnitude, not reverse direction)
    if joint2_force > 0 and effective_force < 0:
        effective_force = 0.0
    elif joint2_force < 0 and effective_force > 0:
        effective_force = 0.0
    
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

# Get initial TCP position and calibrate
error, tcp_pos = robot.GetActualTCPPose()
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting TCP position. Exiting.")
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

calibrate_baseline_forces()

# Convert TCP pose to joint positions for initial setup
error, initial_joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting initial joint positions. Exiting.")
    sys.exit(1)

home_joint_pos = initial_joint_pos.copy()
target_joint_positions = initial_joint_pos.copy()
velocity = [0.0]*6

joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
print("Current joint angles:")
for i, angle in enumerate(joint_pos):
    print(f"  {joint_names[i]}: {angle:.2f}°")

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling control...")
time.sleep(startup_delay)
print("FT Sensor coordinate mapping:")
print("Press Ctrl+C to stop.")

# Verify parameters before starting main loop
verify_parameters()

# --- MAIN LOOP ---
def control_loop():
    global target_joint_positions, velocity, home_joint_pos, baseline_forces
    # IMPORTANT: Do NOT use global M, B, K here to avoid accidental modification
        
    while True:
        # Get current TCP pose for force transformation
        error, current_tcp_pose = robot.GetActualTCPPose()
        if error != 0:
            print(f"TCP pose read error: {error}")
            time.sleep(dt)
            continue
        
        tcp_orientation = current_tcp_pose[3:6]  # [rx, ry, rz]
        
        # Get FT sensor data
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(dt)
            continue

        # Get raw forces from FT sensor
        raw_forces = [
            ft_data[1][0],   # Fx
            ft_data[1][1],   # Fy  
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]

        # Apply gravity compensation
        if baseline_forces is not None:
            compensated_forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            compensated_forces = raw_forces

        # Apply deadband to reduce noise
        deadband = 0.5
        for i in range(6):
            if abs(compensated_forces[i]) < deadband:
                compensated_forces[i] = 0.0
        
        # Transform forces to TCP frame and then to world frame
        tcp_forces, world_forces = transform_forces_to_tcp_and_world(
            compensated_forces, tcp_orientation
        )

        # Get current joint positions
        error, current_joint_pos = robot.GetActualJointPosDegree()
        if error != 0:
            print(f"Joint position read error: {error}")
            time.sleep(dt)
            continue

        # Apply resistance to ALL forces for monitoring (even if not all are used)
        forces_with_resistance = [apply_resistance_force(force) for force in world_forces]

        # Calculate desired movement for joint 2 only
        # Use world_forces[2] (Fz in world frame) for TCP-based mapping
        j = 2
        force_to_deg = 5 #7.70
        # Use Fz in world frame for joint 2
        joint2_force = world_forces[2]
        fz_force_effective = forces_with_resistance[2]

        if abs(joint2_force) > 0.1:  # Only print if there's significant force
            resistance_amount = joint2_force - fz_force_effective
            print(f"[Joint 2 Standing] Raw Fz: {joint2_force:.2f}N → Effective: {fz_force_effective:.2f}N (Resistance: {resistance_amount:.2f}N)")
            #print(f"Resistance set to: {Resistance_KG}kg which is: {calculated_resistance} Newtons of force ")

        # --- CHANGE: Do NOT overwrite home_joint_pos; do not force spring-back to initial home ---
        if abs(fz_force_effective) < single_force_threshold:
            # No significant force - hold current position (no spring back)
            home_joint_pos[j]=target_joint_positions[j]
            spring_force = -K[j] * (target_joint_positions[j] - home_joint_pos[j]) / force_to_deg
            acc = (spring_force - B[j] * velocity[j]) / M[j]
        else:
            # Apply force-based acceleration with impedance (external force + spring term)
            acc = (fz_force_effective - B[j] * velocity[j]) / M[j]
        velocity[j] += acc * dt
        delta = velocity[j] * dt * force_to_deg
        potential_pos = target_joint_positions[j] + delta
        #target_joint_positions[j] += delta 

        # # Clamp joint 2 position to safe limits (optional, can add your own limits)
        UPPER_LIMIT = 60.0
        LOWER_LIMIT = 143.0
        if potential_pos < UPPER_LIMIT:
            potential_pos = UPPER_LIMIT
            velocity[j] = 0.0
        elif potential_pos > LOWER_LIMIT:
            potential_pos = LOWER_LIMIT
            velocity[j] = 0.0
        target_joint_positions[j] = potential_pos

        # --- CHANGE: Lock all other joints to their CURRENT positions (not initial home) ---
        for lock_j in range(6):
            if lock_j != j:
                # home_joint_pos[lock_j] = target_joint_positions[lock_j]  # Update to current position
                # target_joint_positions[lock_j] = home_joint_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # Send the joint positions to the robot
        err = robot.ServoJ(
            joint_pos=target_joint_positions,
            axisPos=[0]*6,
            cmdT=dt
        )
        if err != 0:
            print("ServoJ error:", err)
        time.sleep(dt)

control_loop()
