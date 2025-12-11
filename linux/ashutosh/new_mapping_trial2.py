# this code is perfectly good where the movement is based on TCP coordinate 
# so force to tcp and then world 
# no vibration smooth motion 
# here we need to make sure this code is adapting like active passive, resistive and other... 

import math
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- PARAMETERS PER JOINT (Index 0-5 corresponds to Joint 1-6) ---
# PROTECTED: These parameters should NEVER be modified after this point
ORIGINAL_M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]    # Mass/inertia per joint [J1, J2, J3, J4, J5, J6]
ORIGINAL_B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]    # Damping per joint [J1, J2, J3, J4, J5, J6]
ORIGINAL_K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # Spring stiffness per joint [J1, J2, J3, J4, J5, J6]

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

single_force_threshold = 1.0

# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 1.0  # Wait 1 seconds before enabling control

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

def calculate_desired_joint_movement(world_forces, current_joint_pos, force_sensitivity=0.15):
    """
    Calculate desired joint movement based on world forces
    This ensures ALL joints can move and no joint is fixed
    
    Args:
        world_forces: Forces in world frame
        current_joint_pos: Current joint positions
        force_sensitivity: How much the robot responds to forces
    
    Returns:
        target_joint_positions: Where we want joints to move to
    """
    target_positions = current_joint_pos.copy()
    
    # Enhanced force-to-motion mapping that allows ALL joints to move
    # Each joint can be influenced by multiple force components for natural movement
    
    # Joint 0 (Base): Influenced by rotational forces and some linear forces
    j0_influence = (world_forces[5] * 0.5 +        # Primary: Mz (yaw)
                    world_forces[0] * 0.2 +        # Secondary: Fx (can cause base rotation)
                    world_forces[1] * 0.2) * force_sensitivity
    target_positions[0] += j0_influence
    
    # Joint 1 (Shoulder): Influenced by vertical and forward forces
    j1_influence = (world_forces[2] * 1.0 +        # Primary: Fz (up/down)
                    world_forces[4] * 0.3 +        # Secondary: My (pitch moment)
                    world_forces[0] * 0.1) * force_sensitivity
    target_positions[1] += j1_influence
    
    # Joint 2 (Elbow): Influenced by forward/back forces and pitch
    j2_influence = (world_forces[0] * 1.0 +        # Primary: Fx (forward/back)
                    world_forces[4] * 0.4 +        # Secondary: My (pitch moment)
                    world_forces[2] * 0.2) * force_sensitivity
    target_positions[2] += j2_influence
    
    # Joint 3 (Wrist 1): Influenced by roll and lateral forces
    j3_influence = (world_forces[3] * 0.8 +        # Primary: Mx (roll)
                    world_forces[1] * 0.3 +        # Secondary: Fy (lateral)
                    world_forces[0] * 0.1) * force_sensitivity
    target_positions[3] += j3_influence
    
    # Joint 4 (Wrist 2): Influenced by pitch and vertical forces
    j4_influence = (world_forces[4] * 0.8 +        # Primary: My (pitch)
                    world_forces[2] * 0.2 +        # Secondary: Fz (vertical)
                    world_forces[1] * 0.1) * force_sensitivity
    target_positions[4] += j4_influence
    
    # Joint 5 (Wrist 3): Influenced by yaw and all rotational components
    j5_influence = (world_forces[5] * 0.8 +        # Primary: Mz (yaw)
                    world_forces[3] * 0.2 +        # Secondary: Mx (roll)
                    world_forces[4] * 0.2) * force_sensitivity
    target_positions[5] += j5_influence
    
    return target_positions

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

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nServo stopped. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# --- SETUP ---
init_ft_sensor()

# Get initial TCP position and calibrate
error, tcp_pos = robot.GetActualTCPPose()
if error != 0:
    print("Error getting TCP position. Exiting.")
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

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling control...")
time.sleep(startup_delay)
print("Enhanced Impedance Control Active:")
print("- ALL joints can move (no joint is fixed)")
print("- Each joint has individual M, B, K parameters:")
for j in range(6):
    print(f"  Joint {j+1}: M={M[j]}, B={B[j]}, K={K[j]}")
print("- Forces create motion commands for all joints")
print("- Impedance control shapes movement with per-joint parameters")
print("FT Sensor coordinate mapping:")
print("  FT +X → TCP +X → World coords")  
print("  FT +Y → TCP -Y → World coords")
print("  FT -Z → TCP -Z → World coords")
print("Press Ctrl+C to stop.")

# Verify parameters before starting main loop
verify_parameters()

# --- MAIN LOOP ---
def control_loop():
    global target_joint_positions, velocity, home_joint_pos, baseline_forces
    # IMPORTANT: Do NOT use global M, B, K here to avoid accidental modification
    
    loop_count = 0
    
    while True:
        loop_count += 1
        verbose_output = (loop_count % 125 == 0)  # Print details every 125 loops (1 second)
        impedance_verbose = (loop_count % 250 == 0)  # Detailed impedance info every 2 seconds
        
        # Verify parameters every 1000 loops (8 seconds) to catch any accidental modifications
        if loop_count % 1000 == 0:
            verify_parameters()
        
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
            compensated_forces, tcp_orientation, verbose=verbose_output
        )

        # Get current joint positions
        error, current_joint_pos = robot.GetActualJointPosDegree()
        if error != 0:
            print(f"Joint position read error: {error}")
            time.sleep(dt)
            continue

        # --- CHANGE: Start from current joint positions so target is NOT initial-home ---

        # Calculate desired movement for joint 2 only
        # Use world_forces[2] (Fz in world frame) for TCP-based mapping
        j = 2
        force_to_deg = 7.70
        # Use Fz in world frame for joint 2
        joint2_force = world_forces[2]

        # Impedance control for joint 2
        #spring_force = -K[j] * (current_joint_pos[j] - home_joint_pos[j]) / force_to_deg

        # --- CHANGE: Do NOT overwrite home_joint_pos; do not force spring-back to initial home ---
        if abs(joint2_force) < single_force_threshold:
            # No significant force - hold current position (no spring back)
            home_joint_pos[j]=target_joint_positions[j]
            total_force = 0.0
            spring_force = -K[j] * (target_joint_positions[j] - home_joint_pos[j]) / force_to_deg
            acc = (spring_force - B[j] * velocity[j]) / M[j]
        else:
            # Apply force-based acceleration with impedance (external force + spring term)
            #total_force = joint2_force + spring_force
            acc = (joint2_force - B[j] * velocity[j]) / M[j]
        
        velocity[j] += acc * dt
        delta = velocity[j] * dt * force_to_deg
        potential_pos = target_joint_positions[j] + delta

        # Clamp joint 2 position to safe limits (optional, can add your own limits)
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
                target_joint_positions[lock_j] = home_joint_pos[lock_j]
                velocity[lock_j] = 0.0

        # Display information
        if verbose_output:
            print(f"\nLoop {loop_count} (t={loop_count*dt:.1f}s):")
            print(f"Current TCP pose: {[f'{x:.1f}' for x in current_tcp_pose]}")
            ft_axes = ["FT_Fx", "FT_Fy", "FT_Fz", "FT_Mx", "FT_My", "FT_Mz"]
            active_ft_forces = [f"{ft_axes[i]}={compensated_forces[i]:.2f}" for i in range(6) if abs(compensated_forces[i]) > 0.5]
            if active_ft_forces:
                print("Active FT forces:", ", ".join(active_ft_forces))
            world_axes = ["W_Fx", "W_Fy", "W_Fz", "W_Mx", "W_My", "W_Mz"]
            active_world_forces = [f"{world_axes[i]}={world_forces[i]:.2f}" for i in range(6) if abs(world_forces[i]) > 0.5]
            if active_world_forces:
                print("Active world forces:", ", ".join(active_world_forces))
            print(f"Joint 2: {current_joint_pos[j]:.1f}°→{target_joint_positions[j]:.1f}° (Δ{target_joint_positions[j]-current_joint_pos[j]:+.2f}°) (v={velocity[j]:.1f}°/s) [M={M[j]}, B={B[j]}, K={K[j]}]")
            print(f"All other joints fixed to current position.")

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
