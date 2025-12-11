# This code was developed on 6th NOV 2025
# This code simply moves to that loaction with a vibrations only on each cycle !?
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")
error,version = robot.GetSDKVersion()
print(f"SDK version: {version}")
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
    FT sensor -Y → TCP +Y (CORRECTED: reversed mapping)
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
        +ft_forces[1],   # FT -Y → TCP +Y (CORRECTED: no flip)
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
    """Initialize and zero the FT sensor"""
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
def calibrate_baseline_forces(samples=100):
    """Capture baseline forces for gravity compensation"""
    print("Calibrating baseline forces (gravity compensation)...")
    
    force_samples = []
    for i in range(samples):
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
        return baseline_forces
    else:
        print("Warning: Could not capture baseline forces!")
        return [0.0]*6

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    print("\nShutting down gracefully...")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# ============================================================================
# MAIN PROGRAM: Force-Controlled Linear Motion in Y-axis
# ============================================================================

# Display current joint position for reference
current_pose = robot.GetActualJointPosDegree(flag=1)
print("\nCurrent Joint Position [J1, J2, J3, J4, J5, J6]:")
print(current_pose)

# Initialize FT sensor
init_ft_sensor()

# Get initial TCP position
tcp_result = robot.GetActualTCPPose(flag=1)
error_code = tcp_result[0]
current_tcp = tcp_result[1] if len(tcp_result) > 1 else None

if error_code != 0 or current_tcp is None or len(current_tcp) < 6:
    print(f"\n✗ Failed to get current TCP position (error code: {error_code})")
    sys.exit(1)

print(f"\nCurrent TCP Position [X, Y, Z, Rx, Ry, Rz]: {current_tcp}")

# Extract current position
x_current = current_tcp[0]
y_current = current_tcp[1]
z_current = current_tcp[2]
rx_current = current_tcp[3]
ry_current = current_tcp[4]
rz_current = current_tcp[5]

print(f"\nInitial Position:")
print(f"  X  = {x_current:.2f} mm")
print(f"  Y  = {y_current:.2f} mm")
print(f"  Z  = {z_current:.2f} mm")
print(f"  Rx = {rx_current:.2f}°")
print(f"  Ry = {ry_current:.2f}°")
print(f"  Rz = {rz_current:.2f}°")

# Calibrate baseline forces for gravity compensation
baseline_forces = calibrate_baseline_forces()

# Movement parameters
LINEAR_VEL = 5.0  # % of max Cartesian speed
FORCE_THRESHOLD = 2.0  # Minimum force (N) to trigger movement
DEADBAND = 0.5  # Noise filter threshold (N)
FORCE_TO_VELOCITY_SCALE = 10.0  # Scale factor for force to velocity conversion
UPDATE_RATE = 0.008  # seconds between updates
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]
rtn = robot.ForceAndJointImpedanceStartStop(1, 0, M,K,B, 100, 100)
print(f"ForceAndJointImpedanceStartStop rtn is {rtn}")
print("\n" + "="*60)
print("FORCE-CONTROLLED Y-AXIS MOTION")
print("="*60)
print(f"Movement velocity: {LINEAR_VEL}% of max speed")
print(f"Force threshold: {FORCE_THRESHOLD}N")
print("Apply force in Y-axis direction to move the robot")
print("Press Ctrl+C to stop")
print("="*60 + "\n")

# Main control loop
try:
    while True:
        # Get current TCP pose for force transformation
        error, current_tcp_pose = robot.GetActualTCPPose()
        if error != 0:
            print(f"TCP pose read error: {error}")
            time.sleep(UPDATE_RATE)
            continue
        
        tcp_orientation = current_tcp_pose[3:6]  # [rx, ry, rz]
        
        # Get FT sensor data
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(UPDATE_RATE)
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
        compensated_forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]

        # Apply deadband to reduce noise
        for i in range(6):
            if abs(compensated_forces[i]) < DEADBAND:
                compensated_forces[i] = 0.0
        
        # Transform forces to TCP frame and then to world frame
        tcp_forces, world_forces = transform_forces_to_tcp_and_world(
            compensated_forces, tcp_orientation
        )

        # Get force in world Y-axis (index 1)
        fy_world = world_forces[1]
        
        # Check if force exceeds threshold
        if abs(fy_world) > FORCE_THRESHOLD:
            # Calculate movement based on force
            # Positive force = move in +Y direction
            y_movement = fy_world * FORCE_TO_VELOCITY_SCALE
            
            # Get current position
            y_current_live = current_tcp_pose[1]
            y_target = y_current_live + y_movement
            
            # Create target position
            target_desc_pos = [
                current_tcp_pose[0],   # X (no change)
                y_target,              # Y (moved based on force)
                current_tcp_pose[2],   # Z (no change)
                current_tcp_pose[3],   # Rx (no change)
                current_tcp_pose[4],   # Ry (no change)
                current_tcp_pose[5]    # Rz (no change)
            ]
            
            print(f"Force detected: Fy={fy_world:.2f}N → Moving Y by {y_movement:.2f}mm (Target Y: {y_target:.2f}mm)")

            # Execute linear motion
            ret_linear = robot.MoveL(
                desc_pos    = target_desc_pos,
                tool        = 0,
                user        = 0,
                joint_pos   = [0.0]*7,
                vel         = LINEAR_VEL,
                acc         = 0.0,
                ovl         = 100.0,
                blendR      = -1.0,
                exaxis_pos  = [0.0]*4,
                search      = 0,
                offset_flag = 0,
                offset_pos  = [0.0]*6
            )
            
            if ret_linear != 0:
                print(f"✗ MoveL failed with error code: {ret_linear}")
        else:
            # No significant force - hold position
            print(f"Holding position (Force: Fy={fy_world:.2f}N < threshold {FORCE_THRESHOLD}N)")

        time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    print("\n\nMotion control stopped by user")
    print("="*60)
    
    # Get final position
    final_tcp_result = robot.GetActualTCPPose(flag=1)
    if final_tcp_result[0] == 0 and len(final_tcp_result) > 1:
        final_tcp = final_tcp_result[1]
        print(f"\nFinal TCP Position [X, Y, Z, Rx, Ry, Rz]: {final_tcp}")
        y_total_movement = final_tcp[1] - y_current
        print(f"\nTotal Y-axis movement: {y_total_movement:.2f}mm")
        print(f"  Initial Y: {y_current:.2f}mm")
        print(f"  Final Y:   {final_tcp[1]:.2f}mm")
    
    print("="*60)

