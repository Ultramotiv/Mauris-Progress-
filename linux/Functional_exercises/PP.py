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
error, version = robot.GetSDKVersion()
print(f"SDK version: {version}")

# --- COORDINATE TRANSFORMATION FUNCTIONS ---
def euler_to_rotation_matrix(rx, ry, rz):
    """Convert Euler angles (degrees) to rotation matrix"""
    rx_rad = math.radians(rx)
    ry_rad = math.radians(ry) 
    rz_rad = math.radians(rz)
    
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
    
    R = Rz @ Ry @ Rx
    return R

def transform_forces_to_tcp_and_world(ft_forces, tcp_orientation, verbose=False):
    """
    Transform FT sensor forces to TCP frame and then to world coordinate frame
    """
    rx, ry, rz = tcp_orientation
    R = euler_to_rotation_matrix(rx, ry, rz)

    # Step 1: Convert FT sensor readings to TCP frame
    tcp_force_vector = np.array([
        +ft_forces[0],   # FT +X → TCP +X
        +ft_forces[1],   # FT -Y → TCP +Y
        -ft_forces[2]    # FT -Z → TCP -Z
    ])

    tcp_moment_vector = np.array([
        +ft_forces[3],   # Mx: FT +X → TCP +X
        -ft_forces[4],   # My: FT +Y → TCP -Y
        -ft_forces[5]    # Mz: FT -Z → TCP -Z
    ])

    tcp_forces = [
        tcp_force_vector[0],  # Fx_tcp
        tcp_force_vector[1],  # Fy_tcp
        tcp_force_vector[2],  # Fz_tcp
        tcp_moment_vector[0], # Mx_tcp
        tcp_moment_vector[1], # My_tcp
        tcp_moment_vector[2]  # Mz_tcp
    ]

    # Step 2: Transform TCP frame forces to world frame
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
        tcp_axes = ['TCP_X', 'TCP_Y', 'TCP_-Z', 'TCP_Mx', 'TCP_-My', 'TCP_-Mz']
        
        for i in range(6):
            if abs(ft_forces[i]) > 0.5:
                print(f"  {ft_axes[i]}={ft_forces[i]:.2f} → {tcp_axes[i]}={tcp_forces[i]:.2f}")
        
        print(f"TCP Orientation: [rx={rx:.1f}°, ry={ry:.1f}°, rz={rz:.1f}°]")
        print("TCP to World transformation:")
        world_axes = ['World_X', 'World_Y', 'World_Z', 'World_Mx', 'World_My', 'World_Mz']
        for i in range(6):
            if abs(world_forces[i]) > 0.5:
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
        if ft_data[0] == 0:
            forces = [
                ft_data[1][0], ft_data[1][1], ft_data[1][2],
                ft_data[1][3], ft_data[1][4], ft_data[1][5]
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
    # Disable FT_Control before exiting (positional arguments)
    robot.FT_Control(
        0,              # flag: 0=OFF
        0,              # sensor_id
        [0, 0, 0, 0, 0, 0],  # select
        [0.0]*6,        # ft
        [0.0]*6,        # ft_pid
        0,              # adj_sign
        0,              # ILC_sign
        0,              # max_dis
        0               # max_ang
    )
    print("FT_Control disabled.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# ============================================================================
# MAIN PROGRAM: Force-Controlled Linear Motion in Y-axis using FT_Control
# ============================================================================

# Display current joint position
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

# Calibrate baseline forces
baseline_forces = calibrate_baseline_forces()

# FT_Control parameters
SENSOR_ID = 0
SELECT = [0, 1, 0, 0, 0, 0]  # Control Y-axis only
TARGET_FORCE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # No constant force, free motion
FT_PID = [0.3, 0.01, 0.05, 0.3, 0.01, 0.05]  # [f_p, f_i, f_d, m_p, m_i, m_d]
MAX_DIS = 200.0  # Maximum adjustment distance in mm
MAX_ANG = 10.0   # Maximum angle adjustment in degrees

# Monitoring parameters
UPDATE_RATE = 0.05  # Monitor every 50ms
DEADBAND = 0.5  # Noise filter

print("\n" + "="*60)
print("FORCE-CONTROLLED Y-AXIS MOTION WITH FT_CONTROL")
print("="*60)
print("Starting constant force control...")
print("The robot will now respond to forces in Y-axis direction")
print("Apply force to move the robot - it will follow your hand!")
print("Press Ctrl+C to stop")
print("="*60 + "\n")

# Enable FT_Control for Y-axis
# Note: Use positional arguments to match API signature
ret = robot.FT_Control(
    1,              # flag: 1=ON
    SENSOR_ID,      # sensor_id (positional, not keyword)
    SELECT,         # select
    TARGET_FORCE,   # ft
    FT_PID,         # ft_pid
    0,              # adj_sign
    0,              # ILC_sign
    MAX_DIS,        # max_dis
    MAX_ANG,        # max_ang
    # M,              # M (optional)
    # B,              # B (optional)
    0,              # polishRadio
    1,              # filter_Sign: 1=ON for smooth motion
    0,              # posAdapt_sign
    1               # isNoBlock: 1=non-blocking
)

if ret != 0:
    print(f"✗ FT_Control failed to start with error code: {ret}")
    sys.exit(1)

print("✓ FT_Control enabled successfully!")
print("Robot is now in force-controlled mode.\n")

# Monitoring loop - just display forces, robot moves automatically
try:
    last_display_time = time.time()
    display_interval = 0.5  # Display every 0.5 seconds
    
    while True:
        # Get current TCP pose
        error, current_tcp_pose = robot.GetActualTCPPose()
        if error != 0:
            print(f"TCP pose read error: {error}")
            time.sleep(UPDATE_RATE)
            continue
        
        tcp_orientation = current_tcp_pose[3:6]
        
        # Get FT sensor data
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(UPDATE_RATE)
            continue

        # Get raw forces
        raw_forces = [
            ft_data[1][0], ft_data[1][1], ft_data[1][2],
            ft_data[1][3], ft_data[1][4], ft_data[1][5]
        ]

        # Apply gravity compensation
        compensated_forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]

        # Apply deadband
        for i in range(6):
            if abs(compensated_forces[i]) < DEADBAND:
                compensated_forces[i] = 0.0
        
        # Transform forces to world frame
        tcp_forces, world_forces = transform_forces_to_tcp_and_world(
            compensated_forces, tcp_orientation
        )

        # Display status periodically
        current_time = time.time()
        if current_time - last_display_time >= display_interval:
            fy_world = world_forces[1]
            y_position = current_tcp_pose[1]
            y_movement = y_position - y_current
            
            print(f"Y-Force: {fy_world:+6.2f}N | Y-Position: {y_position:7.2f}mm | Movement: {y_movement:+7.2f}mm")
            last_display_time = current_time

        time.sleep(UPDATE_RATE)

except KeyboardInterrupt:
    print("\n\nStopping force control...")
    
    # Disable FT_Control (positional arguments)
    ret = robot.FT_Control(
        0,              # flag: 0=OFF
        SENSOR_ID,      # sensor_id
        [0, 0, 0, 0, 0, 0],  # select
        [0.0]*6,        # ft
        [0.0]*6,        # ft_pid
        0,              # adj_sign
        0,              # ILC_sign
        0,              # max_dis
        0               # max_ang
    )
    
    if ret == 0:
        print("✓ FT_Control disabled successfully")
    else:
        print(f"✗ FT_Control disable failed with error: {ret}")
    
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