# Ultra-Lightweight TCP Admittance Control with ServoJ First try on IK 
# DUAL-RATE CONTROL: 500Hz IK calculation + 125Hz ServoJ commands
# Single thread with different update rates for IK and control

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math

# ============================================================================
# GLOBAL VARIABLES
# ============================================================================
robot = None
running = True

# Configuration
Y_START = 0.0
Y_MOVEMENT_LIMIT = 350.0
Y_END = 0.0
baseline_forces = [0.0] * 6

# DUAL-RATE CONTROL PARAMETERS
IK_UPDATE_RATE = 0.002  # 500Hz for IK calculation
SERVO_UPDATE_RATE = 0.008  # 125Hz for ServoJ commands
IK_TO_SERVO_RATIO = int(SERVO_UPDATE_RATE / IK_UPDATE_RATE)  # = 4

# ULTRA-LIGHTWEIGHT PARAMETERS
FORCE_THRESHOLD = 1.0  # Very low threshold for feather-light response
DEADBAND = 0.15  # Minimal deadband
Y_END_TOLERANCE = 0.5  # mm tolerance for Y_END detection

# ============================================================================
# SIMPLIFIED ADMITTANCE CONTROL
# ============================================================================
M = [2.5, 2.5, 2.0, 2.5, 2.5, 2.5]  # Mass - moderate
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]  # Damping - LOW
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Zero stiffness

# Direct force-to-motion scaling
FORCE_TO_MOTION_SCALE = 10.0  # Direct scaling factor

# Joint velocity storage
joint_velocity = [0.0] * 6
home_joint_pos = [0.0] * 6

# Lightweight filtering
FORCE_FILTER_ALPHA = 0.2  # Light filtering only
filtered_fy_world = 0.0

# State tracking
y_end_reached = False

# ============================================================================
# COORDINATE TRANSFORMATION FUNCTIONS
# ============================================================================
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
    
    return Rz @ Ry @ Rx

def transform_forces_to_world(ft_forces, tcp_orientation):
    """Transform FT sensor forces to world frame and return Fy"""
    rx, ry, rz = tcp_orientation
    R = euler_to_rotation_matrix(rx, ry, rz)
    
    tcp_force_vector = np.array([
        +ft_forces[0],
        +ft_forces[1],
        -ft_forces[2]
    ])
    
    world_force_vector = R @ tcp_force_vector
    return world_force_vector[1]  # Return only Fy

def exponential_moving_average(new_value, old_value, alpha):
    """Lightweight EMA filter"""
    return alpha * new_value + (1 - alpha) * old_value

# ============================================================================
# FT SENSOR INITIALIZATION
# ============================================================================
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

def calibrate_baseline_forces(samples=100):
    """Capture baseline forces for gravity compensation"""
    print("Calibrating baseline forces...")
    
    force_samples = []
    for _ in range(samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:
            force_samples.append(ft_data[1][:6])
        time.sleep(0.01)
    
    if force_samples:
        baseline = [sum(col)/len(force_samples) for col in zip(*force_samples)]
        print(f"Baseline forces: {[f'{v:.2f}' for v in baseline]}")
        return baseline
    
    print("Warning: Could not capture baseline forces!")
    return [0.0] * 6

# ============================================================================
# DUAL-RATE CONTROL LOOP
# IK @ 500Hz, ServoJ @ 125Hz
# ============================================================================
def control_loop():
    """
    DUAL-RATE control loop:
    - Inner loop @ 500Hz (2ms): IK calculation and admittance computation
    - Outer loop @ 125Hz (8ms): ServoJ command execution
    
    Every 4 IK cycles = 1 ServoJ command
    """
    global joint_velocity, home_joint_pos, filtered_fy_world, y_end_reached, running
    
    print(f"Starting DUAL-RATE control loop")
    print(f"  IK calculation: {1/IK_UPDATE_RATE:.0f}Hz ({IK_UPDATE_RATE*1000:.1f}ms)")
    print(f"  ServoJ command: {1/SERVO_UPDATE_RATE:.0f}Hz ({SERVO_UPDATE_RATE*1000:.1f}ms)")
    print(f"  Ratio: {IK_TO_SERVO_RATIO}:1 (IK:ServoJ)")
    
    # Start servo mode
    ret = robot.ServoMoveStart()
    if ret != 0:
        print(f"Failed to start servo mode: {ret}")
        return
    
    print("Servo mode activated")
    
    # Get initial joint positions
    joint_result = robot.GetActualJointPosDegree(flag=0)
    if joint_result[0] != 0:
        print("Failed to get initial joint positions")
        return
    
    desired_joint_pos = joint_result[1][:6].copy()
    home_joint_pos = desired_joint_pos.copy()
    
    # Accumulated target joints for averaging
    accumulated_target_joints = None
    ik_cycle_count = 0
    servo_cycle_count = 0
    
    try:
        while running:
            loop_start_time = time.time()
            
            # ================================================================
            # STEP 1: Read current TCP pose (every IK cycle @ 500Hz)
            # ================================================================
            error, current_tcp = robot.GetActualTCPPose()
            if error != 0:
                time.sleep(IK_UPDATE_RATE)
                continue
            
            y_current = current_tcp[1]
            
            # Check Y_END limit
            if y_current >= (Y_END - Y_END_TOLERANCE):
                if not y_end_reached:
                    y_end_reached = True
                    print(f"\n{'='*60}")
                    print(f"🛑 Y_END LIMIT REACHED!")
                    print(f"   Current Y: {y_current:.2f} mm")
                    print(f"   Y_END: {Y_END:.2f} mm")
                    print(f"{'='*60}\n")
                
                # Stop servo mode
                robot.ServoMoveEnd()
                print("Servo mode ended")
                break
            
            # ================================================================
            # STEP 2: Read FT sensor and get Fy force (every IK cycle @ 500Hz)
            # ================================================================
            ft_data = robot.FT_GetForceTorqueRCS()
            if ft_data[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue
            
            raw_forces = ft_data[1][:6]
            
            # Compensate for gravity
            compensated_forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
            
            # Apply deadband
            for i in range(6):
                if abs(compensated_forces[i]) < DEADBAND:
                    compensated_forces[i] = 0.0
            
            # Transform to world frame and get Fy
            tcp_orientation = current_tcp[3:6]
            fy_world = transform_forces_to_world(compensated_forces, tcp_orientation)
            
            # Light filtering
            filtered_fy_world = exponential_moving_average(fy_world, filtered_fy_world, FORCE_FILTER_ALPHA)
            
            # ================================================================
            # STEP 3: Calculate target TCP (every IK cycle @ 500Hz)
            # ================================================================
            # Only apply force if above threshold
            active_force = filtered_fy_world if abs(filtered_fy_world) > FORCE_THRESHOLD else 0.0
            
            # Calculate target TCP with simple offset
            if abs(active_force) > 0:
                # Direct force-to-position conversion
                y_delta = active_force * FORCE_TO_MOTION_SCALE
                target_y = np.clip(y_current + y_delta, Y_START, Y_END)
            else:
                target_y = y_current
            
            # Create target TCP
            target_tcp = [
                current_tcp[0],
                target_y,
                current_tcp[2],
                current_tcp[3],
                current_tcp[4],
                current_tcp[5]
            ]
            
            # ================================================================
            # STEP 4: Inverse Kinematics @ 500Hz
            # ================================================================
            ik_result = robot.GetInverseKin(
                type=0,
                desc_pos=target_tcp,
                config=-1
            )
            
            if ik_result[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue
            
            target_joints = ik_result[1][:6]
            
            # Accumulate target joints for averaging
            if accumulated_target_joints is None:
                accumulated_target_joints = np.array(target_joints)
            else:
                accumulated_target_joints += np.array(target_joints)
            
            ik_cycle_count += 1
            
            # ================================================================
            # STEP 5: Apply ServoJ command @ 125Hz (every 4th IK cycle)
            # ================================================================
            if ik_cycle_count >= IK_TO_SERVO_RATIO:
                # Average the accumulated target joints
                averaged_target_joints = (accumulated_target_joints / IK_TO_SERVO_RATIO).tolist()
                
                # Get current joint angles for reference
                joint_result = robot.GetActualJointPosDegree(flag=0)
                if joint_result[0] == 0:
                    current_joints = joint_result[1][:6]
                
                # Apply LIGHTWEIGHT admittance to each joint
                for j in range(6):
                    # Simple position error
                    position_error = averaged_target_joints[j] - desired_joint_pos[j]
                    
                    # DIRECT force (simple proportional gain)
                    force = position_error * 10.0
                    
                    # Simple admittance: F = Ma + Bv
                    acc = (force - B[j] * joint_velocity[j]) / M[j]
                    
                    # Update velocity (using SERVO_UPDATE_RATE for integration)
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    
                    # Apply velocity limits
                    max_vel = 50.0  # deg/s
                    if abs(joint_velocity[j]) > max_vel:
                        joint_velocity[j] = np.sign(joint_velocity[j]) * max_vel
                    
                    # Update position DIRECTLY
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE
                
                # Update home position when no force
                if abs(active_force) < FORCE_THRESHOLD * 0.5:
                    home_joint_pos = desired_joint_pos.copy()
                
                # Send ServoJ command
                ret = robot.ServoJ(
                    joint_pos=desired_joint_pos,
                    axisPos=[0.0] * 6,
                    acc=0.0,
                    vel=0.0,
                    cmdT=SERVO_UPDATE_RATE,
                    filterT=0.0,
                    gain=0.0
                )
                
                if ret != 0:
                    print(f"ServoJ failed: {ret}")
                
                # Print status every 25 servo cycles (~0.2s)
                if servo_cycle_count % 25 == 0:
                    remaining = Y_END - y_current
                    print(f"Fy={filtered_fy_world:6.2f}N | Y={y_current:7.2f}mm | "
                          f"Remaining={remaining:6.2f}mm | "
                          f"J2_vel={joint_velocity[1]:6.2f}°/s | "
                          f"IK_cycles={ik_cycle_count}")
                
                # Reset accumulators
                accumulated_target_joints = None
                ik_cycle_count = 0
                servo_cycle_count += 1
            
            # Timing control for 500Hz IK loop
            elapsed = time.time() - loop_start_time
            sleep_time = IK_UPDATE_RATE - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            
    except Exception as e:
        print(f"Control loop error: {e}")
    
    finally:
        robot.ServoMoveEnd()
        print("Servo mode ended")

# ============================================================================
# SIGNAL HANDLER
# ============================================================================
def shutdown(sig, frame):
    global running
    print("\n\nShutdown signal received...")
    running = False
    time.sleep(0.5)
    
    # Get final position
    final_tcp_result = robot.GetActualTCPPose(flag=1)
    if final_tcp_result[0] == 0 and len(final_tcp_result) > 1:
        final_tcp = final_tcp_result[1]
        print(f"\nFinal TCP Position: {final_tcp}")
        total_movement = final_tcp[1] - Y_START
        print(f"Total Y movement: {total_movement:.2f} mm")
        print(f"Initial Y: {Y_START:.2f} mm")
        print(f"Final Y: {final_tcp[1]:.2f} mm")
        
        if y_end_reached:
            print(f"✓ Y_END was reached during operation")
        else:
            print(f"✗ Y_END was NOT reached")
    
    print("="*60)
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# ============================================================================
# MAIN PROGRAM
# ============================================================================
if __name__ == "__main__":
    # Connect to robot
    robot = Robot.RPC('192.168.58.2')
    print("Robot connected.")
    
    error, version = robot.GetSDKVersion()
    print(f"SDK version: {version}")
    
    # Display current position
    current_pose = robot.GetActualJointPosDegree(flag=1)
    print(f"\nCurrent Joint Position: {current_pose}")
    
    # Initialize FT sensor
    init_ft_sensor()
    
    # Get initial TCP position
    tcp_result = robot.GetActualTCPPose(flag=1)
    if tcp_result[0] != 0:
        print(f"Failed to get TCP position: {tcp_result[0]}")
        sys.exit(1)
    
    initial_tcp = tcp_result[1]
    Y_START = initial_tcp[1]
    Y_END = Y_START + Y_MOVEMENT_LIMIT
    
    print(f"\nInitial TCP Position: {initial_tcp}")
    print(f"\nY-AXIS MOVEMENT RANGE:")
    print(f"  START Y = {Y_START:.2f} mm")
    print(f"  END Y   = {Y_END:.2f} mm")
    print(f"  MAX +Y  = +{Y_MOVEMENT_LIMIT:.1f} mm")
    print(f"  Tolerance = ±{Y_END_TOLERANCE:.1f} mm")
    print("="*60)
    
    # Calibrate baseline forces
    baseline_forces = calibrate_baseline_forces()
    
    print("\n" + "="*60)
    print("DUAL-RATE TCP ADMITTANCE CONTROL")
    print("="*60)
    print("🚀 DUAL-RATE ARCHITECTURE:")
    print(f"  - IK calculation @ {1/IK_UPDATE_RATE:.0f}Hz ({IK_UPDATE_RATE*1000:.1f}ms)")
    print(f"  - ServoJ command @ {1/SERVO_UPDATE_RATE:.0f}Hz ({SERVO_UPDATE_RATE*1000:.1f}ms)")
    print(f"  - Ratio: {IK_TO_SERVO_RATIO} IK cycles per ServoJ")
    print("  - Single thread architecture")
    print("  - Target joint averaging over 4 IK cycles")
    print("\n💡 KEY FEATURES:")
    print("  ✓ High-frequency IK for responsiveness")
    print("  ✓ Appropriate ServoJ rate for stability")
    print("  ✓ Low damping (B=2.5)")
    print("  ✓ Direct force-to-motion mapping")
    print("  ✓ Lightweight filtering (α=0.2)")
    print(f"\nAdmittance Parameters:")
    print(f"  M (Mass):    {M}")
    print(f"  B (Damping): {B}")
    print(f"  K (Spring):  {K}")
    print(f"\nControl Parameters:")
    print(f"  Force threshold: {FORCE_THRESHOLD}N")
    print(f"  Deadband: {DEADBAND}N")
    print(f"  Force filter α: {FORCE_FILTER_ALPHA}")
    print(f"  Direct scaling: {FORCE_TO_MOTION_SCALE}")
    print(f"\nY range: {Y_START:.1f} to {Y_END:.1f} mm")
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    # Run control loop
    try:
        control_loop()
    except KeyboardInterrupt:
        shutdown(None, None)
    
    print("\nProgram terminated successfully")