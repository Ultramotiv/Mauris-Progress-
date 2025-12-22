# FAIRINO COBOT — TRAJECTORY FOLLOWING WITH ADMITTANCE CONTROL (FIXED)
# Load trajectory → Move to start → Follow path based on FT sensor force

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import math
import json

# ============================================================================
# GLOBAL VARIABLES & TUNING
# ============================================================================
robot = None
running = True
trajectory_data = []
trajectory_index = 0.0  # Changed to float for smooth progression
user_started = False

# TUNING — Trajectory speed control
TRAJECTORY_SPEED = 6.0  # mm/sec - constant speed along trajectory
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]   # Virtual mass
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]   # Damping per joint
IK_TO_SERVO_RATIO = 2
IK_UPDATE_RATE = 0.0025
SERVO_UPDATE_RATE = 0.008
FORCE_THRESHOLD = 0.8
FORCE_FILTER_ALPHA = 0.28
force_thresholds = [2.0, 2.0, 2.5, 1.0, 1.0, 1.0]
joint_velocity = [0.0] * 6
desired_joint_pos = [0.0] * 6
filtered_desired_joints = None
filtered_force = 0.0
filtered_fx_world = 0.0
filtered_fy_world = 0.0
filtered_fz_world = 0.0
baseline_forces = [0.0] * 6

MAX_JOINT_VELOCITY = 60.0  # deg/s (safe operating speed)

# Trajectory segment lengths (calculated once)
segment_lengths = []

# ============================================================================
# TRAJECTORY LOADING
# ============================================================================
def load_trajectory(filename):
    """Load trajectory from JSON file"""
    global trajectory_data, segment_lengths
    
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        
        trajectory_data = data['trajectory']
        
        # Calculate distance between consecutive waypoints
        segment_lengths = []
        for i in range(len(trajectory_data) - 1):
            p1 = trajectory_data[i]['tcp_position'][:3]
            p2 = trajectory_data[i+1]['tcp_position'][:3]
            dist = math.sqrt(sum((p2[j] - p1[j])**2 for j in range(3)))
            segment_lengths.append(dist)
        segment_lengths.append(0)  # Last segment has no length
        
        print(f"✓ Loaded trajectory: {len(trajectory_data)} waypoints")
        print(f"  Duration: {data['metadata']['duration_seconds']:.2f} seconds")
        print(f"  Sampling rate: {data['metadata']['sampling_rate_hz']:.1f} Hz")
        print(f"  Total path length: {sum(segment_lengths):.2f} mm")
        return True
    except Exception as e:
        print(f"✗ Error loading trajectory: {e}")
        return False

def find_nearest_trajectory_point(current_tcp):
    """Find the nearest point in the trajectory to current TCP position"""
    if not trajectory_data:
        return 0
    
    min_dist = float('inf')
    nearest_idx = 0
    
    for idx, waypoint in enumerate(trajectory_data):
        tcp = waypoint['tcp_position']
        # Calculate Euclidean distance in XYZ
        dist = math.sqrt(
            (current_tcp[0] - tcp[0])**2 +
            (current_tcp[1] - tcp[1])**2 +
            (current_tcp[2] - tcp[2])**2
        )
        if dist < min_dist:
            min_dist = dist
            nearest_idx = idx
    
    return nearest_idx

# ============================================================================
# HELPERS
# ============================================================================
def euler_to_rotation_matrix(rx, ry, rz):
    rx = math.radians(rx); ry = math.radians(ry); rz = math.radians(rz)
    c, s = math.cos, math.sin
    Rx = np.array([[1,0,0], [0,c(rx),-s(rx)], [0,s(rx),c(rx)]])
    Ry = np.array([[c(ry),0,s(ry)], [0,1,0], [-s(ry),0,c(ry)]])
    Rz = np.array([[c(rz),-s(rz),0], [s(rz),c(rz),0], [0,0,1]])
    return Rz @ Ry @ Rx

def transform_forces_to_world(ft_forces, orientation):
    """Transform all forces from TCP frame to world frame"""
    R = euler_to_rotation_matrix(*orientation)
    tcp_force = np.array([ft_forces[0], ft_forces[1], -ft_forces[2]])
    world_force = R @ tcp_force
    return world_force

def calculate_trajectory_tangent(current_idx):
    """Calculate the tangent direction of trajectory at current point"""
    idx = int(current_idx)
    if idx >= len(trajectory_data) - 1:
        idx = len(trajectory_data) - 2
    if idx < 0:
        idx = 0
    
    # Get current and next waypoint
    curr_tcp = trajectory_data[idx]['tcp_position']
    next_tcp = trajectory_data[idx + 1]['tcp_position']
    
    # Calculate tangent vector (direction of trajectory)
    tangent = np.array([
        next_tcp[0] - curr_tcp[0],
        next_tcp[1] - curr_tcp[1],
        next_tcp[2] - curr_tcp[2]
    ])
    
    # Normalize tangent vector
    tangent_magnitude = np.linalg.norm(tangent)
    if tangent_magnitude > 0.001:
        tangent = tangent / tangent_magnitude
    else:
        tangent = np.array([1.0, 0.0, 0.0])
    
    return tangent

def interpolate_trajectory_position(index):
    """Get interpolated TCP position at fractional trajectory index"""
    idx_floor = int(index)
    idx_floor = max(0, min(idx_floor, len(trajectory_data) - 1))
    
    # If at the end, return last waypoint
    if idx_floor >= len(trajectory_data) - 1:
        return trajectory_data[-1]['tcp_position']
    
    # Linear interpolation between waypoints
    t = index - idx_floor  # Fractional part
    p1 = trajectory_data[idx_floor]['tcp_position']
    p2 = trajectory_data[idx_floor + 1]['tcp_position']
    
    interpolated = [
        p1[i] + t * (p2[i] - p1[i]) for i in range(6)
    ]
    
    return interpolated

def project_force_on_trajectory(world_force, trajectory_tangent):
    """Project the 3D force vector onto the trajectory tangent direction"""
    force_along_trajectory = np.dot(world_force, trajectory_tangent)
    return force_along_trajectory

def ema(new, old, alpha):
    return alpha * new + (1 - alpha) * old

# ============================================================================
# FT SENSOR
# ============================================================================
def init_ft_sensor():
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor ready")

def calibrate_baseline(samples=150):
    print("Calibrating baseline... (keep tool still)")
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        global baseline_forces
        baseline_forces = np.mean(forces, axis=0).tolist()
        print(f"Baseline: {[f'{x:+.3f}' for x in baseline_forces]}")

# ============================================================================
# MAIN LOOP — TRAJECTORY FOLLOWING WITH CONSTANT SPEED
# ============================================================================
def control_loop():
    global running, filtered_force, desired_joint_pos, joint_velocity
    global filtered_desired_joints, trajectory_index, user_started
    global filtered_fx_world, filtered_fy_world, filtered_fz_world

    print("\n" + "="*70)
    print(" TRAJECTORY FOLLOWING WITH FORCE CONTROL")
    print(" Push → move forward at 6 mm/s | Pull → move backward at 6 mm/s")
    print(" No force → stay at current position")
    print("="*70)

    # Get current position and find nearest trajectory point
    err, current_tcp = robot.GetActualTCPPose()
    if err != 0:
        print("Failed to get current TCP position")
        return
    
    # Find nearest point in trajectory to current position
    trajectory_index = float(find_nearest_trajectory_point(current_tcp))
    progress_pct = (trajectory_index / len(trajectory_data)) * 100
    print(f"\n✓ Starting from nearest trajectory point: {int(trajectory_index)}/{len(trajectory_data)} ({progress_pct:.1f}%)")
    print(f"  Current TCP: {[f'{x:.2f}' for x in current_tcp[:3]]}")
    print(f"  Nearest waypoint: {[f'{x:.2f}' for x in trajectory_data[int(trajectory_index)]['tcp_position'][:3]]}")

    if robot.ServoMoveStart() != 0: 
        print("Failed to start servo mode")
        return

    j = robot.GetActualJointPosDegree(flag=0)
    if j[0] == 0:
        desired_joint_pos[:] = j[1][:6]
    filtered_desired_joints = desired_joint_pos[:]

    acc_joints = None
    ik_count = servo_count = 0
    last_update_time = time.time()

    try:
        while running:
            t0 = time.time()
            dt = t0 - last_update_time
            last_update_time = t0

            err, current_tcp = robot.GetActualTCPPose()
            if err != 0:
                time.sleep(IK_UPDATE_RATE)
                continue

            # Get FT sensor data
            ft = robot.FT_GetForceTorqueRCS()
            if ft[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue

            raw = ft[1][:6]
            compensated = [raw[i] - baseline_forces[i] for i in range(6)]

            # Apply deadzone
            for i in range(6):
                if abs(compensated[i]) < force_thresholds[i]:
                    compensated[i] = 0.0

            # Transform forces to world frame
            world_force = transform_forces_to_world(compensated, current_tcp[3:6])
            
            # Filter each force component
            filtered_fx_world = ema(world_force[0], filtered_fx_world, FORCE_FILTER_ALPHA)
            filtered_fy_world = ema(world_force[1], filtered_fy_world, FORCE_FILTER_ALPHA)
            filtered_fz_world = ema(world_force[2], filtered_fz_world, FORCE_FILTER_ALPHA)
            
            filtered_world_force = np.array([filtered_fx_world, filtered_fy_world, filtered_fz_world])
            
            # Calculate trajectory tangent at current position
            trajectory_tangent = calculate_trajectory_tangent(trajectory_index)
            
            # Project force onto trajectory direction
            force_along_trajectory = project_force_on_trajectory(filtered_world_force, trajectory_tangent)
            
            # Determine movement direction based on force
            if abs(force_along_trajectory) > FORCE_THRESHOLD:
                # Force detected - move along trajectory
                direction = 1.0 if force_along_trajectory > 0 else -1.0
                
                # Calculate distance to move this cycle
                distance_mm = TRAJECTORY_SPEED * dt  # mm = mm/s * s
                
                # Convert distance to trajectory index increment
                # We need to know segment length at current position
                idx = int(trajectory_index)
                if 0 <= idx < len(segment_lengths) and segment_lengths[idx] > 0:
                    index_increment = distance_mm / segment_lengths[idx]
                else:
                    index_increment = 0.001  # Small default increment
                
                # Update trajectory index
                trajectory_index += direction * index_increment
                trajectory_index = max(0, min(trajectory_index, len(trajectory_data) - 1))
            # else: No force - stay at current trajectory_index (don't move)

            # Get target TCP from current trajectory index (interpolated)
            target_tcp = interpolate_trajectory_position(trajectory_index)

            # Calculate inverse kinematics
            ik = robot.GetInverseKin(0, target_tcp, -1)
            if ik[0] != 0:
                time.sleep(IK_UPDATE_RATE)
                continue

            tj = np.array(ik[1][:6])
            acc_joints = tj if acc_joints is None else acc_joints + tj
            ik_count += 1

            if ik_count >= IK_TO_SERVO_RATIO:
                avg_joints = (acc_joints / IK_TO_SERVO_RATIO).tolist()

                # Apply admittance control with safe velocity limit
                for j in range(6):
                    err = avg_joints[j] - desired_joint_pos[j]
                    f = err * 3.9
                    acc = (f - B[j] * joint_velocity[j]) / M[j]
                    joint_velocity[j] += acc * SERVO_UPDATE_RATE
                    joint_velocity[j] = np.clip(joint_velocity[j], -MAX_JOINT_VELOCITY, MAX_JOINT_VELOCITY)
                    desired_joint_pos[j] += joint_velocity[j] * SERVO_UPDATE_RATE

                # Output smoothing
                alpha = 0.32
                if filtered_desired_joints is None:
                    filtered_desired_joints = desired_joint_pos[:]
                else:
                    for j in range(6):
                        filtered_desired_joints[j] = alpha * desired_joint_pos[j] + (1 - alpha) * filtered_desired_joints[j]

                robot.ServoJ(filtered_desired_joints, [0]*6, 0, 0, SERVO_UPDATE_RATE, 0, 0)

                if servo_count % 20 == 0:
                    max_jv = max(abs(v) for v in joint_velocity)
                    progress_pct = (trajectory_index / len(trajectory_data)) * 100
                    print(f"F_world=[{filtered_fx_world:+5.1f},{filtered_fy_world:+5.1f},{filtered_fz_world:+5.1f}]N | "
                          f"F_traj={force_along_trajectory:+5.1f}N | "
                          f"Progress={progress_pct:5.1f}% | "
                          f"Index={trajectory_index:7.2f}/{len(trajectory_data)} | "
                          f"JointVel={max_jv:4.1f}°/s")

                acc_joints = None
                ik_count = 0
                servo_count += 1

            sleep_t = IK_UPDATE_RATE - (time.time() - t0)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except Exception as e:
        print("Error:", e)
    finally:
        robot.ServoMoveEnd()

# ============================================================================
# SHUTDOWN
# ============================================================================
def shutdown(sig, frame):
    global running
    print("\nStopping...")
    running = False
    time.sleep(0.5)
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

if __name__ == "__main__":
    robot = Robot.RPC('192.168.58.2')
    print("Connected to Fairino Cobot")
    
    # Ask for trajectory file
    trajectory_file = input("\nEnter trajectory filename (e.g., tcp_trajectory_20241212_143022.json): ").strip()
    
    if not load_trajectory(trajectory_file):
        print("Failed to load trajectory. Exiting.")
        sys.exit(1)
    
    # Initialize FT sensor
    init_ft_sensor()
    calibrate_baseline()
    
    # Wait for user confirmation
    print("\n" + "="*70)
    print("READY TO START")
    print("="*70)
    print("Position the robot manually where you want to start in the trajectory")
    print("The system will find the nearest trajectory point automatically")
    input("\nPress ENTER when ready to begin trajectory following...")
    
    print("\nSTARTING TRAJECTORY FOLLOWING")
    print("Push forward → robot moves forward at 6 mm/s along trajectory")
    print("Pull backward → robot moves backward at 6 mm/s along trajectory")
    print("No force → robot stays at current position")
    print("Press Ctrl+C to stop\n")
    
    try:
        control_loop()
    except KeyboardInterrupt:
        shutdown(None, None)