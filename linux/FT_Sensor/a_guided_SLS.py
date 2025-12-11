# DONE 
# #This code is Guided mode where we have set a degree from which assistance is needed so that patient must push the robot from 144deg to 
#120 deg after that robot will go to its minimum position and be back at 120 

import Robot
import time
import signal
import sys
import numpy as np

# --- PARAMETERS PER JOINT ---
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Joint 2 limits
JOINT2_MIN_LIMIT = 70.0    # degrees 
JOINT2_MAX_LIMIT = 144.0   # degrees
ASSISTANCE_ANGLE = 122.0   # degrees - robot will auto-continue from this point

# --- SAFETY LIMITS FOR EACH JOINT (degrees) ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),      # Joint 1: min, max
    2: (-179.0, -35.0),   # Joint 2: min, max
    3: (2.0, 144.0),      # Joint 3: min, max
    4: (-258.0, 80.0),    # Joint 4: min, max
    5: (-170.0, 12.0),    # Joint 5: min, max
    6: (-170.0, 170.0),   # Joint 6: min, max
}

# Motion control state
motion_paused = False

# Extended motion control variables
extended_motion_active = False
extended_motion_phase = "none"  # "auto_descending", "ascending", "complete"
extended_motion_completed = False  # Flag to prevent repeated extended motion
trajectory_positions = []  # Store trajectory for exact return
current_trajectory_index = 0

# ★ NEW: Angle direction monitoring variables ★
previous_angle = None
angle_history = []  # Store recent angles for direction analysis
ANGLE_HISTORY_SIZE = 5  # Number of recent angles to track
approaching_from_above = False  # Flag to track if robot approached 120° from above

# Enhanced trajectory control
TRAJECTORY_STEP_SIZE = 0.5  # degrees per step for smooth trajectory
descent_velocity = 8.0  # degrees/second for descent
ascent_velocity = 12.0  # degrees/second for ascent

# Control parameters
force_threshold_for_motion = 1.0  # Minimum force to trigger motion
assistance_reached_threshold = 1.0  # Tolerance for reaching assistance angle

# Scaling: degrees per Newton
force_to_deg = 2

# Control loop timing
dt = 0.008

# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 1.0  # Wait 1 second before enabling control

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# --- NEW: ANGLE DIRECTION MONITORING FUNCTIONS ---
def update_angle_history(current_angle):
    """Update the angle history and determine approach direction"""
    global angle_history, approaching_from_above
    
    # Add current angle to history
    angle_history.append(current_angle)
    
    # Keep only recent history
    if len(angle_history) > ANGLE_HISTORY_SIZE:
        angle_history.pop(0)
    
    # Determine if we're approaching from above when we have enough history
    if len(angle_history) >= 3:
        # Check if the trend was descending (coming from above)
        recent_angles = angle_history[-3:]  # Last 3 angles
        descending_trend = all(recent_angles[i] >= recent_angles[i+1] for i in range(len(recent_angles)-1))
        
        if descending_trend and recent_angles[0] > ASSISTANCE_ANGLE:
            approaching_from_above = True
        elif recent_angles[0] < ASSISTANCE_ANGLE:
            approaching_from_above = False

def check_assistance_angle_trigger(current_angle):
    """Check if we should trigger extended motion based on angle and direction"""
    global approaching_from_above, extended_motion_active, extended_motion_completed
    
    # Only trigger if:
    # 1. Not already in extended motion
    # 2. Extended motion not already completed in this cycle
    # 3. Currently at assistance angle (within tolerance)
    # 4. Approached from above (previous angle was greater than current)
    
    at_assistance_angle = abs(current_angle - ASSISTANCE_ANGLE) <= assistance_reached_threshold
    
    if (not extended_motion_active and 
        not extended_motion_completed and 
        at_assistance_angle):
        
        if approaching_from_above:
            print(f"★ REACHED ASSISTANCE ANGLE {ASSISTANCE_ANGLE}° FROM ABOVE!")
            print(f"★ Previous motion was descending - AUTO-TRIGGERING extended motion...")
            return True
        else:
            print(f"★ REACHED ASSISTANCE ANGLE {ASSISTANCE_ANGLE}° FROM BELOW!")
            print(f"★ Previous motion was ascending - STOPPING at assistance angle")
            print(f"★ Robot will remain at {ASSISTANCE_ANGLE}° until new force is applied")
            return False
    
    return False

# --- MOTION CONTROL FUNCTIONS ---
def check_joint2_limits(target_position):
    """Check if joint 2 target position is within limits"""
    if target_position < JOINT2_MIN_LIMIT or target_position > JOINT2_MAX_LIMIT:
        return False
    return True

def generate_descent_trajectory(start_pos, end_pos):
    """Generate smooth trajectory points from start to end position"""
    trajectory = []
    current = start_pos
    direction = -1 if end_pos < start_pos else 1
    
    while abs(current - end_pos) > TRAJECTORY_STEP_SIZE:
        trajectory.append(current)
        current += direction * TRAJECTORY_STEP_SIZE
    
    trajectory.append(end_pos)  # Ensure we reach the exact end position
    return trajectory

def start_extended_motion(current_pos):
    """Start extended motion sequence from assistance angle to minimum"""
    global extended_motion_active, extended_motion_phase, trajectory_positions, current_trajectory_index
    
    extended_motion_active = True
    extended_motion_phase = "auto_descending"
    
    # Generate trajectory from assistance angle to minimum limit
    trajectory_positions = generate_descent_trajectory(ASSISTANCE_ANGLE, JOINT2_MIN_LIMIT)
    current_trajectory_index = 0
    
    print(f"★ AUTO-EXTENDED MOTION STARTED!")
    print(f"   Phase 1: Auto-descending from {ASSISTANCE_ANGLE}° to {JOINT2_MIN_LIMIT}°")
    print(f"   Trajectory points: {len(trajectory_positions)}")
    print(f"   Will return to assistance angle: {ASSISTANCE_ANGLE}°")

def handle_extended_motion():
    """Handle the extended motion sequence with exact trajectory return"""
    global extended_motion_active, extended_motion_phase, desired_pos, velocity
    global trajectory_positions, current_trajectory_index
    
    if extended_motion_phase == "auto_descending":
        # Phase 1: Auto-descend from assistance angle to minimum position
        if current_trajectory_index < len(trajectory_positions):
            target_pos = trajectory_positions[current_trajectory_index]
            current_pos = desired_pos[2]
            
            # Move towards next trajectory point
            distance_to_target = target_pos - current_pos
            
            if abs(distance_to_target) > 0.1:
                # Calculate movement step
                move_step = -descent_velocity * dt  # Negative for descending
                if abs(move_step) > abs(distance_to_target):
                    move_step = distance_to_target
                
                desired_pos[2] = current_pos + move_step
                velocity[2] = move_step / dt
            else:
                # Reached current trajectory point, move to next
                desired_pos[2] = target_pos
                current_trajectory_index += 1
                
                if current_trajectory_index >= len(trajectory_positions):
                    # Reached minimum position, start ascending
                    extended_motion_phase = "ascending"
                    print(f"★ Reached minimum limit {JOINT2_MIN_LIMIT}°")
                    print(f"★ Phase 2: Ascending to assistance angle {ASSISTANCE_ANGLE}°")
        
        return
    
    elif extended_motion_phase == "ascending":
        # Phase 2: Return to assistance angle
        current_pos = desired_pos[2]
        target_pos = ASSISTANCE_ANGLE
        
        distance_to_target = target_pos - current_pos
        
        if abs(distance_to_target) > 0.5:  # Still moving up
            # Calculate movement step
            move_step = ascent_velocity * dt  # Positive for ascending
            if move_step > distance_to_target:
                move_step = distance_to_target
            
            desired_pos[2] = current_pos + move_step
            velocity[2] = move_step / dt
        else:
            # Reached target position
            desired_pos[2] = target_pos
            velocity[2] = 0.0
            extended_motion_phase = "complete"
            print(f"★ RETURNED to assistance angle: {target_pos:.1f}°")
        
        return
    
    elif extended_motion_phase == "complete":
        # Phase 3: Motion complete, return to normal operation
        extended_motion_active = False
        extended_motion_phase = "none"
        extended_motion_completed = True  # Mark extended motion as completed
        trajectory_positions = []
        current_trajectory_index = 0
        velocity[2] = 0.0
        
        # ★ NEW: Reset angle tracking for next cycle ★
        global angle_history, approaching_from_above
        angle_history = []
        approaching_from_above = False
        
        print(f"★ EXTENDED MOTION COMPLETE!")
        print(f"★ Robot at assistance angle: {ASSISTANCE_ANGLE}°")
        print(f"★ Extended motion cycle COMPLETED - will NOT repeat automatically")
        print(f"★ Ready for force application to move toward {JOINT2_MAX_LIMIT}°")

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
                -ft_data[1][1],  # Fy (inverted)
                ft_data[1][2],   # Fz
                ft_data[1][3],   # Mx
                ft_data[1][4],   # My
                ft_data[1][5]    # Mz
            ]
            force_samples.append(forces)
        time.sleep(0.01)  # 10ms between samples
    
    # Calculate average baseline forces
    if force_samples:
        baseline_forces = [sum(forces[i] for forces in force_samples) / len(force_samples) 
                          for i in range(6)]
        print(f"Baseline forces captured: {[f'{f:.2f}' for f in baseline_forces]}")
        print("Gravity compensation calibrated.")
    else:
        print("Warning: Could not capture baseline forces!")
        baseline_forces = [0.0] * 6

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nServo stopped. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# --- SETUP ---
init_ft_sensor()

# Get initial position and calibrate
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

# Check if initial joint 2 position is within limits
if not check_joint2_limits(joint_pos[2]):
    print(f"Warning: Initial Joint 2 position ({joint_pos[2]:.2f}°) is outside safe limits ({JOINT2_MIN_LIMIT}° to {JOINT2_MAX_LIMIT}°)")
    print("Please move robot to safe position before starting.")
    sys.exit(1)

# Validate assistance angle
if ASSISTANCE_ANGLE <= JOINT2_MIN_LIMIT or ASSISTANCE_ANGLE >= JOINT2_MAX_LIMIT:
    print(f"Error: ASSISTANCE_ANGLE ({ASSISTANCE_ANGLE}°) must be between {JOINT2_MIN_LIMIT}° and {JOINT2_MAX_LIMIT}°")
    sys.exit(1)

# Calibrate baseline forces at startup position
calibrate_baseline_forces()

# ★ NEW: Initialize angle tracking with current position ★
previous_angle = joint_pos[2]
angle_history = [joint_pos[2]]

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

# --- USER INPUT FOR MODE ---
mode = None
while mode not in ["sitting", "standing"]:
    mode = input("Select mode (sitting/standing): ").strip().lower()
    if mode not in ["sitting", "standing"]:
        print("Invalid input. Please enter 'sitting' or 'standing'.")

if mode == "sitting":
    print("Mode: Sitting - Joints 1, 2, 3 are free, rest fixed.")
    free_joints = [1, 2, 3]
else:
    print("Mode: Standing - Only Joint 2 is free, rest fixed.")
    free_joints = [2]

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling control...")
time.sleep(startup_delay)

print("★ PRE-DEFINED ASSISTANCE CONTROL STARTED:")
print(f"   Joint 2 Range: {JOINT2_MIN_LIMIT}° to {JOINT2_MAX_LIMIT}°")
print(f"   ★ ASSISTANCE ANGLE: {ASSISTANCE_ANGLE}° (CHANGE THIS IN CODE IF NEEDED)")
print("★ OPERATION SEQUENCE:")
print(f"   1. Apply force → robot moves from {JOINT2_MAX_LIMIT}° toward {ASSISTANCE_ANGLE}°")
print(f"   2. Robot reaches {ASSISTANCE_ANGLE}° FROM ABOVE → AUTO-TRIGGERS extended motion (ONCE)")
print(f"   3. Robot reaches {ASSISTANCE_ANGLE}° FROM BELOW → STOPS at {ASSISTANCE_ANGLE}°")
print(f"   4. If triggered: Robot continues automatically to minimum ({JOINT2_MIN_LIMIT}°)")
print(f"   5. If triggered: Robot returns automatically to EXACT assistance angle ({ASSISTANCE_ANGLE}°)")
print(f"   6. Apply force again → robot moves from {ASSISTANCE_ANGLE}° toward {JOINT2_MAX_LIMIT}°")
print(f"   ★ NEW: Direction monitoring ensures extended motion only when approaching from above!")
print("Press Ctrl+C to stop.")

def is_within_safety_limits(joint_idx, angle):
    """Check if the given angle for a joint is within its safety limits."""
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True  # No limit specified

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces, motion_paused
    global extended_motion_active, extended_motion_completed, previous_angle
    global approaching_from_above, angle_history
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
        if baseline_forces is not None:
            forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            forces = raw_forces
        deadband = 0.5
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0
        # Print active forces (debug)
        if any(abs(forces[i]) > 0.3 for i in [0, 1, 2]):
            direction_info = "FROM_ABOVE" if approaching_from_above else "FROM_BELOW"
            print(f"Forces: Fx={forces[0]:.2f}, Fy={forces[1]:.2f}, Fz={forces[2]:.2f} | J1:{desired_pos[0]:.1f}°, J2:{desired_pos[1]:.1f}°, J3:{desired_pos[2]:.1f}° | Dir: {direction_info}, Phase: {extended_motion_phase}")
        # --- Assistance angle tracking ---
        current_joint2_pos = desired_pos[2]
        update_angle_history(current_joint2_pos)
        if check_assistance_angle_trigger(current_joint2_pos):
            start_extended_motion(current_joint2_pos)
        if (extended_motion_completed and current_joint2_pos > ASSISTANCE_ANGLE + 5.0):
            extended_motion_completed = False
            angle_history = [current_joint2_pos]
            approaching_from_above = False
            print(f"★ Robot moved away from assistance angle - extended motion reset for next cycle")
        if extended_motion_active:
            handle_extended_motion()
        else:
            # Admittance control for joints
            if mode == "sitting":
                # Joints 1, 2, 3 free
                # Joint 1 (index 1) reacts to Fz
                j1 = 1
                fz_force = forces[2]
                if abs(fz_force) < 1.0:
                    home_pos[j1] = desired_pos[j1]
                    spring_force = -K[j1] * (desired_pos[j1] - home_pos[j1]) / force_to_deg
                    acc = (spring_force - B[j1] * velocity[j1]) / M[j1]
                else:
                    acc = (fz_force - B[j1] * velocity[j1]) / M[j1]
                velocity[j1] += acc * dt
                desired_pos[j1] += velocity[j1] * dt * force_to_deg
                # Joint 2 (index 2) reacts to Fx
                j2 = 2
                fx_force = forces[0]
                if abs(fx_force) < 1.0:
                    home_pos[j2] = desired_pos[j2]
                    spring_force = -K[j2] * (desired_pos[j2] - home_pos[j2]) / force_to_deg
                    acc = (spring_force - B[j2] * velocity[j2]) / M[j2]
                else:
                    acc = ((-fx_force) - B[j2] * velocity[j2]) / M[j2]
                velocity[j2] += acc * dt
                desired_pos[j2] += velocity[j2] * dt * force_to_deg
                # Joint 3 (index 3) reacts to Fx (same as SIT)
                j3 = 3
                fx_force = forces[0]
                if abs(fx_force) < 1.0:
                    home_pos[j3] = desired_pos[j3]
                    spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
                    acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
                else:
                    acc = ((-fx_force) - B[j3] * velocity[j3]) / M[j3]
                velocity[j3] += acc * dt
                desired_pos[j3] += velocity[j3] * dt * force_to_deg
            else:
                # Only Joint 2 (index 2) free, reacts to Fz
                j = 2
                fz_force = forces[2]
                if abs(fz_force) < 1.0:
                    home_pos[j] = desired_pos[j]
                    spring_force = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
                    acc = (spring_force - B[j] * velocity[j]) / M[j]
                else:
                    acc = (fz_force - B[j] * velocity[j]) / M[j]
                velocity[j] += acc * dt
                delta = velocity[j] * dt * force_to_deg
                potential_pos = desired_pos[j] + delta
                # STRICT BOUNDS CHECKING for joint 2
                if potential_pos < JOINT2_MIN_LIMIT:
                    potential_pos = JOINT2_MIN_LIMIT
                    velocity[j] = 0.0
                    print(f"Hit minimum limit! Clamped to {JOINT2_MIN_LIMIT}°")
                elif potential_pos > JOINT2_MAX_LIMIT:
                    potential_pos = JOINT2_MAX_LIMIT
                    velocity[j] = 0.0
                    print(f"Hit maximum limit! Clamped to {JOINT2_MAX_LIMIT}°")
                else:
                    velocity[j] = velocity[j]
                desired_pos[j] = potential_pos
        # Lock all other joints
        for lock_j in range(6):
            if (mode == "sitting" and lock_j not in [1,2,3]) or (mode == "standing" and lock_j != 2):
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        previous_angle = current_joint2_pos

        # --- SAFETY CHECK FOR ALL JOINTS ---
        safety_ok = True
        for j in range(6):  # Indices 0-5 for joints 1-6
            joint_num = j + 1
            if not is_within_safety_limits(joint_num, desired_pos[j]):
                print(f"Invalid joint angle for Joint {joint_num}: {desired_pos[j]:.2f}° (Safety range: {JOINT_SAFETY_LIMITS[joint_num][0]}° to {JOINT_SAFETY_LIMITS[joint_num][1]}°)")
                safety_ok = False
        if not safety_ok:
            time.sleep(dt)
            continue

        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
        time.sleep(dt)

control_loop()

