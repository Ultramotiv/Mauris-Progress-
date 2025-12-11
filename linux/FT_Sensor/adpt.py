#This code will give you data of everything 
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import sys
import numpy as np

# --- PARAMETERS PER JOINT ---
# M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
# B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
# K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
# force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]
# Scaling: degrees per Newton
force_to_deg = 5

# Control loop timing
dt = 0.008

# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 2.0  # Wait 2 seconds before enabling control

# ★ NEW: Force and velocity monitoring variables ★
motion_started = False
motion_start_time = None
motion_start_position = None
force_display_counter = 0
force_display_interval = 12  # Display forces every 12 iterations (96ms at 8ms cycle)
MOTION_THRESHOLD = 1.0  # Force threshold to start motion tracking (N)

# Velocity calculation variables
previous_positions = [0.0] * 6  # Previous joint positions for velocity calculation
joint_velocities = [0.0] * 6   # Calculated joint velocities (degrees/second)
velocity_filter_alpha = 0.2    # Low-pass filter coefficient for velocity smoothing

# Motion tracking variables
total_distance_moved = [0.0] * 6  # Total distance moved from start position
max_velocities = [0.0] * 6       # Maximum velocities reached for each joint
motion_duration = 0.0            # Total time robot has been in motion

# --- SAFETY LIMITS FOR EACH JOINT (degrees) ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),      # Joint 1: min, max
    2: (-179.0, -35.0),   # Joint 2: min, max
    3: (83.0, 144.0),      # Joint 3: min, max
    4: (-258.0, 80.0),    # Joint 4: min, max
    5: (-170.0, 12.0),    # Joint 5: min, max
    6: (-170.0, 170.0),   # Joint 6: min, max
}

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# ★ NEW: Motion detection and tracking functions ★
def detect_motion_start(forces):
    """Detect when robot starts moving based on force threshold"""
    global motion_started, motion_start_time, motion_start_position
    
    # Check if any force exceeds the motion threshold
    force_magnitude = np.sqrt(sum(f**2 for f in forces[:3]))  # Linear forces only
    
    if not motion_started and force_magnitude > MOTION_THRESHOLD:
        motion_started = True
        motion_start_time = time.time()
        motion_start_position = desired_pos.copy()
        print(f"\n🚀 MOTION STARTED! Force magnitude: {force_magnitude:.3f} N")
        print(f"   Start time: {time.strftime('%H:%M:%S')}")
        print(f"   Start position: {[f'{p:.2f}°' for p in motion_start_position]}")
        print("="*80)

def calculate_joint_velocities(current_positions):
    """Calculate joint velocities based on position change and time"""
    global previous_positions, joint_velocities, velocity_filter_alpha, max_velocities
    
    for i in range(6):
        # Calculate raw velocity (degrees/second)
        raw_velocity = (current_positions[i] - previous_positions[i]) / dt
        
        # Apply low-pass filter to smooth velocity
        joint_velocities[i] = (velocity_filter_alpha * raw_velocity + 
                              (1 - velocity_filter_alpha) * joint_velocities[i])
        
        # Track maximum velocity
        if abs(joint_velocities[i]) > max_velocities[i]:
            max_velocities[i] = abs(joint_velocities[i])
        
        # Update previous position
        previous_positions[i] = current_positions[i]

def update_motion_statistics():
    """Update motion statistics if robot is in motion"""
    global total_distance_moved, motion_duration
    
    if motion_started and motion_start_position is not None:
        # Calculate total distance moved from start position
        for i in range(6):
            total_distance_moved[i] = abs(desired_pos[i] - motion_start_position[i])
        
        # Update motion duration
        motion_duration = time.time() - motion_start_time

def print_comprehensive_monitoring(raw_forces, compensated_forces, joint_positions):
    """Print comprehensive force and velocity information"""
    global force_display_counter, force_display_interval
    
    force_display_counter += 1
    
    # Display every force_display_interval iterations
    if force_display_counter >= force_display_interval:
        force_display_counter = 0
        
        print("\n" + "="*100)
        print("                         COMPREHENSIVE ROBOT MONITORING")
        print("="*100)
        
        # ★ FORCE INFORMATION ★
        print("📊 FORCE DATA:")
        print("-" * 50)
        print("RAW FORCES (from sensor):")
        print(f"  Fx: {raw_forces[0]:8.3f} N  |  Fy: {raw_forces[1]:8.3f} N  |  Fz: {raw_forces[2]:8.3f} N")
        print(f"  Mx: {raw_forces[3]:8.3f} Nm |  My: {raw_forces[4]:8.3f} Nm |  Mz: {raw_forces[5]:8.3f} Nm")
        
        print("\nCOMPENSATED FORCES (gravity compensated):")
        print(f"  Fx: {compensated_forces[0]:8.3f} N  |  Fy: {compensated_forces[1]:8.3f} N  |  Fz: {compensated_forces[2]:8.3f} N")
        print(f"  Mx: {compensated_forces[3]:8.3f} Nm |  My: {compensated_forces[4]:8.3f} Nm |  Mz: {compensated_forces[5]:8.3f} Nm")
        
        # Force magnitudes
        linear_force_mag = np.sqrt(compensated_forces[0]**2 + compensated_forces[1]**2 + compensated_forces[2]**2)
        torque_mag = np.sqrt(compensated_forces[3]**2 + compensated_forces[4]**2 + compensated_forces[5]**2)
        print(f"\nFORCE MAGNITUDES:")
        print(f"  Total Linear Force:  {linear_force_mag:8.3f} N")
        print(f"  Total Torque:        {torque_mag:8.3f} Nm")
        
        # ★ JOINT POSITIONS ★
        print(f"\n🔧 JOINT POSITIONS (degrees):")
        print("-" * 50)
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        for i in range(6):
            print(f"  {joint_names[i]}: {joint_positions[i]:8.2f}°", end="  ")
            if (i + 1) % 3 == 0:  # New line every 3 joints
                print()
        if len(joint_positions) % 3 != 0:
            print()
        
        # ★ VELOCITY INFORMATION ★
        print(f"\n⚡ VELOCITY DATA (degrees/second):")
        print("-" * 50)
        for i in range(6):
            print(f"  {joint_names[i]}: {joint_velocities[i]:8.2f}°/s", end="  ")
            if (i + 1) % 3 == 0:  # New line every 3 joints
                print()
        if len(joint_velocities) % 3 != 0:
            print()
        
        # Overall robot speed
        total_speed = np.sqrt(sum(v**2 for v in joint_velocities))
        print(f"\nOVERALL ROBOT SPEED: {total_speed:8.2f}°/s")
        
        # ★ MOTION STATISTICS ★
        if motion_started:
            update_motion_statistics()
            print(f"\n🏃 MOTION STATISTICS (since motion started):")
            print("-" * 50)
            print(f"Motion Duration:     {motion_duration:8.2f} seconds")
            print(f"Motion Start Time:   {time.strftime('%H:%M:%S', time.localtime(motion_start_time))}")
            
            print("\nDISTANCE MOVED from start position:")
            for i in range(6):
                print(f"  {joint_names[i]}: {total_distance_moved[i]:8.2f}°", end="  ")
                if (i + 1) % 3 == 0:
                    print()
            if len(total_distance_moved) % 3 != 0:
                print()
            
            total_distance = np.sqrt(sum(d**2 for d in total_distance_moved))
            print(f"\nTOTAL DISTANCE MOVED: {total_distance:8.2f}°")
            
            print("\nMAXIMUM VELOCITIES reached:")
            for i in range(6):
                print(f"  {joint_names[i]}: {max_velocities[i]:8.2f}°/s", end="  ")
                if (i + 1) % 3 == 0:
                    print()
            if len(max_velocities) % 3 != 0:
                print()
            
            max_speed_overall = max(max_velocities)
            print(f"\nMAXIMUM OVERALL SPEED: {max_speed_overall:8.2f}°/s")
            
            # Average speed calculation
            if motion_duration > 0:
                avg_speed = total_distance / motion_duration
                print(f"AVERAGE SPEED:         {avg_speed:8.2f}°/s")
        else:
            print(f"\n🏃 MOTION STATUS: WAITING FOR FORCE > {MOTION_THRESHOLD} N TO START TRACKING")
        
        # ★ CONTROL STATUS ★
        print(f"\n⚙️  CONTROL STATUS:")
        print("-" * 50)
        print(f"Control Mode:        {mode.capitalize()}")
        if mode == "sitting":
            active_joints = "J1, J2, J3 (free)"
            locked_joints = "J4, J5, J6 (locked)"
        else:
            active_joints = "J2 (free)"
            locked_joints = "J1, J3, J4, J5, J6 (locked)"
        print(f"Active Joints:       {active_joints}")
        print(f"Locked Joints:       {locked_joints}")
        print(f"Force Threshold:     {MOTION_THRESHOLD} N")
        print(f"Control Loop Rate:   {1/dt:.0f} Hz ({dt*1000:.1f}ms)")
        
        print("="*100)

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
    
    # Print final motion summary if motion occurred
    if motion_started:
        print("\n" + "="*80)
        print("                    FINAL MOTION SUMMARY")
        print("="*80)
        update_motion_statistics()
        print(f"Total Motion Duration:    {motion_duration:.2f} seconds")
        print(f"Total Distance Moved:     {np.sqrt(sum(d**2 for d in total_distance_moved)):.2f}°")
        print(f"Maximum Speed Achieved:   {max(max_velocities):.2f}°/s")
        if motion_duration > 0:
            avg_speed = np.sqrt(sum(d**2 for d in total_distance_moved)) / motion_duration
            print(f"Average Speed:            {avg_speed:.2f}°/s")
        print("="*80)
    
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

# Get initial position and calibrate
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
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

# Calibrate baseline forces at startup position
calibrate_baseline_forces()

# ★ NEW: Initialize motion tracking variables ★
previous_positions = joint_pos.copy()
joint_velocities = [0.0] * 6

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
print("Initial joint angles:")
for i, angle in enumerate(joint_pos):
    print(f"  {joint_names[i]}: {angle:.2f}°")

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

print("★ ENHANCED SHOULDER FLEXION CONTROL WITH COMPREHENSIVE MONITORING ★")
print(f"Mode: {mode.capitalize()}")
print(f"Motion tracking starts when force > {MOTION_THRESHOLD} N")
print(f"Force/Velocity display rate: Every {force_display_interval * dt * 1000:.0f}ms")
print("Press Ctrl+C to stop and see final motion summary.")
print("="*80)

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces
    
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(dt)
            continue
            
        # Get raw forces
        raw_forces = [
            ft_data[1][0],   # Fx
            -ft_data[1][1],  # Fy (inverted)
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]
        
        # Apply gravity compensation
        if baseline_forces is not None:
            forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            forces = raw_forces
        
        # Apply deadband to reduce noise
        deadband = 0.5
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0
        
        # ★ NEW: Detect motion start and calculate velocities ★
        detect_motion_start(forces)
        calculate_joint_velocities(desired_pos)
        
        # ★ NEW: Comprehensive monitoring display ★
        print_comprehensive_monitoring(raw_forces, forces, desired_pos)
        
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
            desired_pos[j] += delta
        
        # Lock all other joints
        for lock_j in range(6):
            if lock_j not in free_joints:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # --- SAFETY CHECK FOR ALL JOINTS ---
        safety_ok = True
        for j in range(6):  # Indices 0-5 for joints 1-6
            joint_num = j + 1
            if not is_within_safety_limits(joint_num, desired_pos[j]):
                print(f"⚠️  SAFETY VIOLATION: Joint {joint_num}: {desired_pos[j]:.2f}° (Range: {JOINT_SAFETY_LIMITS[joint_num][0]}° to {JOINT_SAFETY_LIMITS[joint_num][1]}°)")
                safety_ok = False
        if not safety_ok:
            time.sleep(dt)
            continue
            
        # Only send command if all joints are within their safety limits
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
        time.sleep(dt)

control_loop()