# =====================================================================
# FAIRINO ROBOT - Linear + Arc with AUTO then FORCE-GUIDED playback
# Updated: 22 NOV 2025 - SMOOTHNESS OPTIMIZED
# Phase 1: Automatic trajectory execution + storage
# Phase 2: Force-guided replay with ARC TANGENT constraint (SMOOTH)


## perfect code for now ###
# =====================================================================
import sys
import time
import math
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import numpy as np

robot = Robot.RPC('192.168.58.2')

# ====================== ADMITTANCE PARAMETERS ======================
M = [1.6, 1.6, 1.4, 1.8, 1.8, 1.8]   # Virtual mass (light = responsive)
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]  # Damping
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Stiffness

FORCE_SCALE = 0.08
MIN_TANGENTIAL_FORCE = 2.5
FORCE_FILTER_ALPHA = 0.35  # ⬆️ INCREASED from 0.15 for faster response
DEADZONE = [1.5, 1.5, 2.0, 1.0, 1.0, 1.0]  # ⬇️ REDUCED from [3.5, 3.5, 4.0, ...] for sensitivity
MAX_ANGULAR_VEL = 25.0

# ========================== USER SETTINGS ==========================
X_MOVEMENT        = 300.0  # Linear move distance and arc radius
ARC_POINTS        = 1200   # Number of arc points
LINEAR_VEL        = 5.0    # Speed for automatic MoveL (%)
SERVO_UPDATE_RATE = 0.008  # 8ms = 125Hz for ServoJ

# FORCE-GUIDED PARAMETERS - SMOOTHNESS OPTIMIZED
FORCE_LOOKAHEAD   = 10      # How many points ahead to search based on force
FORCE_THRESHOLD   = 0.5     # ⬇️ REDUCED from 2.0N for smoother engagement
MAX_STEP_SIZE     = 2       # Maximum points to skip per update
INTERPOLATION_SMOOTHING = 0.25  # 🆕 Blend factor for smooth position transitions

# ====================== GLOBAL VARIABLES ======================
baseline_forces = [0.0] * 6
filtered_forces = [0.0] * 6
trajectory_points = []  # Will store all Cartesian points
trajectory_joints = []  # Will store all joint positions
current_trajectory_index = 0
current_trajectory_position = 0.0  # 🆕 Continuous (float) position for smooth interpolation
velocity_filter = 0.0  # 🆕 Smoothed velocity for gradual acceleration/deceleration

# ================== Helper Functions ==================
def euler_to_rot(rx, ry, rz):
    """Convert Euler angles to rotation matrix"""
    rx, ry, rz = map(math.radians, [rx, ry, rz])
    c, s = math.cos, math.sin
    Rx = np.array([[1,0,0],[0,c(rx),-s(rx)],[0,s(rx),c(rx)]])
    Ry = np.array([[c(ry),0,s(ry)],[0,1,0],[-s(ry),0,c(ry)]])
    Rz = np.array([[c(rz),-s(rz),0],[s(rz),c(rz),0],[0,0,1]])
    return Rz @ Ry @ Rx

def ema(new, old, a): 
    """Exponential moving average filter"""
    return a * new + (1 - a) * old

def smooth_deadzone(value, threshold, smoothing=0.5):
    """
    🆕 Smooth deadzone with gradual transition instead of hard cutoff
    Returns 0 near zero, gradually increasing past threshold
    """
    abs_val = abs(value)
    if abs_val < threshold:
        # Cubic smoothing in deadzone region
        ratio = abs_val / threshold
        factor = ratio * ratio * ratio  # Cubic easing
        return value * factor
    else:
        # Linear region past threshold, compensated for deadzone
        sign = 1 if value > 0 else -1
        return sign * (abs_val - threshold * (1 - smoothing))

def interpolate_joint_position(pos_a, pos_b, t):
    """
    🆕 Linearly interpolate between two joint positions
    t should be in [0, 1] where 0=pos_a, 1=pos_b
    """
    return [a + t * (b - a) for a, b in zip(pos_a, pos_b)]

def init_ft_sensor():
    """Initialize and zero the F/T sensor"""
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("✓ FT sensor initialized and zeroed")

def calibrate_baseline(samples=180):
    """Calibrate force/torque baseline by averaging samples"""
    global baseline_forces
    print("Calibrating force/torque baseline — keep tool completely still...")
    forces = []
    for _ in range(samples):
        ret = robot.FT_GetForceTorqueRCS()
        if ret[0] == 0:
            forces.append(ret[1][:6])
        time.sleep(0.01)
    if forces:
        baseline_forces = np.mean(forces, axis=0).tolist()
    print(f"✓ Baseline: {[f'{x:+.2f}' for x in baseline_forces]} N/Nm")

def get_corrected_forces():
    """Get force/torque data with baseline correction"""
    ret = robot.FT_GetForceTorqueRCS()
    if ret[0] == 0:
        raw = ret[1][:6]
        return [raw[i] - baseline_forces[i] for i in range(6)]
    return [0.0] * 6

def compute_tangential_force_on_arc(center_x, center_y):
    """
    Compute tangential force component that is ALIGNED with arc direction.
    🆕 Now with smooth deadzone processing for gradual engagement
    
    Returns: 
        tangential_force: Positive = forward along arc
                         Negative = backward along arc
                         Smoothly approaches zero near rest
    """
    global filtered_forces
    
    # Get current TCP position and orientation
    tcp_result = robot.GetActualTCPPose(flag=1)
    if tcp_result[0] != 0:
        return 0.0
    
    tcp_pose = tcp_result[1]
    tcp_x, tcp_y = tcp_pose[0], tcp_pose[1]
    orientation = tcp_pose[3:6]
    
    # Get corrected forces and apply EMA filter + SMOOTH deadzone
    corrected = get_corrected_forces()
    for i in range(6):
        # Apply smooth deadzone instead of hard cutoff
        corrected[i] = smooth_deadzone(corrected[i], DEADZONE[i % 6])
        filtered_forces[i] = ema(corrected[i], filtered_forces[i], FORCE_FILTER_ALPHA)
    
    # Transform forces from TCP frame to world frame
    R = euler_to_rot(*orientation)
    f_tcp = np.array([filtered_forces[0], filtered_forces[1], filtered_forces[2]])
    f_world = R @ f_tcp
    fx_world, fy_world = f_world[0], f_world[1]
    
    # Calculate radius vector from arc center to current TCP position
    radius_x = tcp_x - center_x
    radius_y = tcp_y - center_y
    radius_mag = math.sqrt(radius_x**2 + radius_y**2)
    
    if radius_mag < 1.0:  # Safety check - too close to center
        return 0.0
    
    # Normalize radius vector
    radius_x_norm = radius_x / radius_mag
    radius_y_norm = radius_y / radius_mag
    
    # Calculate arc tangent vector (perpendicular to radius)
    tangent_x = -radius_y_norm
    tangent_y = radius_x_norm
    
    # Project applied force onto arc tangent direction
    tangential_force = (fx_world * tangent_x + fy_world * tangent_y) * FORCE_SCALE
    
    return tangential_force


# =====================================================================
# PHASE 1: AUTOMATIC TRAJECTORY EXECUTION + STORAGE
# =====================================================================
print("\n" + "="*80)
print(" PHASE 1: AUTOMATIC TRAJECTORY EXECUTION")
print(" Building trajectory database for force-guided replay")
print("="*80)

# --------------------- Get Initial Position ---------------------
tcp_result = robot.GetActualTCPPose(flag=1)
if tcp_result[0] != 0:
    print(f"Failed to get TCP pose! Error: {tcp_result[0]}")
    sys.exit(1)

P0 = tcp_result[1]
x0, y0, z0, rx0, ry0, rz0 = P0
print(f"\nInitial Position (P0):")
print(f" X={x0:+.2f} Y={y0:+.2f} Z={z0:+.2f} | Rx={rx0:+.2f}° Ry={ry0:+.2f}° Rz={rz0:+.2f}°")

# --------------------- Move to Arc Center ---------------------
print(f"\n{'='*70}")
print(f" Moving to Arc Center (X - {X_MOVEMENT:.0f} mm)")
print(f"{'='*70}")

x_center = x0 - X_MOVEMENT
P1 = [x_center, y0, z0, rx0, ry0, rz0]

ret = robot.MoveL(desc_pos=P1, tool=0, user=0, vel=LINEAR_VEL, acc=0, ovl=100, blendR=-1)
if ret != 0:
    print(f"MoveL failed! Error: {ret}")
    sys.exit(1)

time.sleep(1.0)
current_pos = robot.GetActualTCPPose(flag=1)[1]
center_x, center_y = current_pos[0], current_pos[1]
print(f"✓ Arc Center: X={center_x:+.2f} Y={center_y:+.2f}")

# --------------------- Calculate Arc Geometry ---------------------
radius = X_MOVEMENT
dx = x0 - center_x
dy = y0 - center_y
start_angle_rad = math.atan2(dy, dx)
half_arc_rad = math.radians(90)

arc_start_angle = start_angle_rad - half_arc_rad
x_start = center_x + radius * math.cos(arc_start_angle)
y_start = center_y + radius * math.sin(arc_start_angle)
P_start = [x_start, y_start, z0, rx0, ry0, rz0]

print(f"\n{'='*70}")
print(f" Moving to Arc Start: X={x_start:+.2f} Y={y_start:+.2f}")
print(f"{'='*70}")

user_input = input("\nType 'ok' to move to arc start: ").strip().lower()
if user_input != 'ok':
    print("Cancelled.")
    sys.exit(0)

ret = robot.MoveL(desc_pos=P_start, tool=0, user=0, vel=LINEAR_VEL, acc=0, ovl=100, blendR=-1)
if ret != 0:
    print(f"MoveL failed! Error: {ret}")
    sys.exit(1)

time.sleep(1.0)
print("✓ At arc start position")

# --------------------- Generate and Store Trajectory ---------------------
print(f"\n{'='*80}")
print(f" GENERATING TRAJECTORY DATABASE")
print(f" Points: {ARC_POINTS + 1} | Radius: {radius:.1f} mm | Arc: 180°")
print(f"{'='*80}\n")

print("Computing trajectory points and joint positions...")

for i in range(ARC_POINTS + 1):
    # Calculate Cartesian point on arc
    angle_offset = -half_arc_rad + (i / ARC_POINTS) * (2 * half_arc_rad)
    angle_rad = start_angle_rad + angle_offset
    x_point = center_x + radius * math.cos(angle_rad)
    y_point = center_y + radius * math.sin(angle_rad)
    desc_pose = [x_point, y_point, z0, rx0, ry0, rz0]
    
    # Get joint configuration via IK
    ret_ik = robot.GetInverseKin(type=0, desc_pos=desc_pose, config=-1)
    if ret_ik[0] != 0:
        print(f"WARNING: IK failed at point {i}!")
        continue
    
    joint_pos = ret_ik[1][:6]
    
    # Store in trajectory database
    trajectory_points.append(desc_pose)
    trajectory_joints.append(joint_pos)
    
    if i % 100 == 0:
        print(f"  Generated point {i}/{ARC_POINTS}")

print(f"\n✓ Trajectory database complete: {len(trajectory_points)} points stored")

# --------------------- Execute Automatic Trajectory ---------------------
print(f"\n{'='*80}")
print(" EXECUTING AUTOMATIC TRAJECTORY (Slow ServoJ)")
print(f"{'='*80}\n")

user_input = input("Type 'ok' to execute automatic arc: ").strip().lower()
if user_input != 'ok':
    print("Cancelled.")
    sys.exit(0)

print("Executing automatic trajectory...\n")
start_time = time.time()

for i, joint_pos in enumerate(trajectory_joints):
    ret_servo = robot.ServoJ(joint_pos, [0]*6, 0.0, 0.0, SERVO_UPDATE_RATE, 0, 0)
    
    if ret_servo != 0:
        print(f"WARNING: ServoJ failed at point {i}!")
    
    if i % 100 == 0 or i == len(trajectory_joints) - 1:
        elapsed = time.time() - start_time
        x, y = trajectory_points[i][0], trajectory_points[i][1]
        print(f"Point {i:4d}/{len(trajectory_joints)-1} ({elapsed:.1f}s): X={x:7.2f} Y={y:7.2f}")
    
    time.sleep(SERVO_UPDATE_RATE)

auto_time = time.time() - start_time

print(f"\n✓ Automatic trajectory complete in {auto_time:.1f}s")
time.sleep(2.0)


# =====================================================================
# PHASE 2: FORCE-GUIDED TRAJECTORY REPLAY WITH ARC CONSTRAINT (SMOOTH)
# =====================================================================
print(f"\n{'='*80}")
print(" PHASE 2: FORCE-GUIDED TRAJECTORY REPLAY (SMOOTHNESS OPTIMIZED)")
print(" Same trajectory, but controlled by TANGENTIAL force only")
print(f"{'='*80}")

# Initialize FT sensor
init_ft_sensor()
calibrate_baseline()

# Return to start of trajectory
print(f"\nReturning to trajectory start...")
ret = robot.MoveL(desc_pos=trajectory_points[0], tool=0, user=0, vel=LINEAR_VEL, acc=0, ovl=100, blendR=-1)
if ret != 0:
    print(f"Return move failed! Error: {ret}")
    sys.exit(1)

time.sleep(2.0)
current_trajectory_index = 0
current_trajectory_position = 0.0  # Start at exact beginning
velocity_filter = 0.0

print(f"\n{'='*80}")
print(" FORCE-GUIDED MODE READY - SMOOTH ARC TANGENT CONSTRAINT")
print(f" Force threshold: {FORCE_THRESHOLD:.1f} N (soft)")
print(f" Trajectory points: {len(trajectory_points)}")
print(f" Arc Center: X={center_x:+.2f} Y={center_y:+.2f}")
print(f"")
print(f" ✨ SMOOTHNESS FEATURES:")
print(f" • Reduced deadzones for sensitivity")
print(f" • Soft force threshold with gradual engagement")
print(f" • Continuous interpolation between waypoints")
print(f" • Velocity filtering for smooth acceleration")
print(f"")
print(f" ⚠️  CONSTRAINT: Force must be tangent to arc path!")
print(f" • At arc LEFT   → Push UPWARD to move forward")
print(f" • At arc TOP    → Push RIGHTWARD to move forward") 
print(f" • At arc RIGHT  → Push DOWNWARD to move forward")
print(f"{'='*80}\n")

user_input = input("Type 'ok' to start force-guided mode (or 'skip' to end): ").strip().lower()
if user_input == 'skip':
    print("Skipping force-guided phase.")
    sys.exit(0)
elif user_input != 'ok':
    print("Cancelled.")
    sys.exit(0)

print("\n🤖 FORCE-GUIDED MODE ACTIVE (SMOOTH)")
print("   Reduced force threshold + continuous interpolation")
print("   Press Ctrl+C to stop\n")

try:
    loop_count = 0
    last_print = time.time()
    
    while True:
        loop_count += 1
        
        # Get tangential force component aligned with arc at current position
        tangential_force = compute_tangential_force_on_arc(center_x, center_y)
        
        # 🆕 SMOOTH VELOCITY CONTROL with soft threshold
        # Map force to desired velocity (points per update)
        if abs(tangential_force) > FORCE_THRESHOLD:
            # Proportional velocity based on force magnitude
            raw_velocity = (tangential_force / 3.0) * MAX_STEP_SIZE
            raw_velocity = max(-MAX_STEP_SIZE, min(MAX_STEP_SIZE, raw_velocity))
        else:
            # Gradual slowdown instead of instant stop
            raw_velocity = tangential_force * 0.5  # Allows subtle movements below threshold
        
        # Apply velocity smoothing (exponential filter)
        velocity_filter = ema(raw_velocity, velocity_filter, 0.3)
        
        # Update continuous position
        current_trajectory_position += velocity_filter
        current_trajectory_position = max(0.0, min(len(trajectory_joints) - 1.0, current_trajectory_position))
        
        # 🆕 INTERPOLATE between adjacent waypoints for smooth motion
        base_index = int(current_trajectory_position)
        next_index = min(base_index + 1, len(trajectory_joints) - 1)
        interpolation_factor = current_trajectory_position - base_index
        
        # Blend between current and next joint positions
        joint_target = interpolate_joint_position(
            trajectory_joints[base_index],
            trajectory_joints[next_index],
            interpolation_factor
        )
        
        # Send interpolated position to robot
        ret_servo = robot.ServoJ(joint_target, [0]*6, 0.0, 0.0, SERVO_UPDATE_RATE, 0, 0)
        
        if ret_servo == 0:
            current_trajectory_index = base_index
        
        # Print status every 0.5s
        if time.time() - last_print > 0.5:
            progress = (current_trajectory_position / len(trajectory_joints)) * 100
            
            # Interpolate Cartesian position for display
            x = trajectory_points[base_index][0] + interpolation_factor * (trajectory_points[next_index][0] - trajectory_points[base_index][0])
            y = trajectory_points[base_index][1] + interpolation_factor * (trajectory_points[next_index][1] - trajectory_points[base_index][1])
            
            # Direction indicator based on velocity
            if abs(velocity_filter) < 0.1:
                direction = "○ HOLD"
            elif velocity_filter > 0:
                direction = "→ FWD "
            else:
                direction = "← BACK"
            
            print(f"{direction} | Pos: {current_trajectory_position:7.2f}/{len(trajectory_joints)-1} ({progress:5.1f}%) | "
                  f"Force: {tangential_force:+6.2f}N | Vel: {velocity_filter:+5.2f} | X={x:7.2f} Y={y:7.2f}")
            last_print = time.time()
        
        # No automatic exit - user must press Ctrl+C to stop
        # This allows bidirectional movement along the entire trajectory
        
        time.sleep(SERVO_UPDATE_RATE)

except KeyboardInterrupt:
    print("\n\n⚠ Force-guided mode stopped by user")

print(f"\n{'='*80}")
print(" ✓ FORCE-GUIDED REPLAY COMPLETE")
print(f" Final position: {current_trajectory_position:.2f}/{len(trajectory_joints)-1}")
print(f"{'='*80}")
print("\n✓ All phases complete!")