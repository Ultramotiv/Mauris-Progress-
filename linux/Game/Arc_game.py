# =====================================================================
# FAIRINO ROBOT - ULTRA SMOOTH CONTROL (MAXIMUM STABILITY)
# → 1 Lap = +90° → -90° → back to +90° (round trip)
# → Switches IMMEDIATELY after ONE full lap
# → MAX JOINT VELOCITY: 60°/sec (buttery smooth)
# → ALL FUNCTIONALITY PRESERVED
# =====================================================================
# CHANGE ONLY THESE 3 LINES
ARC1_RADIUS = 300
ARC2_RADIUS = 200
ARC3_RADIUS = 100
# =====================================================================

import sys
import time
import math
import numpy as np
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
robot = Robot.RPC('192.168.58.2')

X_MOVEMENT = 300.0
ARC_POINTS = 1200
SERVO_RATE = 0.008

# ULTRA-SMOOTH TUNING - MAXIMUM STABILITY
FORCE_SCALE = 0.05               # Very gentle response
FILTER_ALPHA = 0.12              # Very strong force filtering
DEADZONE = [4.0, 4.0, 4.5, 2.5, 2.5, 2.5]  # Large deadzones to ignore noise
FORCE_THRESHOLD = 1.5            # High threshold - only respond to clear intent
MAX_STEP = 1.2                   # Small max step size
MAX_JOINT_VEL_DEG = 60.0         # Maximum joint velocity in degrees/second

# Multi-stage smoothing for ultra-stable motion
VELOCITY_SMOOTHING = 0.82        # Very heavy velocity smoothing
POSITION_SMOOTHING = 0.28        # Smooth position changes
JOINT_TARGET_SMOOTHING = 0.25    # Final joint command smoothing

center_x = center_y = 0.0
baseline = [0.0] * 6
filtered = [0.0] * 6

arcs = {
    ARC1_RADIUS: {"p": [], "j": [], "name": f"Arc1 {ARC1_RADIUS}mm"},
    ARC2_RADIUS: {"p": [], "j": [], "name": f"Arc2 {ARC2_RADIUS}mm"},
    ARC3_RADIUS: {"p": [], "j": [], "name": f"Arc3 {ARC3_RADIUS}mm"}
}

sequence = [ARC1_RADIUS, ARC2_RADIUS, ARC3_RADIUS]
current_index = 0
current_radius = sequence[current_index]

current_points = current_joints = []
pos = 0.0
smoothed_pos = 0.0  # Smoothed position tracking
vel = 0.0
prev_joints = None
smoothed_target = None

lap_counter = {ARC1_RADIUS: 0, ARC2_RADIUS: 0, ARC3_RADIUS: 0}
has_passed_bottom = False
lap_detected_this_cycle = False
transition_lock = False

def get_forces():
    """Enhanced force reading with maximum noise rejection"""
    global filtered
    tcp = robot.GetActualTCPPose(flag=1)[1]
    x, y = tcp[0], tcp[1]
    rx, ry, rz = map(math.radians, tcp[3:6])
    ret = robot.FT_GetForceTorqueRCS()
    if ret[0] != 0: return 0.0
    raw = [ret[1][i] - baseline[i] for i in range(6)]
    
    # Apply deadzone with cubic smoothing for gradual response
    for i in range(6):
        v = raw[i]
        av = abs(v)
        if av < DEADZONE[i % 6]:
            # Cubic deadzone for smooth transition
            v = v * (av / DEADZONE[i % 6]) ** 3
        else:
            # Linear outside deadzone
            v = v - DEADZONE[i % 6] if v > 0 else v + DEADZONE[i % 6]
        # Very strong filtering to eliminate noise
        filtered[i] = FILTER_ALPHA * v + (1 - FILTER_ALPHA) * filtered[i]
    
    # Transform to world frame
    c, s = math.cos, math.sin
    R = np.array([
        [c(rz)*c(ry), c(rz)*s(ry)*s(rx) - s(rz)*c(rx), c(rz)*s(ry)*c(rx) + s(rz)*s(rx)],
        [s(rz)*c(ry), s(rz)*s(ry)*s(rx) + c(rz)*c(rx), s(rz)*s(ry)*c(rx) - c(rz)*s(rx)],
        [-s(ry),      c(ry)*s(rx),                     c(ry)*c(rx)]
    ])
    f_world = R @ np.array(filtered[:3])
    fx, fy = f_world[0], f_world[1]
    
    # Calculate tangential force
    dx = x - center_x
    dy = y - center_y
    mag = math.hypot(dx, dy)
    if mag < 10: return 0.0
    tx = +dy / mag
    ty = -dx / mag
    return fx * tx + fy * ty

def ema(n, o, alpha=0.45): 
    """Exponential moving average for smooth filtering"""
    return alpha * n + (1 - alpha) * o

def interp(a, b, t): 
    """Linear interpolation between two joint configurations"""
    return [aa + t*(bb-aa) for aa, bb in zip(a, b)]

def compute_velocity_clamped_position(current_j, target_j, dt):
    """
    Compute velocity-limited joint positions with smooth clamping
    Ensures no joint exceeds MAX_JOINT_VEL_DEG
    """
    new_j = []
    for i in range(6):
        velocity = (target_j[i] - current_j[i]) / dt
        clamped_velocity = np.clip(velocity, -MAX_JOINT_VEL_DEG, MAX_JOINT_VEL_DEG)
        new_j.append(current_j[i] + clamped_velocity * dt)
    return new_j

def init_ft():
    """Initialize force-torque sensor"""
    robot.FT_SetConfig(24, 0)
    robot.FT_Activate(1)
    time.sleep(1.0)
    robot.SetLoadWeight(0, 0.0)
    robot.FT_SetZero(1)
    time.sleep(0.5)
    print("FT sensor initialized")

def calibrate():
    """Calibrate baseline forces with extended sampling"""
    global baseline
    print("Calibrating baseline (keep still for 3 seconds)...")
    forces = []
    for _ in range(300):  # Extended sampling for stable baseline
        r = robot.FT_GetForceTorqueRCS()
        if r[0] == 0: forces.append(r[1][:6])
        time.sleep(0.01)
    if forces:
        baseline = np.mean(forces, axis=0).tolist()
    print(f"Baseline calibrated: {[f'{x:+.2f}' for x in baseline]}")

print("\n" + "="*80)
print("FAIRINO - ULTRA SMOOTH CONTROL (MAXIMUM STABILITY)")
print("→ Max joint velocity: 60°/sec")
print("→ Triple-stage smoothing for buttery motion")
print("→ One full round trip = 1 lap → auto switch")
print("→ All functionality preserved")
print("="*80)

# Move to center position
p0 = robot.GetActualTCPPose(flag=1)[1]
robot.MoveL(desc_pos=[p0[0] - X_MOVEMENT, p0[1], p0[2], p0[3], p0[4], p0[5]], tool=0, user=0, vel=15, acc=0)
time.sleep(2.0)
center = robot.GetActualTCPPose(flag=1)[1]
center_x, center_y = center[0], center[1]

# Move to starting position (top of first arc)
robot.MoveL(desc_pos=[center_x, center_y + ARC1_RADIUS, p0[2], p0[3], p0[4], p0[5]], tool=0, user=0, vel=15, acc=0)
time.sleep(1.5)

# Generate all arc trajectories
for radius in [ARC1_RADIUS, ARC2_RADIUS, ARC3_RADIUS]:
    print(f"Generating {arcs[radius]['name']}...")
    arcs[radius]["p"] = []
    arcs[radius]["j"] = []
    for i in range(ARC_POINTS + 1):
        angle_deg = 90 - (i / ARC_POINTS) * 180
        theta = math.radians(angle_deg)
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        p = [x, y, p0[2], p0[3], p0[4], p0[5]]
        ik = robot.GetInverseKin(0, p, -1)
        j = ik[1][:6] if ik[0] == 0 else arcs[radius]["j"][-1] if arcs[radius]["j"] else [0]*6
        arcs[radius]["p"].append(p)
        arcs[radius]["j"].append(j)

# Initialize control variables
current_radius = ARC1_RADIUS
current_points = arcs[current_radius]["p"]
current_joints = arcs[current_radius]["j"]
pos = 0.0
smoothed_pos = 0.0

# Get initial joint positions
init_joints = robot.GetActualJointPosDegree(flag=1)[1][:6]
prev_joints = init_joints
smoothed_target = init_joints[:]

# Initialize sensor
init_ft()
calibrate()

print("\n" + "="*80)
print("AUTOMATIC MODE RUNNING - ULTRA SMOOTH CONTROL")
print("→ Push gently on FT sensor to move")
print("→ Buttery smooth motion - no vibrations")
print("="*80 + "\n")

last_print = time.time()

try:
    while True:
        if transition_lock:
            time.sleep(0.01)
            continue

        # Read force
        tangential_force = get_forces()

        # Calculate velocity with conservative scaling
        raw_v = tangential_force * FORCE_SCALE
        if abs(tangential_force) > FORCE_THRESHOLD:
            # More conservative for large forces
            raw_v = (tangential_force / 6.0) * MAX_STEP
        raw_v = max(-MAX_STEP, min(MAX_STEP, raw_v))
        
        # Stage 1: Heavy velocity smoothing
        vel = ema(raw_v, vel, VELOCITY_SMOOTHING)
        
        # Stage 2: Smooth position changes
        pos += vel
        pos = max(0.0, min(len(current_joints) - 1.0, pos))
        smoothed_pos = ema(pos, smoothed_pos, POSITION_SMOOTHING)

        # Use smoothed position for trajectory lookup
        i = int(smoothed_pos)
        t = smoothed_pos - i
        ni = min(i + 1, len(current_joints) - 1)
        desired_j = interp(current_joints[i], current_joints[ni], t)
        
        # Apply velocity clamping (60°/s limit)
        target_j = compute_velocity_clamped_position(prev_joints, desired_j, SERVO_RATE)
        
        # Stage 3: Final joint command smoothing
        for k in range(6):
            smoothed_target[k] = ema(target_j[k], smoothed_target[k], JOINT_TARGET_SMOOTHING)
        
        # Send smooth command to robot
        robot.ServoJ(smoothed_target, [0]*6, 0, 0, SERVO_RATE, 0, 0)
        prev_joints = smoothed_target[:]

        # Lap detection logic (using actual pos, not smoothed_pos)
        total = len(current_joints) - 1
        at_top = pos <= 80 or pos >= total - 80
        at_bottom = 0.35 < (pos / total) < 0.65

        # Detect passing bottom (going forward)
        if at_bottom and vel > 0.15 and not has_passed_bottom:
            has_passed_bottom = True
            print("   [Passed bottom - halfway done]")

        # FULL LAP: returned to top AFTER passing bottom
        if at_top and has_passed_bottom and pos <= 80 and not lap_detected_this_cycle:
            lap_counter[current_radius] += 1
            lap_detected_this_cycle = True
            print(f"\n>>> FULL LAP {lap_counter[current_radius]} COMPLETED on {arcs[current_radius]['name']} <<<")

            # AUTO SWITCH to next arc
            current_index = (current_index + 1) % 3
            next_radius = sequence[current_index]
            print(f">>> SWITCHING TO {arcs[next_radius]['name']} <<<\n")

            transition_lock = True
            robot.MoveL(desc_pos=arcs[next_radius]["p"][0], tool=0, user=0, vel=35, acc=0)
            time.sleep(1.2)

            # Reset for new arc
            current_radius = next_radius
            current_points = arcs[current_radius]["p"]
            current_joints = arcs[current_radius]["j"]
            pos = 0.0
            smoothed_pos = 0.0
            vel = 0.0
            
            # Update joint tracking
            prev_joints = robot.GetActualJointPosDegree(flag=1)[1][:6]
            smoothed_target = prev_joints[:]
            
            has_passed_bottom = False
            lap_detected_this_cycle = False
            transition_lock = False
            continue

        # Reset lap detection when leaving top zone
        if pos > 150:
            lap_detected_this_cycle = False

        # Status display
        if time.time() - last_print > 0.6:
            prog = pos / total * 100
            x = current_points[i][0] + t * (current_points[ni][0] - current_points[i][0])
            y = current_points[i][1] + t * (current_points[ni][1] - current_points[i][1])
            status = "FWD" if vel > 0.3 else "BACK" if vel < -0.3 else "STOP"
            top = " [TOP]" if at_top else ""
            bottom = " [BOTTOM]" if at_bottom else ""

            print(f"{status:4} {arcs[current_radius]['name']:>16} | Lap {lap_counter[current_radius]:>2} | "
                  f"{prog:5.1f}%{top}{bottom} | Ftan:{tangential_force:+5.2f} | Vel:{vel:+5.2f}")
            last_print = time.time()

        time.sleep(SERVO_RATE)

except KeyboardInterrupt:
    print("\n\n" + "="*80)
    print("STOPPED")
    for r in sequence:
        print(f"   {arcs[r]['name']:>16} → {lap_counter[r]} lap(s)")
    print("="*80)