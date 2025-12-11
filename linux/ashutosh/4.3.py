# Done
# Merged Active/Guided/passive Mode with speed based guideed mode and force-based Assistance and Rep Counting
# speed and force based with rep counting and 80% assistance 
# this code shifts between active to passive continously based on position and force

#Along with this now it is restricted from moving to lower limit if its not reached upper limit and vice versa
# i.e it has to complete full rep in and then only it can go to other limit

## this is modification of 4.2.py code so that speed based decision will be taken 
# here if speed is 0 passive shift occurs 

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np

# --- PARAMETERS PER JOINT ---
# M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
# B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
# K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
# force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]
# --- Speed_only params copied below 4 lines ---
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]
# --- LIMITS ---
UPPER_LIMIT = 60.0   # Up position (smaller angle)
LOWER_LIMIT = 143.0  # Down position (larger angle)
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),
    2: (-179.0, -35.0),
    3: (2.0, 144.0),
    4: (-258.0, 80.0),
    5: (-170.0, 12.0),
    6: (-170.0, 170.0),
}

# --- SPEED/ASSISTANCE ---
AVG_SPEED = 7.70  # deg/sec
TARGET_SPEED = AVG_SPEED
ASSISTANCE_HIGH = 0.4  #0.8 assistance
ASSISTANCE_LOW = 0.01   # 20% assistance

# --- REP COUNTING ---
rep_count = 0
movement_threshold = 2.0  # deg
rep_state = None

# --- CONTROL ---
force_to_deg = 7.70
assistance_force_to_deg = 7.70
assistance_angle = 122.0  # for guided mode, not strictly used here

dt = 0.008
motion_paused = False

# --- GRAVITY COMPENSATION ---
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 0.5

# --- ROBOT CONNECTION ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

def check_joint2_limits(target_position):
    return UPPER_LIMIT <= target_position <= LOWER_LIMIT

def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True

def count_repetitions(current_pos):
    global rep_count, rep_state
    near_upper = abs(current_pos - UPPER_LIMIT) < movement_threshold    # Near 60°
    near_lower = abs(current_pos - LOWER_LIMIT) < movement_threshold    # Near 143°
    
    if rep_state == "at_lower":        # At 143° (down position)
        if not near_lower:
            rep_state = "moving_up"
            print("⬆️ Moving UP from LOWER (143°)")
    elif rep_state == "moving_up":     # Moving from 143° to 60°
        if near_upper:
            rep_state = "at_upper"
            print("🔼 Reached UPPER (60°)")
    elif rep_state == "at_upper":      # At 60° (up position)
        if not near_upper:
            rep_state = "moving_down"
            print("⬇️ Moving DOWN from UPPER (60°)")
    elif rep_state == "moving_down":   # Moving from 60° to 143°
        if near_lower:
            rep_state = "at_lower"
            rep_count += 1
            print(f"🎯 REP COMPLETED! Total Reps: {rep_count}")

def init_ft_sensor():
    company = 24
    device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0); time.sleep(0.5)
    robot.FT_Activate(1); time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0)
    robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0); time.sleep(0.5)
    robot.FT_SetZero(1); time.sleep(0.5)
    print("FT Sensor initialized.")

def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating baseline forces...")
    force_samples = []
    for i in range(gravity_compensation_samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:
            forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
            force_samples.append(forces)
        time.sleep(0.01)
    if force_samples:
        baseline_forces = [sum(forces[i] for forces in force_samples)/len(force_samples) for i in range(6)]
        print("Baseline forces:", baseline_forces)
    else:
        baseline_forces = [0.0]*6

def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nServo stopped. Exiting.")
    sys.exit(0)
signal.signal(signal.SIGINT, shutdown)

# --- SETUP ---
init_ft_sensor()
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0: sys.exit(1)
if not check_joint2_limits(joint_pos[2]): sys.exit(1)
calibrate_baseline_forces()
home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0]*6

# initialize state machine for rep counting - CORRECTED LOGIC
if abs(home_pos[2] - LOWER_LIMIT) < abs(home_pos[2] - UPPER_LIMIT):
    rep_state = "at_lower"  # Closer to 143° (lower/down position)
else:
    rep_state = "at_upper"  # Closer to 60° (upper/up position)

# --- USER INPUT ---
mode = None
while mode not in ["sitting","standing"]:
    mode = input("Select mode (sitting/standing): ").strip().lower()
if mode == "sitting":
    free_joints = [1,2,3]
else:
    free_joints = [2]

if robot.ServoMoveStart() != 0: sys.exit(1)
time.sleep(startup_delay)

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces, rep_count, rep_state
    mode_state = "active"  # can be 'active', 'guided', or 'passive'
    previous_pos = desired_pos[2]
    avg_speed_window = []
    SPEED_AVG_WINDOW = 30
    assistance_level = 0.0
    passive_timer = None
    passive_direction = None
    passive_waiting = False
    in_passive_rep = False
    passive_at_limit = False  # New flag to track if we're at the limit in passive mode
    
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt); continue
        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
        for i in range(6):
            if abs(forces[i]) < 0.5: forces[i] = 0.0
        j = 2  # Joint 2 for standing
        fz_force = forces[2]
        # Calculate speed
        current_velocity = velocity[j] * force_to_deg
        current_speed = abs(current_velocity)
        # Rolling average speed
        avg_speed_window.append(current_speed)
        if len(avg_speed_window) > SPEED_AVG_WINDOW:
            avg_speed_window.pop(0)
        rolling_avg_speed = sum(avg_speed_window)/len(avg_speed_window) if avg_speed_window else 0.0

        # --- CORRECTED DIRECTION LOGIC ---
        movement_direction = None
        if desired_pos[j] > previous_pos:
            movement_direction = "up"    # Moving from 143° towards 60° (going up - decreasing angle)
        elif desired_pos[j] < previous_pos:
            movement_direction = "down"  # Moving from 60° towards 143° (going down - increasing angle)
        else:
            movement_direction = "stopped"

        # --- CORRECTED PASSIVE MODE LOGIC ---
        # Passive mode ALLOWED: 60° to 125° (activation zone)
        # Passive mode BLOCKED: 125° to 144° (dead zone)
        
        # Check if in dead zone (passive mode disabled)
        in_dead_zone = 125.0 < desired_pos[j] <= 144.0
        
        if in_dead_zone:
            # Cancel any ongoing passive mode activities in dead zone
            if passive_waiting or in_passive_rep:
                passive_waiting = False
                passive_timer = None
                passive_direction = None
                in_passive_rep = False
                mode_state = "active"
                print(f"[PASSIVE] In dead zone ({desired_pos[j]:.1f}°). Passive mode disabled.")
        
        # Passive mode trigger: 60° to 125° and speed <= 0.5°/s
        elif 60.0 <= desired_pos[j] <= 125.0 and current_speed <= 0.5:
            # Determine passive direction based on position when stopped
            if movement_direction == "stopped":
                # When stopped, determine direction based on position relative to midpoint of active zone
                midpoint = (60.0 + 125.0) / 2  # 92.5°
                if desired_pos[j] < midpoint:
                    movement_direction = "up"    # Closer to 60°, go up
                else:
                    movement_direction = "down"  # Closer to 125°, go down toward 144°
                print(f"[PASSIVE] Stopped at {desired_pos[j]:.1f}°, inferring direction: {movement_direction}")
            
            if not passive_waiting and not in_passive_rep:
                passive_timer = time.time()
                passive_waiting = True
                passive_direction = movement_direction
                in_passive_rep = False
                passive_at_limit = False
                print(f"[PASSIVE TIMER] Speed <= 0.5. Starting timer for direction: {passive_direction}")
            elif passive_waiting:
                elapsed = time.time() - passive_timer if passive_timer else 0
                print(f"[PASSIVE TIMER] Speed <= 0.5 for {elapsed:.1f} seconds in activation zone (60°-125°) (direction: {passive_direction})")
                if elapsed < 1.0 and not in_passive_rep:
                    # Check if conditions are still met
                    if current_speed > 0.5 or not (60.0 <= desired_pos[j] <= 125.0):
                        passive_waiting = False
                        passive_timer = None
                        passive_direction = None
                        print("[PASSIVE] Speed > 0.5 or out of activation zone. Cancelling passive timer.")
                        continue
                if elapsed >= 1.0 and not in_passive_rep:
                    print(f"[PASSIVE] Speed <= 0.5 for 1s with direction {passive_direction}. Entering passive mode.")
                    in_passive_rep = True
                    mode_state = "passive"
                    passive_at_limit = False
                
            # In passive mode, always apply 20N force in correct direction until limit is reached
            if in_passive_rep:
                passive_force = 20.0
                if passive_direction == "up":
                    # Moving up means going to 60° (UPPER_LIMIT) - decreasing angle
                    acc = (-passive_force - B[j]*velocity[j]) / M[j]
                    velocity[j] += acc*dt
                    delta = velocity[j]*dt*force_to_deg
                    desired_pos[j] += delta  # Apply the calculated delta
                    if desired_pos[j] < UPPER_LIMIT:
                        desired_pos[j] = UPPER_LIMIT
                    if abs(desired_pos[j] - UPPER_LIMIT) < 1.0:
                        print(f"[PASSIVE] Reached UPPER_LIMIT: {UPPER_LIMIT}°")
                        in_passive_rep = False
                        passive_waiting = False
                        passive_timer = None
                        passive_direction = None
                        mode_state = "active"
                elif passive_direction == "down":
                    # Moving down means going to 143° (LOWER_LIMIT) - increasing angle
                    acc = (passive_force - B[j]*velocity[j]) / M[j]
                    velocity[j] += acc*dt
                    delta = velocity[j]*dt*force_to_deg
                    desired_pos[j] += delta  # Apply the calculated delta
                    if desired_pos[j] > LOWER_LIMIT:
                        desired_pos[j] = LOWER_LIMIT
                    if abs(desired_pos[j] - LOWER_LIMIT) < 1.0:
                        print(f"[PASSIVE] Reached LOWER_LIMIT: {LOWER_LIMIT}°")
                        in_passive_rep = False
                        passive_waiting = False
                        passive_timer = None
                        passive_direction = None
                        mode_state = "active"
                else:
                    # If stopped, do not hold, just exit passive mode
                    in_passive_rep = False
                    passive_waiting = False
                    passive_timer = None
                    passive_direction = None
                    mode_state = "active"
                robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
                time.sleep(dt)
                continue
        else:
            if not in_passive_rep:
                passive_waiting = False
                passive_timer = None
                passive_direction = None
                passive_at_limit = False

        if mode_state == "passive":
            time.sleep(dt)
            continue

        # --- MODE SWITCHING & ASSISTANCE LOGIC (Active/Guided) ---
        # Modified logic: Guided mode only when force > 0.0 and 0.5 < speed <= AVG_SPEED
        if abs(fz_force) > 0.0 and 0.5 < current_speed <= AVG_SPEED:
            assistance_level = ASSISTANCE_HIGH
            if mode_state != "guided":
                print(f"[ASSISTANCE] 40% assistance (force nonzero, speed {current_speed:.2f}°/s in range 0.5-7.70)")
            mode_state = "guided"
        elif abs(fz_force) > 0.0 and current_speed > AVG_SPEED:
            assistance_level = ASSISTANCE_LOW
            if mode_state != "active":
                print(f"[ASSISTANCE] Reducing to 1% (force nonzero, speed {current_speed:.2f}°/s above 7.70)")
            mode_state = "active"
        elif abs(fz_force) > 0.0 and current_speed <= 0.5:
            assistance_level = ASSISTANCE_LOW
            if mode_state != "active":
                print(f"[ASSISTANCE] 1% assistance (force nonzero, speed {current_speed:.2f}°/s <= 0.5)")
            mode_state = "active"
        else:
            assistance_level = 0.0
            if mode_state != "active":
                print("[ASSISTANCE] No assistance (force zero)")
            mode_state = "active"
        
        # --- CONTROL LOGIC ---
        assisted_force = fz_force
        if assistance_level > 0.0:
            assist = assistance_level * fz_force
            assisted_force = fz_force + assist
        
        # --- CORRECTED REP STATE ENFORCEMENT ---
        if rep_state == "moving_up":
            # Only allow movement towards UPPER_LIMIT (60°) - upward force should decrease angle
            if assisted_force > 0:  # Positive force pushes down (increases angle)
                assisted_force = 0.0
                print("⛔ Ignoring DOWN force, must go UP to 60° first")
        elif rep_state == "moving_down":
            # Only allow movement towards LOWER_LIMIT (143°) - downward force should increase angle
            if assisted_force < 0:  # Negative force pulls up (decreases angle)
                assisted_force = 0.0
                print("⛔ Ignoring UP force, must go DOWN to 143° first")
        
        # Motion update
        if abs(assisted_force) < 1.0 and current_speed < 0.5:
            home_pos[j] = desired_pos[j]
            spring_force = -K[j]*(desired_pos[j]-home_pos[j])/force_to_deg
            acc = (spring_force - B[j]*velocity[j]) / M[j]
        else:
            acc = (assisted_force - B[j]*velocity[j]) / M[j]
        
        velocity[j] += acc*dt
        delta = velocity[j]*dt*force_to_deg
        potential_pos = desired_pos[j] + delta
        
        # Joint 2 limits
        if potential_pos < UPPER_LIMIT:
            potential_pos = UPPER_LIMIT
        elif potential_pos > LOWER_LIMIT:
            potential_pos = LOWER_LIMIT
        
        desired_pos[j] = potential_pos
        
        # Rep counting
        count_repetitions(desired_pos[j])
        previous_pos = desired_pos[j]
        
        # Lock other joints
        for lock_j in range(6):
            if (mode=="sitting" and lock_j not in [1,2,3]) or (mode=="standing" and lock_j!=2):
                desired_pos[lock_j]=home_pos[lock_j]; velocity[lock_j]=0.0
        
        # Safety checks
        safety_ok=True
        for idx in range(6):
            joint_num=idx+1
            if not is_within_safety_limits(joint_num,desired_pos[idx]): safety_ok=False
        if not safety_ok: time.sleep(dt); continue
        
        # --- TERMINAL OUTPUT ---
        print(f"Mode: {mode_state.upper()} | Reps: {rep_count} | State: {rep_state} | Dir: {movement_direction} | "
              f"Speed: {current_speed:.2f}°/s | Avg: {rolling_avg_speed:.2f}°/s | Pos: {desired_pos[j]:.1f}° | "
              f"Force: {fz_force:.2f}N | Assistance: {assistance_level*100:.0f}%")
        
        robot.ServoJ(joint_pos=desired_pos,axisPos=[0]*6)
        time.sleep(dt)

control_loop()