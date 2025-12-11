# old 4.6 + Active to assistance based on force and not force+Speed
# assistive to passive is very working gooooood 
# need to remove jerks at the end of each cycle
# this is almost final adaptive mode 
# did not like the change we did here it slows down near end points!!!!!!!


import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import sys
import numpy as np

# --- Speed_only params copied below 4 lines ---
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# --- LIMITS ---
UPPER_LIMIT = 60.0
LOWER_LIMIT = 143.0
# Enhanced smooth transition zones for better deceleration
LIMIT_BUFFER = 5.0  # degrees - increased buffer zone for smoother deceleration
DECEL_ZONE = 8.0    # degrees - stronger deceleration zone
UPPER_SOFT_LIMIT = UPPER_LIMIT + LIMIT_BUFFER
LOWER_SOFT_LIMIT = LOWER_LIMIT - LIMIT_BUFFER

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
ASSISTANCE_HIGH = 0.7  # 70% assistance
ASSISTANCE_LOW = 0.01  # 1% assistance

# --- REP COUNTING ---
rep_count = 0
movement_threshold = 2.0  # deg
rep_state = None

# --- CONTROL ---
force_to_deg = 7.70
assistance_force_to_deg = 7.70
assistance_angle = 122.0
dt = 0.008
motion_paused = False

# --- GRAVITY COMPENSATION ---
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 0.5

# --- PASSIVE MODE ---
PASSIVE_START_LIMIT = 130.0
PASSIVE_COOLDOWN = 1.5  # seconds to wait after passive mode completion

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

def calculate_deceleration_force(position, velocity, joint_idx=2):
    """
    Calculate opposing force to create smooth deceleration near limits
    Returns a force that opposes motion when approaching limits
    """
    decel_force = 0.0
    
    # Distance to limits
    dist_to_upper = position - UPPER_LIMIT
    dist_to_lower = LOWER_LIMIT - position
    
    # Strong deceleration when approaching upper limit (moving up)
    if dist_to_upper < DECEL_ZONE and velocity < 0:  # Moving towards upper limit
        # Exponential deceleration force - gets stronger as closer to limit
        decel_strength = max(0.0, (DECEL_ZONE - dist_to_upper) / DECEL_ZONE)
        decel_force = decel_strength ** 2 * 15.0  # Strong opposing force
        
    # Strong deceleration when approaching lower limit (moving down)  
    elif dist_to_lower < DECEL_ZONE and velocity > 0:  # Moving towards lower limit
        # Exponential deceleration force - gets stronger as closer to limit
        decel_strength = max(0.0, (DECEL_ZONE - dist_to_lower) / DECEL_ZONE)
        decel_force = -decel_strength ** 2 * 15.0  # Strong opposing force
    
    return decel_force

def apply_enhanced_limits(position, velocity, joint_idx=2):
    """
    Apply enhanced position and velocity limits with strong deceleration
    """
    new_pos = position
    new_vel = velocity
    
    # Calculate deceleration force
    decel_force = calculate_deceleration_force(position, velocity, joint_idx)
    
    # Apply deceleration to velocity using physics model
    if abs(decel_force) > 0.1:
        decel_acc = decel_force / M[joint_idx]
        new_vel = velocity + decel_acc * dt
        
        # Additional velocity limiting based on distance to limit
        dist_to_upper = position - UPPER_LIMIT
        dist_to_lower = LOWER_LIMIT - position
        
        # Speed limit based on distance - closer = slower max speed
        if dist_to_upper < DECEL_ZONE and velocity < 0:
            max_speed = max(0.5, (dist_to_upper / DECEL_ZONE) * 3.0)
            if abs(new_vel) > max_speed:
                new_vel = -max_speed if new_vel < 0 else max_speed
                
        elif dist_to_lower < DECEL_ZONE and velocity > 0:
            max_speed = max(0.5, (dist_to_lower / DECEL_ZONE) * 3.0)
            if abs(new_vel) > max_speed:
                new_vel = max_speed if new_vel > 0 else -max_speed
    
    # Hard position limits with smooth stop
    if position <= UPPER_LIMIT:
        new_pos = UPPER_LIMIT
        if velocity < 0:  # Moving towards upper limit
            new_vel = 0.0
    elif position >= LOWER_LIMIT:
        new_pos = LOWER_LIMIT  
        if velocity > 0:  # Moving towards lower limit
            new_vel = 0.0
    
    return new_pos, new_vel

def count_repetitions(current_pos):
    global rep_count, rep_state
    near_min = abs(current_pos - UPPER_LIMIT) < movement_threshold
    near_max = abs(current_pos - LOWER_LIMIT) < movement_threshold
    if rep_state == "at_max":
        if not near_max:
            rep_state = "moving_down"
            print("⬇️ Moving DOWN from MAX")
    elif rep_state == "moving_down":
        if near_min:
            rep_state = "at_min"
            print("🔽 Reached MIN")
    elif rep_state == "at_min":
        if not near_min:
            rep_state = "moving_up"
            print("⬆️ Moving UP from MIN")
    elif rep_state == "moving_up":
        if near_max:
            rep_state = "at_max"
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
            forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2],
                      ft_data[1][3], ft_data[1][4], ft_data[1][5]]
            force_samples.append(forces)
        time.sleep(0.01)
    if force_samples:
        baseline_forces = [sum(forces[i] for forces in force_samples)/len(force_samples)
                           for i in range(6)]
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

# initialize state machine for rep counting
if abs(home_pos[2]-LOWER_LIMIT) < abs(home_pos[2]-UPPER_LIMIT):
    rep_state = "at_max"
else:
    rep_state = "at_min"

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
    mode_state = "active"
    previous_pos = desired_pos[2]
    avg_speed_window = []
    SPEED_AVG_WINDOW = 30
    assistance_level = 0.0
    passive_timer = None
    passive_direction = None
    passive_waiting = False
    passive_prev_pos = None

    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt); continue
        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2],
                      ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
        for i in range(6):
            if abs(forces[i]) < 0.5: forces[i] = 0.0
        j = 2
        fz_force = forces[2]

        current_velocity = velocity[j] * force_to_deg
        current_speed = abs(current_velocity)

        # Rolling average speed
        avg_speed_window.append(current_speed)
        if len(avg_speed_window) > SPEED_AVG_WINDOW:
            avg_speed_window.pop(0)
        rolling_avg_speed = sum(avg_speed_window)/len(avg_speed_window)

        # Movement state
        movement_state = None
        if current_velocity > 0.1:
            movement_state = "moving_up"
        elif current_velocity < -0.1:
            movement_state = "moving_down"

        # --- PASSIVE MODE LOGIC ---
        if current_speed < 0.5 and desired_pos[j] <= PASSIVE_START_LIMIT:
            if not passive_waiting:
                passive_timer = time.time()
                passive_waiting = True
                err_pos, actual_jpos = robot.GetActualJointPosDegree()
                passive_prev_pos = actual_jpos[2] if err_pos==0 else desired_pos[j]
                passive_direction = None
                in_passive_rep = False
            else:
                elapsed = time.time() - passive_timer
                if not (current_speed < 0.5 and desired_pos[j] <= PASSIVE_START_LIMIT):
                    passive_waiting = False
                    passive_timer = None
                    passive_direction = None
                    passive_prev_pos = None
                    continue

                if passive_direction is None and passive_prev_pos is not None:
                    err_pos, actual_jpos = robot.GetActualJointPosDegree()
                    actual_pos_j = actual_jpos[2] if err_pos==0 else desired_pos[j]
                    eps = 1e-3
                    if actual_pos_j < passive_prev_pos - eps:
                        passive_direction = "up"
                    elif actual_pos_j > passive_prev_pos + eps:
                        passive_direction = "down"
                    else:
                        passive_direction = "down" if rep_state!="moving_up" else "up"

                if elapsed >= 0.2 and passive_direction:
                    # --- Passive move UP with enhanced deceleration ---
                    if passive_direction == "up":
                        while abs(desired_pos[j]-UPPER_LIMIT) > 0.5:
                            assist_force = 10.0
                            acc = (assist_force - B[j]*velocity[j])/M[j]
                            velocity[j] += acc*dt
                            delta = velocity[j]*dt*force_to_deg
                            potential_pos = desired_pos[j] - abs(delta)
                            
                            # Apply enhanced limits with strong deceleration
                            potential_pos, velocity[j] = apply_enhanced_limits(
                                potential_pos, velocity[j], j)
                            
                            desired_pos[j] = potential_pos
                            robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
                            time.sleep(dt)

                    # --- Passive move DOWN with enhanced deceleration ---
                    elif passive_direction == "down":
                        while abs(desired_pos[j]-LOWER_LIMIT) > 0.5:
                            assist_force = -10.0
                            acc = (assist_force - B[j]*velocity[j])/M[j]
                            velocity[j] += acc*dt
                            delta = velocity[j]*dt*force_to_deg
                            potential_pos = desired_pos[j] + abs(delta)
                            
                            # Apply enhanced limits with strong deceleration
                            potential_pos, velocity[j] = apply_enhanced_limits(
                                potential_pos, velocity[j], j)
                            
                            desired_pos[j] = potential_pos
                            robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
                            time.sleep(dt)

                    # ✅ Cooldown after passive rep
                    time.sleep(PASSIVE_COOLDOWN)

                    # --- Smooth velocity reset ---
                    velocity[j] *= 0.1  # Gradual velocity reduction instead of instant zero
                    home_pos[j] = desired_pos[j]

                    passive_waiting = False
                    passive_timer = None
                    passive_direction = None
                    passive_prev_pos = None
                    in_passive_rep = False
                    continue
        else:
            passive_waiting = False
            passive_timer = None
            passive_direction = None
            passive_prev_pos = None

        # --- MODE SWITCHING & ASSISTANCE LOGIC ---
        if 0.0 < abs(fz_force) < 5.0:
            assistance_level = ASSISTANCE_HIGH
            mode_state = "Assistive"
        else:
            assistance_level = 0.0
            mode_state = "active"

        assisted_force = fz_force
        if assistance_level > 0.0:
            assist = assistance_level * fz_force
            assisted_force = fz_force + assist

        if rep_state == "moving_down" and assisted_force > 0:
            assisted_force = 0.0
        elif rep_state == "moving_up" and assisted_force < 0:
            assisted_force = 0.0

        if abs(assisted_force) < 1.0 and current_speed < 0.5:
            home_pos[j] = desired_pos[j]
            spring_force = -K[j]*(desired_pos[j]-home_pos[j])/force_to_deg
            acc = (spring_force - B[j]*velocity[j])/M[j]
        else:
            acc = (assisted_force - B[j]*velocity[j])/M[j]

        velocity[j] += acc*dt
        delta = velocity[j]*dt*force_to_deg
        potential_pos = desired_pos[j] + delta
        
        # ✅ APPLY ENHANCED LIMITS with strong deceleration
        potential_pos, velocity[j] = apply_enhanced_limits(potential_pos, velocity[j], j)
        
        desired_pos[j] = potential_pos

        count_repetitions(desired_pos[j])
        previous_pos = desired_pos[j]

        for lock_j in range(6):
            if (mode=="sitting" and lock_j not in [1,2,3]) or (mode=="standing" and lock_j!=2):
                desired_pos[lock_j]=home_pos[lock_j]
                velocity[lock_j]=0.0

        safety_ok=True
        for idx in range(6):
            joint_num=idx+1
            if not is_within_safety_limits(joint_num,desired_pos[idx]):
                safety_ok=False
        if not safety_ok:
            time.sleep(dt)
            continue

        print(f"Mode: {mode_state.upper()} | Reps: {rep_count} | State: {rep_state} | "
              f"Dir: {movement_state} | Speed: {current_speed:.2f}°/s | "
              f"Avg: {rolling_avg_speed:.2f}°/s | Pos: {desired_pos[j]:.1f}° | "
              f"Force: {fz_force:.2f}N | Assistance: {assistance_level*100:.0f}%")

        robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        time.sleep(dt)

control_loop()


