# Done for better adaptive mode 
# Merged Active/Guided Mode with Speed-Based Assistance and Rep Counting
# speed and force based with rep counting and 200% assistance 
#little adaptive # falty code as passive is always max to min to max
# now trying to change to passive assistance
import Robot
import time
import signal
import sys
import numpy as np

# --- PARAMETERS PER JOINT ---
M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# --- LIMITS ---
JOINT2_MIN_LIMIT = 60.0
JOINT2_MAX_LIMIT = 144.0
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
ASSISTANCE_HIGH = 2.0  # 50% assistance
ASSISTANCE_LOW = 0.2   # 20% assistance

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
    return JOINT2_MIN_LIMIT <= target_position <= JOINT2_MAX_LIMIT

def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True

def count_repetitions(current_pos):
    global rep_count, rep_state
    near_min = abs(current_pos - JOINT2_MIN_LIMIT) < movement_threshold
    near_max = abs(current_pos - JOINT2_MAX_LIMIT) < movement_threshold
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

# initialize state machine for rep counting
if abs(home_pos[2]-JOINT2_MAX_LIMIT) < abs(home_pos[2]-JOINT2_MIN_LIMIT):
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
    mode_state = "active"  # can be 'active' or 'guided'
    previous_pos = desired_pos[2]
    avg_speed_window = []
    SPEED_AVG_WINDOW = 30
    assistance_level = 0.0
    passive_timer = None
    passive_direction = None
    passive_waiting = False
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
        rolling_avg_speed = sum(avg_speed_window)/len(avg_speed_window)
        movement_direction = "up" if current_velocity > 0.1 else ("down" if current_velocity < -0.1 else "stopped")

        # --- PASSIVE MODE LOGIC ---
        # Passive mode only if:
        #   - 0 < force < 4N
        #   - 0 < speed < AVG_SPEED
        #   - Passive mode only starts after force and speed remain in range for 1 second
        #   - Passive mode can only start if position is <= 130 (not at 144)
        PASSIVE_START_LIMIT = 130.0
        if (0.0 < abs(fz_force) < 4.0 and 0.0 < current_speed < AVG_SPEED and desired_pos[j] <= PASSIVE_START_LIMIT):
            if not passive_waiting:
                passive_timer = time.time()
                passive_waiting = True
                passive_direction = movement_direction if movement_direction in ["up", "down"] else None
                in_passive_rep = False
            else:
                elapsed = time.time() - passive_timer if passive_timer else 0
                print(f"[PASSIVE TIMER] 0 < Force < 4N and 0 < Speed < {AVG_SPEED} for {elapsed:.1f} seconds (direction: {passive_direction})")
                # If force, speed, or position go out of range, reset timer and waiting
                if not (0.0 < abs(fz_force) < 4.0 and 0.0 < current_speed < AVG_SPEED and desired_pos[j] <= PASSIVE_START_LIMIT):
                    passive_waiting = False
                    passive_timer = None
                    passive_direction = None
                    continue
                # Only exit passive if force >= 10N during timer
                ft_data_check = robot.FT_GetForceTorqueRCS()
                if ft_data_check[0] == 0:
                    raw_forces_check = [ft_data_check[1][0], -ft_data_check[1][1], ft_data_check[1][2], ft_data_check[1][3], ft_data_check[1][4], ft_data_check[1][5]]
                    forces_check = [raw_forces_check[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces_check
                    fz_force_check = forces_check[2]
                    if abs(fz_force_check) >= 10.0:
                        print("[PASSIVE] Force >= 10N detected during timer. Exiting passive mode and resuming assistance.")
                        passive_waiting = False
                        passive_timer = None
                        passive_direction = None
                        continue
                if elapsed >= 1.0:
                    print("[PASSIVE] Timer met. Triggering passive mode and completing full passive rep: MAX → MIN → MAX.")
                    in_passive_rep = True
                    # 1. Assist to JOINT2_MIN_LIMIT
                    while abs(desired_pos[j] - JOINT2_MIN_LIMIT) > 0.5:
                        assist_force = -10.0
                        acc = (assist_force - B[j]*velocity[j]) / M[j]
                        velocity[j] += acc*dt
                        delta = velocity[j]*dt*force_to_deg
                        potential_pos = desired_pos[j] + delta
                        if potential_pos < JOINT2_MIN_LIMIT:
                            potential_pos = JOINT2_MIN_LIMIT
                            velocity[j] = 0.0
                        desired_pos[j] = potential_pos
                        robot.ServoJ(joint_pos=desired_pos,axisPos=[0]*6)
                        print(f"[PASSIVE] Assisting to MIN: {desired_pos[j]:.1f}°")
                        ft_data = robot.FT_GetForceTorqueRCS()
                        if ft_data[0] == 0:
                            raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
                            forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
                            fz_force = forces[2]
                            if abs(fz_force) >= 10.0:
                                print("[PASSIVE] Force >= 10N detected. Exiting passive assist.")
                                in_passive_rep = False
                                break
                        time.sleep(dt)
                    # 2. Assist back to JOINT2_MAX_LIMIT (144)
                    while in_passive_rep and abs(desired_pos[j] - JOINT2_MAX_LIMIT) > 0.5:
                        assist_force = 10.0
                        acc = (assist_force - B[j]*velocity[j]) / M[j]
                        velocity[j] += acc*dt
                        delta = velocity[j]*dt*force_to_deg
                        potential_pos = desired_pos[j] + delta
                        if potential_pos > JOINT2_MAX_LIMIT:
                            potential_pos = JOINT2_MAX_LIMIT
                            velocity[j] = 0.0
                        desired_pos[j] = potential_pos
                        robot.ServoJ(joint_pos=desired_pos,axisPos=[0]*6)
                        print(f"[PASSIVE] Assisting to MAX: {desired_pos[j]:.1f}°")
                        ft_data = robot.FT_GetForceTorqueRCS()
                        if ft_data[0] == 0:
                            raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
                            forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
                            fz_force = forces[2]
                            if abs(fz_force) >= 10.0:
                                print("[PASSIVE] Force >= 10N detected. Exiting passive assist.")
                                in_passive_rep = False
                                break
                        time.sleep(dt)
                    # Only after full rep, allow mode change
                    passive_waiting = False
                    passive_timer = None
                    passive_direction = None
                    in_passive_rep = False
        else:
            passive_waiting = False
            passive_timer = None
            passive_direction = None

        # --- MODE SWITCHING & ASSISTANCE LOGIC ---
        if passive_waiting and passive_timer and (time.time() - passive_timer) >= 10.0:
            # After moving to limit, hold at limit until force is received
            while True:
                ft_data = robot.FT_GetForceTorqueRCS()
                if ft_data[0] == 0:
                    raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
                    forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
                    fz_force = forces[2]
                robot.ServoJ(joint_pos=desired_pos,axisPos=[0]*6)
                print(f"[PASSIVE] Holding at limit {desired_pos[j]:.1f}°. Waiting for force to exit passive mode.")
                if abs(fz_force) > 0.0:
                    print("[PASSIVE] Force detected. Exiting passive mode.")
                    passive_waiting = False
                    passive_timer = None
                    passive_direction = None
                    break
                time.sleep(dt)

        # --- MODE SWITCHING & ASSISTANCE LOGIC ---
        if abs(fz_force) > 0.0 and current_speed <= AVG_SPEED:
            assistance_level = ASSISTANCE_HIGH
            if mode_state != "guided":
                print("[ASSISTANCE] 50% assistance (force nonzero, speed below or equal to 6.0)")
            mode_state = "guided"
        elif abs(fz_force) > 0.0 and current_speed > AVG_SPEED:
            assistance_level = ASSISTANCE_LOW
            if mode_state != "active":
                print("[ASSISTANCE] Reducing to 20% (force nonzero, speed above 6.0)")
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
        # Direction enforcement for rep state
        if rep_state == "moving_down" and movement_direction == "up":
            assisted_force = min(0.0, assisted_force)
            print("⛔ Ignoring UP movement, must go DOWN to MIN first")
        elif rep_state == "moving_up" and movement_direction == "down":
            assisted_force = max(0.0, assisted_force)
            print("⛔ Ignoring DOWN movement, must go UP to MAX first")
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
        if potential_pos < JOINT2_MIN_LIMIT:
            potential_pos = JOINT2_MIN_LIMIT
        elif potential_pos > JOINT2_MAX_LIMIT:
            potential_pos = JOINT2_MAX_LIMIT
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


