#almost_final.py file modified for joint 6

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import sys
import numpy as np

# --- Speed_only params copied below 4 lines ---
M = [3.0, 3.0, 2.0, 3.0, 3.0, 1.5]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 2.5]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.0]

# --- LIMITS FOR JOINT 6 ---
UPPER_LIMIT = -158.0
LOWER_LIMIT = -20.0
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
ASSISTANCE_HIGH = 0.95  # 70% assistance
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
PASSIVE_START_LIMIT = 0.0  # Adjust based on joint 6 behavior
PASSIVE_COOLDOWN = 1.5  # seconds to wait after passive mode completion

# --- ROBOT CONNECTION ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

def check_joint6_limits(target_position):
    return UPPER_LIMIT <= target_position <= LOWER_LIMIT

def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True

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
        print("Warning: Could not capture baseline forces!")
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
if not check_joint6_limits(joint_pos[5]): sys.exit(1)  # Check joint 6 (index 5)
calibrate_baseline_forces()
home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0]*6

# initialize state machine for rep counting (for joint 6)
if abs(home_pos[5]-LOWER_LIMIT) < abs(home_pos[5]-UPPER_LIMIT):
    rep_state = "at_max"
else:
    rep_state = "at_min"

# --- ONLY JOINT 6 IS FREE ---
free_joints = [6]
print("Mode: Joint 6 only (all other joints locked)")

if robot.ServoMoveStart() != 0: sys.exit(1)
time.sleep(startup_delay)

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces, rep_count, rep_state
    mode_state = "active"
    previous_pos = desired_pos[5]  # Joint 6
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
        
        j = 5  # Joint 6 (index 5)
        # Use torque around Z-axis for joint 6
        fz_force = forces[5]

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
                passive_prev_pos = actual_jpos[5] if err_pos==0 else desired_pos[j]
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
                    actual_pos_j = actual_jpos[5] if err_pos==0 else desired_pos[j]
                    eps = 1e-3
                    if actual_pos_j < passive_prev_pos - eps:
                        passive_direction = "up"
                    elif actual_pos_j > passive_prev_pos + eps:
                        passive_direction = "down"
                    else:
                        passive_direction = "down" if rep_state!="moving_up" else "up"

                if elapsed >= 0.2 and passive_direction:
                    # --- Passive move UP ---
                    if passive_direction == "up":
                        while abs(desired_pos[j]-UPPER_LIMIT) > 0.5:
                            assist_force = 10.0
                            acc = (assist_force - B[j]*velocity[j])/M[j]
                            velocity[j] += acc*dt
                            delta = velocity[j]*dt*force_to_deg
                            potential_pos = desired_pos[j] - abs(delta)
                            if potential_pos < UPPER_LIMIT:
                                potential_pos = UPPER_LIMIT
                                velocity[j] = 0.0
                            desired_pos[j] = potential_pos
                            robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
                            time.sleep(dt)

                    # --- Passive move DOWN ---
                    elif passive_direction == "down":
                        while abs(desired_pos[j]-LOWER_LIMIT) > 0.5:
                            assist_force = -10.0
                            acc = (assist_force - B[j]*velocity[j])/M[j]
                            velocity[j] += acc*dt
                            delta = velocity[j]*dt*force_to_deg
                            potential_pos = desired_pos[j] + abs(delta)
                            if potential_pos > LOWER_LIMIT:
                                potential_pos = LOWER_LIMIT
                                velocity[j] = 0.0
                            desired_pos[j] = potential_pos
                            robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
                            time.sleep(dt)

                    # ✅ Cooldown after passive rep
                    time.sleep(PASSIVE_COOLDOWN)

                    # --- FIX BOUNCE BACK ---
                    velocity[j] = 0.0
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
            mode_state = "guided"
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
        if potential_pos < UPPER_LIMIT: potential_pos = UPPER_LIMIT
        elif potential_pos > LOWER_LIMIT: potential_pos = LOWER_LIMIT
        desired_pos[j] = potential_pos

        count_repetitions(desired_pos[j])
        previous_pos = desired_pos[j]

        # --- LOCK ALL JOINTS EXCEPT JOINT 6 ---
        for lock_j in range(6):
            if lock_j != 5:  # Lock all joints except joint 6 (index 5)
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0

        safety_ok = True
        for idx in range(6):
            joint_num = idx + 1
            if not is_within_safety_limits(joint_num, desired_pos[idx]):
                safety_ok = False
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