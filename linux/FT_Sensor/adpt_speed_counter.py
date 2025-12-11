# Done 
# this code will give active assistance and the motion is restricted with a counter for reps so all reps must be completed 
# this code counts reps stops if your prev is 140 and goint to 70 can't change the path in between 

import Robot
import time
import signal
import sys
import numpy as np

# --- PARAMETERS PER JOINT ---
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]     # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Joint 2 limits
JOINT2_MIN_LIMIT = 70.0    # degrees 
JOINT2_MAX_LIMIT = 144.0   # degrees

# --- SPEED-BASED ASSISTANCE ---
AVG_SPEED = 7.70  # deg/sec - target speed to maintain
TARGET_SPEED = AVG_SPEED  # Target speed to maintain

# --- REPETITION COUNTING ---
rep_count = 0
movement_threshold = 2.0  # degrees - tolerance near limits

# --- STATE MACHINE ---
rep_state = None  # will be initialized after home position

# --- SAFETY LIMITS FOR EACH JOINT (degrees) ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),
    2: (-179.0, -35.0),
    3: (2.0, 144.0),
    4: (-258.0, 80.0),
    5: (-170.0, 12.0),
    6: (-170.0, 170.0),
}

motion_paused = False

# Control parameters
force_to_deg = 8
dt = 0.008

# Gravity compensation
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 1.0

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# --- FUNCTIONS ---
def check_joint2_limits(target_position):
    return JOINT2_MIN_LIMIT <= target_position <= JOINT2_MAX_LIMIT

def count_repetitions(current_pos):
    """
    Full rep = MAX → MIN → MAX (or reverse)
    Uses a state machine to prevent bouncing.
    """
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

# initialize state machine
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

def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces, rep_count, rep_state
    previous_pos = desired_pos[2]
    
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt); continue

        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
        for i in range(6):
            if abs(forces[i]) < 0.5: forces[i] = 0.0

        if mode == "sitting":
            pass  # unchanged sitting mode
        else:
            j = 2
            fz_force = forces[2]

            # Get current speed
            current_velocity = velocity[j] * force_to_deg
            current_speed = abs(current_velocity)
            movement_direction = "up" if current_velocity > 0.1 else ("down" if current_velocity < -0.1 else "stopped")

            # assistance calculation
            assisted_force = fz_force
            if abs(fz_force) > 0.0 and current_speed < TARGET_SPEED:
                speed_deficit = TARGET_SPEED - current_speed
                assistance_factor = speed_deficit / TARGET_SPEED
                if movement_direction == "up":
                    assistance_force = (abs(fz_force) if fz_force > 0 else 2.0) * assistance_factor
                    assisted_force = fz_force + assistance_force
                elif movement_direction == "down":
                    assistance_force = (abs(fz_force) if fz_force < 0 else 2.0) * assistance_factor
                    assisted_force = fz_force - assistance_force
                else:
                    assisted_force = fz_force * (1+assistance_factor)
            elif abs(fz_force) > 0.0 and current_speed >= TARGET_SPEED:
                pass  # already at speed

            # --- enforce direction based on state machine ---
            if rep_state == "moving_down" and movement_direction == "up":
                assisted_force = min(0.0, assisted_force)
                print("⛔ Ignoring UP movement, must go DOWN to MIN first")
            elif rep_state == "moving_up" and movement_direction == "down":
                assisted_force = max(0.0, assisted_force)
                print("⛔ Ignoring DOWN movement, must go UP to MAX first")

            # motion update
            if abs(assisted_force) < 1.0 and current_speed < 0.5:
                home_pos[j] = desired_pos[j]
                spring_force = -K[j]*(desired_pos[j]-home_pos[j])/force_to_deg
                acc = (spring_force - B[j]*velocity[j]) / M[j]
            else:
                acc = (assisted_force - B[j]*velocity[j]) / M[j]

            velocity[j] += acc*dt
            delta = velocity[j]*dt*force_to_deg
            potential_pos = desired_pos[j] + delta

            # limits
            if potential_pos < JOINT2_MIN_LIMIT:
                potential_pos = JOINT2_MIN_LIMIT; velocity[j]=0.0
            elif potential_pos > JOINT2_MAX_LIMIT:
                potential_pos = JOINT2_MAX_LIMIT; velocity[j]=0.0
            desired_pos[j] = potential_pos

            # repetition counter
            count_repetitions(desired_pos[j])
            previous_pos = desired_pos[j]

            print(f"[STANDING] Reps: {rep_count} | State: {rep_state} | Dir: {movement_direction} | "
                  f"Speed: {current_speed:.2f}°/s | Pos: {desired_pos[j]:.1f}° | Force: {fz_force:.2f}N")

        # lock other joints
        for lock_j in range(6):
            if (mode=="sitting" and lock_j not in [1,2,3]) or (mode=="standing" and lock_j!=2):
                desired_pos[lock_j]=home_pos[lock_j]; velocity[lock_j]=0.0

        # safety checks
        safety_ok=True
        for j in range(6):
            joint_num=j+1
            if not is_within_safety_limits(joint_num,desired_pos[j]): safety_ok=False
        if not safety_ok: time.sleep(dt); continue

        robot.ServoJ(joint_pos=desired_pos,axisPos=[0]*6)
        time.sleep(dt)

control_loop()
