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
ASSISTANCE_ANGLE = 122.0   # degrees - robot will auto-continue from this point

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

# Extended motion control variables
extended_motion_active = False
extended_motion_phase = "none"
extended_motion_completed = False
trajectory_positions = []
current_trajectory_index = 0

# Angle direction monitoring
previous_angle = None
angle_history = []
ANGLE_HISTORY_SIZE = 5
approaching_from_above = False

# Trajectory control
TRAJECTORY_STEP_SIZE = 0.5
descent_velocity = 8.0
ascent_velocity = 12.0

# Control parameters
force_threshold_for_motion = 1.0
assistance_reached_threshold = 1.0
force_to_deg = 2
dt = 0.008

# Gravity compensation
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 1.0

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# --- FUNCTIONS (angle tracking, trajectory, motion, etc.) ---
def update_angle_history(current_angle):
    global angle_history, approaching_from_above
    angle_history.append(current_angle)
    if len(angle_history) > ANGLE_HISTORY_SIZE:
        angle_history.pop(0)
    if len(angle_history) >= 3:
        recent_angles = angle_history[-3:]
        descending_trend = all(recent_angles[i] >= recent_angles[i+1] for i in range(len(recent_angles)-1))
        if descending_trend and recent_angles[0] > ASSISTANCE_ANGLE:
            approaching_from_above = True
        elif recent_angles[0] < ASSISTANCE_ANGLE:
            approaching_from_above = False

def check_assistance_angle_trigger(current_angle):
    global approaching_from_above, extended_motion_active, extended_motion_completed
    at_assistance_angle = abs(current_angle - ASSISTANCE_ANGLE) <= assistance_reached_threshold
    if (not extended_motion_active and not extended_motion_completed and at_assistance_angle):
        if approaching_from_above:
            print(f"★ REACHED ASSISTANCE ANGLE {ASSISTANCE_ANGLE}° FROM ABOVE!")
            print("★ AUTO-TRIGGERING extended motion...")
            return True
        else:
            print(f"★ REACHED ASSISTANCE ANGLE {ASSISTANCE_ANGLE}° FROM BELOW!")
            print(f"★ Robot will remain at {ASSISTANCE_ANGLE}° until new force is applied")
            return False
    return False

def check_joint2_limits(target_position):
    return JOINT2_MIN_LIMIT <= target_position <= JOINT2_MAX_LIMIT

def generate_descent_trajectory(start_pos, end_pos):
    trajectory = []
    current = start_pos
    direction = -1 if end_pos < start_pos else 1
    while abs(current - end_pos) > TRAJECTORY_STEP_SIZE:
        trajectory.append(current)
        current += direction * TRAJECTORY_STEP_SIZE
    trajectory.append(end_pos)
    return trajectory

def start_extended_motion(current_pos):
    global extended_motion_active, extended_motion_phase, trajectory_positions, current_trajectory_index
    extended_motion_active = True
    extended_motion_phase = "auto_descending"
    trajectory_positions = generate_descent_trajectory(ASSISTANCE_ANGLE, JOINT2_MIN_LIMIT)
    current_trajectory_index = 0
    print("★ AUTO-EXTENDED MOTION STARTED!")

def handle_extended_motion():
    global extended_motion_active, extended_motion_phase, desired_pos, velocity
    global trajectory_positions, current_trajectory_index, extended_motion_completed
    global angle_history, approaching_from_above
    if extended_motion_phase == "auto_descending":
        if current_trajectory_index < len(trajectory_positions):
            target_pos = trajectory_positions[current_trajectory_index]
            current_pos = desired_pos[2]
            distance_to_target = target_pos - current_pos
            if abs(distance_to_target) > 0.1:
                move_step = -descent_velocity * dt
                if abs(move_step) > abs(distance_to_target):
                    move_step = distance_to_target
                desired_pos[2] = current_pos + move_step
                velocity[2] = move_step / dt
            else:
                desired_pos[2] = target_pos
                current_trajectory_index += 1
                if current_trajectory_index >= len(trajectory_positions):
                    extended_motion_phase = "ascending"
                    print(f"★ Reached minimum limit {JOINT2_MIN_LIMIT}°")
        return
    elif extended_motion_phase == "ascending":
        current_pos = desired_pos[2]
        target_pos = ASSISTANCE_ANGLE
        distance_to_target = target_pos - current_pos
        if abs(distance_to_target) > 0.5:
            move_step = ascent_velocity * dt
            if move_step > distance_to_target:
                move_step = distance_to_target
            desired_pos[2] = current_pos + move_step
            velocity[2] = move_step / dt
        else:
            desired_pos[2] = target_pos
            velocity[2] = 0.0
            extended_motion_phase = "complete"
            print(f"★ RETURNED to assistance angle: {target_pos:.1f}°")
        return
    elif extended_motion_phase == "complete":
        extended_motion_active = False
        extended_motion_phase = "none"
        extended_motion_completed = True
        trajectory_positions = []
        current_trajectory_index = 0
        velocity[2] = 0.0
        angle_history = []
        approaching_from_above = False
        print("★ EXTENDED MOTION COMPLETE!")

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
if ASSISTANCE_ANGLE <= JOINT2_MIN_LIMIT or ASSISTANCE_ANGLE >= JOINT2_MAX_LIMIT: sys.exit(1)
calibrate_baseline_forces()
previous_angle = joint_pos[2]
angle_history = [joint_pos[2]]
home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0]*6

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
    global desired_pos, velocity, home_pos, baseline_forces
    global extended_motion_active, extended_motion_completed, previous_angle
    global approaching_from_above, angle_history
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt); continue
        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
        for i in range(6):
            if abs(forces[i]) < 0.5: forces[i] = 0.0
        current_joint2_pos = desired_pos[2]
        update_angle_history(current_joint2_pos)
        if check_assistance_angle_trigger(current_joint2_pos): start_extended_motion(current_joint2_pos)
        if (extended_motion_completed and current_joint2_pos > ASSISTANCE_ANGLE+5.0):
            extended_motion_completed = False
            angle_history = [current_joint2_pos]
            approaching_from_above = False
        if extended_motion_active:
            handle_extended_motion()
        else:
            if mode == "sitting":
                pass # (unchanged sitting code)
            else:
                j = 2
                fz_force = forces[2]
                if abs(fz_force) < 1.0:
                    home_pos[j] = desired_pos[j]
                    spring_force = -K[j]*(desired_pos[j]-home_pos[j])/force_to_deg
                    acc = (spring_force - B[j]*velocity[j]) / M[j]
                else:
                    acc = (fz_force - B[j]*velocity[j]) / M[j]
                velocity[j] += acc*dt
                delta = velocity[j]*dt*force_to_deg
                potential_pos = desired_pos[j] + delta
                if potential_pos < JOINT2_MIN_LIMIT:
                    potential_pos = JOINT2_MIN_LIMIT; velocity[j]=0.0
                elif potential_pos > JOINT2_MAX_LIMIT:
                    potential_pos = JOINT2_MAX_LIMIT; velocity[j]=0.0
                desired_pos[j] = potential_pos
                # ★ DEBUG PRINTS FOR STANDING MODE ★
                joint_speed = delta/dt   # deg/s (signed)
                abs_joint_speed = abs(joint_speed)
                abs_velocity = abs(velocity[j])
                print(
                    f"[STANDING] Fz={fz_force:.2f} N | "
                    f"Joint2 Speed={joint_speed:.2f} °/s (|{abs_joint_speed:.2f}|) | "
                    f"Admittance Velocity={velocity[j]:.4f} (|{abs_velocity:.4f}|)"
                )
        for lock_j in range(6):
            if (mode=="sitting" and lock_j not in [1,2,3]) or (mode=="standing" and lock_j!=2):
                desired_pos[lock_j]=home_pos[lock_j]; velocity[lock_j]=0.0
        previous_angle=current_joint2_pos
        safety_ok=True
        for j in range(6):
            joint_num=j+1
            if not is_within_safety_limits(joint_num,desired_pos[j]): safety_ok=False
        if not safety_ok: time.sleep(dt); continue
        err=robot.ServoJ(joint_pos=desired_pos,axisPos=[0]*6)
        time.sleep(dt)

control_loop()
