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

# --- NEW SPEED-BASED ASSISTANCE ---
AVG_SPEED = 7.70  # deg/sec
ASSISTANCE_THRESHOLD = 0.2 * AVG_SPEED  # 20% of AVG_SPEED

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
force_to_deg = 2
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
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt); continue

        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2], ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i]-baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces
        for i in range(6):
            if abs(forces[i]) < 0.5: forces[i] = 0.0

        if mode == "sitting":
            pass # (unchanged sitting mode logic)
        else:
            j = 2  # controlling joint 2
            fz_force = forces[2]

            # assistance condition based on joint speed
            joint_speed = velocity[j] * force_to_deg  # deg/s
            assisted_force = fz_force

            if abs(joint_speed) <= ASSISTANCE_THRESHOLD:
                assisted_force = fz_force + 0.4 * fz_force
                print(f"★ Assistance Triggered:\n" 
                      f" Raw={fz_force:.2f}N\n" 
                      f" Modified={assisted_force:.2f}N\n" 
                      f" Speed={joint_speed:.2f}°/s\n" 
                      f"(Threshold={ASSISTANCE_THRESHOLD:.2f})")
            else:
                print(f"No Assistance: Raw={fz_force:.2f}N\n" 
                      f" Speed={joint_speed:.2f}°/s\n" 
                      f" (Threshold={ASSISTANCE_THRESHOLD:.2f})")

            if abs(fz_force) < 1.0:
                home_pos[j] = desired_pos[j]
                spring_force = -K[j]*(desired_pos[j]-home_pos[j])/force_to_deg
                acc = (spring_force - B[j]*velocity[j]) / M[j]
            else:
                acc = (assisted_force - B[j]*velocity[j]) / M[j]

            velocity[j] += acc*dt
            delta = velocity[j]*dt*force_to_deg
            potential_pos = desired_pos[j] + delta
            if potential_pos < JOINT2_MIN_LIMIT:
                potential_pos = JOINT2_MIN_LIMIT; velocity[j]=0.0
            elif potential_pos > JOINT2_MAX_LIMIT:
                potential_pos = JOINT2_MAX_LIMIT; velocity[j]=0.0
            desired_pos[j] = potential_pos

            # Debug print
            print(
                f"[STANDING] RawFz={fz_force:.2f} N | ModifiedFz={assisted_force:.2f} N | "
                f"Joint2 Speed={joint_speed:.2f} °/s | Velocity={velocity[j]:.4f}"
            )

        # Lock other joints
        for lock_j in range(6):
            if (mode=="sitting" and lock_j not in [1,2,3]) or (mode=="standing" and lock_j!=2):
                desired_pos[lock_j]=home_pos[lock_j]; velocity[lock_j]=0.0

        # Safety checks
        safety_ok=True
        for j in range(6):
            joint_num=j+1
            if not is_within_safety_limits(joint_num,desired_pos[j]): safety_ok=False
        if not safety_ok: time.sleep(dt); continue

        err=robot.ServoJ(joint_pos=desired_pos,axisPos=[0]*6)
        time.sleep(dt)

control_loop()
