import Robot
import time
import signal
import sys
import threading

# --- PARAMETERS PER JOINT (ALL JOINTS NOW ACTIVE) ---
M = [10.0, 10.0, 10.0, 20.0, 20.0, 15.0]  # Virtual mass per joint
B = [5.0, 4.0, 5.0, 10.0, 10.0, 15.0]     # Damping per joint
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Spring stiffness per joint
#K = [30.0, 20.0, 30.0, 30.0, 30.0, 25.0]  # Spring stiffness per joint
force_thresholds = [2.5, 2.5, 30.0, 1.2, 1.2, 1.2]  # Force thresholds (added for Joint 5)

# Scaling: degrees per Newton
force_to_deg = 10

# Control loop timing
dt = 0.008

# Force axis to joint mapping (0: Fx, 1: Fy, 2: Fz, 3: Mx, 4: My, 5: Mz)
force_joint_map = {
    0: 1,  # Fx controls Joint 1 (inverted direction if needed)
    1: 0,  # Fy controls Joint 0
    2: 2,  # Fz controls Joint 2
    3: 5,  # Mx controls Joint 5  ← NEW: Joint 5 now active
    4: 3,  # My controls Joint 3
    5: 4   # Mz controls Joint 4
}

# --- CONNECT TO ROBOT ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

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

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nServo stopped. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# --- SETUP ---
init_ft_sensor()

error, joint_pos = robot.GetActualJointPosDegree()
if error != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

# --- START SERVO MODE ---
if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print("Admittance control started. All joints active. Press Ctrl+C to stop.")

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity
    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            print(f"FT read error: {ft_data[0]}")
            time.sleep(dt)
            continue

        forces = [
            ft_data[1][0],   # Fx (inverted)
            -ft_data[1][1],  # Fy
            ft_data[1][2],   # Fz
            ft_data[1][3],   # Mx
            ft_data[1][4],   # My
            ft_data[1][5]    # Mz
        ]

        def update_joint(joint, axis_force):
            threshold = force_thresholds[joint]
            if abs(axis_force) < threshold:
                spring_force = -K[joint] * (desired_pos[joint] - home_pos[joint]) / force_to_deg
                acc = (spring_force - B[joint] * velocity[joint]) / M[joint]
            else:
                acc = (axis_force - B[joint] * velocity[joint]) / M[joint]

            velocity[joint] += acc * dt
            delta = velocity[joint] * dt * force_to_deg
            desired_pos[joint] += delta

        threads = []
        for axis, joint in force_joint_map.items():
            # Removed the "if joint == 5: continue" check - now all joints are active
            t = threading.Thread(target=update_joint, args=(joint, forces[axis]))
            threads.append(t)
            t.start()

        for t in threads:
            t.join()

        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
        time.sleep(dt)

control_loop()