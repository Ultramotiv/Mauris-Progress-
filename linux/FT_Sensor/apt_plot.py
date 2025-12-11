# adpt_plot1.py
import Robot
import time
import signal
import sys
import numpy as np
import threading
import queue
import matplotlib.pyplot as plt
from collections import deque

# =======================
# --- PARAMETERS ---
# =======================
M = [5.0, 5.0, 4.0, 5.0, 5.0, 5.0]
B = [10.0, 10.0, 5.0, 5.0, 5.0, 5.0]
K = [10.0, 10.0, 5.0, 10.0, 10.0, 10.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

force_to_deg = 5            # degrees per Newton
dt = 0.008                  # 125 Hz control
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 2.0

# Motion / velocity tracking
motion_started = False
motion_start_time = None
motion_start_position = None
force_display_counter = 0
force_display_interval = 12
MOTION_THRESHOLD = 1.0

previous_positions = [0.0] * 6
joint_velocities = [0.0] * 6
velocity_filter_alpha = 0.2

total_distance_moved = [0.0] * 6
max_velocities = [0.0] * 6
motion_duration = 0.0

# Safety limits (degrees)
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),
    2: (-179.0, -35.0),
    3: (83.0, 144.0),
    4: (-258.0, 80.0),
    5: (-170.0, 12.0),
    6: (-170.0, 170.0),
}

# =======================
# --- ROBOT CONNECTION ---
# =======================
print("SDK连接机器人")
robot = Robot.RPC('192.168.58.2')
print(robot)
print("Robot connected.")

# =======================
# --- GLOBAL STATE ---
# =======================
mode = None
free_joints = []

home_pos = None
desired_pos = None
velocity = None

# Plotting control
plot_running = True
data_queue = queue.Queue(maxsize=2000)  # bounded queue, non-blocking put

# Ring buffers for smooth plotting (last N points)
plot_window = 500
time_buffer = deque(maxlen=plot_window)
force_buffer = deque(maxlen=plot_window)
distance_buffer = deque(maxlen=plot_window)
velocity_buffer = deque(maxlen=plot_window)
speed_buffer = deque(maxlen=plot_window)
avg_velocity_buffer = deque(maxlen=plot_window)
avg_speed_buffer = deque(maxlen=plot_window)

joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]

# =======================
# --- FUNCTIONS ---
# =======================
def detect_motion_start(forces):
    global motion_started, motion_start_time, motion_start_position
    force_magnitude = np.sqrt(sum(f**2 for f in forces[:3]))  # linear forces
    if not motion_started and force_magnitude > MOTION_THRESHOLD:
        motion_started = True
        motion_start_time = time.time()
        motion_start_position = desired_pos.copy()
        print(f"\n🚀 MOTION STARTED! Force magnitude: {force_magnitude:.3f} N")
        print(f"   Start time: {time.strftime('%H:%M:%S')}")
        print(f"   Start position: {[f'{p:.2f}°' for p in motion_start_position]}")
        print("="*80)

def calculate_joint_velocities(current_positions):
    global previous_positions, joint_velocities, velocity_filter_alpha, max_velocities
    for i in range(6):
        raw_velocity = (current_positions[i] - previous_positions[i]) / dt
        joint_velocities[i] = (velocity_filter_alpha * raw_velocity +
                               (1 - velocity_filter_alpha) * joint_velocities[i])
        if abs(joint_velocities[i]) > max_velocities[i]:
            max_velocities[i] = abs(joint_velocities[i])
        previous_positions[i] = current_positions[i]

def update_motion_statistics():
    global total_distance_moved, motion_duration
    if motion_started and motion_start_position is not None:
        for i in range(6):
            total_distance_moved[i] = abs(desired_pos[i] - motion_start_position[i])
        motion_duration = time.time() - motion_start_time

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

def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating baseline forces...")
    force_samples = []
    for _ in range(gravity_compensation_samples):
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] == 0:  # ok
            forces = [
                ft_data[1][0],
                -ft_data[1][1],
                ft_data[1][2],
                ft_data[1][3],
                ft_data[1][4],
                ft_data[1][5]
            ]
            force_samples.append(forces)
        time.sleep(0.01)
    if force_samples:
        baseline_forces = [sum(s[i] for s in force_samples)/len(force_samples) for i in range(6)]
        print(f"Baseline forces: {baseline_forces}")
    else:
        print("Warning: Could not capture baseline forces!")
        baseline_forces = [0.0]*6

def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        mn, mx = JOINT_SAFETY_LIMITS[joint_idx]
        return mn <= angle <= mx
    return True

def shutdown(sig, frame):
    global plot_running
    try:
        robot.ServoMoveEnd()
    except Exception:
        pass
    if motion_started:
        print("\n" + "="*80)
        print("                    FINAL MOTION SUMMARY")
        print("="*80)
        update_motion_statistics()
        total_dist = np.sqrt(sum(d**2 for d in total_distance_moved))
        print(f"Total Motion Duration:    {motion_duration:.2f} s")
        print(f"Total Distance Moved:     {total_dist:.2f}°")
        print(f"Maximum Speed Achieved:   {max(max_velocities):.2f}°/s")
        if motion_duration > 0:
            avg_speed = total_dist / motion_duration
            print(f"Average Speed:            {avg_speed:.2f}°/s")
        print("="*80)
    plot_running = False
    print("\nServo stopped. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# =======================
# --- SETUP ---
# =======================
init_ft_sensor()

err, joint_pos = robot.GetActualJointPosDegree()
if err != 0:
    print("Error getting joint positions. Exiting.")
    sys.exit(1)

ok = True
for j in range(6):
    if not is_within_safety_limits(j+1, joint_pos[j]):
        print(f"Initial joint angle out of safety range for Joint {j+1}: "
              f"{joint_pos[j]:.2f}° (Range: {JOINT_SAFETY_LIMITS[j+1][0]}° to {JOINT_SAFETY_LIMITS[j+1][1]}°)")
        ok = False
if not ok:
    print("Move robot to safe position and restart.")
    sys.exit(1)

calibrate_baseline_forces()

previous_positions = joint_pos.copy()
joint_velocities = [0.0] * 6

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

print("Initial joint angles:")
for i, angle in enumerate(joint_pos):
    print(f"  {joint_names[i]}: {angle:.2f}°")

while mode not in ["sitting", "standing"]:
    mode = input("Select mode (sitting/standing): ").strip().lower()
    if mode not in ["sitting", "standing"]:
        print("Invalid input. Please enter 'sitting' or 'standing'.")

if mode == "sitting":
    print("Mode: Sitting - Joints 1, 2, 3 are free, rest fixed.")
    free_joints = [1, 2, 3]
else:
    print("Mode: Standing - Only Joint 2 is free, rest fixed.")
    free_joints = [2]

if robot.ServoMoveStart() != 0:
    print("Failed to start servo mode.")
    sys.exit(1)

print(f"Waiting {startup_delay} seconds before enabling control...")
time.sleep(startup_delay)

print("★ ENHANCED SHOULDER FLEXION CONTROL WITH COMPREHENSIVE MONITORING ★")
print(f"Mode: {mode.capitalize()}")
print(f"Motion tracking starts when force > {MOTION_THRESHOLD} N")
print(f"Force/Velocity display rate: Every {force_display_interval * dt * 1000:.0f}ms")
print("Press Ctrl+C to stop and see final motion summary.")
print("="*80)

# =======================
# --- CONTROL LOOP (THREAD) ---
# =======================
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces, plot_running

    next_cycle = time.monotonic()
    while plot_running:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            next_cycle += dt
            sleep_time = next_cycle - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            continue

        raw_forces = [
            ft_data[1][0],
            -ft_data[1][1],
            ft_data[1][2],
            ft_data[1][3],
            ft_data[1][4],
            ft_data[1][5]
        ]

        if baseline_forces is not None:
            forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            forces = raw_forces

        deadband = 0.5
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0

        detect_motion_start(forces)
        calculate_joint_velocities(desired_pos)

        if mode == "sitting":
            # Sitting mode control...
            # J1 reacts to Fz
            j1 = 1
            fz_force = forces[2]
            if abs(fz_force) < 1.0:
                home_pos[j1] = desired_pos[j1]
                spring_force = -K[j1] * (desired_pos[j1] - home_pos[j1]) / force_to_deg
                acc = (spring_force - B[j1] * velocity[j1]) / M[j1]
            else:
                acc = (fz_force - B[j1] * velocity[j1]) / M[j1]
            velocity[j1] += acc * dt
            desired_pos[j1] += velocity[j1] * dt * force_to_deg

            # J2 reacts to Fx
            j2 = 2
            fx_force = forces[0]
            if abs(fx_force) < 1.0:
                home_pos[j2] = desired_pos[j2]
                spring_force = -K[j2] * (desired_pos[j2] - home_pos[j2]) / force_to_deg
                acc = (spring_force - B[j2] * velocity[j2]) / M[j2]
            else:
                acc = ((-fx_force) - B[j2] * velocity[j2]) / M[j2]
            velocity[j2] += acc * dt
            desired_pos[j2] += velocity[j2] * dt * force_to_deg

            # J3 reacts to Fx
            j3 = 3
            fx_force = forces[0]
            if abs(fx_force) < 1.0:
                home_pos[j3] = desired_pos[j3]
                spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
                acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
            else:
                acc = ((-fx_force) - B[j3] * velocity[j3]) / M[j3]
            velocity[j3] += acc * dt
            desired_pos[j3] += velocity[j3] * dt * force_to_deg

        else:
            # Standing mode control...
            j = 2
            fz_force = forces[2]
            if abs(fz_force) < 1.0:
                home_pos[j] = desired_pos[j]
                spring_force = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
                acc = (spring_force - B[j] * velocity[j]) / M[j]
            else:
                acc = (fz_force - B[j] * velocity[j]) / M[j]
            velocity[j] += acc * dt
            desired_pos[j] += velocity[j] * dt * force_to_deg

        for lock_j in range(6):
            if lock_j not in free_joints:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0

        safe = True
        for j in range(6):
            if not is_within_safety_limits(j+1, desired_pos[j]):
                print(f"⚠️  SAFETY VIOLATION: Joint {j+1}: {desired_pos[j]:.2f}° "
                      f"(Range: {JOINT_SAFETY_LIMITS[j+1][0]}° to {JOINT_SAFETY_LIMITS[j+1][1]}°)")
                safe = False
        if not safe:
            next_cycle += dt
            sleep_time = next_cycle - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            continue

        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)

        if mode == "standing" and motion_started:
            update_motion_statistics()
            current_time = time.time() - motion_start_time
            distance = total_distance_moved[2]
            vel = joint_velocities[2]
            spd = abs(vel)
            avg_v = (distance / motion_duration) if motion_duration > 0 else 0.0
            total_distance = np.sqrt(sum(d**2 for d in total_distance_moved))
            avg_s = (total_distance / motion_duration) if motion_duration > 0 else 0.0
            F_plot = forces[2]
            try:
                data_queue.put_nowait((current_time, F_plot, distance, vel, spd, avg_v, avg_s))
            except queue.Full:
                pass

        next_cycle += dt
        sleep_time = next_cycle - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)

# =======================
# --- PLOTTING (MAIN THREAD) ---
# =======================
def plotting_loop_main_thread():
    plt.ion()
    fig, axs = plt.subplots(3, 2, figsize=(12, 8))
    fig.suptitle("Real-Time Free Joint Data (Standing Mode)")
    ax_force, ax_distance, ax_velocity, ax_speed, ax_avg_v, ax_avg_s = axs.flatten()

    # Initialize line objects
    ln_force, = ax_force.plot([], [], label="Force (N)")
    ln_distance, = ax_distance.plot([], [], label="Distance (°)")
    ln_velocity, = ax_velocity.plot([], [], label="Velocity (°/s)")
    ln_speed, = ax_speed.plot([], [], label="Speed (°/s)")
    ln_avg_v, = ax_avg_v.plot([], [], label="Avg Velocity (°/s)")
    ln_avg_s, = ax_avg_s.plot([], [], label="Avg Speed (°/s)")

    for ax in axs.flatten():
        ax.grid(True)
        ax.legend()

    last_draw = time.time()
    while plot_running:
        updated = False
        try:
            while True:
                (t, f, d, v, s, avgv, avgs) = data_queue.get_nowait()
                time_buffer.append(t)
                force_buffer.append(f)
                distance_buffer.append(d)
                velocity_buffer.append(v)
                speed_buffer.append(s)
                avg_velocity_buffer.append(avgv)
                avg_speed_buffer.append(avgs)
                updated = True
        except queue.Empty:
            pass

        if updated and (time.time() - last_draw) > 0.05:
            ln_force.set_data(time_buffer, force_buffer)
            ln_distance.set_data(time_buffer, distance_buffer)
            ln_velocity.set_data(time_buffer, velocity_buffer)
            ln_speed.set_data(time_buffer, speed_buffer)
            ln_avg_v.set_data(time_buffer, avg_velocity_buffer)
            ln_avg_s.set_data(time_buffer, avg_speed_buffer)

            for ax in [ax_force, ax_distance, ax_velocity, ax_speed, ax_avg_v, ax_avg_s]:
                ax.relim()
                ax.autoscale_view()

            plt.pause(0.001)
            last_draw = time.time()

        time.sleep(0.001)

    plt.ioff()
    plt.show()

# =======================
# --- START THREADS ---
# =======================
ctrl_thread = threading.Thread(target=control_loop, daemon=True)
ctrl_thread.start()

plotting_loop_main_thread()
