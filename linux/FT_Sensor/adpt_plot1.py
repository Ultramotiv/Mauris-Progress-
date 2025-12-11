# adpt_plot1_fixed.py
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

# FIXED: Proper distance tracking
cumulative_distance = [0.0] * 6  # Total distance traveled (always increases)
total_displacement = [0.0] * 6   # Net displacement from start
max_velocities = [0.0] * 6
motion_duration = 0.0

# Safety limits (degrees)
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),
    2: (-179.0, -35.0),
    3: (80.0, 144.0),
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
data_queue = queue.Queue(maxsize=10000)

# Full trajectory storage
time_data = []
force_data = []
cumulative_distance_data = []
displacement_data = []
velocity_data = []
speed_data = []
avg_velocity_data = []
avg_speed_data = []

# Trajectory tracking
trajectory_start_time = None
trajectory_start_position = None
trajectory_active = False

joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]

# =======================
# --- FUNCTIONS ---
# =======================
def detect_motion_start(forces):
    global motion_started, motion_start_time, motion_start_position, trajectory_start_time, trajectory_start_position, trajectory_active
    force_magnitude = np.sqrt(sum(f**2 for f in forces[:3]))
    
    if not motion_started and force_magnitude > MOTION_THRESHOLD:
        motion_started = True
        motion_start_time = time.time()
        motion_start_position = desired_pos.copy()
        print(f"\n🚀 MOTION STARTED! Force magnitude: {force_magnitude:.3f} N")
        print(f"   Start time: {time.strftime('%H:%M:%S')}")
        print(f"   Start position: {[f'{p:.2f}°' for p in motion_start_position]}")
        print("="*80)
    
    if not trajectory_active:
        trajectory_active = True
        trajectory_start_time = time.time()
        trajectory_start_position = desired_pos.copy()

def calculate_joint_velocities(current_positions):
    global previous_positions, joint_velocities, velocity_filter_alpha, max_velocities
    global cumulative_distance, total_displacement
    
    for i in range(6):
        # FIXED: Calculate raw velocity properly
        raw_velocity = (current_positions[i] - previous_positions[i]) / dt
        
        # FIXED: Apply less aggressive filtering for more accurate instantaneous values
        joint_velocities[i] = (0.7 * raw_velocity + 0.3 * joint_velocities[i])
        
        # Track maximum velocity
        if abs(joint_velocities[i]) > max_velocities[i]:
            max_velocities[i] = abs(joint_velocities[i])
        
        # FIXED: Calculate cumulative distance correctly (always increases)
        if trajectory_active and trajectory_start_position is not None:
            distance_increment = abs(current_positions[i] - previous_positions[i])
            cumulative_distance[i] += distance_increment
            
            # Calculate net displacement from start
            total_displacement[i] = current_positions[i] - trajectory_start_position[i]
        
        previous_positions[i] = current_positions[i]

def update_motion_statistics():
    global motion_duration
    if trajectory_active and trajectory_start_time is not None:
        motion_duration = time.time() - trajectory_start_time

def calculate_trajectory_averages(total_time, cumulative_dist, net_displacement):
    """FIXED: Calculate average velocity and speed correctly"""
    if total_time > 0:
        # Average velocity = net displacement / total time (can be negative)
        avg_velocity = net_displacement / total_time
        
        # Average speed = total distance traveled / total time (always positive)
        avg_speed = cumulative_dist / total_time
        
        return avg_velocity, avg_speed
    return 0.0, 0.0

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
        if ft_data[0] == 0:
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
    global plot_running, trajectory_active
    try:
        robot.ServoMoveEnd()
    except Exception:
        pass
    
    if trajectory_active:
        print("\n" + "="*80)
        print("                    FINAL TRAJECTORY SUMMARY")
        print("="*80)
        update_motion_statistics()
        
        if mode == "standing":
            joint_idx = 2
            total_dist_traveled = cumulative_distance[joint_idx]
            net_displacement = total_displacement[joint_idx]
            max_vel = max_velocities[joint_idx]
            
            print(f"Joint 2 (Primary) Analysis:")
            print(f"  Total Motion Duration:        {motion_duration:.2f} s")
            print(f"  Total Distance Traveled:      {total_dist_traveled:.2f}° (cumulative)")
            print(f"  Net Displacement from Start:  {net_displacement:.2f}°")
            print(f"  Maximum Speed Achieved:       {max_vel:.2f}°/s")
            
            if motion_duration > 0:
                # FIXED: Use motion_duration for final calculations
                avg_velocity_final = net_displacement / motion_duration
                avg_speed_final = total_dist_traveled / motion_duration
                print(f"  Average Velocity (net):       {avg_velocity_final:.2f}°/s")
                print(f"  Average Speed (total):        {avg_speed_final:.2f}°/s")
            
            total_cumulative_all = np.sqrt(sum(d**2 for d in cumulative_distance))
            total_displacement_all = np.sqrt(sum(d**2 for d in total_displacement))
            print(f"\nOverall Trajectory:")
            print(f"  Total 3D Distance Traveled:   {total_cumulative_all:.2f}°")
            print(f"  Total 3D Net Displacement:    {total_displacement_all:.2f}°")
            print(f"  Maximum Joint Speed:          {max(max_velocities):.2f}°/s")
        
        elif mode == "sitting":
            print("Free Joints Analysis (1, 2, 3):")
            for i in [0, 1, 2]:
                cum_dist = cumulative_distance[i]
                net_disp = total_displacement[i]
                max_vel = max_velocities[i]
                print(f"  Joint {i+1}: Cumulative Distance={cum_dist:.2f}°, Net Displacement={net_disp:.2f}°")
                print(f"           Max Speed={max_vel:.2f}°/s")
                if motion_duration > 0:
                    avg_vel = net_disp / motion_duration
                    avg_spd = cum_dist / motion_duration
                    print(f"           Avg Velocity={avg_vel:.2f}°/s, Avg Speed={avg_spd:.2f}°/s")
        
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
print(f"Joint 3 Range: 80° to 144° (Total: 64° range)")
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
            # Standing mode control
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

        # FIXED: Update plotting data with correct calculations
        if trajectory_active:
            update_motion_statistics()
            current_time = time.time() - trajectory_start_time
            
            if mode == "standing":
                # Focus on Joint 2 for standing mode
                cumulative_dist = cumulative_distance[2]
                net_displacement = total_displacement[2]
                instantaneous_vel = joint_velocities[2]  # Current velocity (can be negative)
                instantaneous_spd = abs(instantaneous_vel)  # Current speed (always positive)
                
                # FIXED: Calculate running averages using total elapsed time
                if motion_duration > 0:
                    avg_velocity_current = net_displacement / motion_duration
                    avg_speed_current = cumulative_dist / motion_duration
                else:
                    avg_velocity_current = 0.0
                    avg_speed_current = 0.0
                
                F_plot = forces[2]
                
                try:
                    data_queue.put_nowait((current_time, F_plot, cumulative_dist, net_displacement, 
                                         instantaneous_vel, instantaneous_spd, 
                                         avg_velocity_current, avg_speed_current))
                except queue.Full:
                    try:
                        data_queue.get_nowait()
                        data_queue.put_nowait((current_time, F_plot, cumulative_dist, net_displacement, 
                                             instantaneous_vel, instantaneous_spd, 
                                             avg_velocity_current, avg_speed_current))
                    except queue.Empty:
                        pass

        next_cycle += dt
        sleep_time = next_cycle - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)

# =======================
# --- PLOTTING (MAIN THREAD) ---
# =======================
def plotting_loop_main_thread():
    global time_data, force_data, cumulative_distance_data, displacement_data, velocity_data, speed_data, avg_velocity_data, avg_speed_data
    
    plt.ion()
    fig, axs = plt.subplots(3, 3, figsize=(16, 12))
    fig.suptitle(f"Real-Time Complete Trajectory Data ({mode.capitalize()} Mode - Joint 2)", fontsize=14)
    
    # Arrange subplots
    ax_force = axs[0,0]
    ax_cumulative = axs[0,1]
    ax_displacement = axs[0,2]
    ax_velocity = axs[1,0]
    ax_speed = axs[1,1]
    ax_avg_v = axs[1,2]
    ax_avg_s = axs[2,0]
    
    # Hide unused subplots
    axs[2,1].axis('off')
    axs[2,2].axis('off')

    # Set up subplot titles and labels
    ax_force.set_title("Applied Force (Fz)")
    ax_force.set_ylabel("Force (N)")
    ax_cumulative.set_title("Cumulative Distance Traveled")
    ax_cumulative.set_ylabel("Distance (°)")
    ax_displacement.set_title("Net Displacement from Start")
    ax_displacement.set_ylabel("Displacement (°)")
    ax_velocity.set_title("Instantaneous Velocity")
    ax_velocity.set_ylabel("Velocity (°/s)")
    ax_speed.set_title("Instantaneous Speed")
    ax_speed.set_ylabel("Speed (°/s)")
    ax_avg_v.set_title("Average Velocity (Net/Time)")
    ax_avg_v.set_ylabel("Avg Velocity (°/s)")
    ax_avg_s.set_title("Average Speed (Total/Time)")
    ax_avg_s.set_ylabel("Avg Speed (°/s)")

    # Initialize line objects
    ln_force, = ax_force.plot([], [], 'b-', linewidth=1.5, label="Force (N)")
    ln_cumulative, = ax_cumulative.plot([], [], 'g-', linewidth=1.5, label="Cumulative (°)")
    ln_displacement, = ax_displacement.plot([], [], 'orange', linewidth=1.5, label="Displacement (°)")
    ln_velocity, = ax_velocity.plot([], [], 'r-', linewidth=1.5, label="Velocity (°/s)")
    ln_speed, = ax_speed.plot([], [], 'm-', linewidth=1.5, label="Speed (°/s)")
    ln_avg_v, = ax_avg_v.plot([], [], 'c-', linewidth=2.0, label="Avg Velocity (°/s)")
    ln_avg_s, = ax_avg_s.plot([], [], 'darkred', linewidth=2.0, label="Avg Speed (°/s)")

    active_axes = [ax_force, ax_cumulative, ax_displacement, ax_velocity, ax_speed, ax_avg_v, ax_avg_s]
    for ax in active_axes:
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_xlabel("Time (s)")

    last_draw = time.time()
    update_interval = 0.1
    
    while plot_running:
        updated = False
        try:
            while True:
                (t, f, cum_d, net_d, v, s, avgv, avgs) = data_queue.get_nowait()
                time_data.append(t)
                force_data.append(f)
                cumulative_distance_data.append(cum_d)
                displacement_data.append(net_d)
                velocity_data.append(v)
                speed_data.append(s)
                avg_velocity_data.append(avgv)
                avg_speed_data.append(avgs)
                updated = True
        except queue.Empty:
            pass

        if updated and (time.time() - last_draw) > update_interval:
            ln_force.set_data(time_data, force_data)
            ln_cumulative.set_data(time_data, cumulative_distance_data)
            ln_displacement.set_data(time_data, displacement_data)
            ln_velocity.set_data(time_data, velocity_data)
            ln_speed.set_data(time_data, speed_data)
            ln_avg_v.set_data(time_data, avg_velocity_data)
            ln_avg_s.set_data(time_data, avg_speed_data)

            for ax in active_axes:
                ax.relim()
                ax.autoscale_view()

            plt.pause(0.001)
            last_draw = time.time()

        time.sleep(0.01)

    plt.ioff()
    plt.show()

# =======================
# --- START THREADS ---
# =======================
ctrl_thread = threading.Thread(target=control_loop, daemon=True)
ctrl_thread.start()

plotting_loop_main_thread()

