import numpy as np
import Robot # Assuming this is the correct library for your Fairino robot
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections
import threading

# --- DH Kinematics and Jacobian ---
# (Keep the functions dh_transformation, calculate_jacobian,
#  and calculate_end_effector_force exactly as they were in your code
#  or the previous refined version)
def dh_transformation(theta, d, a, alpha):
    # Standard DH Transformation Matrix
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    cos_a, sin_a = np.cos(alpha), np.sin(alpha)
    return np.array([
        [cos_t, -sin_t * cos_a,  sin_t * sin_a, a * cos_t],
        [sin_t,  cos_t * cos_a, -cos_t * sin_a, a * sin_t],
        [0,          sin_a,          cos_a,          d],
        [0,              0,              0,          1]
    ])

def calculate_jacobian(dh_params, joint_angles):
    # Calculates the geometric Jacobian
    n = len(joint_angles)
    J = np.zeros((6, n))
    T_current = np.eye(4)
    p_end_effector = np.zeros(3) # Will be calculated fully at the end
    origins = [np.zeros(3)] # Origin of base frame
    z_axes = [np.array([0, 0, 1])] # Z-axis of base frame

    # Forward pass to get frame origins and z-axes
    for i in range(n):
        # Standard DH parameters: theta_offset, d, a, alpha
        theta_offset, d, a, alpha = dh_params[i]
        # Apply actual joint angle relative to the DH frame definition
        current_theta = joint_angles[i] + theta_offset
        T_i = dh_transformation(current_theta, d, a, alpha)
        T_current = T_current @ T_i
        origins.append(T_current[:3, 3])
        z_axes.append(T_current[:3, 2])

    p_end_effector = origins[-1] # Position of the end-effector

    # Backward pass to calculate Jacobian columns
    for i in range(n):
        p_i = origins[i]
        z_i = z_axes[i]
        # Assuming all joints are revolute for this standard calculation
        J[:3, i] = np.cross(z_i, (p_end_effector - p_i)) # Linear velocity component
        J[3:, i] = z_i # Angular velocity component

    return J

def calculate_end_effector_force(joint_torques, dh_params, joint_angles):
    # Calculates F = (J^T)^# * tau
    J = calculate_jacobian(dh_params, joint_angles)
    try:
        # Use pinv on J.T directly, adjust rcond as needed for stability vs accuracy
        J_T_pseudo_inv = np.linalg.pinv(J.T, rcond=1e-4)
        force_moment = J_T_pseudo_inv @ joint_torques
        return force_moment
    except np.linalg.LinAlgError:
        print("Singularity encountered in Jacobian transpose pseudo-inverse calculation.")
        return np.zeros(6) # Return zero or handle error appropriately

# --- Robot and DH Parameters ---
# STANDARD DH PARAMETERS (theta_offset, d, a, alpha)
# PLEASE VERIFY THESE with your robot documentation for Standard DH convention
DH_PARAMS = [
    (0, 0.18, 0, np.pi/2),  # Joint 1 -> 2
    (0, 0, 0.7, 0),       # Joint 2 -> 3 (Link length 'a' = 0.7)
    (0, 0, 0.586, 0),     # Joint 3 -> 4 (Link length 'a' = 0.586)
    (0, 0.159, 0, np.pi/2), # Joint 4 -> 5
    (0, 0.114, 0, -np.pi/2),# Joint 5 -> 6
    (0, 0.106, 0, 0)        # Joint 6 -> EE
]
# Note: Make sure 'a' values (link lengths) are correct and typically non-negative in Standard DH.
ROBOT_IP = '192.168.58.2'
NUM_JOINTS = 6

# --- Shared Data ---
history_size = 200 # Number of data points for plotting history
latest_force_history = [collections.deque(maxlen=history_size) for _ in range(NUM_JOINTS)]
latest_diff_history = [collections.deque(maxlen=history_size) for _ in range(NUM_JOINTS)]
latest_time_history = collections.deque(maxlen=history_size)

# --- Collision Detection Parameters ---
COLLISION_THRESHOLD = 50.0 # !!! CRITICAL: TUNE THIS VALUE CAREFULLY !!!
DETECTED_COLLISION = False # Global flag to track collision state

# --- 100 Hz Calculation and Detection Loop (Runs in a separate thread) ---
def collision_detection_loop():
    global latest_force_history, latest_diff_history, latest_time_history
    global DETECTED_COLLISION, COLLISION_THRESHOLD
    print("Connecting to robot...")
    try:
        robot = Robot.RPC(ROBOT_IP)
        # Optional: Add a check here to confirm connection status if the library provides one
        robot_connected = True
        print("Robot connected.")
    except Exception as e:
        print(f"CRITICAL: Failed to connect to robot @ {ROBOT_IP}. Error: {e}")
        robot_connected = False
        return # Exit thread if connection fails

    previous_forces_moments = None
    start_loop_time = time.perf_counter()
    desired_period = 0.01 # Target 100 Hz

    # It's good practice to have a way to signal this thread to stop cleanly
    # For simplicity here, it runs while connected / until main thread exits (daemon=True)
    while robot_connected:
        loop_start_time = time.perf_counter()

        try:
            # 1. Get Data from Robot
            ret_torques, joint_torques_raw = robot.GetJointTorques()
            ret_angles, joint_angles_raw = robot.GetActualJointPosRadian()

            # Basic error checking on robot communication
            if ret_torques != 0 or ret_angles != 0:
                print(f"Warning: Error getting robot data (T:{ret_torques}, A:{ret_angles}). Skipping cycle.")
                time.sleep(desired_period * 0.5) # Avoid busy-looping on error
                continue

            joint_torques = np.array(joint_torques_raw)
            joint_angles = np.array(joint_angles_raw)

            # Check data validity
            if len(joint_torques) != NUM_JOINTS or len(joint_angles) != NUM_JOINTS:
                 print(f"Warning: Incorrect data length (T:{len(joint_torques)}, A:{len(joint_angles)}). Skipping cycle.")
                 time.sleep(desired_period * 0.5)
                 continue

            # 2. Calculate End-Effector Forces/Moments
            forces_moments = calculate_end_effector_force(joint_torques, DH_PARAMS, joint_angles)

            # 3. Calculate Difference from Previous Step
            if previous_forces_moments is not None:
                differences = forces_moments - previous_forces_moments
            else:
                differences = np.zeros_like(forces_moments) # No difference on first iteration

            # --- Update Shared History (Accessed by plotting thread) ---
            current_relative_time = time.perf_counter() - start_loop_time
            latest_time_history.append(current_relative_time)
            for i in range(NUM_JOINTS):
                latest_force_history[i].append(forces_moments[i])
                latest_diff_history[i].append(differences[i])
            # --- End Update ---

            # 4. Perform Collision Detection Check
            if previous_forces_moments is not None:
                # Check if *any* component's absolute difference exceeds threshold
                if np.any(np.abs(differences) > COLLISION_THRESHOLD):
                    if not DETECTED_COLLISION: # Trigger only once per event
                       print(f'COLLISION DETECTED at time {current_relative_time:.3f} s!')
                       print(f'Differences: {np.round(differences, 2)}')
                       DETECTED_COLLISION = True
                       # !!! === ADD ROBOT STOP COMMAND HERE === !!!
                       print("!!! Sending Stop Command (Example) !!!")
                       # try:
                       #     stop_result = robot.Stop() # Replace with actual stop command from Robot.RPC library
                       #     print(f"Stop command result: {stop_result}")
                       # except Exception as stop_e:
                       #     print(f"Error sending stop command: {stop_e}")
                       # !!! ===================================== !!!
                else:
                     # Optional: Reset flag if you want detection to re-trigger if condition persists after stop/recovery
                     # DETECTED_COLLISION = False
                     pass # Keep DETECTED_COLLISION True until manually reset or recovered


            # Store current forces for the next iteration's difference calculation
            previous_forces_moments = forces_moments

        except Exception as e:
            print(f"Error during detection loop cycle: {e}")
            # Consider adding more robust error handling or safe stop logic here

        # 5. Timing Control for 100 Hz
        calculation_time = time.perf_counter() - loop_start_time
        sleep_time = desired_period - calculation_time
        if sleep_time > 0:
            time.sleep(sleep_time)
        # else:
            # Warning if the loop takes too long - indicates inability to meet 100Hz
            # print(f"Warning: Loop cycle took {calculation_time*1000:.2f} ms (Target: {desired_period*1000:.2f} ms)")


# --- Plotting Setup (Runs in Main Thread) ---
fig, axs = plt.subplots(NUM_JOINTS, 2, figsize=(12, 18), sharex=True)
# Create line objects initially, store references for updating
lines_force = [axs[i, 0].plot([], [], lw=1)[0] for i in range(NUM_JOINTS)]
lines_diff = [axs[i, 1].plot([], [], lw=1)[0] for i in range(NUM_JOINTS)]

def init_plot():
    # Set up plot appearance
    for i in range(NUM_JOINTS):
        # Force/Moment plot (Left)
        axs[i, 0].set_title(f'Est. EE Force/Moment {i+1}')
        axs[i, 0].set_ylabel('Value')
        axs[i, 0].grid(True)
        axs[i, 0].set_ylim(-50, 50) # Adjust initial Y limits based on expected values

        # Difference plot (Right)
        axs[i, 1].set_title(f'Difference {i+1}')
        axs[i, 1].set_ylabel('Value Diff.')
        axs[i, 1].grid(True)
        # Set Y limits based on threshold for difference plot
        axs[i, 1].set_ylim(-COLLISION_THRESHOLD*1.5, COLLISION_THRESHOLD*1.5)
        # Add threshold lines for visual reference
        axs[i, 1].axhline(COLLISION_THRESHOLD, color='r', linestyle='--', lw=0.8)
        axs[i, 1].axhline(-COLLISION_THRESHOLD, color='r', linestyle='--', lw=0.8)


    axs[NUM_JOINTS-1, 0].set_xlabel('Time (s)')
    axs[NUM_JOINTS-1, 1].set_xlabel('Time (s)')
    fig.tight_layout()
    # Return list of all line objects that update_plot will modify
    return lines_force + lines_diff

def update_plot(frame):
    # Read the latest data from the shared deques
    # This is generally safe without locks for deque appends/reads if GIL is in effect,
    # but for extreme robustness, a lock could be added around updates/reads.
    times = list(latest_time_history)
    forces = [list(hist) for hist in latest_force_history]
    diffs = [list(hist) for hist in latest_diff_history]

    if not times: # Don't plot if no data yet
        return lines_force + lines_diff

    min_force, max_force = float('inf'), float('-inf')
    min_diff, max_diff = float('inf'), float('-inf')

    # Update the data for each line efficiently
    for i in range(NUM_JOINTS):
        if forces[i]: # Check if list is not empty
            lines_force[i].set_data(times, forces[i])
            # Find min/max for dynamic Y limits if needed
            min_f, max_f = min(forces[i]), max(forces[i])
            min_force = min(min_force, min_f)
            max_force = max(max_force, max_f)
        else:
            lines_force[i].set_data([], []) # Clear line if history is empty

        if diffs[i]:
            lines_diff[i].set_data(times, diffs[i])
            min_d, max_d = min(diffs[i]), max(diffs[i])
            min_diff = min(min_diff, min_d)
            max_diff = max(max_diff, max_d)
        else:
             lines_diff[i].set_data([], [])

    # --- Dynamic Y-Limit Adjustment (Optional, adjust if needed) ---
    # Can sometimes interfere with blitting or cause plot jumps. Fixed limits might be better.
    # force_margin = (max_force - min_force) * 0.1 if max_force > min_force else 10
    # diff_limit_observed = max(abs(min_diff), abs(max_diff)) if times else 0
    # diff_limit_plot = max(diff_limit_observed * 1.1, COLLISION_THRESHOLD * 1.2)
    # diff_margin = diff_limit_plot * 0.1

    for i in range(NUM_JOINTS):
         axs[i, 0].set_xlim(times[0], times[-1]) # Adjust X axis to time window
         # axs[i, 0].set_ylim(min_force - force_margin, max_force + force_margin) # Dynamic Y Force
         axs[i, 1].set_xlim(times[0], times[-1])
         # axs[i, 1].set_ylim(-diff_limit_plot - diff_margin, diff_limit_plot + diff_margin) # Dynamic Y Diff
    # --- End Dynamic Y-Limit ---

    # Return the list of artists that were modified for blitting
    return lines_force + lines_diff

# --- Main Execution ---
if __name__ == "__main__":
    print("Starting collision detection thread...")
    # daemon=True allows program to exit even if this thread is running
    detection_thread = threading.Thread(target=collision_detection_loop, daemon=True)
    detection_thread.start()

    print("Waiting for detection thread to initialize...")
    time.sleep(1.5) # Allow time for connection and first few data points

    print("Starting plot animation...")
    # Animate plot at 10 Hz (100ms interval)
    ani = FuncAnimation(fig, update_plot, init_func=init_plot,
                        interval=100, blit=True, cache_frame_data=False)
    # blit=True gives smoother animation. If resizing/dynamic limits cause issues, try blit=False.

    plt.show() # Displays the plot and blocks main thread until closed

    print("Plot window closed. Program exiting.")
    # Detection thread stops automatically as it's a daemon thread