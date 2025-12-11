import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle, Polygon
from matplotlib.animation import FuncAnimation
import threading

# --- PARAMETERS PER JOINT ---
M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
force_thresholds = [2.5, 2.5, 3.0, 1.2, 1.2, 1.2]

# Scaling: degrees per Newton
force_to_deg = 7.70

# Control loop timing
dt = 0.008

# Gravity compensation variables
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 2.0

# --- LIMITS FOR FLAPPY BIRD ---
UPPER_LIMIT = 60.0
LOWER_LIMIT = 143.0

# --- SAFETY LIMITS FOR EACH JOINT (degrees) ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),
    2: (-179.0, -35.0),
    3: (60.0, 144.0),
    4: (-258.0, 80.0),
    5: (-170.0, 12.0),
    6: (-170.0, 170.0),
}

# --- SHARED DATA FOR COMMUNICATION BETWEEN THREADS ---
shared_data = {
    "joint3_pos": 100.0,
    "running": True
}
data_lock = threading.Lock()

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

# --- GRAVITY COMPENSATION CALIBRATION ---
def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating baseline forces (gravity compensation)...")
    
    force_samples = []
    for i in range(gravity_compensation_samples):
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
        baseline_forces = [sum(forces[i] for forces in force_samples) / len(force_samples) 
                          for i in range(6)]
        print(f"Baseline forces captured: {[f'{f:.2f}' for f in baseline_forces]}")
        print("Gravity compensation calibrated.")
    else:
        print("Warning: Could not capture baseline forces!")
        baseline_forces = [0.0] * 6

# --- SIGNAL HANDLER ---
def shutdown(sig, frame):
    with data_lock:
        shared_data["running"] = False
    robot.ServoMoveEnd()
    print("\nServo stopped. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# --- SAFETY LIMIT CHECK FUNCTION ---
def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        min_lim, max_lim = JOINT_SAFETY_LIMITS[joint_idx]
        return min_lim <= angle <= max_lim
    return True

# --- FLAPPY BIRD GAME ---
def flappy_bird_game():
    fig, ax = plt.subplots(figsize=(16, 10), facecolor='#87CEEB')
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 700)
    ax.set_aspect('equal')
    ax.axis('off')
    
    sun = plt.Circle((800, 600), 60, color='yellow', alpha=0.8)
    ax.add_patch(sun)
    
    bird_x = 200
    body = Ellipse((bird_x, 350), 80, 60, color='#FFD700', ec='orange', lw=3)
    eye = plt.Circle((bird_x + 20, 360), 12, color='white')
    pupil = plt.Circle((bird_x + 25, 362), 6, color='black')
    beak = Polygon([[bird_x + 40, 350], [bird_x + 65, 345], [bird_x + 40, 340]], color='orange')
    wing_l = Polygon([[bird_x - 20, 350], [bird_x - 50, 340], [bird_x - 30, 320]], color='#FFAA00')
    wing_r = Polygon([[bird_x + 20, 350], [bird_x + 50, 340], [bird_x + 30, 320]], color='#FFAA00')
    
    score_text = ax.text(500, 650, 'SCORE: 0', ha='center', fontsize=28, fontweight='bold', color='white',
                         bbox=dict(facecolor='navy', alpha=0.7, boxstyle='round,pad=0.5'))
    joint_text = ax.text(20, 650, '', fontsize=16, color='red', fontweight='bold')
    go_text = ax.text(500, 350, '', ha='center', fontsize=60, color='red', fontweight='bold')
    
    pipes = []
    score = 0
    ground_offset = 0
    restart_timer = 0

    def create_pipe():
        gap_y = np.random.randint(220, 480)
        pipes.append({'x': 1100, 'gap_y': gap_y, 'passed': False})

    def draw_pipes():
        gap_size = 200
        for p in pipes:
            ax.add_patch(Rectangle((p['x'], p['gap_y'] + gap_size//2), 90, 700, color='#228B22', ec='#006400', lw=5))
            ax.add_patch(Rectangle((p['x'] - 15, p['gap_y'] + gap_size//2 - 40), 120, 40, color='#228B22'))
            ax.add_patch(Rectangle((p['x'], 0), 90, p['gap_y'] - gap_size//2, color='#228B22', ec='#006400', lw=5))
            ax.add_patch(Rectangle((p['x'] - 15, p['gap_y'] - gap_size//2), 120, 40, color='#228B22'))

    create_pipe()

    def animate(frame):
        nonlocal score, ground_offset, restart_timer
        
        if not shared_data["running"]:
            plt.close()
            return []
        
        # Get current joint 3 position from robot
        with data_lock:
            j3 = shared_data["joint3_pos"]
        
        # Map joint 3 position to bird Y position
        bird_y = np.interp(j3, [UPPER_LIMIT, LOWER_LIMIT], [560, 140])
        
        # Update bird position
        body.center = (bird_x, bird_y)
        eye.center = (bird_x + 20, bird_y + 10)
        pupil.center = (bird_x + 25, bird_y + 12)
        beak.set_xy([[bird_x + 40, bird_y + 5], [bird_x + 65, bird_y], [bird_x + 40, bird_y - 5]])
        
        # Animate wings
        wing_angle = 12 * np.sin(frame * 0.35)
        wing_l.set_xy([[bird_x - 20, bird_y], [bird_x - 55, bird_y - wing_angle], [bird_x - 30, bird_y - 35]])
        wing_r.set_xy([[bird_x + 20, bird_y], [bird_x + 55, bird_y - wing_angle], [bird_x + 30, bird_y - 35]])

        # Update pipes
        for p in pipes[:]:
            p['x'] -= 4
            if p['x'] < -150:
                pipes.remove(p)
            if not p['passed'] and p['x'] + 90 < bird_x:
                p['passed'] = True
                score += 1
        
        if len(pipes) == 0 or pipes[-1]['x'] < 700:
            create_pipe()

        # Collision detection
        collided = any(
            bird_x + 40 > p['x'] and bird_x - 40 < p['x'] + 90 and
            (bird_y + 35 > p['gap_y'] + 100 or bird_y - 35 < p['gap_y'] - 100)
            for p in pipes
        ) or bird_y < 130 or bird_y > 670

        # Redraw everything
        ax.clear()
        ax.set_xlim(0, 1000)
        ax.set_ylim(0, 700)
        ax.axis('off')
        ax.add_patch(sun)
        
        # Draw clouds
        for i in range(6):
            cx = (100 + i * 180 - frame * 0.6) % 1200
            ax.add_patch(Ellipse((cx, 620), 130, 45, color='white', alpha=0.7))
            ax.add_patch(Ellipse((cx + 50, 635), 90, 35, color='white', alpha=0.7))
        
        # Draw ground
        ground_offset = -frame * 5 % 2000
        ax.add_patch(Rectangle((ground_offset, 0), 2000, 130, color='#8B4513'))
        ax.add_patch(Rectangle((ground_offset, 110), 2000, 20, color='#228B22'))
        
        # Draw pipes and bird
        draw_pipes()
        ax.add_patch(body)
        ax.add_patch(eye)
        ax.add_patch(pupil)
        ax.add_patch(beak)
        ax.add_patch(wing_l)
        ax.add_patch(wing_r)
        
        # Update score and joint position text
        score_text.set_text(f'SCORE: {score}')
        ax.add_artist(score_text)
        joint_text.set_text(f'Joint 3: {j3:.1f}°')
        ax.add_artist(joint_text)

        # Handle game over
        if collided and restart_timer == 0:
            restart_timer = time.time()
            pipes.clear()
            create_pipe()
            score = 0
        
        if restart_timer > 0:
            elapsed = time.time() - restart_timer
            countdown = max(0, int(2 - elapsed))
            go_text.set_text(f'GAME OVER\nRestarting in {countdown}...')
            ax.add_artist(go_text)
            if elapsed > 1.8:
                restart_timer = 0
        
        return []

    ani = FuncAnimation(fig, animate, interval=16, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.subplots_adjust(top=0.95)
    plt.show()

# --- ROBOT CONTROL LOOP ---
def control_loop():
    global baseline_forces
    
    # Initialize
    init_ft_sensor()
    error, joint_pos = robot.GetActualJointPosDegree()
    if error != 0:
        print("Error getting joint positions. Exiting.")
        sys.exit(1)
    
    # Safety check
    safety_ok = True
    for j in range(6):
        joint_num = j + 1
        if not is_within_safety_limits(joint_num, joint_pos[j]):
            print(f"Initial joint angle out of safety range for Joint {joint_num}: {joint_pos[j]:.2f}°")
            safety_ok = False
    if not safety_ok:
        print("One or more joints are out of safety range. Please manually move the robot to a safe position and restart.")
        sys.exit(1)
    
    # Calibrate baseline forces
    calibrate_baseline_forces()
    
    home_pos = joint_pos.copy()
    desired_pos = joint_pos.copy()
    velocity = [0.0] * 6
    
    joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
    print("Current joint angles:")
    for i, angle in enumerate(joint_pos):
        print(f"  {joint_names[i]}: {angle:.2f}°")
    
    # User input for mode
    mode = None
    while mode not in ["sitting", "standing"]:
        mode = input("Select mode (sitting/standing): ").strip().lower()
        if mode not in ["sitting", "standing"]:
            print("Invalid input. Please enter 'sitting' or 'standing'.")
    
    if mode == "sitting":
        print("Mode: Sitting - Joints 1, 2, 3 are free, rest fixed.")
        free_joints = [1, 2, 3]
    else:
        print("Mode: Standing - Only Joint 2 and 3 are active, rest fixed.")
        free_joints = [2, 3]
    
    # Start servo mode
    if robot.ServoMoveStart() != 0:
        print("Failed to start servo mode.")
        sys.exit(1)
    
    print(f"Waiting {startup_delay} seconds before enabling control...")
    time.sleep(startup_delay)
    
    print("Admittance control started:")
    print(f"Mode: {mode.capitalize()}")
    print("Press Ctrl+C to stop.")
    
    # Main control loop
    while shared_data["running"]:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt)
            continue
        
        # Get raw forces
        raw_forces = [
            ft_data[1][0],
            -ft_data[1][1],
            ft_data[1][2],
            ft_data[1][3],
            ft_data[1][4],
            ft_data[1][5]
        ]
        
        # Apply gravity compensation
        if baseline_forces is not None:
            forces = [raw_forces[i] - baseline_forces[i] for i in range(6)]
        else:
            forces = raw_forces
        
        # Apply deadband to reduce noise
        deadband = 0.5
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0
        
        if mode == "sitting":
            # Joint 2 (index 2) reacts to Fx
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
            
            # Joint 3 (index 3) reacts to Fx (same as SIT)
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
            # Only Joint 2 (index 2) free, reacts to Fz
            j = 2
            fz_force = forces[2]
            if abs(fz_force) < 1.0:
                home_pos[j] = desired_pos[j]
                spring_force = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
                acc = (spring_force - B[j] * velocity[j]) / M[j]
            else:
                acc = (fz_force - B[j] * velocity[j]) / M[j]
            velocity[j] += acc * dt
            delta = velocity[j] * dt * force_to_deg
            desired_pos[j] += delta

            # Joint 3 (index 3) reacts to Fx (same as SIT)
            # j3 = 3
            # fx_force = forces[0]
            # if abs(fx_force) < 1.0:
            #     home_pos[j3] = desired_pos[j3]
            #     spring_force = -K[j3] * (desired_pos[j3] - home_pos[j3]) / force_to_deg
            #     acc = (spring_force - B[j3] * velocity[j3]) / M[j3]
            # else:
            #     acc = ((-fx_force) - B[j3] * velocity[j3]) / M[j3]
            # velocity[j3] += acc * dt
            # desired_pos[j3] += velocity[j3] * dt * force_to_deg
        
        # Lock all other joints
        for lock_j in range(6):
            if lock_j not in free_joints:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0
        
        # Safety check for all joints
        safety_ok = True
        for j in range(6):
            joint_num = j + 1
            if not is_within_safety_limits(joint_num, desired_pos[j]):
                safety_ok = False
        
        if not safety_ok:
            time.sleep(dt)
            continue
        
        # Update shared data with Joint 3 position for game
        with data_lock:
            shared_data["joint3_pos"] = desired_pos[2]
        
        # Print joint angles
        print(f"Joint 3: {desired_pos[2]:.2f}° | Fx: {forces[0]:.2f}N | Fz: {forces[2]:.2f}N")
        
        # Send command to robot
        err = robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        if err != 0:
            print("ServoJ error:", err)
        
        time.sleep(dt)

# --- MAIN PROGRAM ---
if __name__ == "__main__":
    # Start robot control in a separate thread
    robot_thread = threading.Thread(target=control_loop, daemon=True)
    robot_thread.start()
    
    # Start Flappy Bird game in main thread
    print("\n🎮 Starting Flappy Bird Game...")
    print("Move the robot's Joint 3 to control the bird!")
    time.sleep(1)
    
    try:
        flappy_bird_game()
    except KeyboardInterrupt:
        pass
    finally:
        with data_lock:
            shared_data["running"] = False
        robot.ServoMoveEnd()
        print("\nGame closed. Robot stopped.")