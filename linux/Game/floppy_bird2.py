import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Rectangle
from matplotlib.animation import FuncAnimation
from PIL import Image
import os
import threading

M = [3.0, 3.0, 2.0, 3.0, 3.0, 3.0]
B = [2.5, 2.5, 2.5, 3.0, 3.0, 3.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]        # Spring stiffness (0 = no return when force < threshold)
force_to_deg = 7.70                         # N → degrees scaling
dt = 0.008
gravity_compensation_samples = 100
baseline_forces = None
startup_delay = 2.0

# --- JOINT 3 LIMITS (REAL ROBOT) ---
JOINT3_MIN = 90.0       # Arm DOWN → Bird at TOP
JOINT3_MAX = 120.0      # Arm UP   → Bird at BOTTOM

JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),   2: (-179.0, -35.0),  3: (60.0, 144.0),
    4: (-258.0, 80.0),  5: (-170.0, 12.0),  6: (-170.0, 170.0),
}

shared_data = {"joint3_pos": 100.0, "running": True}
data_lock = threading.Lock()
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

# ================================
# LOAD BIRD PNG FRAMES
# ================================
BIRD_FRAMES = None
SPRITE_DIR = '/home/um/fairino-python-sdk-main/linux/Game/grumpy bird sprite sheets'
if os.path.isdir(SPRITE_DIR):
    files = sorted([f for f in os.listdir(SPRITE_DIR) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
    paths = [os.path.join(SPRITE_DIR, f) for f in files]
    if paths:
        try:
            BIRD_FRAMES = [np.array(Image.open(p).convert('RGBA')) for p in paths]
            print(f"Loaded {len(BIRD_FRAMES)} bird animation frames")
        except Exception as e:
            print("Error loading bird images:", e)

if not BIRD_FRAMES:
    print("No bird images found → using yellow ellipse bird")
    BIRD_FRAMES = None

# ================================
# FT SENSOR & GRAVITY COMPENSATION
# ================================
def init_ft_sensor():
    company = 24; device = 0
    robot.FT_SetConfig(company, device)
    robot.FT_Activate(0); time.sleep(0.5)
    robot.FT_Activate(1); time.sleep(0.5)
    robot.SetLoadWeight(0, 0.0); robot.SetLoadCoord(0.0, 0.0, 0.0)
    robot.FT_SetZero(0); time.sleep(0.5)
    robot.FT_SetZero(1); time.sleep(0.5)
    print("FT Sensor initialized and zeroed.")

def calibrate_baseline_forces():
    global baseline_forces
    print("Calibrating gravity compensation...")
    samples = []
    for _ in range(gravity_compensation_samples):
        data = robot.FT_GetForceTorqueRCS()
        if data[0] == 0:
            forces = [data[1][0], -data[1][1], data[1][2], data[1][3], data[1][4], data[1][5]]
            samples.append(forces)
        time.sleep(0.01)
    if samples:
        baseline_forces = np.mean(samples, axis=0).tolist()
        print(f"Baseline forces: {[f'{x:.2f}' for x in baseline_forces]}")
    else:
        baseline_forces = [0.0] * 6

def shutdown(sig, frame):
    with data_lock:
        shared_data["running"] = False
    robot.ServoMoveEnd()
    print("\nStopped safely.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

def is_within_safety_limits(joint_idx, angle):
    if joint_idx in JOINT_SAFETY_LIMITS:
        mn, mx = JOINT_SAFETY_LIMITS[joint_idx]
        return mn <= angle <= mx
    return True

# ================================
# DRAW FLUFFY CLOUD FUNCTION
# ================================
def draw_cloud(ax, cx, cy, scale=1.0, alpha=0.7):
    """Draw a fluffy cloud made of overlapping circles"""
    # Main body circles
    ax.add_patch(Ellipse((cx, cy), 80*scale, 50*scale, color='white', alpha=alpha, zorder=5))
    ax.add_patch(Ellipse((cx-30*scale, cy+5), 60*scale, 45*scale, color='white', alpha=alpha, zorder=5))
    ax.add_patch(Ellipse((cx+35*scale, cy+8), 65*scale, 48*scale, color='white', alpha=alpha, zorder=5))
    
    # Top puffs for fluffiness
    ax.add_patch(Ellipse((cx-15*scale, cy+25), 45*scale, 35*scale, color='white', alpha=alpha, zorder=5))
    ax.add_patch(Ellipse((cx+20*scale, cy+28), 50*scale, 38*scale, color='white', alpha=alpha, zorder=5))
    
    # Bottom smoothing
    ax.add_patch(Ellipse((cx-10*scale, cy-12), 50*scale, 30*scale, color='white', alpha=alpha, zorder=5))
    ax.add_patch(Ellipse((cx+25*scale, cy-10), 45*scale, 28*scale, color='white', alpha=alpha, zorder=5))

# ================================
# FLAPPY BIRD GAME
# ================================
def flappy_bird_game():
    fig, ax = plt.subplots(figsize=(16, 10), facecolor='#87CEEB')
    ax.set_xlim(0, 1000)
    ax.set_ylim(0, 700)
    ax.set_aspect('equal')
    ax.axis('off')

    sun = plt.Circle((800, 600), 60, color='yellow', alpha=0.8)
    ax.add_patch(sun)

    bird_x = 200
    bird_size = 90
    pipes = []
    score = 0
    frame_count = 0
    restart_timer = 0
    last_gap_y = 350

    def create_pipe():
        nonlocal last_gap_y
        while True:
            gap_y = np.random.randint(230, 470)
            if abs(gap_y - last_gap_y) > 70:
                last_gap_y = gap_y
                break
        pipes.append({'x': 1100, 'gap_y': gap_y, 'passed': False})

    def draw_pipes():
        gap = 200
        for p in pipes:
            ax.add_patch(Rectangle((p['x'], 0), 90, p['gap_y'] - gap//2, color='#228B22', ec='#006400', lw=6))
            ax.add_patch(Rectangle((p['x']-15, p['gap_y'] - gap//2 - 40), 120, 40, color='#228B22'))
            ax.add_patch(Rectangle((p['x'], p['gap_y'] + gap//2), 90, 700, color='#228B22', ec='#006400', lw=6))
            ax.add_patch(Rectangle((p['x']-15, p['gap_y'] + gap//2), 120, 40, color='#228B22'))

    create_pipe()
    score_text = ax.text(500, 650, 'SCORE: 0', ha='center', fontsize=32, fontweight='bold', color='white',
                         bbox=dict(facecolor='navy', alpha=0.8, boxstyle='round,pad=1'))
    joint_text = ax.text(20, 650, '', fontsize=18, color='lime', fontweight='bold',
                         bbox=dict(facecolor='black', alpha=0.7, boxstyle='round,pad=0.5'))
    go_text = ax.text(500, 350, '', ha='center', fontsize=70, color='red', fontweight='bold')

    # Pre-generate stars for night sky (positions and twinkle params)
    stars = []
    num_stars = 120
    for _ in range(num_stars):
        sx = np.random.uniform(0, 1000)
        sy = np.random.uniform(420, 680)
        size = np.random.uniform(8, 30)
        base_alpha = np.random.uniform(0.25, 0.9)
        phase = np.random.uniform(0, 2*np.pi)
        stars.append({'x': sx, 'y': sy, 'size': size, 'alpha': base_alpha, 'phase': phase})

    def animate(frame):
        nonlocal score, frame_count, restart_timer
        frame_count += 1
        if not shared_data["running"]:
            plt.close()
            return []

        with data_lock:
            j3 = shared_data["joint3_pos"]

        bird_y = np.interp(j3, [JOINT3_MIN, JOINT3_MAX], [560, 140])

        ax.clear()
        ax.set_xlim(0, 1000); ax.set_ylim(0, 700); ax.axis('off')

        # --- Day/Night cycle with left->right wipe transition ---
        # Timing (frame interval ~16ms)
        dt_frame = 0.016
        elapsed = frame_count * dt_frame
        cycle_base = 20.0  # 10s day + 10s night (toggle every 10s)
        tmod = elapsed % cycle_base
        transition_duration = 2.0  # seconds for wipe transition

        # Define palettes
        day_colors = ['#87CEEB', '#A0D8FF', '#B8E3FF']
        night_colors = ['#07112a', '#001022', '#000412']  # gradient bands for night

        # Determine base state and transition
        in_transition = False
        target_state = None
        progress = 0.0

        # day -> night at t=10s, night -> day near cycle end (wrap)
        if 10.0 <= tmod < 10.0 + transition_duration:
            in_transition = True
            target_state = 'night'
            progress = (tmod - 10.0) / transition_duration
        elif tmod >= cycle_base - transition_duration:
            in_transition = True
            target_state = 'day'
            progress = (tmod - (cycle_base - transition_duration)) / transition_duration

        # smoothstep easing
        def smoothstep(x):
            x = np.clip(x, 0.0, 1.0)
            return x*x*(3 - 2*x)

        eased = smoothstep(progress) if in_transition else 0.0
        mask_w = eased * 1000.0 if in_transition else 0.0

        # Draw base background
        if not in_transition:
            if tmod < 10.0:
                # full day
                for i, color in enumerate(day_colors):
                    y_pos = i * (700 / 3)
                    ax.add_patch(Rectangle((0, y_pos), 1000, 700/3, color=color, zorder=0))
            else:
                # full night
                for i, color in enumerate(night_colors):
                    y_pos = i * (700 / 3)
                    ax.add_patch(Rectangle((0, y_pos), 1000, 700/3, color=color, zorder=0))
        else:
            # During transition, draw the source fully and overlay the target in a left->right mask
            if target_state == 'night':
                # source: day full
                for i, color in enumerate(day_colors):
                    y_pos = i * (700 / 3)
                    ax.add_patch(Rectangle((0, y_pos), 1000, 700/3, color=color, zorder=0))
                # overlay night gradient slice from left with alpha = eased
                band_h = 700 / 3
                for i, color in enumerate(night_colors):
                    ax.add_patch(Rectangle((0, i*band_h), mask_w, band_h, color=color, alpha=eased, zorder=1))
            else:
                # source: night full
                for i, color in enumerate(night_colors):
                    y_pos = i * (700 / 3)
                    ax.add_patch(Rectangle((0, y_pos), 1000, 700/3, color=color, zorder=0))
                # overlay day gradient slice from left with alpha = eased
                band_h = 700 / 3
                for i, color in enumerate(day_colors):
                    ax.add_patch(Rectangle((0, i*band_h), mask_w, band_h, color=color, alpha=eased, zorder=1))

        # Add sun (NO SPOTS - always clean yellow or white)
        if not in_transition:
            sun_alpha = 0.9 if tmod < 10.0 else 0.15
        else:
            if target_state == 'night':
                sun_alpha = max(0.05, 0.9 * (1.0 - eased))
            else:
                sun_alpha = max(0.05, 0.15 + 0.85 * eased)

        sun.set_alpha(sun_alpha)
        ax.add_patch(sun)

        # --- STARS: show during night; during transitions reveal/hide left->right using mask_w ---
        star_xs = []
        star_ys = []
        star_sizes = []
        star_colors = []

        # Determine simple night_global flag
        if not in_transition:
            night_global = (tmod >= 10.0)
        else:
            # during transition, treat global as True when target is night and eased>0.5
            night_global = (target_state == 'night' and eased > 0.5) or (target_state == 'day' and eased < 0.5 and tmod >= 10.0)

        for s in stars:
            sx, sy = s['x'], s['y']
            tw = 0.6 + 0.4 * np.sin(frame_count * 0.08 + s['phase'])
            base = s['alpha']
            # compute per-star alpha based on transition and position
            if not in_transition:
                alpha = base * tw if night_global else 0.0
            else:
                if target_state == 'night':
                    # left area revealed as night
                    if sx <= mask_w:
                        alpha = base * tw * eased
                    else:
                        alpha = 0.0
                else:
                    # target is day: left area becomes day, right remains night
                    if sx <= mask_w:
                        alpha = base * tw * (1.0 - eased)
                    else:
                        alpha = base * tw

            if alpha > 0.005:
                star_xs.append(sx)
                star_ys.append(sy)
                star_sizes.append(s['size'] * (0.6 + tw))
                star_colors.append((1.0, 1.0, 1.0, np.clip(alpha, 0.0, 1.0)))

        if star_xs:
            ax.scatter(star_xs, star_ys, s=star_sizes, c=star_colors, marker='o', zorder=0.5)

        # Draw fluffy clouds (6 clouds with different sizes)
        cloud_positions = [
            (100, 620, 1.0),
            (280, 600, 0.85),
            (460, 625, 1.1),
            (640, 605, 0.95),
            (820, 615, 1.05),
            (1000, 595, 0.9)
        ]
        
        for base_x, cy, scale in cloud_positions:
            cx = (base_x - frame_count*1.0) % 1200 ##
            draw_cloud(ax, cx, cy, scale=scale, alpha=0.7)

        # Ground
        offset = -frame_count * 7 % 2000##
        ax.add_patch(Rectangle((offset, 0), 2000, 130, color='#8B4513'))
        ax.add_patch(Rectangle((offset, 110), 2000, 20, color='#228B22'))

        # Pipes movement & scoring
        for p in pipes[:]:
            p['x'] -= 15.0 ## Pipes speed
            if p['x'] < -150:
                pipes.remove(p)
            if not p['passed'] and p['x'] + 90 < bird_x:
                p['passed'] = True
                score += 1

        if len(pipes) < 3 and (not pipes or pipes[-1]['x'] < 500):
            create_pipe()
        draw_pipes()

        # Bird
        if BIRD_FRAMES:
            idx = (frame_count // 5) % len(BIRD_FRAMES)
            img = BIRD_FRAMES[idx]
            ax.imshow(img, extent=[bird_x - bird_size/2, bird_x + bird_size/2,
                                   bird_y - bird_size/2, bird_y + bird_size/2],
                      zorder=10, interpolation='bilinear')
        else:
            ax.add_patch(Ellipse((bird_x, bird_y), 80, 60, color='#FFD700', ec='orange', lw=4, zorder=10))

        # HUD
        score_text.set_text(f'SCORE: {score}')
        ax.add_artist(score_text)
        joint_text.set_text(f'Joint 3: {j3:.1f}°')
        ax.add_artist(joint_text)

        # Collision detection
        hit_pipe = any(
            bird_x + 45 > p['x'] and bird_x - 45 < p['x'] + 90 and
            (bird_y + 45 > p['gap_y'] + 100 or bird_y - 45 < p['gap_y'] - 100)
            for p in pipes
        )
        hit_bounds = bird_y < 140 or bird_y > 560

        if (hit_pipe or hit_bounds) and restart_timer == 0:
            restart_timer = time.time()
            pipes.clear()
            create_pipe()
            score = 0

        if restart_timer:
            elapsed = time.time() - restart_timer
            cd = max(0, int(3 - elapsed))
            go_text.set_text(f'GAME OVER!\nRestarting in {cd}...')
            ax.add_artist(go_text)
            if elapsed > 2.8:
                restart_timer = 0

        return []

    ani = FuncAnimation(fig, animate, interval=16, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.subplots_adjust(top=0.95)
    plt.show()

# ================================
# ROBOT CONTROL LOOP — ONLY JOINT 3 FREE, ALL OTHERS LOCKED
# ================================
def control_loop():
    global baseline_forces
    init_ft_sensor()

    err, pos = robot.GetActualJointPosDegree()
    if err != 0:
        print("Failed to read joint positions")
        sys.exit(1)

    if not all(is_within_safety_limits(i+1, pos[i]) for i in range(6)):
        print("Robot not in safe starting position!")
        sys.exit(1)

    calibrate_baseline_forces()

    home_pos = pos.copy()           # This is the FIXED position for locked joints
    desired_pos = pos.copy()
    velocity = [0.0] * 6
    free_joint_idx = 2              # Joint 3 → index 2

    Mj, Bj, Kj = M[free_joint_idx], B[free_joint_idx], K[free_joint_idx]

    robot.ServoMoveStart()
    time.sleep(startup_delay)

    print("="*70)
    print("FLAPPY BIRD WITH ROBOT ARM")
    print("ONLY JOINT 3 IS ACTIVE — ALL OTHER JOINTS ARE LOCKED")
    print("↓ Lower arm (60°)  → Bird flies UP (TOP)")
    print("↑ Raise arm (143°) → Bird goes DOWN (BOTTOM)")
    print("Apply upward/downward force on tool → control bird height")
    print("="*70)

    while shared_data["running"]:
        data = robot.FT_GetForceTorqueRCS()
        if data[0] != 0:
            time.sleep(dt)
            continue

        raw = [data[1][0], -data[1][1], data[1][2], data[1][3], data[1][4], data[1][5]]
        forces = [r - b for r, b in zip(raw, baseline_forces)] if baseline_forces else raw
        fz_force = forces[2]
        fz_force = 0.0 if abs(fz_force) < 0.5 else fz_force

        # === PHYSICS ONLY FOR JOINT 3 ===
        j = free_joint_idx
        if abs(fz_force) < 1.0:
            # Soft return to home when no strong force
            spring_force = -Kj * (desired_pos[j] - home_pos[j]) / force_to_deg
            acc = (spring_force - Bj * velocity[j]) / Mj
        else:
            # Direct force control
            acc = (fz_force - Bj * velocity[j]) / Mj

        velocity[j] += acc * dt
        delta = velocity[j] * dt * force_to_deg
        desired_pos[j] += delta

        # === HARD LOCK ALL OTHER JOINTS ===
        for i in range(6):
            if i != free_joint_idx:
                desired_pos[i] = home_pos[i]
                velocity[i] = 0.0

        # === SAFETY CLAMP FOR JOINT 3 ===
        if desired_pos[j] < JOINT3_MIN:
            desired_pos[j] = JOINT3_MIN
            if velocity[j] < 0: velocity[j] = 0
        elif desired_pos[j] > JOINT3_MAX:
            desired_pos[j] = JOINT3_MAX
            if velocity[j] > 0: velocity[j] = 0

        # Share with game
        with data_lock:
            shared_data["joint3_pos"] = desired_pos[j]

        # Send command
        robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        time.sleep(dt)

# ================================
# MAIN
# ================================
if __name__ == "__main__":
    robot_thread = threading.Thread(target=control_loop, daemon=True)
    robot_thread.start()

    print("\n" + "="*70)
    print("   FLAPPY BIRD - FORCE CONTROLLED BY ROBOT ARM   ")
    print("   ↓ Lower arm → Bird flies UP!                 ")
    print("   ↑ Raise arm → Bird goes DOWN!                ")
    print("="*70 + "\n")

    try:
        flappy_bird_game()
    except KeyboardInterrupt:
        pass
    finally:
        with data_lock:
            shared_data["running"] = False
        robot.ServoMoveEnd()
        print("\nGame ended safely. Thank you for playing!")