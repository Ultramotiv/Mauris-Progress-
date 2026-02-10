# Shoulder Flexion Therapy – Adaptive Assistance with ALL Trial Peak Display
# maitreyi mam recomendation 
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import signal
import numpy as np
import threading
import tkinter as tk
from tkinter import ttk

# --- SAFETY LIMITS ---
JOINT_SAFETY_LIMITS = {
    1: (-90.0, 85.0),
    2: (-179.0, -35.0),
    3: (60.0, 144.0),      # 60 = top (flexed), 144 = bottom (extended)
    4: (-258.0, 80.0),
    5: (-170.0, 12.0),
    6: (-170.0, 170.0),
}

# --- Control Parameters ---
M = [1.8, 1.8, 1.8, 1.0, 1.0, 1.8]
B = [2.0, 2.0, 2.0, 2.5, 2.5, 2.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
force_to_deg = 5.0
dt = 0.008
MAX_VEL_JOINT3 = 40.0

# --- Rep/Trial Settings ---
TOP_POS = 60.0
BOTTOM_POS = 144.0
REP_TOL = 5.0
MAX_BASELINE_TRIALS = 3

# --- Assistance Settings ---
ASSIST_LEVEL_PERCENT = 0.20  # 20% of applied force
TRIGGER_THRESHOLD_RATIO = 0.60  # 60% of baseline
CONSECUTIVE_LOW_TRIALS = 2

# --- State Variables ---
baseline_forces = None
rep_count = 0
trial_count = 0
state = "bottom"

# --- Force Tracking ---
baseline_trial_forces = [0.0] * MAX_BASELINE_TRIALS  # Trials 1-3
all_trial_peaks = []  # Store ALL trial peaks (including 4+)
current_trial_max_fz = 0.0
tracking_active = False
f_baseline = None
assistance_active = False
assist_force_applied = 0.0  # Actual force being added
recent_trial_peaks = []
startup_delay = 2.0

# --- GUI Data ---
gui_data = {
    "joint3_pos": 144.0,
    "rep_count": 0,
    "trial_count": 0,
    "active": False,
    "fx": 0.0,
    "fy": 0.0,
    "fz": 0.0,
    "current_trial_peak": 0.0,
    "all_trials_display": [],  # List of all trial peak strings
    "progress_text": "Trial Progress: 0/3",
    "baseline_text": "",
    "assistance_text": "Mode: Baseline Calibration",
    "assist_force_display": "Assist Force: 0.00 N",
}

def update_gui():
    root = tk.Tk()
    root.title("Adaptive Therapy Monitor - All Trials Display")
    root.geometry("700x850")  # Taller window to fit large text

    # Top-level frame with padding
    main_frame = tk.Frame(root, bg='#f9f9f9')
    main_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)

    # === HEADER ===
    header = tk.Label(main_frame, text="🏥 Shoulder Flexion Therapy – Adaptive Assistance", 
                      font=("Segoe UI", 16, "bold"), fg="#2c3e50", bg='#f9f9f9')
    header.pack(pady=(0, 15))

    # === STATUS & METRICS (TOP SECTION) ===
    top_frame = tk.Frame(main_frame, bg='#f9f9f9')
    top_frame.pack(fill=tk.X, pady=(0, 10))

    # Variables with larger fonts
    pos_var = tk.StringVar(value="Joint 3: --°")
    rep_var = tk.StringVar(value="Repetitions: 0")
    trial_var = tk.StringVar(value="Trials: 0")
    status_var = tk.StringVar(value="Status: Waiting")
    progress_var = tk.StringVar(value="Trial Progress: 0/3")
    current_peak_var = tk.StringVar(value="Current Trial Peak Fz: 0.00 N")
    baseline_var = tk.StringVar(value="")
    assist_var = tk.StringVar(value="Mode: Baseline Calibration")
    assist_force_var = tk.StringVar(value="Assist Force: 0.00 N")
    
    fx_var = tk.StringVar(value="Fx: 0.00 N")
    fy_var = tk.StringVar(value="Fy: 0.00 N")
    fz_var = tk.StringVar(value="Fz: 0.00 N")

    # Use larger, consistent font
    FONT_LARGE = ("Segoe UI", 14)
    FONT_BOLD = ("Segoe UI", 14, "bold")
    FONT_ITALIC = ("Segoe UI", 13, "italic")

    # Pack all top info
    tk.Label(top_frame, textvariable=pos_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w').pack(anchor='w', pady=2)
    tk.Label(top_frame, textvariable=rep_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w').pack(anchor='w', pady=2)
    tk.Label(top_frame, textvariable=trial_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w').pack(anchor='w', pady=2)
    tk.Label(top_frame, textvariable=status_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w').pack(anchor='w', pady=(2, 8))

    tk.Label(top_frame, textvariable=progress_var, font=FONT_BOLD, bg='#f9f9f9', fg="#2980b9", anchor='w').pack(anchor='w', pady=3)
    tk.Label(top_frame, textvariable=current_peak_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w').pack(anchor='w', pady=(3, 10))

    # Baseline & assistance info
    tk.Label(top_frame, textvariable=baseline_var, font=FONT_ITALIC, bg='#f9f9f9', fg="#27ae60", anchor='w').pack(anchor='w', pady=2)
    tk.Label(top_frame, textvariable=assist_var, font=FONT_BOLD, bg='#f9f9f9', fg="#8e44ad", anchor='w').pack(anchor='w', pady=2)
    tk.Label(top_frame, textvariable=assist_force_var, font=FONT_BOLD, bg='#f9f9f9', fg="blue", anchor='w').pack(anchor='w', pady=(2, 12))

    # === CURRENT FORCES ===
    tk.Label(top_frame, text="Current Force Readings:", font=("Segoe UI", 13, "underline"), 
             bg='#f9f9f9', anchor='w').pack(anchor='w')
    tk.Label(top_frame, textvariable=fx_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w', fg="#e74c3c").pack(anchor='w', pady=1)
    tk.Label(top_frame, textvariable=fy_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w', fg="#3498db").pack(anchor='w', pady=1)
    tk.Label(top_frame, textvariable=fz_var, font=FONT_LARGE, bg='#f9f9f9', anchor='w', fg="#27ae60").pack(anchor='w', pady=(1, 15))

    # === SEPARATOR ===
    separator = tk.Frame(main_frame, height=2, bg="#bdc3c7")
    separator.pack(fill=tk.X, pady=10)

    # === ALL TRIALS HISTORY (BOTTOM SECTION) ===
    history_label = tk.Label(main_frame, text="📋 All Trial Peak Forces (Latest at Bottom)", 
                             font=("Segoe UI", 14, "bold"), bg='#f9f9f9', anchor='w')
    history_label.pack(anchor='w', pady=(0, 8))

    # Scrollable history at the bottom
    history_frame = tk.Frame(main_frame, height=250, bg='#ffffff', relief=tk.SUNKEN, bd=1)
    history_frame.pack(fill=tk.BOTH, expand=True)

    scrollbar = tk.Scrollbar(history_frame)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    trials_listbox = tk.Listbox(
        history_frame, 
        yscrollcommand=scrollbar.set,
        font=("Consolas", 13),  # Monospace large font for clean alignment
        bg='#fdfdfd',
        fg='#2c3e50',
        selectbackground='#3498db'
    )
    trials_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=trials_listbox.yview)

    def refresh():
        # Update all live values
        pos_var.set(f"Joint 3: {gui_data['joint3_pos']:.1f}°")
        rep_var.set(f"Repetitions: {gui_data['rep_count']}")
        trial_var.set(f"Trials: {gui_data['trial_count']}")

        status = "Active" if gui_data["active"] else "Idle"
        status_var.set(f"Status: {status}")

        progress_var.set(gui_data["progress_text"])
        current_peak_var.set(f"Current Trial Peak Fz: {gui_data['current_trial_peak']:.2f} N")
        baseline_var.set(gui_data["baseline_text"])
        assist_var.set(gui_data["assistance_text"])
        assist_force_var.set(gui_data["assist_force_display"])

        fx_var.set(f"Fx: {gui_data['fx']:.2f} N")
        fy_var.set(f"Fy: {gui_data['fy']:.2f} N")
        fz_var.set(f"Fz: {gui_data['fz']:.2f} N")

        # Update trial history
        trials_listbox.delete(0, tk.END)
        for trial_text in gui_data["all_trials_display"]:
            trials_listbox.insert(tk.END, trial_text)
        if gui_data["all_trials_display"]:
            trials_listbox.see(tk.END)  # Auto-scroll to latest

        root.after(100, refresh)

    refresh()
    root.mainloop()

# Start GUI
gui_thread = threading.Thread(target=update_gui, daemon=True)
gui_thread.start()
time.sleep(0.5)

# --- Robot Setup ---
robot = Robot.RPC('192.168.58.2')
print("Robot connected.")

def init_ft_sensor():
    company, device = 24, 0
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
    print("Calibrating gravity baseline...")
    samples = []
    for _ in range(100):
        ft = robot.FT_GetForceTorqueRCS()
        if ft[0] == 0:
            forces = [ft[1][0], -ft[1][1], ft[1][2], ft[1][3], ft[1][4], ft[1][5]]
            samples.append(forces)
        time.sleep(0.01)
    baseline_forces = np.mean(samples, axis=0).tolist() if samples else [0.0]*6

def enforce_joint_limits(pos):
    clamped = pos.copy()
    for idx in range(6):
        j_id = idx + 1
        if j_id in JOINT_SAFETY_LIMITS:
            lo, hi = JOINT_SAFETY_LIMITS[j_id]
            clamped[idx] = np.clip(clamped[idx], lo, hi)
    return clamped

def shutdown(sig, frame):
    robot.ServoMoveEnd()
    print("\nStopped safely.")
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown)

# --- Initialize ---
init_ft_sensor()
error, joint_pos = robot.GetActualJointPosDegree()
if error != 0: sys.exit("Failed to read joints.")
calibrate_baseline_forces()

home_pos = joint_pos.copy()
desired_pos = joint_pos.copy()
velocity = [0.0] * 6

if robot.ServoMoveStart() != 0:
    sys.exit("Failed to start servo mode.")

print(f"Waiting {startup_delay}s...")
time.sleep(startup_delay)
print("✅ Therapy STARTED – Adaptive Assistance Mode")

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces
    global rep_count, trial_count, state
    global current_trial_max_fz, tracking_active
    global f_baseline, assistance_active, assist_force_applied, recent_trial_peaks
    global baseline_trial_forces, all_trial_peaks

    while True:
        ft_data = robot.FT_GetForceTorqueRCS()
        if ft_data[0] != 0:
            time.sleep(dt)
            continue

        raw_forces = [ft_data[1][0], -ft_data[1][1], ft_data[1][2],
                      ft_data[1][3], ft_data[1][4], ft_data[1][5]]
        forces = [raw_forces[i] - baseline_forces[i] for i in range(6)] if baseline_forces else raw_forces

        deadband = 0.5
        for i in range(6):
            if abs(forces[i]) < deadband:
                forces[i] = 0.0

        # --- LIVE RAW Fz ---
        raw_fz = forces[2]
        gui_data["fx"] = forces[0]
        gui_data["fy"] = forces[1]
        gui_data["fz"] = raw_fz

        # --- CALCULATE ASSISTANCE FORCE (20% of applied force) ---
        assist_force_applied = 0.0
        if assistance_active and abs(raw_fz) > 1.0:
            assist_force_applied = ASSIST_LEVEL_PERCENT * abs(raw_fz)
            if raw_fz < 0:  # Maintain direction
                assist_force_applied = -assist_force_applied
        
        gui_data["assist_force_display"] = f"Assist Force: {assist_force_applied:.2f} N"

        # --- EFFECTIVE Fz FOR MOTION CONTROL ---
        effective_fz = raw_fz + assist_force_applied

        # --- Joint 3 Control ---
        j = 2
        if abs(effective_fz) < 1.0:
            home_pos[j] = desired_pos[j]
            spring_force = -K[j] * (desired_pos[j] - home_pos[j]) / force_to_deg
            acc = (spring_force - B[j] * velocity[j]) / M[j]
        else:
            acc = (effective_fz - B[j] * velocity[j]) / M[j]

        velocity[j] += acc * dt
        if abs(velocity[j]) > MAX_VEL_JOINT3:
            velocity[j] = np.sign(velocity[j]) * MAX_VEL_JOINT3

        delta = velocity[j] * dt * force_to_deg
        desired_pos[j] += delta

        # Lock all other joints
        for lock_j in range(6):
            if lock_j != j:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0

        desired_pos = enforce_joint_limits(desired_pos)

        # --- Rep/Trial & Force Tracking ---
        err, curr_pos = robot.GetActualJointPosDegree()
        if err == 0:
            j3_pos = curr_pos[2]
            gui_data["joint3_pos"] = j3_pos
            gui_data["active"] = (abs(raw_fz) >= 1.0)

            # Track peak Fz during downward motion
            if tracking_active and state == "top":
                if abs(raw_fz) > abs(current_trial_max_fz):
                    current_trial_max_fz = raw_fz
            gui_data["current_trial_peak"] = abs(current_trial_max_fz)

            # State transitions
            if state == "bottom" and j3_pos <= TOP_POS + REP_TOL:
                rep_count += 1
                state = "top"
                if trial_count < MAX_BASELINE_TRIALS or f_baseline is not None:
                    tracking_active = True
                    current_trial_max_fz = 0.0

            elif state == "top" and j3_pos >= BOTTOM_POS - REP_TOL:
                trial_count += 1
                state = "bottom"
                tracking_active = False
                current_peak_abs = abs(current_trial_max_fz)

                # --- BASELINE PHASE: Trials 1–3 ---
                if f_baseline is None and trial_count <= MAX_BASELINE_TRIALS:
                    baseline_trial_forces[trial_count - 1] = current_peak_abs
                    all_trial_peaks.append(current_peak_abs)
                    
                    trial_display = f"Trial {trial_count}: {current_peak_abs:.2f} N (Baseline)"
                    gui_data["all_trials_display"].append(trial_display)
                    
                    gui_data["progress_text"] = f"Trial Progress: {trial_count}/{MAX_BASELINE_TRIALS}"

                    if trial_count == MAX_BASELINE_TRIALS:
                        f_baseline = sum(baseline_trial_forces) / len(baseline_trial_forces)
                        gui_data["baseline_text"] = f"✅ Baseline Avg: {f_baseline:.2f} N | Threshold (60%): {f_baseline * TRIGGER_THRESHOLD_RATIO:.2f} N"
                        gui_data["assistance_text"] = "Mode: Monitoring (Waiting for data...)"
                        print(f"✅ Baseline: {f_baseline:.2f} N | Trigger: {f_baseline * TRIGGER_THRESHOLD_RATIO:.2f} N")

                # --- ADAPTIVE PHASE: Trial 4+ ---
                elif f_baseline is not None:
                    all_trial_peaks.append(current_peak_abs)
                    recent_trial_peaks.append(current_peak_abs)
                    if len(recent_trial_peaks) > CONSECUTIVE_LOW_TRIALS:
                        recent_trial_peaks.pop(0)

                    threshold = TRIGGER_THRESHOLD_RATIO * f_baseline
                    is_below = current_peak_abs < threshold
                    
                    trial_display = f"Trial {trial_count}: {current_peak_abs:.2f} N"
                    if is_below:
                        trial_display += " ⚠️ Below 60%"
                    gui_data["all_trials_display"].append(trial_display)

                    # Check if last 2 consecutive trials are below threshold
                    if (len(recent_trial_peaks) == CONSECUTIVE_LOW_TRIALS and
                        all(p < threshold for p in recent_trial_peaks)):
                        assistance_active = True
                        gui_data["assistance_text"] = f"Mode: 🔵 ASSISTANCE ACTIVE (+20% of applied force)"
                        print(f"🔄 Trial {trial_count}: Assistance ACTIVATED (2 consecutive trials < 60%)")
                    else:
                        assistance_active = False
                        gui_data["assistance_text"] = "Mode: Monitoring (No assistance)"

            # Update counters
            gui_data["rep_count"] = rep_count
            gui_data["trial_count"] = trial_count

        # Send command to robot
        robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        time.sleep(dt)

# Start control loop
control_loop()