# Shoulder Flexion Therapy – Adaptive Assistance with Enhanced GUI
#  Archana's modification on maytrei mam's new recomendation
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
    3: (62.0, 131.0),      # 60 = top (flexed), 144 = bottom (extended)
    4: (-258.0, 80.0),
    5: (-170.0, 12.0),
    6: (-170.0, 170.0),
}

# --- Control Parameters ---
M = [1.8, 1.8, 1.8, 1.0, 1.0, 1.8]
B = [2.0, 2.0, 2.0, 2.5, 2.5, 2.0]
K = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

force_to_deg = 7.0
dt = 0.008
MAX_VEL_JOINT3 = 40.0

# --- Rep/Trial Settings ---
TOP_POS = 62.0
BOTTOM_POS = 131.0 #131.0
REP_TOL = 5.0
MAX_BASELINE_TRIALS = 3

# --- State Variables ---
baseline_forces = None
rep_count = 0
trial_count = 0
state = "bottom"

# --- Force Tracking ---
baseline_trial_forces = [0.0] * MAX_BASELINE_TRIALS
all_trial_peaks = []
current_trial_max_fz = 0.0
tracking_active = False
f_baseline = None
next_trial_assist_percent = 0.0
startup_delay = 2.0

# --- Trial Timing ---
trial_start_time = None
trial_durations = []

# --- GUI Data ---
gui_data = {
    "joint3_pos": 131.0,
    "rep_count": 0,
    "trial_count": 0,
    "active": False,
    "fx": 0.0,
    "fy": 0.0,
    "fz": 0.0,
    "current_trial_peak": 0.0,
    "all_trials_display": [],
    "progress_text": "Trial Progress: 0/3",
    "baseline_text": "",
    "assistance_text": "Mode: Baseline Calibration",
    "assist_force_display": "Assist Force: 0.00 N",
    "raw_fz_display": "Raw Fz: 0.00 N",
    "assist_fz_display": "Assist Fz: 0.00 N",
    "effective_fz_display": "Effective Fz: 0.00 N",
    "current_assist_percent_display": "Assist Level: 0%",
}

def update_gui():
    root = tk.Tk()
    root.title("Adaptive Therapy Monitor")
    root.geometry("950x1000")
    root.configure(bg='#f4f6f9')
    
    # Custom style
    style = ttk.Style()
    style.theme_use('clam')
    
    # Configure colors
    style.configure('Header.TLabel', font=('Segoe UI', 16, 'bold'), 
                    background='#2c3e50', foreground='white', padding=12)
    style.configure('Section.TLabel', font=('Segoe UI', 13, 'bold'), 
                    background='#34495e', foreground='white', padding=8)
    style.configure('Data.TLabel', font=('Segoe UI', 12), 
                    background='#ecf0f1', padding=10)
    style.configure('Status.TLabel', font=('Segoe UI', 14, 'bold'), 
                    background='#3498db', foreground='white', padding=10)
    
    # Main container
    main_frame = tk.Frame(root, bg='#f4f6f9')
    main_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)
    
    # Header
    header = tk.Label(main_frame, text="🏥 SHOULDER FLEXION THERAPY", 
                     font=('Segoe UI', 18, 'bold'), bg='#2c3e50', fg='white', pady=14)
    header.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 5))
    
    subtitle = tk.Label(main_frame, text="Adaptive Assistance with Graded Support", 
                       font=('Segoe UI', 12, 'italic'), bg='#34495e', fg='#ecf0f1', pady=7)
    subtitle.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(0, 15))
    
    # Configure grid columns
    main_frame.grid_columnconfigure(0, weight=1, uniform="col")
    main_frame.grid_columnconfigure(1, weight=1, uniform="col")
    
    # Variables
    pos_var = tk.StringVar(value="--°")
    rep_var = tk.StringVar(value="0")
    trial_var = tk.StringVar(value="0")
    status_var = tk.StringVar(value="Waiting")
    progress_var = tk.StringVar(value="Trial Progress: 0/3")
    current_peak_var = tk.StringVar(value="0.00 N")
    baseline_var = tk.StringVar(value="")
    assist_var = tk.StringVar(value="Baseline Calibration")
    assist_force_var = tk.StringVar(value="0.00 N")
    current_assist_percent_var = tk.StringVar(value="0%")
    
    fx_var = tk.StringVar(value="0.00")
    fy_var = tk.StringVar(value="0.00")
    fz_var = tk.StringVar(value="0.00")
    
    raw_fz_var = tk.StringVar(value="0.00 N")
    assist_fz_var = tk.StringVar(value="0.00 N")
    effective_fz_var = tk.StringVar(value="0.00 N")
    
    # =================== STATUS PANEL ===================
    status_frame = tk.Frame(main_frame, bg='#3498db', relief=tk.RAISED, bd=2)
    status_frame.grid(row=2, column=0, columnspan=2, sticky='ew', pady=(0, 15))
    
    tk.Label(status_frame, text="⚡ SYSTEM STATUS", font=('Segoe UI', 13, 'bold'),
             bg='#3498db', fg='white').pack(pady=(8, 0))
    
    status_label = tk.Label(status_frame, textvariable=status_var, 
                           font=('Segoe UI', 16, 'bold'), bg='#3498db', fg='white')
    status_label.pack(pady=8)
    
    # =================== TWO-COLUMN: METRICS + PROGRESS ===================
    # --- Performance Metrics (LEFT) ---
    metrics_frame = tk.Frame(main_frame, bg='#ecf0f1', relief=tk.RAISED, bd=2)
    metrics_frame.grid(row=3, column=0, sticky='nsew', padx=(0, 10), pady=(0, 15))
    
    tk.Label(metrics_frame, text="📊 PERFORMANCE METRICS", 
             font=('Segoe UI', 13, 'bold'), bg='#34495e', fg='white').pack(fill=tk.X)
    
    metrics_grid = tk.Frame(metrics_frame, bg='#ecf0f1')
    metrics_grid.pack(fill=tk.X, padx=20, pady=15)
    
    tk.Label(metrics_grid, text="Joint 3 Position:", font=('Segoe UI', 16, 'bold'),
             bg='#ecf0f1', anchor='e').grid(row=0, column=0, sticky='e', padx=20, pady=15)
    pos_label = tk.Label(metrics_grid, textvariable=pos_var, font=('Segoe UI', 14, 'bold'),
                        bg='white', fg='#2c3e50', width=14, relief=tk.SUNKEN, bd=2)
    pos_label.grid(row=0, column=1, sticky='w', padx=20, pady=15)
    
    tk.Label(metrics_grid, text="Repetitions:", font=('Segoe UI', 16, 'bold'),
             bg='#ecf0f1', anchor='e').grid(row=1, column=0, sticky='e', padx=20, pady=15)
    rep_label = tk.Label(metrics_grid, textvariable=rep_var, font=('Segoe UI', 14, 'bold'),
                        bg='white', fg='#27ae60', width=14, relief=tk.SUNKEN, bd=2)
    rep_label.grid(row=1, column=1, sticky='w', padx=20, pady=15)
    
    tk.Label(metrics_grid, text="Trials Completed:", font=('Segoe UI', 16, 'bold'),
             bg='#ecf0f1', anchor='e').grid(row=2, column=0, sticky='e', padx=20, pady=15)
    trial_label = tk.Label(metrics_grid, textvariable=trial_var, font=('Segoe UI', 14, 'bold'),
                          bg='white', fg='#e74c3c', width=14, relief=tk.SUNKEN, bd=2)
    trial_label.grid(row=2, column=1, sticky='w', padx=20, pady=15)
    
    # --- Trial Progress (RIGHT) ---
    progress_frame = tk.Frame(main_frame, bg='#e8f4f8', relief=tk.RAISED, bd=2)
    progress_frame.grid(row=3, column=1, sticky='nsew', padx=(5, 0), pady=(0, 15))
    
    tk.Label(progress_frame, text="🎯 TRIAL PROGRESS", 
             font=('Segoe UI', 13, 'bold'), bg='#16a085', fg='white').pack(fill=tk.X)
    
    tk.Label(progress_frame, textvariable=progress_var, font=('Segoe UI', 16, 'bold'),
             bg='#e8f4f8', fg='#16a085').pack(pady=10)
    
    tk.Label(progress_frame, text="Current Trial Peak Fz:", font=('Segoe UI', 16, 'bold'),
             bg='#e8f4f8').pack()
    tk.Label(progress_frame, textvariable=current_peak_var, font=('Segoe UI', 20, 'bold'),
             bg='#e8f4f8', fg='#c0392b').pack(pady=(0, 12))
    
    # =================== FULL-WIDTH: TRIAL HISTORY ===================
    trials_outer = tk.Frame(main_frame, bg='#fff8dc', relief=tk.RAISED, bd=2)
    trials_outer.grid(row=4, column=0, columnspan=2, sticky='nsew', pady=(0, 15))
    main_frame.grid_rowconfigure(4, weight=1)  # Allow expansion
    
    tk.Label(trials_outer, text="📋 TRIAL HISTORY", 
             font=('Segoe UI', 16, 'bold'), bg='#d35400', fg='white').pack(fill=tk.X)
    
    trials_frame = tk.Frame(trials_outer, bg='#fff8dc')
    trials_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=15)
    
    scrollbar = tk.Scrollbar(trials_frame)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    
    trials_listbox = tk.Listbox(trials_frame, yscrollcommand=scrollbar.set, 
                                font=('Consolas', 12), height=8, bg='#fffef0',
                                fg='#2c3e50', selectbackground='#f39c12', bd=1)
    trials_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    scrollbar.config(command=trials_listbox.yview)
    
    # =================== TWO-COLUMN: ASSISTANCE + FORCE COMPONENTS ===================
    # --- Assistance System (LEFT) ---
    assist_frame = tk.Frame(main_frame, bg='#d5f4e6', relief=tk.RAISED, bd=2)
    assist_frame.grid(row=5, column=0, sticky='ew', padx=(0, 5), pady=(0, 15))
    
    tk.Label(assist_frame, text="🤖 ASSISTANCE SYSTEM", 
             font=('Segoe UI', 13, 'bold'), bg='#27ae60', fg='white').pack(fill=tk.X)
    
    baseline_label = tk.Label(assist_frame, textvariable=baseline_var, 
                             font=('Segoe UI', 15, 'italic'), bg='#d5f4e6', fg='#16a085')
    baseline_label.pack(pady=(8, 4))
    
    assist_mode_label = tk.Label(assist_frame, textvariable=assist_var, 
                                font=('Segoe UI', 15, 'bold'), bg='#d5f4e6', fg='#27ae60')
    assist_mode_label.pack(pady=4)
    
    assist_percent_label = tk.Label(assist_frame, textvariable=current_assist_percent_var, 
                                   font=('Segoe UI', 15, 'bold'), bg='#d5f4e6', fg='#16a085')
    assist_percent_label.pack(pady=4)
    
    assist_force_label = tk.Label(assist_frame, textvariable=assist_force_var, 
                                 font=('Segoe UI', 15, 'bold'), bg='#d5f4e6', fg='#2980b9')
    assist_force_label.pack(pady=(4, 12))
    
    # --- Force Components (RIGHT) ---
    force_comp_frame = tk.Frame(main_frame, bg='#fef5e7', relief=tk.RAISED, bd=2)
    force_comp_frame.grid(row=5, column=1, sticky='ew', padx=(5, 0), pady=(0, 15))
    
    tk.Label(force_comp_frame, text="⚙️ FORCE COMPONENTS (Motion Control)", 
             font=('Segoe UI', 15, 'bold'), bg='#f39c12', fg='white').pack(fill=tk.X)
    
    force_grid = tk.Frame(force_comp_frame, bg='#fef5e7')
    force_grid.pack(fill=tk.X, padx=20, pady=12)
    
    tk.Label(force_grid, text="Raw Fz:", font=('Segoe UI', 15, 'bold'),
             bg='#fef5e7', anchor='e', width=18).grid(row=0, column=0, sticky='e', pady=5)
    tk.Label(force_grid, textvariable=raw_fz_var, font=('Segoe UI', 12),
             bg='white', fg='#8e44ad', width=18, relief=tk.SUNKEN, bd=1).grid(row=0, column=1, padx=10, pady=5)
    
    tk.Label(force_grid, text="Assist Fz:", font=('Segoe UI', 15, 'bold'),
             bg='#fef5e7', anchor='e', width=18).grid(row=1, column=0, sticky='e', pady=5)
    tk.Label(force_grid, textvariable=assist_fz_var, font=('Segoe UI', 12),
             bg='white', fg='#27ae60', width=18, relief=tk.SUNKEN, bd=1).grid(row=1, column=1, padx=10, pady=5)
    
    tk.Label(force_grid, text="Effective Fz:", font=('Segoe UI', 15, 'bold'),
             bg='#fef5e7', anchor='e', width=18).grid(row=2, column=0, sticky='e', pady=5)
    tk.Label(force_grid, textvariable=effective_fz_var, font=('Segoe UI', 12),
             bg='white', fg='#c0392b', width=18, relief=tk.SUNKEN, bd=1).grid(row=2, column=1, padx=10, pady=5)
    
    # =================== FULL-WIDTH: SENSOR READINGS ===================
    forces_frame = tk.Frame(main_frame, bg='#ebf5fb', relief=tk.RAISED, bd=2)
    forces_frame.grid(row=6, column=0, columnspan=2, sticky='ew', pady=(0, 10))
    
    tk.Label(forces_frame, text="🔧 SENSOR READINGS", 
             font=('Segoe UI', 13, 'bold'), bg='#2980b9', fg='white').pack(fill=tk.X)
    
    sensor_grid = tk.Frame(forces_frame, bg='#ebf5fb')
    sensor_grid.pack(pady=15)
    
    for i, (label, var, color) in enumerate([('Fx:', fx_var, '#e74c3c'), 
                                              ('Fy:', fy_var, '#3498db'), 
                                              ('Fz:', fz_var, '#27ae60')]):
        frame = tk.Frame(sensor_grid, bg='white', relief=tk.RAISED, bd=2)
        frame.grid(row=0, column=i, padx=12, pady=8)
        
        tk.Label(frame, text=label, font=('Segoe UI', 12, 'bold'),
                bg='white', fg='#34495e').pack(side=tk.LEFT, padx=(10, 5))
        tk.Label(frame, textvariable=var, font=('Segoe UI', 13, 'bold'),
                bg='white', fg=color, width=9).pack(side=tk.LEFT, padx=(5, 10))
        tk.Label(frame, text="N", font=('Segoe UI', 12),
                bg='white', fg='#7f8c8d').pack(side=tk.LEFT, padx=(0, 10))
    
    # Configure remaining rows to not expand
    for r in [0,1,2,3,5,6]:
        main_frame.grid_rowconfigure(r, weight=0)
    
    def refresh():
        # Update all values
        pos_var.set(f"{gui_data['joint3_pos']:.1f}°")
        rep_var.set(f"{gui_data['rep_count']}")
        trial_var.set(f"{gui_data['trial_count']}")
        
        if gui_data["active"]:
            status_var.set("● ACTIVE")
            status_label.config(bg='#27ae60')
            status_frame.config(bg='#27ae60')
        else:
            status_var.set("○ IDLE")
            status_label.config(bg='#95a5a6')
            status_frame.config(bg='#95a5a6')
        
        progress_var.set(gui_data["progress_text"])
        current_peak_var.set(f"{gui_data['current_trial_peak']:.2f} N")
        baseline_var.set(gui_data["baseline_text"])
        assist_var.set(gui_data["assistance_text"])
        assist_force_var.set(gui_data["assist_force_display"])
        current_assist_percent_var.set(gui_data["current_assist_percent_display"])
        
        fx_var.set(f"{gui_data['fx']:.2f}")
        fy_var.set(f"{gui_data['fy']:.2f}")
        fz_var.set(f"{gui_data['fz']:.2f}")
        
        raw_fz_var.set(gui_data["raw_fz_display"])
        assist_fz_var.set(gui_data["assist_fz_display"])
        effective_fz_var.set(gui_data["effective_fz_display"])
        
        trials_listbox.delete(0, tk.END)
        for trial_text in gui_data["all_trials_display"]:
            trials_listbox.insert(tk.END, trial_text)
        if gui_data["all_trials_display"]:
            trials_listbox.see(tk.END)
        
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
print("✅ Therapy STARTED – Adaptive Graded Assistance Mode")

# --- MAIN LOOP ---
def control_loop():
    global desired_pos, velocity, home_pos, baseline_forces
    global rep_count, trial_count, state
    global current_trial_max_fz, tracking_active
    global f_baseline, next_trial_assist_percent
    global baseline_trial_forces, all_trial_peaks
    global trial_start_time, trial_durations

    current_trial_assist_percent = 0.0

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

        raw_fz = forces[2]
        gui_data["fx"] = forces[0]
        gui_data["fy"] = forces[1]
        gui_data["fz"] = raw_fz

        # Apply assistance as a percentage of the average baseline force
        assist_force_applied = 0.0
        current_trial_assist_percent = next_trial_assist_percent
        if current_trial_assist_percent > 0 and f_baseline is not None and f_baseline > 1.0:
            assist_force_applied = current_trial_assist_percent * f_baseline
            # Match the sign with current raw_fz direction
            if raw_fz < 0:
                assist_force_applied = -assist_force_applied

        gui_data["assist_force_display"] = f"Assist Force: {assist_force_applied:.2f} N"
        gui_data["current_assist_percent_display"] = f"Assist Level: {int(current_trial_assist_percent * 100)}%"

        effective_fz = raw_fz + assist_force_applied
        gui_data["raw_fz_display"] = f"Raw Fz: {raw_fz:.2f} N"
        gui_data["assist_fz_display"] = f"Assist Fz: {assist_force_applied:.2f} N"
        gui_data["effective_fz_display"] = f"Effective Fz: {effective_fz:.2f} N"

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

        for lock_j in range(6):
            if lock_j != j:
                desired_pos[lock_j] = home_pos[lock_j]
                velocity[lock_j] = 0.0

        desired_pos = enforce_joint_limits(desired_pos)

        err, curr_pos = robot.GetActualJointPosDegree()
        if err == 0:
            j3_pos = curr_pos[2]
            gui_data["joint3_pos"] = j3_pos
            gui_data["active"] = (abs(raw_fz) >= 1.0)

            if tracking_active and state == "top":
                if abs(raw_fz) > abs(current_trial_max_fz):
                    current_trial_max_fz = raw_fz
            gui_data["current_trial_peak"] = abs(current_trial_max_fz)

            if state == "bottom" and j3_pos <= TOP_POS + REP_TOL:
                rep_count += 1
                state = "top"
                trial_start_time = time.time()
                if trial_count < MAX_BASELINE_TRIALS or f_baseline is not None:
                    tracking_active = True
                    current_trial_max_fz = 0.0

            elif state == "top" and j3_pos >= BOTTOM_POS - REP_TOL:
                trial_count += 1
                state = "bottom"
                tracking_active = False
                current_peak_abs = abs(current_trial_max_fz)
                
                # Calculate trial duration (complete cycle from bottom to top to bottom)
                if trial_start_time is not None:
                    trial_duration = time.time() - trial_start_time
                    trial_durations.append(trial_duration)
                else:
                    trial_duration = 0.0

                if f_baseline is None and trial_count <= MAX_BASELINE_TRIALS:
                    baseline_trial_forces[trial_count - 1] = current_peak_abs
                    all_trial_peaks.append(current_peak_abs)
                    
                    duration_str = f" | Duration: {trial_duration:.1f}s" if trial_duration > 0 else ""
                    trial_display = f"Trial {trial_count}: {current_peak_abs:.2f} N (Baseline){duration_str}"
                    gui_data["all_trials_display"].append(trial_display)
                    
                    gui_data["progress_text"] = f"Trial Progress: {trial_count}/{MAX_BASELINE_TRIALS}"

                    if trial_count == MAX_BASELINE_TRIALS:
                        f_baseline = sum(baseline_trial_forces) / len(baseline_trial_forces)
                        gui_data["baseline_text"] = f"✅ Baseline Avg: {f_baseline:.2f} N"
                        gui_data["assistance_text"] = "Mode: Adaptive Assistance Starting..."
                        next_trial_assist_percent = 0.0
                        print(f"✅ Baseline Complete: Avg {f_baseline:.2f} N")
                    
                    # Set trial start time for next trial
                    trial_start_time = time.time()

                elif f_baseline is not None:
                    all_trial_peaks.append(current_peak_abs)
                    ratio = current_peak_abs / f_baseline if f_baseline > 0 else 0.0
                    
                    duration_str = f" | Duration: {trial_duration:.1f}s" if trial_duration > 0 else ""
                    
                    if ratio >= 0.50:
                        next_trial_assist_percent = 0.0
                        assist_label = "No Assistance"
                        icon = ""
                    elif ratio >= 0.40 and ratio < 0.50:
                        next_trial_assist_percent = 0.20
                        assist_label = "20% Assist"
                        icon = "🔵"
                    elif ratio >= 0.30 and ratio < 0.40:
                        next_trial_assist_percent = 0.25
                        assist_label = "25% Assist"
                        icon = "🔵"
                    elif ratio >= 0.20 and ratio < 0.30:
                        next_trial_assist_percent = 0.30
                        assist_label = "30% Assist"
                        icon = "🔵"
                    elif ratio >= 0.10 and ratio < 0.20:
                        next_trial_assist_percent = 0.40
                        assist_label = "40% Assist"
                        icon = "🟠"
                    else:  # ratio < 0.20 means less than 20%
                        next_trial_assist_percent = 1.0  # Full passive mode
                        assist_label = "PASSIVE MODE"
                        icon = "🔴"

                    trial_display = f"Trial {trial_count}: {current_peak_abs:.2f} N ({ratio*100:.0f}% of baseline){duration_str}"
                    
                    if ratio < 0.20:
                        trial_display += f" → {assist_label} next"
                        gui_data["assistance_text"] = f"Mode: {icon} {assist_label} in NEXT Trial"
                        print(f"🔴 Trial {trial_count}: PASSIVE MODE - Peak = {current_peak_abs:.2f} N ({ratio*100:.1f}%) | Duration: {trial_duration:.1f}s")
                    elif ratio < 0.50:
                        trial_display += f" → {assist_label} next"
                        gui_data["assistance_text"] = f"Mode: {icon} {assist_label} in NEXT Trial"
                        print(f"🔄 Trial {trial_count}: Peak = {current_peak_abs:.2f} N ({ratio*100:.1f}%) | Duration: {trial_duration:.1f}s → {assist_label} next trial")
                    else:
                        gui_data["assistance_text"] = "Mode: Monitoring (No assistance)"
                        print(f"✅ Trial {trial_count}: Peak = {current_peak_abs:.2f} N ({ratio*100:.1f}%) | Duration: {trial_duration:.1f}s - No assistance needed")
                    
                    gui_data["all_trials_display"].append(trial_display)
                    # Set trial start time for next trial
                    trial_start_time = time.time()

            gui_data["rep_count"] = rep_count
            gui_data["trial_count"] = trial_count

        robot.ServoJ(joint_pos=desired_pos, axisPos=[0]*6)
        time.sleep(dt)

# Start control loop
control_loop()