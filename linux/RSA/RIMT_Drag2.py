# RIMT_Drag2.py – FINAL VERSION WITH SCROLLABLE TRIALS + SMART RFD DISPLAY
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
from datetime import datetime
import time
import csv
import os
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# === Predefined Axis & Images ===
EXERCISE_AXES = {
    "Hip Flexion": "Fz", "Hip Extension": "Fz", "Hip Abduction": "Fz", "Hip Adduction": "Fz",
    "Shoulder Flexion": "Fz", "Shoulder Extension": "Fz", "Shoulder Abduction": "Fz", "Shoulder Adduction": "Fz",
    "Elbow Flexion": "Fz", "Elbow Extension": "Fz", "Wrist Flexion": "Fz", "Wrist Extension": "Fz",
    "Ankle Plantarflexion": "Fz", "Ankle Dorsiflexion": "Fz"
}

EXERCISE_IMAGES = {
    "Hip Flexion": "/home/um/fairino-python-sdk-main/linux/RSA/images/hip_flexion.jpeg",
    "Hip Extension": "images/hip_abduction.jpeg",
    "Hip Abduction": "/home/um/fairino-python-sdk-main/linux/RSA/images/hip_abduction.jpeg",
    "Hip Adduction": "images/hip_adduction.png",
    "Shoulder Flexion": "/home/um/fairino-python-sdk-main/linux/RSA/images/shoulder_flexion.jpeg",
    "Shoulder Extension": "/home/um/fairino-python-sdk-main/linux/RSA/images/shoulder_extension.jpeg",
    "Shoulder Abduction": "/home/um/fairino-python-sdk-main/linux/RSA/images/shoulder_abduction.jpeg",
    "Shoulder Adduction": "/home/um/fairino-python-sdk-main/linux/RSA/images/shoulder_adduction.jpeg",
    "Elbow Flexion": "/home/um/fairino-python-sdk-main/linux/RSA/images/elbow_flexion.jpeg",
    "Elbow Extension": "/home/um/fairino-python-sdk-main/linux/RSA/images/elbow_extension.jpeg",
    "Wrist Flexion": "images/wrist_flexion.png",
    "Wrist Extension": "images/wrist_extension.png",
    "Ankle Plantarflexion": "images/ankle_plantarflexion.png",
    "Ankle Dorsiflexion": "images/ankle_dorsiflexion.png"
}

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Rehabilitation Control System")
        self.root.configure(bg='#f5f6fa')
        self.robot = None
        self.baseline_forces = None
        self.monitoring_active = False
        self.monitoring_thread = None
        self.drag_mode_active = False
        self.current_robot_joints = None

        self.patient_name = tk.StringVar()
        self.patient_age = tk.StringVar()
        self.patient_gender = tk.StringVar()
        self.patient_diagnosis = tk.StringVar()

        self.pages_container = None
        self.frames = {}
        self.exercise_pages = {}
        self.current_page = None

        os.makedirs("images", exist_ok=True)

        self.pages_container = tk.Frame(self.root, bg='#f5f6fa')
        self.pages_container.pack(fill='both', expand=True)

        self._setup_pages()

        # BIGGER WINDOW FOR MULTIPLE GRAPHS
        self.root.geometry("1400x900")
        self.root.minsize(1200, 800)
        self.root.resizable(True, True)
        self.root.eval('tk::PlaceWindow . center')

        self.log_message("System initialized. Ready to connect.")
        self.show_page("dashboard")

    def _setup_pages(self):
        dashboard = DashboardPage(self.pages_container, self)
        self.frames["dashboard"] = dashboard
        dashboard.grid(row=0, column=0, sticky='nsew')

    def show_page(self, page_name, exercise_name=None):
        if page_name == "exercise" and exercise_name:
            if exercise_name not in self.exercise_pages:
                page = ExerciseTrialPage(self.pages_container, self, exercise_name)
                self.exercise_pages[exercise_name] = page
            else:
                page = self.exercise_pages[exercise_name]
            page.grid(row=0, column=0, sticky='nsew')
            frame = page
        else:
            if page_name == "progress_report" and page_name not in self.frames:
                progress = ProgressReportPage(self.pages_container, self)
                self.frames["progress_report"] = progress
                progress.grid(row=0, column=0, sticky='nsew')
            frame = self.frames.get(page_name)

        if not frame:
            return

        if self.current_page:
            old = self.current_page
            new = frame
            new.place(relx=1.0, rely=0, relwidth=1.0, relheight=1.0)
            old.place(relx=0, rely=0, relwidth=1.0, relheight=1.0)
            self._animate_slide(old, new, direction=-1.0)
        else:
            frame.place(relx=0, rely=0, relwidth=1.0, relheight=1.0)

        self.current_page = frame

    def _animate_slide(self, old_frame, new_frame, direction):
        step = 0.06
        pos = 0.0
        def slide():
            nonlocal pos
            pos += step
            old_x = -pos * direction
            new_x = 1.0 - pos * direction
            old_frame.place(relx=old_x)
            new_frame.place(relx=new_x)
            if pos < 1.0:
                self.root.after(10, slide)
            else:
                old_frame.place_forget()
                new_frame.place(relx=0, rely=0, relwidth=1.0, relheight=1.0)
        slide()

    def log_message(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] {msg}")

    def toggle_drag_mode(self):
        if not self.drag_mode_active:
            threading.Thread(target=self.enable_drag_mode, daemon=True).start()
        else:
            threading.Thread(target=self.disable_drag_mode, daemon=True).start()

    def enable_drag_mode(self):
        try:
            if not self.robot:
                self.log_message("Connecting to robot...")
                self.robot = Robot.RPC('192.168.58.2')
                self.log_message("Robot connected.")
            if not self.init_ft_sensor():
                messagebox.showerror("Sensor Error", "Failed to initialize sensor.")
                return
            if not self.calibrate_baseline_forces():
                self.log_message("Baseline skipped.")
            ret = self.robot.DragTeachSwitch(1)
            if ret != 0:
                messagebox.showerror("Error", f"DragTeachSwitch failed: {ret}")
                return
            self.drag_mode_active = True
            if not self.monitoring_active:
                self.start_ft_monitoring()
            self.log_message("FREE-DRIVE ACTIVE")
            self._update_drag_ui(active=True)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def disable_drag_mode(self):
        try:
            if self.robot:
                self.robot.DragTeachSwitch(0)
                self.drag_mode_active = False
                self._update_drag_ui(active=False)
                self.log_message("Drag mode disabled")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _update_drag_ui(self, active):
        for page in [self.frames["dashboard"]] + list(self.exercise_pages.values()):
            if hasattr(page, "update_drag_status"):
                page.update_drag_status(active)

    def init_ft_sensor(self):
        try:
            self.robot.FT_SetConfig(24, 0)
            self.robot.FT_Activate(0); time.sleep(0.5)
            self.robot.FT_Activate(1); time.sleep(0.5)
            self.robot.SetLoadWeight(0, 0.0)
            self.robot.SetLoadCoord(0, 0, 0)
            self.robot.FT_SetZero(0); time.sleep(0.5)
            self.robot.FT_SetZero(1); time.sleep(0.5)
            return True
        except:
            return False

    def calibrate_baseline_forces(self):
        samples = []
        for _ in range(100):
            try:
                d = self.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    samples.append([d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]])
                time.sleep(0.01)
            except: pass
        if samples:
            self.baseline_forces = [sum(col)/len(samples) for col in zip(*samples)]
            return True
        return False

    def start_ft_monitoring(self):
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self.ft_monitoring_loop, daemon=True)
        self.monitoring_thread.start()

    def ft_monitoring_loop(self):
        while self.monitoring_active:
            try:
                d = self.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                    forces = raw if not self.baseline_forces else [raw[i] - self.baseline_forces[i] for i in range(6)]
                    for i in range(6):
                        if abs(forces[i]) < 0.5: forces[i] = 0.0
                time.sleep(0.008)
            except: time.sleep(0.008)

    def get_patient_data(self):
        return {'name': self.patient_name, 'age': self.patient_age, 'gender': self.patient_gender, 'diagnosis': self.patient_diagnosis}

    def on_closing(self):
        self.monitoring_active = False
        if self.drag_mode_active and self.robot:
            try: self.robot.DragTeachSwitch(0)
            except: pass
        self.root.destroy()


# ====================== EXERCISE TRIAL PAGE – UPGRADED ======================
class ExerciseTrialPage(tk.Frame):
    def __init__(self, parent, controller, exercise_name):
        super().__init__(parent, bg='#f5f6fa')
        self.controller = controller
        self.exercise_name = exercise_name
        self.patient_data = controller.get_patient_data()
        self.baseline_forces = controller.baseline_forces

        self.mode = tk.StringVar(value="isometric")
        self.current_trial = 1
        self.max_trials = 1
        self.trial_results = {}
        self.trial_axis = EXERCISE_AXES.get(exercise_name, "Fz")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.image_path = EXERCISE_IMAGES.get(exercise_name)
        self.photo = None
        self.image_label = None
        self.btn_reset = None  # ADD THIS LINE
        self.graphs_container = None  # Scrollable area for all graphs
        self.canvas = None
        self.scrollbar = None
        self.inner_frame = None

        self._create_widgets()
        controller.log_message(f"Exercise: {exercise_name}")

    def _create_widgets(self):
        container = tk.Frame(self, bg='#f5f6fa')
        container.pack(fill='both', expand=True, padx=25, pady=20)

        # Header & Back
        tk.Button(container, text="Back to Dashboard", command=self.go_back,
                  font=('Inter', 10), bg='#34495e', fg='white', relief='flat', padx=20, pady=10).pack(anchor='w')
        hdr = tk.Frame(container, bg='#3498db', height=60)
        hdr.pack(fill='x', pady=(10, 20)); hdr.pack_propagate(False)
        tk.Label(hdr, text=f"{self.exercise_name} | Axis: {self.trial_axis}",
                 font=('Inter', 16, 'bold'), fg='white', bg='#3498db').pack(expand=True)

        # Mode Selection
        mode_frame = tk.Frame(container, bg='#f5f6fa')
        mode_frame.pack(fill='x', pady=(0, 10))

        tk.Label(mode_frame, text="Testing Mode:", font=('Inter', 16, 'bold'), bg='#f5f6fa').pack(side='left', padx=(0, 20))

        # Define a larger font for the radio buttons
        radio_font = ('Inter', 18)        # You can adjust the size: 16, 18, etc.
        # or bold: ('Inter', 16, 'bold')

        tk.Radiobutton(mode_frame, 
                    text="Fixed Isometric Muscle Testing", 
                    variable=self.mode, 
                    value="isometric",
                    command=self.switch_mode, 
                    bg='#f5f6fa',
                    font=radio_font).pack(side='left', padx=(10, 20))

        tk.Radiobutton(mode_frame, 
                    text="RFD Mode (3 Trials)", 
                    variable=self.mode, 
                    value="rfd",
                    command=self.switch_mode, 
                    bg='#f5f6fa',
                    font=radio_font).pack(side='left', padx=(0, 10))
        
        # Image
        if self.image_path and os.path.exists(self.image_path):
            try:
                img = Image.open(self.image_path)
                img.thumbnail((220, 220), Image.Resampling.LANCZOS)
                self.photo = ImageTk.PhotoImage(img)
                tk.Label(container, image=self.photo, bg='white', relief='solid', bd=1).pack(pady=10)
            except: pass

        # Scrollable Graphs Area
        graphs_card = tk.Frame(container, bg='white', relief='solid', bd=1)
        graphs_card.pack(fill='both', expand=True, pady=(10, 0))

        canvas_frame = tk.Frame(graphs_card)
        canvas_frame.pack(fill='both', expand=True, padx=15, pady=15)

        self.canvas = tk.Canvas(canvas_frame, bg='white', highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(canvas_frame, orient="vertical", command=self.canvas.yview)
        self.inner_frame = tk.Frame(self.canvas, bg='white')

        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.canvas.create_window((0, 0), window=self.inner_frame, anchor="nw")
        self.inner_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))

        # Controls
        controls = tk.Frame(container, bg='#f5f6fa')
        controls.pack(fill='x', pady=15)
        self.btn_start = tk.Button(controls, text="Start Trial 1", command=self.start_trial,
                                   font=('Inter', 11, 'bold'), bg='#3498db', fg='white', padx=25, pady=12)
        self.btn_start.pack(side='left', padx=(0, 15))
        # ADD RESET BUTTON
        self.btn_reset = tk.Button(controls, text="Reset Trials", command=self.reset_trials,
                          font=('Inter', 11, 'bold'), bg='#e67e22', fg='white', padx=25, pady=12)
        self.btn_reset.pack(side='left', padx=(0, 15))
        self.progress_bar = ttk.Progressbar(controls, variable=self.progress_var, maximum=100, length=400)
        self.progress_bar.pack(side='left')
        self.btn_save = tk.Button(controls, text="Save to CSV", command=self.save_to_csv,
                                  font=('Inter', 11, 'bold'), bg='#27ae60', fg='white', padx=25, pady=12, state='disabled')
        self.btn_save.pack(side='right')

        self.switch_mode()

    def switch_mode(self):
        mode = self.mode.get()
        self.max_trials = 1 if mode == "isometric" else 3
        self.current_trial = 1
        self.trial_results.clear()
        for widget in self.inner_frame.winfo_children():
            widget.destroy()
        self.update_buttons()

    # ADD this new method after switch_mode
    def reset_trials(self):
        """Reset all trials and clear graphs"""
        self.current_trial = 1
        self.trial_results.clear()
        for widget in self.inner_frame.winfo_children():
            widget.destroy()
        self.progress_var.set(0)
        self.update_buttons()
        self.controller.log_message(f"{self.exercise_name} trials reset")

    def update_buttons(self):
        has_all = len(self.trial_results) == self.max_trials
        self.btn_start.config(state='normal' if self.controller.monitoring_active and self.current_trial not in self.trial_results else 'disabled')
        self.btn_save.config(state='normal' if has_all else 'disabled')

    def start_trial(self):
        if not self.controller.monitoring_active:
            messagebox.showwarning("Warning", "Enable Drag Mode first!")
            return
        idx = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"].index(self.trial_axis)
        self.progress_var.set(0)
        self.btn_start.config(state='disabled')
        threading.Thread(target=self.run_trial, args=(idx,), daemon=True).start()

        # REPLACE the run_trial method's end section (around line 280) with:
    def run_trial(self, idx):
        total = int(5.0 / 0.008)
        samples = []
        times = []
        start_t = time.time()
        for i in range(total):
            try:
                d = self.controller.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                    forces = raw if not self.baseline_forces else [raw[j] - self.baseline_forces[j] for j in range(6)]
                    val = forces[idx]
                    if self.trial_axis == "Fz": val = -val
                    if abs(val) < 0.5: val = 0.0
                    samples.append(val)
                    times.append(time.time() - start_t)
                    print(f"Time: {times[-1]:.3f}s | Fz: {val:.2f} N")
                self.after(0, self.progress_var.set, (i+1)/total*100)
                time.sleep(0.008)
            except:
                break

        valid = [s for s in samples if s > 0.5]
        avg = sum(valid)/len(valid) if valid else 0
        peak = max(valid) if valid else 0
        
        # Get all RFD metrics
        rfd_metrics = self.calculate_peak_rfd(times, samples)

        self.trial_results[self.current_trial] = {
            "avg": avg,
            "peak": peak,
            "rfd": rfd_metrics["peak_rfd"],
            "avg_rfd": rfd_metrics["avg_rfd"],
            "early_rfd": rfd_metrics["early_rfd"],
            "peak_to_peak_rfd": rfd_metrics["peak_to_peak_rfd"],
            "data": (times, samples),
            "sidx": rfd_metrics["start_idx"],
            "eidx": rfd_metrics["end_idx"]
        }

        self.after(100, lambda: self.plot_trial_graph(self.current_trial))
        if self.mode.get() == "rfd" and self.current_trial == 3:
            self.after(500, self.plot_rfd_summary)

        if self.current_trial < self.max_trials:
            self.after(800, lambda: setattr(self, 'current_trial', self.current_trial + 1) or self.update_buttons())
        else:
            self.after(800, self.update_buttons)

    # # REPLACE the plot_trial_graph method to arrange graphs side by side:
    # def plot_trial_graph(self, trial_num):
    #     data = self.trial_results[trial_num]
    #     times, forces = data["data"]

    #     # Create a container for side-by-side graphs
    #     if trial_num == 1:
    #         # Create a frame to hold all three graphs horizontally
    #         self.graphs_row = tk.Frame(self.inner_frame, bg='white')
    #         self.graphs_row.pack(fill='x', pady=15, padx=10)

    #     fig = plt.Figure(figsize=(4.5, 4), dpi=100)
    #     ax = fig.add_subplot(111)

    #     if self.mode.get() == "rfd":
    #         # RFD MODE: Show force curve with TWO highlighted RFD segments
    #         ax.plot(times, forces, color='#95a5a6', linewidth=2, label='Force Curve', alpha=0.7)
            
    #         # Find onset index
    #         onset_idx = None
    #         threshold = 0.5
    #         for i in range(len(forces)):
    #             if forces[i] > threshold and all(forces[j] > threshold*0.8 for j in range(i, min(i+10, len(forces)))):
    #                 onset_idx = i
    #                 break
            
    #         if onset_idx is not None:
    #             dt = times[1] - times[0] if len(times) > 1 else 0.008
                
    #             # 1. Early RFD line (50ms to 100ms) - BLUE
    #             idx_50ms = onset_idx + int(0.05 / dt)
    #             idx_100ms = onset_idx + int(0.10 / dt)
    #             if idx_100ms < len(forces) and idx_50ms < len(forces):
    #                 t1_early = times[idx_50ms]
    #                 t2_early = times[idx_100ms]
    #                 f1_early = forces[idx_50ms]
    #                 f2_early = forces[idx_100ms]
    #                 early_rfd = data["early_rfd"]
                    
    #                 ax.plot([t1_early, t2_early], [f1_early, f2_early], 
    #                     color='#3498db', linewidth=4, label=f'Early RFD: {early_rfd:.0f} N/s',
    #                     marker='o', markersize=6)
                
    #             # 2. Peak-to-Peak RFD line (0ms to peak force) - RED
    #             peak_force_idx = onset_idx + np.argmax(forces[onset_idx:])
    #             if peak_force_idx > onset_idx:
    #                 t1_peak = times[onset_idx]
    #                 t2_peak = times[peak_force_idx]
    #                 f1_peak = forces[onset_idx]
    #                 f2_peak = forces[peak_force_idx]
    #                 peak_to_peak_rfd = data["peak_to_peak_rfd"]
                    
    #                 ax.plot([t1_peak, t2_peak], [f1_peak, f2_peak], 
    #                     color='#e74c3c', linewidth=4, label=f'Peak RFD: {peak_to_peak_rfd:.0f} N/s',
    #                     marker='s', markersize=6)
            
    #         ax.set_title(f"Trial {trial_num}", fontsize=12, fontweight='bold')
    #         ax.set_xlabel("Time (s)", fontsize=9)
    #         ax.set_ylabel("Force (N)", fontsize=9)
    #         ax.legend(loc='best', fontsize=8)
    #         ax.grid(True, alpha=0.3)
            
    #     else:
    #         # ISOMETRIC MODE: Full detail
    #         ax.plot(times, forces, color='#e74c3c', linewidth=2.5)
    #         if data["avg"] > 0.5: 
    #             ax.axhline(data["avg"], color='blue', linestyle='--', label=f'Avg: {data["avg"]:.2f} N')
    #         if data["peak"] > 0.5: 
    #             ax.axhline(data["peak"], color='green', linestyle=':', label=f'Peak: {data["peak"]:.2f} N')
    #         ax.set_title(f"Trial {trial_num}", fontsize=12, fontweight='bold')
    #         ax.set_xlabel("Time (s)", fontsize=9)
    #         ax.set_ylabel("Force (N)", fontsize=9)
    #         ax.legend(fontsize=8)
    #         ax.grid(True, alpha=0.3)

    #     fig.tight_layout()

    #     # Pack graphs side by side
    #     canvas = FigureCanvasTkAgg(fig, self.graphs_row if self.mode.get() == "rfd" else self.inner_frame)
    #     canvas.draw()
    #     if self.mode.get() == "rfd":
    #         canvas.get_tk_widget().pack(side='left', padx=5)
    #     else:
    #         canvas.get_tk_widget().pack(fill='x', pady=15, padx=10)


    def plot_trial_graph(self, trial_num):
        data = self.trial_results[trial_num]
        times, forces = data["data"]

        if self.mode.get() == "isometric":
            # === ISOMETRIC MODE – perfect ===
            main_row = tk.Frame(self.inner_frame, bg='white')
            main_row.pack(fill='x', pady=25, padx=15)

            peak_force = data["peak"]
            avg_force = data["avg"]
            start_force = forces[0] if forces else 0.0
            peak_idx = int(np.argmax(forces))
            time_to_peak = times[peak_idx] if len(times) > peak_idx else 0.0

            dt = times[1] - times[0] if len(times) > 1 else 0.008
            idx_3s = min(len(forces) - 1, int(3.0 / dt))
            force_at_3s = forces[idx_3s]

            fig = plt.Figure(figsize=(8, 5), dpi=100)
            ax = fig.add_subplot(111)
            ax.plot(times, forces, color='#e74c3c', linewidth=2.8, label='Force (N)')
            ax.set_title(f"{self.exercise_name} – Isometric Trial", fontsize=15, fontweight='bold')
            ax.set_xlabel("Time (s)", fontsize=12)
            ax.set_ylabel("Force (N)", fontsize=12)
            ax.grid(True, alpha=0.3)

            ax.axhline(peak_force, color='#27ae60', linestyle='--', linewidth=2, alpha=0.8)
            ax.axhline(avg_force, color='#3498db', linestyle='--', linewidth=2, alpha=0.8)
            ax.axvline(3.0, color='gray', linestyle=':', alpha=0.7)
            ax.plot(time_to_peak, peak_force, 'o', color='#27ae60', markersize=10)
            if time_to_peak <= 3.0:
                ax.plot(3.0, force_at_3s, 'o', color='#9b59b6', markersize=10)

            ax.legend(fontsize=10)
            fig.tight_layout()

            canvas = FigureCanvasTkAgg(fig, main_row)
            canvas.draw()
            canvas.get_tk_widget().pack(side='left', padx=(0, 30), pady=10)

            # Summary Card
            summary = tk.Frame(main_row, bg='#f8fff9', relief='solid', bd=2)
            summary.pack(side='right', fill='y', ipadx=25, ipady=20)
            tk.Label(summary, text="ISOMETRIC RESULTS",
                    font=('Inter', 17, 'bold'), fg='#27ae60', bg='#f8fff9').pack(pady=(15, 20))

            def add_row(label, value, unit, color='#2c3e50'):
                f = tk.Frame(summary, bg='white', relief='solid', bd=1)
                f.pack(fill='x', padx=18, pady=7)
                tk.Label(f, text=label, font=('Inter', 11, 'bold'), fg='#2c3e50', bg='white',
                        anchor='w').pack(side='left', padx=15, pady=8)
                
                # Right side container for values
                values_frame = tk.Frame(f, bg='white')
                values_frame.pack(side='right', padx=20, pady=8)
                
                if unit == "N":
                    # Convert N to kg: 1 kg = 9.81 N
                    value_kg = float(value) / 9.81
                    
                    # Display both N and kg
                    tk.Label(values_frame, text=f"{value} N", 
                            font=('Inter', 20, 'bold'),
                            fg=color, bg='white').pack(anchor='e')
                    tk.Label(values_frame, text=f"({value_kg:.2f} kg)", 
                            font=('Inter', 11, 'italic'),
                            fg='#7f8c8d', bg='white').pack(anchor='e', pady=(2, 0))
                else:
                    # For time values (unit == "s")
                    tk.Label(values_frame, text=f"{value} {unit}", 
                            font=('Inter', 22, 'bold'),
                            fg=color, bg='white').pack(anchor='e')

            add_row("Peak Force",         f"{peak_force:.2f}",   "N", "#0EB933")
            add_row("Average Force",      f"{avg_force:.2f}",    "N", "#02070a")
            add_row("Start Force (0 s)",  f"{start_force:.2f}",  "N", "#72222F")
            add_row("Time to Peak",       f"{time_to_peak:.2f}", "s", "#54664C")
            add_row("Force at 3.0 s",     f"{force_at_3s:.2f}",  "N", "#7518a3")

            self.trial_results[trial_num].update({
                "start_force": start_force,
                "time_to_peak": time_to_peak,
                "force_at_3s": force_at_3s
            })

        else:
            # ===================== RFD MODE – PEAK RFD FROM 0.00s TO PEAK =====================
            if trial_num == 1:
                self.graphs_row = tk.Frame(self.inner_frame, bg='white')
                self.graphs_row.pack(fill='x', pady=15, padx=10)

            fig = plt.Figure(figsize=(4.5, 4), dpi=100)
            ax = fig.add_subplot(111)

            # Main force-time curve
            ax.plot(times, forces, color='#95a5a6', linewidth=2.2, alpha=0.9)

            # Find absolute peak in this trial
            peak_idx = int(np.argmax(forces))
            peak_force_val = forces[peak_idx]
            peak_time = times[peak_idx]

            # 1. EARLY RFD: 50–100 ms after onset (still uses onset detection)
            onset_idx = data.get("sidx")
            if onset_idx is not None and onset_idx < len(forces) - 20:
                dt = times[1] - times[0] if len(times) > 1 else 0.008
                idx_50 = onset_idx + int(0.05 / dt)
                idx_100 = onset_idx + int(0.10 / dt)
                if idx_100 < len(forces):
                    early_rfd = data["early_rfd"]
                    early_rfd_kg = early_rfd / 9.81  # Convert to kg/s
                    ax.plot([times[idx_50], times[idx_100]],
                            [forces[idx_50], forces[idx_100]],
                            color='#3498db', linewidth=4.5, marker='o', markersize=7,
                            label=f'Early RFD: {early_rfd:.0f} N/s ({early_rfd_kg:.0f} kg/s)')

            # 2. PEAK RFD: FROM TIME = 0.00s → TO THE HIGHEST PEAK (THIS IS WHAT YOU WANTED)
            # This is independent of onset — always starts at t=0
            if len(forces) > 10:
                peak_to_peak_rfd = data["peak_to_peak_rfd"]  # still use your calculated value
                peak_rfd_kg = peak_to_peak_rfd / 9.81  # Convert to kg/s
                ax.plot([times[0], peak_time],           # ← from 0.00s
                        [forces[0], peak_force_val],     # ← to peak
                        color='#e74c3c', linewidth=5.5, marker='s', markersize=9,
                        label=f'Peak RFD (0→Peak): {peak_to_peak_rfd:.0f} N/s ({peak_rfd_kg:.0f} kg/s)')

            # Styling
            ax.set_title(f"Trial {trial_num}", fontsize=12, fontweight='bold')
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.set_ylabel("Force (N)", fontsize=9)
            ax.legend(loc='upper left', fontsize=7.5)  # Slightly smaller to fit both units
            ax.grid(True, alpha=0.3)
            fig.tight_layout()

            canvas = FigureCanvasTkAgg(fig, self.graphs_row)
            canvas.draw()
            canvas.get_tk_widget().pack(side='left', padx=5)


   # REPLACE the plot_rfd_summary method with simplified version:
    def plot_rfd_summary(self):
        """Display simplified RFD summary with only averages and best values"""
        
        # Create a summary frame
        summary_frame = tk.Frame(self.inner_frame, bg='white', relief='solid', bd=2)
        summary_frame.pack(fill='x', pady=20, padx=10)
        
        # Title
        title_label = tk.Label(summary_frame, 
                            text=f"{self.exercise_name} – RFD Analysis Summary",
                            font=('Inter', 18, 'bold'), 
                            fg='#2c3e50', 
                            bg='white')
        title_label.pack(pady=(20, 15))
        
        # Separator
        tk.Frame(summary_frame, bg='#ecf0f1', height=3).pack(fill='x', padx=20, pady=(0, 20))
        
        # Collect values for averaging
        all_early_rfd = []
        all_peak_rfd = []
        
        for trial_num in range(1, 4):
            r = self.trial_results[trial_num]
            all_early_rfd.append(r['early_rfd'])
            all_peak_rfd.append(r['peak_to_peak_rfd'])
        
        # Calculate averages and best values
        avg_early_rfd = np.mean(all_early_rfd)
        avg_peak_rfd = np.mean(all_peak_rfd)
        best_early = max(all_early_rfd)
        best_peak = max(all_peak_rfd)
        
        # Find which trial had each best value
        trial_early = next(i for i in range(1, 4) if self.trial_results[i]['early_rfd'] == best_early)
        trial_peak = next(i for i in range(1, 4) if self.trial_results[i]['peak_to_peak_rfd'] == best_peak)
        
        # ============ AVERAGES SECTION ============
        avg_frame = tk.Frame(summary_frame, bg='#d4edda', relief='solid', bd=2)
        avg_frame.pack(fill='x', padx=40, pady=(0, 20))
        
        tk.Label(avg_frame, text="AVERAGE VALUES (All 3 Trials)",
                font=('Inter', 14, 'bold'),
                fg='#155724',
                bg='#d4edda').pack(pady=(15, 10))
        
        # Average values in a clean layout
        avg_data_frame = tk.Frame(avg_frame, bg='#d4edda')
        avg_data_frame.pack(pady=(0, 15), padx=20)
        
        # Early RFD Average
        early_frame = tk.Frame(avg_data_frame, bg='#ffffff', relief='solid', bd=1)
        early_frame.pack(side='left', padx=15, pady=5)
        tk.Label(early_frame, text="Average Early RFD\n(50-100ms)",
                font=('Inter', 10, 'bold'),
                fg='#2c3e50',
                bg='#ffffff').pack(pady=(10, 5))
        tk.Label(early_frame, text=f"{avg_early_rfd:.1f} N/s",
                font=('Inter', 20, 'bold'),
                fg='#3498db',
                bg='#ffffff').pack(pady=(0, 10), padx=30)
        
        # Peak RFD Average
        peak_frame = tk.Frame(avg_data_frame, bg='#ffffff', relief='solid', bd=1)
        peak_frame.pack(side='left', padx=15, pady=5)
        tk.Label(peak_frame, text="Average Peak RFD\n(0-Peak)",
                font=('Inter', 10, 'bold'),
                fg='#2c3e50',
                bg='#ffffff').pack(pady=(10, 5))
        tk.Label(peak_frame, text=f"{avg_peak_rfd:.1f} N/s",
                font=('Inter', 20, 'bold'),
                fg='#e74c3c',
                bg='#ffffff').pack(pady=(0, 10), padx=30)
        
        # ============ BEST VALUES SECTION ============
        best_frame = tk.Frame(summary_frame, bg='#fff3cd', relief='solid', bd=2)
        best_frame.pack(fill='x', padx=40, pady=(0, 20))
        
        tk.Label(best_frame, text="BEST VALUES (Across All Trials)",
                font=('Inter', 14, 'bold'),
                fg='#856404',
                bg='#fff3cd').pack(pady=(15, 10))
        
        # Best values in a clean layout
        best_data_frame = tk.Frame(best_frame, bg='#fff3cd')
        best_data_frame.pack(pady=(0, 15), padx=20)
        
        # Best Early RFD
        best_early_frame = tk.Frame(best_data_frame, bg='#ffffff', relief='solid', bd=1)
        best_early_frame.pack(side='left', padx=15, pady=5)
        tk.Label(best_early_frame, text="Best Early RFD\n(50-100ms)",
                font=('Inter', 10, 'bold'),
                fg='#2c3e50',
                bg='#ffffff').pack(pady=(10, 5))
        tk.Label(best_early_frame, text=f"{best_early:.1f} N/s",
                font=('Inter', 20, 'bold'),
                fg='#3498db',
                bg='#ffffff').pack(pady=(0, 5), padx=30)
        tk.Label(best_early_frame, text=f"(Trial {trial_early})",
                font=('Inter', 9, 'italic'),
                fg='#7f8c8d',
                bg='#ffffff').pack(pady=(0, 10))
        
        # Best Peak RFD
        best_peak_frame = tk.Frame(best_data_frame, bg='#ffffff', relief='solid', bd=1)
        best_peak_frame.pack(side='left', padx=15, pady=5)
        tk.Label(best_peak_frame, text="Best Peak RFD\n(0-Peak)",
                font=('Inter', 10, 'bold'),
                fg='#2c3e50',
                bg='#ffffff').pack(pady=(10, 5))
        tk.Label(best_peak_frame, text=f"{best_peak:.1f} N/s",
                font=('Inter', 20, 'bold'),
                fg='#e74c3c',
                bg='#ffffff').pack(pady=(0, 5), padx=30)
        tk.Label(best_peak_frame, text=f"(Trial {trial_peak})",
                font=('Inter', 9, 'italic'),
                fg='#7f8c8d',
                bg='#ffffff').pack(pady=(0, 10))
    
    # REPLACE the calculate_peak_rfd method (around line 350) with this enhanced version
    def calculate_peak_rfd(self, times, forces, threshold=0.5):
        """Calculate multiple RFD metrics"""
        # Find force onset
        onset_idx = None
        for i in range(len(forces)):
            if forces[i] > threshold and all(forces[j] > threshold*0.8 for j in range(i, min(i+10, len(forces)))):
                onset_idx = i
                break
        if onset_idx is None:
            return {"peak_rfd": 0.0, "avg_rfd": 0.0, "early_rfd": 0.0, "peak_to_peak_rfd": 0.0,
                    "start_idx": None, "end_idx": None}

        dt = times[1] - times[0] if len(times) > 1 else 0.008
        
        # 1. Peak RFD (best slope in any 250ms window)
        best_peak_rfd = 0.0
        best_s = best_e = None
        end_search = min(len(times), onset_idx + int(0.5 / dt))
        for i in range(onset_idx, end_search):
            for j in range(i+5, min(end_search, i + int(0.25/dt))):
                df = forces[j] - forces[i]
                time_diff = times[j] - times[i]
                if df > 0 and time_diff > 0:
                    rfd = df / time_diff
                    if rfd > best_peak_rfd:
                        best_peak_rfd = rfd
                        best_s = i
                        best_e = j
        
        # 2. Early RFD (50-100ms from onset)
        idx_50ms = onset_idx + int(0.05 / dt)
        idx_100ms = onset_idx + int(0.10 / dt)
        early_rfd = 0.0
        if idx_100ms < len(forces) and idx_50ms < len(forces):
            df_early = forces[idx_100ms] - forces[idx_50ms]
            dt_early = times[idx_100ms] - times[idx_50ms]
            early_rfd = df_early / dt_early if dt_early > 0 else 0.0
        
        # 3. Peak-to-Peak RFD (0ms to peak force)
        peak_force_idx = onset_idx + np.argmax(forces[onset_idx:])
        peak_to_peak_rfd = 0.0
        if peak_force_idx > onset_idx:
            df_peak = forces[peak_force_idx] - forces[onset_idx]
            dt_peak = times[peak_force_idx] - times[onset_idx]
            peak_to_peak_rfd = df_peak / dt_peak if dt_peak > 0 else 0.0
        
        # 4. Average RFD (mean of all positive slopes from onset to peak)
        rfd_values = []
        for i in range(onset_idx, peak_force_idx):
            if i+1 < len(forces):
                df = forces[i+1] - forces[i]
                dt_step = times[i+1] - times[i]
                if df > 0 and dt_step > 0:
                    rfd_values.append(df / dt_step)
        avg_rfd = np.mean(rfd_values) if rfd_values else 0.0
        
        return {
            "peak_rfd": best_peak_rfd,
            "avg_rfd": avg_rfd,
            "early_rfd": early_rfd,
            "peak_to_peak_rfd": peak_to_peak_rfd,
            "start_idx": best_s,
            "end_idx": best_e
        }

    # # UPDATE the save_to_csv method to include all RFD metrics (around line 360)
    # def save_to_csv(self):
    #     name = self.patient_data['name'].get().strip()
    #     if not name:
    #         messagebox.showwarning("Missing", "Enter patient name!")
    #         return
    #     base = {
    #         "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    #         "Exercise": self.exercise_name,
    #         "Name": name,
    #         "Mode": "Isometric" if self.mode.get() == "isometric" else "RFD (3 trials)"
    #     }
    #     rows = []
    #     for t in self.trial_results:
    #         r = self.trial_results[t]
    #         row = {
    #             **base,
    #             "Trial": t,
    #             "Average_N": f"{r['avg']:.3f}",
    #             "Peak_N": f"{r['peak']:.3f}",
    #             "Peak_RFD_N/s": f"{r['rfd']:.0f}",
    #             "Average_RFD_N/s": f"{r['avg_rfd']:.0f}",
    #             "Early_RFD_N/s": f"{r['early_rfd']:.0f}",
    #             "PeaktoPeak_RFD_N/s": f"{r['peak_to_peak_rfd']:.0f}"
    #         }
    #         rows.append(row)
        
    #     exists = os.path.isfile("results.csv")
    #     with open("results.csv", "a", newline="", encoding="utf-8") as f:
    #         writer = csv.DictWriter(f, fieldnames=rows[0].keys())
    #         if not exists:
    #             writer.writeheader()
    #         writer.writerows(rows)
    #     messagebox.showinfo("Saved", "Results saved to results.csv!")


    def save_to_csv(self):
        name = self.patient_data['name'].get().strip()
        if not name:
            messagebox.showwarning("Missing", "Enter patient name!")
            return

        base = {
            "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Exercise": self.exercise_name,
            "Name": name,
            "Mode": "Isometric" if self.mode.get() == "isometric" else "RFD (3 trials)"
        }

        rows = []
        for t in self.trial_results:
            r = self.trial_results[t]
            row = {
                **base,
                "Trial": t,
                "Average_N": f"{r['avg']:.3f}",
                "Peak_N": f"{r['peak']:.3f}",
            }

            if self.mode.get() == "isometric":
                row.update({
                    "Start_Force_N": f"{r.get('start_force', 0):.3f}",
                    "Time_to_Peak_s": f"{r.get('time_to_peak', 0):.3f}",
                    "Force_at_3s_N": f"{r.get('force_at_3s', 0):.3f}",
                })
            else:
                row.update({
                    "Peak_RFD_N/s": f"{r['rfd']:.0f}",
                    "Average_RFD_N/s": f"{r['avg_rfd']:.0f}",
                    "Early_RFD_N/s": f"{r['early_rfd']:.0f}",
                    "PeaktoPeak_RFD_N/s": f"{r['peak_to_peak_rfd']:.0f}"
                })
            rows.append(row)

        # Write to CSV (same as before)
        exists = os.path.isfile("results.csv")
        with open("results.csv", "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            if not exists:
                writer.writeheader()
            writer.writerows(rows)

        messagebox.showinfo("Saved", "Results saved to results.csv!")



    def go_back(self):
        self.controller.show_page("dashboard")

    def animate_gif(self):
        if self.gif_img and getattr(self.gif_img, "is_animated", False):
            try:
                self.gif_frame = (self.gif_frame + 1) % self.gif_img.n_frames
                self.gif_img.seek(self.gif_frame)
                self.photo = ImageTk.PhotoImage(self.gif_img)
                self.image_label.config(image=self.photo)
                self.image_label.image = self.photo
                self.root.after(100, self.animate_gif)
            except:
                pass

# Keep your existing DashboardPage, ProgressReportPage, etc. unchanged...
# (They remain exactly as you had them – only ExerciseTrialPage was replaced above)
# ------------------------------------------------------------------------- #
# DASHBOARD PAGE
# ------------------------------------------------------------------------- #
class DashboardPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg='#f5f6fa')
        self.controller = controller
        self._create_widgets()

    def _create_widgets(self):
        content = tk.Frame(self, bg='#f5f6fa')
        content.pack(fill='both', expand=True, padx=30, pady=30)
        self._create_header(content)
        self._create_drag_control(content)
        self._create_patient_info(content)
        self._create_exercise_grid(content)
        self.progress_report = ProgressReportCard(content, self.controller)
        self.progress_report.pack(fill='x', pady=(15, 0))

    def _create_header(self, parent):
        header = tk.Frame(parent, bg='#2c3e50', height=100)
        header.pack(fill='x', pady=(0, 30))
        header.pack_propagate(False)
        title_frame = tk.Frame(header, bg='#2c3e50')
        title_frame.place(relx=0.5, rely=0.5, anchor='center')
        tk.Label(title_frame, text="ROBOT REHABILITATION SYSTEM",
                 font=('Inter', 24, 'bold'), fg='white', bg='#2c3e50').pack()
        tk.Label(title_frame, text="Advanced Drag Mode Control & Exercise Management",
                 font=('Inter', 11), fg='#bdc3c7', bg='#2c3e50').pack(pady=(5, 0))

    def _create_drag_control(self, parent):
        card = tk.Frame(parent, bg='white', relief='solid', bd=1)
        card.pack(fill='x', pady=(0, 20))
        inner = tk.Frame(card, bg='white')
        inner.pack(fill='both', padx=20, pady=20)
        tk.Label(inner, text="Drag Mode Control", font=('Inter', 20, 'bold'), fg="#3a4c5e", bg='white', anchor='w').pack(fill='x', pady=(0, 15))
        tk.Frame(inner, bg='#ecf0f1', height=2).pack(fill='x', pady=(0, 15))
        status = tk.Frame(inner, bg='white')
        status.pack(fill='x', pady=(0, 15))
        emoji_font = ('Segoe UI Emoji', 26) if sys.platform.startswith('win') else \
                     ('Apple Color Emoji', 26) if sys.platform == 'darwin' else ('Noto Color Emoji', 26)
        self.status_indicator = tk.Label(status, text="Circle", font=emoji_font, fg='#27ae60', bg='white')
        self.status_indicator.pack(side='left', padx=(0, 10))
        txt = tk.Frame(status, bg='white')
        txt.pack(side='left', fill='x', expand=True)
        self.drag_status = tk.Label(txt, text="Status: Disconnected", font=('Inter', 12, 'bold'), fg='#2c3e50', bg='white', anchor='w')
        self.drag_status.pack(anchor='w')
        self.status_label = tk.Label(txt, text="Press 'Enable Drag Mode' to connect", font=('Inter', 9), fg='#7f8c8d', bg='white', anchor='w')
        self.status_label.pack(anchor='w')
        btn_row = tk.Frame(inner, bg='white')
        btn_row.pack(fill='x', pady=(10, 0))
        self.btn_drag_mode = tk.Button(btn_row, text="Enable Drag Mode", command=self.controller.toggle_drag_mode,
                                       font=('Inter', 12, 'bold'), bg='#3498db', fg='white', relief='flat', cursor='hand2', padx=30, pady=15)
        self.btn_drag_mode.pack(side='left', padx=(0, 12))
        self.btn_progress = tk.Button(btn_row, text="Progress Report",
                                      command=lambda: self.controller.show_page("progress_report"),
                                      font=('Inter', 12, 'bold'), bg='#27ae60', fg='white', relief='flat', cursor='hand2', padx=30, pady=15)
        self.btn_progress.pack(side='left')

    def _create_patient_info(self, parent):
        card = tk.Frame(parent, bg='white', relief='solid', bd=1)
        card.pack(fill='x', pady=(0, 20))
        inner = tk.Frame(card, bg='white')
        inner.pack(fill='both', padx=20, pady=20)
        tk.Label(inner, text="Patient Information", font=('Inter', 20, 'bold'), fg='#2c3e50', bg='white', anchor='w').pack(fill='x', pady=(0, 15))
        tk.Frame(inner, bg='#ecf0f1', height=2).pack(fill='x', pady=(0, 15))
        fields = [
            ("Full Name:", self.controller.patient_name, 'entry', None),
            ("Age:", self.controller.patient_age, 'entry', None),
            ("Gender:", self.controller.patient_gender, 'combo', ["Male", "Female", "Other"]),
            ("Diagnosis:", self.controller.patient_diagnosis, 'entry', None)
        ]
        for txt, var, wtype, vals in fields:
            row = tk.Frame(inner, bg='white')
            row.pack(fill='x', pady=8)
            tk.Label(row, text=txt, font=('Inter', 10, 'bold'), fg='#2c3e50', bg='white', width=12, anchor='w').pack(side='left', padx=(0, 15))
            if wtype == 'entry':
                tk.Entry(row, textvariable=var, font=('Inter', 10), bg='#ecf0f1', fg="#252e38", relief='flat', width=50).pack(side='left', fill='x', expand=True, ipady=8, ipadx=10)
            else:
                ttk.Combobox(row, textvariable=var, values=vals, state="readonly", font=('Inter', 10), width=48).pack(side='left', fill='x', expand=True)

    def _create_exercise_grid(self, parent):
        card = tk.Frame(parent, bg='white', relief='solid', bd=1)
        card.pack(fill='both', expand=True, pady=(0, 20))
        inner = tk.Frame(card, bg='white')
        inner.pack(fill='both', padx=20, pady=20)
        tk.Label(inner, text="Exercise Selection", font=('Inter', 20, 'bold'), fg='#2c3e50', bg='white', anchor='w').pack(fill='x', pady=(0, 15))
        tk.Frame(inner, bg='#ecf0f1', height=2).pack(fill='x', pady=(0, 15))
        grid = tk.Frame(inner, bg='white')
        grid.pack(fill='both', expand=True)
        exercises = [
            ("Hip Flexion", "Hip Flexion"), ("Hip Extension", "Hip Extension"),
            ("Hip Abduction", "Hip Abduction"), ("Hip Adduction", "Hip Adduction"),
            ("Shoulder Flexion", "Shoulder Flexion"), ("Shoulder Extension", "Shoulder Extension"),
            ("Shoulder Abduction", "Shoulder Abduction"), ("Shoulder Adduction", "Shoulder Adduction"),
            ("Elbow Flexion", "Elbow Flexion"), ("Elbow Extension", "Elbow Extension"),
            ("Wrist Flexion", "Wrist Flexion"), ("Wrist Extension", "Wrist Extension"),
            ("Ankle Plantarflexion", "Ankle Plantarflexion"), ("Ankle Dorsiflexion", "Ankle Dorsiflexion")
        ]
        colors = ['#3498db', '#9b59b6', '#e74c3c', '#f39c12', '#1abc9c', '#34495e', '#16a085']
        for i, (disp, name) in enumerate(exercises):
            r, c = divmod(i, 3)
            col = colors[i % len(colors)]
            btn = tk.Button(grid, text=disp, command=lambda n=name: self.open_exercise(n),
                            font=('Inter', 10, 'bold'), bg=col, fg='white',
                            activebackground=self._darken(col), relief='flat', cursor='hand2',
                            padx=20, pady=20, wraplength=180)
            btn.grid(row=r, column=c, padx=8, pady=8, sticky='nsew')
            grid.columnconfigure(c, weight=1)
            grid.rowconfigure(r, weight=1)

    def _darken(self, hexcol):
        h = hexcol.lstrip('#')
        r, g, b = [int(h[i:i+2], 16) for i in (0, 2, 4)]
        return f'#{int(r*0.8):02x}{int(g*0.8):02x}{int(b*0.8):02x}'

    def open_exercise(self, exercise_name):
        if self.controller.drag_mode_active:
            self.controller.toggle_drag_mode()
            self.after(400, lambda: self._proceed_to_exercise(exercise_name))
        else:
            self._proceed_to_exercise(exercise_name)

    def _proceed_to_exercise(self, exercise_name):
        if not self.controller.drag_mode_active:
            self.controller.show_page("exercise", exercise_name=exercise_name)
        else:
            messagebox.showwarning("Please Wait", "Drag mode is being disabled. Please try again in a moment.")

    def update_drag_status(self, active):
        if active:
            self.status_indicator.config(fg='#27ae60')
            self.drag_status.config(text="Status: Drag Mode Active")
            self.status_label.config(text="Robot is free to move – Guide it manually")
            self.btn_drag_mode.config(text="Disable Drag Mode", bg='#e74c3c')
        else:
            self.status_indicator.config(fg='#f39c12')
            self.drag_status.config(text="Status: Robot Locked")
            self.status_label.config(text="Robot connected and ready")
            self.btn_drag_mode.config(text="Enable Drag Mode", bg='#3498db')

# ------------------------------------------------------------------------- #
# PROGRESS REPORT PAGE
# ------------------------------------------------------------------------- #
class ProgressReportPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent, bg='#f5f6fa')
        self.controller = controller
        self._create_widgets()

    def _create_widgets(self):
        container = tk.Frame(self, bg='#f5f6fa')
        container.pack(fill='both', expand=True, padx=30, pady=30)
        tk.Button(container, text="Back to Dashboard",
                  command=lambda: self.controller.show_page("dashboard"),
                  font=('Inter', 10), bg='#34495e', fg='white',
                  relief='flat', cursor='hand2', padx=20, pady=10)\
                  .pack(anchor='w', pady=(0, 20))
        tk.Label(container, text="Patient Progress Report",
                 font=('Inter', 24, 'bold'), fg='#2c3e50', bg='#f5f6fa')\
                 .pack(pady=(0, 20))
        self.report_card = ProgressReportCard(container, self.controller)
        self.report_card.pack(fill='both', expand=True)
# ------------------------------------------------------------------------- #
# PROGRESS REPORT CARD – ONE GRAPH PER EXERCISE (SCROLLABLE) + PRINT (PDF)
# ------------------------------------------------------------------------- #
class ProgressReportCard(tk.Frame):
    REQUIRED = {"Timestamp", "Name", "Exercise", "Average", "Peak"}

    def __init__(self, parent, controller):
        super().__init__(parent, bg='white', relief='solid', bd=1)
        self.controller = controller
        self.report_name = tk.StringVar()
        self.graph_canvas = None
        self.graph_frame  = None          # inner frame that holds all graphs
        self.scrollbar    = None
        self.status_lbl   = None
        self.patient_info = {}            # Name, Age, Gender, Diagnosis
        self.figures = []                 # keep matplotlib figures for PDF
        self._create_widgets()

    # --------------------------------------------------------------------- #
    def _create_widgets(self):
        inner = tk.Frame(self, bg='white')
        inner.pack(fill='both', padx=20, pady=15, expand=True)

        # ----- patient name entry ------------------------------------------------
        tk.Label(inner, text="Patient name:",
                 font=('Inter', 12, 'bold'), fg='#2c3e50', bg='white',
                 anchor='w').pack(fill='x', pady=(0, 8))

        row = tk.Frame(inner, bg='white')
        row.pack(fill='x', pady=(0, 12))

        tk.Entry(row, textvariable=self.report_name, font=('Inter', 10),
                 bg='#ecf0f1', fg='#252e38', relief='flat', width=35)\
                 .pack(side='left', ipady=6, ipadx=8)

        tk.Button(row, text="Find & Plot", command=self._plot,
                  font=('Inter', 10, 'bold'), bg='#27ae60', fg='white',
                  relief='flat', cursor='hand2', padx=15, pady=6)\
                  .pack(side='left', padx=(10, 0))

        # ----- PRINT (PDF) BUTTON ------------------------------------------------
        tk.Button(row, text="Print Report (PDF)", command=self._print_report,
                  font=('Inter', 10, 'bold'), bg='#e67e22', fg='white',
                  relief='flat', cursor='hand2', padx=15, pady=6)\
                  .pack(side='left', padx=(10, 0))

        # ----- scrollable canvas -------------------------------------------------
        canvas_frame = tk.Frame(inner, bg='white')
        canvas_frame.pack(fill='both', expand=True)

        self.scrollbar = ttk.Scrollbar(canvas_frame, orient="vertical")
        self.graph_canvas = tk.Canvas(canvas_frame, bg='white',
                                      yscrollcommand=self.scrollbar.set,
                                      highlightthickness=0)
        self.scrollbar.config(command=self.graph_canvas.yview)
        self.scrollbar.pack(side='right', fill='y')
        self.graph_canvas.pack(side='left', fill='both', expand=True)

        self.graph_frame = tk.Frame(self.graph_canvas, bg='white')
        self.graph_canvas.create_window((0, 0), window=self.graph_frame,
                                        anchor='nw')

        # CORRECTED: Fixed missing ">"
        self.graph_frame.bind("<Configure>",
            lambda e: self.graph_canvas.configure(scrollregion=self.graph_canvas.bbox("all")))
        self.graph_canvas.bind("<Configure>",
            lambda e: self.graph_canvas.configure(scrollregion=self.graph_canvas.bbox("all")))

        # ----- status label ------------------------------------------------------
        self.status_lbl = tk.Label(inner,
                                   text="Enter a name and click Find & Plot",
                                   font=('Inter', 9), fg='#7f8c8d', bg='white')
        self.status_lbl.pack(pady=(5, 0))
    # --------------------------------------------------------------------- #
    def _clear(self):
        for w in self.graph_frame.winfo_children():
            w.destroy()
        self.figures.clear()

    # --------------------------------------------------------------------- #
    def _plot(self):
        name = self.report_name.get().strip()
        if not name:
            self._clear()
            self.status_lbl.config(text="Enter a patient name", fg='#e74c3c')
            return

        if not os.path.exists("results.csv"):
            self._clear()
            self.status_lbl.config(text="results.csv not found", fg='#e74c3c')
            return

        rows = []
        try:
            with open("results.csv", "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                if not reader.fieldnames:
                    raise ValueError("CSV is empty")
                missing = self.REQUIRED - set(reader.fieldnames)
                if missing:
                    raise ValueError(f"Missing columns: {', '.join(missing)}")
                for r in reader:
                    if r["Name"].strip().lower() == name.lower():
                        rows.append({
                            "date": r["Timestamp"].split()[0],
                            "ex":   r["Exercise"],
                            "avg":  float(r.get("Average") or 0),
                            "peak": float(r.get("Peak")    or 0),
                            "name": r["Name"],
                            "age":  r.get("Age", ""),
                            "gender": r.get("Gender", ""),
                            "diagnosis": r.get("Diagnosis", "")
                        })
        except Exception as e:
            self._clear()
            self.status_lbl.config(text=f"CSV error: {e}", fg='#e74c3c')
            return

        if not rows:
            self._clear()
            self.status_lbl.config(text="No data for this patient", fg='#e74c3c')
            return

        # store patient info for printing
        self.patient_info = {
            "Name": rows[0]["name"],
            "Age": rows[0]["age"],
            "Gender": rows[0]["gender"],
            "Diagnosis": rows[0]["diagnosis"]
        }

        self._draw_all_exercises(rows)

    # --------------------------------------------------------------------- #
    def _draw_all_exercises(self, rows):
        self._clear()
        patient_name = self.patient_info["Name"]

        # group rows by exercise
        ex_data = {}
        for r in rows:
            ex = r["ex"]
            ex_data.setdefault(ex, []).append(r)

        # one graph per exercise
        for ex, ex_rows in ex_data.items():
            dates = sorted({r["date"] for r in ex_rows})
            avg_vals  = []
            peak_vals = []

            for d in dates:
                day_rows = [r for r in ex_rows if r["date"] == d]
                avgs  = [r["avg"]  for r in day_rows]
                peaks = [r["peak"] for r in day_rows]
                avg_vals.append(np.mean(avgs))
                peak_vals.append(max(peaks) if peaks else 0)

            # ---- create matplotlib figure ------------------------------------
            fig = plt.Figure(figsize=(8, 4), dpi=100)
            ax  = fig.add_subplot(111)

            x = np.arange(len(dates))
            bar_width = 0.35

            ax.bar(x - bar_width/2, avg_vals,  bar_width,
                   label='Average (N)', color='#3498db', alpha=0.8)
            ax.bar(x + bar_width/2, peak_vals, bar_width,
                   label='Peak (N)',    color='#e74c3c', alpha=0.8)

            ax.set_xlabel("Date")
            ax.set_ylabel("Force (N)")
            ax.set_title(f"{ex} – {patient_name}")

            # --- HORIZONTAL DATE LABELS (no rotation) -------------------------
            ax.set_xticks(x)
            ax.set_xticklabels(dates, ha='center', fontsize=9)

            ax.legend(fontsize=9)
            ax.grid(True, axis='y', alpha=0.3)
            fig.tight_layout()

            # ---- embed figure in Tkinter ------------------------------------
            canvas = FigureCanvasTkAgg(fig, self.graph_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill='x', pady=(0, 20))

            # ---- KEEP FIGURE FOR PDF ----------------------------------------
            self.figures.append(fig)

        # ---- final status ----------------------------------------------------
        n_ex = len(ex_data)
        n_dates = len({r["date"] for r in rows})
        self.status_lbl.config(
            text=f"{len(rows)} trial(s) • {n_ex} exercise(s) • {n_dates} date(s)",
            fg='#27ae60')

    # --------------------------------------------------------------------- #
    def _print_report(self):
        if not self.patient_info:
            messagebox.showwarning("No Data", "Please generate a report first (Find & Plot).")
            return

        if not self.figures:
            messagebox.showwarning("No Graphs", "No graphs to print.")
            return

        # ask where to save PDF
        from tkinter import filedialog
        filename = filedialog.asksaveasfilename(
            defaultextension=".pdf",
            filetypes=[("PDF files", "*.pdf"), ("All files", "*.*")],
            title="Save Report As"
        )
        if not filename:
            return

        try:
            from matplotlib.backends.backend_pdf import PdfPages

            with PdfPages(filename) as pdf:
                # ---- PAGE 1: Title + Patient Info -------------------------------
                fig_title = plt.Figure(figsize=(8.5, 11), dpi=100)
                ax = fig_title.add_subplot(111)
                ax.axis('off')

                title = "ROBOT REHABILITATION PROGRESS REPORT"
                ax.text(0.5, 0.95, title, ha='center', va='center',
                        fontsize=16, fontweight='bold')

                info = (
                    f"Patient Name : {self.patient_info['Name']}\n"
                    f"Age          : {self.patient_info['Age']}\n"
                    f"Gender       : {self.patient_info['Gender']}\n"
                    f"Diagnosis    : {self.patient_info['Diagnosis']}\n"
                    f"Generated on : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
                )
                ax.text(0.5, 0.75, info, ha='center', va='top',
                        fontsize=11, linespacing=1.6)

                ax.text(0.5, 0.1, "See following pages for exercise graphs.",
                        ha='center', va='center', fontsize=10, style='italic')

                pdf.savefig(fig_title, bbox_inches='tight')
                plt.close(fig_title)

                # ---- PAGE 2+: One graph per page --------------------------------
                for fig in self.figures:
                    pdf.savefig(fig, bbox_inches='tight')
                    plt.close(fig)

            messagebox.showinfo("Success", f"PDF report saved to:\n{filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save PDF:\n{e}")

def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()