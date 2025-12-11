#Finished Code!
import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from datetime import datetime
import time
import csv
import os
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from matplotlib.patches import Patch

# === Predefined Axis for Each Exercise ===
EXERCISE_AXES = {
    "Hip Flexion": "Fz", "Hip Extension": "Fz", "Hip Abduction": "Fz", "Hip Adduction": "Fz",
    "Shoulder Flexion": "Fz", "Shoulder Extension": "Fz", "Shoulder Abduction": "Fz", "Shoulder Adduction": "Fz",
    "Elbow Flexion": "Fz", "Elbow Extension": "Fz", "Wrist Flexion": "Fz", "Wrist Extension": "Fz",
    "Ankle Plantarflexion": "Fz", "Ankle Dorsiflexion": "Fz"
}

# === Image for Each Exercise ===
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

        self.root.update_idletasks()
        req_w = max(self.pages_container.winfo_reqwidth(), 1000)
        req_h = max(self.pages_container.winfo_reqheight(), 700)
        self.root.geometry(f"{req_w}x{req_h}")
        self.root.minsize(900, 600)
        self.root.resizable(True, True)
        self.root.update_idletasks()
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
                self.log_message("Connecting to robot at 192.168.58.2...")
                self.robot = Robot.RPC('192.168.58.2')
                self.log_message("Robot connected successfully.")
            if not self.init_ft_sensor():
                messagebox.showerror("Sensor Error", "Failed to initialize force/torque sensor.")
                return
            if not self.calibrate_baseline_forces():
                self.log_message("Baseline calibration skipped.")
            ret = self.robot.DragTeachSwitch(1)
            if ret != 0:
                self.log_message(f"DragTeachSwitch(1) failed: {ret}")
                messagebox.showerror("Error", f"Failed to enable drag mode (error {ret})")
                return
            self.drag_mode_active = True
            if not self.monitoring_active:
                self.start_ft_monitoring()
            self.log_message("FREE-DRIVE ACTIVE – Robot can be moved by hand!")
            self._update_drag_ui(active=True)
        except Exception as e:
            self.log_message(f"Enable error: {e}")
            messagebox.showerror("Connection Error", str(e))

    def disable_drag_mode(self):
        try:
            self.log_message("Disabling drag mode...")
            if self.robot:
                ret = self.robot.DragTeachSwitch(0)
                if ret == 0:
                    self.drag_mode_active = False
                    err, joints = self.robot.GetActualJointPosDegree()
                    if err == 0:
                        self.current_robot_joints = joints
                        self.log_message(f"Robot locked at: {[f'{j:.2f}' for j in joints]} deg")
                    self._update_drag_ui(active=False)
                    self.log_message("Drag mode DISABLED – Robot is now locked in position")
        except Exception as e:
            self.log_message(f"Disable error: {e}")
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
            self.log_message("Force/torque sensor initialized successfully")
            return True
        except Exception as e:
            self.log_message(f"FT sensor init error: {e}")
            return False

    def calibrate_baseline_forces(self):
        self.log_message("Calibrating baseline forces...")
        samples = []
        for _ in range(100):
            try:
                d = self.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    samples.append([d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]])
                time.sleep(0.01)
            except:
                pass
        if samples:
            self.baseline_forces = [sum(col)/len(samples) for col in zip(*samples)]
            self.log_message(f"Baseline calibrated: {[f'{x:.2f}' for x in self.baseline_forces]}")
            return True
        self.baseline_forces = [0]*6
        self.log_message("Baseline calibration failed – using zeros")
        return False

    def start_ft_monitoring(self):
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self.ft_monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        self.log_message("Force/torque monitoring started")

    def ft_monitoring_loop(self):
        while self.monitoring_active:
            try:
                d = self.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    raw = [d[1][0], -d[1][1], d[1][2], d[1][3], d[1][4], d[1][5]]
                    forces = raw if not self.baseline_forces else \
                                         [raw[i] - self.baseline_forces[i] for i in range(6)]
                    for i in range(6):
                        if abs(forces[i]) < 0.5:
                            forces[i] = 0.0
                time.sleep(0.008)
            except:
                time.sleep(0.008)

    def get_patient_data(self):
        return {
            'name': self.patient_name,
            'age': self.patient_age,
            'gender': self.patient_gender,
            'diagnosis': self.patient_diagnosis
        }

    def on_closing(self):
        self.log_message("Shutting down application...")
        self.monitoring_active = False
        if self.drag_mode_active and self.robot:
            try:
                self.robot.DragTeachSwitch(0)
                self.log_message("Drag mode safely disabled on exit")
            except Exception as e:
                self.log_message(f"Error disabling drag mode on exit: {e}")
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1)
        self.log_message("Application closed successfully")
        self.root.destroy()
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
# ------------------------------------------------------------------------- #
# EXERCISE TRIAL PAGE
# ------------------------------------------------------------------------- #
class ExerciseTrialPage(tk.Frame):
    def __init__(self, parent, controller, exercise_name):
        super().__init__(parent, bg='#f5f6fa')
        self.controller = controller
        self.exercise_name = exercise_name
        self.patient_data = controller.get_patient_data()
        self.monitoring_active = controller.monitoring_active
        self.baseline_forces = controller.baseline_forces
        self.trial_data = []
        self.trial_force_avg = None
        self.trial_force_peak = None
        self.trial_axis = EXERCISE_AXES.get(exercise_name, "Fz")
        self.progress_var = tk.DoubleVar(value=0.0)
        self.image_path = EXERCISE_IMAGES.get(exercise_name)
        self.photo = None
        self.image_label = None
        self.gif_img = None
        self.gif_frame = 0
        self.root = controller.root
        self._create_widgets()
        controller.log_message(f"Exercise page opened – {exercise_name} (Axis: {self.trial_axis})")

    def _create_widgets(self):
        container = tk.Frame(self, bg='#f5f6fa')
        container.pack(fill='both', expand=True, padx=20, pady=20)
        tk.Button(container, text="Back to Dashboard", command=self.go_back,
                  font=('Inter', 10), bg='#34495e', fg='white', relief='flat', cursor='hand2',
                  padx=20, pady=10).pack(anchor='w', pady=(0, 15))
        hdr = tk.Frame(container, bg='#3498db', height=60)
        hdr.pack(fill='x', pady=(0, 20))
        hdr.pack_propagate(False)
        tk.Label(hdr, text=f"{self.exercise_name} | Axis: {self.trial_axis}",
                 font=('Inter', 16, 'bold'), fg='white', bg='#3498db').pack(expand=True)
        self._create_trial_results(container)
        self._create_action_buttons(container)
        self.update_buttons()

    def _create_trial_results(self, parent):
        card = tk.Frame(parent, bg='white', relief='solid', bd=1)
        card.pack(fill='both', expand=True, pady=(0, 15))
        inner = tk.Frame(card, bg='white')
        inner.pack(fill='both', expand=True, padx=20, pady=20)
        img_frame = tk.Frame(inner, bg='white')
        img_frame.pack(pady=(0, 15), fill='x')
        if self.image_path and os.path.exists(self.image_path):
            try:
                self.gif_img = Image.open(self.image_path)
                max_size = (220, 220)
                self.gif_img.thumbnail(max_size, Image.Resampling.LANCZOS)
                self.photo = ImageTk.PhotoImage(self.gif_img)
                self.image_label = tk.Label(img_frame, image=self.photo, bg='white')
                self.image_label.pack(expand=True)
                if getattr(self.gif_img, "is_animated", False):
                    self.root.after(100, self.animate_gif)
            except Exception as e:
                self.controller.log_message(f"Image load error: {e}")
        else:
            tk.Label(img_frame, text="No image available", fg='gray', bg='white').pack()
        tf = tk.Frame(inner, bg='#ecf0f1', relief='flat')
        tf.pack(fill='x', pady=6)
        tk.Label(tf, text=" 1 ", font=('Inter', 10, 'bold'), fg='white', bg='#95a5a6').pack(side='left', padx=(10, 15), pady=8)
        self.trial_label = tk.Label(tf, text="Trial 1: Awaiting measurement...",
                                    font=('Inter', 10), fg='#2c3e50', bg='#ecf0f1', anchor='w')
        self.trial_label.pack(side='left', fill='x', expand=True, pady=8)
        tk.Frame(inner, bg='#bdc3c7', height=2).pack(fill='x', pady=20)
        self.result_label = tk.Label(inner, text="Results: Run Trial 1",
                                     font=('Inter', 12, 'bold'), fg='black', bg='white', anchor='w')
        self.result_label.pack(fill='x', pady=(0, 10))
        self.graph_frame = tk.Frame(inner, bg='white', height=250)
        self.graph_frame.pack(fill='both', expand=True, pady=(10, 0))
        self.graph_frame.pack_propagate(False)
        btns = tk.Frame(inner, bg='white')
        btns.pack(fill='x', pady=(15, 0))
        self.btn_start_trial = tk.Button(btns, text="Start Trial 1", command=self.start_trial,
                                         font=('Inter', 10, 'bold'), bg='#3498db', fg='white',
                                         relief='flat', padx=20, pady=10)
        self.btn_start_trial.pack(side='left', padx=(0, 10))
        tk.Button(btns, text="Reset", command=self.reset_trial, font=('Inter', 10), bg='#95a5a6',
                  fg='white', relief='flat', padx=20, pady=10).pack(side='left', padx=(5, 0))
        self.progress_bar = ttk.Progressbar(btns, orient="horizontal", length=300, mode="determinate",
                                            variable=self.progress_var, maximum=100)
        self.progress_bar.pack(pady=(8, 0))

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

    def _create_action_buttons(self, parent):
        bf = tk.Frame(parent, bg='#f5f6fa')
        bf.pack(fill='x', pady=(15, 0))
        self.btn_save = tk.Button(bf, text="Save to CSV", command=self.save_to_csv,
                                  font=('Inter', 10, 'bold'), bg='#27ae60', fg='white',
                                  relief='flat', padx=25, pady=12, state='disabled')
        self.btn_save.pack(side='left')

    def go_back(self):
        self.controller.show_page("dashboard")

    def refresh_robot(self):
        self.robot = self.controller.robot
        if not self.robot:
            messagebox.showerror("Robot Disconnected", "Robot is not connected. Enable Drag Mode first.")
            return False
        return True

    def update_buttons(self):
        if self.controller.monitoring_active and not self.trial_data:
            self.btn_start_trial.config(state='normal')
        else:
            self.btn_start_trial.config(state='disabled')
        if self.trial_data:
            self.btn_save.config(state='normal')

    def start_trial(self):
        if not self.refresh_robot():
            return
        if not self.controller.monitoring_active:
            messagebox.showwarning("Warning", "Enable Drag Mode first.", parent=self)
            return
        idx = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"].index(self.trial_axis)
        self.progress_var.set(0.0)
        self.btn_start_trial.config(state='disabled')
        self.trial_data = []
        threading.Thread(target=self.run_trial, args=(idx,), daemon=True).start()

    def run_trial(self, idx):
        if not self.refresh_robot():
            self.after(0, lambda: self.btn_start_trial.config(state='normal'))
            return
        total_samples = int(5.0 / 0.008)
        samples = []
        times = []
        start_time = time.time()
        for i in range(total_samples):
            try:
                data = self.robot.FT_GetForceTorqueRCS()
                if data[0] == 0:
                    raw = [data[1][0], -data[1][1], data[1][2], data[1][3], data[1][4], data[1][5]]
                    forces = raw if not self.baseline_forces else \
                                         [raw[j] - self.baseline_forces[j] for j in range(6)]
                    force_val = forces[idx]
                    if self.trial_axis == "Fz":
                        force_val = -force_val
                    if abs(force_val) < 0.5:
                        force_val = 0.0
                    samples.append(force_val)
                    times.append(time.time() - start_time)
                    percent = (i + 1) / total_samples * 100
                    self.after(0, self.progress_var.set, percent)
                time.sleep(0.008)
            except Exception as e:
                self.controller.log_message(f"Trial error: {e}")
                break

        if samples and len(samples) > 10:
            valid_pushes = [s for s in samples if s > 0.5]
            if valid_pushes:
                self.trial_force_avg = sum(valid_pushes) / len(valid_pushes)
                self.trial_force_peak = max(valid_pushes)
            else:
                self.trial_force_avg = self.trial_force_peak = 0.0
        else:
            self.trial_force_avg = self.trial_force_peak = 0.0
        self.trial_data = (times, samples)
        kg_avg = self.trial_force_avg * 0.10197
        kg_peak = self.trial_force_peak * 0.10197
        self.after(0, lambda: self.trial_label.config(
            text=f"Trial 1: {self.trial_force_avg:.4f} N (Avg) | {self.trial_force_peak:.4f} N (Peak)",
            fg='#27ae60', bg='#d5f4e6'))
        self.after(0, lambda: self.result_label.config(
            text=f"Average: {self.trial_force_avg:.4f} N ({kg_avg:.4f} kg) | Peak: {self.trial_force_peak:.4f} N ({kg_peak:.4f} kg)",
            fg='black'))
        self.after(100, self.plot_graph)
        self.after(200, lambda: self.btn_start_trial.config(text="Trial Complete", state='disabled', bg='#95a5a6'))
        self.after(300, lambda: self.btn_save.config(state='normal'))
        if not any(s > 0.5 for s in samples):
            self.after(0, lambda: self.trial_label.config(text="Trial 1: No significant force detected", fg='#e67e22'))

    def plot_graph(self):
        for widget in self.graph_frame.winfo_children():
            widget.destroy()
        fig = plt.Figure(figsize=(6, 3.5), dpi=100)
        ax = fig.add_subplot(111)
        if self.trial_data:
            times, forces = self.trial_data
            ax.plot(times, forces, label='Trial 1', color='#e74c3c', linewidth=2)
            if self.trial_force_avg > 0.5:
                ax.axhline(y=self.trial_force_avg, color='blue', linestyle='--', linewidth=1.5,
                           label=f'Avg: {self.trial_force_avg:.3f} N')
                ax.text(0.01, self.trial_force_avg, f' Avg: {self.trial_force_avg:.3f} N',
                        color='blue', va='bottom', ha='left', fontsize=9,
                        bbox=dict(boxstyle="round", fc="white", ec="blue", alpha=0.8))
            if self.trial_force_peak > 0.5:
                ax.axhline(y=self.trial_force_peak, color='green', linestyle=':', linewidth=1.5,
                           label=f'Peak: {self.trial_force_peak:.3f} N')
                ax.text(0.01, self.trial_force_peak, f' Peak: {self.trial_force_peak:.3f} N',
                        color='green', va='top', ha='left', fontsize=9,
                        bbox=dict(boxstyle="round", fc="white", ec="green", alpha=0.8))
            ax.set_title(f"{self.exercise_name} - Force Over Time (Trial 1)", fontsize=12, pad=10)
            ax.set_xlabel("Time (seconds)", fontsize=10)
            ax.set_ylabel("Force (N)", fontsize=10)
            ax.legend(fontsize=9)
            ax.grid(True, alpha=0.3)
            ax.margins(0.05)
        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, self.graph_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill='both', expand=True)

    def reset_trial(self):
        self.trial_data = []
        self.trial_force_avg = None
        self.trial_force_peak = None
        self.trial_label.config(text="Trial 1: Awaiting measurement...", fg='#2c3e50', bg='#ecf0f1')
        self.result_label.config(text="Results: Run Trial 1", fg='black')
        for widget in self.graph_frame.winfo_children():
            widget.destroy()
        self.btn_start_trial.config(text="Start Trial 1", bg='#3498db',
                                    state='normal' if self.controller.monitoring_active else 'disabled')
        self.btn_save.config(state='disabled')
        self.progress_var.set(0.0)

    def save_to_csv(self):
        if not self.trial_data:
            messagebox.showwarning("Incomplete", "Please complete Trial 1 first.", parent=self)
            return

        name      = self.patient_data['name'].get().strip()
        age       = self.patient_data['age'].get().strip()
        gender    = self.patient_data['gender'].get()
        diagnosis = self.patient_data['diagnosis'].get().strip()

        if not all([name, age, gender, diagnosis]):
            messagebox.showwarning("Missing Information", "Please fill all patient fields.", parent=self)
            return

        # ---- NEW: use the exact column order you asked for -----------------
        data = {
            "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Exercise":  self.exercise_name,
            "Name":      name,
            "Age":       age,
            "Gender":    gender,
            "Diagnosis": diagnosis,
            "Axis":      self.trial_axis,
            "Trial1":    f"{self.trial_force_avg:.4f}",   # <-- Trial1 = average force
            "Average":   f"{self.trial_force_avg:.4f}",
            "Peak":      f"{self.trial_force_peak:.4f}"
        }

        exists = os.path.isfile("results.csv")
        with open("results.csv", "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=data.keys())
            if not exists:
                w.writeheader()
            w.writerow(data)

        messagebox.showinfo("Success", f"Data for '{self.exercise_name}' saved!", parent=self)
        self.btn_save.config(state='disabled')


def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()