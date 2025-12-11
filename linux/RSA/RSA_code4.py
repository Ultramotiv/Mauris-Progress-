# code created on 1 Nov 2025
# Drag Mode: Enable → Free to Move | Disable → Locked | Re-Enable → Free Again
# DONE

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


class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Drag & Force Monitoring Panel")
        self.root.geometry("800x850")
        self.root.resizable(True, True)

        # ----- variables -------------------------------------------------
        self.robot = None
        self.baseline_forces = None
        self.monitoring_active = False
        self.monitoring_thread = None
        self.drag_mode_active = False
        self.current_robot_joints = None

        self.trials = []
        self.trial_axis = None
        self.current_trial = 0

        self.patient_name = tk.StringVar()
        self.patient_age = tk.StringVar()
        self.patient_gender = tk.StringVar()
        self.patient_diagnosis = tk.StringVar()

        # ----- layout ----------------------------------------------------
        main_container = tk.Frame(root)
        main_container.pack(fill='both', expand=True, padx=15, pady=15)

        canvas = tk.Canvas(main_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(main_container, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # ----- title ----------------------------------------------------
        title = ttk.Label(scrollable_frame, text="ROBOT DRAG & FORCE MONITORING",
                         font=('Arial', 18, 'bold'))
        title.grid(row=0, column=0, columnspan=2, pady=(0, 25), sticky='ew')
        scrollable_frame.columnconfigure(0, weight=1)

        # ----- Drag Mode button -----------------------------------------
        drag_frame = ttk.Frame(scrollable_frame)
        drag_frame.grid(row=1, column=0, pady=15, sticky='ew')
        drag_frame.columnconfigure(0, weight=1)

        self.btn_drag_mode = ttk.Button(drag_frame, text="Enable Drag Mode",
                                       command=self.toggle_drag_mode, width=25)
        self.btn_drag_mode.grid(row=0, column=0, padx=10, pady=8, sticky='ew')

        self.drag_status = ttk.Label(drag_frame, text="Status: Not Active",
                                    foreground="red", font=('Arial', 10, 'italic'))
        self.drag_status.grid(row=1, column=0, pady=5)

        # ----- Force/Torque recording ------------------------------------
        ft_frame = ttk.LabelFrame(scrollable_frame, text=" Force/Torque Recording ", padding=12)
        ft_frame.grid(row=2, column=0, sticky='ew', pady=12, padx=5)

        ttk.Label(ft_frame, text="Axis:").grid(row=0, column=0, padx=8, pady=5, sticky='w')
        self.axis_var = tk.StringVar(value="Fx")
        self.axis_dropdown = ttk.Combobox(ft_frame, textvariable=self.axis_var,
                                         values=["Fx","Fy","Fz","Mx","My","Mz"],
                                         state="readonly", width=10)
        self.axis_dropdown.grid(row=0, column=1, padx=8, pady=5)

        self.btn_record = ttk.Button(ft_frame, text="Record (5 sec)", command=self.record_ft_data)
        self.btn_record.grid(row=0, column=2, padx=8, pady=5)
        self.btn_record.config(state='disabled')

        self.status_label = ttk.Label(ft_frame, text="Status: Not Connected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=15, pady=5)

        # ----- Patient information ---------------------------------------
        patient_frame = ttk.LabelFrame(scrollable_frame, text=" Patient Information ", padding=15)
        patient_frame.grid(row=3, column=0, sticky='ew', pady=12, padx=5)

        labels = ["Name:", "Age:", "Gender:", "Diagnosis:"]
        entries = [
            ttk.Entry(patient_frame, textvariable=self.patient_name, width=35),
            ttk.Entry(patient_frame, textvariable=self.patient_age, width=10),
            ttk.Combobox(patient_frame, textvariable=self.patient_gender,
                         values=["Male","Female","Other"], state="readonly", width=15),
            ttk.Entry(patient_frame, textvariable=self.patient_diagnosis, width=45)
        ]

        for i, (lbl_text, entry) in enumerate(zip(labels, entries)):
            ttk.Label(patient_frame, text=lbl_text, font=('Arial', 11, 'bold')
                     ).grid(row=i, column=0, sticky='w', padx=10, pady=6)
            entry.grid(row=i, column=1, padx=10, pady=6, sticky='w')
            if i in (0, 3):
                entry.configure(width=40)

        self.btn_save = ttk.Button(patient_frame, text="Save to CSV", command=self.save_to_csv)
        self.btn_save.grid(row=4, column=1, pady=12, padx=10, sticky='e')
        self.btn_save.config(state='disabled')

        # ----- Trial results ---------------------------------------------
        result_frame = ttk.LabelFrame(scrollable_frame, text=" Trial Results ", padding=15)
        result_frame.grid(row=4, column=0, sticky='ew', pady=12, padx=5)

        self.trial_labels = []
        for i in range(3):
            lbl = ttk.Label(result_frame, text=f"Trial {i+1}: ---",
                            foreground="blue", font=('Arial', 12, 'bold'))
            lbl.grid(row=i, column=0, sticky='w', padx=12, pady=4)
            self.trial_labels.append(lbl)

        self.result_label = ttk.Label(result_frame, text="Result: ---",
                                     font=('Arial', 13, 'bold'), foreground="darkgreen")
        self.result_label.grid(row=3, column=0, sticky='w', padx=12, pady=15)

        btn_row = ttk.Frame(result_frame)
        btn_row.grid(row=4, column=0, pady=10, sticky='ew')
        btn_row.columnconfigure(0, weight=1)
        btn_row.columnconfigure(1, weight=1)

        self.btn_start_trial = ttk.Button(btn_row, text="Start Trials", command=self.start_next_trial)
        self.btn_start_trial.grid(row=0, column=0, padx=12, sticky='ew')
        reset_btn = ttk.Button(btn_row, text="Reset Trials", command=self.reset_trials)
        reset_btn.grid(row=0, column=1, padx=12, sticky='ew')

        # ----- startup ---------------------------------------------------
        self.log_message("System ready. Press 'Enable Drag Mode' to connect and enter free-drive.")

    # ------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------
    def log_message(self, msg):
        ts = datetime.now().strftime("%H:%M:%S")
        print(f"[{ts}] {msg}")

    def disable_buttons(self):
        for w in (self.btn_drag_mode, self.btn_record,
                  self.btn_start_trial, self.btn_save):
            w.config(state='disabled')

    def enable_buttons(self):
        self.btn_drag_mode.config(state='normal')
        if self.monitoring_active:
            self.btn_record.config(state='normal')
            if len(self.trials) < 3:
                self.btn_start_trial.config(state='normal', text="Start Next Trial")
            if len(self.trials) == 3:
                self.btn_save.config(state='normal')

    # ------------------------------------------------------------------
    # TOGGLE DRAG MODE: ENABLE ↔ DISABLE
    # ------------------------------------------------------------------
    def toggle_drag_mode(self):
        """Smart toggle: Enable → free | Disable → locked"""
        if not self.drag_mode_active:
            self.disable_buttons()
            threading.Thread(target=self.enable_drag_mode, daemon=True).start()
        else:
            self.disable_buttons()
            threading.Thread(target=self.disable_drag_mode, daemon=True).start()

    def enable_drag_mode(self):
        """Connect (if needed) → Enter free-drive"""
        try:
            # Connect if not already
            if not self.robot:
                self.log_message("Connecting to robot at 192.168.58.2...")
                self.robot = Robot.RPC('192.168.58.2')
                self.log_message("Robot connected successfully.")

                if not self.init_ft_sensor():
                    self.root.after(0, lambda: messagebox.showerror("FT Error", "Failed to init FT sensor."))
                    self.root.after(0, self.enable_buttons)
                    return

                if not self.calibrate_baseline_forces():
                    self.log_message("Baseline calibration skipped.")

            # ALWAYS SEND DragTeachSwitch(1)
            self.log_message("Enabling drag mode – robot will be free to move...")
            ret = self.robot.DragTeachSwitch(1)
            if ret != 0:
                self.log_message(f"DragTeachSwitch(1) failed: {ret}")
                self.root.after(0, lambda: messagebox.showerror("Error", f"Failed to enable drag mode (code {ret})"))
                self.root.after(0, self.enable_buttons)
                return

            # SUCCESS
            self.drag_mode_active = True
            if not self.monitoring_active:
                self.start_ft_monitoring()

            self.root.after(0, lambda: self.btn_drag_mode.config(text="Disable Drag Mode"))
            self.root.after(0, lambda: self.drag_status.config(text="Status: DRAG MODE ACTIVE", foreground="orange"))
            self.root.after(0, lambda: self.status_label.config(text="Status: Monitoring Active", foreground="green"))
            self.log_message("FREE-DRIVE ACTIVE – Move the robot by hand!")

        except Exception as e:
            self.log_message(f"Enable error: {e}")
            self.root.after(0, lambda: messagebox.showerror("Error", str(e)))
        finally:
            self.root.after(0, self.enable_buttons)

    def disable_drag_mode(self):
        """Exit free-drive → lock robot"""
        try:
            self.log_message("Disabling drag mode...")
            ret = self.robot.DragTeachSwitch(0)
            if ret == 0:
                self.drag_mode_active = False

                # Save current joint position
                error, joints = self.robot.GetActualJointPosDegree()
                if error == 0:
                    self.current_robot_joints = joints
                    self.log_message(f"  Robot locked at: {[f'{j:.2f}' for j in joints]} deg")
                else:
                    self.log_message("  Failed to read joint position")

                # Update UI
                self.root.after(0, lambda: self.btn_drag_mode.config(text="Enable Drag Mode"))
                self.root.after(0, lambda: self.drag_status.config(text="Status: Not Active", foreground="red"))
                self.root.after(0, lambda: self.status_label.config(
                    text="Status: Monitoring Active" if self.monitoring_active else "Status: Connected",
                    foreground="green" if self.monitoring_active else "blue"
                ))
                self.log_message("Drag mode DISABLED – robot is now locked.")
            else:
                self.log_message(f"Failed to disable drag mode: {ret}")
        except Exception as e:
            self.log_message(f"Disable error: {e}")
        finally:
            self.root.after(0, self.enable_buttons)

    # ------------------------------------------------------------------
    # Trial handling (unchanged)
    # ------------------------------------------------------------------
    def start_next_trial(self):
        if not self.monitoring_active:
            messagebox.showwarning("Warning", "Enable Drag Mode first.")
            return

        axis = self.axis_var.get()
        idx = ["Fx","Fy","Fz","Mx","My","Mz"].index(axis)

        if self.trial_axis and self.trial_axis != axis:
            self.reset_trials()

        self.trial_axis = axis
        self.current_trial = len(self.trials) + 1
        if self.current_trial > 3: return

        self.disable_buttons()
        self.log_message(f"--- Trial {self.current_trial}/3 ({axis}) ---")
        threading.Thread(target=self.run_one_trial,
                         args=(idx, axis, self.current_trial), daemon=True).start()

    def run_one_trial(self, idx, name, no):
        samples = []
        for _ in range(int(5.0 / 0.008)):
            data = self.robot.FT_GetForceTorqueRCS()
            if data[0] == 0:
                raw = [data[1][0], -data[1][1], data[1][2],
                       data[1][3], data[1][4], data[1][5]]
                forces = raw if not self.baseline_forces else \
                         [raw[i] - self.baseline_forces[i] for i in range(6)]
                samples.append(forces[idx])
            time.sleep(0.008)

        if samples:
            avg = sum(samples) / len(samples)
            self.trials.append(avg)
            self.root.after(0, lambda: self.trial_labels[no-1].config(
                text=f"Trial {no}: {avg:.4f} N"))

            if len(self.trials) == 3:
                self.root.after(100, self.show_trial_result)
                self.root.after(200, lambda: self.btn_start_trial.config(
                    state='disabled', text="Complete"))
                self.root.after(300, lambda: self.btn_save.config(state='normal'))
            else:
                self.root.after(100, lambda: self.btn_start_trial.config(
                    state='normal', text="Start Next Trial"))
        self.root.after(0, self.enable_buttons)

    def show_trial_result(self):
        avg = sum(self.trials) / 3
        peak = max(abs(x) for x in self.trials)
        self.result_label.config(text=f"Avg: {avg:.4f} N | Peak: {peak:.4f} N")
        print("\n" + "="*60)
        print(f"FINAL: {self.trial_axis} | Avg: {avg:.4f} N | Peak: {peak:.4f} N")
        print("="*60 + "\n")

    def reset_trials(self):
        self.trials = []
        self.trial_axis = None
        for lbl in self.trial_labels:
            lbl.config(text="---")
        self.result_label.config(text="Result: ---")
        self.btn_start_trial.config(text="Start Trials",
                                   state='normal' if self.monitoring_active else 'disabled')
        self.btn_save.config(state='disabled')

    # ------------------------------------------------------------------
    # CSV export
    # ------------------------------------------------------------------
    def save_to_csv(self):
        if len(self.trials) != 3:
            messagebox.showwarning("Incomplete", "Finish 3 trials first.")
            return
        if not all([self.patient_name.get().strip(),
                    self.patient_age.get().strip(),
                    self.patient_gender.get(),
                    self.patient_diagnosis.get().strip()]):
            messagebox.showwarning("Missing Info", "Fill all patient fields.")
            return

        avg = sum(self.trials)/3
        peak = max(abs(x) for x in self.trials)
        data = {
            "Timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Name": self.patient_name.get().strip(),
            "Age": self.patient_age.get().strip(),
            "Gender": self.patient_gender.get(),
            "Diagnosis": self.patient_diagnosis.get().strip(),
            "Axis": self.trial_axis,
            "Trial1": f"{self.trials[0]:.4f}",
            "Trial2": f"{self.trials[1]:.4f}",
            "Trial3": f"{self.trials[2]:.4f}",
            "Average": f"{avg:.4f}",
            "Peak": f"{peak:.4f}"
        }
        exists = os.path.isfile("results.csv")
        with open("results.csv", "a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=data.keys())
            if not exists: w.writeheader()
            w.writerow(data)
        messagebox.showinfo("Saved", "Data saved to results.csv")
        self.btn_save.config(state='disabled')

    # ------------------------------------------------------------------
    # FT sensor helpers
    # ------------------------------------------------------------------
    def init_ft_sensor(self):
        try:
            self.robot.FT_SetConfig(24, 0)
            self.robot.FT_Activate(0); time.sleep(0.5)
            self.robot.FT_Activate(1); time.sleep(0.5)
            self.robot.SetLoadWeight(0, 0.0)
            self.robot.SetLoadCoord(0,0,0)
            self.robot.FT_SetZero(0); time.sleep(0.5)
            self.robot.FT_SetZero(1); time.sleep(0.5)
            return True
        except Exception as e:
            self.log_message(f"FT init error: {e}")
            return False

    def calibrate_baseline_forces(self):
        samples = []
        for _ in range(100):
            try:
                d = self.robot.FT_GetForceTorqueRCS()
                if d[0] == 0:
                    samples.append([d[1][0], -d[1][1], d[1][2],
                                   d[1][3], d[1][4], d[1][5]])
                time.sleep(0.01)
            except: pass
        if samples:
            self.baseline_forces = [sum(col)/len(samples) for col in zip(*samples)]
            return True
        self.baseline_forces = [0]*6
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
                    raw = [d[1][0], -d[1][1], d[1][2],
                           d[1][3], d[1][4], d[1][5]]
                    forces = raw if not self.baseline_forces else \
                             [raw[i] - self.baseline_forces[i] for i in range(6)]
                    for i in range(6):
                        if abs(forces[i]) < 0.5: forces[i] = 0.0
                time.sleep(0.008)
            except:
                time.sleep(0.008)

    def record_ft_data(self):
        messagebox.showinfo("Info", "Use the 'Start Trials' button below.")

    # ------------------------------------------------------------------
    # Window close
    # ------------------------------------------------------------------
    def on_closing(self):
        self.monitoring_active = False
        if self.drag_mode_active and self.robot:
            try:
                self.robot.DragTeachSwitch(0)
                self.log_message("Drag mode safely disabled on exit.")
            except: pass
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1)
        self.root.destroy()


# ----------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------
def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()