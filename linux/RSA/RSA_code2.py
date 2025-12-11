import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import sys
import threading
from datetime import datetime
import time
import csv
import os

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Panel with FT Monitoring")
        self.root.geometry("800x700")
        self.root.resizable(True, True)
        
        # Robot and FT monitoring variables
        self.robot = None
        self.baseline_forces = None
        self.monitoring_active = False
        self.monitoring_thread = None
        self.drag_mode_active = False
        
        # CSV trajectory data
        self.csv_file_path = None
        self.csv_start_joints = None  # Initial joint angles from CSV
        self.csv_trajectory = []  # All trajectory points from CSV
        self.current_robot_joints = None  # Current position when starting trajectory
        
        # Configure style
        style = ttk.Style()
        style.theme_use('clam')
        
        # Main frame
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(4, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="Robot Control Panel", 
                                font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, pady=10)
        
        # CSV File Selection Frame
        csv_frame = ttk.LabelFrame(main_frame, text="CSV Trajectory File", padding="10")
        csv_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.csv_label = ttk.Label(csv_frame, text="No file selected", foreground="red")
        self.csv_label.grid(row=0, column=0, padx=5, sticky=tk.W)
        
        self.btn_browse = ttk.Button(csv_frame, text="Browse CSV", 
                                     command=self.browse_csv_file, width=15)
        self.btn_browse.grid(row=0, column=1, padx=5)
        
        # Button frame
        button_frame = ttk.Frame(main_frame, padding="10")
        button_frame.grid(row=2, column=0, pady=10)
        
        # Create buttons
        self.btn_elbow_flexion = ttk.Button(button_frame, text="Elbow Flexion", 
                                    command=self.elbow_flexion_action, width=20)
        self.btn_elbow_flexion.grid(row=0, column=0, padx=10, pady=10)
        
        self.btn_drag_mode = ttk.Button(button_frame, text="Enable Drag Mode", 
                               command=self.toggle_drag_mode, width=20)
        self.btn_drag_mode.grid(row=0, column=1, padx=10, pady=10)
        
        self.btn3 = ttk.Button(button_frame, text="Button 3", 
                               command=self.button3_action, width=20)
        self.btn3.grid(row=0, column=2, padx=10, pady=10)
        
        # Trajectory Movement Frame
        trajectory_frame = ttk.LabelFrame(main_frame, text="Move to Trajectory Position", padding="10")
        trajectory_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=10)
        
        # Degree input
        degree_label = ttk.Label(trajectory_frame, text="Enter Degrees:")
        degree_label.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        
        self.degree_entry = ttk.Entry(trajectory_frame, width=15)
        self.degree_entry.grid(row=0, column=1, padx=5, pady=5)
        self.degree_entry.insert(0, "30")  # Default value
        
        # Move button
        self.btn_move_trajectory = ttk.Button(trajectory_frame, text="Move to Position", 
                                              command=self.move_to_trajectory_position, width=20)
        self.btn_move_trajectory.grid(row=0, column=2, padx=10, pady=5)
        self.btn_move_trajectory.config(state='disabled')
        
        # Info label
        info_label = ttk.Label(trajectory_frame, 
                               text="(Calculation: CSV_start - Current + Degrees)", 
                               font=('Arial', 8, 'italic'))
        info_label.grid(row=1, column=0, columnspan=3, pady=2)
        
        # FT Recording frame
        ft_frame = ttk.LabelFrame(main_frame, text="Force/Torque Recording", padding="10")
        ft_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=10)
        
        # Axis selection
        axis_label = ttk.Label(ft_frame, text="Select Axis:")
        axis_label.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        
        self.axis_var = tk.StringVar(value="Fx")
        self.axis_dropdown = ttk.Combobox(ft_frame, textvariable=self.axis_var, 
                                          values=["Fx", "Fy", "Fz", "Mx", "My", "Mz"],
                                          state="readonly", width=15)
        self.axis_dropdown.grid(row=0, column=1, padx=5, pady=5)
        
        # Record button
        self.btn_record = ttk.Button(ft_frame, text="Record (5 sec)", 
                                     command=self.record_ft_data, width=20)
        self.btn_record.grid(row=0, column=2, padx=10, pady=5)
        self.btn_record.config(state='disabled')
        
        # Status indicator
        self.status_label = ttk.Label(ft_frame, text="Status: Not Connected", 
                                      foreground="red")
        self.status_label.grid(row=0, column=3, padx=10, pady=5)
        
        # Status log frame
        log_frame = ttk.LabelFrame(main_frame, text="Status Log", padding="10")
        log_frame.grid(row=5, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)
        
        # Text widget for output
        self.output_text = scrolledtext.ScrolledText(log_frame, 
                                                      height=12, 
                                                      width=90,
                                                      wrap=tk.WORD)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Clear log button
        clear_btn = ttk.Button(main_frame, text="Clear Log", 
                               command=self.clear_log, width=15)
        clear_btn.grid(row=6, column=0, pady=5)
        
        self.log_message("System initialized. Select CSV file and press 'Elbow Flexion' to start.")
    
    def log_message(self, message):
        """Add timestamped message to the output text widget"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.output_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.output_text.see(tk.END)
        self.root.update_idletasks()
    
    def clear_log(self):
        """Clear the output text widget"""
        self.output_text.delete(1.0, tk.END)
        self.log_message("Log cleared.")
    
    def browse_csv_file(self):
        """Browse and select CSV trajectory file"""
        file_path = filedialog.askopenfilename(
            title="Select CSV Trajectory File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if file_path:
            self.csv_file_path = file_path
            filename = os.path.basename(file_path)
            self.csv_label.config(text=f"Selected: {filename}", foreground="green")
            self.log_message(f"CSV file selected: {filename}")
            
            # Load CSV data
            if self.load_csv_trajectory():
                self.log_message(f"✓ CSV loaded: {len(self.csv_trajectory)} points")
                self.log_message(f"  Start joints: {[f'{j:.2f}' for j in self.csv_start_joints]}")
    
    def load_csv_trajectory(self):
        """Load trajectory data from CSV file"""
        try:
            self.csv_trajectory = []
            
            with open(self.csv_file_path, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)  # Read header
                
                # Check if header contains joint columns
                self.log_message(f"CSV Header: {header}")
                
                for row in reader:
                    # CSV format: timestamp,joint1,joint2,joint3,joint4,joint5,joint6
                    # Skip timestamp (column 0), take joints (columns 1-6)
                    if len(row) >= 7:
                        joints = [float(row[i]) for i in range(1, 7)]  # columns 1-6
                        self.csv_trajectory.append(joints)
            
            if self.csv_trajectory:
                # Store the first row as start position
                self.csv_start_joints = self.csv_trajectory[0].copy()
                self.log_message(f"✓ Loaded {len(self.csv_trajectory)} trajectory points")
                return True
            else:
                self.log_message("✗ CSV file is empty")
                return False
                
        except Exception as e:
            self.log_message(f"✗ Error loading CSV: {str(e)}")
            import traceback
            self.log_message(traceback.format_exc())
            return False
    
    def disable_buttons(self):
        """Disable all buttons during operation"""
        self.btn_elbow_flexion.config(state='disabled')
        self.btn_drag_mode.config(state='disabled')
        self.btn3.config(state='disabled')
        self.btn_move_trajectory.config(state='disabled')
    
    def enable_buttons(self):
        """Enable all buttons after operation"""
        self.btn_elbow_flexion.config(state='normal')
        self.btn_drag_mode.config(state='normal')
        self.btn3.config(state='normal')
        if self.csv_start_joints and self.robot:
            self.btn_move_trajectory.config(state='normal')
    
    def toggle_drag_mode(self):
        """Toggle drag mode on/off"""
        if not self.robot:
            messagebox.showwarning("Warning", "Robot not connected. Press 'Elbow Flexion' first.")
            return
        
        thread = threading.Thread(target=self.run_drag_mode_toggle)
        thread.daemon = True
        thread.start()
    
    def run_drag_mode_toggle(self):
        """Enable or disable drag mode"""
        try:
            if not self.drag_mode_active:
                # Enable drag mode
                self.log_message("Enabling drag mode...")
                ret = self.robot.DragTeachSwitch(1)  # 1 = enable
                
                if ret == 0:
                    self.drag_mode_active = True
                    self.btn_drag_mode.config(text="Disable Drag Mode")
                    self.log_message("✓ Drag mode ENABLED - You can now move the robot manually")
                    self.status_label.config(text="Status: Drag Mode Active", foreground="orange")
                else:
                    self.log_message(f"✗ Failed to enable drag mode. Error: {ret}")
            else:
                # Disable drag mode
                self.log_message("Disabling drag mode...")
                ret = self.robot.DragTeachSwitch(0)  # 0 = disable
                
                if ret == 0:
                    self.drag_mode_active = False
                    self.btn_drag_mode.config(text="Enable Drag Mode")
                    self.log_message("✓ Drag mode DISABLED")
                    
                    # Update current position
                    error, joints = self.robot.GetActualJointPosDegree()
                    if error == 0:
                        self.current_robot_joints = joints
                        self.log_message(f"  New position: {[f'{j:.2f}' for j in joints]}")
                    
                    if self.monitoring_active:
                        self.status_label.config(text="Status: Monitoring Active", foreground="green")
                    else:
                        self.status_label.config(text="Status: Connected", foreground="blue")
                else:
                    self.log_message(f"✗ Failed to disable drag mode. Error: {ret}")
                    
        except Exception as e:
            self.log_message(f"✗ Drag mode error: {str(e)}")
    
    def move_to_trajectory_position(self):
        """Move robot to position based on CSV trajectory + degrees offset"""
        if not self.robot:
            messagebox.showwarning("Warning", "Robot not connected.")
            return
        
        if not self.csv_start_joints:
            messagebox.showwarning("Warning", "No CSV file loaded.")
            return
        
        try:
            degrees = float(self.degree_entry.get())
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number for degrees.")
            return
        
        self.disable_buttons()
        self.log_message(f"Moving to trajectory position at {degrees} degrees...")
        
        thread = threading.Thread(target=self.run_trajectory_move, args=(degrees,))
        thread.daemon = True
        thread.start()
    
    def run_trajectory_move(self, target_degrees):
        """Calculate and move to trajectory position"""
        try:
            # Get current robot position if not already stored
            if not self.current_robot_joints:
                error, joints = self.robot.GetActualJointPosDegree()
                if error != 0:
                    self.log_message("✗ Failed to get current joint position")
                    self.enable_buttons()
                    return
                self.current_robot_joints = joints
            
            self.log_message(f"Current joints: {[f'{j:.2f}' for j in self.current_robot_joints]}")
            self.log_message(f"CSV start joints: {[f'{j:.2f}' for j in self.csv_start_joints]}")
            
            # Find the trajectory point closest to target_degrees
            # Assuming the trajectory represents movement in joint space where we track total angular change
            # For simplicity, we'll use the first joint (J2 - elbow) as the reference
            
            # Calculate offset: (CSV_start - Current)
            offset = [self.csv_start_joints[i] - self.current_robot_joints[i] for i in range(6)]
            self.log_message(f"Offset: {[f'{o:.2f}' for o in offset]}")
            
            # Find trajectory point where J2 (elbow) has moved by target_degrees from start
            target_point = None
            j2_start = self.csv_start_joints[1]  # J2 from CSV start
            
            for traj_point in self.csv_trajectory:
                j2_current = traj_point[1]
                j2_change = abs(j2_current - j2_start)
                
                if j2_change >= target_degrees:
                    target_point = traj_point
                    break
            
            if not target_point:
                # If target not found, use last point
                target_point = self.csv_trajectory[-1]
                self.log_message(f"Target {target_degrees}° not found, using last point")
            
            # Calculate final position: trajectory_point + offset
            final_joints = [target_point[i] + offset[i] for i in range(6)]
            
            self.log_message(f"Target trajectory point: {[f'{j:.2f}' for j in target_point]}")
            self.log_message(f"Final target joints: {[f'{j:.2f}' for j in final_joints]}")
            
            # Move to calculated position
            MAX_JOINT_SPEED = 180.0
            desired_speed = 7.70
            vel_percent = (desired_speed / MAX_JOINT_SPEED) * 100.0
            vel_percent = round(vel_percent, 3)
            
            self.log_message(f"Moving robot (vel={vel_percent}%)...")
            
            ret = self.robot.MoveJ(
                joint_pos=final_joints,
                tool=0,
                user=0,
                desc_pos=[0.0]*7,
                vel=vel_percent,
                acc=0.0,
                ovl=100.0,
                exaxis_pos=[0.0]*4,
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0.0]*6
            )
            
            time.sleep(0.5)
            
            if ret == 0:
                error, new_joints = self.robot.GetActualJointPosDegree()
                if error == 0:
                    self.log_message(f"✓ Movement complete!")
                    self.log_message(f"  New position: {[f'{j:.2f}' for j in new_joints]}")
                    self.current_robot_joints = new_joints
            else:
                self.log_message(f"✗ Movement failed with error: {ret}")
                
        except Exception as e:
            self.log_message(f"✗ Trajectory move error: {str(e)}")
        finally:
            self.enable_buttons()
    
    def init_ft_sensor(self):
        """Initialize FT sensor"""
        try:
            company = 24
            device = 0
            self.robot.FT_SetConfig(company, device)
            self.robot.FT_Activate(0)
            time.sleep(0.5)
            self.robot.FT_Activate(1)
            time.sleep(0.5)
            self.robot.SetLoadWeight(0, 0.0)
            self.robot.SetLoadCoord(0.0, 0.0, 0.0)
            self.robot.FT_SetZero(0)
            time.sleep(0.5)
            self.robot.FT_SetZero(1)
            time.sleep(0.5)
            self.log_message("FT Sensor initialized and zeroed.")
            return True
        except Exception as e:
            self.log_message(f"✗ FT Sensor initialization error: {str(e)}")
            return False
    
    def calibrate_baseline_forces(self):
        """Capture baseline forces for gravity compensation"""
        self.log_message("Calibrating baseline forces (gravity compensation)...")
        
        gravity_compensation_samples = 100
        force_samples = []
        
        for i in range(gravity_compensation_samples):
            try:
                ft_data = self.robot.FT_GetForceTorqueRCS()
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
            except Exception as e:
                self.log_message(f"Error during calibration: {str(e)}")
                break
        
        if force_samples:
            self.baseline_forces = [sum(forces[i] for forces in force_samples) / len(force_samples) 
                              for i in range(6)]
            baseline_str = ", ".join([f"{f:.2f}" for f in self.baseline_forces])
            self.log_message(f"Baseline forces: [{baseline_str}]")
            self.log_message("✓ Gravity compensation calibrated.")
            return True
        else:
            self.log_message("✗ Warning: Could not capture baseline forces!")
            self.baseline_forces = [0.0] * 6
            return False
    
    def start_ft_monitoring(self):
        """Start continuous FT monitoring in background thread"""
        self.monitoring_active = True
        self.monitoring_thread = threading.Thread(target=self.ft_monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        self.log_message("✓ FT monitoring started in background.")
        self.status_label.config(text="Status: Monitoring Active", foreground="green")
        self.btn_record.config(state='normal')
    
    def ft_monitoring_loop(self):
        """Background loop for continuous FT monitoring"""
        dt = 0.008
        deadband = 0.5
        
        while self.monitoring_active:
            try:
                ft_data = self.robot.FT_GetForceTorqueRCS()
                if ft_data[0] != 0:
                    time.sleep(dt)
                    continue
                
                raw_forces = [
                    ft_data[1][0],
                    -ft_data[1][1],
                    ft_data[1][2],
                    ft_data[1][3],
                    ft_data[1][4],
                    ft_data[1][5]
                ]
                
                if self.baseline_forces is not None:
                    forces = [raw_forces[i] - self.baseline_forces[i] for i in range(6)]
                else:
                    forces = raw_forces
                
                for i in range(6):
                    if abs(forces[i]) < deadband:
                        forces[i] = 0.0
                
                time.sleep(dt)
                
            except Exception as e:
                self.log_message(f"Monitoring error: {str(e)}")
                time.sleep(dt)
    
    def record_ft_data(self):
        """Record FT data for selected axis for 5 seconds"""
        if not self.monitoring_active:
            self.log_message("✗ Monitoring not active. Press Elbow Flexion first.")
            return
        
        selected_axis = self.axis_var.get()
        axis_index = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"].index(selected_axis)
        
        self.btn_record.config(state='disabled')
        self.log_message(f"Recording {selected_axis} for 5 seconds...")
        
        thread = threading.Thread(target=self.run_recording, args=(axis_index, selected_axis))
        thread.daemon = True
        thread.start()
    
    def run_recording(self, axis_index, axis_name):
        """Execute 5-second recording and calculate average"""
        try:
            samples = []
            duration = 5.0
            dt = 0.008
            num_samples = int(duration / dt)
            
            for i in range(num_samples):
                ft_data = self.robot.FT_GetForceTorqueRCS()
                if ft_data[0] == 0:
                    raw_forces = [
                        ft_data[1][0],
                        -ft_data[1][1],
                        ft_data[1][2],
                        ft_data[1][3],
                        ft_data[1][4],
                        ft_data[1][5]
                    ]
                    
                    if self.baseline_forces is not None:
                        forces = [raw_forces[j] - self.baseline_forces[j] for j in range(6)]
                    else:
                        forces = raw_forces
                    
                    samples.append(forces[axis_index])
                
                time.sleep(dt)
            
            if samples:
                average = sum(samples) / len(samples)
                self.log_message(f"✓ Recording complete for {axis_name}")
                self.log_message(f"  Samples collected: {len(samples)}")
                self.log_message(f"  Average {axis_name}: {average:.4f} N")
                print(f"\n{'='*50}")
                print(f"RECORDING RESULT - {axis_name}")
                print(f"{'='*50}")
                print(f"Average Force: {average:.4f} N")
                print(f"Samples: {len(samples)}")
                print(f"{'='*50}\n")
            else:
                self.log_message(f"✗ No samples collected for {axis_name}")
                
        except Exception as e:
            self.log_message(f"✗ Recording error: {str(e)}")
        finally:
            self.btn_record.config(state='normal')
    
    def elbow_flexion_action(self):
        """Elbow Flexion - Connect and move to CSV start position"""
        if not self.csv_file_path or not self.csv_start_joints:
            messagebox.showwarning("Warning", "Please select a CSV file first.")
            return
        
        self.disable_buttons()
        self.log_message("Connecting to robot and moving to CSV start position...")
        
        thread = threading.Thread(target=self.run_elbow_flexion)
        thread.daemon = True
        thread.start()
    
    def run_elbow_flexion(self):
        """Execute elbow flexion - move to CSV start position"""
        try:
            sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
            import Robot
            
            self.log_message("Connecting to robot at 192.168.58.2...")
            self.robot = Robot.RPC('192.168.58.2')
            self.log_message("✓ Robot connected.")
            
            # Initialize FT sensor
            if not self.init_ft_sensor():
                self.log_message("✗ Failed to initialize FT sensor")
                self.enable_buttons()
                return
            
            # Get current position
            error, current_pose = self.robot.GetActualJointPosDegree()
            if error != 0:
                self.log_message("✗ Failed to get current position")
                self.enable_buttons()
                return
            
            self.log_message(f"Current Joint Position: {[f'{j:.2f}' for j in current_pose]}")
            
            # Move to CSV start position
            target_joints = self.csv_start_joints.copy()
            
            MAX_JOINT_SPEED = 180.0
            desired_speed = 7.70
            vel_percent = (desired_speed / MAX_JOINT_SPEED) * 100.0
            vel_percent = round(vel_percent, 3)
            
            self.log_message(f"Moving to CSV start position: {[f'{j:.2f}' for j in target_joints]}")
            self.log_message(f"Speed: {desired_speed} deg/s (vel={vel_percent}%)")
            
            ret = self.robot.MoveJ(
                joint_pos=target_joints,
                tool=0,
                user=0,
                desc_pos=[0.0]*7,
                vel=vel_percent,
                acc=0.0,
                ovl=100.0,
                exaxis_pos=[0.0]*4,
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0.0]*6
            )
            
            time.sleep(0.5)
            error, final_pose = self.robot.GetActualJointPosDegree()
            if error == 0:
                self.current_robot_joints = final_pose
                self.log_message(f"Final Joint Position: {[f'{j:.2f}' for j in final_pose]}")
            
            if ret == 0:
                self.log_message("✓ MoveJ to CSV start succeeded!")
            else:
                self.log_message(f"✗ MoveJ failed with error code: {ret}")
                self.enable_buttons()
                return
            
            # Calibrate baseline forces
            time.sleep(1.0)
            if not self.calibrate_baseline_forces():
                self.log_message("✗ Failed to calibrate baseline forces")
                self.enable_buttons()
                return
            
            # Start FT monitoring
            self.start_ft_monitoring()
            
        except Exception as e:
            self.log_message(f"✗ Error: {str(e)}")
        finally:
            self.enable_buttons()
    
    def button3_action(self):
        """Button 3 action - Add your code here"""
        self.disable_buttons()
        self.log_message("Button 3 pressed: Executing command...")
        
        thread = threading.Thread(target=self.run_button3_code)
        thread.daemon = True
        thread.start()
    
    def run_button3_code(self):
        """Add your robot code for button 3 here"""
        try:
            self.log_message("Button 3 functionality not yet implemented")
            time.sleep(1)
            
        except Exception as e:
            self.log_message(f"✗ Error: {str(e)}")
        finally:
            self.enable_buttons()
    
    def on_closing(self):
        """Clean shutdown when window is closed"""
        self.monitoring_active = False
        if self.drag_mode_active and self.robot:
            try:
                self.robot.DragTeachSwitch(0)
            except:
                pass
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=1.0)
        self.root.destroy()

def main():
    root = tk.Tk()
    app = RobotControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()