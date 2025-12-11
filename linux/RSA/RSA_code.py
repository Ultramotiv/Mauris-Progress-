import tkinter as tk
from tkinter import ttk, scrolledtext
import sys
import threading
from datetime import datetime
import time

class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control Panel with FT Monitoring")
        self.root.geometry("700x600")
        self.root.resizable(True, True)
        
        # Robot and FT monitoring variables
        self.robot = None
        self.baseline_forces = None
        self.monitoring_active = False
        self.monitoring_thread = None
        
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
        main_frame.rowconfigure(2, weight=1)
        
        # Title
        title_label = ttk.Label(main_frame, text="Robot Control Panel", 
                                font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, pady=10)
        
        # Button frame
        button_frame = ttk.Frame(main_frame, padding="10")
        button_frame.grid(row=1, column=0, pady=10)
        
        # Create buttons
        self.btn_home = ttk.Button(button_frame, text="Elbow Flexion", 
                                    command=self.home_position_action, width=20)
        self.btn_home.grid(row=0, column=0, padx=10, pady=10)
        
        self.btn2 = ttk.Button(button_frame, text="Button 2", 
                               command=self.button2_action, width=20)
        self.btn2.grid(row=0, column=1, padx=10, pady=10)
        
        self.btn3 = ttk.Button(button_frame, text="Button 3", 
                               command=self.button3_action, width=20)
        self.btn3.grid(row=0, column=2, padx=10, pady=10)
        
        # FT Recording frame
        ft_frame = ttk.LabelFrame(main_frame, text="Force/Torque Recording", padding="10")
        ft_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=10)
        
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
        self.btn_record.config(state='disabled')  # Disabled until monitoring starts
        
        # Status indicator
        self.status_label = ttk.Label(ft_frame, text="Status: Not Connected", 
                                      foreground="red")
        self.status_label.grid(row=0, column=3, padx=10, pady=5)
        
        # Status log frame
        log_frame = ttk.LabelFrame(main_frame, text="Status Log", padding="10")
        log_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=10)
        
        # Text widget for output
        self.output_text = scrolledtext.ScrolledText(log_frame, 
                                                      height=15, 
                                                      width=80,
                                                      wrap=tk.WORD)
        self.output_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Clear log button
        clear_btn = ttk.Button(main_frame, text="Clear Log", 
                               command=self.clear_log, width=15)
        clear_btn.grid(row=4, column=0, pady=5)
        
        self.log_message("System initialized. Press 'Home Position' to connect and start monitoring.")
    
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
    
    def disable_buttons(self):
        """Disable all buttons during operation"""
        self.btn_home.config(state='disabled')
        self.btn2.config(state='disabled')
        self.btn3.config(state='disabled')
    
    def enable_buttons(self):
        """Enable all buttons after operation"""
        self.btn_home.config(state='normal')
        self.btn2.config(state='normal')
        self.btn3.config(state='normal')
    
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
                if ft_data[0] == 0:  # No error
                    forces = [
                        ft_data[1][0],   # Fx
                        -ft_data[1][1],  # Fy (inverted)
                        ft_data[1][2],   # Fz
                        ft_data[1][3],   # Mx
                        ft_data[1][4],   # My
                        ft_data[1][5]    # Mz
                    ]
                    force_samples.append(forces)
                time.sleep(0.01)  # 10ms between samples
            except Exception as e:
                self.log_message(f"Error during calibration: {str(e)}")
                break
        
        # Calculate average baseline forces
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
                
                # Get raw forces
                raw_forces = [
                    ft_data[1][0],   # Fx
                    -ft_data[1][1],  # Fy (inverted)
                    ft_data[1][2],   # Fz
                    ft_data[1][3],   # Mx
                    ft_data[1][4],   # My
                    ft_data[1][5]    # Mz
                ]
                
                # Apply gravity compensation
                if self.baseline_forces is not None:
                    forces = [raw_forces[i] - self.baseline_forces[i] for i in range(6)]
                else:
                    forces = raw_forces
                
                # Apply deadband to reduce noise
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
            self.log_message("✗ Monitoring not active. Press Home Position first.")
            return
        
        selected_axis = self.axis_var.get()
        axis_index = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"].index(selected_axis)
        
        self.btn_record.config(state='disabled')
        self.log_message(f"Recording {selected_axis} for 5 seconds...")
        
        # Run recording in separate thread
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
                if ft_data[0] == 0:  # No error
                    raw_forces = [
                        ft_data[1][0],   # Fx
                        -ft_data[1][1],  # Fy (inverted)
                        ft_data[1][2],   # Fz
                        ft_data[1][3],   # Mx
                        ft_data[1][4],   # My
                        ft_data[1][5]    # Mz
                    ]
                    
                    # Apply gravity compensation
                    if self.baseline_forces is not None:
                        forces = [raw_forces[j] - self.baseline_forces[j] for j in range(6)]
                    else:
                        forces = raw_forces
                    
                    samples.append(forces[axis_index])
                
                time.sleep(dt)
            
            # Calculate average
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
    
    def home_position_action(self):
        """Home Position - Connect, move, and start monitoring"""
        self.disable_buttons()
        self.log_message("Connecting to robot and moving to Home Position...")
        
        # Run in separate thread
        thread = threading.Thread(target=self.run_home_position)
        thread.daemon = True
        thread.start()
    
    def run_home_position(self):
        """Execute the home position and start FT monitoring"""
        try:
            # Import robot library
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
            current_pose = self.robot.GetActualJointPosDegree(flag=1)
            self.log_message(f"Current Joint Position: {current_pose}")
            
            # Move to home position
            target_joints = [2.608, -88.384, 127.473, -137.27, -92.275, -90.118]
            
            MAX_JOINT_SPEED = 180.0
            desired_speed = 7.70
            vel_percent = (desired_speed / MAX_JOINT_SPEED) * 100.0
            ovl_percent = 100.0
            vel_percent = round(vel_percent, 3)
            
            self.log_message(f"Moving with {desired_speed} deg/s (vel={vel_percent}%, ovl={ovl_percent}%)")
            
            ret = self.robot.MoveJ(
                joint_pos=target_joints,
                tool=0,
                user=0,
                desc_pos=[0.0]*7,
                vel=vel_percent,
                acc=0.0,
                ovl=ovl_percent,
                exaxis_pos=[0.0]*4,
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0.0]*6
            )
            
            time.sleep(0.5)
            current_pose = self.robot.GetActualJointPosDegree(flag=1)
            self.log_message(f"Final Joint Position: {current_pose}")
            
            if ret == 0:
                self.log_message("✓ MoveJ command succeeded!")
            else:
                self.log_message(f"✗ MoveJ failed with error code: {ret}")
                self.enable_buttons()
                return
            
            # Calibrate baseline forces
            time.sleep(1.0)  # Wait for robot to settle
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
    
    def button2_action(self):
        """Button 2 action - Add your code here"""
        self.disable_buttons()
        self.log_message("Button 2 pressed: Executing command...")
        
        thread = threading.Thread(target=self.run_button2_code)
        thread.daemon = True
        thread.start()
    
    def run_button2_code(self):
        """Add your robot code for button 2 here"""
        try:
            self.log_message("Button 2 functionality not yet implemented")
            time.sleep(1)
            
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