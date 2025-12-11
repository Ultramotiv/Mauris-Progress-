# -*- coding: utf-8 -*-
"""
Simple PyQt6 UI to load trajectories from a CSV file,
select one, allow user input for speed, blend, and repetitions,
and run it multiple times using the Robot library in a separate thread,
showing live count and using GetRobotMotionDone() to check for completion.
Includes Stop/Pause/Resume controls and enhanced logging.
"""

import sys
import time
import traceback # Import traceback for detailed error printing
import csv
import os
from PyQt6.QtCore import Qt, QObject, QThread, pyqtSignal, pyqtSlot, QMetaObject
# --- Robot Import ---
try:
    # Assume Robot library exists with the required methods
    import Robot
    print("Successfully imported Robot library.")
    ROBOT_AVAILABLE = True
except ImportError:
    print("-----------------------------------------------------------")
    print("WARNING: 'Robot' library not found.")
    print("         Robot communication will be simulated.")
    print("-----------------------------------------------------------")
    ROBOT_AVAILABLE = False
    # Dummy Robot for UI testing without library
    class Robot:
        class RPC:
            _dummy_motion_counter = 0
            _dummy_motion_finish_at = 8 # Make it slightly longer for testing pause/resume
            _dummy_is_paused = False
            _dummy_is_stopped = False

            def __init__(self, ip): print(f"Info: Dummy Robot created for IP {ip}")
            def LoadTPD(self, name): print(f"Sim: Loading TPD '{name}'..."); time.sleep(0.3); print(f"Sim: LoadTPD: {name} done."); return 0
            def GetTPDStartPose(self, name): print(f"Sim: Getting Start Pose for '{name}'..."); time.sleep(0.2); print(f"Sim: GetTPDStartPose: {name} done."); return 0, [0.0]*6
            def MoveL(self, pose, v, blend): print(f"Sim: MoveL to {pose[:2]}... V={v}, B={blend}"); time.sleep(0.5); print("Sim: MoveL done."); return 0
            def SetSpeed(self, speed): print(f"Sim: SetSpeed: {speed}"); time.sleep(0.1); return 0
            def MoveTPD(self, name, blend, ovl):
                print(f"Sim: MoveTPD: {name}, Blnd={blend}, Ovl={ovl}");
                self._dummy_motion_counter = 0
                self._dummy_is_paused = False
                self._dummy_is_stopped = False
                return 0
            def GetRobotMotionDone(self):
                # Simulate work/polling delay
                time.sleep(MOTION_CHECK_INTERVAL / 2) # Simulate check time

                if self._dummy_is_stopped:
                    print(f"Sim: GetRobotMotionDone: Stopped (1)"); return 0, 1
                if self._dummy_is_paused:
                    print(f"Sim: GetRobotMotionDone: Paused (0, Cnt:{self._dummy_motion_counter})"); return 0, 0

                self._dummy_motion_counter += 1
                print(f"Sim: GetRobotMotionDone: Checking... (Cnt:{self._dummy_motion_counter})")
                if self._dummy_motion_counter >= self._dummy_motion_finish_at:
                    print(f"Sim: GetRobotMotionDone: Finished (1)"); return 0, 1
                else:
                    return 0, 0 # Not done

            def StopMotion(self):
                print(f"Sim: StopMotion called.");
                self._dummy_is_stopped = True
                time.sleep(0.1) # Simulate command time
                return 0
            def PauseMotion(self):
                print(f"Sim: PauseMotion called.");
                if self._dummy_is_stopped: print("  (Sim: Motion stopped, cannot pause)"); return -1
                self._dummy_is_paused = True
                time.sleep(0.1)
                return 0
            def ResumeMotion(self):
                print(f"Sim: ResumeMotion called.");
                if self._dummy_is_stopped: print("  (Sim: Motion stopped, cannot resume)"); return -1
                if not self._dummy_is_paused: print("  (Sim: Motion not paused)"); return 0 # Allow resuming if not paused
                self._dummy_is_paused = False
                time.sleep(0.1)
                return 0

            def __getattr__(self, name):
                print(f"Warning: Dummy Robot - Called undefined method: {name}")
                return lambda *a, **k: -1

# --- UI Imports ---
from PyQt6.QtWidgets import ( QApplication, QWidget, QPushButton, QVBoxLayout, QLabel,
                              QMessageBox, QComboBox, QHBoxLayout, QSpacerItem,
                              QSizePolicy, QSpinBox, QCheckBox, QFormLayout )
from PyQt6.QtGui import QFont
from PyQt6.QtCore import Qt, QObject, QThread, pyqtSignal, pyqtSlot

# --- Configuration ---
ROBOT_IP = '192.168.58.2'
TRAJECTORY_CSV_FILE = 'trajectory_log.csv'
DEFAULT_SPEED = 100; DEFAULT_BLEND = 1; DEFAULT_REPETITIONS = 1
WAIT_AT_START = 1; WAIT_BETWEEN_REPS = 0.5; MOTION_CHECK_INTERVAL = 0.25 # Used by worker

# =============================================================================
# Worker Object for Running Trajectory in Separate Thread
# =============================================================================
class TrajectoryWorker(QObject):
    """Handles robot communication and trajectory execution in a separate thread."""
    # Signals to communicate back to the main UI thread
    status_update = pyqtSignal(str)       # Signal to update the status label
    rep_update = pyqtSignal(int, int)     # Signal to update repetition count (current, total)
    finished = pyqtSignal(bool, str)      # Signal when execution completes (success:True/False, message:str)
    error = pyqtSignal(str, str)          # Signal on error (title:str, message:str)

    def __init__(self, robot_instance):
        super().__init__()
        self.robot = robot_instance # Use the robot instance passed from the main thread
        self._stop_requested = False
        # Parameters to be set before running
        self.name = ""
        self.speed_val = 0
        self.blend_val = 0
        self.total_repetitions = 0
        self.ovl = 0.0

    @pyqtSlot(str, int, int, int, float)
    def configure_run(self, name, speed, blend, reps, ovl):
        """Sets the parameters for the next trajectory run."""
        self.name = name
        self.speed_val = speed
        self.blend_val = blend
        self.total_repetitions = reps
        self.ovl = ovl
        self._stop_requested = False # Reset stop flag for new run config
        print(f"Worker configured for: {name}, Reps={reps}, Spd={speed}, Blnd={blend}")

    @pyqtSlot()
    def request_stop(self):
        """Slot called by the main thread to signal the worker to stop."""
        print("Worker received stop request.")
        self._stop_requested = True

    @pyqtSlot()
    def run_trajectory(self):
        """The main execution logic, runs in the worker thread."""
        if not self.robot:
            self.error.emit("Worker Error", "Robot object not available in worker.")
            self.finished.emit(False, "Worker Error: No Robot")
            return
        if not self.name:
            self.error.emit("Worker Error", "Trajectory name not configured in worker.")
            self.finished.emit(False, "Worker Error: No Name")
            return

        print(f"--- Worker Thread: Starting Sequence: '{self.name}' ---")
        self.status_update.emit(f"Status: Preparing '{self.name}'...")

        start_pose = None
        motion_check = hasattr(self.robot, 'GetRobotMotionDone')
        current_rep = 0
        error_occurred = False
        final_message = ""

        try:
            # --- Preparation ---
            print("Worker DEBUG: Calling LoadTPD..."); ret = self.robot.LoadTPD(self.name); print(f"  LoadTPD ret={ret}")
            if self._check_stop_and_error(ret, "LoadTPD failed"): return

            print("Worker DEBUG: Calling GetTPDStartPose..."); ret_p, start_pose = self.robot.GetTPDStartPose(self.name); print(f"  GetStartPose ret={ret_p}, pose={start_pose}")
            if self._check_stop_and_error(ret_p, "GetTPDStartPose failed"): return
            if not start_pose:
                 if self._check_stop_and_error(-1, "GetTPDStartPose returned an empty pose"): return # Use -1 or similar for non-robot errors

            print("Worker DEBUG: Calling SetSpeed..."); ret_speed = self.robot.SetSpeed(self.ovl); print(f"  SetSpeed ret={ret_speed}")
            if self._check_stop_and_error(ret_speed, "SetSpeed failed"): return

            # --- Repetition Loop ---
            for i in range(self.total_repetitions):
                if self._stop_requested:
                    final_message = f"Stopped by User during '{self.name}'"
                    print("Worker INFO: Stop requested, aborting repetition loop.")
                    break # Exit the loop

                current_rep = i + 1
                print(f"\n--- Worker Thread: Rep {current_rep}/{self.total_repetitions} ---")
                self.status_update.emit(f"Status: Running '{self.name}' (Rep {current_rep}/{self.total_repetitions})")
                self.rep_update.emit(current_rep, self.total_repetitions)

                try: # Inner try for robot commands within a rep
                    if self._stop_requested: break

                    print("Worker DEBUG: Calling MoveL to start pose..."); ret_move = self.robot.MoveL(start_pose, 0, 0); print(f"  MoveL ret={ret_move}")
                    if self._check_stop_and_error(ret_move, f"MoveL failed (Rep {current_rep})"): return
                    self._wait_interruptible(WAIT_AT_START)

                    if self._stop_requested: break

                    print("Worker DEBUG: Calling MoveTPD..."); ret_tpd = self.robot.MoveTPD(self.name, self.blend_val, self.ovl); print(f"  MoveTPD ret={ret_tpd}")
                    if self._check_stop_and_error(ret_tpd, f"MoveTPD failed (Rep {current_rep})"): return

                    # --- Wait for Motion Completion ---
                    if motion_check:
                        print("Worker DEBUG: Waiting for motion completion..."); self.status_update.emit(f"Status: Executing '{self.name}' (Rep {current_rep}/{self.total_repetitions})")
                        polling_start_time = time.time()
                        while not self._stop_requested:
                            # Use a short sleep even in the worker to prevent busy-waiting 100% CPU if GetRobotMotionDone is fast
                            time.sleep(MOTION_CHECK_INTERVAL / 5) # Shorter sleep, UI thread is separate
                            print("Worker DEBUG: Calling GetRobotMotionDone...")
                            try:
                                ret_state, state = self.robot.GetRobotMotionDone()
                                print(f"  GMD ret={ret_state}, state={state}")
                                if ret_state != 0:
                                    print(f"Worker WARNING: GetRobotMotionDone error ({ret_state}). Stopping motion check for this rep.")
                                    break # Exit GMD loop on error
                                if state == 1:
                                    print("Worker DEBUG: Motion complete."); break # Success
                                if time.time() - polling_start_time > 600: # 10 min timeout
                                     if self._check_stop_and_error(-1, f"GetRobotMotionDone timed out (Rep {current_rep})"): return
                            except Exception as gmd_e:
                                print(f"Worker ERROR during GetRobotMotionDone call: {gmd_e}")
                                if self._check_stop_and_error(-1, f"Exception in GetRobotMotionDone (Rep {current_rep})"): return
                                break # Stop polling on GMD call error

                    else:
                        print("Worker DEBUG: GetRobotMotionDone not available, simulating wait."); self._wait_interruptible(3)

                    if self._stop_requested: break

                    print(f"--- Worker Thread: Rep {current_rep} finished ---")
                    if current_rep < self.total_repetitions and WAIT_BETWEEN_REPS > 0:
                        print(f"Worker DEBUG: Waiting between reps ({WAIT_BETWEEN_REPS}s)...")
                        self.status_update.emit(f"Status: Paused between reps ({current_rep}/{self.total_repetitions})")
                        self._wait_interruptible(WAIT_BETWEEN_REPS)

                except Exception as rep_e: # Catch error within the inner try
                     # This shouldn't normally be reached if _check_stop_and_error handles exceptions
                     print(f"Worker UNEXPECTED ERROR during Rep {current_rep}: {rep_e}\n{traceback.format_exc()}")
                     self._handle_error(f"Unexpected Error on Rep {current_rep}", f"{rep_e}")
                     error_occurred = True
                     return # Exit run_trajectory

            # --- End Repetition Loop ---
            if not final_message and not error_occurred: # If loop finished naturally
                 final_message = f"'{self.name}' Completed ({self.total_repetitions} reps)"
                 print(f"\n--- Worker Thread: Sequence '{self.name}' Completed Successfully ---")

        except Exception as e: # Catch errors during preparation or unexpected loop exit
            print(f"Worker FATAL ERROR during execution: {e}\n{traceback.format_exc()}")
            rep_str = f" on rep {current_rep}" if current_rep > 0 else " during preparation"
            self._handle_error(f"Execution Error{rep_str}", f"{e}")
            error_occurred = True
            return # Exit run_trajectory

        finally:
            # Signal finished state to main thread
            success = not error_occurred and not self._stop_requested
            print(f"Worker finishing. Success={success}, Msg='{final_message}'")
            self.finished.emit(success, final_message)


    def _check_stop_and_error(self, return_code, error_message_base):
        """Helper to check stop flag and robot return codes, emitting signals on error."""
        if self._stop_requested:
            print("Worker INFO: Stop detected during operation check.")
            self.finished.emit(False, f"Stopped by User during '{error_message_base}'")
            return True # Indicate an exit condition met

        if return_code != 0:
            full_error = f"{error_message_base} (Robot Code: {return_code})"
            print(f"Worker ERROR: {full_error}")
            self._handle_error("Robot Command Error", full_error)
            return True # Indicate an exit condition met
        return False # No stop, no error

    def _handle_error(self, title, message):
        """Emits error signal and prepares finish signal."""
        self.error.emit(title, message)
        # self.finished signal will be emitted in the finally block

    def _wait_interruptible(self, duration_sec):
        """Pauses execution, checking stop flag frequently."""
        if duration_sec <= 0: return
        end_time = time.time() + duration_sec
        while time.time() < end_time:
            if self._stop_requested:
                print("Worker DEBUG: Stop requested during wait.")
                break
            # Sleep for a short interval
            time.sleep(min(0.05, end_time - time.time())) # Sleep briefly


# =============================================================================
# Main Application Window
# =============================================================================
class TrajectoryRunnerApp(QWidget):
    # Signal to trigger worker configuration and start
    trigger_run = pyqtSignal(str, int, int, int, float)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trajectory Runner (Threaded)")
        self.setGeometry(300, 300, 500, 470)

        self.robot = None # Robot connection object (RPC)
        self.worker = None # Worker QObject
        self.thread = None # QThread for the worker

        # --- Initialize UI Elements ---
        # (Initialize all to None first for robustness)
        self.trajectory_combo = None; self.run_button = None; self.status_label = None
        self.speed_input = None; self.blend_checkbox = None; self.repetitions_input = None
        self.rep_count_label = None; self.stop_button = None; self.pause_button = None
        self.resume_button = None; self.refresh_button = None
        # --- End UI Elements Init ---

        self._is_running = False # UI state flag, worker manages actual execution

        self._init_ui()          # Create UI widgets
        self._connect_robot()    # Establish robot connection
        self._setup_worker_thread() # Create and connect worker/thread
        self._load_trajectory_names() # Load initial list

    def _init_ui(self):
        """Initializes the user interface elements."""
        main_layout = QVBoxLayout(self)
        # Status Label
        self.status_label = QLabel("Status: Initializing...")
        font_status = QFont(); font_status.setPointSize(10); self.status_label.setFont(font_status)
        main_layout.addWidget(self.status_label)

        # Trajectory Selection
        combo_layout = QHBoxLayout(); combo_label = QLabel("Select Trajectory:")
        self.trajectory_combo = QComboBox(); self.trajectory_combo.setToolTip("Select recorded exercise"); self.trajectory_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.trajectory_combo.addItem("<Load trajectories first>"); self.trajectory_combo.setEnabled(False); self.trajectory_combo.currentIndexChanged.connect(lambda: self._update_ui_state()) # Simple update on change
        self.refresh_button = QPushButton("Refresh List"); self.refresh_button.setToolTip("Reload names from CSV"); self.refresh_button.clicked.connect(self._load_trajectory_names)
        combo_layout.addWidget(combo_label); combo_layout.addWidget(self.trajectory_combo, 1); combo_layout.addWidget(self.refresh_button); main_layout.addLayout(combo_layout)

        # Parameter Inputs
        param_layout = QFormLayout()
        self.speed_input = QSpinBox(); self.speed_input.setRange(1, 150); self.speed_input.setValue(DEFAULT_SPEED); self.speed_input.setSuffix(" %"); self.speed_input.setToolTip("Set speed override (1-150)"); param_layout.addRow("Speed Override:", self.speed_input)
        self.repetitions_input = QSpinBox(); self.repetitions_input.setRange(1, 100); self.repetitions_input.setValue(DEFAULT_REPETITIONS); self.repetitions_input.setToolTip("Set number of repetitions"); param_layout.addRow("Repetitions:", self.repetitions_input)
        self.blend_checkbox = QCheckBox("Smooth Motion (Blend)"); self.blend_checkbox.setChecked(DEFAULT_BLEND == 1); self.blend_checkbox.setToolTip("Check for blend=1, uncheck for blend=0"); param_layout.addRow(self.blend_checkbox); main_layout.addLayout(param_layout)

        # Live Repetition Count Label
        self.rep_count_label = QLabel("Rep: - / -"); font_rep = QFont(); font_rep.setPointSize(10); font_rep.setBold(True); self.rep_count_label.setFont(font_rep); self.rep_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter); main_layout.addWidget(self.rep_count_label)

        # Run Button
        self.run_button = QPushButton("Run Selected Trajectory"); self.run_button.setMinimumHeight(40); font_run = QFont(); font_run.setPointSize(11); font_run.setBold(True); self.run_button.setFont(font_run)
        self.run_button.setToolTip("Load and execute trajectory"); self.run_button.clicked.connect(self._start_trajectory_run); self.run_button.setEnabled(False); main_layout.addWidget(self.run_button)

        # Control Buttons
        control_button_layout = QHBoxLayout()
        self.stop_button = QPushButton("STOP"); self.stop_button.setToolTip("Immediately stop robot motion (StopMotion)"); self.stop_button.setStyleSheet("background-color: #DC143C; color: white; font-weight: bold;"); self.stop_button.clicked.connect(self._stop_motion); self.stop_button.setEnabled(False) # Crimson Red
        self.pause_button = QPushButton("Pause"); self.pause_button.setToolTip("Pause robot motion (PauseMotion)"); self.pause_button.setStyleSheet("background-color: #FFA500;"); self.pause_button.clicked.connect(self._pause_motion); self.pause_button.setEnabled(False) # Orange
        self.resume_button = QPushButton("Resume"); self.resume_button.setToolTip("Resume paused robot motion (ResumeMotion)"); self.resume_button.setStyleSheet("background-color: #90EE90;"); self.resume_button.clicked.connect(self._resume_motion); self.resume_button.setEnabled(False) # Light Green
        control_button_layout.addWidget(self.stop_button)
        control_button_layout.addWidget(self.pause_button)
        control_button_layout.addWidget(self.resume_button)
        main_layout.addLayout(control_button_layout)

        main_layout.addSpacerItem(QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)) # Reduced spacer
        self.setLayout(main_layout)
        print("DEBUG: UI Initialization complete.")

    def _connect_robot(self):
        """Attempts to establish a connection with the robot."""
        # This still happens in the main thread at startup
        if not ROBOT_AVAILABLE:
            self.status_label.setText("Status: <b style='color: orange;'>Robot library not found (Simulating)</b>")
            try: self.robot = Robot.RPC(ROBOT_IP)
            except Exception as e: self.robot=None; print(f"ERROR creating dummy robot: {e}")
            self._update_ui_state(); return

        try:
            print(f"Attempting to connect to robot at {ROBOT_IP}..."); self.status_label.setText("Status: Connecting..."); QApplication.processEvents()
            self.robot = Robot.RPC(ROBOT_IP); print("Robot connection successful."); self.status_label.setText("Status: <b style='color: green;'>Connected</b>")
        except Exception as e:
            self.robot = None; print(f"FATAL: Failed to connect to robot: {e}"); self.status_label.setText("Status: <b style='color: red;'>Connection Failed</b>")
            QMessageBox.critical(self, "Connection Error", f"Could not connect to robot at {ROBOT_IP}:\n{e}\n\nCheck IP address and network connection."); print(f"DEBUG: Connect Robot Exception:\n{traceback.format_exc()}")
        finally: self._update_ui_state()

    def _setup_worker_thread(self):
        """Creates the worker object and thread, connects signals/slots."""
        if not self.robot:
             print("WARNING: Cannot setup worker thread without robot connection.")
             # Optionally disable run button here if it wasn't already
             if self.run_button: self.run_button.setEnabled(False)
             return

        print("DEBUG: Setting up worker thread...")
        self.thread = QThread(self) # Parent thread to app for proper cleanup
        self.worker = TrajectoryWorker(self.robot) # Pass robot instance
        self.worker.moveToThread(self.thread)

        # Connect worker signals to UI slots
        self.worker.status_update.connect(self._update_status_label)
        self.worker.rep_update.connect(self._update_rep_label)
        self.worker.finished.connect(self._on_worker_finished)
        self.worker.error.connect(self._on_worker_error)

        # Connect trigger signal from UI thread to worker's configure slot
        self.trigger_run.connect(self.worker.configure_run)
        # Connect configure slot completion (implicitly) to run_trajectory
        # Better: connect thread started to run_trajectory IF we start/stop thread per run OR trigger run explicitly
        # Let's explicitly trigger run_trajectory after configuration via a signal or invokeMethod
        # Connecting trigger_run directly to configure_run, and then calling run_trajectory after emit works fine.
        # Make sure run_trajectory is marked as a slot.
        # No, easier: trigger_run configures, then we call invokeMethod to start run_trajectory.

        # Connect thread finished signal for cleanup (optional but good practice)
        self.thread.finished.connect(self.worker.deleteLater) # Schedule worker for deletion
        self.thread.finished.connect(self.thread.deleteLater) # Schedule thread for deletion

        self.thread.start()
        print("DEBUG: Worker thread started.")


    def _load_trajectory_names(self):
        """Reads trajectory names from the CSV log file (runs in main thread)."""
        if self._is_running: print("INFO: Cannot refresh list while running."); return
        if not self.trajectory_combo: print("ERROR: trajectory_combo is None, cannot load names."); return
        print(f"Loading trajectories from '{TRAJECTORY_CSV_FILE}'...")
        # --- Same CSV loading logic as before ---
        current_selection = self.trajectory_combo.currentText() # Remember selection
        self.trajectory_combo.clear()
        if not os.path.isfile(TRAJECTORY_CSV_FILE):
            self.trajectory_combo.addItem("<Log file not found>"); self.trajectory_combo.setEnabled(False); self._update_ui_state(); return
        names = set(); name_col_idx = 1
        try:
            with open(TRAJECTORY_CSV_FILE, 'r', newline='', encoding='utf-8') as f:
                r = csv.reader(f); h = next(r, None)
                if h: hdr=[c.lower().strip() for c in h]; name_col_idx = hdr.index('trajectoryname') if 'trajectoryname' in hdr else 1
                for row in r: (names.add(n) if len(row)>name_col_idx and (n:=row[name_col_idx].strip()) else None)
            if names: self.trajectory_combo.addItems(["<Select>"] + sorted(list(names))); self.trajectory_combo.setEnabled(True) # Add placeholder
            else: self.trajectory_combo.addItem("<No trajectories in log>"); self.trajectory_combo.setEnabled(False)
            # Try to restore selection
            index = self.trajectory_combo.findText(current_selection)
            if index != -1: self.trajectory_combo.setCurrentIndex(index)
            else: self.trajectory_combo.setCurrentIndex(0) # Select placeholder

        except Exception as e:
            print(f"ERROR reading CSV: {e}\n{traceback.format_exc()}"); self.trajectory_combo.addItem("<Error reading log>"); self.trajectory_combo.setEnabled(False); QMessageBox.warning(self, "File Error", f"Could not read log:\n{e}")
        finally:
            self._update_ui_state() # Update UI state after loading


    def _update_ui_state(self):
        """Updates UI element enable/disable state based on connection and running status."""
        # This runs in the main thread
        if not hasattr(self, 'status_label') or not self.status_label: return # UI not ready

        robot_connected_and_worker_ready = self.robot is not None and self.worker is not None and self.thread is not None and self.thread.isRunning()

        valid_traj_selected = False
        if self.trajectory_combo:
            cur = self.trajectory_combo.currentText()
            valid_traj_selected = bool(cur) and not cur.startswith("<") # Check if not placeholder

        can_run = robot_connected_and_worker_ready and valid_traj_selected and not self._is_running
        can_edit_params = robot_connected_and_worker_ready and not self._is_running
        can_control_motion = robot_connected_and_worker_ready and self._is_running

        # Safely update widgets using getattr
        getattr(self.run_button, 'setEnabled', lambda x: None)(can_run)
        getattr(self.trajectory_combo, 'setEnabled', lambda x: None)(can_edit_params)
        getattr(self.refresh_button, 'setEnabled', lambda x: None)(can_edit_params)
        getattr(self.speed_input, 'setEnabled', lambda x: None)(can_edit_params)
        getattr(self.blend_checkbox, 'setEnabled', lambda x: None)(can_edit_params)
        getattr(self.repetitions_input, 'setEnabled', lambda x: None)(can_edit_params)

        getattr(self.stop_button, 'setEnabled', lambda x: None)(can_control_motion)
        getattr(self.pause_button, 'setEnabled', lambda x: None)(can_control_motion)
        getattr(self.resume_button, 'setEnabled', lambda x: None)(can_control_motion)

        # Update status text only if idle and connected
        current_status = self.status_label.text()
        is_idle_state = not self._is_running and "Connected" not in current_status and "Simulating" not in current_status and "Failed" not in current_status
        if is_idle_state and robot_connected_and_worker_ready:
             status_text = "<b style='color: green;'>Connected & Ready</b>" if ROBOT_AVAILABLE else "<b style='color: orange;'>Robot Simulated & Ready</b>"
             self._update_status_label(f"Status: {status_text}")
        elif is_idle_state and not robot_connected_and_worker_ready:
            self._update_status_label("Status: <b style='color: red;'>Disconnected or Worker Error</b>")

        # Reset rep count if not running
        if not self._is_running and self.rep_count_label:
             self._update_rep_label(0, 0) # Use specific slot for update


    # --- Slots for Worker Signals ---

    @pyqtSlot(str)
    def _update_status_label(self, text):
        """Slot to update the status label from the worker thread."""
        if self.status_label: self.status_label.setText(text)

    @pyqtSlot(int, int)
    def _update_rep_label(self, current, total):
        """Slot to update the repetition count label."""
        if self.rep_count_label:
             if total > 0 and current > 0: self.rep_count_label.setText(f"Rep: {current} / {total}")
             else: self.rep_count_label.setText("Rep: - / -")

    @pyqtSlot(bool, str)
    def _on_worker_finished(self, success, message):
        """Slot called when the worker thread finishes execution."""
        print(f"Main thread received worker finished signal. Success: {success}, Message: {message}")
        self._is_running = False # Update UI state flag
        if success:
            self._update_status_label(f"Status: <b style='color: blue;'>{message}</b>")
        elif "Stopped by User" in message:
             self._update_status_label(f"Status: <b style='color: orange;'>{message}</b>")
        # Error case handled by _on_worker_error, but could add fallback here
        elif not success and not message: # Generic failure if no specific message
             self._update_status_label("Status: <b style='color: red;'>Execution Failed or Stopped</b>")

        self._update_ui_state() # Re-enable UI elements

    @pyqtSlot(str, str)
    def _on_worker_error(self, title, message):
        """Slot called when the worker thread emits an error."""
        print(f"Main thread received worker error: {title} - {message}")
        self._is_running = False # Update UI state flag
        self._update_status_label(f"Status: <b style='color: red;'>Error: {title}</b>")
        QMessageBox.critical(self, title, message) # Show error popup
        self._update_ui_state() # Re-enable UI elements


    # --- UI Action Handlers ---

    def _start_trajectory_run(self):
        """Starts the trajectory execution by configuring and triggering the worker."""
        if self._is_running: print("INFO: Already running."); return
        if not self.robot or not self.worker or not self.thread or not self.thread.isRunning():
             QMessageBox.critical(self, "Error", "Robot not connected or worker thread not ready."); return

        name = self.trajectory_combo.currentText()
        if not name or name.startswith("<"):
             QMessageBox.warning(self, "Selection Error", "Please select a valid trajectory."); return

        speed_val = self.speed_input.value()
        blend_val = 1 if self.blend_checkbox.isChecked() else 0
        total_repetitions = self.repetitions_input.value()
        ovl = float(speed_val) # Speed override percentage

        print("Main thread: Configuring worker and triggering run...")
        self._is_running = True
        self._update_ui_state() # Disable run button, enable stop etc.
        self._update_status_label(f"Status: Sending '{name}' to worker...")
        self._update_rep_label(0, total_repetitions) # Show total reps immediately

        # Emit signal to configure worker (this runs configure_run in worker thread)
        self.trigger_run.emit(name, speed_val, blend_val, total_repetitions, ovl)

        # Invoke the run_trajectory method in the worker thread's event loop
        # This ensures it runs after configure_run has likely finished setting parameters
        QMetaObject.invokeMethod(self.worker, "run_trajectory", Qt.ConnectionType.QueuedConnection)


    def _stop_motion(self):
        """Requests the worker thread to stop and sends StopMotion command."""
        if not self._is_running: print("INFO: Stop ignored, not running."); return
        if not self.robot or not self.worker: print("ERROR: Robot/Worker not available for stop."); return

        print("Main thread: Requesting STOP.")
        self._update_status_label("Status: <b style='color: red;'>Stopping...</b>")

        # Tell the worker thread to stop its loop
        QMetaObject.invokeMethod(self.worker, "request_stop", Qt.ConnectionType.QueuedConnection)

        # Also send the command directly to the robot for immediate effect (if possible)
        try:
            print("Main thread: Sending StopMotion command directly to robot...")
            err = self.robot.StopMotion()
            print(f"  Direct StopMotion returned: {err}")
            if err != 0:
                 QMessageBox.warning(self, "Stop Warning", f"Direct StopMotion command failed (Code: {err}). Worker thread stop requested.")
        except Exception as e:
            print(f"ERROR sending direct StopMotion: {e}")
            QMessageBox.warning(self, "Stop Error", f"Error sending direct StopMotion command:\n{e}")
        # UI state (_is_running=False, enable/disable buttons) will be updated by _on_worker_finished


    def _pause_motion(self):
        """Sends PauseMotion command directly to the robot."""
        if not self._is_running: print("INFO: Pause ignored, not running."); return
        if not self.robot: print("ERROR: Robot not available for pause."); return

        print("Main thread: Sending PauseMotion command...")
        # Update status immediately for responsiveness
        self._update_status_label("Status: <b style='color: orange;'>Pausing...</b>")
        try:
            err = self.robot.PauseMotion()
            print(f"  PauseMotion returned: {err}")
            if err != 0:
                self._update_status_label(f"Status: <b style='color: red;'>Pause Failed (Code: {err})</b>")
                QMessageBox.warning(self, "Pause Error", f"PauseMotion command failed (Code: {err}).")
            else:
                # Update status more permanently after success
                 self._update_status_label(f"Status: <b style='color: orange;'>Paused</b>")
        except Exception as e:
            print(f"ERROR sending PauseMotion: {e}")
            self._update_status_label(f"Status: <b style='color: red;'>Pause Exception</b>")
            QMessageBox.critical(self, "Pause Exception", f"Error sending Pause command:\n{e}")


    def _resume_motion(self):
        """Sends ResumeMotion command directly to the robot."""
        if not self._is_running: print("INFO: Resume ignored, not running."); return
        if not self.robot: print("ERROR: Robot not available for resume."); return

        print("Main thread: Sending ResumeMotion command...")
        self._update_status_label("Status: <b style='color: lightgreen;'>Resuming...</b>")
        try:
            err = self.robot.ResumeMotion()
            print(f"  ResumeMotion returned: {err}")
            if err != 0:
                self._update_status_label(f"Status: <b style='color: red;'>Resume Failed (Code: {err})</b>")
                QMessageBox.warning(self, "Resume Error", f"ResumeMotion command failed (Code: {err}).")
            else:
                 # Status will be updated back to 'Executing'/'Running' by the worker's status_update signal
                 # We can set it temporarily here if desired, but worker state is truth
                 self._update_status_label(f"Status: <b style='color: lightgreen;'>Resumed</b> (Worker will update)")
        except Exception as e:
            print(f"ERROR sending ResumeMotion: {e}")
            self._update_status_label(f"Status: <b style='color: red;'>Resume Exception</b>")
            QMessageBox.critical(self, "Resume Exception", f"Error sending Resume command:\n{e}")


    def closeEvent(self, event):
        """Cleans up the worker thread on application close."""
        print("Closing application...")
        if self.thread and self.thread.isRunning():
            print("Requesting worker stop and quitting thread...")
            if self._is_running and self.worker:
                 # Request worker stop via its event loop
                 QMetaObject.invokeMethod(self.worker, "request_stop", Qt.ConnectionType.QueuedConnection)
                 # Also try direct robot stop
                 if self.robot:
                     try: self.robot.StopMotion()
                     except: pass # Ignore errors on close

            self.thread.quit() # Tell the thread's event loop to exit
            print("Waiting for thread to finish...")
            if not self.thread.wait(3000): # Wait up to 3 seconds
                print("WARNING: Worker thread did not finish gracefully. Terminating.")
                self.thread.terminate() # Force termination if necessary
            else:
                print("Thread finished.")
        super().closeEvent(event)

# --- Main Execution Block ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    try:
        window = TrajectoryRunnerApp()
        window.show()
        print("DEBUG: Application started successfully.")
    except Exception as main_e:
        print(f"CRITICAL ERROR during application startup: {main_e}\n{traceback.format_exc()}")
        try: QMessageBox.critical(None, "Startup Error", f"Failed to start application:\n{main_e}")
        except: pass
        sys.exit(1)

    sys.exit(app.exec())