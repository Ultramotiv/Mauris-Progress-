# -*- coding: utf-8 -*-
"""
PyQt6 UI for robot control using Tabs.

Features:
- Connect Tab: Connects robot, handles Drag Teach via button.
- Record Tab: Handles Drag Teach via button, records TPD, logs to CSV.
- Playback Tab: Selects TPD from CSV, sets params (speed, blend, reps),
  runs trajectory using a separate worker thread, provides controls.
- Shared Robot Object: All tabs use the same robot connection.
- State Management: UI elements updated based on connection and activity.
- Error Handling: Uses QMessageBox.
"""

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import traceback # Import traceback for detailed error printing
import csv
import os

# --- Robot Import ---
try:
    # IMPORTANT: Make sure the Robot library is accessible in your environment
    import Robot
    print("Successfully imported Robot library.")
    # Check for necessary methods for Connect, Record, AND Playback
    REQUIRED_METHODS = [
        "StopMotion", "DragTeachSwitch", "SetTPDParam",
        "SetTPDStart", "SetWebTPDStop",
        # Added back for Playback:
        "LoadTPD", "GetTPDStartPose", "MoveL", "SetSpeed",
        "GetRobotMotionDone", "PauseMotion", "ResumeMotion", "MoveTPD"
    ]
    ROBOT_AVAILABLE = True # Assume available if import succeeds for playback logic
except ImportError:
    print("-----------------------------------------------------------")
    print("WARNING: 'Robot' library not found.")
    print("         Robot communication will be simulated for Playback.")
    print("-----------------------------------------------------------")
    # Dummy Robot for UI testing without library
    ROBOT_AVAILABLE = False
    # Define MOTION_CHECK_INTERVAL here for the Dummy Robot
    MOTION_CHECK_INTERVAL = 0.25 # Seconds, used by Dummy Robot's GetRobotMotionDone

    class Robot:
        class RPC:
            _dummy_motion_counter = 0
            _dummy_motion_finish_at = 8 # Make it slightly longer for testing pause/resume
            _dummy_is_paused = False
            _dummy_is_stopped = False
            _connected_ip = None

            def __init__(self, ip):
                print(f"Info: Dummy Robot created for IP {ip}")
                self._connected_ip = ip
                # If this is the first connection attempt, raise Dummy error
                # If it's for the worker, allow it. This is imperfect.
                # A better approach would be a singleton dummy or explicit flag.
                # For now, let's assume the first __init__ is the main connection attempt.
                # if not hasattr(Robot.RPC, '_dummy_instance_exists'):
                #     Robot.RPC._dummy_instance_exists = True
                #     # raise ConnectionError("Dummy Robot") # Don't raise for worker
                # else:
                #     print("Info: Subsequent Dummy Robot instantiation (likely worker).")
                pass # Allow dummy creation always for simplicity here

            # --- Dummy Methods from TrajectoryRunnerApp ---
            def LoadTPD(self, name): print(f"Sim (IP:{self._connected_ip}): Loading TPD '{name}'..."); time.sleep(0.3); print(f"Sim: LoadTPD: {name} done."); return 0
            def GetTPDStartPose(self, name): print(f"Sim (IP:{self._connected_ip}): Getting Start Pose for '{name}'..."); time.sleep(0.2); print(f"Sim: GetTPDStartPose: {name} done."); return 0, [0.0]*6
            def MoveL(self, pose, v, blend): print(f"Sim (IP:{self._connected_ip}): MoveL to {pose[:2]}... V={v}, B={blend}"); time.sleep(0.5); print("Sim: MoveL done."); return 0
            def SetSpeed(self, speed): print(f"Sim (IP:{self._connected_ip}): SetSpeed: {speed}"); time.sleep(0.1); return 0
            def MoveTPD(self, name, blend, ovl):
                print(f"Sim (IP:{self._connected_ip}): MoveTPD: {name}, Blnd={blend}, Ovl={ovl}");
                self._dummy_motion_counter = 0
                self._dummy_is_paused = False
                self._dummy_is_stopped = False
                return 0
            def GetRobotMotionDone(self):
                time.sleep(MOTION_CHECK_INTERVAL / 2) # Simulate check time
                if self._dummy_is_stopped: print(f"Sim (IP:{self._connected_ip}): GetRobotMotionDone: Stopped (1)"); return 0, 1
                if self._dummy_is_paused: print(f"Sim (IP:{self._connected_ip}): GetRobotMotionDone: Paused (0, Cnt:{self._dummy_motion_counter})"); return 0, 0
                self._dummy_motion_counter += 1
                print(f"Sim (IP:{self._connected_ip}): GetRobotMotionDone: Checking... (Cnt:{self._dummy_motion_counter})")
                if self._dummy_motion_counter >= self._dummy_motion_finish_at: print(f"Sim: GetRobotMotionDone: Finished (1)"); return 0, 1
                else: return 0, 0 # Not done
            def StopMotion(self): print(f"Sim (IP:{self._connected_ip}): StopMotion called."); self._dummy_is_stopped = True; time.sleep(0.1); return 0
            def PauseMotion(self): print(f"Sim (IP:{self._connected_ip}): PauseMotion called."); self._dummy_is_paused = True; time.sleep(0.1); return 0
            def ResumeMotion(self): print(f"Sim (IP:{self._connected_ip}): ResumeMotion called."); self._dummy_is_paused = False; time.sleep(0.1); return 0

            # --- Dummy Methods from RobotControlTabsApp ---
            def DragTeachSwitch(self, state): print(f"Sim (IP:{self._connected_ip}): DragTeachSwitch({state})"); time.sleep(0.1); return 0
            def SetTPDParam(self, name, period, di_choose): print(f"Sim (IP:{self._connected_ip}): SetTPDParam('{name}', {period}, di={di_choose})"); time.sleep(0.2); return 0
            def SetTPDStart(self, name, period, do_choose): print(f"Sim (IP:{self._connected_ip}): SetTPDStart('{name}', {period}, do={do_choose})"); time.sleep(0.2); return 0
            def SetWebTPDStop(self): print(f"Sim (IP:{self._connected_ip}): SetWebTPDStop()"); time.sleep(0.2); return 0

            # --- Catch-all for missing methods ---
            def __getattr__(self, name):
                print(f"Warning: Dummy Robot (IP:{self._connected_ip}) - Called undefined method: {name}")
                # Provide known return types if possible
                if name in ["LoadTPD", "MoveL", "SetSpeed", "MoveTPD", "StopMotion", "PauseMotion", "ResumeMotion", "DragTeachSwitch", "SetTPDParam", "SetTPDStart", "SetWebTPDStop"]: return lambda *a, **k: 0
                if name == "GetTPDStartPose": return lambda *a, **k: (0, [0.0]*6)
                if name == "GetRobotMotionDone": return lambda *a, **k: (0, 1) # Default to done if not running
                return lambda *a, **k: -1 # Default error code

    print("Warning: Using combined Dummy Robot class.")
    REQUIRED_METHODS = [] # Skip checks if using dummy
# --- End Robot Import ---

# --- UI Imports ---
from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QGridLayout, QMessageBox,
    QInputDialog, QLineEdit, QTabWidget, QVBoxLayout, QLabel,
    QComboBox, QHBoxLayout, QSpacerItem, QSizePolicy, QSpinBox,
    QCheckBox, QFormLayout # Re-added necessary imports
)
from PyQt6.QtGui import QIcon, QFont
from PyQt6.QtCore import Qt, QObject, QThread, pyqtSignal, pyqtSlot, QMetaObject # Re-added necessary imports
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget # Or your base class
from PyQt6.QtGui import QPainter, QPixmap, QIcon # QPainter, QPixmap from QtGui
from PyQt6.QtCore import Qt, QPoint, QRect # Qt namespace, QPoint, QRect from QtCore


# --- Custom Widget for Tab Pages with Watermark ---
class WatermarkedTabPage(QWidget):
    def __init__(self, watermark_path, parent=None):
        super().__init__(parent)
        self.watermark_pixmap = None
        if os.path.exists(watermark_path):
             self.watermark_pixmap = QPixmap(watermark_path)
             if self.watermark_pixmap.isNull():
                 print(f"Warning: Failed to load watermark image '{watermark_path}' for tab.")
                 self.watermark_pixmap = None
        else:
            print(f"Warning: Watermark image file not found at '{watermark_path}'")

        # Optional: Set autoFillBackground to True if you want the palette background
        # self.setAutoFillBackground(True)


    def paintEvent(self, event):
        """Draws the watermark and ensures standard background."""
        painter = QPainter(self)

        # --- 1. Draw Standard Background (Optional but recommended) ---
        # This ensures the area behind the watermark looks normal.
        # You can use a specific color or the default palette color.
        # painter.fillRect(self.rect(), Qt.GlobalColor.white) # Or specific color
        # Or paint using the widget's current palette background:
        # opt = painter.style().standardPalette() # Get the standard palette
        # painter.fillRect(self.rect(), opt.window()) # Fill with window background color
        # Corrected code:
        # Get the palette from the widget (self) and fill with its window background brush
        painter.fillRect(self.rect(), self.palette().window())
        # --- 2. Draw Watermark ---
        if self.watermark_pixmap:
            painter.save() # Save painter state (like opacity)
            # Set opacity (0.0 to 1.0)
            painter.setOpacity(1.0) # <<<--- ADJUST WATERMARK TRANSPARENCY

            # --- Positioning Logic (relative to this widget's area) ---
            # Option A: Center the watermark (original size)
            img_rect = self.watermark_pixmap.rect()
            center_point = self.rect().center()
            top_left_point = center_point - QPoint(img_rect.width() // 2, img_rect.height() // 2)
            painter.drawPixmap(top_left_point, self.watermark_pixmap)

            # Option B: Scale to fill this tab page widget
            # painter.drawPixmap(self.rect(), self.watermark_pixmap)

            painter.restore() # Restore painter state (opacity back to 1.0)
        else:
             # Fallback if image didn't load (optional)
             pass # Do nothing extra if no watermark image

        # Note: We DO NOT call super().paintEvent() here because we painted the background
        # and the watermark. Child widgets added to this WatermarkedTabPage
        # (via layouts) will still paint themselves correctly on top.


# --- Configuration ---
ROBOT_IP = '192.168.58.2'
TRAJECTORY_CSV_FILE = 'trajectory_log.csv'
# Recording Params
RECORDING_PERIOD = 4
RECORDING_DI = 0
RECORDING_DO = 0
# Playback Params (from TrajectoryRunnerApp)
DEFAULT_SPEED = 100; DEFAULT_BLEND = 1; DEFAULT_REPETITIONS = 1
WAIT_AT_START = 1; WAIT_BETWEEN_REPS = 0.5; MOTION_CHECK_INTERVAL = 0.25 # Used by worker

DRAG_TEACH_CONTROLLERS = {"ConnecttoPatient", "RecordExercise"}


# =============================================================================
# Worker Object for Running Trajectory in Separate Thread (Copied)
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
        if not self.robot: print("!!! CRITICAL WORKER WARNING: robot_instance is None during worker __init__ !!!")
        self._stop_requested = False
        # Parameters to be set before running
        self.name = ""
        self.speed_val = 0 # This is the Rep Speed setting, NOT the override %
        self.blend_val = 0
        self.total_repetitions = 0
        self.ovl = 100.0 # This is the Speed Override % from the UI

    @pyqtSlot(str, int, int, int, float)
    def configure_run(self, name, speed_override_percent, blend, reps, ovl_unused):
        """Sets the parameters for the next trajectory run."""
        # Note: The original TrajectoryRunnerApp passed speed_override to both
        # speed_val and ovl. Here, we assume speed_val is for MoveTPD speed param
        # and ovl is the override percentage for SetSpeed. Let's clarify this.
        # Let's assume the UI 'Speed Override' SpinBox controls the ovl (SetSpeed)
        # And maybe MoveTPD speed (speed_val) should be fixed or another input.
        # For now, let's map the UI speed_override directly to ovl for SetSpeed
        # and keep speed_val potentially unused or fixed for MoveTPD.
        # Let's map UI Speed Override % to `ovl` for SetSpeed.
        # And keep `speed_val` potentially zero or unused in MoveTPD? No, MoveTPD needs speed.
        # Re-mapping arguments based on the original TrajectoryRunnerApp UI:
        # name = name
        # speed_override_percent = speed_input.value() -> use for ovl
        # blend = blend_checkbox -> use for blend_val
        # reps = repetitions_input.value() -> use for total_repetitions
        # ovl_unused -> ignore original ovl mapping, use speed_override_percent for self.ovl
        self.name = name
        self.ovl = float(speed_override_percent) # For SetSpeed command
        self.blend_val = blend # For MoveTPD command
        self.total_repetitions = reps
        # What should speed_val be for MoveTPD? Let's use a fixed value for now, maybe 0?
        # Or perhaps it should *also* be the override? The Robot lib docs are key here.
        # Let's assume MoveTPD's speed param is *also* affected by the override for now.
        self.speed_val = speed_override_percent # Use override for MoveTPD speed too (needs check)

        self._stop_requested = False # Reset stop flag for new run config
        print(f"Worker configured for: {name}, Reps={reps}, Ovl={self.ovl}%, Blnd={self.blend_val}, MoveTPD_Spd={self.speed_val}")


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
            print("!!! WORKER ERROR: run_trajectory called but self.robot is None !!!")
            return
        if not self.name:
            self.error.emit("Worker Error", "Trajectory name not configured in worker.")
            self.finished.emit(False, "Worker Error: No Name")
            return

        print(f"--- Worker Thread: Starting Sequence: '{self.name}' ---")
        self.status_update.emit(f"Status: Preparing '{self.name}'...")

        start_pose = None
        # motion_check = hasattr(self.robot, 'GetRobotMotionDone') # Check dynamically
        # Rely on exception handling if method is missing
        current_rep = 0
        error_occurred = False
        final_message = ""

        try:
            # --- Preparation ---
            self.status_update.emit(f"Status: Loading '{self.name}'...")
            print("Worker DEBUG: Calling LoadTPD..."); ret = self.robot.LoadTPD(self.name); print(f"  LoadTPD ret={ret}")
            if self._check_stop_and_error(ret, f"Load '{self.name}' failed"): return

            self.status_update.emit(f"Status: Getting start pose...")
            print("Worker DEBUG: Calling GetTPDStartPose..."); ret_p, start_pose = self.robot.GetTPDStartPose(self.name); print(f"  GetStartPose ret={ret_p}, pose={start_pose}")
            if self._check_stop_and_error(ret_p, "Get Start Pose failed"): return
            if not start_pose:
                 if self._check_stop_and_error(-1, "Get Start Pose returned an empty pose"): return # Use -1 for non-robot errors

            self.status_update.emit(f"Status: Setting speed override {self.ovl}%...")
            print("Worker DEBUG: Calling SetSpeed..."); ret_speed = self.robot.SetSpeed(self.ovl); print(f"  SetSpeed ret={ret_speed}")
            if self._check_stop_and_error(ret_speed, f"Set Speed Override ({self.ovl}%) failed"): return

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

                    self.status_update.emit(f"Status: Moving to start (Rep {current_rep})")
                    print("Worker DEBUG: Calling MoveL to start pose..."); ret_move = self.robot.MoveL(start_pose, 0, 0); print(f"  MoveL ret={ret_move}") # Use blend 0 for start move
                    if self._check_stop_and_error(ret_move, f"MoveL to start failed (Rep {current_rep})"): return
                    self._wait_interruptible(WAIT_AT_START)

                    if self._stop_requested: break

                    self.status_update.emit(f"Status: Executing '{self.name}' (Rep {current_rep})")
                    print("Worker DEBUG: Calling MoveTPD..."); ret_tpd = self.robot.MoveTPD(self.name, self.blend_val, self.speed_val); print(f"  MoveTPD ret={ret_tpd}") # Using self.speed_val here
                    if self._check_stop_and_error(ret_tpd, f"MoveTPD failed (Rep {current_rep})"): return

                    # --- Wait for Motion Completion ---
                    print("Worker DEBUG: Waiting for motion completion...");
                    polling_start_time = time.time()
                    motion_completed_successfully = False
                    while not self._stop_requested:
                        # Use a short sleep even in the worker to prevent busy-waiting 100% CPU if GetRobotMotionDone is fast
                        time.sleep(MOTION_CHECK_INTERVAL / 5) # Shorter sleep, UI thread is separate

                        print("Worker DEBUG: Calling GetRobotMotionDone...")
                        try:
                            ret_state, state = self.robot.GetRobotMotionDone()
                            print(f"  GMD ret={ret_state}, state={state}")
                            if ret_state != 0:
                                print(f"Worker WARNING: GetRobotMotionDone error ({ret_state}). Stopping motion check for this rep.")
                                error_occurred = True
                                final_message = f"GetRobotMotionDone Error (Code: {ret_state}) on Rep {current_rep}"
                                self.error.emit("Playback Error", final_message)
                                break # Exit GMD loop on error
                            if state == 1:
                                print("Worker DEBUG: Motion complete.");
                                motion_completed_successfully = True
                                break # Success
                            # Check for timeout only if not stopped or errored
                            if time.time() - polling_start_time > 600: # 10 min timeout
                                if self._check_stop_and_error(-1, f"Playback timed out (Rep {current_rep})"): return

                        except AttributeError:
                             print("Worker ERROR: Robot object missing 'GetRobotMotionDone'. Cannot monitor completion.");
                             # Simulate wait as fallback, but signal error
                             self._wait_interruptible(3) # Simulate some work time
                             error_occurred = True
                             final_message = f"Missing GetRobotMotionDone on Rep {current_rep}"
                             self.error.emit("Playback Error", final_message)
                             break # Exit GMD loop
                        except Exception as gmd_e:
                            print(f"Worker ERROR during GetRobotMotionDone call: {gmd_e}\n{traceback.format_exc()}")
                            # Treat exception as an error stopping this rep's check
                            error_occurred = True
                            final_message = f"Exception in GetRobotMotionDone (Rep {current_rep})"
                            self.error.emit("Playback Error", final_message)
                            break # Stop polling on GMD call error

                    # End of GMD loop
                    if error_occurred: return # Exit run_trajectory if GMD error occurred
                    if self._stop_requested: break # Exit rep loop if stop requested during GMD poll
                    if not motion_completed_successfully and not self._stop_requested:
                        # This case shouldn't be hit if errors/timeouts are handled, but as a fallback
                        print("Worker WARNING: Motion completion loop exited without success or stop request.")
                        if self._check_stop_and_error(-1, f"Motion check ended unexpectedly (Rep {current_rep})"): return

                    print(f"--- Worker Thread: Rep {current_rep} finished ---")
                    if current_rep < self.total_repetitions and WAIT_BETWEEN_REPS > 0:
                        print(f"Worker DEBUG: Waiting between reps ({WAIT_BETWEEN_REPS}s)...")
                        self.status_update.emit(f"Status: Paused between reps ({current_rep}/{self.total_repetitions})")
                        self._wait_interruptible(WAIT_BETWEEN_REPS)

                except AttributeError as rep_attr_e:
                    print(f"Worker ATTRIBUTE ERROR during Rep {current_rep}: {rep_attr_e}\n{traceback.format_exc()}")
                    self._handle_error(f"Missing Robot Function on Rep {current_rep}", f"Robot object missing function: {rep_attr_e}")
                    error_occurred = True
                    return # Exit run_trajectory
                except Exception as rep_e: # Catch other errors within the inner try
                     print(f"Worker UNEXPECTED ERROR during Rep {current_rep}: {rep_e}\n{traceback.format_exc()}")
                     self._handle_error(f"Unexpected Error on Rep {current_rep}", f"{rep_e}")
                     error_occurred = True
                     return # Exit run_trajectory

            # --- End Repetition Loop ---
            if not final_message and not error_occurred: # If loop finished naturally
                 final_message = f"'{self.name}' Completed ({self.total_repetitions} reps)"
                 print(f"\n--- Worker Thread: Sequence '{self.name}' Completed Successfully ---")

        except AttributeError as prep_attr_e:
             print(f"Worker ATTRIBUTE ERROR during preparation: {prep_attr_e}\n{traceback.format_exc()}")
             self._handle_error("Missing Robot Function (Prep)", f"Robot object missing function: {prep_attr_e}")
             error_occurred = True
             return # Exit run_trajectory
        except Exception as e: # Catch errors during preparation or unexpected loop exit
            print(f"Worker FATAL ERROR during execution: {e}\n{traceback.format_exc()}")
            rep_str = f" on rep {current_rep}" if current_rep > 0 else " during preparation"
            self._handle_error(f"Execution Error{rep_str}", f"{e}")
            error_occurred = True
            return # Exit run_trajectory

        finally:
            # Signal finished state to main thread
            success = not error_occurred and not self._stop_requested
            # If stop was requested, message should already be set
            if self._stop_requested and not final_message:
                final_message = f"Stopped by User during '{self.name}'"

            print(f"Worker finishing. Success={success}, Msg='{final_message}'")
            self.finished.emit(success, final_message)


    def _check_stop_and_error(self, return_code, error_message_base):
        """Helper to check stop flag and robot return codes, emitting signals on error."""
        if self._stop_requested:
            print(f"Worker INFO: Stop detected during operation check ({error_message_base}).")
            # self.finished signal will be emitted in the finally block with stop message
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
            # Sleep for a short interval to avoid busy-waiting
            time.sleep(min(0.05, end_time - time.time(), 0.1)) # Sleep briefly


# =============================================================================
# Main Application Window
# =============================================================================
class RobotControlTabsApp(QWidget):
    # Signal for playback worker
    trigger_playback_run = pyqtSignal(str, int, int, int, float)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface - Connect, Record, Playback")
        self.setGeometry(200, 200, 650, 500) # Adjusted size slightly

        self.watermark_file = "Logo-01.jpg"
        # State Flags & Data
        self.robot = None # THE shared robot instance
        self._is_recording = False
        self._is_playing = False # Added playback state flag
        # Removed _is_paused from here, playback pause is handled by robot state directly
        self.current_recording_name = None
        # self.current_playback_name = None # Worker handles current name

        # Playback Worker/Thread
        self.playback_worker = None
        self.playback_thread = None

        # UI Element References
        self.action_buttons = [] # Keep track of Connect/Record buttons
        self.robot_status_label = None; self.drag_status_label = None
        self.connect_button = None
        self.record_button = None; self.stop_record_button = None; self.record_status_label = None
        # --- Playback UI Elements ---
        self.playback_tab = None # Reference to the tab itself
        self.playback_status_label = None
        self.playback_trajectory_combo = None
        self.playback_refresh_button = None
        self.playback_speed_input = None
        self.playback_repetitions_input = None
        self.playback_blend_checkbox = None
        self.playback_rep_count_label = None
        self.playback_run_button = None
        self.playback_stop_button = None
        self.playback_pause_button = None
        self.playback_resume_button = None
        # --- End Playback UI Elements ---

        # Action Map (Connect/Record only)
        self.action_map = {
            "ConnecttoPatient": self._connect_patient,
            "RecordExercise": self._record_exercise,
        }

        # Initialize UI Phases
        self._init_ui()          # Creates tabs and calls _create_..._tab methods
        self._connect_robot()    # Attempts connection AND sets up worker if successful
        self._apply_styles()
        self._update_ui_state() # Initial state update

   
    def _connect_robot(self):
        """Attempts connection, checks methods, updates status, setups playback worker."""
        # Reset robot instance first
        self.robot = None
        if self.robot_status_label: self.robot_status_label.setText("Status: Connecting...")
        QApplication.processEvents()

        try:
            print(f"Attempting to connect to robot at {ROBOT_IP}...")
            self.robot = Robot.RPC(ROBOT_IP) # Create the shared instance
            print("Robot connection object created successfully.")

            if not ROBOT_AVAILABLE: # If using Dummy
                 if self.robot_status_label: self.robot_status_label.setText("Status: <b style='color: orange;'>Connected (Simulated)</b>")
                 # Don't check methods for Dummy
            else: # If using real Robot lib
                if self.robot_status_label: self.robot_status_label.setText("Status: <b style='color: #2E7D32;'>Connected</b>")
                missing_methods = [m for m in REQUIRED_METHODS if not hasattr(self.robot, m)]
                if missing_methods:
                    warning_msg = f"Missing expected functions: {', '.join(missing_methods)}. Some features might fail."
                    print(f"WARNING: {warning_msg}")
                    QMessageBox.warning(self, "Robot Methods Missing", warning_msg)

            # --- Setup Playback Worker Thread ---
            # Do this ONLY if connection succeeds (or simulation is active)
            self._setup_playback_worker_thread()

        except Exception as e:
            self.robot = None # Ensure robot is None on failure
            print(f"FATAL: Failed to connect to robot: {e}\n{traceback.format_exc()}")
            if self.robot_status_label: self.robot_status_label.setText("Status: <b style='color: #C62828;'>Disconnected</b>")
            # Show message box only for non-dummy connection errors
            if ROBOT_AVAILABLE: # Only show critical error if real library was expected
                 QMessageBox.critical(self, "Robot Connection Failed", f"Could not connect to robot at {ROBOT_IP}.\nError: {e}\n\nCheck IP/network.")
            elif "Dummy Robot" in str(e): # Log dummy error but don't pop up
                 print("Ignoring Dummy Robot connection error for popup.")
            else: # Show other unexpected errors
                 QMessageBox.critical(self, "Connection Error", f"An unexpected error occurred during connection:\n{e}")

        finally:
            # Load trajectory names into the playback combo box if it exists
            if self.playback_trajectory_combo:
                 self._load_trajectory_names()
            else:
                 print("WARNING: Playback combo box not yet initialized when _connect_robot finished.")

            self._update_ui_state() # Update UI based on final connection status


    def _setup_playback_worker_thread(self):
        """Creates the playback worker object and thread, connects signals/slots."""
        # Clean up previous thread/worker if any (e.g., on reconnect)
        if self.playback_thread and self.playback_thread.isRunning():
            print("INFO: Cleaning up previous playback worker thread...")
            self.playback_thread.quit()
            if not self.playback_thread.wait(1000):
                print("WARNING: Previous playback thread did not quit gracefully.")
                self.playback_thread.terminate() # Force if needed
        self.playback_worker = None
        self.playback_thread = None

        if not self.robot:
             print("WARNING: Cannot setup playback worker thread without robot connection.")
             return

        print("DEBUG: Setting up playback worker thread...")
        self.playback_thread = QThread(self) # Parent thread to app for proper cleanup
        self.playback_worker = TrajectoryWorker(self.robot) # <<<< Pass the SHARED robot instance >>>>
        self.playback_worker.moveToThread(self.playback_thread)

        # Connect worker signals to UI slots in the main thread
        self.playback_worker.status_update.connect(self._playback_update_status_label)
        self.playback_worker.rep_update.connect(self._playback_update_rep_label)
        self.playback_worker.finished.connect(self._on_playback_worker_finished)
        self.playback_worker.error.connect(self._on_playback_worker_error)

        # Connect trigger signal from UI thread to worker's configure slot
        self.trigger_playback_run.connect(self.playback_worker.configure_run)

        # Connect thread finished signal for cleanup
        self.playback_thread.finished.connect(self.playback_worker.deleteLater)
        self.playback_thread.finished.connect(self.playback_thread.deleteLater)
        # Keep cleanup connections even if thread might be terminated later

        self.playback_thread.start()
        print("DEBUG: Playback worker thread started.")
        self._update_ui_state() # Update UI now that worker should be ready


    def _init_ui(self):
        """Initializes the main layout and tab structure."""
        main_layout = QVBoxLayout(self)
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)

        # Create Tab Widgets
        # self.connect_tab = QWidget()
        # self.record_tab = QWidget()
        # self.playback_tab = QWidget() # Create the playback tab widget

        # --- !!! CHANGE HERE: Create Tab Pages using WatermarkedTabPage !!! ---
        # Create Tab Widgets using the custom class and pass the watermark path
        self.connect_tab = WatermarkedTabPage(self.watermark_file, parent=self)
        self.record_tab = WatermarkedTabPage(self.watermark_file, parent=self)
        self.playback_tab = WatermarkedTabPage(self.watermark_file, parent=self)
# -------------------------------------------------------------------

        # Add Tabs
        self.tab_widget.addTab(self.connect_tab, "1. Connect")
        self.tab_widget.addTab(self.record_tab, "2. Record Exercise")
        self.tab_widget.addTab(self.playback_tab, "3. Playback Exercise") # Add the playback tab

        # Create Tab Contents
        self._create_connect_tab()
        self._create_record_tab()
        self._create_playback_tab() # Create the playback tab content

        self.setLayout(main_layout)

    # def _create_connect_tab(self):
    #     """Creates UI elements for the Connect Tab."""
    #     layout = QVBoxLayout(self.connect_tab); layout.setAlignment(Qt.AlignmentFlag.AlignTop); layout.setSpacing(15)
    #     # General Status Area (Could be moved outside tabs if desired)
    #     status_layout = QHBoxLayout()
    #     self.robot_status_label = QLabel("Status: Initializing...")
    #     self.drag_status_label = QLabel("Drag Teach: <b style='color: #C62828;'>OFF</b>")
    #     font = QFont(); font.setPointSize(11); self.robot_status_label.setFont(font); self.drag_status_label.setFont(font)
    #     status_layout.addWidget(self.robot_status_label)
    #     status_layout.addStretch()
    #     status_layout.addWidget(self.drag_status_label)
    #     layout.addLayout(status_layout)

    #     # Reconnect Button (Optional but good practice)
    #     reconnect_button = QPushButton("Attempt Reconnect")
    #     reconnect_button.setToolTip("Try to connect to the robot again")
    #     reconnect_button.clicked.connect(self._connect_robot) # Reuse connection logic
    #     layout.addWidget(reconnect_button)

    #     # Connect Button (for Drag Teach)
    #     self.connect_button = QPushButton("Enable Drag Teach (Connect Mode)")
    #     self.connect_button.setObjectName("ConnecttoPatient"); self.connect_button.setMinimumHeight(50); self.connect_button.setProperty("active", False)
    #     self.connect_button.setToolTip("Enables/Disables Drag Teach Mode without recording")
    #     self.connect_button.clicked.connect(lambda: self.on_button_click(self.connect_button))
    #     layout.addWidget(self.connect_button); self.action_buttons.append(self.connect_button)
    #     layout.addStretch()
    # Inside RobotControlTabsApp class
    def _create_connect_tab(self):
        """Creates UI elements for the Connect Tab with responsive centering."""
        # Main layout for the entire tab page (WatermarkedTabPage)
        main_tab_layout = QVBoxLayout(self.connect_tab)
        # AlignTop pushes the centered content block to the top
        main_tab_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        main_tab_layout.setContentsMargins(20, 20, 20, 20) # Add some padding

        # --- Create a container widget for the actual content ---
        content_container_widget = QWidget()
        content_layout = QVBoxLayout(content_container_widget) # Layout for the content
        content_layout.setSpacing(15)
        # Set a maximum width for the content area
        content_container_widget.setMaximumWidth(600) # Adjust this value as needed

        # --- Add UI Elements TO THE CONTENT LAYOUT ---
        # Status Area
        status_layout = QHBoxLayout()
        self.robot_status_label = QLabel("Status: Initializing...")
        self.drag_status_label = QLabel("Drag Teach: <b style='color: #C62828;'>OFF</b>")
        font = QFont(); font.setPointSize(11); self.robot_status_label.setFont(font); self.drag_status_label.setFont(font)
        status_layout.addWidget(self.robot_status_label)
        status_layout.addStretch()
        status_layout.addWidget(self.drag_status_label)
        content_layout.addLayout(status_layout) # Add to content_layout

        # Reconnect Button
        reconnect_button = QPushButton("Attempt Reconnect")
        reconnect_button.setToolTip("Try to connect to the robot again")
        reconnect_button.clicked.connect(self._connect_robot)
        content_layout.addWidget(reconnect_button) # Add to content_layout

        # Connect Button
        self.connect_button = QPushButton("Enable Drag Teach (Connect Mode)")
        self.connect_button.setObjectName("ConnecttoPatient")
        # Remove fixed/minimum size unless stylistically desired for height
        self.connect_button.setMinimumHeight(50) # Keep min height if you like it
        self.connect_button.setProperty("active", False)
        self.connect_button.setToolTip("Enables/Disables Drag Teach Mode without recording")
        self.connect_button.clicked.connect(lambda: self.on_button_click(self.connect_button))
        content_layout.addWidget(self.connect_button) # Add to content_layout
        self.action_buttons.append(self.connect_button)

        # Add vertical stretch *inside* the content if needed, or rely on AlignTop of main layout
        content_layout.addStretch(1)

        # --- Center the content container horizontally ---
        centering_layout = QHBoxLayout()
        centering_layout.addStretch(1)
        centering_layout.addWidget(content_container_widget) # Add the container
        centering_layout.addStretch(1)

        # Add the centering layout to the main tab layout
        main_tab_layout.addLayout(centering_layout)
        # No final stretch needed in main_tab_layout if using AlignTop


    # def _create_record_tab(self):
    #     """Creates UI elements for the Record Tab."""
    #     layout = QVBoxLayout(self.record_tab); layout.setAlignment(Qt.AlignmentFlag.AlignTop); layout.setSpacing(15)
    #     self.record_button = QPushButton("Start Recording Exercise")
    #     self.record_button.setObjectName("RecordExercise"); self.record_button.setMinimumHeight(50); self.record_button.setProperty("active", False)
    #     self.record_button.setToolTip("Enables Drag Teach and starts recording trajectory")
    #     self.record_button.clicked.connect(lambda: self.on_button_click(self.record_button))
    #     layout.addWidget(self.record_button); self.action_buttons.append(self.record_button)
    #     self.stop_record_button = QPushButton("Stop Recording")
    #     self.stop_record_button.setObjectName("StopRecord"); self.stop_record_button.setMinimumHeight(50)
    #     self.stop_record_button.setStyleSheet("background-color: #FF6347; color: white;") # Tomato color
    #     self.stop_record_button.clicked.connect(self._stop_recording_action) # Use specific stop handler
    #     layout.addWidget(self.stop_record_button)
    #     self.record_status_label = QLabel("Recording Status: Idle")
    #     font = QFont(); font.setPointSize(11); self.record_status_label.setFont(font)
    #     layout.addWidget(self.record_status_label)
    #     layout.addStretch()

    # Inside RobotControlTabsApp class
    def _create_record_tab(self):
        """Creates UI elements for the Record Tab with responsive centering."""
        # Main layout for the entire tab page
        main_tab_layout = QVBoxLayout(self.record_tab)
        main_tab_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        main_tab_layout.setContentsMargins(20, 20, 20, 20)

        # --- Create a container widget for the actual content ---
        content_container_widget = QWidget()
        content_layout = QVBoxLayout(content_container_widget) # Layout for the content
        content_layout.setSpacing(15)
        content_container_widget.setMaximumWidth(600) # Adjust max width

        # --- Create Widgets and add TO THE CONTENT LAYOUT ---
        self.record_button = QPushButton("Start Recording Exercise")
        self.record_button.setObjectName("RecordExercise")
        self.record_button.setMinimumHeight(50) # Keep min height
        self.record_button.setProperty("active", False)
        self.record_button.setToolTip("Enables Drag Teach and starts recording trajectory")
        self.record_button.clicked.connect(lambda: self.on_button_click(self.record_button))
        content_layout.addWidget(self.record_button) # Add to content_layout
        self.action_buttons.append(self.record_button)

        self.stop_record_button = QPushButton("Stop Recording")
        self.stop_record_button.setObjectName("StopRecord")
        self.stop_record_button.setMinimumHeight(50) # Keep min height
        self.stop_record_button.setStyleSheet("background-color: #FF6347; color: white;") # Tomato color
        self.stop_record_button.clicked.connect(self._stop_recording_action)
        content_layout.addWidget(self.stop_record_button) # Add to content_layout

        self.record_status_label = QLabel("Recording Status: Idle")
        font = QFont(); font.setPointSize(11); self.record_status_label.setFont(font)
        content_layout.addWidget(self.record_status_label) # Add to content_layout

        # Add vertical stretch *inside* the content
        content_layout.addStretch(1)

        # --- Center the content container horizontally ---
        centering_layout = QHBoxLayout()
        centering_layout.addStretch(1)
        centering_layout.addWidget(content_container_widget) # Add the container
        centering_layout.addStretch(1)

        # Add the centering layout to the main tab layout
        main_tab_layout.addLayout(centering_layout)

    # def _create_playback_tab(self):
    #     """Creates UI elements for the Playback Tab, based on TrajectoryRunnerApp."""
    #     layout = QVBoxLayout(self.playback_tab)
    #     layout.setSpacing(10) # Adjust spacing

    #     # Status Label for Playback
    #     self.playback_status_label = QLabel("Playback Status: Idle")
    #     font_status = QFont(); font_status.setPointSize(10); self.playback_status_label.setFont(font_status)
    #     layout.addWidget(self.playback_status_label)

    #     # Trajectory Selection
    #     combo_layout = QHBoxLayout(); combo_label = QLabel("Select Trajectory:")
    #     self.playback_trajectory_combo = QComboBox(); self.playback_trajectory_combo.setToolTip("Select recorded exercise to play back")
    #     self.playback_trajectory_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    #     self.playback_trajectory_combo.addItem("<Load trajectories first>"); self.playback_trajectory_combo.setEnabled(False)
    #     self.playback_trajectory_combo.currentIndexChanged.connect(self._update_ui_state) # Update UI on selection change
    #     self.playback_refresh_button = QPushButton("Refresh List"); self.playback_refresh_button.setToolTip("Reload names from CSV")
    #     self.playback_refresh_button.clicked.connect(self._load_trajectory_names) # Connect refresh button
    #     combo_layout.addWidget(combo_label); combo_layout.addWidget(self.playback_trajectory_combo, 1); combo_layout.addWidget(self.playback_refresh_button);
    #     layout.addLayout(combo_layout)

    #     # Parameter Inputs
    #     param_layout = QFormLayout()
    #     param_layout.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapAllRows) # Better wrapping if needed
    #     self.playback_speed_input = QSpinBox(); self.playback_speed_input.setRange(1, 150); self.playback_speed_input.setValue(DEFAULT_SPEED); self.playback_speed_input.setSuffix(" %"); self.playback_speed_input.setToolTip("Set speed override (1-150%) for playback")
    #     param_layout.addRow("Speed Override:", self.playback_speed_input)
    #     self.playback_repetitions_input = QSpinBox(); self.playback_repetitions_input.setRange(1, 100); self.playback_repetitions_input.setValue(DEFAULT_REPETITIONS); self.playback_repetitions_input.setToolTip("Set number of repetitions")
    #     param_layout.addRow("Repetitions:", self.playback_repetitions_input)
    #     self.playback_blend_checkbox = QCheckBox("Smooth Motion (Blend)"); self.playback_blend_checkbox.setChecked(DEFAULT_BLEND == 1); self.playback_blend_checkbox.setToolTip("Check for blend=1 (smoother corners), uncheck for blend=0 (sharper corners)")
    #     param_layout.addRow(self.playback_blend_checkbox);
    #     layout.addLayout(param_layout)

    #     # Live Repetition Count Label
    #     self.playback_rep_count_label = QLabel("Rep: - / -"); font_rep = QFont(); font_rep.setPointSize(10); font_rep.setBold(True); self.playback_rep_count_label.setFont(font_rep); self.playback_rep_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter);
    #     layout.addWidget(self.playback_rep_count_label)

    #     # Run Button
    #     self.playback_run_button = QPushButton("Run Selected Trajectory"); self.playback_run_button.setMinimumHeight(40); font_run = QFont(); font_run.setPointSize(11); font_run.setBold(True); self.playback_run_button.setFont(font_run)
    #     self.playback_run_button.setToolTip("Load and execute the selected trajectory with specified parameters");
    #     self.playback_run_button.clicked.connect(self._start_playback_run) # Connect to specific start handler
    #     self.playback_run_button.setEnabled(False);
    #     layout.addWidget(self.playback_run_button)

    #     # Control Buttons
    #     control_button_layout = QHBoxLayout()
    #     self.playback_stop_button = QPushButton("STOP"); self.playback_stop_button.setToolTip("Immediately stop robot motion (StopMotion)"); self.playback_stop_button.setStyleSheet("background-color: #DC143C; color: white; font-weight: bold;");
    #     self.playback_stop_button.clicked.connect(self._stop_playback_motion); # Connect to specific stop handler
    #     self.playback_stop_button.setEnabled(False) # Crimson Red

    #     self.playback_pause_button = QPushButton("Pause"); self.playback_pause_button.setToolTip("Pause robot motion (PauseMotion)"); self.playback_pause_button.setStyleSheet("background-color: #FFA500;");
    #     self.playback_pause_button.clicked.connect(self._pause_playback_motion); # Connect to specific pause handler
    #     self.playback_pause_button.setEnabled(False) # Orange

    #     self.playback_resume_button = QPushButton("Resume"); self.playback_resume_button.setToolTip("Resume paused robot motion (ResumeMotion)"); self.playback_resume_button.setStyleSheet("background-color: #90EE90;");
    #     self.playback_resume_button.clicked.connect(self._resume_playback_motion); # Connect to specific resume handler
    #     self.playback_resume_button.setEnabled(False) # Light Green

    #     control_button_layout.addWidget(self.playback_stop_button)
    #     control_button_layout.addWidget(self.playback_pause_button)
    #     control_button_layout.addWidget(self.playback_resume_button)
    #     layout.addLayout(control_button_layout)

    #     layout.addSpacerItem(QSpacerItem(20, 10, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
    # Inside RobotControlTabsApp class
    def _create_playback_tab(self):
        """Creates UI elements for the Playback Tab with responsive centering."""
        # Main layout for the entire tab page
        main_tab_layout = QVBoxLayout(self.playback_tab)
        # AlignTop might not be desired here if content is tall
        # main_tab_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        main_tab_layout.setContentsMargins(20, 20, 20, 20)

        # --- Create a container widget for the actual content ---
        content_container_widget = QWidget()
        content_layout = QVBoxLayout(content_container_widget) # Layout for the content
        content_layout.setSpacing(10)
        content_container_widget.setMaximumWidth(650) # Adjust max width, maybe slightly wider for this tab

        # --- Add UI Elements TO THE CONTENT LAYOUT ---

        # Status Label for Playback
        self.playback_status_label = QLabel("Playback Status: Idle")
        font_status = QFont(); font_status.setPointSize(10); self.playback_status_label.setFont(font_status)
        content_layout.addWidget(self.playback_status_label)

        # Trajectory Selection
        combo_layout = QHBoxLayout(); combo_label = QLabel("Select Trajectory:")
        self.playback_trajectory_combo = QComboBox(); self.playback_trajectory_combo.setToolTip("Select recorded exercise to play back")
        # Let ComboBox expand horizontally within the content layout
        self.playback_trajectory_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.playback_trajectory_combo.addItem("<Load trajectories first>"); self.playback_trajectory_combo.setEnabled(False)
        self.playback_trajectory_combo.currentIndexChanged.connect(self._update_ui_state)
        self.playback_refresh_button = QPushButton("🔄"); self.playback_refresh_button.setToolTip("Reload names from CSV")
        self.playback_refresh_button.clicked.connect(self._load_trajectory_names)
        combo_layout.addWidget(combo_label); combo_layout.addWidget(self.playback_trajectory_combo, 1); combo_layout.addWidget(self.playback_refresh_button);
        content_layout.addLayout(combo_layout)

        # Parameter Inputs (QFormLayout naturally tries to align)
        param_layout = QFormLayout()
        param_layout.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapAllRows)
        self.playback_speed_input = QSpinBox(); self.playback_speed_input.setRange(1, 150); self.playback_speed_input.setValue(DEFAULT_SPEED); self.playback_speed_input.setSuffix(" %"); self.playback_speed_input.setToolTip("Set speed override (1-150%) for playback")
        param_layout.addRow("Speed Override:", self.playback_speed_input)
        self.playback_repetitions_input = QSpinBox(); self.playback_repetitions_input.setRange(1, 100); self.playback_repetitions_input.setValue(DEFAULT_REPETITIONS); self.playback_repetitions_input.setToolTip("Set number of repetitions")
        param_layout.addRow("Repetitions:", self.playback_repetitions_input)
        self.playback_blend_checkbox = QCheckBox("Smooth Motion (Blend)"); self.playback_blend_checkbox.setChecked(DEFAULT_BLEND == 1); self.playback_blend_checkbox.setToolTip("Check for blend=1 (smoother corners), uncheck for blend=0 (sharper corners)")
        param_layout.addRow(self.playback_blend_checkbox);
        content_layout.addLayout(param_layout)

        # Live Repetition Count Label
        self.playback_rep_count_label = QLabel("Rep: - / -"); font_rep = QFont(); font_rep.setPointSize(10); font_rep.setBold(True); self.playback_rep_count_label.setFont(font_rep);
        self.playback_rep_count_label.setAlignment(Qt.AlignmentFlag.AlignCenter); # Center text
        content_layout.addWidget(self.playback_rep_count_label)

        # Run Button
        self.playback_run_button = QPushButton("Run Selected Trajectory");
        # Remove fixed size, keep min height
        self.playback_run_button.setMinimumHeight(40);
        font_run = QFont(); font_run.setPointSize(11); font_run.setBold(True); self.playback_run_button.setFont(font_run)
        self.playback_run_button.setToolTip("Load and execute the selected trajectory with specified parameters");
        self.playback_run_button.clicked.connect(self._start_playback_run)
        self.playback_run_button.setEnabled(False);
        content_layout.addWidget(self.playback_run_button) # Add directly to content layout

        # Control Buttons (Stop/Pause/Resume)
        control_button_layout = QHBoxLayout() # Layout for the buttons themselves
        # Define a common height and set NORMAL size policy (Preferred)
        COMMON_CTRL_BTN_HEIGHT = 40
        # Use default (Preferred, Fixed) or set explicitly if needed
        # preferred_policy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        self.playback_stop_button = QPushButton("STOP"); self.playback_stop_button.setToolTip("Immediately stop robot motion (StopMotion)"); self.playback_stop_button.setStyleSheet("background-color: #DC143C; color: white; font-weight: bold;");
        self.playback_stop_button.clicked.connect(self._stop_playback_motion);
        self.playback_stop_button.setEnabled(False)
        self.playback_stop_button.setFixedHeight(COMMON_CTRL_BTN_HEIGHT)
        # self.playback_stop_button.setSizePolicy(preferred_policy) # Usually not needed

        self.playback_pause_button = QPushButton("Pause"); self.playback_pause_button.setToolTip("Pause robot motion (PauseMotion)"); self.playback_pause_button.setStyleSheet("background-color: #FFA500;");
        self.playback_pause_button.clicked.connect(self._pause_playback_motion);
        self.playback_pause_button.setEnabled(False)
        self.playback_pause_button.setFixedHeight(COMMON_CTRL_BTN_HEIGHT)
        # self.playback_pause_button.setSizePolicy(preferred_policy)

        self.playback_resume_button = QPushButton("Resume"); self.playback_resume_button.setToolTip("Resume paused robot motion (ResumeMotion)"); self.playback_resume_button.setStyleSheet("background-color: #90EE90;");
        self.playback_resume_button.clicked.connect(self._resume_playback_motion);
        self.playback_resume_button.setEnabled(False)
        self.playback_resume_button.setFixedHeight(COMMON_CTRL_BTN_HEIGHT)
        # self.playback_resume_button.setSizePolicy(preferred_policy)

        control_button_layout.addWidget(self.playback_stop_button)
        control_button_layout.addWidget(self.playback_pause_button)
        control_button_layout.addWidget(self.playback_resume_button)
        # Add the button group layout to the main content layout
        content_layout.addLayout(control_button_layout)

        # Add vertical stretch *inside* the content container
        content_layout.addStretch(1)

        # --- Center the content container horizontally ---
        centering_layout = QHBoxLayout()
        centering_layout.addStretch(1)
        centering_layout.addWidget(content_container_widget) # Add the container
        centering_layout.addStretch(1)

        # Add the centering layout to the main tab layout
        main_tab_layout.addLayout(centering_layout)
    @pyqtSlot()
    def _load_trajectory_names(self):
        """Reads trajectory names from the CSV log file and populates the Playback ComboBox."""
        combo = self.playback_trajectory_combo
        if not combo:
            print("ERROR: _load_trajectory_names called but playback_trajectory_combo is None!")
            return

        if self._is_playing or self._is_recording:
            print("INFO: Cannot refresh list while recording or playing.")
            return

        print(f"Attempting to load trajectories from '{TRAJECTORY_CSV_FILE}' for playback...")
        current_selection = combo.currentText()
        combo.clear()

        if not os.path.isfile(TRAJECTORY_CSV_FILE):
            print(f"  Info: Log file not found at expected location.")
            combo.addItem("<Log file not found>")
            combo.setEnabled(False)
            self._update_ui_state()
            return

        trajectory_names = set()
        name_col_index = 1 # Default index

        try:
            with open(TRAJECTORY_CSV_FILE, 'r', newline='', encoding='utf-8') as csvfile:
                reader = csv.reader(csvfile)
                header = next(reader, None)
                if header:
                    hdr_low = [h.lower().strip() for h in header]
                    try:
                        name_col_index = hdr_low.index('trajectoryname')
                    except ValueError:
                        print(f"  Warning: 'trajectoryname' not found in CSV header. Using column index {name_col_index}.")
                else:
                     print(f"  Warning: No header row found in CSV. Using column index {name_col_index}.")

                for row in reader:
                    if len(row) > name_col_index:
                        name = row[name_col_index].strip()
                        if name: trajectory_names.add(name)

            print(f"  Found {len(trajectory_names)} unique trajectory names.")

        except Exception as e:
            print(f"  ERROR reading CSV file '{TRAJECTORY_CSV_FILE}': {e}\n{traceback.format_exc()}")
            combo.addItem("<Error reading log>")
            combo.setEnabled(False)
            self._update_ui_state()
            QMessageBox.warning(self, "File Error", f"Could not read trajectory log:\n{e}")
            return

        # Populate ComboBox
        if trajectory_names:
            sorted_names = sorted(list(trajectory_names))
            combo.addItem("<Select>") # Add placeholder first
            combo.addItems(sorted_names)
            # Try to restore previous selection or select placeholder
            index = combo.findText(current_selection)
            combo.setCurrentIndex(index if index != -1 else 0)
            combo.setEnabled(True)
        else:
            print("  Info: No valid trajectory names found in the log file.")
            combo.addItem("<No trajectories in log>")
            combo.setEnabled(False)

        self._update_ui_state() # Update button enable states etc.


    # def _apply_styles(self):
    #     """Applies the CSS stylesheet to the application."""
    #     # Stylesheet remains the same for now
    #     bg_color="transparent"; btn_bg_color="#E1E8F0"; btn_text_color="#006400"; btn_border_color="#B0C4DE"; btn_hover_bg="#CAD7E3"; btn_pressed_bg="#B0C4DE"; active_bg_color="#65A8D7"; active_text_color="#FFFFFF"; active_border_color="#5595C7"; active_hover_bg="#85BBE1"; disabled_bg_color="#E8E8E8"; disabled_text_color="#A0A0A0"; disabled_border_color="#D0D0D0"; font_family="Segoe UI, Arial, sans-serif"; font_size="18pt"; tab_selected_color="#FFFFFF"; tab_bg_color="#D4DDE6"; slider_groove_bg="#FFFFFF"; slider_handle_bg=active_bg_color; slider_handle_border=active_border_color;
    #     stylesheet=f"""QWidget{{background-color:{bg_color};font-family:{font_family};font-size:{font_size};color:#333333;}}QTabBar::tab{{background:{tab_bg_color};color:{btn_text_color};border:1px solid {btn_border_color};border-bottom:none;border-top-left-radius:4px;border-top-right-radius:4px;padding:8px 15px;margin-right:2px;font-weight:bold;}}QTabBar::tab:hover{{background:{btn_hover_bg};}}QTabBar::tab:selected{{background:{tab_selected_color};color:{btn_text_color};border-color:{btn_border_color};border-bottom:1px solid {tab_selected_color};}}QTabWidget::pane{{border:1px solid {btn_border_color};border-top:none;background-color:{tab_selected_color};}}QPushButton{{background-color:{btn_bg_color};color:{btn_text_color};border:1px solid {btn_border_color};border-radius:5px;padding:10px 15px;outline:none;min-width:90px;font-weight:bold;}}QPushButton:hover{{background-color:{btn_hover_bg};border:1px solid {btn_border_color};}}QPushButton:pressed{{background-color:{btn_pressed_bg};}}QPushButton:disabled{{background-color:{disabled_bg_color};color:{disabled_text_color};border:1px solid {disabled_border_color};}}QPushButton[active="true"]{{background-color:{active_bg_color};color:{active_text_color};border:1px solid {active_border_color};font-weight:bold;}}QPushButton[active="true"]:hover{{background-color:{active_hover_bg};border:1px solid {active_border_color};}}QLabel{{color:#333333;padding:5px;}}QComboBox{{border:1px solid {btn_border_color};border-radius:3px;padding:4px 8px;min-height:24px;background-color:#FFFFFF;color:{btn_text_color};}}QComboBox:disabled{{background-color:{disabled_bg_color};color:{disabled_text_color};}}QComboBox::drop-down{{border:none;}}QComboBox QAbstractItemView{{border:1px solid {btn_border_color};background-color:#FFFFFF;color:{btn_text_color};selection-background-color:{active_bg_color};selection-color:{active_text_color};}}QSpinBox, QCheckBox {{ padding: 3px; }} QFormLayout {{ margin-top: 5px; }} /* Minor spacing adjustments */"""
    #     # Added QSpinBox, QCheckBox padding and FormLayout margin
    #     self.setStyleSheet(stylesheet)

    def _apply_styles(self):
        """Applies the CSS stylesheet to the application."""

        # --- Define Colors (Text colors set to white as per previous request) ---
        bg_color="transparent"
        font_family="Segoe UI, Arial, sans-serif"
        font_size="18pt" # Applied globally

        # Specific Colors
        label_text_color = "#FFFFFF"         # White for Labels & default text
        checkbox_text_color = "#FFFFFF"      # White for Checkboxes
        button_text_color = "#27496D"       # Original Dark Blue/Gray for Buttons
        combo_text_color = "#27496D"        # Original Dark Blue/Gray for ComboBox text
        tab_unselected_text_color = "#006400"# Dark Green for unselected tabs
        tab_selected_text_color = "#228B22"  # Forest Green for selected tab

        # Button Backgrounds / Borders etc (no changes needed here)
        btn_bg_color="#E1E8F0"; btn_border_color="#B0C4DE"; btn_hover_bg="#CAD7E3"; btn_pressed_bg="#B0C4DE";
        active_bg_color="#65A8D7"; active_text_color="#FFFFFF"; active_border_color="#5595C7"; active_hover_bg="#85BBE1";
        disabled_bg_color="#E8E8E8"; disabled_text_color="#A0A0A0"; disabled_border_color="#D0D0D0";
        tab_selected_color="#FFFFFF"; tab_bg_color="#D4DDE6";
        slider_groove_bg="#FFFFFF"; slider_handle_bg=active_bg_color; slider_handle_border=active_border_color;

        # --- Construct Stylesheet ---
        stylesheet=f"""
        QWidget {{
            background-color: {bg_color};
            font-family: {font_family};
            font-size: {font_size};
            color: {label_text_color}; /* Default text color is white */
            font-weight: bold; /* << ADDED: Make default text bold */
        }}
        QTabBar::tab {{
            background:{tab_bg_color};
            color: {tab_unselected_text_color}; /* Green for unselected tabs */
            border:1px solid {btn_border_color};
            border-bottom:none;
            border-top-left-radius:4px;
            border-top-right-radius:4px;
            padding:8px 15px;
            margin-right:2px;
            font-weight: bold; /* Already bold */
        }}
        QTabBar::tab:hover {{
            background:{btn_hover_bg};
        }}
        QTabBar::tab:selected {{
            background:{tab_selected_color};
            color: {tab_selected_text_color}; /* Green for selected tabs */
            border-color:{btn_border_color};
            border-bottom:1px solid {tab_selected_color};
            font-weight: bold; /* Ensure selected is also bold if not inheriting */
        }}
        QTabWidget::pane {{
            border:1px solid {btn_border_color};
            border-top:none;
             background-color: transparent; /* If you want watermark fully visible */
        }}
        QPushButton {{
            background-color:{btn_bg_color};
            color: {button_text_color}; /* Buttons use specific non-white color */
            border:1px solid {btn_border_color};
            border-radius:5px;
            padding:10px 15px;
            outline:none;
            min-width:90px;
            font-weight: bold; /* Already bold */
        }}
        QPushButton:hover {{ background-color:{btn_hover_bg}; border:1px solid {btn_border_color}; }}
        QPushButton:pressed {{ background-color:{btn_pressed_bg}; }}
        QPushButton:disabled {{ background-color:{disabled_bg_color}; color:{disabled_text_color}; border:1px solid {disabled_border_color}; font-weight: bold;}} /* Keep bold when disabled */
        QPushButton[active="true"] {{ background-color:{active_bg_color}; color:{active_text_color}; border:1px solid {active_border_color}; font-weight:bold; }} /* Already bold */
        QPushButton[active="true"]:hover {{ background-color:{active_hover_bg}; border:1px solid {active_border_color}; }}
        QLabel {{
            color: {label_text_color}; /* Explicitly White for Labels */
            /* font-weight: bold; */ /* No need here, inherits from QWidget */
            padding:5px;
            background-color: transparent; /* Keep labels transparent */
        }}
        QComboBox {{
            border:1px solid {btn_border_color};
            border-radius:3px;
            padding:4px 8px;
            min-height:24px;
            background-color:#FFFFFF;
            color: {combo_text_color}; /* ComboBox uses specific non-white color */
            /* font-weight: bold; */ /* Will now inherit bold from QWidget */
        }}
        QComboBox:disabled {{ background-color:{disabled_bg_color}; color:{disabled_text_color}; }}
        QComboBox::drop-down {{ border:none; }}
        QComboBox QAbstractItemView {{
            border:1px solid {btn_border_color};
            background-color:#FFFFFF;
            color: {combo_text_color}; /* Dropdown list text */
            selection-background-color:{active_bg_color};
            selection-color:{active_text_color};
            /* font-weight: bold; */ /* Make dropdown items bold too? */
        }}
        QSpinBox {{ /* Spinbox text/input likely inherits bold from QWidget */
             padding: 3px;
        }}
        QCheckBox {{
             padding: 3px;
             color: {checkbox_text_color}; /* Explicitly White for CheckBox Text */
             /* font-weight: bold; */ /* No need here, inherits from QWidget */
        }}
        QFormLayout {{
             margin-top: 5px;
        }}
        QWidget#tabContentContainer {{ /* Keep rule for transparent container */
            background-color: transparent;
            border: none;
        }}
        """
        self.setStyleSheet(stylesheet)


    def _refresh_button_style(self, button):
        """Forces a re-evaluation of the button's stylesheet."""
        if button:
            try:
                style=button.style()
                style.unpolish(button)
                style.polish(button)
            except Exception as e:
                print(f"Warn: Style refresh failed for {button.objectName()}: {e}")


    @pyqtSlot(QPushButton)
    def on_button_click(self, clicked_button):
        """Handles clicks on main action buttons (Connect, Record)."""
        # This ONLY handles the Connect/Record buttons now. Playback uses separate handlers.
        if clicked_button not in self.action_buttons:
            print(f"Warn: Click event from non-action button: {clicked_button}")
            return

        # Prevent action if playback is running
        if self._is_playing:
            QMessageBox.warning(self, "Action Blocked", "Cannot start Connect or Record actions while Playback is running.")
            return

        clicked_obj_name=clicked_button.objectName()
        is_active=clicked_button.property("active") or False
        activate = not is_active # Target state
        is_dt_ctrl = clicked_obj_name in DRAG_TEACH_CONTROLLERS

        print(f"Action Button Click: '{clicked_obj_name}'. Currently Active: {is_active}. Target Action: {'Activate' if activate else 'Deactivate'}")

        # --- Deactivation Logic ---
        if not activate:
            print(f"  Deactivating '{clicked_obj_name}'...")
            clicked_button.setProperty("active", False)
            self._refresh_button_style(clicked_button)
            if is_dt_ctrl:
                self._disable_drag_teach()
            # If deactivating recording, ensure stop command sent (if needed)
            if clicked_obj_name == "RecordExercise" and self._is_recording:
                 print("  Deactivating Record button while recording was active. Stopping recording.")
                 self._stop_recording_action() # Ensure recording stops fully
            self._update_ui_state()
            return

        # --- Activation Logic ---
        print(f"  Attempting to Activate '{clicked_obj_name}'...")
        # 1. Deactivate any OTHER active conflicting button first (Connect vs Record)
        other_deactivated = False
        for btn in self.action_buttons:
            if btn and btn is not clicked_button and btn.property("active"):
                other_name = btn.objectName()
                print(f"    Found other active button '{other_name}'. Deactivating it.")
                btn.setProperty("active", False)
                self._refresh_button_style(btn)
                if other_name in DRAG_TEACH_CONTROLLERS:
                    self._disable_drag_teach()
                if other_name == "RecordExercise" and self._is_recording:
                     print("  Implicitly deactivating Record button while recording. Stopping recording.")
                     self._stop_recording_action()
                other_deactivated = True

        if other_deactivated:
            QApplication.processEvents(); time.sleep(0.1)

        # 2. Attempt to activate the clicked button
        can_activate = True
        if is_dt_ctrl:
            print(f"    '{clicked_obj_name}' requires Drag Teach. Attempting enable...")
            can_activate = self._enable_drag_teach()
            if not can_activate:
                print(f"    Drag Teach enable FAILED. Aborting activation of '{clicked_obj_name}'.")

        # 3. If activation possible, proceed
        action_to_dispatch = None
        if can_activate:
            print(f"    Activating visual state for '{clicked_obj_name}'")
            clicked_button.setProperty("active", True)
            self._refresh_button_style(clicked_button)
            if clicked_obj_name in self.action_map:
                action_to_dispatch = self.action_map[clicked_obj_name]
            else:
                print(f"    Warning: No action mapped for button '{clicked_obj_name}'")
        else:
            if clicked_button.property("active"):
                clicked_button.setProperty("active", False); self._refresh_button_style(clicked_button)
            print(f"    Activation of '{clicked_obj_name}' aborted.")


        # 4. Dispatch the associated action function if activation succeeded
        if action_to_dispatch:
            btn_text = clicked_button.text()
            print(f"    Dispatching action: {action_to_dispatch.__name__} for '{btn_text}'")
            try:
                action_to_dispatch() # Call _connect_patient or _record_exercise
            except Exception as e:
                print(f"ERROR executing action {action_to_dispatch.__name__} for '{btn_text}': {e}\n{traceback.format_exc()}")
                self._show_robot_error(f"Error during '{btn_text}' action:\n{e}", None)
                print(f"    Reverting activation of '{clicked_obj_name}' due to error.")
                clicked_button.setProperty("active", False); self._refresh_button_style(clicked_button)
                if is_dt_ctrl: self._disable_drag_teach()
        else:
            print("    No action dispatched.")

        self._update_ui_state()


    # --- Drag Teach Methods (_enable/_disable/_show_robot_error) remain the same ---
    def _enable_drag_teach(self):
        """Sends command to enable robot drag teach. Updates label. Returns True/False."""
        if not self.robot:
            self._show_robot_error("Cannot enable Drag Teach: Robot not connected.", None)
            if self.drag_status_label: self.drag_status_label.setText("Drag Teach: <b style='color: #C62828;'>OFF (Not Conn.)</b>")
            return False
        # Prevent enabling if playback is active
        if self._is_playing:
            self._show_robot_error("Cannot enable Drag Teach while Playback is active.", None)
            return False

        success = False
        label_updated = False
        try:
            print("  Sending Robot Command: DragTeachSwitch(1)")
            ret = self.robot.DragTeachSwitch(1)
            print(f"  DragTeachSwitch(1) Response Code: {ret}")
            success = (ret == 0)
        except AttributeError:
            msg = "Robot object missing the 'DragTeachSwitch' function."
            print(f"ERROR: {msg}"); self._show_robot_error(msg, None); success = False
        except Exception as e:
            print(f"  Exception during DragTeachSwitch(1): {e}"); self._show_robot_error(f"Exception enabling Drag Teach: {e}", None); success = False

        if self.drag_status_label:
            if success: self.drag_status_label.setText("Drag Teach: <b style='color: #2E7D32;'>ON</b>")
            else: self.drag_status_label.setText(f"Drag Teach: <b style='color: #C62828;'>OFF ({'Failed' if self.robot else 'Not Conn.'})</b>")
            label_updated = True

        print(f"  Drag Teach enable attempt result: {'Success' if success else 'FAILED'}.")
        if not success and self.robot and not label_updated: self._show_robot_error("Failed to enable Drag Teach via robot command.", 'Robot Error/Exception')
        return success

    def _disable_drag_teach(self):
        """Sends command to disable robot drag teach. Updates label. Returns True/False."""
        # If robot is already disconnected, Drag Teach is effectively off
        if not self.robot:
            if self.drag_status_label: self.drag_status_label.setText("Drag Teach: <b style='color: #C62828;'>OFF (Not Conn.)</b>")
            return True # Return True as it's already in the desired (off) state

        # If recording or playing, don't explicitly disable here; stop actions handle it.
        # However, this function might be called directly when switching buttons.
        # Let's allow disabling even if playing/recording, as the stop functions should handle state.

        success = False
        label_updated = False
        try:
            print("  Sending Robot Command: DragTeachSwitch(0)")
            # Use a short timeout for the disable command to avoid blocking UI
            # Note: The original Robot library might not support timeouts directly in RPC calls.
            # This is a conceptual addition; actual implementation depends on the library.
            # For now, call it directly.
            ret = self.robot.DragTeachSwitch(0)
            print(f"  DragTeachSwitch(0) Response Code: {ret}")
            success = (ret == 0)
        except AttributeError:
            msg = "Robot object missing the 'DragTeachSwitch' function."
            print(f"ERROR: {msg}"); self._show_robot_error(msg, None); success = False
            # Mark as success=True here? If the function doesn't exist, we can't turn it off,
            # but maybe we should treat it as "off" from the UI perspective?
            # Let's stick to False for now, indicating failure to execute command.
        except Exception as e:
            print(f"  Exception during DragTeachSwitch(0): {e}"); self._show_robot_error(f"Exception disabling Drag Teach: {e}", None); success = False

        if self.drag_status_label:
            # Always set to OFF if disable was attempted, regardless of success,
            # unless the robot is disconnected (handled above).
            # If the command fails, the label should reflect the attempt failed.
            if success:
                self.drag_status_label.setText("Drag Teach: <b style='color: #C62828;'>OFF</b>")
            else:
                # Keep previous error state? Or indicate disable failure?
                 # Let's show OFF but indicate failure if command didn't return 0
                 fail_reason = "(Cmd Failed)" if self.robot else "(Not Conn.)"
                 self.drag_status_label.setText(f"Drag Teach: <b style='color: #C62828;'>OFF {fail_reason}</b>")

            label_updated = True

        print(f"  Drag Teach disable attempt result: {'Success' if success else 'FAILED'}.")
        # Don't show error popup just for disabling failing if robot connected, only log it.
        # Error popups are mainly for enabling failures or connection issues.
        return success


    def _show_robot_error(self, message, title=None):
        """Displays a robot-related error message using QMessageBox."""
        if title is None: title = "Robot Communication Error"
        print(f"ERROR Displayed: [{title}] {message}") # Log the error too
        QMessageBox.warning(self, title, message)


    def _connect_patient(self):
        """ Placeholder function called when 'ConnecttoPatient' button is activated. """
        # This function is called *after* Drag Teach is successfully enabled.
        print("Action: Connect to Patient - Drag Teach should be ON.")
        # Add any specific logic needed for this state, if any.
        # Currently, just enabling Drag Teach via the button logic is the main action.
        self._update_ui_state() # Ensure UI reflects the active state


    def _record_exercise(self):
        """Initiates the recording process after Drag Teach is enabled."""
        print("Action: Record Exercise - Drag Teach should be ON.")
        if not self.robot:
             self._show_robot_error("Cannot start recording: Robot not connected.", "Recording Error")
             self._revert_active_button("RecordExercise") # Turn off button if robot disconnected
             return
        if self._is_recording:
             print("Warning: _record_exercise called while already recording.")
             return
        if self._is_playing: # Should be blocked by on_button_click, but double-check
             self._show_robot_error("Cannot start recording while playback is active.", "Recording Error")
             self._revert_active_button("RecordExercise")
             return

        # --- Get Trajectory Name ---
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        default_name = f"Exercise_{timestamp}"
        text, ok = QInputDialog.getText(self, 'Trajectory Name', 'Enter name for this exercise:', QLineEdit.EchoMode.Normal, default_name)

        if ok and text:
            self.current_recording_name = text.strip().replace(" ", "_").replace(",", "") # Sanitize
            print(f"Starting recording with name: '{self.current_recording_name}'")
            if not self.current_recording_name:
                self._show_robot_error("Invalid or empty trajectory name provided.", "Recording Error")
                self._revert_active_button("RecordExercise") # Turn off button
                self._disable_drag_teach() # Also disable drag teach
                return

            # --- Send Recording Start Commands ---
            recording_started = False
            try:
                print(f"  Sending Robot Command: SetTPDParam('{self.current_recording_name}', {RECORDING_PERIOD}, {RECORDING_DI})")
                ret_param = self.robot.SetTPDParam(self.current_recording_name, RECORDING_PERIOD, RECORDING_DI)
                print(f"  SetTPDParam Response Code: {ret_param}")
                if ret_param == 0:
                    print(f"  Sending Robot Command: SetTPDStart('{self.current_recording_name}', {RECORDING_PERIOD}, {RECORDING_DO})")
                    ret_start = self.robot.SetTPDStart(self.current_recording_name, RECORDING_PERIOD, RECORDING_DO)
                    print(f"  SetTPDStart Response Code: {ret_start}")
                    recording_started = (ret_start == 0)
                else:
                    print(f"  SetTPDParam failed, cannot start recording.")

            except AttributeError as e:
                msg = f"Robot object missing function needed for recording: {e}"
                print(f"ERROR: {msg}"); self._show_robot_error(msg, "Recording Error")
                recording_started = False
            except Exception as e:
                print(f"  Exception during recording start commands: {e}"); self._show_robot_error(f"Exception starting recording: {e}", "Recording Error")
                recording_started = False

            # --- Update State based on Success ---
            if recording_started:
                self._is_recording = True
                if self.record_status_label: self.record_status_label.setText(f"Status: <b style='color: #C62828;'>RECORDING</b> '{self.current_recording_name}'")
                print("Recording started successfully.")
                self._append_to_csv(self.current_recording_name) # Log immediately
            else:
                print("Failed to start recording via robot commands.")
                self._show_robot_error(f"Failed to start recording '{self.current_recording_name}' on the robot.", "Recording Error")
                self.current_recording_name = None
                self._revert_active_button("RecordExercise") # Turn off button
                self._disable_drag_teach() # Turn off drag teach as recording failed

        else: # User cancelled or entered empty name
            print("Recording cancelled by user or empty name.")
            self.current_recording_name = None
            self._revert_active_button("RecordExercise") # Turn off button
            self._disable_drag_teach() # Turn off drag teach

        self._update_ui_state()


    def _revert_active_button(self, object_name):
        """Finds a button by object name and sets its active property to False."""
        for btn in self.action_buttons:
            if btn and btn.objectName() == object_name:
                if btn.property("active"):
                    print(f"  Reverting active state for button '{object_name}'")
                    btn.setProperty("active", False)
                    self._refresh_button_style(btn)
                break


    def _stop_recording_action(self):
        """Handles the Stop Recording button click or implicit stop."""
        print("Stop Recording Action triggered.")
        if not self._is_recording:
            print("  Not currently recording.")
            # Ensure Drag Teach is off if button was active but state mismatch
            if self.record_button and self.record_button.property("active"):
                self._disable_drag_teach()
                self.record_button.setProperty("active", False)
                self._refresh_button_style(self.record_button)
            self._update_ui_state()
            return

        recording_name_was = self.current_recording_name
        print(f"Attempting to stop recording: '{recording_name_was}'")
        self._is_recording = False # Set state flag immediately
        self.current_recording_name = None

        # --- Send Stop Commands ---
        stop_success = False
        if self.robot:
            try:
                print("  Sending Robot Command: SetWebTPDStop()")
                ret_stop = self.robot.SetWebTPDStop()
                print(f"  SetWebTPDStop Response Code: {ret_stop}")
                # Even if stop command fails, proceed to disable drag teach

                print("  Sending Robot Command: DragTeachSwitch(0) (after stopping record)")
                ret_drag = self.robot.DragTeachSwitch(0)
                print(f"  DragTeachSwitch(0) Response Code: {ret_drag}")
                stop_success = (ret_stop == 0 and ret_drag == 0)

            except AttributeError as e:
                msg = f"Robot object missing function needed for stopping recording: {e}"
                print(f"ERROR: {msg}"); self._show_robot_error(msg, "Stopping Error")
            except Exception as e:
                print(f"  Exception during stop recording commands: {e}"); self._show_robot_error(f"Exception stopping recording: {e}", "Stopping Error")
        else:
            print("  Cannot send stop commands: Robot not connected.")
            # Consider it "stopped" locally, but maybe show warning?

        # --- Update UI ---
        if self.drag_status_label: # Update drag teach label explicitly
             off_reason = "(Not Conn.)" if not self.robot else ""
             self.drag_status_label.setText(f"Drag Teach: <b style='color: #C62828;'>OFF {off_reason}</b>")
        if self.record_status_label: self.record_status_label.setText(f"Status: Idle (Stopped '{recording_name_was}')")
        if self.record_button: # Ensure record button is visually deactivated
             self.record_button.setProperty("active", False)
             self._refresh_button_style(self.record_button)

        print(f"Recording '{recording_name_was}' stopped. Robot command success: {stop_success if self.robot else 'N/A'}")
        # Reload trajectory list in playback tab now that a new one might be saved
        self._load_trajectory_names() # Refresh playback list
        self._update_ui_state()


    def _append_to_csv(self, trajectory_name):
        """Appends a new entry (timestamp, name) to the CSV log file."""
        if not trajectory_name: return
        print(f"Logging trajectory '{trajectory_name}' to '{TRAJECTORY_CSV_FILE}'")
        file_exists = os.path.isfile(TRAJECTORY_CSV_FILE)
        try:
            with open(TRAJECTORY_CSV_FILE, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                # Write header if file doesn't exist or is empty
                if not file_exists or os.path.getsize(TRAJECTORY_CSV_FILE) == 0:
                    writer.writerow(['Timestamp', 'TrajectoryName', 'Notes']) # Example header
                    print("  CSV Header written.")
                # Write data row
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                writer.writerow([timestamp, trajectory_name, 'Recorded via UI'])
                print(f"  Appended: {timestamp}, {trajectory_name}")
        except Exception as e:
            print(f"ERROR writing to CSV file '{TRAJECTORY_CSV_FILE}': {e}\n{traceback.format_exc()}")
            self._show_robot_error(f"Could not log trajectory to file:\n{e}", "Logging Error")


    # =========================================================================
    # Playback Tab Methods
    # =========================================================================

    @pyqtSlot()
    def _start_playback_run(self):
        """Handles the 'Run Selected Trajectory' button click."""
        if not self.robot:
            self._show_robot_error("Cannot start playback: Robot not connected.", "Playback Error")
            return
        if not self.playback_worker or not self.playback_thread or not self.playback_thread.isRunning():
             self._show_robot_error("Cannot start playback: Worker thread not ready.", "Playback Error")
             # Try to set it up again?
             self._setup_playback_worker_thread()
             if not self.playback_worker or not self.playback_thread or not self.playback_thread.isRunning():
                 self._show_robot_error("Failed to setup worker thread. Cannot start playback.", "Critical Error")
                 return
             else:
                 QMessageBox.information(self, "Worker Ready", "Playback worker initialized. Please click Run again.")
                 return

        if self._is_playing:
            print("Warning: Playback run requested while already playing.")
            # Maybe stop the current run and start new? Or just ignore? Let's ignore.
            # self._stop_playback_motion() # Optional: Stop current before starting new
            # time.sleep(0.5) # Give time to process stop
            return
        if self._is_recording:
            self._show_robot_error("Cannot start playback while recording is active.", "Playback Error")
            return

        # Get parameters from UI
        selected_trajectory = self.playback_trajectory_combo.currentText()
        if not selected_trajectory or selected_trajectory.startswith("<"):
            self._show_robot_error("Please select a valid trajectory to run.", "Playback Error")
            return

        speed_override = self.playback_speed_input.value()
        repetitions = self.playback_repetitions_input.value()
        blend = 1 if self.playback_blend_checkbox.isChecked() else 0
        # The 5th arg to configure_run (ovl_unused) is ignored in the worker now
        ovl_for_config = float(speed_override) # Pass speed override value

        print(f"\n--- UI Thread: Requesting Playback ---")
        print(f"  Trajectory: '{selected_trajectory}'")
        print(f"  Speed Override: {speed_override}%")
        print(f"  Repetitions: {repetitions}")
        print(f"  Blend Mode: {blend}")

        self._is_playing = True
        # Reset labels for the new run
        self._playback_update_status_label(f"Status: Sending job '{selected_trajectory}'...")
        self._playback_update_rep_label(0, repetitions)
        self._update_ui_state() # Disable run button, enable stop/pause etc.

        # Reset worker's stop flag before triggering
        # self.playback_worker._stop_requested = False # Worker resets this in configure_run

        # Trigger the worker to configure and then run
        # Use invokeMethod for thread-safe call IF configure_run wasn't a slot
        # But since it's a slot connected via signal, direct signal emission is preferred
        print("  Emitting trigger_playback_run signal...")
        # Ensure configure_run runs before run_trajectory
        # Option 1: Two signals (might have race condition if run starts before config finishes?)
        # Option 2: Pass all args to run_trajectory (simpler)
        # Option 3: Use invokeMethod to ensure config finishes first (more complex)
        # Option 4: Let configure_run set flags, and run_trajectory checks flags (current approach)

        # Configure the worker via signal/slot
        self.trigger_playback_run.emit(selected_trajectory, speed_override, blend, repetitions, ovl_for_config)
        # Now trigger the run method using QMetaObject.invokeMethod for thread safety
        # This ensures it queues correctly in the worker's event loop
        QMetaObject.invokeMethod(self.playback_worker, 'run_trajectory', Qt.ConnectionType.QueuedConnection)
        print("  run_trajectory method invoked on worker thread.")


    @pyqtSlot()
    def _stop_playback_motion(self):
        """Handles the STOP button click during playback."""
        print("\n--- UI Thread: STOP Playback Requested ---")
        # Update UI immediately for responsiveness
        if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #DC143C;'>STOP Requested...</b>")
        # Disable Pause/Resume immediately
        self.playback_pause_button.setEnabled(False)
        self.playback_resume_button.setEnabled(False)
        # Keep STOP enabled briefly? Or disable after click? Let's disable.
        self.playback_stop_button.setEnabled(False)

        # 1. Signal the worker thread to stop its loop gracefully
        if self.playback_worker:
            print("  Signaling worker thread to stop...")
            # Use invokeMethod to ensure request_stop is called in the worker's thread
            QMetaObject.invokeMethod(self.playback_worker, 'request_stop', Qt.ConnectionType.QueuedConnection)
        else:
            print("  Warning: Stop requested but playback worker not found.")

        # 2. Send an immediate hardware StopMotion command (potentially redundant but safer)
        if self.robot:
            print("  Sending immediate Robot Command: StopMotion()")
            try:
                ret = self.robot.StopMotion()
                print(f"  StopMotion Response Code: {ret}")
                if ret != 0:
                    self._show_robot_error(f"StopMotion command failed (Code: {ret})", "Playback Stop Error")
            except AttributeError:
                 msg = "Robot object missing 'StopMotion' function."
                 print(f"ERROR: {msg}"); self._show_robot_error(msg, "Playback Stop Error")
            except Exception as e:
                 print(f"  Exception during StopMotion: {e}"); self._show_robot_error(f"Exception during StopMotion: {e}", "Playback Stop Error")
        else:
            print("  Cannot send StopMotion command: Robot not connected.")

        # Note: The worker thread will eventually emit finished(False, "Stopped by User...")
        # The _on_playback_worker_finished slot will handle the final UI state update (re-enabling Run, etc.)
        # We set _is_playing = False there.


    @pyqtSlot()
    def _pause_playback_motion(self):
        """Handles the Pause button click during playback."""
        print("\n--- UI Thread: PAUSE Playback Requested ---")
        if not self._is_playing:
             print("  Cannot pause: Playback not active.")
             return
        if not self.robot:
             self._show_robot_error("Cannot pause: Robot not connected.", "Playback Control Error")
             return

        # Update UI immediately
        self.playback_pause_button.setEnabled(False)
        self.playback_resume_button.setEnabled(True)
        self.playback_stop_button.setEnabled(True) # Stop should still be possible
        if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #FFA500;'>Pausing...</b>")

        # Send PauseMotion command
        print("  Sending Robot Command: PauseMotion()")
        try:
            ret = self.robot.PauseMotion()
            print(f"  PauseMotion Response Code: {ret}")
            if ret == 0:
                if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #FFA500;'>Paused</b>")
                # Note: We don't set self._is_playing = False here. It's still "active" but paused.
                # The worker's GetRobotMotionDone loop will just keep returning 0 until resumed or stopped.
            else:
                self._show_robot_error(f"PauseMotion command failed (Code: {ret})", "Playback Pause Error")
                # Revert UI if pause failed?
                self.playback_pause_button.setEnabled(True)
                self.playback_resume_button.setEnabled(False)
                if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #C62828;'>Pause Failed</b>")

        except AttributeError:
            msg = "Robot object missing 'PauseMotion' function."
            print(f"ERROR: {msg}"); self._show_robot_error(msg, "Playback Pause Error")
            self.playback_pause_button.setEnabled(True); self.playback_resume_button.setEnabled(False) # Revert UI
            if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #C62828;'>Pause Failed (No Func)</b>")
        except Exception as e:
            print(f"  Exception during PauseMotion: {e}"); self._show_robot_error(f"Exception during PauseMotion: {e}", "Playback Pause Error")
            self.playback_pause_button.setEnabled(True); self.playback_resume_button.setEnabled(False) # Revert UI
            if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #C62828;'>Pause Failed (Exception)</b>")

        # No change to self._is_playing state here


    @pyqtSlot()
    def _resume_playback_motion(self):
        """Handles the Resume button click during playback."""
        print("\n--- UI Thread: RESUME Playback Requested ---")
        if not self._is_playing:
             print("  Cannot resume: Playback not active.")
             return
        # Check if pause button is disabled (means pause succeeded or resume already active)
        # A better check might be needed if robot state can be read directly.
        # For now, assume if Resume is enabled, we should try sending the command.
        if self.playback_pause_button.isEnabled():
             print("  Warning: Resume clicked but Pause button is still enabled (implies not paused?). Sending command anyway.")
             # This could happen if Pause failed but UI wasn't reverted perfectly.

        if not self.robot:
             self._show_robot_error("Cannot resume: Robot not connected.", "Playback Control Error")
             return

        # Update UI immediately
        self.playback_pause_button.setEnabled(True)
        self.playback_resume_button.setEnabled(False)
        self.playback_stop_button.setEnabled(True)
        if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #90EE90;'>Resuming...</b>")

        # Send ResumeMotion command
        print("  Sending Robot Command: ResumeMotion()")
        try:
            ret = self.robot.ResumeMotion()
            print(f"  ResumeMotion Response Code: {ret}")
            if ret == 0:
                # Status label will be updated by the worker's status_update signal
                print("  Resume command sent successfully.")
                # Let worker update status to "Running..."
            else:
                self._show_robot_error(f"ResumeMotion command failed (Code: {ret})", "Playback Resume Error")
                # Revert UI if resume failed
                self.playback_pause_button.setEnabled(False)
                self.playback_resume_button.setEnabled(True)
                if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #C62828;'>Resume Failed</b>")

        except AttributeError:
            msg = "Robot object missing 'ResumeMotion' function."
            print(f"ERROR: {msg}"); self._show_robot_error(msg, "Playback Resume Error")
            self.playback_pause_button.setEnabled(False); self.playback_resume_button.setEnabled(True) # Revert UI
            if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #C62828;'>Resume Failed (No Func)</b>")
        except Exception as e:
            print(f"  Exception during ResumeMotion: {e}"); self._show_robot_error(f"Exception during ResumeMotion: {e}", "Playback Resume Error")
            self.playback_pause_button.setEnabled(False); self.playback_resume_button.setEnabled(True) # Revert UI
            if self.playback_status_label: self.playback_status_label.setText("Status: <b style='color: #C62828;'>Resume Failed (Exception)</b>")

        # No change to self._is_playing state here


    # --- Slots to receive signals from Playback Worker ---

    @pyqtSlot(str)
    def _playback_update_status_label(self, message):
        """Updates the playback status label from the worker thread."""
        if self.playback_status_label:
            # Add styling based on keywords if needed
            if "Error" in message or "Failed" in message:
                self.playback_status_label.setText(f"<b style='color: #C62828;'>{message}</b>")
            elif "Completed" in message or "Finished" in message:
                 self.playback_status_label.setText(f"<b style='color: #2E7D32;'>{message}</b>")
            elif "Paused" in message:
                 self.playback_status_label.setText(f"<b style='color: #FFA500;'>{message}</b>")
            else:
                self.playback_status_label.setText(message)
        # print(f"UI STATUS UPDATE: {message}") # Optional: log status updates


    @pyqtSlot(int, int)
    def _playback_update_rep_label(self, current_rep, total_reps):
        """Updates the repetition count label from the worker thread."""
        if self.playback_rep_count_label:
            self.playback_rep_count_label.setText(f"Rep: {current_rep} / {total_reps}")


    @pyqtSlot(bool, str)
    def _on_playback_worker_finished(self, success, message):
        """Handles the finished signal from the playback worker."""
        print(f"\n--- UI Thread: Playback Worker Finished ---")
        print(f"  Success: {success}")
        print(f"  Message: {message}")

        self._is_playing = False # Playback is no longer active

        # Update status label with the final message
        self._playback_update_status_label(f"Status: {message}")

        # Reset rep count label after a short delay? Or immediately?
        # Let's reset it.
        if self.playback_rep_count_label:
             self.playback_rep_count_label.setText("Rep: - / -")

        # Update UI buttons/state
        self._update_ui_state() # This should re-enable Run, disable Stop/Pause/Resume etc.

        # Optional: Show a final message box on success/failure
        if success:
            # QMessageBox.information(self, "Playback Complete", message)
            pass # Status label is usually sufficient
        else:
            # Error message box should have been shown by _on_playback_worker_error
            # If it finished due to Stop, the message indicates that.
            # If it finished due to an error caught by the worker's main try/except,
            # an error signal should have been emitted first.
            if "Stopped by User" not in message and "Error" not in message: # Catch unexpected non-success
                 QMessageBox.warning(self, "Playback Issue", f"Playback finished unexpectedly.\nMessage: {message}")


    @pyqtSlot(str, str)
    def _on_playback_worker_error(self, title, message):
        """Handles error signals from the playback worker."""
        print(f"\n--- UI Thread: Playback Worker ERROR Signal ---")
        print(f"  Title: {title}")
        print(f"  Message: {message}")

        # Show error message box
        self._show_robot_error(message, title)

        # Update status label to reflect the error
        self._playback_update_status_label(f"Status: <b style='color: #C62828;'>Error: {message}</b>")

        # Playback might still be technically "active" until the worker finishes
        # The worker's finished signal (likely with success=False) will trigger the final UI state update
        # So, we don't necessarily set _is_playing=False here, but we can disable controls.

        # Force UI update for controls immediately on error
        if self.playback_run_button: self.playback_run_button.setEnabled(False) # Keep Run disabled on error? Or enable? Let's disable until finished.
        if self.playback_stop_button: self.playback_stop_button.setEnabled(True) # Keep Stop enabled to allow forcing stop if needed
        if self.playback_pause_button: self.playback_pause_button.setEnabled(False)
        if self.playback_resume_button: self.playback_resume_button.setEnabled(False)


    # =========================================================================
    # UI State Management & Close Event
    # =========================================================================

    def _update_ui_state(self):
        """Updates the enabled/disabled state of UI elements based on robot connection and activity."""
        connected = self.robot is not None
        recording = self._is_recording
        playing = self._is_playing

        # --- General ---
        if self.robot_status_label:
            # Update status if not already set by connection logic
            current_text = self.robot_status_label.text()
            if connected and "Connected" not in current_text:
                 sim_text = "(Simulated)" if not ROBOT_AVAILABLE else ""
                 self.robot_status_label.setText(f"Status: <b style='color: {'orange' if not ROBOT_AVAILABLE else '#2E7D32'};'>Connected {sim_text}</b>")
            elif not connected and "Disconnected" not in current_text:
                 self.robot_status_label.setText("Status: <b style='color: #C62828;'>Disconnected</b>")

        # --- Connect Tab ---
        if self.connect_button:
            can_connect_drag = connected and not recording and not playing
            self.connect_button.setEnabled(can_connect_drag)
            # Style update handled by on_button_click

        # --- Record Tab ---
        if self.record_button:
             can_record = connected and not recording and not playing
             self.record_button.setEnabled(can_record)
             # Style update handled by on_button_click
        if self.stop_record_button:
             self.stop_record_button.setEnabled(recording) # Only enabled *during* recording
        if self.record_status_label:
             if not recording and "Idle" not in self.record_status_label.text():
                  # If recording stopped but label wasn't reset, reset it.
                  if "RECORDING" in self.record_status_label.text():
                      self.record_status_label.setText("Recording Status: Idle")
             # Active recording status set by start/stop functions

        # --- Playback Tab ---
        if self.playback_tab: # Check if tab exists
            # Combo box and refresh button
            can_load_list = not recording and not playing
            if self.playback_trajectory_combo:
                 # Enable combo only if connected and not busy
                 self.playback_trajectory_combo.setEnabled(connected and can_load_list)
            if self.playback_refresh_button:
                 self.playback_refresh_button.setEnabled(connected and can_load_list)

            # Parameter inputs
            can_set_params = connected and not playing
            if self.playback_speed_input: self.playback_speed_input.setEnabled(can_set_params)
            if self.playback_repetitions_input: self.playback_repetitions_input.setEnabled(can_set_params)
            if self.playback_blend_checkbox: self.playback_blend_checkbox.setEnabled(can_set_params)

            # Run button: requires connection, worker, trajectory selection, and not busy
            worker_ready = self.playback_worker is not None and self.playback_thread is not None and self.playback_thread.isRunning()

            # --- Start Modification ---
            # Calculate trajectory selection explicitly as boolean
            is_traj_selected = False
            if self.playback_trajectory_combo:
                # Check count > 0 to avoid index -1 if empty after clear/load
                if self.playback_trajectory_combo.count() > 0:
                     # Ensure index 0 is placeholder, valid selection is > 0
                     is_traj_selected = self.playback_trajectory_combo.currentIndex() > 0

            # Calculate can_run state using the explicit boolean
            can_run_bool = (
                connected
                and worker_ready
                and is_traj_selected # Use the explicit boolean
                and not recording
                and not playing
            )

            # Set the button state using the guaranteed boolean
            if self.playback_run_button:
                self.playback_run_button.setEnabled(can_run_bool) # Use the explicit bool
            # --- End Modification ---

            # Control buttons (Stop, Pause, Resume)
            if self.playback_stop_button: self.playback_stop_button.setEnabled(playing)
            if not playing:
                 if self.playback_pause_button: self.playback_pause_button.setEnabled(False)
                 if self.playback_resume_button: self.playback_resume_button.setEnabled(False)
            elif playing:
                 is_currently_paused = self.playback_resume_button.isEnabled()
                 if self.playback_pause_button: self.playback_pause_button.setEnabled(not is_currently_paused)


    def closeEvent(self, event):
        """Handles the application close event."""
        print("\n--- Application Closing ---")

        # 1. Stop any ongoing recording
        if self._is_recording:
            print("Stopping active recording before exit...")
            self._stop_recording_action()
            time.sleep(0.3) # Give a moment for commands

        # 2. Stop any ongoing playback
        if self._is_playing:
            print("Stopping active playback before exit...")
            self._stop_playback_motion()
            time.sleep(0.3) # Give a moment for commands

        # 3. Stop the playback worker thread
        if self.playback_thread and self.playback_thread.isRunning():
            print("Stopping playback worker thread...")
            # Signal worker to stop any waiting loops cleanly if possible
            if self.playback_worker:
                 QMetaObject.invokeMethod(self.playback_worker, 'request_stop', Qt.ConnectionType.QueuedConnection)
            self.playback_thread.quit()
            if not self.playback_thread.wait(1500): # Increase wait time slightly
                print("Warning: Playback thread did not quit gracefully. Terminating.")
                self.playback_thread.terminate() # Force quit if necessary

        # 4. Ensure Drag Teach is off (if connected)
        # Check self.robot directly, as connection might have dropped
        active_button = None
        for btn in self.action_buttons:
             if btn and btn.property("active"):
                 active_button = btn
                 break
        # Only disable if a button thinks it's active and robot seems connected
        if self.robot and active_button and active_button.objectName() in DRAG_TEACH_CONTROLLERS:
             print("Ensuring Drag Teach is disabled before exit...")
             self._disable_drag_teach() # Attempt to disable
             time.sleep(0.1)

        # 5. Clean up robot connection (optional, depends on library)
        # Some libraries might require an explicit disconnect or cleanup
        if self.robot and hasattr(self.robot, 'disconnect'): # Example disconnect
            try:
                print("Disconnecting robot object...")
                self.robot.disconnect()
            except Exception as e:
                print(f"Error during robot disconnect: {e}")
        self.robot = None

        print("Cleanup complete. Exiting application.")
        event.accept() # Accept the close event


# =============================================================================
# Main Execution Block
# =============================================================================
if __name__ == '__main__':
    app = QApplication(sys.argv)

    # Optional: Set application icon, name, etc.
    app.setWindowIcon(QIcon('Logo-01.jpg'))
    app.setApplicationName("Robot Control Interface")
    app.setOrganizationName("MyOrg") # Optional

    # Apply platform-specific style hints if desired
    import platform
    if platform.system() == "Windows":
        QApplication.setStyle("Fusion") # Or "WindowsVista"

    window = RobotControlTabsApp()
    window.show()
    sys.exit(app.exec())

