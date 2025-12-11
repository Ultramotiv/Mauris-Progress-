# -*- coding: utf-8 -*-
"""
PyQt6 UI for robot control using Tabs.

Features:
- Connect Tab: Connects robot, handles Drag Teach via button.
- Record Tab: Handles Drag Teach via button, records TPD, logs to CSV.
- Playback Tab: Selects TPD from CSV, sets global speed via slider,
  moves to start, implicitly starts playback, monitors completion via QTimer.
- Drag Teach: Enabled when 'Connect' OR 'Record' buttons are active.
- State Management: UI elements updated based on connection and activity.
- Error Handling: Uses QMessageBox.

Correction: Simplified widget creation in Play tab to avoid potential
            issues with exception handling masking assignment problems.
            Directly connected refresh button signal.
"""

import sys
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot
import time
import csv
from datetime import datetime
import os

# --- Robot Import ---
try:
    # IMPORTANT: Make sure the Robot library is accessible in your environment
    import Robot
    print("Successfully imported Robot library.")
    # Check for necessary methods instead of a single play command
    REQUIRED_METHODS = [
        "LoadTPD", "GetTPDStartPose", "MoveL", "SetSpeed",
        "GetRobotMotionDone", "StopMotion", "PauseMotion", "ResumeMotion",
        "DragTeachSwitch", "SetTPDParam", "SetTPDStart", "SetWebTPDStop"
        ] # Add any other essential methods used
except ImportError:
    print("-----------------------------------------------------------")
    print("FATAL ERROR: 'Robot' library not found.")
    print("-----------------------------------------------------------")
    # Dummy Robot for UI testing without library
    class Robot:
        class RPC:
            def __init__(self, ip): print(f"Warning: Dummy Robot created for IP {ip}"); raise ConnectionError("Dummy Robot")
            def __getattr__(self, name):
                print(f"Warning: Dummy Robot - Called missing method: {name}")
                if name in ["LoadTPD", "MoveL", "SetSpeed", "SetTPDParam", "SetTPDStart", "SetWebTPDStop", "StopMotion", "PauseMotion", "ResumeMotion", "DragTeachSwitch"]: return lambda *a, **k: 0
                if name == "GetTPDStartPose": return lambda *a, **k: (0, [0.0]*6)
                if name == "GetRobotMotionDone": return lambda *a, **k: (0, 1)
                return lambda *a, **k: None
    print("Warning: Using Dummy Robot class as real library not found.")
    REQUIRED_METHODS = [] # Skip checks if using dummy
# --- End Robot Import ---

from PyQt6.QtWidgets import (
    QApplication, QWidget, QPushButton, QGridLayout, QMessageBox,
    QInputDialog, QLineEdit, QTabWidget, QVBoxLayout, QLabel, QSizePolicy, QHBoxLayout,
    QComboBox, QSlider
)
from PyQt6.QtGui import QIcon, QFont
from PyQt6.QtCore import QSize, pyqtSlot, Qt, QTimer # Added QTimer & Qt

# --- Configuration ---
ROBOT_IP = '192.168.58.2'
TRAJECTORY_CSV_FILE = 'trajectory_log.csv'
RECORDING_PERIOD = 4
RECORDING_DI = 0
RECORDING_DO = 0
PLAYBACK_BLEND = 1
PLAYBACK_MONITOR_INTERVAL = 250 # ms

DRAG_TEACH_CONTROLLERS = {"ConnecttoPatient", "RecordExercise"}

class RobotControlTabsApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control Interface - Tabs")
        self.setGeometry(300, 200, 650, 450)

        # State Flags & Data
        self.robot = None
        self._is_recording = False
        self._is_playing = False
        self._is_paused = False
        self.current_recording_name = None
        self.current_playback_name = None

        # Playback Monitoring Timer
        self.playback_monitor_timer = QTimer(self)
        self.playback_monitor_timer.setInterval(PLAYBACK_MONITOR_INTERVAL)
        self.playback_monitor_timer.timeout.connect(self._check_playback_status)

        # UI Element References (Initialize explicitly to None for clarity)
        self.trajectory_combo = None # Initialize crucial element to None
        self.action_buttons = []
        self.robot_status_label = None; self.drag_status_label = None
        self.connect_button = None
        self.record_button = None; self.stop_record_button = None; self.record_status_label = None
        self.refresh_traj_button = None
        self.speed_slider = None; self.speed_label = None
        self.play_button = None; self.pause_button = None; self.resume_button = None
        self.stop_play_button = None; self.play_status_label = None

        # Action Map
        self.action_map = {
            "ConnecttoPatient": self._connect_patient,
            "RecordExercise": self._record_exercise,
            "PlayExercise": self._play_exercise
        }

        # Initialize UI Phases
        self._init_ui()        # Creates tabs and calls _create_..._tab methods
        self._connect_robot()  # Attempts connection AND calls _load_trajectory_names (conditionally)
        self._apply_styles()
        self._update_ui_state()


    def _connect_robot(self):
        """Attempts connection, checks methods, updates status, loads trajectories."""
        try:
            print(f"Attempting to connect to robot at {ROBOT_IP}...")
            self.robot = Robot.RPC(ROBOT_IP)
            print("Robot connection object created successfully.")
            if self.robot_status_label: self.robot_status_label.setText("Status: <b style='color: #2E7D32;'>Connected</b>")

            missing_methods = [m for m in REQUIRED_METHODS if not hasattr(self.robot, m)]
            if missing_methods:
                 warning_msg = f"Missing expected functions: {', '.join(missing_methods)}. Some features might fail."
                 print(f"WARNING: {warning_msg}")
                 QMessageBox.warning(self, "Robot Methods Missing", warning_msg)

        except Exception as e:
            self.robot = None
            print(f"FATAL: Failed to connect to robot: {e}")
            if self.robot_status_label: self.robot_status_label.setText("Status: <b style='color: #C62828;'>Disconnected</b>")
            if "Dummy Robot" not in str(e) and "RPC" not in str(type(e)): # Avoid popup for internal dummy error
                 QMessageBox.critical(self, "Robot Connection Failed", f"Could not connect to robot at {ROBOT_IP}.\nError: {e}\n\nCheck IP/network.")
            elif "Dummy Robot" in str(e):
                 print("Ignoring Dummy Robot connection error for popup.")
        finally:
             # --- Call Load Trajectories AFTER UI Setup ---
             print("DEBUG: _connect_robot finally block reached.")
             # Check if the combo box was successfully created in _create_play_tab
             if self.trajectory_combo is not None:
                 print("DEBUG: _connect_robot: trajectory_combo exists. Calling _load_trajectory_names.")
                 self._load_trajectory_names()
             else:
                 # This indicates a failure during UI creation
                 print(f"WARNING: _connect_robot: Cannot call _load_trajectory_names during startup because "
                       f"self.trajectory_combo is still None.")
             # ----------------------------------------
             self._update_ui_state() # Update UI based on connection status


    def _init_ui(self):
        """Initializes the main layout and tab structure."""
        main_layout = QVBoxLayout(self)
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)
        self.connect_tab = QWidget(); self.record_tab = QWidget(); self.play_tab = QWidget()
        self.tab_widget.addTab(self.connect_tab, "1. Connect")
        self.tab_widget.addTab(self.record_tab, "2. Record Exercise")
        self.tab_widget.addTab(self.play_tab, "3. Playback Exercise")
        # Create tabs AFTER adding them so references are valid
        self._create_connect_tab(); self._create_record_tab(); self._create_play_tab()
        self.setLayout(main_layout)


    def _create_connect_tab(self):
        """Creates UI elements for the Connect Tab."""
        layout = QVBoxLayout(self.connect_tab); layout.setAlignment(Qt.AlignmentFlag.AlignTop); layout.setSpacing(15)
        self.robot_status_label = QLabel("Status: Connecting...")
        self.drag_status_label = QLabel("Drag Teach: <b style='color: #C62828;'>OFF</b>")
        font = QFont(); font.setPointSize(11); self.robot_status_label.setFont(font); self.drag_status_label.setFont(font)
        layout.addWidget(self.robot_status_label); layout.addWidget(self.drag_status_label)
        self.connect_button = QPushButton("Connect to Patient / Enable Drag")
        self.connect_button.setObjectName("ConnecttoPatient"); self.connect_button.setMinimumHeight(50); self.connect_button.setProperty("active", False)
        self.connect_button.setToolTip("Establishes connection readiness & Enables/Disables Drag Teach Mode")
        self.connect_button.clicked.connect(lambda: self.on_button_click(self.connect_button))
        layout.addWidget(self.connect_button); self.action_buttons.append(self.connect_button)
        layout.addStretch()


    def _create_record_tab(self):
        """Creates UI elements for the Record Tab."""
        layout = QVBoxLayout(self.record_tab); layout.setAlignment(Qt.AlignmentFlag.AlignTop); layout.setSpacing(15)
        self.record_button = QPushButton("Record Exercise")
        self.record_button.setObjectName("RecordExercise"); self.record_button.setMinimumHeight(50); self.record_button.setProperty("active", False)
        self.record_button.setToolTip("Enables Drag Teach and starts recording trajectory")
        self.record_button.clicked.connect(lambda: self.on_button_click(self.record_button))
        layout.addWidget(self.record_button); self.action_buttons.append(self.record_button)
        self.stop_record_button = QPushButton("Stop Recording")
        self.stop_record_button.setObjectName("StopRecord"); self.stop_record_button.setMinimumHeight(50)
        self.stop_record_button.clicked.connect(self._stop_action)
        layout.addWidget(self.stop_record_button)
        self.record_status_label = QLabel("Recording Status: Idle")
        font = QFont(); font.setPointSize(11); self.record_status_label.setFont(font)
        layout.addWidget(self.record_status_label)
        layout.addStretch()


    def _create_play_tab(self):
        """Creates UI elements for the Playback Tab."""
        layout = QVBoxLayout(self.play_tab); layout.setAlignment(Qt.AlignmentFlag.AlignTop); layout.setSpacing(15)

        # --- Trajectory Selection Row ---
        traj_select_layout = QHBoxLayout()
        traj_label = QLabel("Select Exercise:")

        # Create ComboBox - Allow exceptions to propagate if creation fails
        print("DEBUG: _create_play_tab: Attempting to create self.trajectory_combo")
        self.trajectory_combo = QComboBox()
        print(f"DEBUG: _create_play_tab: self.trajectory_combo created: {self.trajectory_combo}")
        self.trajectory_combo.setToolTip("Select a recorded exercise")
        self.trajectory_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        # Create Refresh Button - Allow exceptions to propagate
        print("DEBUG: _create_play_tab: Attempting to create refresh_traj_button")
        self.refresh_traj_button = QPushButton("Refresh List")
        print(f"DEBUG: refresh_traj_button object created: {self.refresh_traj_button}")
        self.refresh_traj_button.setToolTip("Reload exercise names from log")
        # Connect signal directly
        self.refresh_traj_button.clicked.connect(self._load_trajectory_names)
        print(f"DEBUG: Connected refresh_traj_button.clicked signal directly to self._load_trajectory_names")


        # Add widgets to layout
        traj_select_layout.addWidget(traj_label)
        traj_select_layout.addWidget(self.trajectory_combo, 1) # Add the combo box
        traj_select_layout.addWidget(self.refresh_traj_button)
        layout.addLayout(traj_select_layout)

        # --- Speed Control Row ---
        speed_layout = QHBoxLayout()
        speed_title_label = QLabel("Playback Speed:")
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(1, 150); self.speed_slider.setValue(100)
        self.speed_slider.setTickInterval(10); self.speed_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.speed_slider.setToolTip("Adjust playback speed (1% to 150%)")
        self.speed_label = QLabel(f"{self.speed_slider.value()}%")
        self.speed_label.setMinimumWidth(40)
        self.speed_slider.valueChanged.connect(self._update_speed_label)
        speed_layout.addWidget(speed_title_label); speed_layout.addWidget(self.speed_slider); speed_layout.addWidget(self.speed_label)
        layout.addLayout(speed_layout)

        # --- Play Button ---
        self.play_button = QPushButton("Play Selected Exercise")
        self.play_button.setObjectName("PlayExercise"); self.play_button.setMinimumHeight(50)
        self.play_button.setProperty("active", False)
        self.play_button.clicked.connect(lambda: self.on_button_click(self.play_button))
        layout.addWidget(self.play_button); self.action_buttons.append(self.play_button)

        # --- Playback Control Buttons ---
        control_layout = QHBoxLayout()
        self.pause_button = QPushButton("Pause"); self.pause_button.setObjectName("PausePlayback"); self.pause_button.setMinimumHeight(45)
        self.resume_button = QPushButton("Resume"); self.resume_button.setObjectName("ResumePlayback"); self.resume_button.setMinimumHeight(45)
        self.stop_play_button = QPushButton("Stop Playback"); self.stop_play_button.setObjectName("StopPlayback"); self.stop_play_button.setMinimumHeight(45)
        self.pause_button.clicked.connect(self._pause_action); self.resume_button.clicked.connect(self._resume_action); self.stop_play_button.clicked.connect(self._stop_action)
        control_layout.addWidget(self.pause_button); control_layout.addWidget(self.resume_button); control_layout.addWidget(self.stop_play_button)
        layout.addLayout(control_layout)

        # --- Status Label ---
        self.play_status_label = QLabel("Playback Status: Idle")
        font = QFont(); font.setPointSize(11); self.play_status_label.setFont(font)
        layout.addWidget(self.play_status_label)

        layout.addStretch() # Add stretch at the end


    @pyqtSlot()
    def _load_trajectory_names(self):
        """Reads trajectory names from the CSV log file and populates the ComboBox."""
        # --- Guard Clause - Check if ComboBox exists ---
        if not self.trajectory_combo:
            print("ERROR: _load_trajectory_names called but self.trajectory_combo is None!")
            # Attempt to show a user-facing error as well if possible
            if hasattr(self, 'play_status_label') and self.play_status_label:
                self.play_status_label.setText("Status: Error (UI Init Failed)")
            return
        # --- End Guard Clause ---

        print(f"Attempting to load trajectories from '{TRAJECTORY_CSV_FILE}'...")
        try:
            abs_path = os.path.abspath(TRAJECTORY_CSV_FILE)
            print(f"  Checking absolute path: {abs_path}")
        except Exception as e:
            print(f"  Could not get absolute path: {e}")

        current_selection = self.trajectory_combo.currentText()
        self.trajectory_combo.clear() # Clear previous items

        if not os.path.isfile(TRAJECTORY_CSV_FILE):
            print(f"  Info: Log file not found at expected location.")
            self.trajectory_combo.addItem("<Log file not found>")
            self.trajectory_combo.setEnabled(False)
            self._update_ui_state()
            return

        trajectory_names = set()
        updated_count = 0
        name_col_index = 1 # Default to second column (index 1)

        try:
            with open(TRAJECTORY_CSV_FILE, 'r', newline='', encoding='utf-8') as csvfile:
                reader = csv.reader(csvfile)
                header = next(reader, None) # Read header row safely

                if header:
                    print(f"  CSV Header found: {header}")
                    hdr_low = [h.lower().strip() for h in header]
                    try:
                        name_col_index = hdr_low.index('trajectoryname') # Case-insensitive find
                        print(f"  Found 'trajectoryname' header at index {name_col_index}.")
                    except ValueError:
                        print(f"  Warning: 'trajectoryname' not found in header {header}. Defaulting to index 1 (second column).")
                else:
                     print(f"  Warning: No header row found in CSV. Assuming name is in index 1 (second column).")

                print(f"  >>> Using column index for name: {name_col_index}")

                for i, row in enumerate(reader):
                    # print(f"  Raw row {i + (1 if header else 0)}: {row}") # Debug each row
                    if len(row) > name_col_index:
                        name = row[name_col_index].strip()
                        # print(f"    Extracted name (from index {name_col_index}): '{name}'") # Debug extracted name
                        if name: # Only add if name is not empty after stripping
                            trajectory_names.add(name)
                        # else: print(f"    Skipping empty name.")
                    # else: print(f"  Warning: Row {i + (1 if header else 0)} is too short (len={len(row)}) to get index {name_col_index}.")

            updated_count = len(trajectory_names)
            print(f"  >>> Final unique names found: {trajectory_names}") # Debug final set

        except Exception as e:
            print(f"  ERROR reading CSV file '{TRAJECTORY_CSV_FILE}': {e}")
            self.trajectory_combo.addItem("<Error reading log>")
            self.trajectory_combo.setEnabled(False)
            self._update_ui_state()
            return

        # Populate ComboBox
        if trajectory_names:
            sorted_names = sorted(list(trajectory_names))
            self.trajectory_combo.addItems(sorted_names)
            if current_selection in sorted_names: # Try to restore previous selection
                self.trajectory_combo.setCurrentText(current_selection)
            self.trajectory_combo.setEnabled(True)
            print(f"  Successfully loaded {updated_count} unique trajectory names into ComboBox.")
        else:
            print("  Info: No valid trajectory names found in the log file.")
            self.trajectory_combo.addItem("<No trajectories in log>")
            self.trajectory_combo.setEnabled(False)

        self._update_ui_state() # Update button enable states etc.


    @pyqtSlot(int)
    def _update_speed_label(self, value):
        """Updates the label next to the speed slider."""
        if self.speed_label: self.speed_label.setText(f"{value}%")


    def _apply_styles(self):
        """Applies the CSS stylesheet to the application."""
        bg_color="#F0F4F8"; btn_bg_color="#E1E8F0"; btn_text_color="#27496D"; btn_border_color="#B0C4DE"; btn_hover_bg="#CAD7E3"; btn_pressed_bg="#B0C4DE"; active_bg_color="#65A8D7"; active_text_color="#FFFFFF"; active_border_color="#5595C7"; active_hover_bg="#85BBE1"; disabled_bg_color="#E8E8E8"; disabled_text_color="#A0A0A0"; disabled_border_color="#D0D0D0"; font_family="Segoe UI, Arial, sans-serif"; font_size="10pt"; tab_selected_color="#FFFFFF"; tab_bg_color="#D4DDE6"; slider_groove_bg="#FFFFFF"; slider_handle_bg=active_bg_color; slider_handle_border=active_border_color;
        stylesheet=f"""QWidget{{background-color:{bg_color};font-family:{font_family};font-size:{font_size};color:#333333;}}QTabBar::tab{{background:{tab_bg_color};color:{btn_text_color};border:1px solid {btn_border_color};border-bottom:none;border-top-left-radius:4px;border-top-right-radius:4px;padding:8px 15px;margin-right:2px;font-weight:bold;}}QTabBar::tab:hover{{background:{btn_hover_bg};}}QTabBar::tab:selected{{background:{tab_selected_color};color:{btn_text_color};border-color:{btn_border_color};border-bottom:1px solid {tab_selected_color};}}QTabWidget::pane{{border:1px solid {btn_border_color};border-top:none;background-color:{tab_selected_color};}}QPushButton{{background-color:{btn_bg_color};color:{btn_text_color};border:1px solid {btn_border_color};border-radius:5px;padding:10px 15px;outline:none;min-width:90px;font-weight:bold;}}QPushButton:hover{{background-color:{btn_hover_bg};border:1px solid {btn_border_color};}}QPushButton:pressed{{background-color:{btn_pressed_bg};}}QPushButton:disabled{{background-color:{disabled_bg_color};color:{disabled_text_color};border:1px solid {disabled_border_color};}}QPushButton[active="true"]{{background-color:{active_bg_color};color:{active_text_color};border:1px solid {active_border_color};font-weight:bold;}}QPushButton[active="true"]:hover{{background-color:{active_hover_bg};border:1px solid {active_border_color};}}QLabel{{color:#333333;padding:5px;}}QComboBox{{border:1px solid {btn_border_color};border-radius:3px;padding:4px 8px;min-height:24px;background-color:#FFFFFF;color:{btn_text_color};}}QComboBox:disabled{{background-color:{disabled_bg_color};color:{disabled_text_color};}}QComboBox::drop-down{{border:none;}}QComboBox QAbstractItemView{{border:1px solid {btn_border_color};background-color:#FFFFFF;color:{btn_text_color};selection-background-color:{active_bg_color};selection-color:{active_text_color};}}QSlider::groove:horizontal{{border:1px solid #bbb;background:{slider_groove_bg};height:8px;border-radius:4px;}}QSlider::handle:horizontal{{background:{slider_handle_bg};border:1px solid {slider_handle_border};width:18px;margin:-5px 0;border-radius:9px;}}QSlider::add-page:horizontal{{background:#ddd;border-radius:4px;}}QSlider::sub-page:horizontal{{background:{slider_handle_bg};border-radius:4px;}}"""
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
        """Handles clicks on main action buttons (Connect, Record, Play)."""
        if clicked_button not in self.action_buttons:
             print(f"Warn: Click event from unknown button: {clicked_button}")
             return

        clicked_obj_name=clicked_button.objectName()
        is_active=clicked_button.property("active") or False
        activate = not is_active # Target state
        is_dt_ctrl = clicked_obj_name in DRAG_TEACH_CONTROLLERS

        print(f"Button Click: '{clicked_obj_name}'. Currently Active: {is_active}. Target Action: {'Activate' if activate else 'Deactivate'}")

        # --- Deactivation Logic ---
        if not activate: # If the goal is to deactivate the clicked button
            print(f"  Deactivating '{clicked_obj_name}'...")
            clicked_button.setProperty("active", False)
            self._refresh_button_style(clicked_button)
            if is_dt_ctrl:
                self._disable_drag_teach() # Disable DT if it was a controller
            # If deactivating Play, ensure playback stops fully
            if clicked_obj_name == "PlayExercise":
                print("  Deactivating Play button explicitly - stopping playback if active.")
                self._stop_action(stop_button_itself=False) # Stop playback without trying to style sender

            self._update_ui_state()
            return # Finished deactivation

        # --- Activation Logic ---
        print(f"  Attempting to Activate '{clicked_obj_name}'...")
        # 1. Deactivate any OTHER active conflicting button first
        other_deactivated = False
        for btn in self.action_buttons:
            if btn and btn is not clicked_button and btn.property("active"):
                other_name = btn.objectName()
                print(f"    Found other active button '{other_name}'. Deactivating it.")
                btn.setProperty("active", False)
                self._refresh_button_style(btn)
                if other_name in DRAG_TEACH_CONTROLLERS:
                    self._disable_drag_teach() # Disable DT if it was a controller
                # If implicitly deactivating Play, ensure playback stops fully
                if other_name == "PlayExercise":
                     print("    Implicitly deactivating Play button - stopping playback if active.")
                     self._stop_action(stop_button_itself=False) # Stop playback without styling sender
                other_deactivated = True

        if other_deactivated:
            QApplication.processEvents() # Allow UI to update after deactivation
            time.sleep(0.1) # Small delay if something was stopped

        # 2. Attempt to activate the clicked button
        can_activate = True
        if is_dt_ctrl: # If it needs Drag Teach, try enabling it first
            print(f"    '{clicked_obj_name}' requires Drag Teach. Attempting enable...")
            can_activate = self._enable_drag_teach()
            if not can_activate:
                print(f"    Drag Teach enable FAILED. Aborting activation of '{clicked_obj_name}'.")

        # 3. If activation possible (DT enabled or not needed), proceed
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
             # If we failed (e.g., DT failed), ensure button visual state is inactive
             if clicked_button.property("active"):
                  clicked_button.setProperty("active", False)
                  self._refresh_button_style(clicked_button)
             print(f"    Activation of '{clicked_obj_name}' aborted.")


        # 4. Dispatch the associated action function if activation succeeded
        if action_to_dispatch:
            btn_text = clicked_button.text() # Use current text for logging
            print(f"    Dispatching action: {action_to_dispatch.__name__} for '{btn_text}'")
            try:
                action_to_dispatch()
            except Exception as e:
                print(f"ERROR executing action {action_to_dispatch.__name__} for '{btn_text}': {e}")
                self._show_robot_error(f"Error during '{btn_text}' action:\n{e}", None)
                # Revert activation on error
                print(f"    Reverting activation of '{clicked_obj_name}' due to error.")
                clicked_button.setProperty("active", False)
                self._refresh_button_style(clicked_button)
                if is_dt_ctrl:
                    self._disable_drag_teach() # Ensure DT is off if controller failed
        else:
            print("    No action dispatched.")

        self._update_ui_state() # Update UI based on the final state


    def _enable_drag_teach(self):
        """Sends command to enable robot drag teach. Updates label. Returns True/False."""
        if not self.robot:
            self._show_robot_error("Cannot enable Drag Teach: Robot not connected.", None)
            if self.drag_status_label: self.drag_status_label.setText("Drag Teach: <b style='color: #C62828;'>OFF (Not Conn.)</b>")
            return False

        success = False
        label_updated = False
        try:
            print("  Sending Robot Command: DragTeachSwitch(1)")
            ret = self.robot.DragTeachSwitch(1)
            print(f"  DragTeachSwitch(1) Response Code: {ret}")
            success = (ret == 0) # Assume 0 means success
        except AttributeError:
            msg = "Robot object missing the 'DragTeachSwitch' function."
            print(f"ERROR: {msg}")
            self._show_robot_error(msg, None)
            success = False
        except Exception as e:
            print(f"  Exception during DragTeachSwitch(1): {e}")
            self._show_robot_error(f"Exception enabling Drag Teach: {e}", None)
            success = False

        if self.drag_status_label:
            if success:
                self.drag_status_label.setText("Drag Teach: <b style='color: #2E7D32;'>ON</b>")
            else:
                 # Show failure reason if possible
                 reason = "Failed" if self.robot else "Not Conn."
                 self.drag_status_label.setText(f"Drag Teach: <b style='color: #C62828;'>OFF ({reason})</b>")
            label_updated = True

        print(f"  Drag Teach enable attempt result: {'Success' if success else 'FAILED'}.")

        # Extra check: if enabling failed but we have a robot connection, show message
        if not success and self.robot and not label_updated:
             self._show_robot_error("Failed to enable Drag Teach via robot command.", 'Robot Error/Exception')

        return success


    def _disable_drag_teach(self):
        """Sends command to disable robot drag teach. Updates label. Returns True/False."""
        # Always update label regardless of robot state
        if self.drag_status_label:
            self.drag_status_label.setText("Drag Teach: <b style='color: #C62828;'>OFF</b>")

        if not self.robot:
            print("Info: Cannot send disable Drag Teach command - Robot not connected.")
            return False # Cannot confirm success if not connected

        print("  Attempting to disable Drag Teach via robot command...")
        success = False
        try:
            print("  Sending Robot Command: DragTeachSwitch(0)")
            ret = self.robot.DragTeachSwitch(0)
            print(f"  DragTeachSwitch(0) Response Code: {ret}")
            success = (ret == 0) # Assume 0 means success
        except AttributeError:
             # Don't show error popup here, just log it, as it might be called during cleanup
            print("ERROR: Robot object missing the 'DragTeachSwitch' function (during disable).")
            success = False
        except Exception as e:
            print(f"  Exception during DragTeachSwitch(0): {e}")
            # Don't show error popup here either
            success = False

        print(f"  Drag Teach disable command sent. Assumed result: {'OK' if success else 'FAILED/Error'}.")
        # We don't strictly need the return value often, but return success status
        return success


    def _show_robot_error(self, message, error_code):
        """Displays a warning message box for robot-related errors."""
        print(f"--- ROBOT ERROR ---")
        print(f"  Message: {message}")
        if error_code is not None: print(f"  Code: {error_code}")
        print(f"-------------------")
        detail_text = f"Error Code: {error_code}" if error_code is not None else "No error code provided."
        # Check if window is visible to avoid errors during shutdown
        if self.isVisible():
             QMessageBox.warning(self, "Robot Interaction Error", f"{message}\n\n{detail_text}")


    def _update_ui_state(self):
        """Central method to enable/disable UI elements based on current state."""
        is_connected = self.robot is not None
        is_recording = self._is_recording
        is_playing = self._is_playing
        is_paused = self._is_paused

        # Determine if Drag Teach *should* be active based on button states
        is_dt_demanded = any(b.property("active") for b in self.action_buttons if b and b.objectName() in DRAG_TEACH_CONTROLLERS)

        # Conditions for specific UI states
        can_connect = is_connected and not is_recording and not is_playing and not is_paused
        can_record = is_connected and not is_playing and not is_recording # Can't start recording if already recording
        # Playback setup allowed if connected, not recording, AND the UI was created correctly
        # Use direct check for trajectory_combo existence
        can_setup_play = is_connected and not is_recording and (self.trajectory_combo is not None)
        can_start_play = can_setup_play and not is_playing and not is_paused # Can press Play if ready and not already playing/paused

        # Check if a valid trajectory is selected (only if combo exists)
        valid_trajectory_selected = False
        if self.trajectory_combo: # Check existence again
            current_traj = self.trajectory_combo.currentText()
            # Valid if not empty and not one of the placeholder texts
            valid_trajectory_selected = bool(current_traj) and not current_traj.startswith("<")

        can_start_play = can_start_play and valid_trajectory_selected

        # Update Button Enablement
        if self.connect_button: self.connect_button.setEnabled(can_connect)
        if self.record_button: self.record_button.setEnabled(can_record)
        if self.play_button: self.play_button.setEnabled(can_start_play) # Depends on valid selection now

        if self.stop_record_button: self.stop_record_button.setEnabled(is_connected and is_recording)

        # Update Playback Setup Controls Enablement
        if self.trajectory_combo: self.trajectory_combo.setEnabled(can_setup_play)
        if self.refresh_traj_button: self.refresh_traj_button.setEnabled(can_setup_play)
        if self.speed_slider: self.speed_slider.setEnabled(can_setup_play)

        # Update Playback Action Controls Enablement
        if self.pause_button: self.pause_button.setEnabled(is_connected and is_playing and not is_paused)
        if self.resume_button: self.resume_button.setEnabled(is_connected and is_playing and is_paused)
        if self.stop_play_button: self.stop_play_button.setEnabled(is_connected and (is_playing or is_paused))

        # Update Status Labels (simplified, relies on action methods for detailed text)
        # Drag Teach label is updated primarily by _enable/_disable_drag_teach

        print(f"UI State Updated: Connected={is_connected}, Recording={is_recording}, Playing={is_playing}, Paused={is_paused}, PlayReady={can_start_play}, DT_Demanded={is_dt_demanded}, ComboOK={self.trajectory_combo is not None}")


    @pyqtSlot()
    def _connect_patient(self):
        """Action logic for 'Connect to Patient' button activation."""
        print("ACTION: Connect Patient Triggered")
        if not self.robot:
            self._show_robot_error("Robot not connected.", None)
            # Ensure button is deactivated visually if connection failed during enable
            if self.connect_button and self.connect_button.property("active"):
                 self.connect_button.setProperty("active", False)
                 self._refresh_button_style(self.connect_button)
            self._update_ui_state() # Update UI based on failed state
            return

        # If we got here, Drag Teach should have been enabled by on_button_click
        print("  Connect Patient Action: Drag Teach presumed ON.")
        self._update_ui_state()


    @pyqtSlot()
    def _record_exercise(self):
        """Action logic for 'Record Exercise' button activation."""
        print("ACTION: Record Exercise Triggered")
        # Pre-checks
        if not self.robot:
            print("ERROR: Record action called but robot is None.")
            if self.record_button and self.record_button.property("active"):
                 self.record_button.setProperty("active", False); self._refresh_button_style(self.record_button)
                 self._disable_drag_teach() # Ensure DT is off
            self._update_ui_state()
            return
        if self._is_recording or self._is_playing:
            print(f"WARNING: Record action called while already busy (Recording:{self._is_recording}, Playing:{self._is_playing}) - Ignoring.")
            if self.record_button and self.record_button.property("active"):
                 self.record_button.setProperty("active", False); self._refresh_button_style(self.record_button)
                 self._disable_drag_teach() # Ensure DT is off
            self._update_ui_state()
            return

        # Get trajectory name from user
        default_name = "Exercise_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        name, ok = QInputDialog.getText(self, "Record Trajectory",
                                        "Enter a name for this exercise:",
                                        QLineEdit.EchoMode.Normal, default_name)

        if not ok or not name or not name.strip():
            print("Info: Recording cancelled by user or empty name entered.")
            if self.record_button and self.record_button.property("active"):
                 self.record_button.setProperty("active", False); self._refresh_button_style(self.record_button)
                 self._disable_drag_teach() # Turn off DT since recording didn't start
            self._update_ui_state()
            return

        name = name.strip()
        print(f"  User provided name: '{name}'")

        # Log the name to the CSV file first
        log_ok = False
        try:
            file_exists = os.path.isfile(TRAJECTORY_CSV_FILE)
            needs_header = not file_exists or os.path.getsize(TRAJECTORY_CSV_FILE) == 0

            with open(TRAJECTORY_CSV_FILE, 'a', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                if needs_header:
                    writer.writerow(['Timestamp', 'TrajectoryName']) # Write header
                    print(f"  CSV '{TRAJECTORY_CSV_FILE}' was empty or missing. Header written.")
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                writer.writerow([timestamp, name])
                print(f"  Successfully logged '{name}' to '{TRAJECTORY_CSV_FILE}' at {timestamp}")
            log_ok = True
            # Reload names into ComboBox only if it exists
            if self.trajectory_combo is not None:
                self._load_trajectory_names()
            else:
                 print("WARNING: Trajectory logged, but ComboBox does not exist to refresh.")
        except IOError as e:
            self._show_robot_error(f"Failed to write to log file '{TRAJECTORY_CSV_FILE}':\n{e}", None)
            log_ok = False

        # If logging failed, abort the recording process
        if not log_ok:
            print("  Aborting record start due to logging error.")
            if self.record_button and self.record_button.property("active"):
                 self.record_button.setProperty("active", False); self._refresh_button_style(self.record_button)
                 self._disable_drag_teach() # Turn off DT
            self._update_ui_state()
            return

        # Send commands to the robot to start recording
        try:
            print(f"  Configuring TPD recording parameters for '{name}'...")
            retP = self.robot.SetTPDParam(name, RECORDING_PERIOD, di_choose=RECORDING_DI)
            print(f"    SetTPDParam Response: {retP}")
            if retP != 0: raise Exception(f"SetTPDParam failed with code {retP}")

            print(f"  Starting TPD recording for '{name}'...")
            retS = self.robot.SetTPDStart(name, RECORDING_PERIOD, do_choose=RECORDING_DO)
            print(f"    SetTPDStart Response: {retS}")
            if retS != 0: raise Exception(f"SetTPDStart failed with code {retS}")

            # If commands succeed, update state
            print(f"  Robot recording started successfully for '{name}'.")
            self._is_recording = True
            self.current_recording_name = name
            if self.record_status_label: self.record_status_label.setText(f"Status: <b style='color:red;'>RECORDING</b> ({name})")

        except AttributeError as e:
             missing_func = "?"
             if 'SetTPDParam' in str(e): missing_func = 'SetTPDParam'
             elif 'SetTPDStart' in str(e): missing_func = 'SetTPDStart'
             msg = f"Robot object is missing the function '{missing_func}' needed for recording."
             self._show_robot_error(msg, None)
             self._is_recording = False; self.current_recording_name = None
             if self.record_status_label: self.record_status_label.setText("Status: Error (Missing Func)")
             if self.record_button and self.record_button.property("active"):
                  self.record_button.setProperty("active", False); self._refresh_button_style(self.record_button)
                  self._disable_drag_teach()
        except Exception as e:
            self._show_robot_error(f"Failed to start recording on robot for '{name}':\n{e}", None)
            self._is_recording = False; self.current_recording_name = None
            if self.record_status_label: self.record_status_label.setText("Status: Error (Robot Setup)")
            if self.record_button and self.record_button.property("active"):
                 self.record_button.setProperty("active", False); self._refresh_button_style(self.record_button)
                 self._disable_drag_teach()

        self._update_ui_state()


    @pyqtSlot()
    def _play_exercise(self):
        """Action logic for 'Play Exercise' button activation."""
        print("ACTION: Play Exercise Triggered")

        # --- Pre-checks ---
        # Check essential UI elements for playback exist
        if not self.trajectory_combo or not self.speed_slider or not self.play_status_label:
            print("ERROR: Playback UI elements are not initialized correctly. Cannot play.")
            if self.play_button and self.play_button.property("active"):
                self.play_button.setProperty("active", False); self._refresh_button_style(self.play_button)
            self._update_ui_state()
            return

        name = self.trajectory_combo.currentText()
        speed = self.speed_slider.value()

        if not name or name.startswith("<"): # Check for valid selection
            self._show_robot_error("Please select a valid exercise from the list.", None)
            if self.play_button and self.play_button.property("active"):
                self.play_button.setProperty("active", False); self._refresh_button_style(self.play_button)
            self._update_ui_state()
            return

        if not self.robot: # Check robot connection
            self._show_robot_error("Cannot play exercise: Robot not connected.", None)
            if self.play_button and self.play_button.property("active"):
                self.play_button.setProperty("active", False); self._refresh_button_style(self.play_button)
            self._update_ui_state()
            return

        if self._is_recording: # Check for conflicts
            self._show_robot_error("Cannot play exercise while recording is active.", None)
            if self.play_button and self.play_button.property("active"):
                self.play_button.setProperty("active", False); self._refresh_button_style(self.play_button)
            self._update_ui_state()
            return

        if self._is_playing: # Check if already playing
            print("Info: Playback is already in progress.")
            return

        print(f"  Attempting to play '{name}' at {speed}% speed.")

        # --- Robot Commands ---
        try:
            # 1. Load TPD
            print(f"  Sending Robot Command: LoadTPD('{name}')")
            retL = self.robot.LoadTPD(name)
            print(f"    LoadTPD Response: {retL}")
            if retL != 0: raise Exception(f"LoadTPD failed with code {retL}")

            # 2. Get Start Pose
            print(f"  Sending Robot Command: GetTPDStartPose('{name}')")
            retP, start_pose = self.robot.GetTPDStartPose(name)
            print(f"    GetTPDStartPose Response: {retP}, Pose: {start_pose}")
            if retP != 0: raise Exception(f"GetTPDStartPose failed with code {retP}")
            if not start_pose: raise Exception("GetTPDStartPose returned an empty pose.")

            # 3. Set Speed
            print(f"  Sending Robot Command: SetSpeed({speed})")
            retSp = self.robot.SetSpeed(speed)
            print(f"    SetSpeed Response: {retSp}")
            if retSp != 0: raise Exception(f"SetSpeed failed with code {retSp}")

            # 4. Move to Start
            print(f"  Sending Robot Command: MoveL({start_pose}, 0, {PLAYBACK_BLEND})")
            retM = self.robot.MoveL(start_pose, 0, PLAYBACK_BLEND)
            print(f"    MoveL Response: {retM}")
            if retM != 0:
                 self._show_robot_error(f"MoveL command to start pose failed (Code: {retM}).\nPlayback monitoring will still start, but exercise may not run correctly.", retM)
                 print("WARNING: MoveL to start failed, but attempting playback monitoring anyway.")
            else:
                print("  Waiting briefly after MoveL command...")
                QApplication.processEvents() # Process events during wait
                time.sleep(1.5) # Adjust as needed

            # 5. Start monitoring
            print(f"  Starting playback monitoring timer for '{name}'.")
            self._is_playing = True
            self._is_paused = False
            self.current_playback_name = name
            self.play_status_label.setText(f"Status: <b style='color:green;'>PLAYING</b> ({name} @ {speed}%)")
            self.playback_monitor_timer.start() # Start the QTimer

        except AttributeError as e:
            missing_func = "?"
            if 'LoadTPD' in str(e): missing_func = 'LoadTPD'
            elif 'GetTPDStartPose' in str(e): missing_func = 'GetTPDStartPose'
            elif 'SetSpeed' in str(e): missing_func = 'SetSpeed'
            elif 'MoveL' in str(e): missing_func = 'MoveL'
            msg = f"Robot object is missing the function '{missing_func}' needed for playback."
            self._show_robot_error(msg, None)
            self._is_playing = False; self._is_paused = False; self.current_playback_name = None
            if self.play_status_label: self.play_status_label.setText("Status: Error (Missing Func)")
            if self.play_button and self.play_button.property("active"):
                self.play_button.setProperty("active", False); self._refresh_button_style(self.play_button)
        except Exception as e:
            self._show_robot_error(f"Failed to start playback for '{name}':\n{e}", None)
            self._is_playing = False; self._is_paused = False; self.current_playback_name = None
            if self.play_status_label: self.play_status_label.setText("Status: Error (Robot Setup)")
            if self.play_button and self.play_button.property("active"):
                self.play_button.setProperty("active", False); self._refresh_button_style(self.play_button)

        self._update_ui_state()


    @pyqtSlot()
    def _check_playback_status(self):
        """Called periodically by QTimer to check if robot motion is done."""
        if not self.robot or not self._is_playing or self._is_paused:
            if self.playback_monitor_timer.isActive():
                print("WARNING: Playback monitor triggered in unexpected state. Stopping timer.")
                self.playback_monitor_timer.stop()
            return

        try:
            # print("DEBUG: Checking robot motion done status...") # Optional: uncomment for verbose check logging
            ret, motion_done = self.robot.GetRobotMotionDone()
            # print(f"DEBUG: GetRobotMotionDone response: ret={ret}, done={motion_done}") # Optional

            if ret != 0:
                print(f"WARNING: GetRobotMotionDone returned error code {ret}. Cannot determine status.")
                # Optionally stop monitoring or attempt recovery
                # self.playback_monitor_timer.stop()
                # self._stop_action(stop_button_itself=False)
                return

            if motion_done == 1: # Assuming 1 means motion is complete
                print(f"Playback detected as complete for '{self.current_playback_name}'.")
                self.playback_monitor_timer.stop()
                prev_name = self.current_playback_name
                self._is_playing = False
                self._is_paused = False
                self.current_playback_name = None
                if self.play_status_label: self.play_status_label.setText(f"Status: <b style='color:blue;'>Completed</b> ({prev_name})")
                if self.play_button and self.play_button.property("active"):
                    self.play_button.setProperty("active", False)
                    self._refresh_button_style(self.play_button)
                self._update_ui_state()

        except AttributeError:
            msg = "Robot object missing 'GetRobotMotionDone'. Playback cannot be monitored."
            print(f"ERROR: {msg}")
            self._show_robot_error(msg, None)
            self.playback_monitor_timer.stop()
            self._stop_action(stop_button_itself=False) # Force stop state
        except Exception as e:
            print(f"Error during playback status check: {e}")
            # Consider stopping timer on persistent errors
            # self.playback_monitor_timer.stop()
            # self._stop_action(stop_button_itself=False)


    @pyqtSlot()
    def _stop_action(self, stop_button_itself=True):
        """Handles Stop button clicks (Record or Playback) or programmatic stops."""
        print("ACTION: Stop Triggered")
        sender = self.sender() if stop_button_itself else None
        # triggered_by_record_stop = sender is self.stop_record_button
        # triggered_by_play_stop = sender is self.stop_play_button

        # Stop Monitoring
        if self.playback_monitor_timer.isActive():
            self.playback_monitor_timer.stop()
            print("  Playback monitor timer stopped.")

        # Store State Before Resetting
        was_recording = self._is_recording
        recording_name = self.current_recording_name
        # was_playing_or_paused = self._is_playing or self._is_paused
        # playback_name = self.current_playback_name

        # Send Stop Commands to Robot
        if self.robot:
            try:
                print("  Sending Robot Command: StopMotion()")
                err_stop = self.robot.StopMotion()
                print(f"    StopMotion Response: {err_stop}")

                if was_recording and recording_name:
                    print(f"  Sending Robot Command: SetWebTPDStop() for '{recording_name}'")
                    err_tpd_stop = self.robot.SetWebTPDStop()
                    print(f"    SetWebTPDStop Response: {err_tpd_stop}")
                elif was_recording and not recording_name:
                     print("  Warning: Was in recording state but no current_recording_name found.")

            except AttributeError as e:
                 missing_func = "?"
                 if 'StopMotion' in str(e): missing_func = 'StopMotion'
                 elif 'SetWebTPDStop' in str(e): missing_func = 'SetWebTPDStop'
                 print(f"ERROR: Robot object missing function '{missing_func}' needed for stop sequence.")
            except Exception as e:
                print(f"  Exception during robot stop sequence: {e}")
        else:
            print("  Robot not connected. Cannot send stop commands.")

        # Reset Internal State Flags
        print("  Resetting internal state flags...")
        self._is_recording = False
        self.current_recording_name = None
        self._is_playing = False
        self._is_paused = False
        self.current_playback_name = None

        # Update Status Labels
        if self.record_status_label: self.record_status_label.setText("Status: Idle")
        if self.play_status_label: self.play_status_label.setText("Status: Idle") # Check if label exists

        # Deactivate Action Buttons ('active' property controls visual state and logic)
        # Use a loop for clarity
        buttons_to_deactivate = [self.record_button, self.play_button, self.connect_button]
        for button in buttons_to_deactivate:
            if button and button.property("active"):
                 print(f"  Deactivating button '{button.objectName()}' visual state.")
                 button.setProperty("active", False)
                 self._refresh_button_style(button)


        # Disable Drag Teach (should happen after potentially deactivating connect/record buttons)
        self._disable_drag_teach()

        # Final UI Update
        print("Stop action finished.")
        self._update_ui_state()


    @pyqtSlot()
    def _resume_action(self):
        """Action logic for the Resume button."""
        print("ACTION: Resume Triggered")

        if not self.robot:
            self._show_robot_error("Cannot resume: Robot not connected.", None)
            return
        if not self._is_playing or not self._is_paused:
            print("Info: Cannot resume. Playback not in a paused state.")
            return

        print("  Sending Robot Command: ResumeMotion()")
        err = -1
        try:
            err = self.robot.ResumeMotion()
            print(f"    ResumeMotion Response: {err}")
        except AttributeError:
             msg = "Robot object missing the 'ResumeMotion' function."
             print(f"ERROR: {msg}")
             self._show_robot_error(msg, None)
             err = -99 # Use a distinct code for missing function
        except Exception as e:
            self._show_robot_error(f"Exception during ResumeMotion: {e}", None)
            err = -1 # General exception

        if err == 0: # Success
            print("  Resume command successful.")
            self._is_paused = False
            speed = self.speed_slider.value() if self.speed_slider else 100
            if self.play_status_label: self.play_status_label.setText(f"Status: <b style='color:green;'>PLAYING</b> ({self.current_playback_name} @ {speed}%)")
            if not self.playback_monitor_timer.isActive():
                 print("  Restarting playback monitor timer.")
                 self.playback_monitor_timer.start()
        else:
            print(f"  Resume command failed (Code: {err}). Playback remains paused.")
            # Only show popup for actual robot errors, not missing function
            if err != -1 and err != -99:
                 self._show_robot_error("Robot failed to resume playback.", err)

        self._update_ui_state()


    @pyqtSlot()
    def _pause_action(self):
        """Action logic for the Pause button."""
        print("ACTION: Pause Triggered")

        if not self.robot:
            self._show_robot_error("Cannot pause: Robot not connected.", None)
            return
        if not self._is_playing or self._is_paused:
            print("Info: Cannot pause. Playback not active or already paused.")
            return

        # Stop the monitoring timer *before* sending the pause command
        if self.playback_monitor_timer.isActive():
            self.playback_monitor_timer.stop()
            print("  Playback monitor timer stopped for pause.")

        print("  Sending Robot Command: PauseMotion()")
        err = -1
        try:
            err = self.robot.PauseMotion()
            print(f"    PauseMotion Response: {err}")
        except AttributeError:
             msg = "Robot object missing the 'PauseMotion' function."
             print(f"ERROR: {msg}")
             self._show_robot_error(msg, None)
             err = -99 # Distinct code for missing function
        except Exception as e:
            self._show_robot_error(f"Exception during PauseMotion: {e}", None)
            err = -1 # General exception

        if err == 0: # Success
            print("  Pause command successful.")
            self._is_paused = True
            speed = self.speed_slider.value() if self.speed_slider else 100
            if self.play_status_label: self.play_status_label.setText(f"Status: <b style='color:orange;'>PAUSED</b> ({self.current_playback_name} @ {speed}%)")
        else:
            print(f"  Pause command failed (Code: {err}). Playback may not be paused.")
            # Restart timer only if pause failed *and* timer was stopped
            if not self.playback_monitor_timer.isActive():
                 print("  Pause failed, restarting playback monitor timer.")
                 self.playback_monitor_timer.start()
            # Only show popup for actual robot errors
            if err != -1 and err != -99:
                 self._show_robot_error("Robot failed to pause playback.", err)

        self._update_ui_state()


    def closeEvent(self, event):
        """Handles the window close event, ensuring cleanup."""
        print("--- Application Close Event Triggered ---")

        if self.playback_monitor_timer.isActive():
            self.playback_monitor_timer.stop()
            print("  Playback monitor timer stopped on close.")

        needs_stop = self._is_recording or self._is_playing or self._is_paused
        # Check DT status based on buttons *if they exist*
        dt_active = False
        # Ensure action_buttons list itself exists before iterating
        if hasattr(self, 'action_buttons') and self.action_buttons:
             dt_active = any(b.property("active") for b in self.action_buttons if b and b.objectName() in DRAG_TEACH_CONTROLLERS)

        if needs_stop:
            print("  Closing: Active process detected. Sending stop commands...")
            self._stop_action(stop_button_itself=False) # Programmatic stop
            QApplication.processEvents() # Allow stop commands to be processed
            time.sleep(0.3) # Brief delay for commands
        elif dt_active:
             # Ensure drag teach is disabled if connect/record was active but no motion needed stopping
             print("  Closing: Drag Teach was active. Sending disable command...")
             self._disable_drag_teach()
             QApplication.processEvents()
             time.sleep(0.2) # Brief delay

        print("  Closing: Proceeding with closing the application window.")
        super().closeEvent(event)


# --- Main Execution Block ---
if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setApplicationName("Robot Control Interface")
    # Attempt to set icon gracefully
    try:
        icon = QIcon.fromTheme("applications-engineering")
        if not icon.isNull(): app.setWindowIcon(icon)
        else: print("Info: Could not find 'applications-engineering' theme icon.")
    except Exception as e: print(f"Info: Could not set application icon: {e}")

    # Create and show the main window
    # Use a try-except block around window creation for critical UI errors
    try:
        window = RobotControlTabsApp()
        window.show()
    except Exception as e:
        print(f"CRITICAL ERROR during application initialization: {e}")
        # Show a simple message box if possible, otherwise just exit
        try:
            msgBox = QMessageBox()
            msgBox.setIcon(QMessageBox.Icon.Critical)
            msgBox.setText(f"Application failed to initialize.\n\nError: {e}")
            msgBox.setWindowTitle("Initialization Error")
            msgBox.setStandardButtons(QMessageBox.StandardButton.Ok)
            msgBox.exec()
        except Exception as e_msg:
             print(f"Could not even show critical error message box: {e_msg}")
        sys.exit(1) # Exit with error code

    # Start the Qt event loop
    sys.exit(app.exec())