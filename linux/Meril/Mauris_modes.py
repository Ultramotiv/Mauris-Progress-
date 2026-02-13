"""
Passive Therapy Desktop App (PyQt6)
- Standalone (no web server)
- Implements passive-mode features from app.py + passive_therapy.html
"""

import sys
import os
import time
import threading
import math
import numpy as np
from PyQt6.QtCore import Qt, QTimer, QSize
from PyQt6.QtGui import QPixmap, QIcon
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QVBoxLayout,
    QHBoxLayout, QGridLayout, QComboBox, QSpinBox, QMessageBox, QInputDialog,
    QFrame, QLineEdit, QSizePolicy, QStackedWidget, QToolButton
)

# === Robot SDK ===
sys.path.append('/home/um/fairino-python-sdk-main/linux/fairino')
import Robot

ROBOT_IP = '192.168.58.2'
RECORDINGS_DIR = 'recordings'
os.makedirs(RECORDINGS_DIR, exist_ok=True)

# === Parameters (from app.py passive mode) ===
IMPEDANCE_PARAMS = {
    'lamde_dain': [2.5, 2.0, 2.0, 2.0, 2.0, 2.0],
    'b_gain': [20.0, 10.0, 10.0, 5.0, 5.0, 1.0],
    'k_gain': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'max_tcp_vel': 500,
    'max_tcp_ori_vel': 26
}

PASSIVE_MODE_SPEED = 5.0  # Velocity percentage [0-100]
TELEMETRY_UPDATE_RATE = 0.02  # 20 ms

HOME_TARGET_JOINTS = [
    2.608,    # J1
    -88.384,  # J2
    127.473,  # J3
    -137.27,  # J4
    -92.275,  # J5
    -90.118   # J6
]
HOME_MAX_JOINT_SPEED = 180.0  # deg/s
HOME_DESIRED_SPEED = 7.70     # deg/s


class PassiveTherapy:
    def __init__(self, robot_instance):
        self.robot = robot_instance
        self.is_recording = False
        self.current_recording_name = None
        self.playback_active = False
        self.current_direction = None
        self.trajectory_cache = {}
        self.playback_repetition_count = 0

    def custom_drag_teach_mode(self, enable=True):
        if enable:
            self.robot.DragTeachSwitch(1)
            time.sleep(0.5)
            rtn = self.robot.ForceAndJointImpedanceStartStop(
                status=1, impedanceFlag=1,
                lamdeDain=IMPEDANCE_PARAMS['lamde_dain'],
                KGain=IMPEDANCE_PARAMS['k_gain'],
                BGain=IMPEDANCE_PARAMS['b_gain'],
                dragMaxTcpVel=IMPEDANCE_PARAMS['max_tcp_vel'],
                dragMaxTcpOriVel=IMPEDANCE_PARAMS['max_tcp_ori_vel']
            )
            return rtn == 0
        self.robot.ForceAndJointImpedanceStartStop(
            status=0, impedanceFlag=0,
            lamdeDain=[0] * 6, KGain=[0] * 6, BGain=[0] * 6,
            dragMaxTcpVel=1000, dragMaxTcpOriVel=180
        )
        self.robot.DragTeachSwitch(0)
        time.sleep(0.5)
        return True

    def start_recording(self, name):
        if self.is_recording:
            return False, "Already recording"
        if not name:
            return False, "Name is required"
        try:
            success = self.custom_drag_teach_mode(enable=True)
            if not success:
                return False, "Failed to enable drag mode"
            ret1 = self.robot.SetTPDParam(name, 4, 0)
            if ret1 != 0:
                self.custom_drag_teach_mode(enable=False)
                return False, f"SetTPDParam failed (code: {ret1})"
            ret2 = self.robot.SetTPDStart(name, 4, 0)
            if ret2 != 0:
                self.custom_drag_teach_mode(enable=False)
                return False, f"SetTPDStart failed (code: {ret2})"
            self.is_recording = True
            self.current_recording_name = name
            return True, f"Recording started: {name}"
        except Exception as e:
            self.custom_drag_teach_mode(enable=False)
            return False, str(e)

    def stop_recording(self):
        if not self.is_recording:
            return False, "Not recording"
        try:
            ret = self.robot.SetWebTPDStop()
            self.custom_drag_teach_mode(enable=False)
            name = self.current_recording_name
            self.is_recording = False
            self.current_recording_name = None
            if ret != 0:
                return False, f"SetWebTPDStop failed (code: {ret})"
            if name in self.trajectory_cache:
                del self.trajectory_cache[name]
            return True, name
        except Exception as e:
            return False, str(e)

    def list_recordings(self):
        log_file = 'trajectory_log.csv'
        names = []
        if os.path.exists(log_file):
            with open(log_file, 'r') as f:
                for line in f:
                    if ',' in line:
                        parts = line.strip().split(',')
                        if len(parts) >= 2:
                            names.append(parts[1])
        return sorted(set(names))

    def _log_trajectory(self, name):
        log_file = 'trajectory_log.csv'
        file_exists = os.path.exists(log_file)
        with open(log_file, 'a') as f:
            if not file_exists:
                f.write('Timestamp,TrajectoryName,Notes\n')
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            f.write(f'{timestamp},{name},Recorded via Desktop App\n')

    def _get_trajectory_endpoints(self, filename, allow_probe=True):
        if filename in self.trajectory_cache:
            return self.trajectory_cache[filename]
        try:
            ret = self.robot.LoadTPD(filename)
            if ret != 0:
                return None, None
            err, start_pose = self.robot.GetTPDStartPose(filename)
            if err != 0:
                return None, None
            if not allow_probe:
                return start_pose, None

            self.robot.MoveL(start_pose, 0, 0, vel=PASSIVE_MODE_SPEED, overSpeedStrategy=1, speedPercent=2)
            time.sleep(0.3)
            self.robot.MoveTPD(filename, 1, 100)

            timeout = 30
            start_time = time.time()
            while True:
                if time.time() - start_time > timeout:
                    return None, None
                err, done = self.robot.GetRobotMotionDone()
                if err == 0 and done == 1:
                    break
                time.sleep(0.05)

            err, end_pose = self.robot.GetActualTCPPose()
            if err != 0:
                return None, None

            self.trajectory_cache[filename] = (start_pose, end_pose)
            return start_pose, end_pose
        except Exception:
            return None, None

    def _calculate_distance(self, pose1, pose2):
        return math.sqrt(sum((pose1[i] - pose2[i]) ** 2 for i in range(3)))

    def _determine_direction(self, filename, current_pose):
        start_pose, end_pose = self._get_trajectory_endpoints(filename, allow_probe=False)
        if start_pose is None or end_pose is None:
            return 1
        dist_to_start = self._calculate_distance(current_pose, start_pose)
        dist_to_end = self._calculate_distance(current_pose, end_pose)
        return 1 if dist_to_start <= dist_to_end else -1

    def start_playback(self, filename, repetitions=1):
        if self.playback_active:
            return False, "Playback already active"
        try:
            repetitions = int(repetitions)
        except Exception:
            repetitions = 1
        repetitions = max(1, min(100, repetitions))

        def playback_task():
            try:
                self.playback_active = True
                self.playback_repetition_count = 0

                ret = self.robot.LoadTPD(filename)
                if ret != 0:
                    return

                err, current_pose = self.robot.GetActualTCPPose()
                if err != 0:
                    return

                direction = self._determine_direction(filename, current_pose)
                self.current_direction = 'forward' if direction == 1 else 'reverse'

                start_pose, end_pose = self._get_trajectory_endpoints(filename, allow_probe=False)
                if start_pose is None:
                    return
                if end_pose is None and direction == -1:
                    direction = 1
                    self.current_direction = 'forward'

                for _ in range(repetitions):
                    if not self.playback_active:
                        break

                    if direction == 1:
                        self.robot.MoveL(start_pose, 0, 0, vel=PASSIVE_MODE_SPEED, overSpeedStrategy=1, speedPercent=2)
                    else:
                        self.robot.MoveL(end_pose, 0, 0, vel=PASSIVE_MODE_SPEED, overSpeedStrategy=1, speedPercent=2)

                    while self.playback_active:
                        err, done = self.robot.GetRobotMotionDone()
                        if err == 0 and done == 1:
                            break
                        time.sleep(0.02)

                    if not self.playback_active:
                        break

                    time.sleep(0.05)
                    self.robot.MoveTPD(filename, direction, 100)

                    while self.playback_active:
                        err, done = self.robot.GetRobotMotionDone()
                        if err == 0 and done == 1:
                            break
                        time.sleep(0.02)

                    if not self.playback_active:
                        break

                    self.playback_repetition_count += 1
                    time.sleep(0.05)
            finally:
                self.playback_active = False
                self.current_direction = None

        threading.Thread(target=playback_task, daemon=True).start()
        return True, "Playback started"

    def stop_playback(self):
        self.playback_active = False
        try:
            self.robot.StopMotion()
        except Exception:
            pass
        return True, "Playback stopped"

    def pause_playback(self):
        if not self.playback_active:
            return False, "Playback not active"
        try:
            ret = self.robot.PauseMotion()
            if ret != 0:
                return False, f"PauseMotion failed (code: {ret})"
            return True, "Playback paused"
        except Exception as e:
            return False, str(e)

    def resume_playback(self):
        if not self.playback_active:
            return False, "Playback not active"
        try:
            ret = self.robot.ResumeMotion()
            if ret != 0:
                return False, f"ResumeMotion failed (code: {ret})"
            return True, "Playback resumed"
        except Exception as e:
            return False, str(e)


class PassiveTherapyPage(QWidget):
    def __init__(self, on_back=None, on_robot_connected=None):
        super().__init__()
        self.on_back = on_back
        self.on_robot_connected = on_robot_connected
        self.setMinimumWidth(640)

        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                            stop:0 #000000, stop:0.5 #1a3a34,
                                            stop:0.75 #2d5a4f, stop:1 #44b8a6);
            }
            QLabel { color: #e2e8f0; font-size: 18px; }
            QLabel#sectionTitle { font-size: 20px; font-weight: 600; color: #93c5fd; }
            QLabel#sectionSub { color: #94a3b8; font-size: 20px; }
            QPushButton {
                color: white; border: 1px solid rgba(68, 184, 166, 0.3);
                padding: 8px 14px; border-radius: 8px; font-weight: 600;
                font-size: 16px;
                min-height: 40px; max-height: 40px;
            }
            QPushButton:disabled { background: #2a3a3f; color: #94a3b8; }

            QPushButton[class="mode"] {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #9f7aea, stop:1 #805ad5);
            }
            QPushButton[class="mode"]:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #b779da, stop:1 #9f7aea);
            }

            QPushButton[class="record"] {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #d97706, stop:1 #b45309);
            }
            QPushButton[class="record"]:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #f59e0b, stop:1 #d97706);
            }

            QPushButton[class="stop"] {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #e53e3e, stop:1 #c53030);
            }
            QPushButton[class="stop"]:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                            stop:0 #f56565, stop:1 #e53e3e);
            }
            QComboBox, QSpinBox {
                background: #0f1f22; color: #e2e8f0; border: 1px solid #2a3a3f;
                border-radius: 6px; padding: 4px 8px;
            }
            QFrame#container {
                background: rgba(50, 54, 58, 0.95);
                border-radius: 16px;
                padding: 16px;
                color: #f0f4f8;
            }
            QFrame#card {
                background: rgba(30, 41, 59, 0.7);
                border: 1px solid rgba(68, 184, 166, 0.4);
                border-radius: 8px;
                padding: 12px;
            }
        """)

        self.robot = Robot.RPC(ROBOT_IP)
        self.therapy = PassiveTherapy(self.robot)

        # Notify main page that robot is connected
        if self.on_robot_connected:
            try:
                self.on_robot_connected(True, "Robot Connected", "Connection established and ready")
            except Exception as e:
                print(f"Error notifying robot connection: {e}")

        self.drag_active = False
        self.recording = False
        self.playback_running = False
        self.current_fz = 0.0
        self.current_tcp_z = 0.0

        self._build_ui()
        self._start_telemetry_thread()
        self._start_ui_timer()

    def _build_ui(self):
        outer_layout = QHBoxLayout()
        outer_layout.setContentsMargins(16, 16, 16, 16)
        outer_layout.setSpacing(16)

        left_panel = QFrame()
        left_panel.setObjectName("container")
        left_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        layout = QVBoxLayout()
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)

        title = QLabel("Mauris: \n Modular Adaptive universal Robotic Intelligence System")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("font-size: 18px; font-weight: 700; color: #e2e8f0;")
        layout.addWidget(title)

        subtitle = QLabel("Robotic Rehabilitation • Passive Mode")
        subtitle.setAlignment(Qt.AlignmentFlag.AlignCenter)
        subtitle.setObjectName("sectionSub")
        layout.addWidget(subtitle)

        # Controls card (First)
        controls_card = QFrame()
        controls_card.setObjectName("card")
        controls_card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        controls_layout = QVBoxLayout()
        controls_layout.setSpacing(8)
        controls_title = QLabel("Controls")
        controls_title.setObjectName("sectionTitle")
        controls_layout.addWidget(controls_title)

        drag_home_row = QHBoxLayout()
        self.drag_btn = QPushButton("Move Robot")
        self.home_btn = QPushButton("Home")
        self.drag_btn.setProperty("class", "mode")
        self.home_btn.setProperty("class", "mode")
        self.drag_btn.clicked.connect(self.toggle_drag_mode)
        self.home_btn.clicked.connect(self.move_home)
        drag_home_row.addWidget(self.drag_btn)
        drag_home_row.addWidget(self.home_btn)
        controls_layout.addLayout(drag_home_row)
        controls_card.setLayout(controls_layout)
        layout.addWidget(controls_card, stretch=0)

        # Recording card (Second)
        record_card = QFrame()
        record_card.setObjectName("card")
        record_card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        record_card_layout = QVBoxLayout()
        record_card_layout.setSpacing(8)
        record_title = QLabel("Recording")
        record_title.setObjectName("sectionTitle")
        record_card_layout.addWidget(record_title)

        record_row = QHBoxLayout()
        self.start_record_btn = QPushButton("Start Recording")
        self.stop_record_btn = QPushButton("Stop Recording")
        self.stop_record_btn.setEnabled(False)
        self.start_record_btn.setProperty("class", "record")
        self.stop_record_btn.setProperty("class", "stop")
        self.start_record_btn.clicked.connect(self.start_recording)
        self.stop_record_btn.clicked.connect(self.stop_recording)
        record_row.addWidget(self.start_record_btn)
        record_row.addWidget(self.stop_record_btn)
        record_card_layout.addLayout(record_row)

        self.record_status = QLabel("Idle")
        self.record_status.setStyleSheet("color: #93c5fd;")
        record_card_layout.addWidget(self.record_status)
        record_card.setLayout(record_card_layout)
        layout.addWidget(record_card, stretch=0)

        # Playback card
        playback_card = QFrame()
        playback_card.setObjectName("card")
        playback_card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        playback_card_layout = QVBoxLayout()
        playback_card_layout.setSpacing(8)
        playback_title = QLabel("Playback Recorded Exercise")
        playback_title.setObjectName("sectionTitle")
        playback_card_layout.addWidget(playback_title)

        playback_grid = QGridLayout()
        playback_grid.addWidget(QLabel("Recorded Exercise"), 0, 0)

        self.exercise_select = QComboBox()
        self.exercise_select.currentIndexChanged.connect(self._toggle_playback_button)
        playback_grid.addWidget(self.exercise_select, 0, 1)

        playback_grid.addWidget(QLabel("Repetitions"), 1, 0)
        self.reps_input = QSpinBox()
        self.reps_input.setRange(1, 100)
        self.reps_input.setValue(5)
        playback_grid.addWidget(self.reps_input, 1, 1)

        self.current_rep_label = QLabel("Current Rep: 0")
        self.current_rep_label.setStyleSheet("color: #93c5fd; font-weight: 600;")
        playback_grid.addWidget(self.current_rep_label, 2, 0, 1, 2)

        self.playback_btn = QPushButton("Start Playback")
        self.pause_btn = QPushButton("Pause")
        self.resume_btn = QPushButton("Play")
        self.pause_btn.setEnabled(False)
        self.resume_btn.setEnabled(False)

        self.playback_btn.setProperty("class", "mode")
        self.pause_btn.setProperty("class", "mode")
        self.resume_btn.setProperty("class", "mode")

        self.playback_btn.clicked.connect(self.start_playback)
        self.pause_btn.clicked.connect(self.pause_playback)
        self.resume_btn.clicked.connect(self.resume_playback)

        playback_grid.addWidget(self.playback_btn, 3, 0)
        playback_grid.addWidget(self.pause_btn, 3, 1)
        playback_grid.addWidget(self.resume_btn, 4, 0)
        playback_card_layout.addLayout(playback_grid)
        playback_card.setLayout(playback_card_layout)
        layout.addWidget(playback_card, stretch=0)

        # Status + telemetry card (Fourth)
        telemetry_card = QFrame()
        telemetry_card.setObjectName("card")
        telemetry_card.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        telemetry_layout = QVBoxLayout()
        telemetry_layout.setSpacing(8)
        telemetry_title = QLabel("Real-time Telemetry")
        telemetry_title.setObjectName("sectionTitle")
        telemetry_layout.addWidget(telemetry_title)

        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: #fbbf24; font-weight: 600;")
        telemetry_layout.addWidget(self.status_label)

        self.fz_label = QLabel("Fz Force: 0.00 N")
        self.z_label = QLabel("TCP Z Position: 0.00 mm")
        telemetry_layout.addWidget(self.fz_label)
        telemetry_layout.addWidget(self.z_label)
        telemetry_card.setLayout(telemetry_layout)
        layout.addWidget(telemetry_card, stretch=0)

        self.back_btn = QPushButton("Back to Main Menu")
        self.back_btn.setProperty("class", "mode")
        self.back_btn.clicked.connect(self._handle_back)
        layout.addWidget(self.back_btn, stretch=0)
        
        # Add stretch at the end to push cards to the top
        layout.addStretch(1)

        # Uniform button sizing
        for btn in [
            self.start_record_btn, self.stop_record_btn,
            self.playback_btn, self.pause_btn, self.resume_btn,
            self.drag_btn, self.home_btn, self.back_btn
        ]:
            btn.setMinimumWidth(100)
            btn.setMinimumHeight(35)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        left_panel.setLayout(layout)
        outer_layout.addStretch(0)
        outer_layout.addWidget(left_panel, 1)
        outer_layout.addStretch(0)
        self.setLayout(outer_layout)

        self._load_recorded_exercises()
        self._toggle_playback_button()

    def _handle_back(self):
        if self.on_back:
            self.on_back()

    def _load_recorded_exercises(self):
        self.exercise_select.clear()
        self.exercise_select.addItem("Select a recorded exercise...")
        for name in self.therapy.list_recordings():
            self.exercise_select.addItem(name)

    def _toggle_playback_button(self):
        enabled = self.exercise_select.currentIndex() > 0
        self.playback_btn.setEnabled(enabled)

    def _start_telemetry_thread(self):
        def telemetry_loop():
            while True:
                try:
                    err, tcp = self.robot.GetActualTCPPose()
                    if err == 0:
                        self.current_tcp_z = tcp[2]
                    ft = self.robot.FT_GetForceTorqueRCS()
                    if ft[0] == 0:
                        raw = ft[1][:6]
                        self.current_fz = raw[2]
                except Exception:
                    pass
                time.sleep(TELEMETRY_UPDATE_RATE)

        threading.Thread(target=telemetry_loop, daemon=True).start()

    def _start_ui_timer(self):
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(50)

    def _refresh_ui(self):
        self.fz_label.setText(f"Fz Force: {self.current_fz:.2f} N")
        self.z_label.setText(f"TCP Z Position: {self.current_tcp_z:.2f} mm")
        self.current_rep_label.setText(f"Current Rep: {self.therapy.playback_repetition_count}")

    def start_recording(self):
        if self.drag_active:
            QMessageBox.warning(self, "Drag Mode Active", "Disable Drag Mode before recording")
            return
        name, ok = QInputDialog.getText(self, "Start Recording", "Enter a name for this trajectory:")
        if not ok or not name.strip():
            return
        success, msg = self.therapy.start_recording(name.strip())
        if success:
            self.therapy._log_trajectory(name.strip())
            self.recording = True
            self.start_record_btn.setEnabled(False)
            self.stop_record_btn.setEnabled(True)
            self.record_status.setText(f"Recording: {name.strip()}")
            self.status_label.setText("Recording Trajectory...")
        else:
            QMessageBox.warning(self, "Recording Failed", msg)

    def stop_recording(self):
        success, msg = self.therapy.stop_recording()
        if success:
            self.recording = False
            self.start_record_btn.setEnabled(True)
            self.stop_record_btn.setEnabled(False)
            self.record_status.setText(f"Saved as: {msg}")
            self.status_label.setText("Ready")
            self._load_recorded_exercises()
        else:
            QMessageBox.warning(self, "Stop Recording Failed", msg)

    def start_playback(self):
        if self.drag_active:
            QMessageBox.warning(self, "Drag Mode Active", "Disable Drag Mode first")
            return
        filename = self.exercise_select.currentText()
        if filename == "Select a recorded exercise...":
            return
        reps = self.reps_input.value()
        success, msg = self.therapy.start_playback(filename, reps)
        if success:
            self.playback_running = True
            self.playback_btn.setText("Stop Playback")
            self.playback_btn.clicked.disconnect()
            self.playback_btn.clicked.connect(self.stop_playback)
            self.pause_btn.setEnabled(True)
            self.resume_btn.setEnabled(False)
            self.status_label.setText("Playing Back Recorded Exercise")
        else:
            QMessageBox.warning(self, "Playback Failed", msg)

    def stop_playback(self):
        self.therapy.stop_playback()
        self.playback_running = False
        self.playback_btn.setText("Start Playback")
        self.playback_btn.clicked.disconnect()
        self.playback_btn.clicked.connect(self.start_playback)
        self.pause_btn.setEnabled(False)
        self.resume_btn.setEnabled(False)
        self.status_label.setText("Ready")

    def pause_playback(self):
        success, msg = self.therapy.pause_playback()
        if success:
            self.pause_btn.setEnabled(False)
            self.resume_btn.setEnabled(True)
        else:
            QMessageBox.warning(self, "Pause Failed", msg)

    def resume_playback(self):
        success, msg = self.therapy.resume_playback()
        if success:
            self.pause_btn.setEnabled(True)
            self.resume_btn.setEnabled(False)
        else:
            QMessageBox.warning(self, "Resume Failed", msg)

    def toggle_drag_mode(self):
        success = self.therapy.custom_drag_teach_mode(enable=not self.drag_active)
        if success:
            self.drag_active = not self.drag_active
            self.drag_btn.setText("Drag Mode (Active)" if self.drag_active else "Drag Mode")
            self.status_label.setText("Drag Mode: Active" if self.drag_active else "Ready")
        else:
            QMessageBox.warning(self, "Drag Mode Failed", "Failed to toggle drag mode")

    def move_home(self):
        if self.playback_running or self.recording:
            QMessageBox.warning(self, "Busy", "Stop playback/recording before moving home")
            return
        try:
            vel_percent = round((HOME_DESIRED_SPEED / HOME_MAX_JOINT_SPEED) * 100.0, 3)
            ret = self.robot.MoveJ(
                joint_pos=HOME_TARGET_JOINTS,
                tool=0,
                user=0,
                desc_pos=[0.0] * 7,
                vel=vel_percent,
                acc=0.0,
                ovl=100.0,
                exaxis_pos=[0.0] * 4,
                blendT=-1.0,
                offset_flag=0,
                offset_pos=[0.0] * 6
            )
            if ret != 0:
                QMessageBox.warning(self, "Home Failed", f"MoveJ failed (code: {ret})")
            else:
                self.status_label.setText("Moving to Home...")
        except Exception as e:
            QMessageBox.warning(self, "Home Failed", str(e))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Therapy Control Interface")
        self.setMinimumSize(800, 600)
        self.resize(1200, 800)  # Default larger window
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.robot_connected = False
        self.main_page = self._build_main_page()
        self.passive_page = PassiveTherapyPage(on_back=self._show_main_page, on_robot_connected=self._update_robot_status)

        self.stack.addWidget(self.main_page)
        self.stack.addWidget(self.passive_page)
        self.stack.setCurrentWidget(self.main_page)

        # Start connection checking in background (don't block GUI startup)
        self._start_robot_connection_check()

    def _build_main_page(self):
        self.setStyleSheet("""
            QMainWindow { background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                stop:0 #000000, stop:0.5 #1a3a34, stop:0.75 #2d5a4f, stop:1 #44b8a6); }
            QLabel { color: white; }
            QFrame#header {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(26, 58, 52, 0.8), stop:1 rgba(45, 90, 79, 0.8));
                border-radius: 16px; border: 1px solid rgba(68, 184, 166, 0.3);
            }
            QFrame#section {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(26, 58, 52, 0.9), stop:1 rgba(45, 90, 79, 0.9));
                border-radius: 16px; border: 1px solid rgba(68, 184, 166, 0.3);
            }
            QFrame#exerciseSection {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 rgba(26, 58, 52, 0.9), stop:1 rgba(45, 90, 79, 0.9));
                border-radius: 16px; border: 1px solid rgba(68, 184, 166, 0.3);
            }
            QFrame#statusBox {
                background: rgba(0, 0, 0, 0.3);
                border-radius: 12px; border: 1px solid rgba(68, 184, 166, 0.2);
            }
            QLabel#sectionTitle { font-size: 18px; font-weight: 600; }
            QLabel#muted { color: #a0a0a0; }
            QPushButton#primaryBtn {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #44b8a6, stop:1 #2d9d8b);
                color: white; border: none; padding: 12px 18px;
                border-radius: 8px; font-weight: 600; font-size: 14px;
            }
            QPushButton#modeBtn {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2d2d2d, stop:1 #1a1a1a);
                color: white; border: 1px solid rgba(68, 184, 166, 0.3);
                padding: 12px 16px; border-radius: 8px; font-weight: 600;
            }
            QToolButton#modeBtn {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #2d2d2d, stop:1 #1a1a1a);
                color: white; border: 1px solid rgba(68, 184, 166, 0.3);
                padding: 0px; border-radius: 8px; font-weight: 600;
            }
            QLineEdit, QSpinBox, QComboBox {
                background: rgba(0, 0, 0, 0.3); color: white;
                border: 1px solid rgba(68, 184, 166, 0.3); border-radius: 8px;
                padding: 8px;
            }
        """)

        root = QWidget()
        outer = QVBoxLayout()
        outer.setContentsMargins(20, 20, 20, 20)
        outer.setSpacing(16)
        outer.setStretchFactor(outer, 1)

        header = QFrame()
        header.setObjectName("header")
        header.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(16, 16, 16, 16)

        logo_label = QLabel()
        logo_label.setFixedSize(120, 80)
        logo_label.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        logo_path = os.path.join(os.path.dirname(__file__), "..", "..", "images", "header", "logo.webp")
        logo_path = os.path.abspath(logo_path)
        if os.path.exists(logo_path):
            pix = QPixmap(logo_path)
            logo_label.setPixmap(pix.scaled(120, 80, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation))
        else:
            logo_label.setText("")
        header_layout.addWidget(logo_label)

        title_box = QVBoxLayout()
        title = QLabel("MAURIS")
        title.setStyleSheet("font-size: 20px; font-weight: 700;")
        subtitle = QLabel("Modular Adaptive Universal Robotic Intelligent System")
        subtitle.setStyleSheet("font-size: 14px; font-weight: 600;")
        tagline = QLabel("Reversing Disabilities Completely")
        tagline.setObjectName("muted")
        title_box.addWidget(title, alignment=Qt.AlignmentFlag.AlignCenter)
        title_box.addWidget(subtitle, alignment=Qt.AlignmentFlag.AlignCenter)
        title_box.addWidget(tagline, alignment=Qt.AlignmentFlag.AlignCenter)
        header_layout.addLayout(title_box, stretch=1)

        spacer = QWidget()
        spacer.setFixedWidth(120)
        header_layout.addWidget(spacer)

        header.setLayout(header_layout)
        outer.addWidget(header)

        main_grid = QGridLayout()
        main_grid.setSpacing(16)

        # Robot Connection Section
        robot_section = QFrame()
        robot_section.setObjectName("section")
        robot_layout = QVBoxLayout()
        robot_layout.setContentsMargins(18, 18, 18, 18)

        robot_title = QLabel("Robot Connection")
        robot_title.setObjectName("sectionTitle")
        robot_layout.addWidget(robot_title)

        status_box = QFrame()
        status_box.setObjectName("statusBox")
        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(12, 12, 12, 12)

        self.robot_status_dot = QLabel()
        self.robot_status_dot.setFixedSize(12, 12)
        self.robot_status_dot.setStyleSheet("background: #ff9800; border-radius: 6px;")
        status_layout.addWidget(self.robot_status_dot)

        status_text = QVBoxLayout()
        self.robot_status_title = QLabel("Status: Connecting...")
        self.robot_status_desc = QLabel("Attempting to connect to robot")
        self.robot_status_desc.setObjectName("muted")
        status_text.addWidget(self.robot_status_title)
        status_text.addWidget(self.robot_status_desc)
        status_layout.addLayout(status_text)

        status_box.setLayout(status_layout)
        robot_layout.addWidget(status_box)

        report_btn = QPushButton("Progress Report")
        report_btn.setObjectName("primaryBtn")
        report_btn.clicked.connect(self._generate_report)
        robot_layout.addWidget(report_btn)
        robot_section.setLayout(robot_layout)

        # Patient Information Section
        patient_section = QFrame()
        patient_section.setObjectName("section")
        patient_layout = QVBoxLayout()
        patient_layout.setContentsMargins(18, 18, 18, 18)

        patient_title = QLabel("Patient Information")
        patient_title.setObjectName("sectionTitle")
        patient_layout.addWidget(patient_title)

        self.patient_name = QLineEdit()
        self.patient_name.setPlaceholderText("Enter patient name")
        self.patient_age = QSpinBox()
        self.patient_age.setRange(0, 120)
        self.patient_age.setSpecialValueText("Enter age")
        self.patient_gender = QComboBox()
        self.patient_gender.addItems(["Select gender", "Male", "Female", "Other"])
        self.patient_diagnosis = QLineEdit()
        self.patient_diagnosis.setPlaceholderText("Enter diagnosis")

        patient_layout.addWidget(QLabel("Full Name:"))
        patient_layout.addWidget(self.patient_name)
        patient_layout.addWidget(QLabel("Age:"))
        patient_layout.addWidget(self.patient_age)
        patient_layout.addWidget(QLabel("Gender:"))
        patient_layout.addWidget(self.patient_gender)
        patient_layout.addWidget(QLabel("Diagnosis:"))
        patient_layout.addWidget(self.patient_diagnosis)
        patient_section.setLayout(patient_layout)

        robot_section.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        patient_section.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        main_grid.addWidget(robot_section, 0, 0)
        main_grid.addWidget(patient_section, 0, 1)
        main_grid.setColumnStretch(0, 1)
        main_grid.setColumnStretch(1, 1)
        outer.addLayout(main_grid)

        # Therapy Mode Selection
        exercise_section = QFrame()
        exercise_section.setObjectName("exerciseSection")
        exercise_section.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        exercise_layout = QVBoxLayout()
        exercise_layout.setContentsMargins(18, 18, 18, 18)
        exercise_layout.setSpacing(12)

        exercise_title = QLabel("Therapy Mode Selection")
        exercise_title.setObjectName("sectionTitle")
        exercise_layout.addWidget(exercise_title)

        modes_grid = QGridLayout()
        modes_grid.setSpacing(12)

        self.active_btn = QPushButton("Active Mode")
        self.passive_btn = QToolButton()
        self.passive_btn.setText("Passive Mode")
        self.guided_btn = QPushButton("Guided Mode")
        self.resistive_btn = QPushButton("Resistive Mode")
        self.game_btn = QPushButton("Game Mode")

        passive_img_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "images", "passive_mode_button.png")
        )
        if os.path.exists(passive_img_path):
            self.passive_btn.setIcon(QIcon(passive_img_path))
            self.passive_btn.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonIconOnly)
            self.passive_btn.setToolTip("Passive Mode")

        for btn in [self.active_btn, self.passive_btn, self.guided_btn, self.resistive_btn, self.game_btn]:
            btn.setObjectName("modeBtn")
            btn.setMinimumWidth(120)
            btn.setMinimumHeight(120)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Disable all buttons except Passive Mode
        self.active_btn.setEnabled(False)
        self.guided_btn.setEnabled(False)
        self.resistive_btn.setEnabled(False)
        self.game_btn.setEnabled(False)

        self.passive_btn.clicked.connect(self._open_passive_mode)
        self.active_btn.clicked.connect(lambda: self._not_implemented("Active Mode"))
        self.guided_btn.clicked.connect(lambda: self._not_implemented("Guided Mode"))
        self.resistive_btn.clicked.connect(lambda: self._not_implemented("Resistive Mode"))
        self.game_btn.clicked.connect(lambda: self._not_implemented("Game Mode"))

        # All buttons in same row with same size
        modes_grid.addWidget(self.active_btn, 0, 0)
        modes_grid.addWidget(self.passive_btn, 0, 1)
        modes_grid.addWidget(self.guided_btn, 0, 2)
        modes_grid.addWidget(self.resistive_btn, 0, 3)
        modes_grid.addWidget(self.game_btn, 0, 4)

        for col in range(5):
            modes_grid.setColumnStretch(col, 1)
        modes_grid.setRowStretch(0, 1)
        exercise_layout.addLayout(modes_grid, stretch=1)

        exercise_section.setLayout(exercise_layout)
        outer.addWidget(exercise_section, stretch=1)

        root.setLayout(outer)
        return root

    def _generate_report(self):
        name = self.patient_name.text().strip()
        if not name:
            QMessageBox.information(self, "Missing Info", "Please enter patient information first!")
            return
        QMessageBox.information(self, "Progress Report", f"Generating progress report for {name}...\n\nReport will be available shortly.")

    def _not_implemented(self, mode_name):
        QMessageBox.information(self, mode_name, f"{mode_name} is not implemented yet.")

    def _open_passive_mode(self):
        self.stack.setCurrentWidget(self.passive_page)

    def _show_main_page(self):
        self.stack.setCurrentWidget(self.main_page)

    def _start_robot_connection_check(self):
        """Background thread to check robot connection periodically"""
        def check_connection():
            last_status = None
            check_count = 0
            while True:
                try:
                    check_count += 1
                    # Only check every 10 iterations (every 50 seconds instead of every 5)
                    if check_count % 10 == 0:
                        robot = Robot.RPC(ROBOT_IP)
                        time.sleep(0.2)
                        
                        try:
                            err, state = robot.GetRobotState()
                            is_connected = (err == 0)
                        except:
                            is_connected = False
                        
                        if is_connected != last_status:
                            if is_connected:
                                self._update_robot_status(True, "Robot Connected", "Connection established and ready")
                            else:
                                self._update_robot_status(False, "Status: Disconnected", "Unable to communicate with robot")
                            last_status = is_connected
                            self.robot_connected = is_connected
                    
                except Exception as e:
                    if last_status != False:
                        self._update_robot_status(False, "Status: Disconnected", "Connection failed")
                        last_status = False
                        self.robot_connected = False
                
                time.sleep(5)
        
        threading.Thread(target=check_connection, daemon=True).start()

    def _update_robot_status(self, connected, title, description):
        """Update robot status display on main page"""
        try:
            if connected:
                self.robot_status_dot.setStyleSheet("background: #10b981; border-radius: 6px;")  # Green
                status_title_text = f"Status: {title}"
            else:
                self.robot_status_dot.setStyleSheet("background: #ef4444; border-radius: 6px;")  # Red
                status_title_text = title
            
            self.robot_status_title.setText(status_title_text)
            self.robot_status_desc.setText(description)
        except Exception as e:
            print(f"Error updating robot status: {e}")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "passive_btn"):
            size = self.passive_btn.size()
            # Set icon size to fill the entire button with minimal padding
            padding = 5
            icon_width = max(1, size.width() - padding)
            icon_height = max(1, size.height() - padding)
            self.passive_btn.setIconSize(QSize(icon_width, icon_height))


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
